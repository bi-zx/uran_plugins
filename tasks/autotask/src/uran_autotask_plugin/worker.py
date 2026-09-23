from __future__ import annotations

import json
import logging
import re
import threading
import time
from dataclasses import dataclass
from typing import Any, Mapping

from uran_interfaces import ContractValidator
from uran_localization_plugin import ClosedLoopManager, GeoReferenceConfig, MapProjector, PoseRegistry
from uran_navigation_plugin import StraightDriveController
from uran_navigation_plugin.depth_scan_adapter import VirtualLaserScan
from uran_worker_sdk import Worker, WorkerServer

from .adapters import CyberDogAlgorithmManagerBackend, CyberDogCameraCaptureBackend, SimulatedNavigationBackend
from .mission import MissionManager
from .task_models import TERMINAL_TASK_STAGES


def _segment(value: str) -> str:
    return "id_" + (re.sub(r"[^A-Za-z0-9_]", "_", str(value)) or "unknown")


@dataclass
class _MissionContext:
    operation_id: str
    task_id: str
    target: dict
    manager: MissionManager
    move_gateway: Any
    straight_drive: Any


class _LocalMoveGateway:
    def __init__(self, worker, *, operation_id: str, task_id: str, target: dict):
        self.worker = worker
        self.operation_id = operation_id
        self.task_id = task_id
        self.target = dict(target)
        self.sequence = 0

    def _send(self, payload: dict, *, reason: str) -> None:
        if (not self.worker.arbiter_instance and self.worker.backend_is_simulation and
                self.worker.allow_unarbitrated_simulation):
            return
        self.sequence += 1
        now_ns = time.time_ns()
        command = {
            "schema_version": "1.0",
            "operation_id": f"{self.operation_id}-motion-{self.sequence}",
            "target": self.target,
            "controller": "auto",
            "sequence": now_ns,
            "lease_id": self.worker.lease_id(self.operation_id),
            "issued_at_ns": now_ns,
            "valid_until_ns": now_ns + self.worker.motion_validity_ns,
            "payload": payload,
            "metadata": {"source": self.worker.lease_owner(self.operation_id),
                         "task_id": self.task_id, "reason": reason},
        }
        self.worker.call_local({
            "message_type": "uran.motion.command.v1",
            "correlation_id": command["operation_id"],
            "target": {"device_id": self.target["device_id"], "instance_id": self.worker.arbiter_instance,
                       "capability_id": "motion.control.gateway"},
            "payload": command,
        })

    def velocity(self, *, linear_x: float, angular_z: float, source: str,
                 reason: str, task_id: str) -> None:
        del source, task_id
        self._send({"kind": "base_velocity", "frame_id": "base_link",
                    "linear_mps": {"x": float(linear_x), "y": 0.0, "z": 0.0},
                    "angular_rps": {"x": 0.0, "y": 0.0, "z": float(angular_z)}},
                   reason=reason)

    def stop(self, *, reason: str) -> None:
        self._send({"kind": "safety_action", "action": "stop", "reason": reason}, reason=reason)


class AutotaskWorker(Worker):
    plugin_id = "uran-autotask"
    plugin_version = "0.1.0"
    capabilities = ("tasks.waypoint", "tasks.outdoor", "tasks.lifecycle")

    def configure(self, config: Mapping[str, Any]) -> None:
        self.validator = ContractValidator()
        self.config = dict(config)
        self.defaults = dict(config.get("mission_defaults") or {})
        self.expected_device_id = str(config.get("device_id") or self.identity.get("device_id") or "")
        self.arbiter_instance = str(config.get("arbiter_instance") or "")
        self.backend_is_simulation = False
        self.allow_unarbitrated_simulation = bool(
            config.get("allow_unarbitrated_simulation", False))
        self.media_instance = str(config.get("media_instance") or "")
        self.motion_validity_ns = int(float(config.get("motion_validity_s", 0.5)) * 1_000_000_000)
        self.lease_duration_ns = int(float(config.get("lease_duration_s", 3600.0)) * 1_000_000_000)
        self.require_auto_mode = bool(config.get("require_auto_mode", True))
        self.control_mode = str(config.get("control_mode", "auto"))
        self.controller = str(config.get("controller", "auto"))
        self.battery_level = config.get("battery_level")
        self.registry = PoseRegistry(history_size=int(config.get("pose_history_size", 600)))
        self.projector = MapProjector(GeoReferenceConfig.from_dict(dict(config.get("geo_reference") or {})))
        self.closed_loop = ClosedLoopManager()
        self.contexts = {}
        self.by_target_task = {}
        self.event_sequences = {}
        self.state_fields = set()
        self.state_reporting = bool(config.get("state_reporting", True))
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.tick_interval_s = max(0.02, float(config.get("tick_interval_s", 0.1)))
        self.backend = None
        self.camera_capture = None
        self.cancelled_operations = set()
        self._ros = None
        backend_name = str(config.get("backend") or "unavailable")
        if backend_name == "simulation":
            self.backend = SimulatedNavigationBackend()
            self.backend_is_simulation = True
        elif backend_name == "cyberdog2_ros2":
            self._configure_ros2(config)
        elif backend_name != "unavailable":
            raise ValueError(f"unsupported autotask backend: {backend_name}")

    def _configure_ros2(self, config: Mapping[str, Any]) -> None:
        try:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor
        except ImportError as exc:
            raise RuntimeError("ROS2 runtime is unavailable") from exc
        rclpy.init(args=None)
        node = rclpy.create_node(str(config.get("node_name") or "uran_autotask_worker"))
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            self.backend = CyberDogAlgorithmManagerBackend(node, dict(config.get("cyberdog_backend") or {}))
            camera_cfg = dict(config.get("camera_capture") or {})
            if camera_cfg.get("enabled", True):
                self.camera_capture = CyberDogCameraCaptureBackend(node, camera_cfg)
        except Exception:
            executor.shutdown(timeout_sec=2.0)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            thread.join(timeout=2.0)
            raise
        self._ros = (rclpy, node, executor, thread)

    def activate(self) -> None:
        self.stop_event.clear()
        self.tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self.tick_thread.start()

    def lease_id(self, operation_id: str) -> str:
        return f"task-lease-{operation_id}"

    def lease_owner(self, operation_id: str) -> str:
        return f"task:{operation_id}"

    @staticmethod
    def _target_key(target: Mapping[str, Any], task_id: str):
        return str(target["device_id"]), str(target["component_id"]), str(task_id)

    def _validate_target(self, request: Mapping[str, Any], command: Mapping[str, Any]) -> None:
        target = command["target"]
        routed = dict(request.get("target") or {})
        for field in ("device_id", "component_id"):
            if routed.get(field) and routed[field] != target[field]:
                raise ValueError(f"routed and document {field} differ")
        if self.expected_device_id and target["device_id"] != self.expected_device_id:
            raise ValueError("task targets another device")

    def _emit_progress(self, context: _MissionContext, *, data_type: str,
                       payload: Mapping[str, Any], urgent: bool = False) -> None:
        del data_type, urgent
        runtime = context.manager.runtime
        status = "cancelled" if context.operation_id in self.cancelled_operations else (
            "succeeded" if runtime.stage == "completed" else (
                "failed" if runtime.stage == "aborted" else "running"))
        sequence = self.event_sequences.get(context.operation_id, 0) + 1
        self.event_sequences[context.operation_id] = sequence
        self.emit_operation(context.operation_id, sequence, status, dict(payload))

    def _write_context_state(self, context: _MissionContext, name: str, value: Any) -> None:
        if not self.state_reporting:
            return
        field = f"task.{_segment(context.operation_id)}.{_segment(name)}"
        if field not in self.state_fields:
            self.register_state(field, "any", persistent=False)
            self.state_fields.add(field)
        self.set_state(field, value, persistent=False)

    def _lease(self, context: _MissionContext, *, acquire: bool, reason: str) -> None:
        if not self.arbiter_instance:
            if self.backend_is_simulation and self.allow_unarbitrated_simulation:
                return
            raise RuntimeError("arbiter_instance is required for mission motion")
        now_ns = time.time_ns()
        document = {
            "schema_version": "1.0", "lease_id": self.lease_id(context.operation_id),
            "resource": {"device_id": context.target["device_id"],
                         "component_id": context.target["component_id"],
                         "resource_group": str(self.config.get("resource_group") or "base-motion")},
            "owner": self.lease_owner(context.operation_id), "task_id": context.task_id,
            "reason": reason, "priority": int(self.config.get("lease_priority", 50)),
            "acquired_at_ns": now_ns, "expires_at_ns": now_ns + self.lease_duration_ns,
        }
        self.call_local({
            "message_type": "uran.control-lease.acquire.v1" if acquire else "uran.control-lease.release.v1",
            "correlation_id": f"{context.operation_id}-lease-{now_ns}",
            "target": {"device_id": context.target["device_id"], "instance_id": self.arbiter_instance,
                       "capability_id": "motion.control.acquire" if acquire else "motion.control.release"},
            "payload": document,
        })

    def _publish_media(self, context: _MissionContext, *, action: str, channel_id: str) -> None:
        if not self.media_instance:
            raise RuntimeError("media_instance is required for mission media actions")
        now_ns = time.time_ns()
        operation_id = f"{context.operation_id}-media-{now_ns}"
        document = {
            "schema_version": "1.0", "operation_id": operation_id,
            "target": {"device_id": context.target["device_id"], "component_id": "camera"},
            "session_id": channel_id, "channel_id": channel_id, "action": action,
            "protocol": str(self.config.get("media_protocol") or "webrtc"), "timestamp_ns": now_ns,
        }
        self.call_local({"message_type": "uran.media.session-command.v1",
                             "correlation_id": operation_id,
                             "target": {**document["target"], "instance_id": self.media_instance,
                                        "capability_id": "media.session"},
                             "payload": document})

    def _new_context(self, command: Mapping[str, Any]) -> _MissionContext:
        operation_id = str(command["operation_id"])
        target = dict(command["target"])
        gateway = _LocalMoveGateway(self, operation_id=operation_id,
                                    task_id=str(command["task_id"]), target=target)
        logger = logging.getLogger(f"uran_autotask.{operation_id}")
        straight = StraightDriveController(
            config=dict(self.config.get("straight_drive") or {}), move_gateway=gateway,
            pose_getter=self.registry.latest_visual_pose, map_pose_getter=self.registry.latest_map_pose,
            now_ns_getter=time.time_ns, logger=logger,
            control_state_getter=lambda: {"control_mode": self.control_mode,
                                          "controller": self.controller,
                                          "battery_level": self.battery_level})
        context = _MissionContext(operation_id, str(command["task_id"]), target, None, gateway, straight)
        context.manager = MissionManager(
            logger=logger, backend=self.backend, camera_capture=self.camera_capture,
            projector=self.projector, closed_loop_manager=self.closed_loop,
            pose_registry=self.registry, control_mode_getter=lambda: self.control_mode,
            controller_getter=lambda: self.controller, battery_level_getter=lambda: self.battery_level,
            now_ns_getter=time.time_ns, stamp_getter=time.time_ns,
            publish_uplink=lambda **kwargs: self._emit_progress(context, **kwargs),
            write_state=lambda name, value: self._write_context_state(context, name, value),
            write_motion_control_lock=lambda active, task_id, reason: self._lease(
                context, acquire=active, reason=reason),
            publish_media_control=lambda **kwargs: self._publish_media(context, **kwargs),
            set_map_pose_reporting_enabled=lambda enabled: None, move_gateway=gateway,
            straight_drive_controller=straight, mission_defaults=self.defaults,
            outdoor_goal_resolver_cfg=dict(self.config.get("outdoor_goal_resolver") or {}),
            outdoor_pose_aligner_cfg=dict(self.config.get("outdoor_pose_aligner") or {}),
            gps_vo_yaw_aligner_cfg=dict(self.config.get("gps_vo_yaw_aligner") or {}),
            outdoor_start_alignment_cfg=dict(self.config.get("outdoor_start_alignment") or {}),
            progress_report_interval_s=float(self.config.get("progress_report_interval_s", 1.0)),
            require_auto_mode=self.require_auto_mode,
            media_actions_cfg=dict(self.config.get("media_actions") or {}))
        return context

    def invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        if request.get("message_type") == "uran.sensors.observation.v1":
            return self._observation(request)
        if request.get("message_type") != "uran.tasks.command.v1":
            raise ValueError("unsupported message_type")
        command = dict(request.get("payload") or {})
        self.validator.validate("tasks/command.schema.json", command)
        self._validate_target(request, command)
        key = self._target_key(command["target"], command["task_id"])
        action = str(command["action"])
        with self.lock:
            if action == "start":
                if key in self.by_target_task:
                    existing = self.contexts[self.by_target_task[key]]
                    if existing.manager.runtime.stage not in TERMINAL_TASK_STAGES:
                        raise ValueError("matching task is already active")
                if self.backend is None:
                    raise RuntimeError("task navigation backend is unavailable")
                if not self.arbiter_instance and not self.allow_unarbitrated_simulation:
                    raise RuntimeError("arbiter_instance is required for mission motion")
                context = self._new_context(command)
                self.contexts[context.operation_id] = context
                self.by_target_task[key] = context.operation_id
                context.manager.handle_task_command(
                    action="start", task_id=context.task_id, task_type=str(command["task_type"]),
                    task_params_json=json.dumps(command.get("parameters") or {}))
            else:
                original = self.by_target_task.get(key)
                if not original:
                    raise ValueError("matching task does not exist")
                context = self.contexts[original]
                if action == "pause":
                    result = context.manager.pause_current_operation(reason="paused_by_command")
                elif action == "resume":
                    result = context.manager.resume_current_operation()
                elif action == "stop":
                    result = context.manager.stop_current_operation(reason="stopped_by_command")
                else:
                    raise ValueError("update_params is not implemented")
                if not result["accepted"]:
                    raise ValueError(result["message"])
            status_payload = context.manager.build_status_payload()
            status = "failed" if context.manager.runtime.stage == "aborted" else (
                "accepted" if action == "start" else "succeeded")
            return {"status": status, "operation_id": request["operation_id"], "payload": status_payload}

    def _observation(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        document = dict(request.get("payload") or {})
        self.validator.validate("sensors/observation.schema.json", document)
        value = document.get("value")
        if not isinstance(value, Mapping):
            raise ValueError("observation value must be an object")
        value = dict(value)
        kind = str(document["observation_type"])
        timestamp_ns = int(document["timestamp_ns"])
        with self.lock:
            if kind == "gps_fix":
                self.registry.update_gps_payload(
                    lat=float(value.get("latitude_deg", value.get("lat"))),
                    lon=float(value.get("longitude_deg", value.get("lon"))),
                    alt=float(value.get("altitude_m", value.get("alt", 0.0))),
                    fix_type=int(value.get("fix_type", 0)), num_sv=int(value.get("num_sv", 0)),
                    timestamp_ns=timestamp_ns)
            elif kind in {"map_pose", "odometry"}:
                self.registry.update_map_pose({**value, "timestamp_ns": timestamp_ns})
            elif kind in {"visual_pose", "visual_odometry"}:
                self.registry.update_visual_pose({**value, "timestamp_ns": timestamp_ns})
            elif kind == "laser_scan":
                scan = VirtualLaserScan(angle_min=float(value["angle_min"]),
                    angle_max=float(value["angle_max"]), angle_increment=float(value["angle_increment"]),
                    range_min=float(value["range_min"]), range_max=float(value["range_max"]),
                    ranges=[float(item) for item in value["ranges"]])
                for context in self.contexts.values():
                    context.straight_drive.update_scan(scan)
            else:
                raise ValueError(f"unsupported task observation_type: {kind}")
        return {"status": "succeeded", "operation_id": request["operation_id"],
                "payload": {"observation_type": kind, "updated": True}}

    def cancel(self, operation_id: str) -> Mapping[str, Any]:
        with self.lock:
            context = self.contexts.get(operation_id)
            if context is None:
                raise ValueError("operation does not exist")
            self.cancelled_operations.add(operation_id)
            result = context.manager.stop_current_operation(reason="cancelled_by_operation")
            if not result["accepted"]:
                raise ValueError(result["message"])
        return {"status": "cancelled", "operation_id": operation_id,
                "payload": context.manager.build_status_payload()}

    def on_state(self, subscription_id: str, field: str,
                 value: Mapping[str, Any], sequence: int) -> None:
        del subscription_id, sequence
        item = value.get("value")
        if field == f"device.{self.expected_device_id}.control.mode":
            self.control_mode = str(item)
        elif field == f"device.{self.expected_device_id}.control.controller":
            self.controller = str(item)
        elif field.endswith(".battery.id_level"):
            self.battery_level = float(item)

    def _tick_loop(self) -> None:
        while not self.stop_event.wait(self.tick_interval_s):
            with self.lock:
                for context in list(self.contexts.values()):
                    if context.manager.runtime.stage in TERMINAL_TASK_STAGES:
                        continue
                    try:
                        context.manager.tick()
                        context.manager.tick_control()
                    except Exception as exc:
                        logging.getLogger(__name__).error("mission tick failed: %s", exc)

    def deactivate(self) -> None:
        self.stop_event.set()
        if hasattr(self, "tick_thread"):
            self.tick_thread.join(timeout=2.0)
        with self.lock:
            for context in self.contexts.values():
                if context.manager.has_active_task():
                    context.manager.stop_current_operation(reason="worker_deactivated")

    def shutdown(self) -> None:
        if self._ros is None:
            return
        rclpy, node, executor, thread = self._ros
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)
        self._ros = None


def worker_factory() -> Worker:
    return AutotaskWorker()


def main() -> int:
    return WorkerServer(worker_factory()).serve()
