import json
import math
import threading
import time
from typing import Any, Mapping

from uran_worker_sdk import Worker, WorkerServer, now_ns
from uran_interfaces import ContractValidator

from .controller import MotionCommand, MotionController, SimulatedBackend


class CyberDogWorker(Worker):
    plugin_id = "uran-cyberdog2-motion"
    plugin_version = "0.1.0"
    capabilities = ("motion.base.velocity", "motion.base.posture", "motion.base.safety")

    def configure(self, config: Mapping[str, Any]) -> None:
        self.validator = ContractValidator()
        backend_name = str(config.get("backend", "unavailable"))
        self._ros = None
        if backend_name == "simulation":
            backend = SimulatedBackend()
        elif backend_name == "cyberdog2_ros2":
            try:
                import rclpy
                from rclpy.executors import MultiThreadedExecutor
                from .ros2_backend import CyberDog2Ros2Backend
            except ImportError as exc:
                raise RuntimeError("ROS2/CyberDog2 runtime is unavailable") from exc
            rclpy.init(args=None)
            node = rclpy.create_node(str(config.get("node_name", "uran_cyberdog2_worker")))
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(node)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            self._ros = (rclpy, node, executor, thread)
            backend = CyberDog2Ros2Backend(
                node, namespace=str(config.get("namespace", "")),
                result_timeout_s=float(config.get("result_timeout_s", 3.0)),
                servo_publish_hz=float(config.get("servo_publish_hz", 20.0)),
                bms_status_topic=str(config.get("bms_status_topic", "bms_status")),
                cmd_timeout_s=float(config.get("cmd_timeout_s", 0.5)),
                recovery_wait_timeout_s=float(config.get("recovery_wait_timeout_s", 5.0)),
                auto_recover_from_estop=bool(config.get("auto_recover_from_estop", True)),
                charging_recover_timeout_s=float(config.get("charging_recover_timeout_s", 1.0)),
                idle_to_stand_timeout_s=float(config.get("idle_to_stand_timeout_s", 0.0)),
                auto_stand_retry_interval_s=float(config.get("auto_stand_retry_interval_s", 10.0)),
                zero_velocity_epsilon=float(config.get("zero_velocity_epsilon", 0.02)))
            self._ros = (rclpy, node, executor, thread)
        else:
            raise ValueError(f"unsupported backend: {backend_name}")
        self.controller = MotionController(
            backend, linear_limit=float(config.get("linear_limit", 1.0)),
            angular_limit=float(config.get("angular_limit", 1.0)),
            failsafe_timeout_s=float(config.get("failsafe_timeout_s", 5.0)),
            recover_delay_s=float(config.get("failsafe_recover_delay_s", 2.0)))
        self.expected_device_id = str(config.get("device_id") or self.identity.get("device_id") or "")
        self.expected_component_id = str(config.get("component_id") or self.identity.get("component_id") or "")
        self.expected_arbiter_instance = str(config.get("required_arbiter_instance") or "")
        self.sequences = {}
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.connection_field = str(config.get("connection_field") or "")
        self.parameter_prefix = f"device.{self.expected_device_id}.component.{self.expected_component_id}.parameters."
        self.parameter_fields = {"linear_vel_limit": "linear_limit", "angular_vel_limit": "angular_limit"}
        self.persistent_parameters = bool(config.get("persistent_parameters", False))
        self.state_reporting = bool(config.get("state_reporting", True))
        self.controller.control_mode = str(config.get("control_mode", "manual"))
        if self.connection_field:
            self.controller.heartbeat(False)

    def activate(self) -> None:
        if self.state_reporting:
            for field, attribute in self.parameter_fields.items():
                key = self.parameter_prefix + field
                self.register_state(key, "double", persistent=self.persistent_parameters)
                restored = self.state_snapshot().get(key, getattr(self.controller, attribute))
                if not isinstance(restored, (float, int)) or not math.isfinite(restored) or restored < 0:
                    raise ValueError(f"invalid persisted parameter: {key}")
                setattr(self.controller, attribute, float(restored))
                self.set_state(key, float(restored), persistent=self.persistent_parameters)
        self.stop_event.clear()
        self.tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self.tick_thread.start()

    def _tick_loop(self) -> None:
        while not self.stop_event.wait(0.05):
            with self.lock:
                self.controller.tick()

    def on_state(self, subscription_id, field, value, sequence):
        item = value.get("value")
        with self.lock:
            if field == self.connection_field:
                self.controller.heartbeat(bool(item))
            elif field == f"device.{self.expected_device_id}.control.mode":
                if item not in {"manual", "auto"}:
                    raise ValueError("invalid control mode")
                if self.controller.control_mode != item:
                    self.controller.execute(MotionCommand(action="stop"))
                self.controller.control_mode = item

    def _parameters(self, request):
        document = request["payload"]
        target = document.get("target", {})
        if target.get("device_id") != self.expected_device_id or target.get("component_id") != self.expected_component_id:
            raise ValueError("parameter update targets another component")
        values = document.get("params_json", document.get("parameters", {}))
        if isinstance(values, str):
            values = json.loads(values)
        if not isinstance(values, dict) or not values or set(values) - self.parameter_fields.keys():
            raise ValueError("unsupported motion parameters")
        for value in values.values():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError("motion limits must be finite nonnegative numbers")
        for field, value in values.items():
            if self.state_reporting:
                self.set_state(self.parameter_prefix + field, float(value), persistent=self.persistent_parameters)
            setattr(self.controller, self.parameter_fields[field], float(value))
        return {"status": "succeeded", "payload": {"parameters": values}}

    def invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        with self.lock:
            if request.get("message_type") == "uran.legacy.param-update.v1":
                return self._parameters(request)
            return self._motion(request)

    def _motion(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        if request.get("message_type") != "uran.motion.command.v1":
            raise ValueError("unsupported message_type")
        document = dict(request.get("payload") or {})
        self.validator.validate("motion/command.schema.json", document)
        target = document["target"]
        routed_target = dict(request.get("target") or {})
        if (routed_target.get("device_id") and
                routed_target.get("device_id") != target["device_id"]):
            raise ValueError("routed and document device_id differ")
        if (routed_target.get("component_id") and
                routed_target.get("component_id") != target["component_id"]):
            raise ValueError("routed and document component_id differ")
        if self.expected_device_id and target["device_id"] != self.expected_device_id:
            raise ValueError("motion command targets another device")
        if self.expected_component_id and target["component_id"] != self.expected_component_id:
            raise ValueError("motion command targets another component")
        if int(document["valid_until_ns"]) <= now_ns():
            raise ValueError("motion command is expired")
        sequence_key = (target["device_id"], target["component_id"], document["controller"])
        sequence = int(document["sequence"])
        if sequence <= self.sequences.get(sequence_key, -1):
            raise ValueError("motion command sequence is stale or duplicated")
        body = dict(document.get("payload") or document)
        kind = body.get("kind", "")
        safety_bypass = (kind == "safety_action" and
                         body.get("action") in {"stop", "emergency_stop"})
        source = dict(request.get("source") or {})
        if (self.expected_arbiter_instance and not safety_bypass and
                source.get("instance_id") != self.expected_arbiter_instance):
            raise ValueError("motion command did not pass through the configured control arbiter")
        command = MotionCommand(
            controller=str(document.get("controller", "cloud")),
            timestamp_ns=int(document.get("issued_at_ns", 0)),
            command_seq=int(document.get("sequence", 0)),
            source=str(document.get("metadata", {}).get("source", "")),
            extra=dict(document.get("metadata", {})),
        )
        if kind == "base_velocity":
            linear = body.get("linear_mps", {})
            angular = body.get("angular_rps", {})
            command.linear_vel_x = linear.get("x", 0.0)
            command.linear_vel_y = linear.get("y", 0.0)
            command.linear_vel_z = linear.get("z", 0.0)
            command.angular_vel_z = angular.get("z", 0.0)
            posture = body.get("posture_rad", {})
            command.target_roll = posture.get("roll", 0.0)
            command.target_pitch = posture.get("pitch", 0.0)
            command.target_yaw = posture.get("yaw", 0.0)
        elif kind == "base_posture":
            command.target_roll = body.get("roll_rad", 0.0)
            command.target_pitch = body.get("pitch_rad", 0.0)
            command.target_yaw = body.get("yaw_rad", 0.0)
        elif kind == "safety_action":
            command.action = str(body.get("action", ""))
        else:
            raise ValueError(f"unsupported motion payload kind: {kind}")
        command = MotionCommand.from_dict(vars(command))
        self.sequences[sequence_key] = sequence
        result = self.controller.execute(command)
        return {"status": "succeeded" if result.accepted else "failed",
                "operation_id": request["operation_id"], "payload": result.to_dict()}

    def deactivate(self) -> None:
        if hasattr(self, "stop_event"):
            self.stop_event.set()
        if hasattr(self, "tick_thread"):
            self.tick_thread.join(timeout=2)
        if hasattr(self, "controller"):
            self.controller.execute(MotionCommand(action="stop"))

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
    return CyberDogWorker()


def main() -> int:
    return WorkerServer(worker_factory()).serve()
