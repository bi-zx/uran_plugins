"""SO101 arm worker.

The worker owns one Feetech bus and exposes the generic URAN arm motion
contract.  It keeps the transport behind a small backend so the complete IK
and safety path can run in simulation on CI and on CyberDog2 with pyserial.
"""
import json
import math
import os
import threading
import time
from typing import Any, Dict, Mapping

from uran_interfaces import ContractValidator
from uran_worker_sdk import Worker, WorkerServer, now_ns

from .calibration import (JOINT_NAMES, apply_safety_range, is_complete,
                          load_calibration, raw_to_values, validate_raw_targets,
                          values_to_raw)
from .feetech_bus import FeetechBus, FeetechError, discover_port
from .kinematics import SO101Kinematics


class SimulatedArmBackend(object):
    def __init__(self, positions=None):
        self.positions = list(positions or [2048] * 6)
        self.torque = False

    def open(self):
        return

    def close(self):
        return

    def set_torque(self, enabled):
        self.torque = bool(enabled)

    def read_positions(self):
        return {index + 1: value for index, value in enumerate(self.positions)}

    def write_positions(self, targets):
        self.positions = [int(value) for value in targets]


class FeetechArmBackend(object):
    def __init__(self, port, baudrate, timeout):
        self.port = str(port or "auto")
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.bus = None
        self.resolved_port = ""
        self.torque = False
        self.lock = threading.RLock()

    def open(self):
        with self.lock:
            if self.bus is not None:
                return
            port = self.port
            if port == "auto":
                port = discover_port(self.baudrate)
            if not port:
                raise FeetechError("SO101 serial port was not found")
            bus = FeetechBus(port, self.baudrate, self.timeout)
            bus.open()
            # Probe the complete chain before accepting a hardware backend.
            found = bus.read_positions(range(1, 7))
            if len(found) != 6:
                bus.close()
                raise FeetechError("SO101 requires motors 1..6")
            self.bus = bus
            self.resolved_port = str(port)

    def close(self):
        with self.lock:
            if self.bus is not None:
                self.bus.close()
            self.bus = None
            self.torque = False

    def set_torque(self, enabled):
        self.open()
        self.bus.set_torque(range(1, 7), bool(enabled))
        self.torque = bool(enabled)

    def read_positions(self):
        self.open()
        return self.bus.read_positions(range(1, 7))

    def write_positions(self, targets):
        self.open()
        self.bus.sync_write(42, 2, {index + 1: int(value) for index, value in enumerate(targets)})


class SO101Worker(Worker):
    plugin_id = "uran-so101-arm"
    plugin_version = "0.1.0"
    capabilities = ("motion.arm.joints", "motion.arm.pose", "motion.arm.safety")

    def configure(self, config: Mapping[str, Any]) -> None:
        self.validator = ContractValidator()
        self.expected_device_id = str(config.get("device_id") or self.identity.get("device_id") or "")
        self.expected_component_id = str(config.get("component_id") or self.identity.get("component_id") or "arm")
        self.lock = threading.RLock()
        self.sequences = {}
        self.state_reporting = bool(config.get("state_reporting", True))
        self.torque_on_start = bool(config.get("enable_torque_on_start", False))
        self.allow_motion = bool(config.get("allow_motion", True))
        self.joint_offsets = [float(value) for value in config.get("joint_offsets_deg", [0.0] * 5)]
        if len(self.joint_offsets) != 5:
            raise ValueError("joint_offsets_deg must contain five values")
        package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        # A source checkout has config beside src.  Deployments may relocate
        # the package, so URAN_SO101_CONFIG_DIR is an explicit override.
        default_config = os.environ.get("URAN_SO101_CONFIG_DIR", os.path.join(package_root, "config"))
        self.calibration_file = str(config.get("calibration_file") or os.path.join(default_config, "uran_arm_calibration.yaml"))
        self.safety_range_file = str(config.get("safety_range_file") or os.path.join(default_config, "so101_follower_com9.json"))
        self.urdf_path = str(config.get("urdf_path") or os.path.join(default_config, "so101_chain.urdf"))
        self.calibration = load_calibration(self.calibration_file)
        if os.path.isfile(self.safety_range_file):
            self.calibration = apply_safety_range(self.calibration, self.safety_range_file)
            scale = float(config.get("safety_range_scale", 1.0))
            from .calibration import narrow_safety_range
            self.calibration = narrow_safety_range(self.calibration, scale)
        self.kinematics = SO101Kinematics(self.urdf_path)
        backend_name = str(config.get("backend", "so101_feetech"))
        if backend_name == "simulation":
            self.backend = SimulatedArmBackend()
        elif backend_name in ("so101_feetech", "feetech"):
            self.backend = FeetechArmBackend(config.get("serial_port", "auto"), config.get("baudrate", 1000000), config.get("serial_timeout_s", 0.08))
        else:
            raise ValueError("unsupported SO101 backend: {}".format(backend_name))
        self.max_step_degrees = float(config.get("ik_max_step_degrees", 8.0))
        self.ik_max_iterations = int(config.get("ik_max_iterations", 80))
        self.ik_damping = float(config.get("ik_damping", 0.02))
        self.ik_tolerance_mm = float(config.get("ik_tolerance_mm", 1.5))
        self.ik_acceptance_mm = float(config.get("ik_acceptance_mm", 5.0))
        self.gripper_open_rad = float(config.get("gripper_open_rad", 1.57))
        self.gripper_closed_rad = float(config.get("gripper_closed_rad", 0.0))
        self.last_raw = [2048] * 6
        self.last_values = [0.0] * 6
        self.last_error = ""
        self.connected = False
        self.torque_enabled = False

    def activate(self) -> None:
        self._register_states()
        try:
            self.backend.open()
            self.connected = True
            self._refresh()
            if self.torque_on_start:
                self.backend.set_torque(True)
                self.torque_enabled = True
        except Exception as exc:
            # Keep the component loaded and observable when the arm is unplugged;
            # commands return a clear serial error until it is connected.
            self.last_error = str(exc)
            self.connected = False
        self._publish_states()

    def _register_states(self):
        if not self.state_reporting:
            return
        prefix = "device.{}.component.{}.".format(self.expected_device_id, self.expected_component_id)
        for name, kind in (("connection.connected", "bool"), ("torque.enabled", "bool"), ("calibration.complete", "bool"), ("position.raw", "json"), ("position.values", "json")):
            self.register_state(prefix + name, kind, persistent=False)

    def _set(self, suffix, value):
        if self.state_reporting:
            prefix = "device.{}.component.{}.".format(self.expected_device_id, self.expected_component_id)
            self.set_state(prefix + suffix, value, persistent=False)

    def _refresh(self):
        positions = self.backend.read_positions()
        self.last_raw = [int(positions[index + 1]) for index in range(6)]
        if is_complete(self.calibration):
            self.last_values = raw_to_values(self.calibration, positions)
        self._publish_states()

    def _publish_states(self):
        self._set("connection.connected", bool(self.connected))
        self._set("torque.enabled", bool(self.torque_enabled))
        self._set("calibration.complete", bool(is_complete(self.calibration)))
        self._set("position.raw", list(self.last_raw))
        self._set("position.values", list(self.last_values))

    def _ensure_backend(self):
        if not self.connected:
            self.backend.open()
            self.connected = True
        self._refresh()

    def _target_values(self, body):
        kind = body.get("kind")
        current = list(self.last_values)
        if kind == "arm_joints":
            names = list(body.get("joint_names") or [])
            positions = list(body.get("positions_rad") or [])
            if len(names) != len(positions) or not names:
                raise ValueError("joint_names and positions_rad must have equal nonzero length")
            for name, value in zip(names, positions):
                if name not in JOINT_NAMES:
                    raise ValueError("unknown SO101 joint: {}".format(name))
                if not math.isfinite(float(value)):
                    raise ValueError("joint position must be finite")
                index = JOINT_NAMES.index(name)
                current[index] = math.degrees(float(value)) if name != "gripper" else self._gripper_percent(float(value))
            return current
        if kind == "arm_pose":
            position = body.get("position_m")
            if not is_complete(self.calibration):
                raise ValueError("calibration is incomplete; Cartesian IK is disabled")
            seed = current[:5]
            limits = []
            for name in JOINT_NAMES[:5]:
                item = self.calibration["joints"][name]
                direction = float(item.get("direction", 1))
                limits.append((direction * float(item["raw_min"] - item["raw_center"]) * 360.0 / 4095.0,
                               direction * float(item["raw_max"] - item["raw_center"]) * 360.0 / 4095.0))
                limits[-1] = tuple(sorted(limits[-1]))
            result = self.kinematics.inverse_position([float(v) * 1000.0 for v in position], seed,
                max_iterations=self.ik_max_iterations, joint_limits_degrees=limits,
                damping=self.ik_damping, max_step_degrees=self.max_step_degrees,
                tolerance_mm=self.ik_tolerance_mm, acceptance_mm=self.ik_acceptance_mm)
            if not result["success"]:
                raise ValueError("IK did not converge: {:.2f} mm".format(result["position_error_mm"]))
            current[:5] = result["joints"]
            return current
        raise ValueError("unsupported arm payload kind: {}".format(kind))

    def _gripper_percent(self, value):
        lo, hi = sorted((self.gripper_closed_rad, self.gripper_open_rad))
        if hi <= lo:
            raise ValueError("invalid gripper radian range")
        return min(100.0, max(0.0, (float(value) - lo) * 100.0 / (hi - lo)))

    def _write_values(self, values):
        if not self.allow_motion:
            raise ValueError("arm motion is disabled by profile")
        if not is_complete(self.calibration):
            raise ValueError("calibration is incomplete or disabled")
        targets = values_to_raw(self.calibration, values)
        validate_raw_targets(self.calibration, targets)
        self._ensure_backend()
        if not self.torque_enabled:
            self.backend.set_torque(True)
            self.torque_enabled = True
        self.backend.write_positions(targets)
        self.last_raw = list(targets)
        self.last_values = list(values)
        self._publish_states()
        return targets

    def invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        with self.lock:
            if request.get("message_type") != "uran.motion.command.v1":
                raise ValueError("unsupported message_type")
            document = dict(request.get("payload") or {})
            self.validator.validate("motion/command.schema.json", document)
            target = document["target"]
            if target.get("device_id") != self.expected_device_id or target.get("component_id") != self.expected_component_id:
                raise ValueError("motion command targets another component")
            if int(document["valid_until_ns"]) <= now_ns():
                raise ValueError("motion command is expired")
            key = (target["device_id"], target["component_id"], document["controller"])
            sequence = int(document["sequence"])
            if sequence <= self.sequences.get(key, -1):
                raise ValueError("motion command sequence is stale or duplicated")
            self.sequences[key] = sequence
            body = dict(document.get("payload") or {})
            kind = body.get("kind")
            if kind == "safety_action":
                action = body.get("action")
                if action not in ("stop", "emergency_stop"):
                    raise ValueError("unsupported arm safety action")
                if action == "emergency_stop" and self.connected:
                    self.backend.set_torque(False)
                    self.torque_enabled = False
                self._publish_states()
                return {"status": "succeeded", "operation_id": request["operation_id"], "payload": {"accepted": True, "action": action}}
            values = self._target_values(body)
            targets = self._write_values(values)
            return {"status": "succeeded", "operation_id": request["operation_id"], "payload": {"accepted": True, "joint_names": list(JOINT_NAMES), "positions_rad": [math.radians(v) for v in values[:5]] + [self._gripper_rad(values[5])], "raw_targets": targets}}

    def _gripper_rad(self, percent):
        return self.gripper_closed_rad + (self.gripper_open_rad - self.gripper_closed_rad) * float(percent) / 100.0

    def deactivate(self) -> None:
        with self.lock:
            if self.connected and self.torque_enabled:
                try:
                    self.backend.set_torque(False)
                except Exception:
                    pass
            self.torque_enabled = False
            self._publish_states()

    def shutdown(self) -> None:
        backend = getattr(self, "backend", None)
        try:
            if backend is not None:
                backend.close()
        finally:
            self.connected = False


def worker_factory():
    return SO101Worker()


def main():
    return WorkerServer(worker_factory()).serve()
