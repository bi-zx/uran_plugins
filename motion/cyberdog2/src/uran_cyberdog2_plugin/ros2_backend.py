"""Real CyberDog2 ROS2 transport for :class:`CyberDogPolicyBackend`.

The backend targets Xiaomi's ``protocol`` package. It is installable without
ROS, but construction fails clearly unless those vendor interfaces exist.
Hardware execution has not been validated in this migration environment.
"""

from __future__ import annotations

import time
from typing import Any, Dict

from .controller import MotionCommand, MotionResult
from .vendor_policy import CyberDogPolicyBackend

SOURCE_APP = -1
MOTION_WALK_ADAPTIVELY = 304


class _Ros2Transport:
    def __init__(self, node, namespace: str, result_timeout_s: float):
        try:
            from protocol.msg import MotionServoCmd, MotionStatus, BmsStatus
            from protocol.srv import MotionResultCmd
        except ImportError as exc:
            raise RuntimeError("CyberDog2 protocol ROS2 package is unavailable") from exc
        ns = namespace.rstrip("/")
        self.node = node
        self.MotionServoCmd = MotionServoCmd
        self.MotionResultCmd = MotionResultCmd
        self.publisher = node.create_publisher(MotionServoCmd, ns + "/motion_servo_cmd", 10)
        self.client = node.create_client(MotionResultCmd, ns + "/motion_result_cmd")
        self.MotionStatus = MotionStatus
        self.BmsStatus = BmsStatus
        self.namespace = ns
        self.result_timeout_s = float(result_timeout_s)

    def publish_servo(self, *, vx: float, vy: float, wz: float,
                      roll: float = 0.0, pitch: float = 0.0) -> None:
        message = self.MotionServoCmd()
        message.motion_id = MOTION_WALK_ADAPTIVELY
        message.cmd_type = 0
        message.cmd_source = SOURCE_APP
        message.vel_des = [float(vx), float(vy), float(wz)]
        message.rpy_des = [float(roll), float(pitch), 0.0]
        message.step_height = [0.05, 0.05]
        self.publisher.publish(message)

    def motion_result(self, motion_id: int) -> MotionResult:
        if not self.client.service_is_ready():
            return MotionResult(False, "E_VENDOR_SERVICE_UNAVAILABLE")
        request = self.MotionResultCmd.Request()
        request.motion_id = int(motion_id)
        request.cmd_source = SOURCE_APP
        request.vel_des = [0.0, 0.0, 0.0]
        request.rpy_des = [0.0, 0.0, 0.0]
        request.pos_des = [0.0, 0.0, 0.0]
        request.acc_des = [0.0] * 6
        request.ctrl_point = [0.0, 0.0, 0.0]
        request.foot_pose = [0.0] * 6
        request.step_height = [0.0, 0.0]
        request.duration = 0
        future = self.client.call_async(request)
        deadline = time.monotonic() + self.result_timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return MotionResult(False, "E_VENDOR_TIMEOUT")
        response = future.result()
        return MotionResult(bool(response.result),
                            "OK" if response.result else "E_VENDOR_REJECTED",
                            f"code={response.code}")


class CyberDog2Ros2Backend:
    def __init__(self, node, *, namespace: str = "", result_timeout_s: float = 3.0,
                 servo_publish_hz: float = 20.0, bms_status_topic: str = "bms_status",
                 **policy_config):
        if servo_publish_hz <= 0:
            raise ValueError("servo_publish_hz must be positive")
        self.transport = _Ros2Transport(node, namespace, result_timeout_s)
        self.policy = CyberDogPolicyBackend(self.transport, **policy_config)
        ns = namespace.rstrip("/")
        self.status_subscription = node.create_subscription(
            self.transport.MotionStatus, ns + "/motion_status", self._on_motion_status, 10)
        topic = bms_status_topic if bms_status_topic.startswith("/") else (
            f"{ns}/{bms_status_topic}" if ns else bms_status_topic)
        self.bms_subscription = node.create_subscription(
            self.transport.BmsStatus, topic, self._on_bms, 10)
        # This timer retains the legacy 20 Hz timeout/zero-velocity safety loop.
        self.keepalive_timer = node.create_timer(1.0 / servo_publish_hz, self.policy.tick)
        self.battery_level = None

    def _on_motion_status(self, message) -> None:
        self.policy.update_motion_status(int(message.switch_status))

    def _on_bms(self, message) -> None:
        self.battery_level = max(0.0, min(100.0, float(message.batt_soc)))
        connected = any((bool(message.power_wired_charging),
                         bool(message.power_wp_charging),
                         bool(message.power_finished_charging),
                         bool(message.power_expower_supply)))
        self.policy.update_bms(connected=connected)

    def execute(self, command: MotionCommand) -> MotionResult:
        return self.policy.execute(command)

    def stop(self, *, emergency: bool = False) -> MotionResult:
        return self.policy.stop(emergency=emergency)

    def suspend(self) -> None:
        self.policy.suspend()

    def resume(self) -> None:
        self.policy.resume()

    def state(self) -> Dict[str, Any]:
        state = self.policy.state()
        state["battery_level"] = self.battery_level
        return state
