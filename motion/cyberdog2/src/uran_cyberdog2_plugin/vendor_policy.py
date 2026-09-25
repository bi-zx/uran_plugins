"""CyberDog2 vendor-state policy with an injectable transport and clock."""

import time
from typing import Any, Callable, Dict

try:
    from typing import Protocol
except ImportError:  # Python 3.6
    class Protocol(object):
        pass

from .controller import MotionCommand, MotionResult

MOTION_WALK_ADAPTIVELY = 304
MOTION_RECOVERY_STAND = 111
MOTION_GET_DOWN = 101
MOTION_ESTOP = 0
SWITCH_NORMAL = 0
SWITCH_ESTOP = 2
SWITCH_CHARGING = 14


class VendorTransport(Protocol):
    def publish_servo(self, *, vx: float, vy: float, wz: float,
                      roll: float = 0.0, pitch: float = 0.0) -> None: ...
    def motion_result(self, motion_id: int) -> MotionResult: ...


class CyberDogPolicyBackend:
    """Preserves the legacy CyberDog2 safety and keepalive behavior."""

    def __init__(self, transport: VendorTransport, *, cmd_timeout_s: float = 0.5,
                 recovery_wait_timeout_s: float = 5.0,
                 auto_recover_from_estop: bool = True,
                 charging_recover_timeout_s: float = 1.0,
                 idle_to_stand_timeout_s: float = 0.0,
                 auto_stand_retry_interval_s: float = 10.0,
                 zero_velocity_epsilon: float = 0.02,
                 clock: Callable[[], float] = time.monotonic,
                 sleeper: Callable[[float], None] = time.sleep):
        self.transport = transport
        self.cmd_timeout_s = max(0.0, float(cmd_timeout_s))
        self.recovery_wait_timeout_s = max(0.0, float(recovery_wait_timeout_s))
        self.auto_recover_from_estop = bool(auto_recover_from_estop)
        self.charging_recover_timeout_s = max(0.0, float(charging_recover_timeout_s))
        self.idle_to_stand_timeout_s = max(0.0, float(idle_to_stand_timeout_s))
        self.auto_stand_retry_interval_s = max(0.0, float(auto_stand_retry_interval_s))
        self.zero_velocity_epsilon = max(0.0, float(zero_velocity_epsilon))
        self.clock = clock
        self.sleeper = sleeper
        self.switch_status = SWITCH_NORMAL
        self.charger_connected = None
        self.charger_disconnected_since = None
        self.last_execute_ts = 0.0
        self.idle_zero_since = None
        self.auto_stand_sent = False
        self.auto_stand_retry_after = 0.0
        self.suspended = False
        self.last_velocity = {"linear_x": 0.0, "linear_y": 0.0,
                              "linear_z": 0.0, "angular_z": 0.0}

    def execute(self, command: MotionCommand) -> MotionResult:
        if command.action == "emergency_stop":
            self._reset_motion(keep_idle=False)
            self._publish_zero()
            return self.transport.motion_result(MOTION_ESTOP)
        if command.action == "stop":
            self._reset_motion(keep_idle=True)
            self._publish_zero()
            return MotionResult(True, "OK", applied=dict(self.last_velocity))
        if self.suspended:
            return MotionResult(False, "E_BACKEND_SUSPENDED")

        motion_id = command.extra.get("motion_id")
        if motion_id is not None:
            ready = self._ensure_ready(command, int(motion_id))
            if ready is not None:
                return ready
            self._reset_motion(keep_idle=False)
            return self.transport.motion_result(int(motion_id))
        if command.action in {"stand", "sit"}:
            requested = MOTION_RECOVERY_STAND if command.action == "stand" else MOTION_GET_DOWN
            ready = self._ensure_ready(command, requested)
            if ready is not None:
                return ready
            self._reset_motion(keep_idle=False)
            return self.transport.motion_result(requested)

        ready = self._ensure_ready(command, None)
        if ready is not None:
            return ready
        self.last_execute_ts = self.clock()
        if self._is_zero(command):
            if self.idle_zero_since is None:
                self.idle_zero_since = self.last_execute_ts
        else:
            self.idle_zero_since = None
            self.auto_stand_sent = False
            self.auto_stand_retry_after = 0.0
        self.transport.publish_servo(vx=command.linear_vel_x, vy=command.linear_vel_y,
                                     wz=command.angular_vel_z, roll=command.target_roll,
                                     pitch=command.target_pitch)
        self.last_velocity = {"linear_x": command.linear_vel_x,
                              "linear_y": command.linear_vel_y, "linear_z": 0.0,
                              "angular_z": command.angular_vel_z}
        return MotionResult(True, "OK", applied=dict(self.last_velocity))

    def stop(self, *, emergency: bool = False) -> MotionResult:
        return self.execute(MotionCommand(action="emergency_stop" if emergency else "stop"))

    def _ensure_ready(self, command: MotionCommand, motion_id) -> Any:
        if self.switch_status == SWITCH_NORMAL:
            return None
        if motion_id == MOTION_ESTOP:
            return None
        if self.switch_status == SWITCH_ESTOP and motion_id == MOTION_RECOVERY_STAND:
            return None
        has_motion = any(abs(value) > 1e-6 for value in (
            command.linear_vel_x, command.linear_vel_y, command.angular_vel_z,
            command.target_roll, command.target_pitch))
        if self.switch_status == SWITCH_ESTOP and self.auto_recover_from_estop and has_motion:
            result = self.transport.motion_result(MOTION_RECOVERY_STAND)
            if not result.accepted:
                return MotionResult(False, "E_RECOVERY_FAILED", result.message)
            deadline = self.clock() + self.recovery_wait_timeout_s
            while self.switch_status != SWITCH_NORMAL and self.clock() < deadline:
                self.sleeper(0.05)
            if self.switch_status == SWITCH_NORMAL:
                return None
            return MotionResult(False, "E_RECOVERY_TIMEOUT",
                                f"switch_status={self.switch_status}")
        return MotionResult(False, "E_VENDOR_SWITCH_STATUS",
                            f"switch_status={self.switch_status}")

    def update_motion_status(self, switch_status: int) -> None:
        status = int(switch_status)
        if status == SWITCH_CHARGING and self._charger_disconnected_timed_out():
            status = SWITCH_NORMAL
        self.switch_status = status

    def update_bms(self, *, connected: bool) -> None:
        self.charger_connected = bool(connected)
        if self.charger_connected:
            self.charger_disconnected_since = None
        elif self.charger_disconnected_since is None:
            self.charger_disconnected_since = self.clock()

    def tick(self) -> None:
        now = self.clock()
        if self.last_execute_ts > 0.0 and now - self.last_execute_ts > self.cmd_timeout_s:
            if self.idle_zero_since is None:
                self.idle_zero_since = now
            self._publish_zero()
        if self.switch_status == SWITCH_CHARGING and self._charger_disconnected_timed_out():
            self.switch_status = SWITCH_NORMAL
        if (self.idle_to_stand_timeout_s > 0.0 and self.idle_zero_since is not None and
                not self.auto_stand_sent and self.switch_status == SWITCH_NORMAL and
                now >= self.auto_stand_retry_after and
                now - self.idle_zero_since >= self.idle_to_stand_timeout_s):
            self.last_execute_ts = 0.0
            result = self.transport.motion_result(MOTION_RECOVERY_STAND)
            if result.accepted:
                self.auto_stand_sent = True
                self.idle_zero_since = None
                self.auto_stand_retry_after = 0.0
            else:
                self.idle_zero_since = now
                self.auto_stand_retry_after = now + self.auto_stand_retry_interval_s

    def _charger_disconnected_timed_out(self) -> bool:
        return (self.charger_connected is False and
                self.charger_disconnected_since is not None and
                self.clock() - self.charger_disconnected_since >= self.charging_recover_timeout_s)

    def _is_zero(self, command: MotionCommand) -> bool:
        return all(abs(value) <= self.zero_velocity_epsilon for value in (
            command.linear_vel_x, command.linear_vel_y, command.angular_vel_z,
            command.target_roll, command.target_pitch))

    def _publish_zero(self) -> None:
        self.transport.publish_servo(vx=0.0, vy=0.0, wz=0.0)
        self.last_velocity = {"linear_x": 0.0, "linear_y": 0.0,
                              "linear_z": 0.0, "angular_z": 0.0}

    def _reset_motion(self, *, keep_idle: bool) -> None:
        self.last_execute_ts = 0.0
        self.idle_zero_since = self.clock() if keep_idle else None
        self.auto_stand_sent = False
        self.auto_stand_retry_after = 0.0

    def suspend(self) -> None:
        self._publish_zero()
        self._reset_motion(keep_idle=False)
        self.suspended = True

    def resume(self) -> None:
        self.suspended = False

    def state(self) -> Dict[str, Any]:
        return {"switch_status": self.switch_status, "suspended": self.suspended,
                "charger_connected": self.charger_connected,
                "last_velocity": dict(self.last_velocity), "simulation": False}
