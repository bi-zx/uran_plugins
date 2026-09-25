from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Protocol


class Px4Transport(Protocol):
    def publish_position(self, command: Mapping[str, Any]) -> None: ...
    def publish_takeoff_land(self, action: str) -> None: ...


@dataclass
class MotionCommand:
    action: str = ""
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_z: float = 0.0
    target_yaw: float | None = None
    source: str = ""

    @classmethod
    def from_contract(cls, document: Mapping[str, Any]) -> "MotionCommand":
        payload = dict(document.get("payload") or {})
        kind = str(payload.get("kind") or "")
        if kind == "safety_action":
            return cls(action=str(payload.get("action") or ""), source=str(document.get("controller") or ""))
        if kind == "base_posture":
            return cls(action="posture", target_yaw=float(payload["yaw_rad"]),
                       source=str(document.get("controller") or ""))
        if kind != "base_velocity":
            raise ValueError(f"unsupported motion payload kind: {kind}")
        linear = payload.get("linear_mps") or {}
        angular = payload.get("angular_rps") or {}
        posture = payload.get("posture_rad") or {}
        target_yaw = posture.get("yaw")
        if target_yaw is not None:
            target_yaw = float(target_yaw)
        return cls(linear_x=float(linear.get("x", 0.0)), linear_y=float(linear.get("y", 0.0)),
                   linear_z=float(linear.get("z", 0.0)), angular_z=float(angular.get("z", 0.0)),
                   target_yaw=target_yaw, source=str(document.get("controller") or ""))


@dataclass(frozen=True)
class MotionResult:
    accepted: bool
    code: str
    message: str = ""
    applied: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {"accepted": self.accepted, "code": self.code,
                "message": self.message, "applied": dict(self.applied)}


class Px4Controller:
    """PX4/px4ctrl policy independent of rospy and generated message classes.

    Unified velocity commands are body-frame (x forward, y left, z up). The
    px4ctrl PositionCommand target is world-frame, so this class integrates a
    world setpoint using the latest odometry yaw and republishes it frequently
    enough to keep OFFBOARD alive.
    """

    def __init__(self, transport: Px4Transport, *, clock: Callable[[], float] = time.monotonic,
                 linear_limit: float = 3.0, angular_limit: float = 1.0,
                 cmd_timeout_s: float = 0.5, land_retry_delay_s: float = 0.6,
                 rc_throttle: tuple[int, int] = (1400, 1600),
                 rc_high: tuple[int, int] = (1900, 2100)) -> None:
        self.transport = transport
        self.clock = clock
        self.linear_limit = max(0.0, float(linear_limit))
        self.angular_limit = max(0.0, float(angular_limit))
        self.cmd_timeout_s = max(0.01, float(cmd_timeout_s))
        self.land_retry_delay_s = max(0.0, float(land_retry_delay_s))
        self.rc_throttle, self.rc_high = rc_throttle, rc_high
        self.ready = False
        self.takeoff_pending = False
        self.rc_channels: list[int] = []
        self.odom: tuple[float, float, float, float] | None = None
        self.position: list[float] | None = None
        self.yaw = 0.0
        self.velocity = (0.0, 0.0, 0.0)
        self.yaw_rate = 0.0
        self.request = MotionCommand()
        self.setpoint_time: float | None = None
        self.last_command_time: float | None = None
        self.land_due: float | None = None

    def update_rc(self, channels: list[int] | tuple[int, ...]) -> None:
        self.rc_channels = [int(value) for value in channels]

    def update_odom(self, x: float, y: float, z: float, yaw: float) -> None:
        values = (float(x), float(y), float(z), float(yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("odometry must be finite")
        self.odom = values

    def on_traj_start(self, x: float, y: float, z: float, yaw: float) -> None:
        self.position = [float(x), float(y), float(z)]
        self.yaw = self._normalize(float(yaw))
        self.velocity = (0.0, 0.0, 0.0)
        self.yaw_rate = 0.0
        self.setpoint_time = self.clock()
        self.request = MotionCommand()
        self.last_command_time = None
        self.ready = True
        self.takeoff_pending = False
        self.transport.publish_position(self._position_message())

    def execute(self, command: MotionCommand, *, now: float | None = None) -> MotionResult:
        now = self.clock() if now is None else float(now)
        action = command.action.strip().lower()
        if action == "takeoff":
            if not self._rc_safe():
                return MotionResult(False, "E_RC_UNSAFE", "takeoff blocked by RC safety gate")
            self.ready = False
            self.takeoff_pending = True
            self._reset()
            self.transport.publish_takeoff_land("takeoff")
            return MotionResult(True, "OK", applied={"action": "takeoff"})
        if action in {"land", "return_home", "emergency_stop"}:
            self.ready = False
            self.takeoff_pending = False
            self._reset()
            self.land_due = now + self.land_retry_delay_s
            return MotionResult(True, "OK", applied={"action": action, "land_due": self.land_due})
        if action in {"stop", ""} and action == "stop":
            command = MotionCommand(action="stop")
        if not self.ready:
            return MotionResult(False, "E_NOT_READY", "px4ctrl has not entered CMD_CTRL")
        if not self._ensure_position(now):
            return MotionResult(False, "E_NO_ODOMETRY", "no odometry or trajectory setpoint available")
        self._advance(now)
        self.request = self._bounded(command)
        self.last_command_time = now
        self._advance(now)
        message = self._position_message()
        self.transport.publish_position(message)
        return MotionResult(True, "OK", applied=message)

    def tick(self, *, now: float | None = None) -> MotionResult | None:
        now = self.clock() if now is None else float(now)
        if self.land_due is not None and now >= self.land_due:
            self.land_due = None
            self.transport.publish_takeoff_land("land")
            return MotionResult(True, "OK", applied={"action": "land"})
        if not self.ready:
            return None
        if not self._ensure_position(now):
            return None
        if self.last_command_time is not None and now - self.last_command_time > self.cmd_timeout_s:
            cutoff = self.last_command_time + self.cmd_timeout_s
            if self.setpoint_time is None or cutoff > self.setpoint_time:
                self._advance(cutoff)
            self.request = MotionCommand(action="stop")
        self._advance(now)
        message = self._position_message()
        self.transport.publish_position(message)
        return MotionResult(True, "OK", applied=message)

    def _rc_safe(self) -> bool:
        if len(self.rc_channels) <= 5:
            return False
        throttle, ch5, ch6 = self.rc_channels[2], self.rc_channels[4], self.rc_channels[5]
        return (self.rc_throttle[0] <= throttle <= self.rc_throttle[1] and
                self.rc_high[0] <= ch5 <= self.rc_high[1] and
                self.rc_high[0] <= ch6 <= self.rc_high[1])

    def _bounded(self, command: MotionCommand) -> MotionCommand:
        values = (command.linear_x, command.linear_y, command.linear_z, command.angular_z)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("motion values must be finite")
        norm = math.sqrt(command.linear_x ** 2 + command.linear_y ** 2 + command.linear_z ** 2)
        scale = min(1.0, self.linear_limit / norm) if norm and self.linear_limit >= 0 else 0.0
        yaw_rate = max(-self.angular_limit, min(self.angular_limit, command.angular_z))
        target = command.target_yaw
        if target is not None and not math.isfinite(target):
            target = None
        return MotionCommand(action=command.action, linear_x=command.linear_x * scale,
                             linear_y=command.linear_y * scale, linear_z=command.linear_z * scale,
                             angular_z=yaw_rate, target_yaw=self._normalize(target) if target is not None else None,
                             source=command.source)

    def _ensure_position(self, now: float) -> bool:
        if self.position is not None:
            return True
        if self.odom is None:
            return False
        self.position = list(self.odom[:3])
        self.yaw = self.odom[3]
        self.velocity = (0.0, 0.0, 0.0)
        self.yaw_rate = 0.0
        self.setpoint_time = now
        return True

    def _advance(self, now: float) -> None:
        if self.position is None:
            return
        if self.setpoint_time is None:
            self.setpoint_time = now
        dt = max(0.0, now - self.setpoint_time)
        heading = self.odom[3] if self.odom is not None else self.yaw
        cos_yaw, sin_yaw = math.cos(heading), math.sin(heading)
        vx = cos_yaw * self.request.linear_x - sin_yaw * self.request.linear_y
        vy = sin_yaw * self.request.linear_x + cos_yaw * self.request.linear_y
        vz = self.request.linear_z
        self.position[0] += vx * dt
        self.position[1] += vy * dt
        self.position[2] += vz * dt
        if self.request.target_yaw is not None:
            self.yaw = self.request.target_yaw
            self.yaw_rate = 0.0
        else:
            self.yaw = self._normalize(self.yaw + self.request.angular_z * dt)
            self.yaw_rate = self.request.angular_z
        self.velocity = (vx, vy, vz)
        self.setpoint_time = now

    def _position_message(self) -> dict[str, Any]:
        if self.position is None:
            raise RuntimeError("setpoint not initialized")
        return {"position": {"x": self.position[0], "y": self.position[1], "z": self.position[2]},
                "velocity": {"x": self.velocity[0], "y": self.velocity[1], "z": self.velocity[2]},
                "yaw": self.yaw, "yaw_rate": self.yaw_rate, "frame_id": "world"}

    def _reset(self) -> None:
        self.position = None
        self.velocity = (0.0, 0.0, 0.0)
        self.yaw_rate = 0.0
        self.setpoint_time = None
        self.request = MotionCommand()
        self.last_command_time = None

    @staticmethod
    def _normalize(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))
