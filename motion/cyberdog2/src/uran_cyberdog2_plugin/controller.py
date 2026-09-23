"""CyberDog2 motion policy separated from the ROS2 vendor transport."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
import math
import time
from typing import Any, Dict, Optional, Protocol


@dataclass
class MotionCommand:
    controller: str = "cloud"
    action: str = ""
    linear_vel_x: float = 0.0
    linear_vel_y: float = 0.0
    linear_vel_z: float = 0.0
    angular_vel_z: float = 0.0
    target_roll: float = 0.0
    target_pitch: float = 0.0
    target_yaw: float = 0.0
    timestamp_ns: int = 0
    command_seq: int = 0
    source: str = ""
    extra: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "MotionCommand":
        known = {item.name for item in cls.__dataclass_fields__.values()}
        values = {key: payload[key] for key in known if key in payload}
        values["extra"] = dict(payload.get("extra") or {})
        command = cls(**values)
        for name in ("linear_vel_x", "linear_vel_y", "linear_vel_z",
                     "angular_vel_z", "target_roll", "target_pitch", "target_yaw"):
            try:
                value = float(getattr(command, name))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{name} must be numeric") from exc
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            setattr(command, name, value)
        return command


@dataclass
class MotionResult:
    accepted: bool
    code: str
    message: str = ""
    requested: Dict[str, float] = field(default_factory=dict)
    applied: Dict[str, float] = field(default_factory=dict)
    backend: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class MotionBackend(Protocol):
    def execute(self, command: MotionCommand) -> MotionResult: ...
    def stop(self, *, emergency: bool = False) -> MotionResult: ...
    def suspend(self) -> None: ...
    def resume(self) -> None: ...
    def state(self) -> Dict[str, Any]: ...


class SimulatedBackend:
    """Deterministic backend for CI. It never talks to a physical actuator."""

    def __init__(self):
        self.commands = []
        self.suspended = False

    def execute(self, command: MotionCommand) -> MotionResult:
        for value in _velocity(command).values():
            if not math.isfinite(value):
                return MotionResult(False, "E_INVALID_NUMBER", "motion values must be finite")
        if self.suspended and command.action not in {"stop", "emergency_stop"}:
            return MotionResult(False, "E_BACKEND_SUSPENDED", "backend is suspended")
        self.commands.append(command)
        applied = _velocity(command)
        return MotionResult(True, "OK", requested=applied, applied=applied,
                            backend={"simulation": True})

    def stop(self, *, emergency: bool = False) -> MotionResult:
        command = MotionCommand(action="emergency_stop" if emergency else "stop")
        self.commands.append(command)
        return MotionResult(True, "OK", applied=_velocity(command),
                            backend={"simulation": True})

    def suspend(self) -> None:
        self.suspended = True

    def resume(self) -> None:
        self.suspended = False

    def state(self) -> Dict[str, Any]:
        return {"simulation": True, "suspended": self.suspended,
                "command_count": len(self.commands)}


def _velocity(command: MotionCommand) -> Dict[str, float]:
    return {
        "linear_x": float(command.linear_vel_x),
        "linear_y": float(command.linear_vel_y),
        "linear_z": float(command.linear_vel_z),
        "angular_z": float(command.angular_vel_z),
    }


class MotionController:
    SAFETY_ACTIONS = {"stop", "emergency_stop"}

    def __init__(self, backend: MotionBackend, *, linear_limit: float = 1.0,
                 angular_limit: float = 1.0, failsafe_timeout_s: float = 5.0,
                 recover_delay_s: float = 2.0):
        self.backend = backend
        self.linear_limit = max(0.0, float(linear_limit))
        self.angular_limit = max(0.0, float(angular_limit))
        self.failsafe_timeout_s = max(0.0, float(failsafe_timeout_s))
        self.recover_delay_s = max(0.0, float(recover_delay_s))
        self.control_mode = "manual"
        self.lock: Dict[str, Any] = {"active": False, "owner": ""}
        self.failsafe_active = False
        self._link_lost_since: Optional[float] = None
        self._link_recovered_since: Optional[float] = None
        self.last_received_seq = 0
        self.last_executed_seq = 0
        self.last_rejected_seq = 0

    def execute(self, command: MotionCommand) -> MotionResult:
        self.last_received_seq = int(command.command_seq)
        safety = command.action.strip().lower() in self.SAFETY_ACTIONS
        requested = _velocity(command)

        # Safety commands pass all normal gates and reach the backend immediately.
        if safety:
            result = self.backend.stop(emergency=command.action == "emergency_stop")
            result.requested = requested
            result.applied = {key: 0.0 for key in requested}
            return self._record(command, result)

        rejection = self._gate(command)
        if rejection:
            return self._record(command, MotionResult(False, rejection,
                                                       requested=requested))
        command = self._clamp(command)
        result = self.backend.execute(command)
        result.requested = requested
        if not result.applied:
            result.applied = _velocity(command)
        return self._record(command, result)

    def _gate(self, command: MotionCommand) -> str:
        if self.failsafe_active:
            return "E_FAILSAFE_ACTIVE"
        if self.lock.get("active"):
            owner = str(self.lock.get("owner") or "")
            if not (owner and command.controller == "auto" and command.source == owner):
                return "E_CONTROL_LOCKED"
        if self.control_mode == "manual" and command.controller == "auto":
            return "E_CONTROL_MODE"
        if self.control_mode == "auto" and command.controller in {"cloud", "field"}:
            return "E_CONTROL_MODE"
        return ""

    def _clamp(self, command: MotionCommand) -> MotionCommand:
        vx, vy, vz = (float(command.linear_vel_x), float(command.linear_vel_y),
                      float(command.linear_vel_z))
        norm = math.sqrt(vx * vx + vy * vy + vz * vz)
        if norm > self.linear_limit and norm > 0:
            scale = self.linear_limit / norm
            vx, vy, vz = vx * scale, vy * scale, vz * scale
        wz = max(-self.angular_limit,
                 min(self.angular_limit, float(command.angular_vel_z)))
        return replace(command, linear_vel_x=vx, linear_vel_y=vy,
                       linear_vel_z=vz, angular_vel_z=wz)

    def _record(self, command: MotionCommand, result: MotionResult) -> MotionResult:
        if result.accepted:
            self.last_executed_seq = int(command.command_seq)
        else:
            self.last_rejected_seq = int(command.command_seq)
        result.backend.update({
            "received_command_seq": self.last_received_seq,
            "executed_command_seq": self.last_executed_seq,
            "rejected_command_seq": self.last_rejected_seq,
        })
        return result

    def set_lock(self, lock: Dict[str, Any]) -> None:
        was_active = bool(self.lock.get("active"))
        self.lock = dict(lock or {})
        active = bool(self.lock.get("active"))
        if active and not was_active:
            self.backend.stop()
            self.backend.suspend()
        elif was_active and not active:
            self.backend.resume()

    def heartbeat(self, ok: bool, *, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else now
        if ok:
            if self._link_lost_since is not None and self._link_recovered_since is None:
                self._link_recovered_since = now
            self._link_lost_since = None
        else:
            if self._link_lost_since is None:
                self._link_lost_since = now
            self._link_recovered_since = None

    def tick(self, *, now: Optional[float] = None) -> Optional[MotionResult]:
        now = time.monotonic() if now is None else now
        if (not self.failsafe_active and self._link_lost_since is not None and
                now - self._link_lost_since >= self.failsafe_timeout_s):
            self.failsafe_active = True
            return self.backend.stop()
        if (self.failsafe_active and self._link_recovered_since is not None and
                now - self._link_recovered_since >= self.recover_delay_s):
            self.failsafe_active = False
            self._link_recovered_since = None
        return None
