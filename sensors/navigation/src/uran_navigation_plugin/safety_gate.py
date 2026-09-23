import math
from typing import Any, Dict, Optional


class MotionSafetyGate:
    """Central decision point for whether local control may move the robot."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        cfg = dict(config or {})
        self.require_auto_mode = bool(cfg.get('require_auto_mode', True))
        self.pose_timeout_s = max(0.0, float(cfg.get('control_pose_timeout_s', 0.30)))
        self.scan_timeout_s = max(0.0, float(cfg.get('scan_timeout_s', 0.35)))
        self.min_scan_valid_ratio = _as_ratio(cfg.get('min_scan_valid_ratio'), 0.50)
        self.min_front_valid_ratio = _as_ratio(cfg.get('min_front_valid_ratio'), 0.50)
        self.degraded_speed_cap_mps = max(
            0.0,
            float(cfg.get('degraded_sensor_speed_cap_mps', 0.15)),
        )
        self.warning_battery_percent = _optional_float(
            cfg.get('warning_battery_percent'),
        )
        self.critical_battery_percent = _optional_float(
            cfg.get('critical_battery_percent'),
        )

    def check(
        self,
        *,
        control_mode: str = 'auto',
        controller: str = 'auto',
        pose_available: bool = True,
        pose_age_s: Optional[float] = None,
        scan_available: bool = True,
        scan_age_s: Optional[float] = None,
        scan_quality: str = 'good',
        scan_valid_ratio: Optional[float] = None,
        front_valid_ratio: Optional[float] = None,
        bottom_status: Any = None,
        battery_level: Optional[float] = None,
        failsafe_active: bool = False,
    ) -> Dict[str, Any]:
        if bool(failsafe_active):
            return self._blocked(
                'E_FAILSAFE_ACTIVE',
                'uran_move failsafe is active',
                hard_stop=True,
            )

        mode = str(control_mode or '').strip().lower()
        source = str(controller or '').strip().lower()
        if self.require_auto_mode and mode != 'auto':
            return self._blocked(
                'E_CONTROL_MODE',
                'control_mode is not auto',
                hard_stop=True,
            )
        if self.require_auto_mode and source != 'auto':
            return self._blocked(
                'E_CONTROLLER_MODE',
                'controller is not auto',
                hard_stop=True,
            )

        if not pose_available:
            return self._blocked(
                'E_POSE_UNAVAILABLE',
                'control pose is unavailable',
                hard_stop=True,
            )
        if pose_age_s is not None and self.pose_timeout_s > 0.0:
            if float(pose_age_s) > self.pose_timeout_s:
                return self._blocked(
                    'E_POSE_STALE',
                    'control pose is stale',
                    hard_stop=True,
                )

        if not scan_available:
            return self._blocked(
                'E_SCAN_UNAVAILABLE',
                'laser scan is unavailable',
                hard_stop=True,
            )
        if scan_age_s is not None and self.scan_timeout_s > 0.0:
            if float(scan_age_s) > self.scan_timeout_s:
                return self._blocked(
                    'E_SCAN_STALE',
                    'laser scan is stale',
                    hard_stop=True,
                )

        quality = str(scan_quality or 'good').lower()
        valid = _ratio_or_none(scan_valid_ratio)
        front_valid = _ratio_or_none(front_valid_ratio)
        if quality in {'invalid', 'fault'}:
            return self._blocked(
                'E_SCAN_QUALITY',
                'laser scan quality is invalid',
                hard_stop=True,
            )
        if valid is not None and valid < self.min_scan_valid_ratio:
            return self._blocked(
                'E_SCAN_QUALITY',
                'overall laser scan valid ratio is too low',
                hard_stop=True,
            )
        if front_valid is not None and front_valid < self.min_front_valid_ratio:
            return self._blocked(
                'E_SCAN_FRONT_QUALITY',
                'front laser scan valid ratio is too low',
                hard_stop=True,
            )

        bottom = _normalize_bottom_status(bottom_status)
        if bottom not in {'', 'NORMAL', '0'}:
            return self._blocked(
                'E_BOTTOM_STATUS',
                'CyberDog motion switch_status is not NORMAL',
                hard_stop=True,
            )

        speed_cap = None
        if quality in {'degraded', 'warning'}:
            speed_cap = self.degraded_speed_cap_mps

        if battery_level is not None and _is_finite(battery_level):
            battery = float(battery_level)
            if (
                self.critical_battery_percent is not None and
                battery <= self.critical_battery_percent
            ):
                return self._blocked(
                    'E_LOW_BATTERY',
                    'battery is below critical motion threshold',
                    hard_stop=True,
                )
            if (
                self.warning_battery_percent is not None and
                battery <= self.warning_battery_percent
            ):
                speed_cap = (
                    self.degraded_speed_cap_mps
                    if speed_cap is None else
                    min(speed_cap, self.degraded_speed_cap_mps)
                )

        return {
            'allowed': True,
            'level': 'degraded' if speed_cap is not None else 'normal',
            'code': '',
            'reason': '',
            'hard_stop': False,
            'speed_cap_mps': speed_cap,
        }

    def _blocked(self, code: str, reason: str, *, hard_stop: bool) -> Dict[str, Any]:
        return {
            'allowed': False,
            'level': 'fault',
            'code': code,
            'reason': reason,
            'hard_stop': bool(hard_stop),
            'speed_cap_mps': 0.0,
        }


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _optional_float(value: Any) -> Optional[float]:
    if value in (None, ''):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _as_ratio(value: Any, default: float) -> float:
    number = _optional_float(value)
    if number is None:
        number = default
    return max(0.0, min(1.0, number))


def _ratio_or_none(value: Any) -> Optional[float]:
    number = _optional_float(value)
    if number is None:
        return None
    return max(0.0, min(1.0, number))


def _normalize_bottom_status(value: Any) -> str:
    if value is None:
        return ''
    if isinstance(value, bool):
        return 'NORMAL' if value else 'UNKNOWN'
    if isinstance(value, (int, float)):
        return str(int(value))
    text = str(value).strip().upper()
    if text.startswith('{'):
        # Keep this helper dependency-free; callers may pass the already
        # extracted switch status from uran_move instead of raw JSON.
        return text
    return text
