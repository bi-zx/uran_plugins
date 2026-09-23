from typing import Any, Dict, Optional, Tuple


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _step_towards(previous: float, target: float, maximum_delta: float) -> float:
    if target > previous:
        return min(target, previous + maximum_delta)
    return max(target, previous - maximum_delta)


class VelocityShaper:
    """Apply hard limits and rate limits to a planar velocity command."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        cfg = dict(config or {})
        self.max_speed_mps = max(0.0, float(cfg.get('max_speed_mps', 0.8)))
        self.max_angular_speed_radps = max(
            0.0,
            float(cfg.get('max_angular_speed_radps', 0.7)),
        )
        self.max_linear_accel_mps2 = max(
            0.0,
            float(cfg.get('max_linear_accel_mps2', 0.25)),
        )
        self.max_linear_decel_mps2 = max(
            0.0,
            float(cfg.get('max_linear_decel_mps2', 0.5)),
        )
        self.max_angular_accel_radps2 = max(
            0.0,
            float(cfg.get('max_angular_accel_radps2', 0.8)),
        )
        self.max_angular_decel_radps2 = max(
            0.0,
            float(cfg.get('max_angular_decel_radps2', 1.2)),
        )
        self._last_vx = 0.0
        self._last_wz = 0.0
        self._last_desired_vx = 0.0
        self._last_desired_wz = 0.0
        self._last_dt = 0.0

    def reset(self):
        self._last_vx = 0.0
        self._last_wz = 0.0
        self._last_desired_vx = 0.0
        self._last_desired_wz = 0.0
        self._last_dt = 0.0

    def shape(
        self,
        vx: float,
        wz: float,
        *,
        dt_s: float,
        hard_stop: bool = False,
        force_zero_linear: bool = False,
        speed_cap_mps: Optional[float] = None,
        angular_cap_radps: Optional[float] = None,
    ) -> Tuple[float, float]:
        dt = _clamp(float(dt_s), 0.001, 0.5)
        cap_v = self.max_speed_mps
        if speed_cap_mps is not None:
            cap_v = min(cap_v, max(0.0, float(speed_cap_mps)))
        cap_w = self.max_angular_speed_radps
        if angular_cap_radps is not None:
            cap_w = min(cap_w, max(0.0, float(angular_cap_radps)))

        desired_vx = _clamp(float(vx), 0.0, cap_v)
        desired_wz = _clamp(float(wz), -cap_w, cap_w)
        self._last_desired_vx = desired_vx
        self._last_desired_wz = desired_wz
        self._last_dt = dt

        if hard_stop:
            self._last_vx = 0.0
            self._last_wz = 0.0
            return 0.0, 0.0

        # A front obstacle may still leave a safe direction for turning.  In
        # that case the caller must be able to force the translational part to
        # zero without discarding the angular command.
        if force_zero_linear:
            self._last_vx = 0.0
        else:
            vx_rate = (
                self.max_linear_accel_mps2
                if desired_vx >= self._last_vx else
                self.max_linear_decel_mps2
            )
            self._last_vx = _step_towards(
                self._last_vx,
                desired_vx,
                vx_rate * dt,
            )

        wz_rate = (
            self.max_angular_accel_radps2
            if abs(desired_wz) >= abs(self._last_wz) else
            self.max_angular_decel_radps2
        )
        self._last_wz = _step_towards(
            self._last_wz,
            desired_wz,
            wz_rate * dt,
        )
        return self._last_vx, self._last_wz

    def snapshot(self) -> Dict[str, Any]:
        return {
            'last_vx': self._last_vx,
            'last_wz': self._last_wz,
            'desired_vx': self._last_desired_vx,
            'desired_wz': self._last_desired_wz,
            'last_dt_s': self._last_dt,
            'max_speed_mps': self.max_speed_mps,
            'max_angular_speed_radps': self.max_angular_speed_radps,
            'max_linear_accel_mps2': self.max_linear_accel_mps2,
            'max_linear_decel_mps2': self.max_linear_decel_mps2,
            'max_angular_accel_radps2': self.max_angular_accel_radps2,
            'max_angular_decel_radps2': self.max_angular_decel_radps2,
        }
