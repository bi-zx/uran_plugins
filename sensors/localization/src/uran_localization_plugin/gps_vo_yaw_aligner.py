import math
from collections import deque
from typing import Any, Deque, Dict, Optional, Tuple

from .geo import EARTH_RADIUS_M


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


def _normalize_degrees(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def _angle_lerp_degrees(old: float, new: float, alpha: float) -> float:
    delta = _normalize_degrees(new - old)
    return _normalize_degrees(old + float(alpha) * delta)


class GpsVoYawAligner:
    """Estimate global yaw offset from GPS displacement and local odometry displacement.

    The estimated offset maps local odometry yaw into the GPS projected world frame:
    global_yaw = visual_yaw + yaw_offset.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        cfg = dict(config or {})
        self.enabled = bool(cfg.get('enabled', True))
        self.min_fix_type = _as_int(cfg.get('min_fix_type'), 3)
        self.min_num_sv = _as_int(cfg.get('min_num_sv'), 4)
        self.min_gps_displacement_m = max(0.1, _as_float(cfg.get('min_gps_displacement_m'), 3.0))
        self.min_visual_displacement_m = max(
            0.1,
            _as_float(cfg.get('min_visual_displacement_m'), 2.0),
        )
        minimum_drive_distance = max(
            self.min_gps_displacement_m,
            self.min_visual_displacement_m,
        ) + 0.5
        self.calibration_drive_distance_m = max(
            minimum_drive_distance,
            _as_float(cfg.get('calibration_drive_distance_m'), 4.0),
        )
        self.max_length_ratio_error = max(0.0, _as_float(cfg.get('max_length_ratio_error'), 0.6))
        self.max_sample_yaw_delta_deg = max(
            0.0,
            _as_float(cfg.get('max_sample_yaw_delta_deg'), 30.0),
        )
        self.smoothing_alpha = min(1.0, max(0.0, _as_float(cfg.get('smoothing_alpha'), 0.1)))
        self.gps_jump_reject_m = max(0.0, _as_float(cfg.get('gps_jump_reject_m'), 10.0))
        self.max_sample_age_s = max(0.0, _as_float(cfg.get('max_sample_age_s'), 20.0))
        self.max_history = max(2, _as_int(cfg.get('max_history'), 80))

        self._origin_lat: Optional[float] = None
        self._origin_lon: Optional[float] = None
        self._samples: Deque[Dict[str, Any]] = deque(maxlen=self.max_history)
        self._yaw_offset_deg: Optional[float] = None
        self._accepted_samples = 0
        self._rejected_samples = 0
        self._last_result: Dict[str, Any] = {
            'status': 'disabled' if not self.enabled else 'waiting',
            'message': 'waiting for gps and local odometry samples',
        }

    def reset(self):
        self._origin_lat = None
        self._origin_lon = None
        self._samples.clear()
        self._yaw_offset_deg = None
        self._accepted_samples = 0
        self._rejected_samples = 0
        self._last_result = {
            'status': 'disabled' if not self.enabled else 'waiting',
            'message': 'waiting for gps and local odometry samples',
        }

    def set_initial_offset(self, value: Optional[float], *, reason: str = ''):
        if value in (None, ''):
            return
        self._yaw_offset_deg = _normalize_degrees(float(value))
        self._last_result = {
            'status': 'initialized',
            'accepted': True,
            'yaw_offset_deg': self._yaw_offset_deg,
            'message': reason or 'initial yaw offset set',
        }

    @property
    def yaw_offset_deg(self) -> Optional[float]:
        return self._yaw_offset_deg

    def update(
        self,
        *,
        gps_position: Optional[Dict[str, Any]],
        visual_pose: Optional[Dict[str, Any]],
        timestamp_ns: int,
    ) -> Dict[str, Any]:
        if not self.enabled:
            self._last_result = {
                'status': 'disabled',
                'accepted': False,
                'message': 'gps/visual yaw alignment is disabled',
            }
            return dict(self._last_result)

        valid, reason = self._quality_ok(gps_position=gps_position, visual_pose=visual_pose)
        if not valid:
            self._last_result = {
                'status': 'waiting',
                'accepted': False,
                'message': reason,
                'yaw_offset_deg': self._yaw_offset_deg,
            }
            return dict(self._last_result)

        gps_xy = self._project_gps(gps_position)
        visual_xy = self._visual_xy(visual_pose)
        if gps_xy is None or visual_xy is None:
            self._last_result = {
                'status': 'waiting',
                'accepted': False,
                'message': 'failed to project gps or visual pose',
                'yaw_offset_deg': self._yaw_offset_deg,
            }
            return dict(self._last_result)

        sample = {
            'timestamp_ns': int(timestamp_ns),
            'gps_x': gps_xy[0],
            'gps_y': gps_xy[1],
            'visual_x': visual_xy[0],
            'visual_y': visual_xy[1],
            'fix_type': _as_int((gps_position or {}).get('fix_type')),
            'num_sv': _as_int((gps_position or {}).get('num_sv')),
        }

        if self._samples:
            prev = self._samples[-1]
            last_gps_step = math.hypot(sample['gps_x'] - prev['gps_x'], sample['gps_y'] - prev['gps_y'])
            if self.gps_jump_reject_m > 0.0 and last_gps_step > self.gps_jump_reject_m:
                self._rejected_samples += 1
                self._last_result = {
                    'status': 'rejected',
                    'accepted': False,
                    'message': (
                        f'gps jumped {last_gps_step:.2f}m, exceeds '
                        f'{self.gps_jump_reject_m:.2f}m'
                    ),
                    'yaw_offset_deg': self._yaw_offset_deg,
                    'gps_step_m': last_gps_step,
                }
                return dict(self._last_result)

        self._samples.append(sample)
        result = self._evaluate_window(sample)
        self._last_result = dict(result)
        return dict(result)

    def state_snapshot(self) -> Dict[str, Any]:
        return {
            'enabled': self.enabled,
            'status': self._last_result.get('status', 'waiting'),
            'yaw_offset_deg': self._yaw_offset_deg,
            'aligned': self._yaw_offset_deg is not None,
            'accepted_samples': self._accepted_samples,
            'rejected_samples': self._rejected_samples,
            'sample_count': len(self._samples),
            'config': self.config_snapshot(),
            'last_result': dict(self._last_result),
        }

    def config_snapshot(self) -> Dict[str, Any]:
        return {
            'min_fix_type': self.min_fix_type,
            'min_num_sv': self.min_num_sv,
            'min_gps_displacement_m': self.min_gps_displacement_m,
            'min_visual_displacement_m': self.min_visual_displacement_m,
            'calibration_drive_distance_m': self.calibration_drive_distance_m,
            'max_length_ratio_error': self.max_length_ratio_error,
            'max_sample_yaw_delta_deg': self.max_sample_yaw_delta_deg,
            'smoothing_alpha': self.smoothing_alpha,
            'gps_jump_reject_m': self.gps_jump_reject_m,
            'max_sample_age_s': self.max_sample_age_s,
            'max_history': self.max_history,
        }

    def _quality_ok(
        self,
        *,
        gps_position: Optional[Dict[str, Any]],
        visual_pose: Optional[Dict[str, Any]],
    ) -> Tuple[bool, str]:
        if not gps_position or 'lat' not in gps_position or 'lon' not in gps_position:
            return False, 'gps position is unavailable'
        if _as_int(gps_position.get('fix_type')) < self.min_fix_type:
            return False, 'gps fix_type is below yaw alignment threshold'
        if _as_int(gps_position.get('num_sv')) < self.min_num_sv:
            return False, 'gps satellite count is below yaw alignment threshold'
        if not visual_pose or 'x' not in visual_pose or 'y' not in visual_pose:
            return False, 'local odometry pose is unavailable'
        return True, ''

    def _project_gps(self, gps_position: Dict[str, Any]) -> Optional[Tuple[float, float]]:
        try:
            lat = float(gps_position['lat'])
            lon = float(gps_position['lon'])
        except Exception:
            return None
        if self._origin_lat is None or self._origin_lon is None:
            self._origin_lat = lat
            self._origin_lon = lon
        ref_lat_rad = math.radians(float(self._origin_lat))
        x = math.radians(lon - float(self._origin_lon)) * EARTH_RADIUS_M * math.cos(ref_lat_rad)
        y = math.radians(lat - float(self._origin_lat)) * EARTH_RADIUS_M
        return float(x), float(y)

    def _visual_xy(self, visual_pose: Dict[str, Any]) -> Optional[Tuple[float, float]]:
        try:
            return float(visual_pose['x']), float(visual_pose['y'])
        except Exception:
            return None

    def _evaluate_window(self, latest: Dict[str, Any]) -> Dict[str, Any]:
        if len(self._samples) < 2:
            return {
                'status': 'collecting',
                'accepted': False,
                'message': 'need at least two samples',
                'yaw_offset_deg': self._yaw_offset_deg,
            }

        start = self._oldest_usable_start(latest)
        if start is None:
            return {
                'status': 'collecting',
                'accepted': False,
                'message': 'waiting for enough gps/visual displacement',
                'yaw_offset_deg': self._yaw_offset_deg,
            }

        gps_dx = float(latest['gps_x']) - float(start['gps_x'])
        gps_dy = float(latest['gps_y']) - float(start['gps_y'])
        visual_dx = float(latest['visual_x']) - float(start['visual_x'])
        visual_dy = float(latest['visual_y']) - float(start['visual_y'])
        gps_dist = math.hypot(gps_dx, gps_dy)
        visual_dist = math.hypot(visual_dx, visual_dy)

        if gps_dist < self.min_gps_displacement_m:
            return self._collecting_distance_result(gps_dist, visual_dist)
        if visual_dist < self.min_visual_displacement_m:
            return self._collecting_distance_result(gps_dist, visual_dist)

        length_ratio_error = abs(gps_dist - visual_dist) / max(gps_dist, visual_dist, 1e-6)
        if length_ratio_error > self.max_length_ratio_error:
            self._rejected_samples += 1
            return {
                'status': 'rejected',
                'accepted': False,
                'message': (
                    f'gps/visual displacement length mismatch {length_ratio_error:.2f}, '
                    f'exceeds {self.max_length_ratio_error:.2f}'
                ),
                'yaw_offset_deg': self._yaw_offset_deg,
                'gps_displacement_m': gps_dist,
                'visual_displacement_m': visual_dist,
                'length_ratio_error': length_ratio_error,
            }

        gps_heading_deg = math.degrees(math.atan2(gps_dy, gps_dx))
        visual_heading_deg = math.degrees(math.atan2(visual_dy, visual_dx))
        sample_offset = _normalize_degrees(gps_heading_deg - visual_heading_deg)

        if self._yaw_offset_deg is None or self._accepted_samples <= 0:
            new_offset = sample_offset
            sample_delta = 0.0
        else:
            sample_delta = abs(_normalize_degrees(sample_offset - self._yaw_offset_deg))
            if sample_delta > self.max_sample_yaw_delta_deg:
                self._rejected_samples += 1
                return {
                    'status': 'rejected',
                    'accepted': False,
                    'message': (
                        f'yaw offset sample changed {sample_delta:.1f}deg, exceeds '
                        f'{self.max_sample_yaw_delta_deg:.1f}deg'
                    ),
                    'yaw_offset_deg': self._yaw_offset_deg,
                    'sample_yaw_offset_deg': sample_offset,
                    'sample_delta_deg': sample_delta,
                    'gps_displacement_m': gps_dist,
                    'visual_displacement_m': visual_dist,
                }
            new_offset = _angle_lerp_degrees(
                self._yaw_offset_deg,
                sample_offset,
                self.smoothing_alpha,
            )

        self._yaw_offset_deg = new_offset
        self._accepted_samples += 1
        return {
            'status': 'aligned',
            'accepted': True,
            'message': 'yaw offset updated from gps and local odometry displacement',
            'yaw_offset_deg': self._yaw_offset_deg,
            'sample_yaw_offset_deg': sample_offset,
            'sample_delta_deg': sample_delta,
            'gps_heading_deg': gps_heading_deg,
            'visual_heading_deg': visual_heading_deg,
            'gps_displacement_m': gps_dist,
            'visual_displacement_m': visual_dist,
            'length_ratio_error': length_ratio_error,
        }

    def _oldest_usable_start(self, latest: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        newest_ns = int(latest.get('timestamp_ns', 0) or 0)
        fallback = None
        for sample in self._samples:
            if sample is latest:
                continue
            if self.max_sample_age_s > 0.0 and newest_ns:
                sample_ns = int(sample.get('timestamp_ns', 0) or 0)
                if sample_ns and float(newest_ns - sample_ns) / 1_000_000_000.0 > self.max_sample_age_s:
                    continue
            gps_dist = math.hypot(
                float(latest['gps_x']) - float(sample['gps_x']),
                float(latest['gps_y']) - float(sample['gps_y']),
            )
            visual_dist = math.hypot(
                float(latest['visual_x']) - float(sample['visual_x']),
                float(latest['visual_y']) - float(sample['visual_y']),
            )
            fallback = sample
            if gps_dist >= self.min_gps_displacement_m and visual_dist >= self.min_visual_displacement_m:
                return sample
        return fallback

    def _collecting_distance_result(self, gps_dist: float, visual_dist: float) -> Dict[str, Any]:
        return {
            'status': 'collecting',
            'accepted': False,
            'message': 'waiting for enough gps/visual displacement',
            'yaw_offset_deg': self._yaw_offset_deg,
            'gps_displacement_m': gps_dist,
            'visual_displacement_m': visual_dist,
            'required_gps_displacement_m': self.min_gps_displacement_m,
            'required_visual_displacement_m': self.min_visual_displacement_m,
        }
