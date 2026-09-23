import math
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any, Deque, Dict, Optional, Tuple


@dataclass
class AlignmentSample:
    timestamp_ns: int
    target_x: float
    target_y: float
    gps_map_x: float
    gps_map_y: float
    map_pose_x: Optional[float]
    map_pose_y: Optional[float]
    visual_pose_x: Optional[float]
    visual_pose_y: Optional[float]
    offset_x: float
    offset_y: float
    gps_error_m: float
    target_error_m: float
    gps_delta_from_prev_m: Optional[float]
    visual_delta_from_prev_m: Optional[float]
    gps_fix_type: int
    gps_num_sv: int
    gps_weight: float
    accepted_for_alignment: bool
    reject_reason: str

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class SlidingOffsetWindow:
    def __init__(self, *, max_samples: int = 8):
        self._samples: Deque[AlignmentSample] = deque(maxlen=max(1, int(max_samples)))

    def clear(self):
        self._samples.clear()

    def add(self, sample: AlignmentSample):
        if sample.accepted_for_alignment and sample.gps_weight > 0.0:
            self._samples.append(sample)

    def size(self) -> int:
        return len(self._samples)

    def estimate(self) -> Dict[str, Any]:
        if not self._samples:
            return {
                'available': False,
                'sample_count': 0,
                'offset_x': 0.0,
                'offset_y': 0.0,
                'offset_norm_m': 0.0,
                'std_m': 0.0,
            }

        total_weight = sum(max(0.0, sample.gps_weight) for sample in self._samples)
        if total_weight <= 1e-9:
            return {
                'available': False,
                'sample_count': len(self._samples),
                'offset_x': 0.0,
                'offset_y': 0.0,
                'offset_norm_m': 0.0,
                'std_m': 0.0,
            }

        offset_x = sum(sample.offset_x * sample.gps_weight for sample in self._samples) / total_weight
        offset_y = sum(sample.offset_y * sample.gps_weight for sample in self._samples) / total_weight
        variance = (
            sum(
                sample.gps_weight *
                ((sample.offset_x - offset_x) ** 2 + (sample.offset_y - offset_y) ** 2)
                for sample in self._samples
            ) /
            total_weight
        )
        return {
            'available': True,
            'sample_count': len(self._samples),
            'offset_x': offset_x,
            'offset_y': offset_y,
            'offset_norm_m': math.hypot(offset_x, offset_y),
            'std_m': math.sqrt(max(0.0, variance)),
            'latest_timestamp_ns': int(self._samples[-1].timestamp_ns),
        }

    def samples_snapshot(self, *, limit: int = 8) -> list:
        if limit <= 0:
            return []
        return [sample.to_dict() for sample in list(self._samples)[-limit:]]


class OutdoorPoseAligner:
    """Estimate the stable offset between projected GPS and the Nav2 map frame.

    This first implementation is intentionally simple: it is a weighted sliding
    window over GPS-to-map offsets. The class boundary is the important part.
    Later we can replace the window internals with a Kalman filter or a factor
    graph without changing MissionManager's call pattern.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        config = dict(config or {})
        self._enabled = _as_bool(config.get('enabled'), True)
        self._window = SlidingOffsetWindow(
            max_samples=int(_as_float(config.get('window_size'), 8.0))
        )
        self._min_fix_type = int(_as_float(config.get('min_fix_type'), 2.0))
        self._min_num_sv = int(_as_float(config.get('min_num_sv'), 0.0))
        self._jump_reject_m = _as_float(config.get('jump_reject_m'), 15.0)
        self._visual_support_ratio = _as_float(config.get('visual_support_ratio'), 0.5)
        self._max_stable_std_m = _as_float(config.get('max_stable_std_m'), 5.0)
        self._min_stable_samples = int(_as_float(config.get('min_stable_samples'), 2.0))
        self._enable_goal_correction = _as_bool(config.get('enable_goal_correction'), False)
        self._max_goal_correction_m = _as_float(config.get('max_goal_correction_m'), 10.0)
        self._last_sample: Optional[AlignmentSample] = None
        self._last_result: Dict[str, Any] = {}

    def reset(self):
        self._window.clear()
        self._last_sample = None
        self._last_result = {}

    def config_snapshot(self) -> Dict[str, Any]:
        return {
            'enabled': self._enabled,
            'window_size': self._window._samples.maxlen,
            'min_fix_type': self._min_fix_type,
            'min_num_sv': self._min_num_sv,
            'jump_reject_m': self._jump_reject_m,
            'visual_support_ratio': self._visual_support_ratio,
            'max_stable_std_m': self._max_stable_std_m,
            'min_stable_samples': self._min_stable_samples,
            'enable_goal_correction': self._enable_goal_correction,
            'max_goal_correction_m': self._max_goal_correction_m,
        }

    def state_snapshot(self) -> Dict[str, Any]:
        estimate = self._window.estimate()
        stable = (
            bool(estimate.get('available')) and
            int(estimate.get('sample_count', 0)) >= self._min_stable_samples and
            float(estimate.get('std_m', 0.0)) <= self._max_stable_std_m
        )
        return {
            'enabled': self._enabled,
            'stable': stable,
            'estimate': estimate,
            'last_result': dict(self._last_result),
            'recent_samples': self._window.samples_snapshot(limit=8),
            'config': self.config_snapshot(),
        }

    def evaluate(
        self,
        *,
        projector,
        target_lat: float,
        target_lon: float,
        target_x: float,
        target_y: float,
        gps_position: Dict[str, Any],
        map_pose: Optional[Dict[str, Any]],
        visual_pose: Optional[Dict[str, Any]],
        gps_fix_type: int,
        gps_num_sv: int,
        timestamp_ns: int,
    ) -> Dict[str, Any]:
        if not self._enabled:
            result = {
                'status': 'disabled',
                'message': 'outdoor pose aligner is disabled',
                'stable': False,
            }
            self._last_result = dict(result)
            return result

        if not projector.is_ready():
            result = {
                'status': 'unavailable',
                'message': 'geo reference is not enabled',
                'stable': False,
            }
            self._last_result = dict(result)
            return result

        if not gps_position or 'lat' not in gps_position or 'lon' not in gps_position:
            result = {
                'status': 'pending',
                'message': 'gps position is unavailable',
                'stable': False,
            }
            self._last_result = dict(result)
            return result

        gps_map_x, gps_map_y = projector.latlon_to_map_xy(
            float(gps_position['lat']),
            float(gps_position['lon']),
        )
        gps_error_m = _latlon_distance_m(
            float(target_lat),
            float(target_lon),
            float(gps_position['lat']),
            float(gps_position['lon']),
        )
        target_error_m = math.hypot(gps_map_x - float(target_x), gps_map_y - float(target_y))

        if map_pose is not None:
            offset_x = gps_map_x - float(map_pose['x'])
            offset_y = gps_map_y - float(map_pose['y'])
            map_pose_x = float(map_pose['x'])
            map_pose_y = float(map_pose['y'])
        else:
            offset_x = gps_map_x - float(target_x)
            offset_y = gps_map_y - float(target_y)
            map_pose_x = None
            map_pose_y = None

        gps_delta_from_prev_m = None
        visual_delta_from_prev_m = None
        if self._last_sample is not None:
            gps_delta_from_prev_m = math.hypot(
                gps_map_x - self._last_sample.gps_map_x,
                gps_map_y - self._last_sample.gps_map_y,
            )
            if (
                visual_pose is not None and
                self._last_sample.visual_pose_x is not None and
                self._last_sample.visual_pose_y is not None
            ):
                visual_delta_from_prev_m = math.hypot(
                    float(visual_pose['x']) - self._last_sample.visual_pose_x,
                    float(visual_pose['y']) - self._last_sample.visual_pose_y,
                )

        accepted, reject_reason = self._accept_sample(
            gps_fix_type=gps_fix_type,
            gps_num_sv=gps_num_sv,
            gps_delta_from_prev_m=gps_delta_from_prev_m,
            visual_delta_from_prev_m=visual_delta_from_prev_m,
        )
        gps_weight = self._gps_weight(
            gps_fix_type=gps_fix_type,
            gps_num_sv=gps_num_sv,
            accepted=accepted,
        )

        sample = AlignmentSample(
            timestamp_ns=int(timestamp_ns),
            target_x=float(target_x),
            target_y=float(target_y),
            gps_map_x=float(gps_map_x),
            gps_map_y=float(gps_map_y),
            map_pose_x=map_pose_x,
            map_pose_y=map_pose_y,
            visual_pose_x=float(visual_pose['x']) if visual_pose is not None else None,
            visual_pose_y=float(visual_pose['y']) if visual_pose is not None else None,
            offset_x=float(offset_x),
            offset_y=float(offset_y),
            gps_error_m=float(gps_error_m),
            target_error_m=float(target_error_m),
            gps_delta_from_prev_m=gps_delta_from_prev_m,
            visual_delta_from_prev_m=visual_delta_from_prev_m,
            gps_fix_type=int(gps_fix_type),
            gps_num_sv=int(gps_num_sv),
            gps_weight=float(gps_weight),
            accepted_for_alignment=accepted,
            reject_reason=reject_reason,
        )
        self._last_sample = sample
        self._window.add(sample)

        estimate = self._window.estimate()
        stable = (
            bool(estimate.get('available')) and
            int(estimate.get('sample_count', 0)) >= self._min_stable_samples and
            float(estimate.get('std_m', 0.0)) <= self._max_stable_std_m
        )
        correction = self._goal_correction(estimate=estimate, stable=stable)
        result = {
            'status': 'aligned' if stable else 'observing',
            'message': 'alignment sample accepted' if accepted else reject_reason,
            'stable': stable,
            'sample': sample.to_dict(),
            'estimate': estimate,
            'goal_correction': correction,
        }
        self._last_result = dict(result)
        return result

    def _accept_sample(
        self,
        *,
        gps_fix_type: int,
        gps_num_sv: int,
        gps_delta_from_prev_m: Optional[float],
        visual_delta_from_prev_m: Optional[float],
    ) -> Tuple[bool, str]:
        if int(gps_fix_type) < self._min_fix_type:
            return False, 'gps fix type is lower than required'
        if self._min_num_sv > 0 and int(gps_num_sv) < self._min_num_sv:
            return False, 'gps satellite count is lower than required'

        gps_jump = (
            gps_delta_from_prev_m is not None and
            float(gps_delta_from_prev_m) > self._jump_reject_m
        )
        visual_supports_jump = (
            visual_delta_from_prev_m is not None and
            float(visual_delta_from_prev_m) >= self._jump_reject_m * self._visual_support_ratio
        )
        if gps_jump and not visual_supports_jump:
            return False, 'gps jumped but local odometry did not support the same motion'

        return True, ''

    def _gps_weight(self, *, gps_fix_type: int, gps_num_sv: int, accepted: bool) -> float:
        if not accepted:
            return 0.0
        fix_bonus = max(0, int(gps_fix_type) - self._min_fix_type)
        sv_weight = min(1.0, max(0.2, float(gps_num_sv) / 12.0)) if gps_num_sv > 0 else 0.5
        return min(1.0, 0.6 + 0.2 * fix_bonus) * sv_weight

    def _goal_correction(self, *, estimate: Dict[str, Any], stable: bool) -> Dict[str, Any]:
        if not self._enable_goal_correction:
            return {
                'enabled': False,
                'applied': False,
                'dx': 0.0,
                'dy': 0.0,
                'reason': 'goal correction is disabled',
            }
        if not stable:
            return {
                'enabled': True,
                'applied': False,
                'dx': 0.0,
                'dy': 0.0,
                'reason': 'alignment estimate is not stable',
            }

        offset_x = float(estimate.get('offset_x', 0.0))
        offset_y = float(estimate.get('offset_y', 0.0))
        norm = math.hypot(offset_x, offset_y)
        if norm <= 1e-6:
            dx = dy = 0.0
        elif norm > self._max_goal_correction_m:
            scale = self._max_goal_correction_m / norm
            dx = -offset_x * scale
            dy = -offset_y * scale
        else:
            dx = -offset_x
            dy = -offset_y
        return {
            'enabled': True,
            'applied': True,
            'dx': dx,
            'dy': dy,
            'reason': 'stable gps/map offset correction',
        }


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y', 'on'}
    return bool(value)


def _as_float(value: Any, default: float) -> float:
    if value in (None, ''):
        return default
    return float(value)


def _latlon_distance_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    lat_a_rad = math.radians(lat_a)
    lat_b_rad = math.radians(lat_b)
    dlat = lat_b_rad - lat_a_rad
    dlon = math.radians(lon_b - lon_a)
    a = (
        math.sin(dlat / 2.0) ** 2 +
        math.cos(lat_a_rad) * math.cos(lat_b_rad) * math.sin(dlon / 2.0) ** 2
    )
    return 2.0 * 6378137.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))
