import math
from typing import Any, Dict, Optional, Sequence, Tuple

from .mission_contract import OutdoorMissionPlan


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {'1', 'true', 'yes', 'on'}:
        return True
    if text in {'0', 'false', 'no', 'off'}:
        return False
    return default


def _as_float(value: Any, default: float) -> float:
    if value in (None, ''):
        return float(default)
    return float(value)


class OutdoorStartAligner:
    """Gate an outdoor route before the first inspection target is dispatched."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        config = dict(config or {})
        self._enabled = _as_bool(config.get('enabled'), True)
        self._first_inspection_direct_distance_m = max(
            0.0,
            _as_float(config.get('first_inspection_direct_distance_m'), 5.0),
        )
        self._home_calibration_accept_distance_m = max(
            0.0,
            _as_float(config.get('home_calibration_accept_distance_m'), 5.0),
        )
        self._home_calibration_reject_distance_m = max(
            0.0,
            _as_float(config.get('home_calibration_reject_distance_m'), 10.0),
        )
        if self._home_calibration_reject_distance_m < self._home_calibration_accept_distance_m:
            self._home_calibration_reject_distance_m = self._home_calibration_accept_distance_m
        self._max_pose_age_s = max(
            0.0,
            _as_float(config.get('max_pose_age_s'), 3.0),
        )

    def config_snapshot(self) -> Dict[str, Any]:
        return {
            'enabled': self._enabled,
            'first_inspection_direct_distance_m': self._first_inspection_direct_distance_m,
            'home_calibration_accept_distance_m': self._home_calibration_accept_distance_m,
            'home_calibration_reject_distance_m': self._home_calibration_reject_distance_m,
            'max_pose_age_s': self._max_pose_age_s,
        }

    def evaluate_and_apply(
        self,
        *,
        mission: Optional[OutdoorMissionPlan],
        current_pose: Optional[Dict[str, Any]],
        now_ns: int = 0,
    ) -> Dict[str, Any]:
        if not self._enabled:
            return self._result(status='disabled', action='none', message='start alignment is disabled')

        if mission is None or not mission.execution_points:
            return self._result(
                status='passed',
                action='none',
                message='outdoor mission has no execution point to align',
            )

        if not current_pose or 'x' not in current_pose or 'y' not in current_pose:
            return self._result(
                status='unavailable',
                action='wait',
                message='current map pose is unavailable',
            )
        pose_timestamp_ns = int(current_pose.get('timestamp_ns', 0))
        pose_age_s = None
        if now_ns and pose_timestamp_ns:
            pose_age_s = max(0.0, float(int(now_ns) - pose_timestamp_ns) / 1_000_000_000.0)
            if self._max_pose_age_s > 0.0 and pose_age_s > self._max_pose_age_s:
                return self._result(
                    status='stale_pose',
                    action='wait',
                    message=(
                        f'current map pose is stale: age {pose_age_s:.2f}s exceeds '
                        f'{self._max_pose_age_s:.2f}s'
                    ),
                    pose_age_s=pose_age_s,
                    pose_timestamp_ns=pose_timestamp_ns,
                )

        current_x = float(current_pose['x'])
        current_y = float(current_pose['y'])
        first_index, first_point = self._first_inspection_point(mission.execution_points)
        if first_point is None:
            return self._result(
                status='failed',
                action='reject',
                message='outdoor mission has no inspection point',
            )

        home_index, home_point = self._home_point(mission.execution_points)
        first_distance_m = self._distance(current_x, current_y, first_point)
        base = {
            'current_pose': {
                'x': current_x,
                'y': current_y,
                'z': float(current_pose.get('z', 0.0)),
                'frame_id': str(current_pose.get('frame_id', '')),
                'timestamp_ns': pose_timestamp_ns,
                'age_s': pose_age_s,
            },
            'first_inspection_index': first_index,
            'first_inspection_point': first_point.to_dict(),
            'first_inspection_distance_m': first_distance_m,
            'home_index': home_index,
            'home_point': home_point.to_dict() if home_point is not None else None,
            'home_calibration_accept_distance_m': self._home_calibration_accept_distance_m,
            'home_calibration_reject_distance_m': self._home_calibration_reject_distance_m,
        }

        if first_distance_m <= self._first_inspection_direct_distance_m:
            return self._result(
                status='direct_first_inspection',
                action='go_first_inspection',
                next_current_waypoint_index=first_index - 1,
                message='current pose is close enough to the first inspection point',
                **base,
            )

        if home_point is None:
            return self._result(
                status='failed',
                action='reject',
                message='current pose is far from first inspection point and mission has no home point',
                **base,
            )

        home_distance_m = self._distance(current_x, current_y, home_point)
        base['home_distance_m'] = home_distance_m

        if home_distance_m <= self._home_calibration_accept_distance_m:
            return self._result(
                status='home_calibration_passed',
                action='go_first_inspection',
                next_current_waypoint_index=first_index - 1,
                message='home calibration is already within the accepted distance',
                **base,
            )

        if home_distance_m <= self._home_calibration_reject_distance_m:
            return self._result(
                status='home_calibration_required',
                action='run_home_calibration',
                home_calibration_index=home_index,
                message='current pose is far from first inspection point; run home calibration first',
                **base,
            )

        return self._result(
            status='failed',
            action='reject',
            message=(
                f'current pose is {first_distance_m:.2f}m from first inspection point and '
                f'{home_distance_m:.2f}m from home, exceeding reject distance '
                f'{self._home_calibration_reject_distance_m:.2f}m'
            ),
            **base,
        )

    def _first_inspection_point(self, points: Sequence) -> Tuple[int, Any]:
        for index, point in enumerate(points):
            if str(point.kind or '').strip().lower() == 'inspection':
                return index, point
        for index, point in enumerate(points):
            if str(point.kind or '').strip().lower() == 'calibration':
                return index, point
        return -1, None

    def _home_point(self, points: Sequence) -> Tuple[int, Any]:
        for index, point in enumerate(points):
            if str(point.kind or '').strip().lower() == 'home':
                return index, point
        return -1, None

    def _distance(self, current_x: float, current_y: float, point) -> float:
        return math.hypot(
            float(current_x) - float(point.x),
            float(current_y) - float(point.y),
        )

    def _result(self, *, status: str, action: str, message: str, **extra) -> Dict[str, Any]:
        result = {
            'enabled': self._enabled,
            'status': status,
            'action': action,
            'message': message,
        }
        result.update(extra)
        return result
