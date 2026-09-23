from typing import Any, Dict, Optional


class GpsSupervisor:
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self._config = dict(config or {})
        self._last_gps_check_ts = 0.0
        self._gps_violation_count = 0
        self._state: Dict[str, Any] = {}

    def enabled(self) -> bool:
        return bool(self._config.get('enabled', False))

    def reset(self):
        self._last_gps_check_ts = 0.0
        self._gps_violation_count = 0
        self._state = {}

    def state_snapshot(self) -> Dict[str, Any]:
        return dict(self._state)

    def evaluate(
        self,
        *,
        projector,
        pose_registry,
        now_ns: int,
        monotonic_s: float,
    ) -> Optional[Dict[str, Any]]:
        if not self.enabled():
            return None

        interval = float(self._config.get('check_interval_s', 3.0))
        if monotonic_s - self._last_gps_check_ts < interval:
            return None
        self._last_gps_check_ts = monotonic_s

        latest_map_pose = pose_registry.latest_map_pose()
        gps_position = pose_registry.gps_position()
        if latest_map_pose is None or not gps_position:
            return None
        if pose_registry.gps_fix_type() < int(self._config.get('min_fix_type', 2)):
            return None
        if not projector.is_ready():
            return None

        gps_map_x, gps_map_y = projector.latlon_to_map_xy(
            gps_position['lat'],
            gps_position['lon'],
        )
        dx = gps_map_x - float(latest_map_pose['x'])
        dy = gps_map_y - float(latest_map_pose['y'])
        error_m = (dx * dx + dy * dy) ** 0.5
        threshold = float(self._config.get('max_pose_error_m', 15.0))

        self._state = {
            'gps_fix_type': pose_registry.gps_fix_type(),
            'gps_num_sv': pose_registry.gps_num_sv(),
            'gps_map_x': gps_map_x,
            'gps_map_y': gps_map_y,
            'map_pose_x': float(latest_map_pose['x']),
            'map_pose_y': float(latest_map_pose['y']),
            'error_m': error_m,
            'threshold_m': threshold,
            'timestamp_ns': int(now_ns),
        }

        if error_m <= threshold:
            self._gps_violation_count = 0
            self._state['violation_count'] = 0
            return None

        self._gps_violation_count += 1
        self._state['violation_count'] = self._gps_violation_count
        required = int(self._config.get('violation_count_threshold', 2))
        if self._gps_violation_count < required:
            return None

        description = (
            f'gps/map pose mismatch too large: error={error_m:.2f}m, '
            f'threshold={threshold:.2f}m, fix_type={pose_registry.gps_fix_type()}, '
            f'num_sv={pose_registry.gps_num_sv()}'
        )
        action = str(self._config.get('on_large_error', 'pause')).strip().lower()
        return {
            'action': 'abort' if action == 'abort' else 'pause',
            'code': 'E_GPS_MAP_MISMATCH',
            'description': description,
            'suggested_action': 'relocalize_and_restart' if action == 'abort' else 'relocalize',
        }
