import math
from typing import Any, Dict, Optional, Tuple


class GeoPoseFuser:
    """Fuse GPS and local odometry into a best-effort WGS84 position.

    The first implementation is deliberately conservative. It reports raw GPS
    while GPS is fresh, reports aligned map pose when a stable GPS-map offset is
    available, and falls back to short-term dead reckoning after GPS is stale.
    The local odometry input can be visual odometry, leg odometry, fused
    odometry, or TF. The class boundary is where a Kalman filter can replace
    the internal logic later.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        config = dict(config or {})
        self._gps_stale_timeout_s = max(0.1, _as_float(config.get('gps_stale_timeout_s'), 3.0))
        self._max_visual_pose_age_s = max(0.1, _as_float(config.get('max_visual_pose_age_s'), 2.0))
        self._max_time_alignment_error_s = max(
            0.0,
            _as_float(config.get('max_time_alignment_error_s'), 0.3),
        )
        dead_reckoning = dict(config.get('dead_reckoning') or {})
        self._dead_reckoning_enabled = _as_bool(dead_reckoning.get('enabled'), True)
        self._dead_reckoning_max_age_s = max(0.1, _as_float(dead_reckoning.get('max_age_s'), 30.0))
        self._require_map_pose_anchor = _as_bool(dead_reckoning.get('require_map_pose_anchor'), True)
        self._allow_unrotated_visual_fallback = _as_bool(
            dead_reckoning.get('allow_unrotated_visual_fallback'),
            False,
        )
        self._min_fix_type = int(_as_float(dead_reckoning.get('min_fix_type'), 2.0))
        self._min_num_sv = int(_as_float(dead_reckoning.get('min_num_sv'), 0.0))
        self._filter_cfg = dict(config.get('filter') or {})
        self._altitude_cfg = dict(config.get('altitude') or {})
        self._anchor: Optional[Dict[str, Any]] = None

    def fuse(
        self,
        *,
        pose_registry,
        projector,
        alignment_state: Optional[Dict[str, Any]],
        timestamp_ns: int,
    ) -> Dict[str, Any]:
        gps_position = pose_registry.gps_position()
        map_pose = pose_registry.latest_map_pose()
        visual_pose = pose_registry.latest_visual_pose()
        alignment_state = dict(alignment_state or {})

        self._maybe_refresh_anchor(
            pose_registry=pose_registry,
            gps_position=gps_position,
            map_pose=map_pose,
            visual_pose=visual_pose,
            timestamp_ns=timestamp_ns,
        )

        aligned_position = self._aligned_map_position(
            gps_position=gps_position,
            map_pose=map_pose,
            projector=projector,
            alignment_state=alignment_state,
            timestamp_ns=timestamp_ns,
        )
        if aligned_position is not None:
            return aligned_position

        if self._gps_is_fresh(gps_position, timestamp_ns) and self._gps_quality_ok(gps_position):
            payload = dict(gps_position)
            payload.update({
                'available': True,
                'fusion_state': 'raw_gps',
                'timestamp_ns': int(timestamp_ns or gps_position.get('timestamp_ns', 0)),
                'map_pose': dict(map_pose or {}),
                'visual_pose': dict(visual_pose or {}),
                'alignment': self._alignment_summary(alignment_state),
                'quality': {
                    'gps_age_s': self._age_s(gps_position.get('timestamp_ns', 0), timestamp_ns),
                    'visual_pose_age_s': self._pose_age_s(visual_pose, timestamp_ns),
                    'time_alignment': self._latest_time_alignment(),
                },
            })
            payload['alt'] = self._altitude(
                gps_position=gps_position,
                map_pose=map_pose,
                visual_pose=visual_pose,
            )
            return payload

        dead_reckoned = self._dead_reckoned_position(
            projector=projector,
            map_pose=map_pose,
            visual_pose=visual_pose,
            timestamp_ns=timestamp_ns,
            alignment_state=alignment_state,
        )
        if dead_reckoned is not None:
            return dead_reckoned

        return {
            'available': False,
            'fusion_state': 'unavailable',
            'timestamp_ns': int(timestamp_ns),
            'map_pose': dict(map_pose or {}),
            'visual_pose': dict(visual_pose or {}),
            'alignment': self._alignment_summary(alignment_state),
            'quality': {
                'gps_age_s': self._age_s(gps_position.get('timestamp_ns', 0), timestamp_ns) if gps_position else None,
                'visual_pose_age_s': self._pose_age_s(visual_pose, timestamp_ns),
                'dead_reckoning_anchor': dict(self._anchor or {}),
            },
        }

    def reset(self):
        self._anchor = None

    def state_snapshot(self) -> Dict[str, Any]:
        return {
            'gps_stale_timeout_s': self._gps_stale_timeout_s,
            'max_visual_pose_age_s': self._max_visual_pose_age_s,
            'max_time_alignment_error_s': self._max_time_alignment_error_s,
            'dead_reckoning_enabled': self._dead_reckoning_enabled,
            'dead_reckoning_max_age_s': self._dead_reckoning_max_age_s,
            'require_map_pose_anchor': self._require_map_pose_anchor,
            'allow_unrotated_visual_fallback': self._allow_unrotated_visual_fallback,
            'anchor': dict(self._anchor or {}),
            'filter': dict(self._filter_cfg),
            'altitude': dict(self._altitude_cfg),
        }

    def _maybe_refresh_anchor(
        self,
        *,
        pose_registry,
        gps_position: Dict[str, Any],
        map_pose: Optional[Dict[str, Any]],
        visual_pose: Optional[Dict[str, Any]],
        timestamp_ns: int,
    ):
        if not self._gps_is_fresh(gps_position, timestamp_ns):
            return
        if not self._gps_quality_ok(gps_position):
            return

        gps_timestamp_ns = int(gps_position.get('timestamp_ns') or timestamp_ns)
        aligned_map_pose = pose_registry.nearest_map_pose(
            timestamp_ns=gps_timestamp_ns,
            max_delta_s=self._max_time_alignment_error_s,
        )
        aligned_visual_pose = pose_registry.nearest_visual_pose(
            timestamp_ns=gps_timestamp_ns,
            max_delta_s=self._max_time_alignment_error_s,
        )

        anchor: Dict[str, Any] = {
            'timestamp_ns': gps_timestamp_ns,
            'lat': float(gps_position['lat']),
            'lon': float(gps_position['lon']),
            'alt': float(gps_position.get('alt', 0.0)),
            'fix_type': int(gps_position.get('fix_type', 0)),
            'num_sv': int(gps_position.get('num_sv', 0)),
            'time_alignment': {
                'max_error_s': self._max_time_alignment_error_s,
                'map_pose_time_error_s': (
                    float(aligned_map_pose.get('time_error_s'))
                    if aligned_map_pose is not None and 'time_error_s' in aligned_map_pose else None
                ),
                'visual_pose_time_error_s': (
                    float(aligned_visual_pose.get('time_error_s'))
                    if aligned_visual_pose is not None and 'time_error_s' in aligned_visual_pose else None
                ),
                'map_pose_aligned': aligned_map_pose is not None,
                'visual_pose_aligned': aligned_visual_pose is not None,
            },
        }
        if aligned_map_pose is not None:
            anchor['map_pose'] = dict(aligned_map_pose)
        elif map_pose is not None and self._max_time_alignment_error_s <= 0.0:
            anchor['map_pose'] = dict(map_pose)
        if aligned_visual_pose is not None:
            anchor['visual_pose'] = dict(aligned_visual_pose)
        elif visual_pose is not None and self._max_time_alignment_error_s <= 0.0:
            anchor['visual_pose'] = dict(visual_pose)
        self._anchor = anchor

    def _aligned_map_position(
        self,
        *,
        gps_position: Dict[str, Any],
        map_pose: Optional[Dict[str, Any]],
        projector,
        alignment_state: Dict[str, Any],
        timestamp_ns: int,
    ) -> Optional[Dict[str, Any]]:
        estimate = dict(alignment_state.get('estimate') or {})
        stable = bool(alignment_state.get('stable', False))
        if (
            not stable or
            map_pose is None or
            not projector.is_ready() or
            not bool(estimate.get('available', False))
        ):
            return None

        fused_map_x = float(map_pose['x']) + float(estimate.get('offset_x', 0.0))
        fused_map_y = float(map_pose['y']) + float(estimate.get('offset_y', 0.0))
        lat, lon = projector.map_xy_to_latlon(fused_map_x, fused_map_y)
        return {
            'available': True,
            'fusion_state': 'aligned_map_pose',
            'lat': lat,
            'lon': lon,
            'alt': self._altitude(gps_position=gps_position, map_pose=map_pose, visual_pose=None),
            'fix_type': int(gps_position.get('fix_type', 0)) if gps_position else 0,
            'num_sv': int(gps_position.get('num_sv', 0)) if gps_position else 0,
            'timestamp_ns': int(timestamp_ns),
            'map_pose': dict(map_pose),
            'alignment': {
                'offset_x': float(estimate.get('offset_x', 0.0)),
                'offset_y': float(estimate.get('offset_y', 0.0)),
                'offset_norm_m': float(estimate.get('offset_norm_m', 0.0)),
                'std_m': float(estimate.get('std_m', 0.0)),
                'sample_count': int(estimate.get('sample_count', 0)),
            },
            'quality': {
                'gps_age_s': self._age_s(gps_position.get('timestamp_ns', 0), timestamp_ns) if gps_position else None,
                'visual_pose_age_s': None,
            },
        }

    def _dead_reckoned_position(
        self,
        *,
        projector,
        map_pose: Optional[Dict[str, Any]],
        visual_pose: Optional[Dict[str, Any]],
        timestamp_ns: int,
        alignment_state: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        if not self._dead_reckoning_enabled or self._anchor is None:
            return None
        if not projector.is_ready():
            return None

        anchor_age_s = self._age_s(self._anchor.get('timestamp_ns', 0), timestamp_ns)
        if anchor_age_s is None or anchor_age_s > self._dead_reckoning_max_age_s:
            return None

        dx_dy = self._dead_reckoning_delta(
            map_pose=map_pose,
            visual_pose=visual_pose,
            timestamp_ns=timestamp_ns,
        )
        if dx_dy is None:
            return None

        anchor_x, anchor_y = projector.latlon_to_map_xy(
            float(self._anchor['lat']),
            float(self._anchor['lon']),
        )
        lat, lon = projector.map_xy_to_latlon(
            anchor_x + dx_dy[0],
            anchor_y + dx_dy[1],
        )
        return {
            'available': True,
                'fusion_state': 'local_odom_dead_reckoning',
                # Backward-compatible alias for older dashboards.
                'legacy_fusion_state': 'visual_dead_reckoning',
            'lat': lat,
            'lon': lon,
            'alt': self._altitude(
                gps_position={'alt': self._anchor.get('alt', 0.0)},
                map_pose=map_pose,
                visual_pose=visual_pose,
            ),
            'fix_type': int(self._anchor.get('fix_type', 0)),
            'num_sv': int(self._anchor.get('num_sv', 0)),
            'timestamp_ns': int(timestamp_ns),
            'map_pose': dict(map_pose or {}),
            'visual_pose': dict(visual_pose or {}),
            'alignment': self._alignment_summary(alignment_state),
            'dead_reckoning': {
                'anchor': dict(self._anchor),
                'delta_x': dx_dy[0],
                'delta_y': dx_dy[1],
                'age_s': anchor_age_s,
                'source': dx_dy[2],
            },
            'quality': {
                'gps_age_s': anchor_age_s,
                'visual_pose_age_s': self._pose_age_s(visual_pose, timestamp_ns),
                'time_alignment': self._latest_time_alignment(),
            },
        }

    def _dead_reckoning_delta(
        self,
        *,
        map_pose: Optional[Dict[str, Any]],
        visual_pose: Optional[Dict[str, Any]],
        timestamp_ns: int,
    ) -> Optional[Tuple[float, float, str]]:
        anchor_map_pose = self._anchor.get('map_pose') if self._anchor else None
        if map_pose is not None and anchor_map_pose is not None:
            return (
                float(map_pose['x']) - float(anchor_map_pose['x']),
                float(map_pose['y']) - float(anchor_map_pose['y']),
                'map_pose',
            )

        anchor_visual_pose = self._anchor.get('visual_pose') if self._anchor else None
        if visual_pose is None or anchor_visual_pose is None:
            return None
        visual_age_s = self._pose_age_s(visual_pose, timestamp_ns)
        if visual_age_s is not None and visual_age_s > self._max_visual_pose_age_s:
            return None
        if self._require_map_pose_anchor and anchor_map_pose is None:
            return None

        yaw_offset_rad = 0.0
        if anchor_map_pose is not None:
            yaw_offset_rad = math.radians(
                _normalize_angle_deg(
                    float(anchor_map_pose.get('yaw_deg', 0.0)) -
                    float(anchor_visual_pose.get('yaw_deg', 0.0))
                )
            )
        elif not self._allow_unrotated_visual_fallback:
            return None

        dx_visual = float(visual_pose['x']) - float(anchor_visual_pose['x'])
        dy_visual = float(visual_pose['y']) - float(anchor_visual_pose['y'])
        dx = math.cos(yaw_offset_rad) * dx_visual - math.sin(yaw_offset_rad) * dy_visual
        dy = math.sin(yaw_offset_rad) * dx_visual + math.cos(yaw_offset_rad) * dy_visual
        return dx, dy, 'visual_pose'

    def _gps_is_fresh(self, gps_position: Dict[str, Any], timestamp_ns: int) -> bool:
        if not gps_position or 'lat' not in gps_position or 'lon' not in gps_position:
            return False
        age_s = self._age_s(gps_position.get('timestamp_ns', 0), timestamp_ns)
        return age_s is None or age_s <= self._gps_stale_timeout_s

    def _gps_quality_ok(self, gps_position: Dict[str, Any]) -> bool:
        if not gps_position:
            return False
        if int(gps_position.get('fix_type', 0)) < self._min_fix_type:
            return False
        if self._min_num_sv > 0 and int(gps_position.get('num_sv', 0)) < self._min_num_sv:
            return False
        return True

    def _alignment_summary(self, alignment_state: Dict[str, Any]) -> Dict[str, Any]:
        estimate = dict(alignment_state.get('estimate') or {})
        return {
            'stable': bool(alignment_state.get('stable', False)),
            'sample_count': int(estimate.get('sample_count', 0)),
            'std_m': float(estimate.get('std_m', 0.0)),
            'offset_x': float(estimate.get('offset_x', 0.0)),
            'offset_y': float(estimate.get('offset_y', 0.0)),
        }

    def _latest_time_alignment(self) -> Dict[str, Any]:
        if self._anchor is None:
            return {
                'max_error_s': self._max_time_alignment_error_s,
                'map_pose_aligned': False,
                'visual_pose_aligned': False,
            }
        return dict(self._anchor.get('time_alignment') or {})

    def _altitude(
        self,
        *,
        gps_position: Optional[Dict[str, Any]],
        map_pose: Optional[Dict[str, Any]],
        visual_pose: Optional[Dict[str, Any]],
    ) -> float:
        if not _as_bool(self._altitude_cfg.get('enabled'), False):
            if gps_position and 'alt' in gps_position:
                return float(gps_position.get('alt', 0.0))
            return 0.0

        source = str(self._altitude_cfg.get('source', 'gps')).strip().lower()
        if source == 'map_pose' and map_pose is not None:
            return float(map_pose.get('z', 0.0))
        if source == 'visual_pose' and visual_pose is not None:
            return float(visual_pose.get('z', 0.0))
        if gps_position and 'alt' in gps_position:
            return float(gps_position.get('alt', 0.0))
        return 0.0

    def _pose_age_s(self, pose: Optional[Dict[str, Any]], timestamp_ns: int) -> Optional[float]:
        if pose is None:
            return None
        pose_ns = int(pose.get('stamp_sec', 0)) * 1_000_000_000 + int(pose.get('stamp_nanosec', 0))
        return self._age_s(pose_ns, timestamp_ns)

    def _age_s(self, source_timestamp_ns: Any, timestamp_ns: int) -> Optional[float]:
        try:
            source_ns = int(source_timestamp_ns)
        except Exception:
            source_ns = 0
        if source_ns <= 0 or timestamp_ns <= 0:
            return None
        return max(0.0, float(timestamp_ns - source_ns) / 1_000_000_000.0)


def _normalize_angle_deg(angle_deg: float) -> float:
    angle = float(angle_deg)
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


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
