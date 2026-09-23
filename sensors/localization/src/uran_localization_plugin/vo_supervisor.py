import math
from typing import Any, Dict, Optional


def _normalize_angle_deg(angle_deg: float) -> float:
    angle = float(angle_deg)
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


class VisualPoseSupervisor:
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self._config = dict(config or {})
        self._last_check_ts = 0.0
        self._violation_count = 0
        self._state: Dict[str, Any] = {}
        self._anchor: Optional[Dict[str, float]] = None

    def enabled(self) -> bool:
        return bool(self._config.get('enabled', False))

    def reset(self):
        self._last_check_ts = 0.0
        self._violation_count = 0
        self._state = {}
        self._anchor = None

    def state_snapshot(self) -> Dict[str, Any]:
        snapshot = dict(self._state)
        if self._anchor is not None:
            snapshot['anchor'] = dict(self._anchor)
        return snapshot

    def evaluate(
        self,
        *,
        pose_registry,
        now_ns: int,
        monotonic_s: float,
    ) -> Optional[Dict[str, Any]]:
        if not self.enabled():
            return None

        interval = float(self._config.get('check_interval_s', 1.5))
        if monotonic_s - self._last_check_ts < interval:
            return None
        self._last_check_ts = monotonic_s

        map_pose = pose_registry.latest_map_pose()
        visual_pose = pose_registry.latest_visual_pose()
        if map_pose is None or visual_pose is None:
            return None

        if self._anchor is None:
            self._anchor = self._build_anchor(map_pose=map_pose, visual_pose=visual_pose)

        predicted_map_pose = self._predict_map_pose(visual_pose=visual_pose, anchor=self._anchor)
        dx = float(predicted_map_pose['x']) - float(map_pose['x'])
        dy = float(predicted_map_pose['y']) - float(map_pose['y'])
        position_error_m = (dx * dx + dy * dy) ** 0.5
        yaw_error_deg = abs(
            _normalize_angle_deg(float(predicted_map_pose['yaw_deg']) - float(map_pose['yaw_deg']))
        )

        max_pose_error_m = float(self._config.get('max_pose_error_m', 1.5))
        max_yaw_error_deg = float(self._config.get('max_yaw_error_deg', 20.0))

        self._state = {
            'map_pose_frame': map_pose.get('frame_id', ''),
            'visual_pose_frame': visual_pose.get('frame_id', ''),
            'predicted_map_x': float(predicted_map_pose['x']),
            'predicted_map_y': float(predicted_map_pose['y']),
            'predicted_map_yaw_deg': float(predicted_map_pose['yaw_deg']),
            'map_pose_x': float(map_pose['x']),
            'map_pose_y': float(map_pose['y']),
            'map_pose_yaw_deg': float(map_pose['yaw_deg']),
            'visual_pose_x': float(visual_pose['x']),
            'visual_pose_y': float(visual_pose['y']),
            'visual_pose_yaw_deg': float(visual_pose['yaw_deg']),
            'position_error_m': position_error_m,
            'yaw_error_deg': yaw_error_deg,
            'max_pose_error_m': max_pose_error_m,
            'max_yaw_error_deg': max_yaw_error_deg,
            'timestamp_ns': int(now_ns),
        }

        if position_error_m <= max_pose_error_m and yaw_error_deg <= max_yaw_error_deg:
            self._violation_count = 0
            self._state['violation_count'] = 0
            return None

        self._violation_count += 1
        self._state['violation_count'] = self._violation_count
        required = int(self._config.get('violation_count_threshold', 2))
        if self._violation_count < required:
            return None

        action = str(self._config.get('on_large_error', 'pause')).strip().lower()
        description = (
            'visual pose/map pose mismatch too large: '
            f'position_error={position_error_m:.2f}m, '
            f'yaw_error={yaw_error_deg:.1f}deg, '
            f'map_frame={map_pose.get("frame_id", "")}, '
            f'visual_frame={visual_pose.get("frame_id", "")}'
        )
        return {
            'action': 'abort' if action == 'abort' else 'pause',
            'code': 'E_VISUAL_MAP_MISMATCH',
            'description': description,
            'suggested_action': 'relocalize',
        }

    def _build_anchor(self, *, map_pose: Dict[str, Any], visual_pose: Dict[str, Any]) -> Dict[str, float]:
        visual_yaw_rad = math.radians(float(visual_pose['yaw_deg']))
        map_yaw_deg = float(map_pose['yaw_deg'])
        yaw_offset_deg = _normalize_angle_deg(map_yaw_deg - float(visual_pose['yaw_deg']))
        yaw_offset_rad = math.radians(yaw_offset_deg)

        rotated_visual_x = (
            math.cos(yaw_offset_rad) * float(visual_pose['x']) -
            math.sin(yaw_offset_rad) * float(visual_pose['y'])
        )
        rotated_visual_y = (
            math.sin(yaw_offset_rad) * float(visual_pose['x']) +
            math.cos(yaw_offset_rad) * float(visual_pose['y'])
        )
        return {
            'anchor_x': float(map_pose['x']) - rotated_visual_x,
            'anchor_y': float(map_pose['y']) - rotated_visual_y,
            'yaw_offset_deg': yaw_offset_deg,
            'visual_initial_yaw_deg': float(visual_pose['yaw_deg']),
            'map_initial_yaw_deg': map_yaw_deg,
            'visual_initial_yaw_rad': visual_yaw_rad,
        }

    def _predict_map_pose(self, *, visual_pose: Dict[str, Any], anchor: Dict[str, float]) -> Dict[str, float]:
        yaw_offset_rad = math.radians(float(anchor['yaw_offset_deg']))
        visual_x = float(visual_pose['x'])
        visual_y = float(visual_pose['y'])
        rotated_x = math.cos(yaw_offset_rad) * visual_x - math.sin(yaw_offset_rad) * visual_y
        rotated_y = math.sin(yaw_offset_rad) * visual_x + math.cos(yaw_offset_rad) * visual_y
        return {
            'x': float(anchor['anchor_x']) + rotated_x,
            'y': float(anchor['anchor_y']) + rotated_y,
            'yaw_deg': _normalize_angle_deg(
                float(visual_pose['yaw_deg']) + float(anchor['yaw_offset_deg'])
            ),
        }
