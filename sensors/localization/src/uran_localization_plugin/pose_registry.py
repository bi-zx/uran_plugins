import math
from collections import deque
from typing import Any, Dict, Optional


def _quaternion_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class PoseRegistry:
    def __init__(self, *, history_size: int = 600):
        self._cloud_position: Dict[str, Any] = {}
        self._gps_position: Dict[str, Any] = {}
        self._gps_fix_type = 0
        self._gps_num_sv = 0
        self._latest_map_pose: Optional[Dict[str, Any]] = None
        self._latest_visual_pose: Optional[Dict[str, Any]] = None
        max_history = max(1, int(history_size))
        self._gps_history = deque(maxlen=max_history)
        self._map_pose_history = deque(maxlen=max_history)
        self._visual_pose_history = deque(maxlen=max_history)

    def update_cloud_position(self, position: Optional[Dict[str, Any]]):
        if isinstance(position, dict):
            self._cloud_position = dict(position)
            return
        self._cloud_position = {}

    def update_gps_payload(
        self,
        *,
        lat: float,
        lon: float,
        alt: float = 0.0,
        fix_type: int = 0,
        num_sv: int = 0,
        timestamp_ns: int = 0,
    ):
        self._gps_fix_type = int(fix_type)
        self._gps_num_sv = int(num_sv)
        self._gps_position = {
            'lat': float(lat),
            'lon': float(lon),
            'alt': float(alt),
            'fix_type': self._gps_fix_type,
            'num_sv': self._gps_num_sv,
            'timestamp_ns': int(timestamp_ns),
        }
        self._gps_history.append(dict(self._gps_position))

    def update_map_pose(self, pose: Dict[str, Any]):
        self._latest_map_pose = self._normalize_pose_dict(pose)
        self._map_pose_history.append(dict(self._latest_map_pose))

    def update_visual_pose(self, pose: Dict[str, Any]):
        self._latest_visual_pose = self._normalize_pose_dict(pose)
        self._visual_pose_history.append(dict(self._latest_visual_pose))

    def cloud_position(self) -> Dict[str, Any]:
        return dict(self._cloud_position)

    def gps_position(self) -> Dict[str, Any]:
        return dict(self._gps_position)

    def effective_position(self) -> Dict[str, Any]:
        if self._cloud_position:
            return dict(self._cloud_position)
        if self._gps_position:
            return dict(self._gps_position)
        return {}

    def latest_map_pose(self) -> Optional[Dict[str, Any]]:
        if self._latest_map_pose is None:
            return None
        return dict(self._latest_map_pose)

    def latest_visual_pose(self) -> Optional[Dict[str, Any]]:
        if self._latest_visual_pose is None:
            return None
        return dict(self._latest_visual_pose)

    def gps_fix_type(self) -> int:
        return self._gps_fix_type

    def gps_num_sv(self) -> int:
        return self._gps_num_sv

    def map_pose_dict(self) -> Dict[str, Any]:
        if self._latest_map_pose is None:
            return {}
        return dict(self._latest_map_pose)

    def visual_pose_dict(self) -> Dict[str, Any]:
        if self._latest_visual_pose is None:
            return {}
        return dict(self._latest_visual_pose)

    def nearest_map_pose(
        self,
        *,
        timestamp_ns: int,
        max_delta_s: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        return self._nearest_pose(
            self._map_pose_history,
            timestamp_ns=timestamp_ns,
            max_delta_s=max_delta_s,
        )

    def nearest_visual_pose(
        self,
        *,
        timestamp_ns: int,
        max_delta_s: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        return self._nearest_pose(
            self._visual_pose_history,
            timestamp_ns=timestamp_ns,
            max_delta_s=max_delta_s,
        )

    def history_summary(self) -> Dict[str, Any]:
        return {
            'gps_samples': len(self._gps_history),
            'map_pose_samples': len(self._map_pose_history),
            'visual_pose_samples': len(self._visual_pose_history),
            'latest_gps_timestamp_ns': int(self._gps_position.get('timestamp_ns', 0)),
            'latest_map_pose_timestamp_ns': (
                int(self._latest_map_pose.get('timestamp_ns', 0))
                if self._latest_map_pose is not None else 0
            ),
            'latest_visual_pose_timestamp_ns': (
                int(self._latest_visual_pose.get('timestamp_ns', 0))
                if self._latest_visual_pose is not None else 0
            ),
        }

    def fused_geo_position(
        self,
        *,
        projector,
        alignment_state: Optional[Dict[str, Any]] = None,
        timestamp_ns: int = 0,
    ) -> Dict[str, Any]:
        gps_position = self.gps_position()
        map_pose = self.latest_map_pose()
        alignment_state = dict(alignment_state or {})
        estimate = dict(alignment_state.get('estimate') or {})
        stable = bool(alignment_state.get('stable', False))

        if (
            stable and
            map_pose is not None and
            projector.is_ready() and
            bool(estimate.get('available', False))
        ):
            fused_map_x = float(map_pose['x']) + float(estimate.get('offset_x', 0.0))
            fused_map_y = float(map_pose['y']) + float(estimate.get('offset_y', 0.0))
            lat, lon = projector.map_xy_to_latlon(fused_map_x, fused_map_y)
            return {
                'available': True,
                'fusion_state': 'aligned_map_pose',
                'lat': lat,
                'lon': lon,
                'alt': float(gps_position.get('alt', map_pose.get('z', 0.0))) if gps_position else float(map_pose.get('z', 0.0)),
                'fix_type': int(gps_position.get('fix_type', self._gps_fix_type)) if gps_position else self._gps_fix_type,
                'num_sv': int(gps_position.get('num_sv', self._gps_num_sv)) if gps_position else self._gps_num_sv,
                'timestamp_ns': int(timestamp_ns or map_pose.get('stamp_sec', 0) * 1_000_000_000 + map_pose.get('stamp_nanosec', 0)),
                'map_pose': dict(map_pose),
                'alignment': {
                    'offset_x': float(estimate.get('offset_x', 0.0)),
                    'offset_y': float(estimate.get('offset_y', 0.0)),
                    'offset_norm_m': float(estimate.get('offset_norm_m', 0.0)),
                    'std_m': float(estimate.get('std_m', 0.0)),
                    'sample_count': int(estimate.get('sample_count', 0)),
                },
            }

        if gps_position:
            payload = dict(gps_position)
            payload.update({
                'available': True,
                'fusion_state': 'raw_gps',
                'timestamp_ns': int(timestamp_ns or gps_position.get('timestamp_ns', 0)),
                'map_pose': dict(map_pose or {}),
                'alignment': {
                    'stable': stable,
                    'sample_count': int(estimate.get('sample_count', 0)),
                    'std_m': float(estimate.get('std_m', 0.0)),
                },
            })
            return payload

        return {
            'available': False,
            'fusion_state': 'unavailable',
            'timestamp_ns': int(timestamp_ns),
            'map_pose': dict(map_pose or {}),
            'alignment': {
                'stable': stable,
                'sample_count': int(estimate.get('sample_count', 0)),
            },
        }

    def _normalize_pose_dict(self, pose: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        pose = dict(pose or {})
        qx = float(pose.get('qx', 0.0))
        qy = float(pose.get('qy', 0.0))
        qz = float(pose.get('qz', 0.0))
        qw = float(pose.get('qw', 1.0))
        return {
            'frame_id': str(pose.get('frame_id', '')),
            'stamp_sec': int(pose.get('stamp_sec', 0)),
            'stamp_nanosec': int(pose.get('stamp_nanosec', 0)),
            'timestamp_ns': (
                int(pose.get('timestamp_ns', 0)) or
                int(pose.get('stamp_sec', 0)) * 1_000_000_000 +
                int(pose.get('stamp_nanosec', 0))
            ),
            'x': float(pose.get('x', 0.0)),
            'y': float(pose.get('y', 0.0)),
            'z': float(pose.get('z', 0.0)),
            'qx': qx,
            'qy': qy,
            'qz': qz,
            'qw': qw,
            'yaw_deg': float(pose.get('yaw_deg', _quaternion_to_yaw_deg(qx, qy, qz, qw))),
        }

    def _nearest_pose(
        self,
        history,
        *,
        timestamp_ns: int,
        max_delta_s: Optional[float],
    ) -> Optional[Dict[str, Any]]:
        try:
            target_ns = int(timestamp_ns)
        except Exception:
            target_ns = 0
        if target_ns <= 0 or not history:
            return None

        best_pose = None
        best_delta_ns = None
        for pose in history:
            pose_ns = int(pose.get('timestamp_ns', 0))
            if pose_ns <= 0:
                continue
            delta_ns = abs(pose_ns - target_ns)
            if best_delta_ns is None or delta_ns < best_delta_ns:
                best_delta_ns = delta_ns
                best_pose = pose

        if best_pose is None or best_delta_ns is None:
            return None

        time_error_s = float(best_delta_ns) / 1_000_000_000.0
        if max_delta_s is not None and time_error_s > float(max_delta_s):
            return None

        result = dict(best_pose)
        result['time_error_s'] = time_error_s
        return result
