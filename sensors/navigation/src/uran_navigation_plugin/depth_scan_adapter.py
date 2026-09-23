import math
import struct
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple


_POINT_FIELD_FORMATS = {
    1: ('b', 1),
    2: ('B', 1),
    3: ('h', 2),
    4: ('H', 2),
    5: ('i', 4),
    6: ('I', 4),
    7: ('f', 4),
    8: ('d', 8),
}


@dataclass(frozen=True)
class RigidTransform:
    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]

    @classmethod
    def identity(cls):
        return cls(
            translation=(0.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
        )

    @classmethod
    def from_ros(cls, transform):
        value = getattr(transform, 'transform', transform)
        translation = getattr(value, 'translation', None)
        rotation = getattr(value, 'rotation', None)
        if translation is None or rotation is None:
            raise ValueError('transform has no translation or rotation')
        return cls(
            translation=(
                float(translation.x),
                float(translation.y),
                float(translation.z),
            ),
            rotation=(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        ).normalized()

    def normalized(self):
        qx, qy, qz, qw = self.rotation
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-12 or not math.isfinite(norm):
            raise ValueError('transform quaternion is invalid')
        return RigidTransform(
            translation=self.translation,
            rotation=(qx / norm, qy / norm, qz / norm, qw / norm),
        )

    def apply(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        qx, qy, qz, qw = self.rotation
        tx, ty, tz = self.translation

        # Quaternion-vector rotation without allocating intermediate objects.
        uv_x = qy * z - qz * y
        uv_y = qz * x - qx * z
        uv_z = qx * y - qy * x
        uuv_x = qy * uv_z - qz * uv_y
        uuv_y = qz * uv_x - qx * uv_z
        uuv_z = qx * uv_y - qy * uv_x
        scale = 2.0 * qw
        return (
            x + scale * uv_x + 2.0 * uuv_x + tx,
            y + scale * uv_y + 2.0 * uuv_y + ty,
            z + scale * uv_z + 2.0 * uuv_z + tz,
        )


@dataclass
class VirtualLaserScan:
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: List[float]


class DepthPointCloudScanAdapter:
    """Convert an organized PointCloud2 into a base-frame planar scan."""

    def __init__(
        self,
        config: Optional[Dict[str, Any]] = None,
        *,
        monotonic_getter=None,
    ):
        cfg = dict(config or {})
        self.target_frame = str(cfg.get('depth_target_frame', 'base_link') or 'base_link')
        self.min_height_m = float(cfg.get('depth_min_height_m', 0.05))
        self.max_height_m = float(cfg.get('depth_max_height_m', 0.80))
        if self.max_height_m <= self.min_height_m:
            raise ValueError('depth_max_height_m must exceed depth_min_height_m')

        self.min_range_m = max(0.01, float(cfg.get('depth_min_range_m', 0.15)))
        self.max_range_m = max(
            self.min_range_m + 0.01,
            float(cfg.get('depth_max_range_m', 3.0)),
        )
        self.sensor_max_range_m = max(
            self.max_range_m,
            float(cfg.get('depth_sensor_max_range_m', 10.0)),
        )
        self.horizontal_fov_rad = math.radians(
            max(1.0, min(179.0, float(cfg.get('depth_horizontal_fov_deg', 80.0))))
        )
        self.angle_increment_rad = math.radians(
            max(0.1, float(cfg.get('depth_angle_increment_deg', 1.0)))
        )
        self.row_stride = max(1, int(cfg.get('depth_sample_row_stride', 4)))
        self.column_stride = max(1, int(cfg.get('depth_sample_column_stride', 4)))
        self.min_obstacle_points_per_bin = max(
            1,
            int(cfg.get('depth_min_obstacle_points_per_bin', 2)),
        )
        self.min_clear_points_per_bin = max(
            1,
            int(cfg.get('depth_min_clear_points_per_bin', 2)),
        )
        self.distance_percentile = max(
            0.0,
            min(1.0, float(cfg.get('depth_distance_percentile', 0.10))),
        )
        self._monotonic_getter = monotonic_getter or time.monotonic
        self._last_update_monotonic_s: Optional[float] = None
        self._state: Dict[str, Any] = {
            'source': 'depth_pointcloud',
            'target_frame': self.target_frame,
            'status': 'waiting_for_pointcloud',
            'received_frames': 0,
            'converted_frames': 0,
            'skipped_frames': 0,
            'conversion_errors': 0,
            'last_error': '',
            'source_frame': '',
        }

    def record_received(self, source_frame: str = ''):
        self._state['received_frames'] = int(self._state['received_frames']) + 1
        self._state['source_frame'] = str(source_frame or '')
        self._state['status'] = 'converting'

    def record_error(self, message: str, *, status: str = 'conversion_error'):
        self._state['conversion_errors'] = int(self._state['conversion_errors']) + 1
        self._state['status'] = str(status)
        self._state['last_error'] = str(message)

    def record_skipped(self):
        self._state['skipped_frames'] = int(self._state['skipped_frames']) + 1

    def convert(
        self,
        cloud,
        transform: Optional[RigidTransform] = None,
        *,
        monotonic_s: Optional[float] = None,
    ) -> VirtualLaserScan:
        source_frame = str(getattr(getattr(cloud, 'header', None), 'frame_id', '') or '')
        self.record_received(source_frame)
        rigid_transform = transform or RigidTransform.identity()
        try:
            scan, metrics = self._convert(cloud, rigid_transform)
        except Exception as exc:
            self.record_error(str(exc))
            raise

        now = self._monotonic_getter() if monotonic_s is None else float(monotonic_s)
        self._last_update_monotonic_s = now
        self._state.update(metrics)
        self._state['converted_frames'] = int(self._state['converted_frames']) + 1
        self._state['status'] = (
            'healthy' if metrics['valid_bin_ratio'] >= 0.5 else 'degraded'
        )
        self._state['last_error'] = ''
        return scan

    def _convert(
        self,
        cloud,
        transform: RigidTransform,
    ) -> Tuple[VirtualLaserScan, Dict[str, Any]]:
        width = int(getattr(cloud, 'width', 0) or 0)
        height = int(getattr(cloud, 'height', 0) or 0)
        point_step = int(getattr(cloud, 'point_step', 0) or 0)
        row_step = int(getattr(cloud, 'row_step', 0) or 0)
        data = getattr(cloud, 'data', b'')
        if width <= 0 or height <= 0 or point_step <= 0 or row_step <= 0 or not data:
            raise ValueError('point cloud dimensions or data are empty')
        if row_step < width * point_step:
            raise ValueError('point cloud row_step is smaller than width * point_step')
        if len(data) < row_step * height:
            raise ValueError('point cloud data is shorter than row_step * height')

        readers = self._coordinate_readers(
            getattr(cloud, 'fields', []),
            point_step=point_step,
            is_bigendian=bool(getattr(cloud, 'is_bigendian', False)),
        )
        bin_count = max(2, int(round(self.horizontal_fov_rad / self.angle_increment_rad)) + 1)
        angle_min = -0.5 * self.horizontal_fov_rad
        angle_increment = self.horizontal_fov_rad / float(bin_count - 1)
        observed_counts = [0] * bin_count
        obstacle_ranges: List[List[float]] = [[] for _ in range(bin_count)]

        sampled_count = 0
        finite_count = 0
        in_fov_count = 0
        height_band_count = 0
        unpack_x, unpack_y, unpack_z = readers
        for row in range(0, height, self.row_stride):
            row_offset = row * row_step
            for column in range(0, width, self.column_stride):
                sampled_count += 1
                point_offset = row_offset + column * point_step
                x = unpack_x(data, point_offset)
                y = unpack_y(data, point_offset)
                z = unpack_z(data, point_offset)
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                finite_count += 1
                base_x, base_y, base_z = transform.apply(x, y, z)
                if not (
                    math.isfinite(base_x) and
                    math.isfinite(base_y) and
                    math.isfinite(base_z)
                ):
                    continue
                planar_range = math.hypot(base_x, base_y)
                if base_x <= 0.0 or planar_range > self.sensor_max_range_m:
                    continue
                angle = math.atan2(base_y, base_x)
                if angle < angle_min or angle > -angle_min:
                    continue
                index = int(round((angle - angle_min) / angle_increment))
                index = max(0, min(bin_count - 1, index))
                in_fov_count += 1
                observed_counts[index] += 1
                if self.min_height_m <= base_z <= self.max_height_m:
                    height_band_count += 1
                    obstacle_ranges[index].append(max(self.min_range_m, planar_range))

        ranges: List[float] = []
        obstacle_bin_count = 0
        clear_bin_count = 0
        unknown_bin_count = 0
        for index in range(bin_count):
            values = obstacle_ranges[index]
            if len(values) >= self.min_obstacle_points_per_bin:
                values.sort()
                percentile_index = int(math.floor((len(values) - 1) * self.distance_percentile))
                distance = values[percentile_index]
                ranges.append(distance if distance <= self.max_range_m else float('inf'))
                obstacle_bin_count += int(distance <= self.max_range_m)
                clear_bin_count += int(distance > self.max_range_m)
            elif observed_counts[index] >= self.min_clear_points_per_bin:
                ranges.append(float('inf'))
                clear_bin_count += 1
            else:
                ranges.append(float('nan'))
                unknown_bin_count += 1

        valid_bin_count = bin_count - unknown_bin_count
        scan = VirtualLaserScan(
            angle_min=angle_min,
            angle_max=-angle_min,
            angle_increment=angle_increment,
            range_min=self.min_range_m,
            range_max=self.max_range_m,
            ranges=ranges,
        )
        metrics = {
            'input_width': width,
            'input_height': height,
            'input_point_count': width * height,
            'sampled_point_count': sampled_count,
            'finite_point_count': finite_count,
            'in_fov_point_count': in_fov_count,
            'height_band_point_count': height_band_count,
            'scan_bin_count': bin_count,
            'obstacle_bin_count': obstacle_bin_count,
            'clear_bin_count': clear_bin_count,
            'unknown_bin_count': unknown_bin_count,
            'valid_bin_ratio': float(valid_bin_count) / float(bin_count),
        }
        return scan, metrics

    @staticmethod
    def _coordinate_readers(
        fields: Sequence[Any],
        *,
        point_step: int,
        is_bigendian: bool,
    ):
        by_name = {str(getattr(field, 'name', '')): field for field in fields}
        readers = []
        endian = '>' if is_bigendian else '<'
        for name in ('x', 'y', 'z'):
            field = by_name.get(name)
            if field is None:
                raise ValueError(f'point cloud is missing {name} field')
            datatype = int(getattr(field, 'datatype', 0) or 0)
            format_info = _POINT_FIELD_FORMATS.get(datatype)
            if format_info is None:
                raise ValueError(f'point cloud {name} field datatype {datatype} is unsupported')
            format_code, size = format_info
            offset = int(getattr(field, 'offset', -1))
            count = int(getattr(field, 'count', 1) or 1)
            if count < 1 or offset < 0 or offset + size > point_step:
                raise ValueError(f'point cloud {name} field layout is invalid')
            unpacker = struct.Struct(endian + format_code)
            readers.append(
                lambda data, point_offset, unpacker=unpacker, offset=offset: float(
                    unpacker.unpack_from(data, point_offset + offset)[0]
                )
            )
        return tuple(readers)

    def snapshot(self, *, monotonic_s: Optional[float] = None) -> Dict[str, Any]:
        result = dict(self._state)
        if self._last_update_monotonic_s is None:
            result['scan_age_s'] = None
        else:
            now = self._monotonic_getter() if monotonic_s is None else float(monotonic_s)
            result['scan_age_s'] = max(0.0, now - self._last_update_monotonic_s)
        result['height_filter_m'] = [self.min_height_m, self.max_height_m]
        result['range_filter_m'] = [self.min_range_m, self.max_range_m]
        result['horizontal_fov_deg'] = math.degrees(self.horizontal_fov_rad)
        result['angle_increment_deg'] = math.degrees(self.angle_increment_rad)
        return result
