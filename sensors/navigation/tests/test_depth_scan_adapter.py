import math
import struct
from types import SimpleNamespace

import pytest

from uran_navigation_plugin.depth_scan_adapter import (
    DepthPointCloudScanAdapter,
    RigidTransform,
)
from uran_navigation_plugin.obstacle_avoidance import GapAvoidancePlanner


def _cloud(points, *, width=None, fields=None):
    width = len(points) if width is None else int(width)
    height = int(math.ceil(float(len(points)) / float(width)))
    padded = list(points) + [(float('nan'),) * 3] * (width * height - len(points))
    data = b''.join(struct.pack('<fff', *point) for point in padded)
    default_fields = [
        SimpleNamespace(name='x', offset=0, datatype=7, count=1),
        SimpleNamespace(name='y', offset=4, datatype=7, count=1),
        SimpleNamespace(name='z', offset=8, datatype=7, count=1),
    ]
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='base_link'),
        width=width,
        height=height,
        point_step=12,
        row_step=width * 12,
        is_bigendian=False,
        fields=default_fields if fields is None else fields,
        data=data,
    )


def _adapter(**overrides):
    config = {
        'depth_horizontal_fov_deg': 80.0,
        'depth_angle_increment_deg': 1.0,
        'depth_sample_row_stride': 1,
        'depth_sample_column_stride': 1,
        'depth_min_obstacle_points_per_bin': 1,
        'depth_min_clear_points_per_bin': 1,
        'depth_min_height_m': 0.05,
        'depth_max_height_m': 0.80,
        'depth_min_range_m': 0.15,
        'depth_max_range_m': 3.0,
    }
    config.update(overrides)
    return DepthPointCloudScanAdapter(config)


def _range_at(scan, angle_deg):
    index = int(round((math.radians(angle_deg) - scan.angle_min) / scan.angle_increment))
    return scan.ranges[index]


def test_front_obstacle_is_projected_to_center_bin():
    scan = _adapter().convert(_cloud([(1.0, 0.0, 0.30)]))

    assert _range_at(scan, 0.0) == pytest.approx(1.0)


def test_left_and_right_obstacles_keep_base_link_angle_sign():
    scan = _adapter().convert(_cloud([
        (2.0, 1.0, 0.30),
        (2.0, -1.0, 0.30),
    ]))

    expected = math.sqrt(5.0)
    assert _range_at(scan, math.degrees(math.atan2(1.0, 2.0))) == pytest.approx(expected)
    assert _range_at(scan, math.degrees(math.atan2(-1.0, 2.0))) == pytest.approx(expected)


def test_ground_and_high_points_are_clear_not_obstacles():
    scan = _adapter().convert(_cloud([
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 1.0),
    ]))

    assert math.isinf(_range_at(scan, 0.0))


def test_missing_depth_evidence_remains_unknown():
    adapter = _adapter()
    scan = adapter.convert(_cloud([(float('nan'),) * 3]))

    assert all(math.isnan(value) for value in scan.ranges)
    assert adapter.snapshot()['status'] == 'degraded'


def test_transform_is_applied_before_filtering_and_projection():
    transform = RigidTransform(
        translation=(1.0, 0.0, 0.0),
        rotation=(0.0, 0.0, 0.0, 1.0),
    )
    scan = _adapter().convert(_cloud([(1.0, 0.0, 0.30)]), transform)

    assert _range_at(scan, 0.0) == pytest.approx(2.0)


def test_invalid_point_cloud_layout_is_rejected_and_reported():
    cloud = _cloud(
        [(1.0, 0.0, 0.30)],
        fields=[SimpleNamespace(name='x', offset=0, datatype=7, count=1)],
    )
    adapter = _adapter()

    with pytest.raises(ValueError, match='missing y field'):
        adapter.convert(cloud)

    assert adapter.snapshot()['status'] == 'conversion_error'
    assert adapter.snapshot()['conversion_errors'] == 1


def test_virtual_scan_drives_existing_obstacle_planner():
    points = []
    for angle_deg in range(-40, 41):
        distance = 0.4 if -4 <= angle_deg <= 4 else 4.0
        angle = math.radians(angle_deg)
        points.append((
            distance * math.cos(angle),
            distance * math.sin(angle),
            0.30,
        ))
    scan = _adapter().convert(_cloud(points))
    planner = GapAvoidancePlanner({
        'usable_fov_deg': 80.0,
        'min_scan_valid_ratio': 0.5,
        'min_front_valid_ratio': 0.5,
        'min_stop_distance_m': 0.65,
    })

    decision = planner.plan(scan, now_s=0.0)

    assert decision.state in {'stop', 'blocked'}
    assert decision.front_clearance_m == pytest.approx(0.4)


def test_unknown_depth_scan_is_rejected_by_existing_quality_gate():
    scan = _adapter().convert(_cloud([(float('nan'),) * 3]))
    planner = GapAvoidancePlanner({
        'usable_fov_deg': 80.0,
        'min_scan_valid_ratio': 0.5,
        'min_front_valid_ratio': 0.5,
    })

    decision = planner.plan(scan, now_s=0.0)

    assert decision.state == 'blocked'
    assert decision.scan_quality == 'invalid'
