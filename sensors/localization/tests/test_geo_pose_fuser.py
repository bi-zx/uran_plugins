from uran_localization_plugin.geo import GeoReferenceConfig, MapProjector
from uran_localization_plugin.geo_pose_fuser import GeoPoseFuser
from uran_localization_plugin.pose_registry import PoseRegistry


def _projector():
    return MapProjector(
        GeoReferenceConfig(
            enabled=True,
            reference_lat=39.0,
            reference_lon=116.0,
        )
    )


def test_fuser_reports_raw_gps_when_gps_is_fresh():
    registry = PoseRegistry()
    registry.update_gps_payload(
        lat=39.0,
        lon=116.0,
        fix_type=2,
        num_sv=10,
        timestamp_ns=1_000_000_000,
    )
    fuser = GeoPoseFuser({'gps_stale_timeout_s': 3.0})

    result = fuser.fuse(
        pose_registry=registry,
        projector=_projector(),
        alignment_state={},
        timestamp_ns=2_000_000_000,
    )

    assert result['available'] is True
    assert result['fusion_state'] == 'raw_gps'
    assert result['lat'] == 39.0
    assert result['lon'] == 116.0


def test_fuser_rejects_invalid_fresh_gps():
    registry = PoseRegistry()
    registry.update_gps_payload(
        lat=0.0,
        lon=0.0,
        fix_type=0,
        num_sv=0,
        timestamp_ns=1_000_000_000,
    )
    fuser = GeoPoseFuser({'gps_stale_timeout_s': 3.0})

    result = fuser.fuse(
        pose_registry=registry,
        projector=_projector(),
        alignment_state={},
        timestamp_ns=2_000_000_000,
    )

    assert result['available'] is False
    assert result['fusion_state'] == 'unavailable'


def test_fuser_dead_reckons_from_map_pose_after_gps_is_stale():
    registry = PoseRegistry()
    registry.update_gps_payload(
        lat=39.0,
        lon=116.0,
        fix_type=2,
        num_sv=10,
        timestamp_ns=1_000_000_000,
    )
    registry.update_map_pose({
        'frame_id': 'map',
        'stamp_sec': 1,
        'stamp_nanosec': 0,
        'x': 10.0,
        'y': 20.0,
        'z': 0.0,
        'qw': 1.0,
    })
    fuser = GeoPoseFuser({'gps_stale_timeout_s': 3.0})
    projector = _projector()

    fuser.fuse(
        pose_registry=registry,
        projector=projector,
        alignment_state={},
        timestamp_ns=2_000_000_000,
    )
    registry.update_map_pose({
        'frame_id': 'map',
        'stamp_sec': 6,
        'stamp_nanosec': 0,
        'x': 15.0,
        'y': 20.0,
        'z': 0.0,
        'qw': 1.0,
    })

    result = fuser.fuse(
        pose_registry=registry,
        projector=projector,
        alignment_state={},
        timestamp_ns=6_000_000_000,
    )

    assert result['available'] is True
    assert result['fusion_state'] == 'local_odom_dead_reckoning'
    assert result['legacy_fusion_state'] == 'visual_dead_reckoning'
    assert result['dead_reckoning']['source'] == 'map_pose'
    assert abs(result['dead_reckoning']['delta_x'] - 5.0) < 1e-6


def test_fuser_uses_time_aligned_pose_for_gps_anchor():
    registry = PoseRegistry()
    registry.update_map_pose({
        'frame_id': 'map',
        'stamp_sec': 1,
        'stamp_nanosec': 0,
        'x': 10.0,
        'y': 0.0,
        'z': 0.0,
        'qw': 1.0,
    })
    registry.update_map_pose({
        'frame_id': 'map',
        'stamp_sec': 2,
        'stamp_nanosec': 0,
        'x': 20.0,
        'y': 0.0,
        'z': 0.0,
        'qw': 1.0,
    })
    registry.update_gps_payload(
        lat=39.0,
        lon=116.0,
        fix_type=2,
        num_sv=10,
        timestamp_ns=1_050_000_000,
    )
    fuser = GeoPoseFuser({
        'gps_stale_timeout_s': 3.0,
        'max_time_alignment_error_s': 0.2,
    })

    result = fuser.fuse(
        pose_registry=registry,
        projector=_projector(),
        alignment_state={},
        timestamp_ns=1_100_000_000,
    )
    anchor = result['quality']['time_alignment']

    assert anchor['map_pose_aligned'] is True
    assert abs(anchor['map_pose_time_error_s'] - 0.05) < 1e-6
    assert fuser.state_snapshot()['anchor']['map_pose']['x'] == 10.0


def test_fuser_returns_unavailable_without_gps_anchor():
    registry = PoseRegistry()
    fuser = GeoPoseFuser({'dead_reckoning': {'enabled': True}})

    result = fuser.fuse(
        pose_registry=registry,
        projector=_projector(),
        alignment_state={},
        timestamp_ns=1_000_000_000,
    )

    assert result['available'] is False
    assert result['fusion_state'] == 'unavailable'
