from uran_localization_plugin.geo import GeoReferenceConfig, MapProjector
from uran_localization_plugin.outdoor_pose_aligner import OutdoorPoseAligner


def _projector():
    return MapProjector(
        GeoReferenceConfig(
            enabled=True,
            reference_lat=39.0,
            reference_lon=116.0,
        )
    )


def test_alignment_estimates_stable_offset():
    aligner = OutdoorPoseAligner({
        'window_size': 4,
        'min_stable_samples': 2,
        'max_stable_std_m': 1.0,
        'enable_goal_correction': True,
    })
    projector = _projector()

    for index in range(2):
        result = aligner.evaluate(
            projector=projector,
            target_lat=39.0,
            target_lon=116.0,
            target_x=0.0,
            target_y=0.0,
            gps_position={'lat': 39.0, 'lon': 116.0},
            map_pose={'x': -3.0, 'y': 4.0, 'yaw_deg': 0.0},
            visual_pose={'x': float(index), 'y': 0.0, 'yaw_deg': 0.0},
            gps_fix_type=2,
            gps_num_sv=10,
            timestamp_ns=index,
        )

    assert result['stable'] is True
    assert abs(result['estimate']['offset_x'] - 3.0) < 1e-6
    assert abs(result['estimate']['offset_y'] + 4.0) < 1e-6
    assert result['goal_correction']['applied'] is True
    assert abs(result['goal_correction']['dx'] + 3.0) < 1e-6
    assert abs(result['goal_correction']['dy'] - 4.0) < 1e-6


def test_alignment_rejects_gps_jump_without_visual_support():
    aligner = OutdoorPoseAligner({
        'window_size': 4,
        'jump_reject_m': 5.0,
        'visual_support_ratio': 0.5,
    })
    projector = _projector()

    first = aligner.evaluate(
        projector=projector,
        target_lat=39.0,
        target_lon=116.0,
        target_x=0.0,
        target_y=0.0,
        gps_position={'lat': 39.0, 'lon': 116.0},
        map_pose={'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0},
        visual_pose={'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0},
        gps_fix_type=2,
        gps_num_sv=10,
        timestamp_ns=1,
    )
    second = aligner.evaluate(
        projector=projector,
        target_lat=39.0,
        target_lon=116.0,
        target_x=0.0,
        target_y=0.0,
        gps_position={'lat': 39.0001, 'lon': 116.0},
        map_pose={'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0},
        visual_pose={'x': 0.1, 'y': 0.0, 'yaw_deg': 0.0},
        gps_fix_type=2,
        gps_num_sv=10,
        timestamp_ns=2,
    )

    assert first['sample']['accepted_for_alignment'] is True
    assert second['sample']['accepted_for_alignment'] is False
    assert second['sample']['gps_weight'] == 0.0
    assert 'gps jumped' in second['message']
