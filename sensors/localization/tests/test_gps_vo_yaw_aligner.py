from uran_localization_plugin.geo import EARTH_RADIUS_M
from uran_localization_plugin.gps_vo_yaw_aligner import GpsVoYawAligner


def test_calibration_drive_distance_keeps_margin_above_sample_thresholds():
    aligner = GpsVoYawAligner({
        'min_gps_displacement_m': 3.0,
        'min_visual_displacement_m': 3.8,
        'calibration_drive_distance_m': 3.0,
    })

    assert aligner.calibration_drive_distance_m == 4.3
    assert aligner.config_snapshot()['calibration_drive_distance_m'] == 4.3


def _gps_from_local(lat0, lon0, x, y):
    lat = lat0 + (y / EARTH_RADIUS_M) * 180.0 / 3.141592653589793
    lon = lon0 + (x / (EARTH_RADIUS_M * 0.7771459614569709)) * 180.0 / 3.141592653589793
    return lat, lon


def test_yaw_alignment_accepts_four_satellites():
    aligner = GpsVoYawAligner({
        'min_fix_type': 3,
        'min_num_sv': 4,
        'min_gps_displacement_m': 3.0,
        'min_visual_displacement_m': 2.0,
    })
    lat0 = 39.0
    lon0 = 116.0

    lat, lon = _gps_from_local(lat0, lon0, 0.0, 0.0)
    first = aligner.update(
        gps_position={'lat': lat, 'lon': lon, 'fix_type': 3, 'num_sv': 4},
        visual_pose={'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0},
        timestamp_ns=1_000_000_000,
    )
    lat, lon = _gps_from_local(lat0, lon0, 0.0, 4.0)
    second = aligner.update(
        gps_position={'lat': lat, 'lon': lon, 'fix_type': 3, 'num_sv': 4},
        visual_pose={'x': 4.0, 'y': 0.0, 'yaw_deg': 0.0},
        timestamp_ns=2_000_000_000,
    )

    assert first['accepted'] is False
    assert second['accepted'] is True
    assert second['status'] == 'aligned'
    assert abs(second['yaw_offset_deg'] - 90.0) < 0.1


def test_yaw_alignment_rejects_low_satellite_count():
    aligner = GpsVoYawAligner({'min_num_sv': 4})

    result = aligner.update(
        gps_position={'lat': 39.0, 'lon': 116.0, 'fix_type': 3, 'num_sv': 3},
        visual_pose={'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0},
        timestamp_ns=1,
    )

    assert result['accepted'] is False
    assert result['status'] == 'waiting'
    assert 'satellite count' in result['message']


def test_yaw_alignment_waits_for_enough_displacement():
    aligner = GpsVoYawAligner({
        'min_fix_type': 3,
        'min_num_sv': 4,
        'min_gps_displacement_m': 3.0,
        'min_visual_displacement_m': 2.0,
    })
    lat0 = 39.0
    lon0 = 116.0

    lat, lon = _gps_from_local(lat0, lon0, 0.0, 0.0)
    aligner.update(
        gps_position={'lat': lat, 'lon': lon, 'fix_type': 3, 'num_sv': 4},
        visual_pose={'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0},
        timestamp_ns=1,
    )
    lat, lon = _gps_from_local(lat0, lon0, 1.0, 0.0)
    result = aligner.update(
        gps_position={'lat': lat, 'lon': lon, 'fix_type': 3, 'num_sv': 4},
        visual_pose={'x': 1.0, 'y': 0.0, 'yaw_deg': 0.0},
        timestamp_ns=2,
    )

    assert result['accepted'] is False
    assert result['status'] == 'collecting'
    assert 'enough gps/visual displacement' in result['message']


def test_yaw_alignment_rejects_gps_jump():
    aligner = GpsVoYawAligner({
        'min_fix_type': 3,
        'min_num_sv': 4,
        'gps_jump_reject_m': 10.0,
    })
    lat0 = 39.0
    lon0 = 116.0

    lat, lon = _gps_from_local(lat0, lon0, 0.0, 0.0)
    aligner.update(
        gps_position={'lat': lat, 'lon': lon, 'fix_type': 3, 'num_sv': 4},
        visual_pose={'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0},
        timestamp_ns=1,
    )
    lat, lon = _gps_from_local(lat0, lon0, 20.0, 0.0)
    result = aligner.update(
        gps_position={'lat': lat, 'lon': lon, 'fix_type': 3, 'num_sv': 4},
        visual_pose={'x': 1.0, 'y': 0.0, 'yaw_deg': 0.0},
        timestamp_ns=2,
    )

    assert result['accepted'] is False
    assert result['status'] == 'rejected'
    assert 'gps jumped' in result['message']
