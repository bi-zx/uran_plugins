from uran_localization_plugin.gps_status import GpsStatusTracker


def test_gps_status_unknown_before_first_message():
    tracker = GpsStatusTracker()

    result = tracker.snapshot(now_ns=1_000_000_000)

    assert result['available'] is False
    assert result['state'] == 'unknown'
    assert result['message'] == 'gps status has not been received'


def test_gps_status_reports_disconnected():
    tracker = GpsStatusTracker()

    result = tracker.update_from_json(
        '{"connected":false,"stale":true,"last_error":"no serial port matched"}',
        received_timestamp_ns=1_000_000_000,
    )

    assert result['available'] is False
    assert result['state'] == 'disconnected'
    assert result['message'] == 'no serial port matched'


def test_gps_status_reports_no_signal_when_connected_without_satellites():
    tracker = GpsStatusTracker()

    result = tracker.update_from_json(
        '{"connected":true,"stale":false,"valid_fix":false,"fix_type":0,"num_sv":0}',
        received_timestamp_ns=1_000_000_000,
    )

    assert result['available'] is False
    assert result['state'] == 'no_signal'


def test_gps_status_reports_no_fix_when_satellites_exist_but_fix_is_invalid():
    tracker = GpsStatusTracker()

    result = tracker.update_from_json(
        '{"connected":true,"stale":false,"valid_fix":false,"fix_type":1,"num_sv":6}',
        received_timestamp_ns=1_000_000_000,
    )

    assert result['available'] is False
    assert result['state'] == 'no_fix'


def test_gps_status_reports_valid_fix():
    tracker = GpsStatusTracker()

    result = tracker.update_from_json(
        (
            '{"connected":true,"stale":false,"valid_fix":true,'
            '"fix_type":3,"num_sv":12,"lat":39.0,"lon":116.0}'
        ),
        received_timestamp_ns=1_000_000_000,
    )

    assert result['available'] is True
    assert result['state'] == 'valid_fix'
    assert result['lat'] == 39.0
    assert result['lon'] == 116.0


def test_gps_status_reports_stale_status_topic():
    tracker = GpsStatusTracker({'status_stale_timeout_s': 2.0})
    tracker.update_from_json(
        '{"connected":true,"stale":false,"valid_fix":true,"fix_type":3,"num_sv":12}',
        received_timestamp_ns=1_000_000_000,
    )

    result = tracker.snapshot(now_ns=4_000_000_001)

    assert result['available'] is False
    assert result['state'] == 'status_stale'


def test_gps_status_handles_invalid_json():
    tracker = GpsStatusTracker()

    result = tracker.update_from_json('{bad json', received_timestamp_ns=1_000_000_000)

    assert result['available'] is False
    assert result['state'] == 'invalid_status'
    assert result['parse_error']
