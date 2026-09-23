import pytest

from uran_mqtt_plugin.codec import CloudCodec, DecodeError


def test_timestamp_ms_and_embedded_task_type_are_migrated():
    decoded = CloudCodec().decode({
        "msg_type": "task_ctrl",
        "task_id": "t1",
        "task_type": "default_task",
        "timestamp_ms": "12",
        "task_params_json": {"task_type": "mission_planner_route"},
    })
    assert decoded["payload"]["timestamp_ns"] == 12_000_000
    assert decoded["payload"]["task_type"] == "mission_planner_route"


def test_invalid_numbers_are_rejected_instead_of_becoming_motion():
    with pytest.raises(DecodeError, match="linear_vel_x"):
        CloudCodec().decode({"msg_type": "move_cmd", "linear_vel_x": "fast"})

    with pytest.raises(DecodeError, match="linear_vel_x"):
        CloudCodec().decode({"msg_type": "move_cmd", "linear_vel_x": "nan"})


def test_nanosecond_timestamp_keeps_integer_precision():
    value = 9_223_372_036_854_775_001
    decoded = CloudCodec().decode({"msg_type": "state_query", "timestamp_ns": str(value)})
    assert decoded["payload"]["timestamp_ns"] == value

    with pytest.raises(DecodeError, match="timestamp_ns"):
        CloudCodec().decode({"msg_type": "state_query", "timestamp_ns": "1.5"})


def test_uplink_keeps_legacy_payload_json_for_existing_cloud():
    encoded = CloudCodec.encode_uplink(
        device_id="dog-1", source="motion", data_type="move_result",
        timestamp_ns=42, payload={"success": True})
    assert encoded["payload"] == {"success": True}
    assert encoded["payload_json"] == '{"success":true}'
