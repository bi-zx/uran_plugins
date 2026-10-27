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


def test_timestamp_accepts_integral_json_number_and_exponent_form():
    value = 1_791_302_932_204_183_552
    decoded = CloudCodec().decode({"msg_type": "state_query", "timestamp_ns": float(value)})
    assert decoded["payload"]["timestamp_ns"] == int(float(value))
    decoded = CloudCodec().decode({"msg_type": "state_query", "timestamp_ns": "1.7913029322041836e18"})
    assert decoded["payload"]["timestamp_ns"] == 1_791_302_932_204_183_552


def test_media_signal_accepts_direct_signal_object():
    decoded = CloudCodec().decode({
        "msg_type": "media_ctrl", "action": "switch", "protocol": "webrtc",
        "channel_id": "realsense_left",
        "signal": {"type": "candidate", "sdpMLineIndex": 1,
                    "sdpMid": "1", "candidate": "candidate:test"},
    })
    assert decoded["payload"]["signal"]["sdpMLineIndex"] == 1


def test_uplink_keeps_legacy_payload_json_for_existing_cloud():
    encoded = CloudCodec.encode_uplink(
        device_id="dog-1", source="motion", data_type="move_result",
        timestamp_ns=42, payload={"success": True})
    assert encoded["payload"] == {"success": True}
    assert encoded["payload_json"] == '{"success":true}'


def test_move_cmd_routes_arm_velocity_and_toggle():
    from uran_interfaces import ContractValidator
    codec = CloudCodec()
    for action, kind in (("", "arm_cartesian_velocity"), ("toggle", "gripper_action")):
        message = codec.decode_public(
            {"msg_type": "move_cmd", "component_id": "arm", "linear_vel_z": 0.02,
             "target_pitch": None, "action": action}, operation_id="arm-test", sequence=1,
            default_device_id="dog", default_component_id="base", received_at_ns=100)
        assert message["payload"]["payload"]["kind"] == kind
        ContractValidator().validate("motion/command.schema.json", message["payload"])
    base = codec.decode_public({"msg_type": "move_cmd", "component_id": "base"},
        operation_id="base", sequence=1, default_device_id="dog", default_component_id="base")
    assert base["payload"]["payload"]["kind"] == "base_velocity"


@pytest.mark.parametrize('timestamp', [1013907328, -1208728896, '1013907328'])
def test_legacy_move_truncated_timestamp_uses_receipt_time(timestamp):
    receipt = 1791345957119161344
    result = CloudCodec().decode_public(
        {'msg_type': 'move_cmd', 'component_id': 'arm', 'timestamp_ns': timestamp},
        operation_id='test', sequence=1, default_device_id='dog',
        default_component_id='base', received_at_ns=receipt)
    # A deliberately arbitrary low word is reconstructed to the nearest
    # representable timestamp; it must be in the same 32-bit epoch window.
    assert abs(result['payload']['issued_at_ns'] - receipt) <= 0x80000000
    assert result['payload']['valid_until_ns'] == result['payload']['issued_at_ns'] + 1000000000


def test_real_old_unix_ns_is_not_refreshed():
    old = 1700000000000000000
    result = CloudCodec().decode_public(
        {'msg_type': 'move_cmd', 'timestamp_ns': old}, operation_id='old', sequence=1,
        default_device_id='dog', default_component_id='base', received_at_ns=1791345957119161344)
    assert result['payload']['issued_at_ns'] == old
