import threading

from uran_mqtt_plugin.codec import CloudCodec
from uran_mqtt_plugin.worker import MqttWorker


def test_legacy_media_ctrl_defaults_to_media_component():
    worker = object.__new__(MqttWorker)
    worker.codec = CloudCodec()
    worker.config = {}
    worker._commands_enabled_at = 0.0
    worker._sequence = 0
    worker._sequence_lock = threading.Lock()
    worker.default_device_id = "device_001"
    worker.default_component_id = "base"
    worker.media_component_id = "media"
    worker.validity_window_ns = 1_000_000_000
    worker._publish_legacy = lambda payload: True
    received = []
    worker.emit_downlink = lambda message_type, payload, correlation_id, target: received.append(
        (message_type, payload, correlation_id, target))

    worker._on_downlink({
        "msg_type": "media_ctrl",
        "msg_version": "1.0",
        "device_id": "device_001",
        "action": "start",
        "protocol": "webrtc",
        "channel_id": "cyberdog_main",
        "signal_json": '{"type":"offer","sdp":"v=0"}',
    })

    assert len(received) == 1
    assert received[0][0] == "uran.media.session-command.v1"
    assert received[0][1]["target"]["component_id"] == "media"
    assert received[0][3]["component_id"] == "media"
