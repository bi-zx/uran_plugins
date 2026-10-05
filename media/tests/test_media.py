import asyncio

import pytest

from uran_media_plugin.manager import MediaManager, MemoryMediaBackend
from uran_media_plugin import webrtc_channel
from uran_media_plugin.webrtc_channel import WebRTCChannel


def test_inline_media_config_is_loaded_with_explicit_backend_override():
    from uran_media_plugin.worker import MediaWorker
    worker = MediaWorker()
    worker.configure({"simulation": True, "state_reporting": False, "webrtc": {
        "implementation": "cyberdog2", "stun_server": "stun:example.test:3478",
        "turn_servers": [{"url": "turn:example.test:3478", "username": "test", "credential": "secret"}]}})
    assert worker._defaults["webrtc"]["stun_server"] == "stun:example.test:3478"
    assert worker._defaults["webrtc"]["turn_servers"][0]["credential"] == "secret"
    assert worker._defaults["webrtc"]["implementation"] == "cyberdog2"


def test_missing_backend_is_an_explicit_failure():
    result = MediaManager({}).start(channel_id="front", protocol="webrtc", config={})
    assert result["success"] is False
    assert result["code"] == "E_BACKEND_UNAVAILABLE"


def test_legacy_media_channel_state_contains_configured_channels():
    from uran_media_plugin.worker import MediaWorker
    worker = MediaWorker()
    registered = []
    published = {}
    worker.register_state = lambda field, value_type="any", unit="", persistent=False: registered.append((field, value_type))
    worker.set_state = lambda field, value, persistent=False: published.update({field: value})
    worker.configure({"simulation": True, "channel_metadata": {
        "front_cam": {"source_type": "ros_topic", "ros_topic": "/camera/image_raw", "fps": 30},
        "cyberdog_main": {"source_type": "camera_service", "ros_topic": "image", "fps": 10},
    }})
    assert {item[0] for item in registered} == {
        "media_active_protocol", "media_channel_count", "media_camera_list"}
    assert published["media_active_protocol"] == "none"
    assert published["media_channel_count"] == 0
    assert [item["channel_id"] for item in published["media_camera_list"]] == ["front_cam", "cyberdog_main"]


def test_start_with_offer_forwards_signal_after_channel_registration():
    import time
    from uran_media_plugin.worker import MediaWorker
    worker = MediaWorker()
    worker.identity = {"device_id": "dog", "component_id": "media"}
    worker.configure({"simulation": True, "state_reporting": False})
    now = time.time_ns()
    result = worker.invoke({
        "message_type": "uran.media.session-command.v1",
        "operation_id": "op-1",
        "payload": {
            "schema_version": "1.0", "operation_id": "op-1",
            "target": {"device_id": "dog", "component_id": "media"},
            "session_id": "cyberdog_main", "channel_id": "cyberdog_main",
            "action": "start", "protocol": "webrtc", "timestamp_ns": now,
            "signal": {"type": "offer", "sdp": "v=0"},
        },
    })
    assert result["status"] == "succeeded"
    assert result["payload"]["signal"]["success"] is True
    assert result["payload"]["signal"]["reply"]["signal"]["type"] == "offer"


def test_simulation_backend_is_identified_as_simulation():
    manager = MediaManager({"webrtc": MemoryMediaBackend()})
    result = manager.start(channel_id="front", protocol="webrtc", config={})
    assert result["success"] is True
    assert result["backend"]["simulated"] is True


def test_missing_aiortc_never_returns_empty_sdp_success(monkeypatch):
    monkeypatch.setattr(webrtc_channel, "_AIORTC_AVAILABLE", False)
    channel = WebRTCChannel("front", "stun:example.invalid", lambda *_: None)
    with pytest.raises(RuntimeError, match="not installed"):
        asyncio.run(channel.start())
