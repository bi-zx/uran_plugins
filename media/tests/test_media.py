import asyncio

import pytest

from uran_media_plugin.manager import MediaManager, MemoryMediaBackend
from uran_media_plugin import webrtc_channel
from uran_media_plugin.webrtc_channel import WebRTCChannel


def test_missing_backend_is_an_explicit_failure():
    result = MediaManager({}).start(channel_id="front", protocol="webrtc", config={})
    assert result["success"] is False
    assert result["code"] == "E_BACKEND_UNAVAILABLE"


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
