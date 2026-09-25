import asyncio
from dataclasses import dataclass
import json
import threading
from typing import Any, Dict

try:
    from typing import Protocol
except ImportError:  # Python 3.6
    class Protocol(object):
        pass

from .cyberdog2_webrtc_bridge import from_img_trans, make_stop, to_img_trans
from .rtsp_server import RTSPServer
from .webrtc_channel import WebRTCChannel


class MediaBackend(Protocol):
    def start(self, channel_id: str, config: Dict[str, Any]) -> Dict[str, Any]: ...
    def signal(self, channel_id: str, signal: Dict[str, Any]) -> Dict[str, Any]: ...
    def stop(self, channel_id: str) -> None: ...


class MemoryMediaBackend:
    """Stateful CI backend. Responses are marked as simulated."""

    def __init__(self):
        self.channels: Dict[str, Dict[str, Any]] = {}

    def start(self, channel_id: str, config: Dict[str, Any]) -> Dict[str, Any]:
        self.channels[channel_id] = dict(config)
        return {"channel_id": channel_id, "simulated": True}

    def signal(self, channel_id: str, signal: Dict[str, Any]) -> Dict[str, Any]:
        if channel_id not in self.channels:
            raise KeyError(f"channel is not active: {channel_id}")
        return {"channel_id": channel_id, "signal": dict(signal), "simulated": True}

    def stop(self, channel_id: str) -> None:
        self.channels.pop(channel_id, None)


class AiortcMediaBackend:
    """Owns one asyncio loop and a WebRTC peer per media channel."""

    def __init__(self, config: Dict[str, Any], on_signal=None):
        self.config = dict(config)
        self.on_signal = on_signal or (lambda *_: None)
        self.channels: Dict[str, WebRTCChannel] = {}
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()

    def _run(self, coroutine):
        timeout = float(self.config.get("operation_timeout_s", 10.0))
        return asyncio.run_coroutine_threadsafe(coroutine, self.loop).result(timeout=timeout)

    def start(self, channel_id: str, config: Dict[str, Any]) -> Dict[str, Any]:
        if channel_id in self.channels:
            raise RuntimeError(f"WebRTC channel is already active: {channel_id}")
        merged = {**self.config, **config}
        channel = WebRTCChannel(
            channel_id,
            str(merged.get("stun_server") or "stun:stun.l.google.com:19302"),
            self.on_signal,
            on_closed_cb=lambda closed: self.channels.pop(closed, None),
            turn_servers=list(merged.get("turn_servers") or []),
        )
        offer = json.loads(self._run(channel.start()))
        self.channels[channel_id] = channel
        return {"channel_id": channel_id, "offer": offer, "simulated": False}

    def signal(self, channel_id: str, signal: Dict[str, Any]) -> Dict[str, Any]:
        channel = self.channels.get(channel_id)
        if channel is None:
            raise KeyError(f"channel is not active: {channel_id}")
        signal_type = str(signal.get("type") or ("answer" if signal.get("sdp") else ""))
        if signal_type == "answer":
            self._run(channel.set_answer(json.dumps({"type": "answer", "sdp": signal["sdp"]})))
        elif signal_type == "candidate":
            self._run(channel.add_ice_candidate(json.dumps(signal)))
        else:
            raise ValueError(f"unsupported WebRTC signal: {signal_type or '<empty>'}")
        return {"channel_id": channel_id, "accepted": signal_type, "simulated": False}

    def push_frame(self, frame) -> None:
        for channel in list(self.channels.values()):
            channel.push_frame(frame)

    def stop(self, channel_id: str) -> None:
        channel = self.channels.pop(channel_id, None)
        if channel is not None:
            self._run(channel.stop())

    def close(self) -> None:
        for channel_id in list(self.channels):
            self.stop(channel_id)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2.0)
        self.loop.close()


class RtspMediaBackend:
    def __init__(self, config: Dict[str, Any]):
        self.config = dict(config)
        self.server = RTSPServer(port=int(config.get("port", 8554)))

    def start(self, channel_id: str, config: Dict[str, Any]) -> Dict[str, Any]:
        merged = {**self.config, **config}
        url = self.server.start(
            channel_id,
            width=int(merged.get("width", 640)),
            height=int(merged.get("height", 480)),
            fps=int(merged.get("fps", 30)),
        )
        return {"channel_id": channel_id, "url": url, "simulated": False}

    def signal(self, channel_id: str, signal: Dict[str, Any]) -> Dict[str, Any]:
        raise ValueError("RTSP does not accept WebRTC signaling")

    def push_frame(self, frame) -> None:
        for channel_id in list(self.server._channels):
            self.server.push_frame(channel_id, frame)

    def stop(self, channel_id: str) -> None:
        self.server.stop(channel_id)

    def close(self) -> None:
        self.server.stop_all()


class CyberDogWebRtcBackend:
    """Relays signaling through Xiaomi image_transmission ROS topics."""

    def __init__(self, node, config: Dict[str, Any], on_signal=None):
        from std_msgs.msg import String

        self._String = String
        self.on_signal = on_signal or (lambda *_: None)
        self.channels = set()
        self.publisher = node.create_publisher(
            String, str(config.get("signal_in_topic") or "img_trans_signal_in"), 10)
        self.subscription = node.create_subscription(
            String, str(config.get("signal_out_topic") or "img_trans_signal_out"),
            self._on_signal, 10)

    def start(self, channel_id: str, config: Dict[str, Any]) -> Dict[str, Any]:
        self.channels.add(channel_id)
        return {"channel_id": channel_id, "transport": "image_transmission",
                "simulated": False}

    def signal(self, channel_id: str, signal: Dict[str, Any]) -> Dict[str, Any]:
        if channel_id not in self.channels:
            raise KeyError(f"channel is not active: {channel_id}")
        raw = to_img_trans(channel_id, signal)
        if not raw:
            raise ValueError("unsupported image_transmission signal")
        message = self._String()
        message.data = raw
        self.publisher.publish(message)
        return {"channel_id": channel_id, "relayed": True, "simulated": False}

    def _on_signal(self, message) -> None:
        channel_id, signal = from_img_trans(message.data)
        if channel_id in self.channels and signal:
            self.on_signal(channel_id, signal)

    def stop(self, channel_id: str) -> None:
        if channel_id in self.channels:
            message = self._String()
            message.data = make_stop(channel_id)
            self.publisher.publish(message)
            self.channels.discard(channel_id)

    def close(self) -> None:
        for channel_id in list(self.channels):
            self.stop(channel_id)


class SourceManagedBackend:
    """Acquires a physical camera source while a protocol channel is active."""

    def __init__(self, backend, source):
        self.backend = backend
        self.source = source
        self.channels = set()

    def start(self, channel_id: str, config: Dict[str, Any]) -> Dict[str, Any]:
        if not self.source.acquire():
            raise RuntimeError("camera source activation failed")
        try:
            result = self.backend.start(channel_id, config)
        except Exception:
            self.source.release()
            raise
        self.channels.add(channel_id)
        return result

    def signal(self, channel_id: str, signal: Dict[str, Any]) -> Dict[str, Any]:
        return self.backend.signal(channel_id, signal)

    def stop(self, channel_id: str) -> None:
        if channel_id not in self.channels:
            return
        try:
            self.backend.stop(channel_id)
        finally:
            self.channels.discard(channel_id)
            self.source.release()

    def close(self) -> None:
        for channel_id in list(self.channels):
            self.stop(channel_id)
        close = getattr(self.backend, "close", None)
        if close:
            close()


@dataclass
class Channel:
    channel_id: str
    protocol: str
    state: str
    backend_state: Dict[str, Any]


class MediaManager:
    def __init__(self, backends: Dict[str, MediaBackend]):
        self.backends = dict(backends)
        self.channels: Dict[str, Channel] = {}

    def start(self, *, channel_id: str, protocol: str,
              config: Dict[str, Any]) -> Dict[str, Any]:
        if not channel_id:
            return {"success": False, "code": "E_CHANNEL_ID_REQUIRED"}
        backend = self.backends.get(protocol)
        if backend is None:
            return {"success": False, "code": "E_BACKEND_UNAVAILABLE",
                    "message": f"{protocol} backend is not installed"}
        try:
            state = backend.start(channel_id, config)
        except Exception as exc:
            return {"success": False, "code": "E_MEDIA_START_FAILED",
                    "message": str(exc)}
        self.channels[channel_id] = Channel(channel_id, protocol, "active", state)
        return {"success": True, "channel_id": channel_id, "protocol": protocol,
                "backend": state}

    def signal(self, *, channel_id: str, signal: Dict[str, Any]) -> Dict[str, Any]:
        channel = self.channels.get(channel_id)
        if channel is None:
            return {"success": False, "code": "E_CHANNEL_NOT_ACTIVE"}
        try:
            reply = self.backends[channel.protocol].signal(channel_id, signal)
        except Exception as exc:
            return {"success": False, "code": "E_SIGNAL_FAILED", "message": str(exc)}
        return {"success": True, "channel_id": channel_id, "reply": reply}

    def stop(self, channel_id: str) -> Dict[str, Any]:
        channel = self.channels.pop(channel_id, None)
        if channel is None:
            return {"success": False, "code": "E_CHANNEL_NOT_ACTIVE"}
        self.backends[channel.protocol].stop(channel_id)
        return {"success": True, "channel_id": channel_id}

    def snapshot(self) -> Dict[str, Any]:
        return {key: vars(value) for key, value in self.channels.items()}
