"""Small GStreamer 1.14 WebRTC backend for ROS image topics.

This intentionally uses the WebRTC stack already shipped on CyberDog2.  It
avoids requiring an ARM/Python wheel for aiortc and PyAV.
"""
import json
import threading


class GstWebRtcBackend:
    def __init__(self, config, on_signal=None):
        import gi
        gi.require_version("Gst", "1.0")
        gi.require_version("GstSdp", "1.0")
        gi.require_version("GstWebRTC", "1.0")
        from gi.repository import Gst, GstSdp, GstWebRTC
        Gst.init(None)
        self.Gst, self.GstSdp, self.GstWebRTC = Gst, GstSdp, GstWebRTC
        self.config = dict(config or {})
        self.on_signal = on_signal or (lambda *_: None)
        self.channels = {}
        self.lock = threading.RLock()

    def start(self, channel_id, config):
        with self.lock:
            if channel_id in self.channels:
                raise RuntimeError("WebRTC channel is already active: " + channel_id)
            cfg = dict(self.config)
            cfg.update(config or {})
            width, height = int(cfg.get("width", 640)), int(cfg.get("height", 480))
            fps = max(1, int(cfg.get("fps", 15)))
            pipeline = self.Gst.parse_launch(
                "appsrc name=src is-live=true format=time do-timestamp=true "
                "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1 ! "
                "videoconvert ! queue ! vp8enc deadline=1 cpu-used=8 ! "
                "rtpvp8pay pt=96 ! application/x-rtp,media=video,encoding-name=VP8,"
                "payload=96,clock-rate=90000 ! webrtcbin name=webrtc bundle-policy=max-bundle" %
                (width, height, fps))
            src = pipeline.get_by_name("src")
            webrtc = pipeline.get_by_name("webrtc")
            if cfg.get("stun_server"):
                webrtc.set_property("stun-server", str(cfg["stun_server"]))
            turns = list(cfg.get("turn_servers") or [])
            if turns and turns[0].get("url") and "your-turn-server" not in str(turns[0]["url"]):
                turn = turns[0]
                url = str(turn["url"])
                if url.startswith("turn:"):
                    url = "turn://" + url[len("turn:"):]
                if turn.get("username"):
                    scheme, rest = url.split("://", 1)
                    url = "%s://%s:%s@%s" % (scheme, turn["username"],
                                               turn.get("credential", ""), rest)
                webrtc.set_property("turn-server", url)
            webrtc.connect("on-ice-candidate", self._on_ice, channel_id)
            pipeline.set_state(self.Gst.State.PLAYING)
            self.channels[channel_id] = {"pipeline": pipeline, "src": src,
                                         "webrtc": webrtc, "width": width,
                                         "height": height, "fps": fps,
                                         "pending": []}
            return {"channel_id": channel_id, "transport": "gstreamer-webrtc",
                    "simulated": False}

    def signal(self, channel_id, signal):
        with self.lock:
            item = self.channels.get(channel_id)
            if item is None:
                raise KeyError("channel is not active: " + channel_id)
            kind = str(signal.get("type") or ("offer" if signal.get("sdp") else ""))
            if kind == "offer":
                self._set_offer(item, channel_id, signal["sdp"])
            elif kind == "candidate":
                item["webrtc"].emit("add-ice-candidate",
                                    int(signal.get("sdpMLineIndex", 0)),
                                    str(signal.get("candidate", "")))
            else:
                raise ValueError("unsupported WebRTC signal: " + kind)
            return {"channel_id": channel_id, "accepted": kind,
                    "transport": "gstreamer-webrtc", "simulated": False}

    def _set_offer(self, item, channel_id, text):
        result, message = self.GstSdp.sdp_message_new()
        result = self.GstSdp.sdp_message_parse_buffer(
            str(text).encode("utf-8"), message)
        if result != self.GstSdp.SDPResult.OK:
            raise ValueError("invalid SDP offer")
        offer = self.GstWebRTC.WebRTCSessionDescription.new(
            self.GstWebRTC.WebRTCSDPType.OFFER, message)
        item["webrtc"].emit("set-remote-description", offer, None)
        promise = self.Gst.Promise.new()
        item["webrtc"].emit("create-answer", None, promise)
        promise.wait()
        self._answer_ready(promise, (item, channel_id))

    def _answer_ready(self, promise, user_data):
        item, channel_id = user_data
        reply = promise.get_reply()
        answer = reply.get_value("answer")
        item["webrtc"].emit("set-local-description", answer, None)
        self.on_signal(channel_id, {"type": "answer", "sdp": answer.sdp.as_text()})

    def _on_ice(self, _element, mline, candidate, channel_id):
        # GStreamer reports the media-line index; browsers require the
        # corresponding sdpMid as well.  They are not always both "0" when
        # the remote offer contains an audio section.
        self.on_signal(channel_id, {"type": "candidate", "sdpMLineIndex": int(mline),
                                    "sdpMid": str(mline), "candidate": str(candidate)})

    def push_frame(self, channel_id, frame):
        with self.lock:
            item = self.channels.get(channel_id)
            if item is None:
                return
            try:
                import numpy as np
                frame = np.ascontiguousarray(frame, dtype=np.uint8)
                data = frame.tobytes()
                buffer = self.Gst.Buffer.new_allocate(None, len(data), None)
                buffer.fill(0, data)
                duration = self.Gst.util_uint64_scale_int(1, self.Gst.SECOND, item["fps"])
                buffer.duration = duration
                item["src"].emit("push-buffer", buffer)
            except Exception:
                return

    def stop(self, channel_id):
        with self.lock:
            item = self.channels.pop(channel_id, None)
            if item:
                item["src"].emit("end-of-stream")
                item["pipeline"].set_state(self.Gst.State.NULL)

    def close(self):
        for channel_id in list(self.channels):
            self.stop(channel_id)
