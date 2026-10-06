import threading
from typing import Any, Mapping

from uran_worker_sdk import Worker, WorkerServer
from uran_interfaces import ContractValidator

from .cyberdog2_camera import CyberDog2CameraAdapter
from .manager import (
    AiortcMediaBackend,
    ChannelRoutingWebRtcBackend,
    CyberDogWebRtcBackend,
    MediaManager,
    MemoryMediaBackend,
    RtspMediaBackend,
    SourceManagedBackend,
)
from .realsense_lifecycle import RealSenseLifecycleAdapter


class _CameraSource:
    def __init__(self, adapter):
        self.adapter = adapter
        self.references = 0

    def acquire(self) -> bool:
        if self.references == 0 and not self.adapter.activate():
            return False
        self.references += 1
        return True

    def release(self) -> None:
        if self.references == 0:
            return
        self.references -= 1
        if self.references == 0:
            self.adapter.deactivate()


class _AlwaysReadySource:
    def acquire(self) -> bool:
        return True

    def release(self) -> None:
        pass


class MediaWorker(Worker):
    plugin_id = "uran-media"
    plugin_version = "0.1.0"
    capabilities = ("media.session", "media.webrtc", "media.rtsp")

    def configure(self, config: Mapping[str, Any]) -> None:
        config = dict(config)
        self.validator = ContractValidator()
        self.state_reporting = bool(config.get("state_reporting", True))
        self.channel_metadata = self._channel_metadata(config)
        self._ros = None
        self._closables = []
        self._defaults = {key: dict(config.get(key) or {}) for key in ("webrtc", "rtsp")}
        backends = {}
        if config.get("simulation", False):
            backends = {"webrtc": MemoryMediaBackend(), "rtsp": MemoryMediaBackend()}
        elif str(config.get("backend") or "") == "ros2":
            backends = self._configure_ros2(config)
        self.manager = MediaManager(backends)
        self._register_media_state()
        self._publish_media_state()

    @staticmethod
    def _channel_metadata(config: Mapping[str, Any]):
        """Normalize profile channel declarations to the legacy list format."""
        configured = config.get("channel_metadata") or config.get("video_sources") or []
        if isinstance(configured, Mapping):
            configured = [dict(value, channel_id=key)
                          for key, value in configured.items()
                          if isinstance(value, Mapping)]
        result = []
        for item in configured:
            if not isinstance(item, Mapping) or not item.get("channel_id"):
                continue
            result.append({
                "channel_id": str(item.get("channel_id")),
                "source_type": str(item.get("source_type") or ""),
                "ros_topic": str(item.get("ros_topic") or ""),
                "fps": item.get("fps", 0),
            })
        return result

    def _register_media_state(self):
        if not self.state_reporting:
            return
        for field, value_type in (("media_active_protocol", "string"),
                                   ("media_channel_count", "int"),
                                   ("media_camera_list", "json")):
            try:
                self.register_state(field, value_type, persistent=False)
            except RuntimeError:
                # Standalone SDK tests can configure a worker before binding
                # it to WorkerServer; the Runtime host registers these fields.
                pass

    def _publish_media_state(self):
        if not self.state_reporting:
            return
        active = self.manager.snapshot()
        protocols = [str(item.get("protocol") or "") for item in active.values()]
        protocol = "webrtc" if "webrtc" in protocols else ("rtsp" if "rtsp" in protocols else "none")
        for field, value in (("media_active_protocol", protocol),
                             ("media_channel_count", len(active)),
                             ("media_camera_list", self.channel_metadata)):
            try:
                self.set_state(field, value, persistent=False)
            except RuntimeError:
                pass

    def _configure_ros2(self, config: Mapping[str, Any]):
        try:
            import rclpy
            from cv_bridge import CvBridge
            from rclpy.executors import MultiThreadedExecutor
            from sensor_msgs.msg import Image
        except ImportError as exc:
            raise RuntimeError("ROS2 image runtime is unavailable") from exc

        rclpy.init(args=None)
        node = rclpy.create_node(str(config.get("node_name") or "uran_media_worker"))
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        self._ros = (rclpy, node, executor, thread)

        camera_cfg = dict(config.get("camera") or {})
        source_type = str(camera_cfg.get("type") or "none")
        if source_type == "cyberdog2":
            # The image_transmission bridge owns the CyberDog2 camera lifecycle
            # for WebRTC.  Its START_LIVE_STREAM transition happens after ICE
            # succeeds; calling START_IMAGE_PUBLISH here puts camera_server in
            # the wrong state and prevents video frames from being produced.
            if bool(camera_cfg.get("activate_service", False)):
                adapter = CyberDog2CameraAdapter(
                    node, str(camera_cfg.get("service_name") or "camera_service"),
                    int(camera_cfg.get("width", 640)), int(camera_cfg.get("height", 480)),
                    int(camera_cfg.get("fps", 30)))
                source = _CameraSource(adapter)
            else:
                source = _AlwaysReadySource()
        elif source_type == "realsense":
            adapter = RealSenseLifecycleAdapter(
                node, str(camera_cfg.get("lifecycle_node_name") or "/camera/camera"),
                launch_pkg=str(camera_cfg.get("launch_pkg") or ""),
                launch_file=str(camera_cfg.get("launch_file") or ""),
                keep_active_after_release=bool(camera_cfg.get("keep_active_after_release", False)))
            source = _CameraSource(adapter)
            self._closables.append(adapter)
        elif source_type == "none":
            source = _AlwaysReadySource()
        else:
            raise ValueError(f"unsupported camera source: {source_type}")

        def outgoing_signal(channel_id, signal):
            # Keep the legacy cloud boundary used by uran_ws.  MQTT encodes
            # message_type as data_type, so media_signal must remain the
            # externally visible type and its payload must stay two fields.
            self.publish_uplink({"message_type": "media_signal", "payload": {
                "channel_id": channel_id, "signal": signal,
            }, "correlation_id": channel_id})

        webrtc_cfg = dict(config.get("webrtc") or {})
        if str(webrtc_cfg.get("implementation") or "aiortc") == "cyberdog2":
            bridge_backend = CyberDogWebRtcBackend(node, webrtc_cfg, outgoing_signal)
            aiortc_backend = AiortcMediaBackend(webrtc_cfg, outgoing_signal)
            sources = {item["channel_id"]: item for item in self.channel_metadata}
            webrtc = ChannelRoutingWebRtcBackend(bridge_backend, aiortc_backend, sources)
        else:
            webrtc = AiortcMediaBackend(webrtc_cfg, outgoing_signal)
        rtsp = RtspMediaBackend(dict(config.get("rtsp") or {}))
        self._closables.extend((webrtc, rtsp))

        bridge = CvBridge()
        namespace = str(camera_cfg.get("namespace") or config.get("namespace") or "").rstrip("/")

        def resolve_topic(topic):
            topic = str(topic or "")
            if namespace and topic != namespace and not topic.startswith(namespace + "/"):
                return namespace + (topic if topic.startswith("/") else "/" + topic)
            return topic

        topic_channels = {}
        for item in self.channel_metadata:
            if item.get("source_type") == "camera_service":
                continue
            topic = resolve_topic(item.get("ros_topic"))
            if topic:
                topic_channels.setdefault(topic, []).append(item["channel_id"])
        if not topic_channels:
            topic = resolve_topic(camera_cfg.get("image_topic") or "/camera/color/image_raw")
            if topic:
                topic_channels[topic] = []

        def convert_image(message):
            import numpy as np
            try:
                frame = bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            except Exception:
                frame = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            frame = np.asarray(frame)
            if frame.ndim == 2:
                if frame.dtype != np.uint8:
                    values = frame[np.isfinite(frame)] if np.issubdtype(frame.dtype, np.floating) else frame
                    high = float(values.max()) if values.size else 1.0
                    low = float(values.min()) if values.size else 0.0
                    frame = (np.zeros(frame.shape, dtype=np.uint8) if high <= low else
                             ((frame.astype(np.float32) - low) * 255.0 / (high - low)).clip(0, 255).astype(np.uint8))
                frame = np.repeat(frame[:, :, None], 3, axis=2)
            elif frame.ndim == 3 and frame.shape[2] == 4:
                frame = frame[:, :, :3]
            return np.ascontiguousarray(frame, dtype=np.uint8)

        self._image_subscriptions = []
        for topic, channel_ids in topic_channels.items():
            def on_image(message, ids=tuple(channel_ids), source_topic=topic):
                try:
                    frame = convert_image(message)
                except Exception as exc:
                    node.get_logger().warning(f"image conversion failed on {source_topic}: {exc}")
                    return
                targets = ids or tuple(self.manager.channels.keys())
                for channel_id in targets:
                    push = getattr(webrtc, "push_frame", None)
                    if push:
                        push(channel_id, frame)
                    rtsp.push_frame(frame, channel_id=channel_id)
            self._image_subscriptions.append(node.create_subscription(Image, topic, on_image, 10))
        return {"webrtc": SourceManagedBackend(webrtc, source),
                "rtsp": SourceManagedBackend(rtsp, source)}

    def invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        if request.get("message_type") != "uran.media.session-command.v1":
            raise ValueError("unsupported message_type")
        command = dict(request.get("payload") or {})
        self.validator.validate("media/session-command.schema.json", command)
        for field in ("device_id", "component_id"):
            if self.identity.get(field) and command["target"][field] != self.identity[field]:
                raise ValueError(f"media targets another {field}")
        channel = str(command.get("channel_id") or command.get("session_id") or "")
        action = command.get("action")
        signal = dict(command.get("signal") or {})
        if action == "start":
            result = self.manager.start(channel_id=channel,
                                        protocol=str(command.get("protocol", "")),
                                        config=dict(self._defaults.get(str(command.get("protocol"))) or {}))
            if result.get("success") and signal:
                signal_result = self.manager.signal(channel_id=channel, signal=signal)
                if signal_result.get("success"):
                    result = dict(result)
                    result["signal"] = signal_result
                else:
                    result = signal_result
        elif action == "switch" and signal:
            if channel not in self.manager.channels:
                result = self.manager.start(channel_id=channel,
                                            protocol=str(command.get("protocol", "")),
                                            config=dict(self._defaults.get(str(command.get("protocol"))) or {}))
                if result.get("success"):
                    signal_result = self.manager.signal(channel_id=channel, signal=signal)
                    if signal_result.get("success"):
                        result = dict(result)
                        result["signal"] = signal_result
                    else:
                        result = signal_result
            else:
                result = self.manager.signal(channel_id=channel, signal=signal)
        elif action == "switch":
            self.manager.stop(channel)
            result = self.manager.start(channel_id=channel,
                                        protocol=str(command.get("protocol", "")),
                                        config=dict(self._defaults.get(str(command.get("protocol"))) or {}))
        elif signal:
            # Legacy MQTT/WebSocket clients send SDP/ICE follow-up packets
            # with action="" (the action belongs to the channel lifecycle).
            # Route every signal packet to the already registered channel.
            result = self.manager.signal(channel_id=channel, signal=signal)
        elif action == "stop":
            result = self.manager.stop(channel)
        else:
            result = {"success": False, "code": "E_UNSUPPORTED_MEDIA_ACTION"}
        self._publish_media_state()
        return {"status": "succeeded" if result.get("success") else "failed",
                "operation_id": request["operation_id"], "payload": result}

    def deactivate(self) -> None:
        for channel_id in list(self.manager.channels):
            self.manager.stop(channel_id)
        self._publish_media_state()

    def shutdown(self) -> None:
        for item in reversed(self._closables):
            close = getattr(item, "close", None) or getattr(item, "shutdown", None)
            if close:
                close()
        self._closables = []
        if self._ros is not None:
            rclpy, node, executor, thread = self._ros
            executor.shutdown(timeout_sec=2.0)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            thread.join(timeout=2.0)
            self._ros = None


def worker_factory() -> Worker:
    return MediaWorker()


def main() -> int:
    return WorkerServer(worker_factory()).serve()
