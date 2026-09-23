"""rtsp_server.py — RTSP Server 封装

使用 GStreamer appsrc pipeline 推送帧到 RTSP Server。
所有通道共享同一个 GstRtspServer 实例（同一端口），每个 channel 独立 mount point。
通过 media-configure 信号在客户端连接时获取 appsrc 引用。
若 GStreamer 不可用，启动明确失败。
"""

import logging
import threading
from typing import Dict

logger = logging.getLogger(__name__)

try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstRtspServer', '1.0')
    from gi.repository import Gst, GstRtspServer, GLib
    Gst.init(None)
    _GST_AVAILABLE = True
except Exception:
    _GST_AVAILABLE = False


class _ChannelState:
    def __init__(self, channel_id: str, url: str, fps: int):
        self.channel_id = channel_id
        self.url = url
        self.fps = fps
        self.appsrc = None
        self.pts = 0
        self.frame_duration = (
            Gst.util_uint64_scale_int(1, Gst.SECOND, fps) if _GST_AVAILABLE else 0
        )


class RTSPServer:
    """多通道 RTSP Server，所有 channel 共享同一端口，各自独立 mount point。"""

    def __init__(self, port: int = 8554):
        self._channels: Dict[str, _ChannelState] = {}
        self._factories: Dict[str, object] = {}
        self._port = port
        self._server = None
        self._mounts = None
        self._loop: object = None
        self._loop_thread: threading.Thread = None
        self._available = _GST_AVAILABLE

        if not self._available:
            logger.warning('GStreamer RTSP backend is unavailable')

    def _ensure_server(self):
        """确保共享的 GstRtspServer 和 GLib 主循环已启动。"""
        if self._server is not None:
            return

        self._loop = GLib.MainLoop()
        self._loop_thread = threading.Thread(target=self._loop.run, daemon=True)
        self._loop_thread.start()

        self._server = GstRtspServer.RTSPServer()
        self._server.set_service(str(self._port))
        self._mounts = self._server.get_mount_points()
        self._server.attach(None)
        logger.info(f'[RTSP] Server started on port {self._port}')

    def start(self, channel_id: str, width: int = 640, height: int = 480,
              fps: int = 30, port: int = 8554) -> str:
        """为 channel 添加 mount point，返回 rtsp URL。"""
        url = f'rtsp://localhost:{self._port}/{channel_id}'

        if not self._available:
            raise RuntimeError('GStreamer/GstRtspServer backend is not installed')

        if channel_id in self._channels:
            return self._channels[channel_id].url

        try:
            self._ensure_server()

            factory = GstRtspServer.RTSPMediaFactory()
            pipeline_str = (
                f'( appsrc name=src is-live=true block=false do-timestamp=true format=time '
                f'caps=video/x-raw,format=BGR,width={width},height={height},'
                f'framerate={fps}/1 ! '
                f'videoconvert ! video/x-raw,format=I420 ! '
                f'x264enc tune=zerolatency speed-preset=ultrafast '
                f'key-int-max={fps} ! '
                f'rtph264pay name=pay0 pt=96 config-interval=1 )'
            )
            factory.set_launch(pipeline_str)
            factory.set_shared(True)

            state = _ChannelState(channel_id, url, fps)
            self._channels[channel_id] = state

            factory.connect('media-configure',
                            lambda fac, media, cid=channel_id: self._on_media_configure(cid, media))

            self._mounts.add_factory(f'/{channel_id}', factory)
            self._factories[channel_id] = factory

            logger.info(f'[RTSP] Mount added: {url}')
        except Exception as e:
            logger.error(f'[RTSP] Failed to start {channel_id}: {e}')
            self._channels.pop(channel_id, None)
            raise RuntimeError(f'RTSP channel start failed: {e}') from e

        return url

    def _on_media_configure(self, channel_id: str, media):
        """客户端首次连接时触发，从 pipeline 中获取 appsrc 元素。"""
        state = self._channels.get(channel_id)
        if state is None:
            return

        pipeline = media.get_element()
        appsrc = pipeline.get_by_name('src')
        if appsrc:
            state.appsrc = appsrc
            state.pts = 0
            logger.info(f'[RTSP] appsrc acquired for channel: {channel_id}')
        else:
            logger.error(f'[RTSP] Failed to get appsrc for channel: {channel_id}')

    def push_frame(self, channel_id: str, numpy_frame):
        """推送 numpy BGR 帧到对应通道的 appsrc。"""
        if not self._available:
            return

        state = self._channels.get(channel_id)
        if state is None or state.appsrc is None:
            return

        try:
            data = numpy_frame.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.pts = state.pts
            buf.duration = state.frame_duration
            state.pts += state.frame_duration

            ret = state.appsrc.emit('push-buffer', buf)
            if ret != Gst.FlowReturn.OK:
                logger.debug(f'[RTSP] push-buffer returned {ret} for {channel_id}')
        except Exception as e:
            logger.debug(f'[RTSP] push_frame error ({channel_id}): {e}')

    def stop(self, channel_id: str):
        """停止对应通道（移除 mount point）。"""
        state = self._channels.pop(channel_id, None)
        if state and state.appsrc:
            try:
                state.appsrc.emit('end-of-stream')
            except Exception:
                pass

        if self._mounts and channel_id in self._factories:
            try:
                self._mounts.remove_factory(f'/{channel_id}')
            except Exception:
                pass
        self._factories.pop(channel_id, None)
        logger.info(f'[RTSP] Stopped channel: {channel_id}')

    def stop_all(self):
        for cid in list(self._channels.keys()):
            self.stop(cid)
