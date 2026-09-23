"""webrtc_channel.py — 单路 WebRTC PeerConnection 封装（aiortc）

每个 channel_id 对应一个 WebRTCChannel 实例。
信令上报通过 on_signal_cb 回调传递给主节点。
"""

import asyncio
import json
import queue
import time
from typing import Callable, Optional

try:
    import av
    from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
    from aiortc.contrib.media import MediaStreamTrack
    _AIORTC_AVAILABLE = True
except ImportError:
    _AIORTC_AVAILABLE = False


class _RosVideoTrack:
    """从 ROS Image 帧队列读取帧的 VideoStreamTrack 子类（aiortc）。"""

    if _AIORTC_AVAILABLE:
        class _Track(MediaStreamTrack):
            kind = 'video'

            def __init__(self, frame_queue: queue.Queue):
                super().__init__()
                self._queue = frame_queue
                self._pts = 0

            async def recv(self):
                # 等待帧，最多 1 秒
                deadline = asyncio.get_event_loop().time() + 1.0
                while True:
                    try:
                        frame_data = self._queue.get_nowait()
                        frame = av.VideoFrame.from_ndarray(frame_data, format='bgr24')
                        frame.pts = self._pts
                        frame.time_base = '1/90000'
                        self._pts += 3000  # ~30fps
                        return frame
                    except queue.Empty:
                        if asyncio.get_event_loop().time() > deadline:
                            # 返回黑帧避免超时
                            frame = av.VideoFrame(width=640, height=480, format='bgr24')
                            frame.pts = self._pts
                            frame.time_base = '1/90000'
                            self._pts += 3000
                            return frame
                        await asyncio.sleep(0.01)


class WebRTCChannel:
    """单路 WebRTC 通道，封装 aiortc PeerConnection。"""

    def __init__(self, channel_id: str, stun_server: str, on_signal_cb: Callable,
                 on_closed_cb: Optional[Callable] = None,
                 turn_servers: Optional[list] = None):
        self._channel_id = channel_id
        self._stun_server = stun_server
        self._turn_servers = turn_servers or []
        self._on_signal_cb = on_signal_cb
        self._on_closed_cb = on_closed_cb
        self._pc: Optional[object] = None
        self._frame_queue: queue.Queue = queue.Queue(maxsize=10)
        self._track = None
        self._available = _AIORTC_AVAILABLE

        if not self._available:
            import logging
            logging.getLogger(__name__).warning('aiortc is unavailable')

    async def start(self) -> str:
        """创建 PeerConnection，添加 VideoTrack，生成 SDP Offer，返回 offer JSON。"""
        if not self._available:
            raise RuntimeError('aiortc/av backend is not installed')

        ice_servers = [RTCIceServer(urls=self._stun_server)]
        for t in self._turn_servers:
            ice_servers.append(RTCIceServer(
                urls=t['url'],
                username=t.get('username'),
                credential=t.get('credential'),
            ))
        self._pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=ice_servers))

        self._track = _RosVideoTrack._Track(self._frame_queue)
        self._pc.addTrack(self._track)

        # 连接状态变化回调（对端断开时触发清理）
        @self._pc.on('connectionstatechange')
        async def on_state_change():
            state = self._pc.connectionState if self._pc else None
            if state in ('failed', 'closed', 'disconnected') and self._on_closed_cb:
                self._on_closed_cb(self._channel_id)

        # ICE candidate 收集回调
        @self._pc.on('icecandidate')
        def on_ice(candidate):
            if candidate:
                self._on_signal_cb(self._channel_id, {
                    'type': 'candidate',
                    'candidate': candidate.to_sdp(),
                    'sdpMid': candidate.sdpMid,
                    'sdpMLineIndex': candidate.sdpMLineIndex,
                })

        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)

        return json.dumps({
            'type': self._pc.localDescription.type,
            'sdp': self._pc.localDescription.sdp,
        })

    async def set_answer(self, answer_json: str):
        """设置远端 SDP Answer。"""
        if not self._available:
            raise RuntimeError('aiortc/av backend is not installed')
        if self._pc is None:
            raise RuntimeError('WebRTC channel is not started')
        try:
            data = json.loads(answer_json)
            answer = RTCSessionDescription(sdp=data['sdp'], type=data['type'])
            await self._pc.setRemoteDescription(answer)
        except Exception as e:
            import logging
            logging.getLogger(__name__).error(f'[WebRTC:{self._channel_id}] set_answer error: {e}')

    async def add_ice_candidate(self, candidate_json: str):
        """添加 ICE Candidate。"""
        if not self._available:
            raise RuntimeError('aiortc/av backend is not installed')
        if self._pc is None:
            raise RuntimeError('WebRTC channel is not started')
        try:
            data = json.loads(candidate_json)
            candidate = RTCIceCandidate(
                sdp=data.get('candidate', ''),
                sdpMid=data.get('sdpMid'),
                sdpMLineIndex=data.get('sdpMLineIndex'),
            )
            await self._pc.addIceCandidate(candidate)
        except Exception as e:
            import logging
            logging.getLogger(__name__).error(
                f'[WebRTC:{self._channel_id}] add_ice_candidate error: {e}'
            )

    def push_frame(self, numpy_frame):
        """将 numpy BGR 帧放入队列（线程安全）。"""
        try:
            self._frame_queue.put_nowait(numpy_frame)
        except queue.Full:
            # 丢弃最旧帧，放入新帧
            try:
                self._frame_queue.get_nowait()
                self._frame_queue.put_nowait(numpy_frame)
            except queue.Empty:
                pass

    async def stop(self):
        """关闭 PeerConnection。"""
        if self._pc is not None:
            await self._pc.close()
            self._pc = None
