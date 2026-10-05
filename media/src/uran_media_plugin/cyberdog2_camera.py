"""cyberdog2_camera.py — CyberDog2 摄像头适配器

通过 protocol/srv/CameraService 激活摄像头并管理生命周期。
若 protocol 包不可用（非 CyberDog2 环境），激活明确失败。
"""

import time


# CameraService 命令常量（与 CameraService.srv 保持一致）
CMD_START_IMAGE_PUBLISH = 9
CMD_STOP_IMAGE_PUBLISH = 10
RESULT_SUCCESS = 0


class CyberDog2CameraAdapter:
    """管理 CyberDog2 camera_service 的激活与停止。"""

    def __init__(self, node, service_name: str, width: int, height: int, fps: int):
        self._node = node
        self._service_name = service_name
        self._width = width
        self._height = height
        self._fps = fps
        self._client = None
        self._available = False

        self._init_client()

    def _init_client(self):
        """尝试导入 protocol 包并创建服务客户端。"""
        try:
            from protocol.srv import CameraService
            self._CameraService = CameraService
            self._client = self._node.create_client(CameraService, self._service_name)
            self._available = True
            self._node.get_logger().info(
                f'[CyberDog2Camera] camera_service client created: {self._service_name}'
            )
        except (ImportError, Exception) as e:
            self._node.get_logger().warning(
                f'[CyberDog2Camera] protocol package not available ({e}), '
                'camera activation is unavailable'
            )
            self._available = False

    def activate(self) -> bool:
        """激活摄像头：调用 START_IMAGE_PUBLISH，返回是否成功。

        不再预先调用 STOP —— camera_service 可能被 image_transmission 占用，
        STOP 超时会白白浪费 5 秒。若 START 也超时（摄像头已由其他节点激活），
        降级为 WARN 并返回 True，让调用方直接订阅 topic。
        """
        if not self._available:
            self._node.get_logger().info(
                '[CyberDog2Camera] Skipping activation (protocol not available)'
            )
            # image_transmission owns the CyberDog2 camera lifecycle in bridge
            # mode.  The media channel must still be registered so signaling
            # can reach that node when the service is absent from this worker.
            return True

        if not self._client.wait_for_service(timeout_sec=3.0):
            self._node.get_logger().warning(
                f'[CyberDog2Camera] Service {self._service_name} not available, '
                'will try to subscribe topic directly'
            )
            return True

        result = self._call_service(CMD_START_IMAGE_PUBLISH)
        if result and result.result == RESULT_SUCCESS:
            self._node.get_logger().info('[CyberDog2Camera] Camera activated successfully')
            return True
        else:
            code = result.result if result else -1
            self._node.get_logger().warning(
                f'[CyberDog2Camera] START_IMAGE_PUBLISH result={code} '
                '(camera may already be active via image_transmission), proceeding anyway'
            )
            return True

    def deactivate(self):
        """停止摄像头图像发布。"""
        if not self._available or self._client is None:
            return
        self._call_service(CMD_STOP_IMAGE_PUBLISH)
        self._node.get_logger().info('[CyberDog2Camera] Camera deactivated')

    def _call_service(self, command: int):
        """同步调用 camera_service，返回 response 或 None。"""
        try:
            req = self._CameraService.Request()
            req.command = command
            req.width = self._width
            req.height = self._height
            req.fps = self._fps

            future = self._client.call_async(req)
            deadline = time.time() + 5.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)

            if future.done():
                return future.result()
            else:
                self._node.get_logger().warning(
                    f'[CyberDog2Camera] Service call timeout (command={command})'
                )
                return None
        except Exception as e:
            self._node.get_logger().error(
                f'[CyberDog2Camera] Service call error (command={command}): {e}'
            )
            return None
