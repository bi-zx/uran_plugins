"""realsense_lifecycle.py — RealSense 生命周期管理

完整流程：
  1. 检查 lifecycle service 是否存在（节点是否已运行）
  2. 若不存在，自动 ros2 launch 启动节点，等待 service 就绪
  3. configure → activate

多个 channel 共享同一 lifecycle 节点时使用引用计数，避免重复激活/停止。
"""

import subprocess
import time


class RealSenseLifecycleAdapter:
    TRANSITION_CONFIGURE = 1
    TRANSITION_CLEANUP = 2
    TRANSITION_ACTIVATE = 3
    TRANSITION_DEACTIVATE = 4

    def __init__(self, node, lifecycle_node_name: str,
                 launch_pkg: str = '', launch_file: str = '',
                 keep_active_after_release: bool = False):
        self._node = node
        self._name = lifecycle_node_name
        self._launch_pkg = launch_pkg
        self._launch_file = launch_file
        self._keep_active_after_release = bool(keep_active_after_release)
        self._ref_count = 0
        self._client = None
        self._ChangeState = None
        self._available = False
        self._launch_proc = None
        self._init_client()

    def _init_client(self):
        try:
            from lifecycle_msgs.srv import ChangeState
            svc = f'{self._name}/change_state'
            self._client = self._node.create_client(ChangeState, svc)
            self._ChangeState = ChangeState
            self._available = True
            self._node.get_logger().info(f'[RealSense] lifecycle client: {svc}')
        except Exception as e:
            self._node.get_logger().warning(
                f'[RealSense] lifecycle_msgs unavailable ({e})'
            )

    def _launch_node(self) -> bool:
        """自动启动 RealSense 节点（ros2 launch）。"""
        if not self._launch_pkg or not self._launch_file:
            self._node.get_logger().warning(
                '[RealSense] launch_pkg/launch_file not configured, cannot auto-launch'
            )
            return False

        if self._launch_proc and self._launch_proc.poll() is None:
            return True  # 已在运行

        cmd = ['ros2', 'launch', self._launch_pkg, self._launch_file]
        self._node.get_logger().info(f'[RealSense] Launching: {" ".join(cmd)}')
        try:
            self._launch_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            return True
        except Exception as e:
            self._node.get_logger().error(f'[RealSense] Launch failed: {e}')
            return False

    def activate(self) -> bool:
        self._ref_count += 1
        if self._ref_count > 1:
            return True

        if not self._available:
            return False

        # 先尝试连接 service，若不存在则自动 launch
        if not self._client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().info(
                f'[RealSense] {self._name} not running, attempting auto-launch...'
            )
            if not self._launch_node():
                self._node.get_logger().warning(
                    '[RealSense] Auto-launch failed'
                )
                self._ref_count = 0
                return False

            # 等待 launch 后 service 就绪
            if not self._client.wait_for_service(timeout_sec=15.0):
                self._node.get_logger().warning(
                    f'[RealSense] {self._name}/change_state still not available '
                    'after launch'
                )
                self._ref_count = 0
                return False

        # configure（若已 configured/active 会返回失败，忽略）
        self._transition(self.TRANSITION_CONFIGURE)
        # activate
        ok = self._transition(self.TRANSITION_ACTIVATE)
        if ok:
            self._node.get_logger().info(f'[RealSense] {self._name} activated')
        else:
            self._node.get_logger().warning(
                f'[RealSense] activate failed'
            )
            self._ref_count = 0
        return ok

    def deactivate(self):
        self._ref_count = max(0, self._ref_count - 1)
        if self._ref_count > 0:
            return
        if not self._available or self._client is None:
            return
        if self._keep_active_after_release:
            self._node.get_logger().info(
                '[RealSense] kept active for non-media consumers'
            )
            return
        self._transition(self.TRANSITION_DEACTIVATE)
        self._node.get_logger().info(f'[RealSense] {self._name} deactivated')

    def shutdown(self):
        """终止 launch 进程（节点销毁时调用）。"""
        if self._launch_proc and self._launch_proc.poll() is None:
            self._launch_proc.terminate()
            self._node.get_logger().info('[RealSense] Launch process terminated')

    def _transition(self, transition_id: int) -> bool:
        try:
            req = self._ChangeState.Request()
            req.transition.id = transition_id
            future = self._client.call_async(req)
            deadline = time.time() + 10.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if future.done():
                return future.result().success
            self._node.get_logger().warning(
                f'[RealSense] transition {transition_id} timeout'
            )
            return False
        except Exception as e:
            self._node.get_logger().error(f'[RealSense] transition error: {e}')
            return False
