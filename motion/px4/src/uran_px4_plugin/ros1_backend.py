from __future__ import annotations

import threading
from typing import Any, Mapping

from .controller import MotionCommand, Px4Controller


class Ros1Px4Backend:
    """Thin ROS 1 Noetic transport; all motion policy stays in controller.py."""

    def __init__(self, config: Mapping[str, Any]):
        try:
            import rospy
            from geometry_msgs.msg import PoseStamped
            from mavros_msgs.msg import RCIn
            from nav_msgs.msg import Odometry
            from quadrotor_msgs.msg import PositionCommand, TakeoffLand
        except ImportError as exc:
            raise RuntimeError("ROS1 PX4 backend needs rospy, mavros_msgs and quadrotor_msgs") from exc
        self.rospy = rospy
        if not rospy.core.is_initialized():
            rospy.init_node(str(config.get("ros_node_name", "uran_px4_worker")),
                            anonymous=bool(config.get("ros_anonymous", True)),
                            disable_signals=True)
        self.PositionCommand, self.TakeoffLand = PositionCommand, TakeoffLand
        self.lock = threading.RLock()
        self.subscribers = []
        self.pub_position = rospy.Publisher(str(config.get("position_topic", "/position_cmd")),
                                             PositionCommand, queue_size=10)
        self.pub_takeoff = rospy.Publisher(str(config.get("takeoff_topic", "/px4ctrl/takeoff_land")),
                                            TakeoffLand, queue_size=5)
        self.controller = Px4Controller(self, clock=lambda: rospy.get_time(),
            linear_limit=float(config.get("linear_limit", 3.0)),
            angular_limit=float(config.get("angular_limit", 1.0)),
            cmd_timeout_s=float(config.get("cmd_timeout_s", 0.5)),
            land_retry_delay_s=float(config.get("land_retry_delay_s", 0.6)))
        self.subscribers.append(rospy.Subscriber(
            str(config.get("rc_topic", "/mavros/rc/in")), RCIn,
            lambda msg: self.controller.update_rc(msg.channels), queue_size=5))
        self.subscribers.append(rospy.Subscriber(
            str(config.get("odom_topic", "/robot/dlio/odom_node/odom")), Odometry,
            self._on_odom, queue_size=20, tcp_nodelay=True))
        self.subscribers.append(rospy.Subscriber(
            str(config.get("traj_trigger_topic", "/traj_start_trigger")), PoseStamped,
            self._on_trigger, queue_size=5))
        keepalive_hz = float(config.get("keepalive_hz", 20.0))
        if keepalive_hz <= 0.0:
            raise ValueError("keepalive_hz must be positive")
        self.timer = rospy.Timer(rospy.Duration(1.0 / keepalive_hz), self._on_tick)

    def _on_tick(self, _event) -> None:
        with self.lock:
            self.controller.tick()

    def _on_odom(self, message) -> None:
        q = message.pose.pose.orientation
        yaw = __import__("math").atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.controller.update_odom(message.pose.pose.position.x, message.pose.pose.position.y,
                                        message.pose.pose.position.z, yaw)

    def _on_trigger(self, message) -> None:
        q = message.pose.orientation
        yaw = __import__("math").atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.controller.on_traj_start(message.pose.position.x, message.pose.position.y,
                                          message.pose.position.z, yaw)

    def publish_position(self, command: Mapping[str, Any]) -> None:
        message = self.PositionCommand()
        message.header.stamp = self.rospy.Time.now()
        message.header.frame_id = command.get("frame_id", "world")
        for name, values in (("position", message.position), ("velocity", message.velocity)):
            data = command[name]
            values.x, values.y, values.z = data["x"], data["y"], data["z"]
        message.acceleration.x = message.acceleration.y = message.acceleration.z = 0.0
        message.jerk.x = message.jerk.y = message.jerk.z = 0.0
        message.yaw = command["yaw"]
        message.yaw_dot = command["yaw_rate"]
        message.kx, message.kv = [0.0] * 3, [0.0] * 3
        message.trajectory_flag = getattr(self.PositionCommand, "TRAJECTORY_STATUS_READY", 0)
        self.pub_position.publish(message)

    def publish_takeoff_land(self, action: str) -> None:
        message = self.TakeoffLand()
        message.takeoff_land_cmd = (self.TakeoffLand.TAKEOFF if action == "takeoff"
                                    else self.TakeoffLand.LAND)
        self.pub_takeoff.publish(message)

    def invoke(self, document: Mapping[str, Any]):
        with self.lock:
            return self.controller.execute(MotionCommand.from_contract(document)).to_dict()

    def close(self) -> None:
        self.timer.shutdown()
        for subscriber in self.subscribers:
            subscriber.unregister()
        self.subscribers = []
