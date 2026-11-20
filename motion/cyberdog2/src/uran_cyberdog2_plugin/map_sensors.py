"""CyberDog2 time-stamped ToF measurements and laser SLAM sessions."""
import time
import math
import numpy as np
from rclpy.qos import qos_profile_sensor_data

from protocol.action import Navigation
from protocol.srv import GetMapLabel, StopAlgoTask
from rclpy.action import ActionClient
from std_srvs.srv import SetBool


class LaserSession:
    def __init__(self, node, report):
        self.report = report
        self.navigation = ActionClient(node,Navigation,"start_algo_task")
        self.map_label = node.create_client(GetMapLabel,"get_label")
        self.stop_mapping = node.create_client(StopAlgoTask,"stop_algo_task")
        self.stop_localization = node.create_client(SetBool,"reset_stop_lidar_localization")
        self.operation = self.map_id = self.mode = self.command = ""
        self.stage = "idle"
        self.future = self.result = None
        self.got_feedback = False
        self.reason = ""
        self.owns_goal = False

    def destroy(self):
        self.navigation.destroy()

    def reply(self):
        self.report(self.operation,self.map_id,self.stage,self.reason)

    def request(self, operation, map_id, mode, command):
        if operation != self.operation:
            if self.stage not in ("idle","closed","failed"):
                return
            self.operation,self.map_id,self.mode = operation,map_id,mode
            self.future = self.result = None
            self.close_started = None
            self.got_feedback = self.owns_goal = False
            self.reason = ""
            self.stage = "check_map" if mode == "localization" else "start"
            self.started = time.monotonic()
            if command != "start":
                self.stage = "failed"
                self.reason = "no matching laser session in this bridge"
        self.command = command
        if command in ("stop","save") and self.close_started is None:
            self.close_started = time.monotonic()
        if self.stage in ("closed","failed") or self.stage == "ready" and command == "start":
            self.reply()

    def feedback(self,msg):
        code = msg.feedback.feedback_code
        # CyberDog algorithm_manager uses 3108 on successful laser relocalization.
        expected = 3108 if self.mode == "localization" else Navigation.Feedback.NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS
        if code == expected:
            self.got_feedback = True

    def tick(self):
        if self.stage in ("idle","failed","closed"):
            return
        try:
            self.advance()
        except (RuntimeError,ValueError) as exc:
            self.reason = str(exc)
            if self.owns_goal and self.stage not in ("closing","await_close"):
                self.command = "stop"
                self.close_started = time.monotonic()
                self.stage = "closing"
            else:
                self.stage = "failed"
                self.reply()

    def advance(self):
        if self.command in ("stop","save") and self.stage not in ("await_goal","closing","await_close"):
            if not self.owns_goal:
                self.stage = "closed"
                self.reply()
                return
            self.stage = "closing"
        if self.stage not in ("ready","closing","await_close") and time.monotonic()-self.started > 150:
            raise RuntimeError("timed out waiting for the CyberDog laser SLAM service")
        if self.stage == "check_map" and self.map_label.service_is_ready():
            request = GetMapLabel.Request()
            request.map_name = self.map_id
            self.future = self.map_label.call_async(request)
            self.stage = "await_label"
        if self.stage == "await_label" and self.future.done():
            response = self.future.result()
            if response.success != GetMapLabel.Response.RESULT_SUCCESS or response.label.map_name != self.map_id:
                raise RuntimeError("CyberDog active laser map does not match "+self.map_id)
            self.stage = "start"
        if self.stage == "start" and self.navigation.server_is_ready():
            goal = Navigation.Goal()
            goal.nav_type = goal.NAVIGATION_TYPE_START_MAPPING if self.mode == "mapping" else goal.NAVIGATION_TYPE_START_LOCALIZATION
            goal.map_name = self.map_id
            # This flag selects LaserMapping/LaserLocalization in CyberDog's Task.toml.
            goal.outdoor = False
            self.future = self.navigation.send_goal_async(goal,feedback_callback=self.feedback)
            self.stage = "await_goal"
        if self.stage == "await_goal" and self.future.done():
            handle = self.future.result()
            if not handle.accepted:
                raise RuntimeError("CyberDog rejected laser "+self.mode)
            self.result = handle.get_result_async()
            self.owns_goal = True
            self.stage = "await_ready"
        if self.stage in ("await_ready","ready") and self.result is not None and self.result.done():
            code = self.result.result().result.result
            self.result = None
            # Localization leaves the pose producer running after its action succeeds.
            if self.mode == "localization" and code == Navigation.Result.NAVIGATION_RESULT_TYPE_SUCCESS:
                self.got_feedback = True
            else:
                self.owns_goal = False
                raise RuntimeError("laser "+self.mode+" ended with result "+str(code))
        if self.stage == "await_ready" and self.got_feedback and self.command == "start":
            self.stage = "ready"
            self.reply()
        if self.stage in ("closing","await_close") and self.close_started is not None and time.monotonic()-self.close_started > 30:
            raise RuntimeError("timed out closing CyberDog laser "+self.mode)
        if self.stage == "closing":
            if self.mode == "mapping":
                service,request = self.stop_mapping,StopAlgoTask.Request()
                request.task_id = request.ALGO_TASK_MAPPING
                request.map_name = self.map_id if self.command == "save" else ""
            else:
                service,request = self.stop_localization,SetBool.Request()
                request.data = True
            if service.service_is_ready():
                self.future = service.call_async(request)
                self.stage = "await_close"
        if self.stage == "await_close" and self.future.done():
            response = self.future.result()
            ok = response.result == StopAlgoTask.Response.SUCCESS if self.mode == "mapping" else response.success
            if not ok:
                raise RuntimeError("CyberDog failed to "+self.command+" laser "+self.mode)
            self.owns_goal = False
            self.stage = "failed" if self.reason else "closed"
            self.reply()


# CyberDog2 motion_utils/head_tof_pcl_publisher.py supplies this ray layout and both mounts.
def tof_points(part, rear=False):
    if not part.data_available or len(part.data) != 64:
        return None
    depth = np.asarray(part.data[::-1])
    valid = (depth >= 0.15) & (depth < 0.6595)
    if not valid.any():
        return None
    angles = np.array([[25.8,21.4,19.2,17.9], [21.4,18.5,14.9,13.1],
                       [19.2,14.9,11.1,8.2], [17.9,13.1,8.2,3.7]])
    row,col = np.divmod(np.flatnonzero(valid),8)
    angle = angles[np.minimum(row,7-row),np.minimum(col,7-col)]
    dx,dy = 7-2*col,2*row-7
    if rear:
        dx,dy = -dx,-dy
    r = depth[valid]*np.tan(np.radians(angle))/np.hypot(dx,dy)
    return np.column_stack((r*dx,r*dy,-depth[valid]))


def head_points(part, side):
    xyz = tof_points(part)
    if xyz is None:
        return None
    roll,pitch = side*0.296,-0.266
    cr,sr,cp,sp = math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch)
    rotation = np.array([[cp,sp*sr,sp*cr],[0,cr,-sr],[-sp,cp*sr,cp*cr]])
    origin = np.array([0.259,side*0.030,0.102])
    return origin, xyz @ rotation.T + origin



def bind_tof(node):
    from protocol.msg import HeadTofPayload
    tof = node.declare_parameter("tof_topic","head_tof_payload").value
    if tof:
        mount_frame = node.declare_parameter("tof_mount_frame","base_link").value
        def on_head(msg):
            for part,side in ((msg.left_head,1),(msg.right_head,-1)):
                rays = head_points(part,side)
                if rays is not None:
                    node.queue("tof_"+str(side),(mount_frame,rays[0],rays[1]),part.header.stamp)
        node.create_subscription(HeadTofPayload,tof,on_head,qos_profile_sensor_data)
    rear_tof = node.declare_parameter("rear_tof_topic","").value
    if rear_tof:
        from protocol.msg import RearTofPayload
        def on_rear(msg):
            for part in (msg.left_rear,msg.right_rear):
                points = tof_points(part,rear=True)
                if points is not None and part.header.frame_id:
                    node.queue(part.header.frame_id,(part.header.frame_id,np.zeros(3),points),part.header.stamp)
        node.create_subscription(RearTofPayload,rear_tof,on_rear,qos_profile_sensor_data)


def bind_sensors(node):
    from protocol.msg import GpsPayload, MotionStatus
    from .vendor_policy import support_state
    bind_tof(node)
    if not hasattr(node,"base_support"):
        return
    motion = node.declare_parameter("support_topic","motion_status").value
    if motion:
        def on_motion(msg):
            # MotionStatus has no header. Timestamp it on receipt.
            node.base_support = (node.get_clock().now().nanoseconds,
                                 support_state(msg.switch_status,msg.motion_id,msg.contact))
        node.create_subscription(MotionStatus,motion,on_motion,qos_profile_sensor_data)
    gps = node.declare_parameter("receiver_topic","").value
    if gps:
        def on_fix(msg):
            if msg.fix_type >= 3:
                node.geographic_fix(msg.sec*10**9+msg.nanosec,msg.lat,msg.lon,{"fix_type":msg.fix_type,"num_sv":msg.num_sv})
        node.create_subscription(GpsPayload,gps,on_fix,qos_profile_sensor_data)
