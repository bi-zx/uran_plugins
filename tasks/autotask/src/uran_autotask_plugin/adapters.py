from __future__ import annotations

import math
import time
from collections import deque
from typing import Any, Dict, Optional


class SimulatedNavigationBackend:
    def __init__(self):
        self.goal = None
        self.events = deque()

    def has_active_goal(self) -> bool:
        return self.goal is not None

    def navigate_to_pose(self, pose, *, map_name: str, outdoor: bool) -> bool:
        if self.goal is not None:
            return False
        self.goal = {"pose": dict(pose), "map_name": map_name, "outdoor": outdoor}
        return True

    def send_raw_navigation_goal(self, *, nav_type: int, poses, map_name: str,
                                 outdoor: bool) -> bool:
        if self.goal is not None:
            return False
        self.goal = {"nav_type": nav_type, "poses": list(poses),
                     "map_name": map_name, "outdoor": outdoor}
        return True

    def stop_all_tasks(self, *, map_name: str = "") -> bool:
        del map_name
        if self.goal is not None:
            self.goal = None
            self.events.append({"state": "stopped", "reason": "simulated_stop",
                                "simulated": True})
        return True

    def refresh_status(self, *, force: bool = False) -> None:
        del force

    def take_terminal_event(self):
        return self.events.popleft() if self.events else None

    def get_execution_snapshot(self) -> Dict[str, Any]:
        return {"goal_state": "active" if self.goal else "idle",
                "goal": self.goal, "simulated": True}


class CyberDogAlgorithmManagerBackend:
    """Plugin-owned client for Xiaomi's algorithm_manager action and services."""

    def __init__(self, node, config: Dict[str, Any]):
        try:
            from geometry_msgs.msg import PoseStamped
            from protocol.action import Navigation
            from protocol.srv import AlgoTaskStatus, StopAlgoTask
            from rclpy.action import ActionClient
        except ImportError as exc:
            raise RuntimeError("CyberDog algorithm_manager interfaces are unavailable") from exc
        self.node = node
        self.PoseStamped = PoseStamped
        self.Navigation = Navigation
        self.StopAlgoTask = StopAlgoTask
        self.AlgoTaskStatus = AlgoTaskStatus
        namespace = str(config.get("namespace") or "").strip("/")
        resolve = lambda name: f"/{namespace}/{name}" if namespace else f"/{name}"
        self.action = ActionClient(node, Navigation,
                                   resolve(str(config.get("start_action_name") or "start_algo_task")))
        self.stop_client = node.create_client(
            StopAlgoTask, resolve(str(config.get("stop_service_name") or "stop_algo_task")))
        self.status_client = node.create_client(
            AlgoTaskStatus, resolve(str(config.get("status_service_name") or "algo_task_status")))
        self.ready_timeout_s = float(config.get("ready_timeout_s", 5.0))
        self.status_interval_s = float(config.get("status_poll_interval_s", 1.0))
        self.state = "idle"
        self.goal_handle = None
        self.events = deque()
        self.last_status = {}
        self.last_status_poll = 0.0

    def _pose(self, value):
        if not isinstance(value, dict):
            return value
        message = self.PoseStamped()
        message.header.frame_id = str(value.get("frame_id") or "map")
        stamp = int(value.get("timestamp_ns") or time.time_ns())
        message.header.stamp.sec = stamp // 1_000_000_000
        message.header.stamp.nanosec = stamp % 1_000_000_000
        message.pose.position.x = float(value.get("x", 0.0))
        message.pose.position.y = float(value.get("y", 0.0))
        message.pose.position.z = float(value.get("z", 0.0))
        yaw = float(value.get("yaw_rad", math.radians(value.get("yaw_deg", 0.0))))
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        return message

    def has_active_goal(self) -> bool:
        return self.state in {"waiting_accept", "active", "stopping"}

    def navigate_to_pose(self, pose, *, map_name: str, outdoor: bool) -> bool:
        nav_type = self.Navigation.Goal.NAVIGATION_TYPE_START_AB
        return self.send_raw_navigation_goal(nav_type=nav_type, poses=[pose],
                                             map_name=map_name, outdoor=outdoor)

    def send_raw_navigation_goal(self, *, nav_type: int, poses, map_name: str,
                                 outdoor: bool) -> bool:
        if self.has_active_goal() or not self.action.wait_for_server(timeout_sec=self.ready_timeout_s):
            return False
        goal = self.Navigation.Goal()
        goal.nav_type = int(nav_type)
        goal.poses = [self._pose(pose) for pose in poses]
        goal.label_id = ""
        goal.map_name = str(map_name or "")
        goal.relative_pos = getattr(self.Navigation.Goal, "TRACING_AUTO", 200)
        goal.keep_distance = 0.0
        goal.outdoor = bool(outdoor)
        goal.object_tracking = False
        self.state = "waiting_accept"
        self.action.send_goal_async(goal, feedback_callback=self._on_feedback).add_done_callback(
            self._on_goal_response)
        return True

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.state = "failed"
            self.events.append({"state": "failed", "description": str(exc)})
            return
        if not handle.accepted:
            self.state = "rejected"
            self.events.append({"state": "rejected", "reason": "goal_rejected"})
            return
        self.goal_handle = handle
        self.state = "active"
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, message):
        feedback = message.feedback
        self.last_status.update(feedback_code=int(feedback.feedback_code),
                                feedback_msg=str(feedback.feedback_msg))

    def _on_result(self, future):
        try:
            wrapped = future.result()
            code = int(wrapped.result.result)
            success = self.Navigation.Result.NAVIGATION_RESULT_TYPE_SUCCESS
            cancelled = self.Navigation.Result.NAVIGATION_RESULT_TYPE_CANCEL
            self.state = "succeeded" if code == success else ("canceled" if code == cancelled else "failed")
            self.events.append({"state": self.state, "result_code": code,
                                "status_code": int(wrapped.status), **self.last_status})
        except Exception as exc:
            self.state = "failed"
            self.events.append({"state": "failed", "description": str(exc)})
        self.goal_handle = None

    def stop_all_tasks(self, *, map_name: str = "") -> bool:
        if not self.stop_client.wait_for_service(timeout_sec=self.ready_timeout_s):
            return False
        request = self.StopAlgoTask.Request()
        request.task_id = self.StopAlgoTask.Request.ALGO_TASK_ALL
        request.map_name = str(map_name or "")
        self.state = "stopping"
        self.stop_client.call_async(request).add_done_callback(self._on_stop)
        return True

    def _on_stop(self, future):
        try:
            response = future.result()
            success = int(response.result) == int(response.SUCCESS)
            self.state = "stopped" if success else "failed"
            self.events.append({"state": self.state, "code": int(response.code)})
        except Exception as exc:
            self.state = "failed"
            self.events.append({"state": "failed", "description": str(exc)})
        self.goal_handle = None

    def refresh_status(self, *, force: bool = False):
        now = time.monotonic()
        if not force and now - self.last_status_poll < self.status_interval_s:
            return
        if not self.status_client.wait_for_service(timeout_sec=0.05):
            return
        self.last_status_poll = now
        self.status_client.call_async(self.AlgoTaskStatus.Request()).add_done_callback(
            lambda future: self._status_done(future))

    def _status_done(self, future):
        try:
            response = future.result()
            self.last_status.update(manager_status=int(response.status),
                                    manager_code=int(response.code))
        except Exception:
            pass

    def take_terminal_event(self):
        return self.events.popleft() if self.events else None

    def get_execution_snapshot(self) -> Dict[str, Any]:
        return {"goal_state": self.state, **self.last_status, "simulated": False}


class CyberDogCameraCaptureBackend:
    def __init__(self, node, config: Dict[str, Any]):
        try:
            from protocol.srv import CameraService, TakePhoto
        except ImportError as exc:
            raise RuntimeError("CyberDog camera capture interfaces are unavailable") from exc
        self.CameraService = CameraService
        self.TakePhoto = TakePhoto
        namespace = str(config.get("namespace") or "").strip("/")
        resolve = lambda name: f"/{namespace}/{name}" if namespace else f"/{name}"
        self.camera = node.create_client(
            CameraService, resolve(str(config.get("camera_service_name") or "camera_service")))
        self.photo = node.create_client(
            TakePhoto, resolve(str(config.get("take_photo_service_name") or "take_photo")))
        self.prefer_photo = bool(config.get("prefer_take_photo_service", False))
        self.timeout_s = float(config.get("ready_timeout_s", 5.0))
        self.width = int(config.get("width", 1280))
        self.height = int(config.get("height", 960))
        self.fps = int(config.get("fps", 10))
        self.pending = None

    def cancel_pending(self):
        self.pending = None

    def start_capture(self):
        if self.pending is not None:
            return {"accepted": False, "result": {"success": False,
                    "message": "camera capture already pending"}}
        choices = (("take_photo", self.photo), ("camera_service", self.camera))
        if not self.prefer_photo:
            choices = tuple(reversed(choices))
        for kind, client in choices:
            if not client.wait_for_service(timeout_sec=0.1):
                continue
            if kind == "take_photo":
                request = self.TakePhoto.Request()
            else:
                request = self.CameraService.Request()
                request.command = self.CameraService.Request.TAKE_PICTURE
                request.args = ""
                request.width, request.height, request.fps = self.width, self.height, self.fps
            self.pending = (client.call_async(request), time.monotonic() + self.timeout_s, kind)
            return {"accepted": True, "backend": kind}
        return {"accepted": False, "result": {"success": False,
                "message": "no camera capture service is ready"}}

    def poll_capture(self):
        if self.pending is None:
            return {"success": False, "message": "no capture is pending"}
        future, deadline, kind = self.pending
        if not future.done() and time.monotonic() < deadline:
            return None
        self.pending = None
        if not future.done():
            return {"success": False, "backend": kind, "message": "capture timeout"}
        try:
            response = future.result()
            if kind == "take_photo":
                return {"success": bool(response.result), "backend": kind,
                        "message": str(response.message), "code": int(response.code)}
            return {"success": int(response.result) == int(self.CameraService.Response.RESULT_SUCCESS),
                    "backend": kind, "message": str(response.msg), "code": int(response.code)}
        except Exception as exc:
            return {"success": False, "backend": kind, "message": str(exc)}
