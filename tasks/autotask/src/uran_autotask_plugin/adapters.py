from __future__ import annotations

from importlib import import_module
from collections import deque
from typing import Any, Dict


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


def ros_backend_factory(config):
    adapter = config.get("ros_adapter")
    if not adapter and config.get("backend") == "cyberdog2_ros2":
        adapter = "uran_autotask_plugin.ros.devices.cyberdog2:create_backend"
    if not adapter:
        return None
    module, factory = adapter.split(":")
    return getattr(import_module(module), factory)


# Original import paths for callers of the waypoint backend.
def __getattr__(name):
    if name in {"CyberDogAlgorithmManagerBackend", "CyberDogCameraCaptureBackend"}:
        return getattr(import_module(".ros.devices.cyberdog2", __package__), name)
    raise AttributeError(name)
