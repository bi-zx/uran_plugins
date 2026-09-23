from __future__ import annotations

import base64
from types import SimpleNamespace
from typing import Any, Mapping

from uran_interfaces import ContractValidator
from uran_worker_sdk import Worker, WorkerServer

from .depth_scan_adapter import DepthPointCloudScanAdapter, RigidTransform, VirtualLaserScan
from .obstacle_avoidance import GapAvoidancePlanner


class NavigationWorker(Worker):
    plugin_id = "uran-navigation"
    plugin_version = "0.1.0"
    capabilities = ("sensors.depth.scan", "sensors.obstacle.avoidance")

    def configure(self, config: Mapping[str, Any]) -> None:
        self.validator = ContractValidator()
        self.depth = DepthPointCloudScanAdapter(dict(config.get("depth_scan") or {}))
        self.planner = GapAvoidancePlanner(dict(config.get("obstacle_avoidance") or {}))
        self.expected_device_id = str(config.get("device_id") or self.identity.get("device_id") or "")

    @staticmethod
    def _scan(value: Mapping[str, Any]) -> VirtualLaserScan:
        return VirtualLaserScan(
            angle_min=float(value["angle_min"]), angle_max=float(value["angle_max"]),
            angle_increment=float(value["angle_increment"]),
            range_min=float(value["range_min"]), range_max=float(value["range_max"]),
            ranges=[float(item) for item in value["ranges"]])

    @staticmethod
    def _cloud(value: Mapping[str, Any]):
        fields = [SimpleNamespace(**dict(field)) for field in value["fields"]]
        return SimpleNamespace(
            width=int(value["width"]), height=int(value["height"]),
            point_step=int(value["point_step"]), row_step=int(value["row_step"]),
            is_bigendian=bool(value.get("is_bigendian", False)), fields=fields,
            data=base64.b64decode(value["data_base64"], validate=True),
            header=SimpleNamespace(frame_id=str(value.get("frame_id") or "")))

    def invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        if request.get("message_type") != "uran.sensors.observation.v1":
            raise ValueError("unsupported message_type")
        observation = dict(request.get("payload") or {})
        self.validator.validate("sensors/observation.schema.json", observation)
        source = dict(observation["source"])
        routed = dict(request.get("target") or {})
        if routed.get("device_id") and routed["device_id"] != source["device_id"]:
            raise ValueError("routed and observation device_id differ")
        if self.expected_device_id and source["device_id"] != self.expected_device_id:
            raise ValueError("observation belongs to another device")
        value = observation.get("value")
        if not isinstance(value, Mapping):
            raise ValueError("navigation observation value must be an object")
        kind = str(observation["observation_type"])
        if kind == "laser_scan":
            scan = self._scan(value)
        elif kind == "depth_pointcloud":
            transform_data = dict(value.get("transform") or {})
            transform = RigidTransform(
                tuple(transform_data.get("translation") or (0.0, 0.0, 0.0)),
                tuple(transform_data.get("rotation_xyzw") or (0.0, 0.0, 0.0, 1.0))).normalized()
            scan = self.depth.convert(self._cloud(value), transform)
        else:
            raise ValueError(f"unsupported navigation observation_type: {kind}")
        decision = self.planner.plan(
            scan, theta_goal=float(value.get("theta_goal_rad", 0.0)),
            stop_distance_m=value.get("stop_distance_m"))
        result = {
            "scan": {
                "frame_id": str(observation.get("frame_id") or self.depth.target_frame),
                "angle_min": scan.angle_min, "angle_max": scan.angle_max,
                "angle_increment": scan.angle_increment,
                "range_min": scan.range_min, "range_max": scan.range_max,
                "ranges": list(scan.ranges),
            },
            "obstacle": decision.to_dict(),
            "depth_adapter": self.depth.snapshot(),
        }
        return {"status": "succeeded", "operation_id": request["operation_id"],
                "payload": result}


def worker_factory() -> Worker:
    return NavigationWorker()


def main() -> int:
    return WorkerServer(worker_factory()).serve()
