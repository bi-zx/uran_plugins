from __future__ import annotations

import time
from typing import Any, Mapping

from uran_interfaces import ContractValidator
from uran_worker_sdk import Worker, WorkerServer

from .geo import GeoReferenceConfig, MapProjector
from .geo_pose_fuser import GeoPoseFuser
from .gps_vo_yaw_aligner import GpsVoYawAligner
from .pose_registry import PoseRegistry


class LocalizationWorker(Worker):
    plugin_id = "uran-localization"
    plugin_version = "0.2.0"
    capabilities = ("sensors.localization", "sensors.localization.fusion")

    def configure(self, config: Mapping[str, Any]) -> None:
        self.validator = ContractValidator()
        self.registry = PoseRegistry(history_size=int(config.get("history_size", 600)))
        self.projector = MapProjector(GeoReferenceConfig.from_dict(
            dict(config.get("geo_reference") or {})))
        self.fuser = GeoPoseFuser(dict(config.get("fusion") or {}))
        self.yaw_aligner = GpsVoYawAligner(dict(config.get("yaw_alignment") or {}))
        self.expected_device_id = str(config.get("device_id") or self.identity.get("device_id") or "")

    def _guard_source(self, request: Mapping[str, Any], source: Mapping[str, Any]) -> None:
        routed = dict(request.get("target") or {})
        if routed.get("device_id") and routed["device_id"] != source.get("device_id"):
            raise ValueError("routed and observation device_id differ")
        if self.expected_device_id and source.get("device_id") != self.expected_device_id:
            raise ValueError("observation belongs to another device")

    def invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        if request.get("message_type") != "uran.sensors.observation.v1":
            raise ValueError("unsupported message_type")
        observation = dict(request.get("payload") or {})
        self.validator.validate("sensors/observation.schema.json", observation)
        source = dict(observation["source"])
        self._guard_source(request, source)
        timestamp_ns = int(observation["timestamp_ns"] or time.time_ns())
        value = observation.get("value")
        if not isinstance(value, Mapping):
            raise ValueError("localization observation value must be an object")
        value = dict(value)
        kind = str(observation["observation_type"])

        if not self.registry.record_observation(kind, value, timestamp_ns, observation.get("frame_id")) and kind != "localization_query":
            raise ValueError(f"unsupported localization observation_type: {kind}")

        yaw_result = self.yaw_aligner.update(
            gps_position=self.registry.gps_position(),
            visual_pose=self.registry.latest_visual_pose(),
            timestamp_ns=timestamp_ns)
        fused = self.fuser.fuse(
            pose_registry=self.registry,
            projector=self.projector,
            alignment_state={},
            timestamp_ns=timestamp_ns)
        result = {
            "source_observation": kind,
            "fused_position": fused,
            "yaw_alignment": yaw_result,
            "poses": {
                "gps": self.registry.gps_position(),
                "map": self.registry.map_pose_dict(),
                "visual": self.registry.visual_pose_dict(),
            },
            "history": self.registry.history_summary(),
        }
        return {"status": "succeeded", "operation_id": request["operation_id"],
                "payload": result}


def worker_factory() -> Worker:
    return LocalizationWorker()


def main() -> int:
    return WorkerServer(worker_factory()).serve()
