from __future__ import annotations

import threading
import time
from typing import Any, Mapping

from uran_interfaces import ContractValidator
from uran_worker_sdk import Worker, WorkerServer

from .controller import MotionCommand, Px4Controller


class SimulatedTransport:
    def __init__(self):
        self.positions: list[Mapping[str, Any]] = []
        self.actions: list[str] = []

    def publish_position(self, command): self.positions.append(dict(command))
    def publish_takeoff_land(self, action): self.actions.append(action)


class Px4Worker(Worker):
    plugin_id = "uran-px4-motion"
    plugin_version = "0.1.0"
    capabilities = ("motion.base.velocity", "motion.base.safety")

    def configure(self, config: Mapping[str, Any]) -> None:
        self.validator = ContractValidator()
        self.backend = None
        backend_name = str(config.get("backend", "ros1_px4"))
        if backend_name == "simulation":
            transport = SimulatedTransport()
            self.backend = Px4Controller(transport, linear_limit=float(config.get("linear_limit", 3.0)),
                                         angular_limit=float(config.get("angular_limit", 1.0)),
                                         cmd_timeout_s=float(config.get("cmd_timeout_s", 0.5)),
                                         land_retry_delay_s=float(config.get("land_retry_delay_s", 0.6)))
            self.transport = transport
        elif backend_name == "ros1_px4":
            from .ros1_backend import Ros1Px4Backend
            self.backend = Ros1Px4Backend(config)
        else:
            raise ValueError(f"unsupported backend: {backend_name}")
        self.expected_device_id = str(config.get("device_id") or self.identity.get("device_id") or "")
        self.expected_component_id = str(config.get("component_id") or self.identity.get("component_id") or "")
        self.expected_arbiter_instance = str(config.get("required_arbiter_instance") or "")
        self.sequences: dict[tuple[str, str, str], int] = {}
        self.lock = threading.RLock()

    def invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        if request.get("message_type") != "uran.motion.command.v1":
            raise ValueError("unsupported message_type")
        document = dict(request.get("payload") or {})
        self.validator.validate("motion/command.schema.json", document)
        target = document["target"]
        routed = request.get("target") or {}
        if self.expected_device_id and target["device_id"] != self.expected_device_id:
            raise ValueError("motion command targets another device")
        if self.expected_component_id and target["component_id"] != self.expected_component_id:
            raise ValueError("motion command targets another component")
        if routed.get("device_id") and routed["device_id"] != target["device_id"]:
            raise ValueError("routed and document device_id differ")
        if routed.get("component_id") and routed["component_id"] != target["component_id"]:
            raise ValueError("routed and document component_id differ")
        if int(document["valid_until_ns"]) <= time.time_ns():
            raise ValueError("motion command is expired")
        source = (request.get("source") or {}).get("instance_id", "")
        safety = (document.get("payload") or {}).get("kind") == "safety_action"
        if self.expected_arbiter_instance and not safety and source != self.expected_arbiter_instance:
            raise ValueError("motion command did not pass through configured control arbiter")
        key = (target["device_id"], target["component_id"], document.get("controller", ""))
        sequence = int(document["sequence"])
        if sequence <= self.sequences.get(key, -1):
            raise ValueError("motion command sequence is stale or duplicated")
        with self.lock:
            result = self.backend.invoke(document) if hasattr(self.backend, "invoke") else self.backend.execute(MotionCommand.from_contract(document)).to_dict()
        self.sequences[key] = sequence
        return {"status": "succeeded" if result["accepted"] else "failed",
                "operation_id": request["operation_id"], "payload": result}

    def deactivate(self) -> None:
        if self.backend and hasattr(self.backend, "close"):
            self.backend.close()
        elif self.backend:
            self.backend.execute(MotionCommand(action="emergency_stop"))


def worker_factory() -> Worker:
    return Px4Worker()


def main() -> int:
    return WorkerServer(worker_factory()).serve()
