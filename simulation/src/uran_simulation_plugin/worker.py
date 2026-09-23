from __future__ import annotations

import time
from typing import Any, Mapping

from uran_interfaces import ContractValidator
from uran_worker_sdk import Worker, WorkerServer

from .robot import MultiComponentRobot


class SimulationWorker(Worker):
    plugin_id = "uran-multi-component-simulation"
    plugin_version = "0.1.0"
    capabilities = ("motion.base.velocity", "motion.arm.joints",
                    "motion.arm.pose", "motion.gripper.position",
                    "motion.base.safety", "motion.arm.safety", "motion.gripper.safety")

    def configure(self, config: Mapping[str, Any]) -> None:
        self.validator = ContractValidator()
        self.robot = MultiComponentRobot(config.get("components") or [])
        self.sequences = {}
        self.state_fields = set()
        self.expected_device_id = str(config.get("device_id") or self.identity.get("device_id") or "")
        self.expected_arbiter_instance = str(config.get("required_arbiter_instance") or "")
        self.state_reporting = bool(config.get("state_reporting", True))

    def invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        document = dict(request.get("payload") or {})
        self.validator.validate("motion/command.schema.json", document)
        if document["valid_until_ns"] <= time.time_ns():
            raise ValueError("motion command is expired")
        target = document["target"]
        routed = dict(request.get("target") or {})
        for field in ("device_id", "component_id"):
            if routed.get(field) and routed[field] != target[field]:
                raise ValueError(f"routed and document {field} differ")
        if self.expected_device_id and target["device_id"] != self.expected_device_id:
            raise ValueError("motion command targets another device")
        body = document["payload"]
        safety_bypass = (body.get("kind") == "safety_action" and
                         body.get("action") in {"stop", "emergency_stop"})
        source = dict(request.get("source") or {})
        if (self.expected_arbiter_instance and not safety_bypass and
                source.get("instance_id") != self.expected_arbiter_instance):
            raise ValueError("motion command did not pass through the configured control arbiter")
        key = (target["device_id"], target["component_id"], document["controller"])
        if document["sequence"] <= self.sequences.get(key, -1):
            raise ValueError("motion sequence is stale or duplicated")
        self.sequences[key] = document["sequence"]
        result = self.robot.execute(target, body)
        field = f"plugin.{self.identity.get('instance_id', 'simulation')}.last_command"
        if self.state_reporting and field not in self.state_fields:
            self.register_state(field)
            self.state_fields.add(field)
        if self.state_reporting:
            self.set_state(field, result)
        return {"status": "succeeded", "operation_id": request["operation_id"],
                "payload": result}


def worker_factory() -> Worker:
    return SimulationWorker()


def main() -> int:
    return WorkerServer(worker_factory()).serve()
