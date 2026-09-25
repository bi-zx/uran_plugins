import re
from typing import Any, Mapping

from uran_worker_sdk import Worker, WorkerServer


def _segment(value: str) -> str:
    return "id_" + (re.sub(r"[^A-Za-z0-9_]", "_", str(value)) or "unknown")


class CloudStatePolicyWorker(Worker):
    """Owns cloud-controlled StateStore fields; MQTT remains a transport."""

    plugin_id = "uran-cloud-state-policy"
    plugin_version = "0.1.0"
    capabilities = ("state.policy.control", "state.policy.parameters")

    def configure(self, config: Mapping[str, Any]) -> None:
        self.device_id = str(config.get("device_id") or self.identity.get("device_id") or "")
        if not self.device_id:
            raise ValueError("device_id is required")
        self.prefix = f"device.{_segment(self.device_id)}"
        self.defaults = dict(config.get("control_defaults") or {
            "mode": "manual", "controller": "cloud", "primary_uplink_protocol": "mqtt"})

    def activate(self) -> None:
        for name, value in self.defaults.items():
            field = f"{self.prefix}.control.{_segment(name)}"
            self.register_state(field, "any", persistent=True)
            self.set_state(field, value, persistent=True)

    def _validate_target(self, payload: Mapping[str, Any]) -> None:
        target = dict(payload.get("target") or {})
        if target.get("device_id") != self.device_id:
            raise ValueError("state policy command targets another device")

    def invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        message_type = str(request.get("message_type") or "")
        payload = dict(request.get("payload") or {})
        self._validate_target(payload)
        changed = {}
        if message_type == "uran.legacy.control-switch.v1":
            switch = payload.get("switch")
            if not isinstance(switch, Mapping):
                raise ValueError("control switch must be an object")
            for name, value in switch.items():
                field = f"{self.prefix}.control.{_segment(name)}"
                self.register_state(field, "any", persistent=True)
                self.set_state(field, value, persistent=True)
                changed[field] = value
        elif message_type == "uran.legacy.param-update.v1":
            parameters = payload.get("params_json", payload.get("params", {}))
            if not isinstance(parameters, Mapping):
                raise ValueError("parameters must be an object")
            for name, value in parameters.items():
                field = f"{self.prefix}.parameters.{_segment(name)}"
                self.register_state(field, "any", persistent=True)
                self.set_state(field, value, persistent=True)
                changed[field] = value
        else:
            raise ValueError("unsupported state policy message_type")
        return {"status": "succeeded", "operation_id": request["operation_id"],
                "payload": {"changed": changed}}


def worker_factory() -> Worker:
    return CloudStatePolicyWorker()


def main() -> int:
    return WorkerServer(worker_factory()).serve()
