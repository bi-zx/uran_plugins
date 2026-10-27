from __future__ import annotations

import time
import threading
import logging
from typing import Any, Mapping

from uran_worker_sdk import Worker, WorkerServer
from uran_interfaces import ContractValidator

from .arbiter import ControlArbiter, Lease


class ControlArbiterWorker(Worker):
    plugin_id = "uran-control-arbiter"
    plugin_version = "0.1.0"
    capabilities = ("motion.control.acquire", "motion.control.release",
                    "motion.control.query", "motion.control.gateway")

    def configure(self, config: Mapping[str, Any]) -> None:
        self.arbiter = ControlArbiter()
        self.validator = ContractValidator()
        self.backends = dict(config.get("backend_instances") or {})
        self.safety_capabilities = dict(config.get("safety_capabilities") or {})
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.pending = {}
        self.lease_sources = {}
        self.stop_timeout_s = float(config.get("stop_timeout_s", 2.0))
        self.expiry_interval_s = max(0.01, float(config.get("expiry_interval_s", 0.05)))
        self.pending_stops = {}
        self.resource_groups = dict(config.get("resource_groups") or {})
        self.expected_device_id = str(config.get("device_id") or self.identity.get("device_id") or "")
        self._motion_sequence = 0

    def _resource_group(self, component_id: str) -> str:
        return str(self.resource_groups.get(component_id) or
                   ("base-motion" if component_id == "base" else f"{component_id}-motion"))

    @staticmethod
    def _capability(payload: Mapping[str, Any]) -> str:
        return {
            "base_velocity": "motion.base.velocity",
            "base_posture": "motion.base.posture",
            "safety_action": "motion.base.safety",
            "arm_cartesian_velocity": "motion.arm.cartesian_velocity",
            "arm_pose": "motion.arm.pose",
            "gripper_action": "motion.gripper.action",
        }.get(str(payload.get("kind") or ""), "")

    def _motion_message(self, document: Mapping[str, Any]) -> Mapping[str, Any]:
        target = dict(document["target"])
        backend = str(self.backends.get(target["component_id"]) or
                      self.backends.get("*") or "")
        if not backend:
            raise RuntimeError(f"no motion backend configured for {target['component_id']}")
        capability = self._capability(document["payload"])
        if document["payload"].get("kind") == "safety_action":
            capability = str(self.safety_capabilities.get(target["component_id"]) or "motion.base.safety")
        if not capability:
            raise ValueError("motion payload kind has no routable capability")
        return {
            "message_type": "uran.motion.command.v1",
            "payload": dict(document),
            "correlation_id": str(document["operation_id"]),
            "target": {**target, "instance_id": backend, "capability_id": capability},
        }

    def _dispatch_stop(self, lease: Lease, *, reason: str, now_ns: int) -> None:
        self._motion_sequence += 1
        operation_id = f"lease-stop-{lease.lease_id}-{self._motion_sequence}"
        document = {
            "schema_version": "1.0",
            "operation_id": operation_id,
            "target": {"device_id": lease.device_id, "component_id": lease.component_id},
            "controller": "control-arbiter",
            "sequence": self._motion_sequence,
            "issued_at_ns": now_ns,
            "valid_until_ns": now_ns + 1_000_000_000,
            "payload": {"kind": "safety_action", "action": "stop", "reason": reason},
            "metadata": {"source": "control-arbiter", "lease_id": lease.lease_id},
        }
        self.call_local(self._motion_message(document), timeout=self.stop_timeout_s)

    def _expire_and_stop(self, now_ns: int) -> None:
        for lease in self.arbiter.expire(now_ns=now_ns):
            self.pending_stops[(lease.device_id, lease.resource_group)] = lease
            self.lease_sources.pop(lease.lease_id, None)
        for key, lease in list(self.pending_stops.items()):
            self._dispatch_stop(lease, reason="lease_expired", now_ns=now_ns)
            self.pending_stops.pop(key, None)

    def activate(self) -> None:
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._expiry_loop, daemon=True)
        self.thread.start()

    def _expiry_loop(self) -> None:
        while not self.stop_event.wait(self.expiry_interval_s):
            try:
                with self.lock:
                    self._expire_and_stop(time.time_ns())
                    for operation_id, pending in list(self.pending.items()):
                        if time.monotonic() >= pending["deadline"]:
                            self.emit_operation(operation_id, pending["sequence"] + 1, "failed",
                                                {"reason": "backend_result_timeout"})
                            self.pending.pop(operation_id, None)
            except Exception as exc:
                logging.getLogger(__name__).error("lease stop failed; resource remains blocked: %s", exc)

    def deactivate(self) -> None:
        self.stop_event.set()
        if hasattr(self, "thread"):
            self.thread.join(timeout=self.stop_timeout_s + 1)

    def on_operation(self, result: Mapping[str, Any], source: Mapping[str, str]) -> None:
        self.validator.validate("operations/state.schema.json", result)
        with self.lock:
            pending = self.pending.get(result["operation_id"])
            if not pending or source.get("instance_id") != pending["provider"]:
                return
            if result["sequence"] <= pending["sequence"]:
                return
            pending["sequence"] = result["sequence"]
            self.emit_operation(result["operation_id"], result["sequence"], result["status"],
                                result.get("payload") or result.get("error") or {})
            if result["status"] in {"succeeded", "failed", "rejected", "cancelled"}:
                self.pending.pop(result["operation_id"], None)

    def _validate_routing(self, request: Mapping[str, Any], target: Mapping[str, Any]) -> None:
        routed = dict(request.get("target") or {})
        for field in ("device_id", "component_id"):
            if routed.get(field) and routed[field] != target.get(field):
                raise ValueError(f"routed and document {field} differ")
        if self.expected_device_id and target.get("device_id") != self.expected_device_id:
            raise ValueError("command targets another device")

    def invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        with self.lock:
            return self._invoke(request)

    def _invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        message_type = str(request.get("message_type") or "")
        payload = dict(request.get("payload") or {})
        now_ns = time.time_ns()
        self._expire_and_stop(now_ns)
        if message_type == "uran.motion.command.v1":
            self.validator.validate("motion/command.schema.json", payload)
            target = payload["target"]
            self._validate_routing(request, target)
            body = payload["payload"]
            safety_bypass = (body.get("kind") == "safety_action" and
                             body.get("action") in {"stop", "emergency_stop"})
            if not safety_bypass:
                lease_id = str(payload.get("lease_id") or "")
                owner = str(payload.get("metadata", {}).get("source") or
                            payload.get("controller") or "")
                if not lease_id or not self.arbiter.permits(
                        lease_id=lease_id, device_id=target["device_id"],
                        component_id=target["component_id"],
                        resource_group=self._resource_group(target["component_id"]),
                        owner=owner, now_ns=now_ns):
                    raise ValueError("motion command is not authorized by the active lease")
                bound_source = self.lease_sources.get(lease_id, "")
                if bound_source != str(request.get("source", {}).get("instance_id") or ""):
                    raise ValueError("motion sender differs from lease holder")
            if payload["valid_until_ns"] <= now_ns:
                raise ValueError("motion command is expired")
            operation_id = request["operation_id"]
            if operation_id != payload["operation_id"] or operation_id in self.pending:
                raise ValueError("motion operation identity is mismatched or already pending")
            message = self._motion_message(payload)
            self.pending[operation_id] = {"provider": message["target"]["instance_id"],
                                          "sequence": 0, "deadline": time.monotonic() + 5.0}
            try:
                self.dispatch_local(message)
            except Exception:
                self.pending.pop(operation_id, None)
                raise
            return {"status": "accepted", "operation_id": operation_id,
                    "payload": {"forwarded": True}}
        elif message_type == "uran.control-lease.acquire.v1":
            self.validator.validate("components/control-lease.schema.json", payload)
            lease = Lease.from_contract(payload)
            self._validate_routing(request, payload["resource"])
            if lease.resource_group != self._resource_group(lease.component_id):
                raise ValueError("lease resource_group differs from configured component resource")
            if lease.expires_at_ns <= now_ns or lease.acquired_at_ns > now_ns:
                raise ValueError("lease interval is not active")
            sender = str(request.get("source", {}).get("instance_id") or "")
            if lease.lease_id in self.lease_sources and self.lease_sources[lease.lease_id] != sender:
                raise ValueError("lease renewal sender differs from lease holder")
            current = self.arbiter.get(lease.device_id, lease.resource_group, now_ns=now_ns)
            if current and current.lease_id != lease.lease_id:
                if lease.priority <= current.priority:
                    self.arbiter.acquire(lease, now_ns=now_ns)
                self._dispatch_stop(current, reason="lease_preempted", now_ns=now_ns)
            displaced = self.arbiter.acquire(lease, now_ns=now_ns)
            self.lease_sources[lease.lease_id] = sender
            if displaced:
                self.lease_sources.pop(displaced.lease_id, None)
            result = {"lease": lease.to_dict(),
                      "preempted": displaced.to_dict() if displaced else None}
        elif message_type == "uran.control-lease.release.v1":
            self.validator.validate("components/control-lease.schema.json", payload)
            lease = Lease.from_contract(payload)
            self._validate_routing(request, payload["resource"])
            current = self.arbiter.get(lease.device_id, lease.resource_group, now_ns=now_ns)
            if current is None:
                result = {"released": False}
            else:
                if current.lease_id != lease.lease_id or current.owner != lease.owner:
                    raise ValueError("lease identity or owner does not match")
                if self.lease_sources.get(lease.lease_id, "") != str(request.get("source", {}).get("instance_id") or ""):
                    raise ValueError("lease release sender differs from lease holder")
                self._dispatch_stop(current, reason="lease_released", now_ns=now_ns)
                result = {"released": self.arbiter.release(
                    lease_id=lease.lease_id, device_id=lease.device_id,
                    resource_group=lease.resource_group, owner=lease.owner, now_ns=now_ns)}
                self.lease_sources.pop(lease.lease_id, None)
        elif message_type == "uran.control-lease.query.v1":
            result = {"leases": self.arbiter.snapshot(now_ns=now_ns)}
        else:
            raise ValueError("unsupported control arbiter message_type")
        return {"status": "succeeded", "operation_id": request["operation_id"],
                "payload": result}


def worker_factory() -> Worker:
    return ControlArbiterWorker()


def main() -> int:
    return WorkerServer(worker_factory()).serve()
