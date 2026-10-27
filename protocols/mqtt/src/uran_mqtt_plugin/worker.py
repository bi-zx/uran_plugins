import threading
import time
import re
from typing import Any, Mapping

from uran_worker_sdk import Worker, WorkerServer, now_ns

from .client import MqttClient
from .codec import CloudCodec


class MqttWorker(Worker):
    plugin_id = "uran-mqtt"
    plugin_version = "0.1.0"
    capabilities = ("protocol.mqtt", "protocol.legacy-cloud-codec")

    def configure(self, config: Mapping[str, Any]) -> None:
        self.codec = CloudCodec()
        self.config = dict(config)
        self.client = None
        self._sequence = 0
        self._sequence_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._state = {}
        self._active = False
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread = None
        self._publish_failures = 0
        self._commands_enabled_at = float("inf")
        self.transport_enabled = bool(config.get("transport_enabled", False))
        self.default_device_id = str(config.get("device_id") or self.identity.get("device_id") or "")
        if (self.identity.get("device_id") and config.get("device_id") and
                config["device_id"] != self.identity["device_id"]):
            raise ValueError("MQTT device_id differs from the Core identity")
        self.config["device_id"] = self.default_device_id
        self.default_component_id = str(config.get("default_component_id") or "base")
        # Legacy media_ctrl packets did not carry component_id.  Keep motion
        # commands on the normal default component while routing those media
        # packets to the configured media worker internally.
        self.media_component_id = str(config.get("media_component_id") or "media")
        self.validity_window_ns = int(float(config.get("command_validity_s", 1.0)) * 1_000_000_000)
        self.recovery_delay_s = max(0.0, float(config.get("reconnect_recovery_delay_s", 2.0)))
        self.heartbeat_interval_s = max(0.1, float(config.get("heartbeat_interval_s", 30.0)))
        self.report_on_change = bool(config.get("report_on_change", True))
        self.publish_failure_threshold = max(1, int(config.get("publish_failure_threshold", 3)))
        self.failsafe_targets = [dict(item) for item in config.get("failsafe_targets", [])]
        if self.transport_enabled:
            self.client = MqttClient(self.config, self._on_downlink, self._on_connection_state)

    def _on_downlink(self, payload):
        # Management has an independent signed contract and is deliberately
        # excluded from the legacy timestamp/target conversion. Never infer
        # authorization from an MQTT controller name or a "user approved" bit.
        if payload.get("msg_type") == "management_cmd":
            if payload.get("device_id") != self.default_device_id:
                raise ValueError("management packet targets a different device")
            envelope = payload.get("envelope")
            if not isinstance(envelope, dict):
                raise ValueError("management_cmd requires a signed envelope")
            self.emit_downlink("uran.management.command.v1", envelope,
                str(envelope.get("payload", {}).get("request_id") or ""),
                {"device_id": self.default_device_id, "component_id": "management",
                 "instance_id": str(self.config.get("management_instance_id") or "uran_management"),
                 "capability_id": "management.device"})
            return
        if payload.get("msg_type") == "frpc_ctrl":
            self._publish_legacy({"msg_type": "command_rejected", "msg_version": "1.0",
                "device_id": self.default_device_id, "timestamp_ns": now_ns(),
                "reason": "frpc_requires_signed_user_grant", "original_msg_type": "frpc_ctrl"})
            return
        if time.monotonic() < self._commands_enabled_at:
            self._publish_legacy({
                "msg_type": "command_rejected", "msg_version": "1.0",
                "device_id": self.default_device_id, "timestamp_ns": now_ns(),
                "reason": "mqtt_reconnect_recovery_delay",
                "original_msg_type": str(payload.get("msg_type") or ""),
            })
            return
        with self._sequence_lock:
            self._sequence += 1
            sequence = self._sequence
        received_at_ns = now_ns()
        operation_id = str(payload.get("operation_id") or payload.get("request_id") or
                           f"mqtt-{received_at_ns}-{sequence}")
        default_component_id = str(payload.get("component_id") or self.default_component_id)
        if payload.get("msg_type") == "media_ctrl" and not payload.get("component_id"):
            default_component_id = self.media_component_id
        message = self.codec.decode_public(
            payload,
            operation_id=operation_id,
            sequence=int(payload.get("sequence") or sequence),
            default_device_id=self.default_device_id,
            default_component_id=default_component_id,
            validity_window_ns=self.validity_window_ns,
            received_at_ns=received_at_ns,
            component_types=self.config.get("component_types", {"base": "base", "arm": "arm"}),
        )
        if payload.get("msg_type") == "state_query":
            requested = message["payload"].get("field_names") or []
            with self._state_lock:
                fields = ({key: value for key, value in self._state.items() if key in requested}
                          if requested else dict(self._state))
            self._publish_legacy({
                "msg_type": "state_query_response", "msg_version": "1.0",
                "device_id": self.default_device_id, "timestamp_ns": received_at_ns,
                "fields": fields,
            })
            return
        capability = {
            "uran.motion.command.v1": {
                "base_velocity": "motion.base.velocity",
                "base_posture": "motion.base.posture",
                "arm_pose": "motion.arm.pose",
                "arm_cartesian_velocity": "motion.arm.cartesian_velocity",
                "gripper_action": "motion.gripper.action",
                "safety_action": "motion.base.safety",
            }.get(str(message["payload"].get("payload", {}).get("kind") or ""), ""),
            "uran.tasks.command.v1": "tasks.lifecycle",
            "uran.media.session-command.v1": "media.session",
            "uran.legacy.control-switch.v1": "state.policy.control",
            "uran.legacy.param-update.v1": "state.policy.parameters",
            "uran.legacy.frpc-ctrl.v1": "network.frpc.control",
        }.get(message["message_type"], "")
        target = dict(message["payload"].get("target") or {})
        if message["payload"].get("payload", {}).get("kind") == "safety_action" and self.config.get("component_types", {"arm": "arm"}).get(default_component_id) == "arm":
            capability = "motion.arm.safety"
        if capability:
            target["capability_id"] = capability
        self.emit_downlink(message["message_type"], message["payload"],
                           message["correlation_id"], target)

    def activate(self) -> None:
        self._active = True

    def deactivate(self) -> None:
        self._active = False
        self.disconnect()

    def connect(self) -> None:
        if not self.client:
            raise RuntimeError("MQTT transport is disabled")
        if not self.client.connect():
            raise RuntimeError("MQTT broker connection failed")
        result = self.client.register(timeout_s=float(self.config.get("register_timeout_s", 10)))
        if result not in {"registered", "auto_registered", "legacy_connected"}:
            self.client.disconnect()
            raise RuntimeError(f"MQTT registration failed: {result}")
        self._register_connection_state()
        self._heartbeat_stop.clear()
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

    def disconnect(self) -> None:
        self._heartbeat_stop.set()
        if self.client:
            self.client.disconnect()
        if (self._heartbeat_thread and self._heartbeat_thread.is_alive() and
                self._heartbeat_thread is not threading.current_thread()):
            self._heartbeat_thread.join(timeout=2.0)
        self._heartbeat_thread = None

    def publish(self, message: Mapping[str, Any]) -> Mapping[str, Any]:
        if not self.client:
            raise RuntimeError("MQTT transport is disabled")
        source = dict(message.get("source") or {})
        payload = self.codec.encode_uplink(
            device_id=str(source.get("device_id") or self.default_device_id),
            source=str(source.get("component_id") or source.get("instance_id") or "runtime"),
            data_type=str(message.get("message_type") or "unknown"),
            timestamp_ns=now_ns(),
            payload=message.get("payload"),
        )
        if message.get("correlation_id"):
            payload["correlation_id"] = str(message["correlation_id"])
        if not self.client.publish_uplink(payload):
            self._record_publish_failure()
            raise RuntimeError("MQTT publish failed")
        self._publish_failures = 0
        return {"status": "succeeded", "payload": {"published": True}}

    def subscribe(self, subscription_id: str, field_prefix: str) -> None:
        del subscription_id, field_prefix

    def on_state(self, subscription_id: str, field: str,
                 value: Mapping[str, Any], sequence: int) -> None:
        del subscription_id, sequence
        item = value.get("value")
        with self._state_lock:
            changed = self._state.get(field) != item
            self._state[field] = item
        if changed and self.report_on_change and self.client and self.client.is_registered():
            self._publish_state_snapshot(reason="state_changed", changed_fields=[field])

    @staticmethod
    def _field_segment(value: str) -> str:
        normalized = re.sub(r"[^A-Za-z0-9_]", "_", str(value))
        return "id_" + (normalized or "unknown")

    def _register_connection_state(self) -> None:
        instance = self._field_segment(str(self.identity.get("instance_id") or "mqtt"))
        self._connection_field = f"plugin.{instance}.connection.connected"
        self.register_state(self._connection_field, "bool", persistent=False)
        self.set_state(self._connection_field, True, persistent=False)

    def _on_connection_state(self, connected: bool, reason_code: int) -> None:
        if connected:
            self._commands_enabled_at = time.monotonic() + self.recovery_delay_s
            self._publish_failures = 0
        else:
            self._commands_enabled_at = float("inf")
            if self._active and reason_code != 0:
                self._dispatch_failsafe("mqtt_disconnected")
        if hasattr(self, "_connection_field"):
            try:
                self.set_state(self._connection_field, connected, persistent=False)
            except Exception:
                pass

    def _dispatch_failsafe(self, reason: str) -> None:
        now_ns_value = now_ns()
        for index, configured in enumerate(self.failsafe_targets, start=1):
            target = {key: str(configured.get(key) or "") for key in
                      ("device_id", "component_id", "instance_id", "capability_id")}
            target = {key: value for key, value in target.items() if value}
            if not target.get("device_id") or not target.get("component_id"):
                continue
            operation_id = f"mqtt-failsafe-{now_ns_value}-{index}"
            command_target = {"device_id": target["device_id"],
                              "component_id": target["component_id"]}
            try:
                self.dispatch_local({
                    "message_type": "uran.motion.command.v1",
                    "correlation_id": operation_id,
                    "target": target,
                    "payload": {
                        "schema_version": "1.0", "operation_id": operation_id,
                        "target": command_target, "controller": "mqtt-failsafe",
                        "sequence": now_ns_value + index, "issued_at_ns": now_ns_value,
                        "valid_until_ns": now_ns_value + 1_000_000_000,
                        "payload": {"kind": "safety_action", "action": "stop",
                                    "reason": reason},
                        "metadata": {"source": "mqtt-failsafe"},
                    },
                })
            except Exception:
                continue

    def _record_publish_failure(self) -> None:
        self._publish_failures += 1
        if self._publish_failures == self.publish_failure_threshold:
            self._dispatch_failsafe("mqtt_publish_failures")

    def _publish_legacy(self, payload: Mapping[str, Any]) -> bool:
        if not self.client or not self.client.publish_uplink(dict(payload)):
            self._record_publish_failure()
            return False
        self._publish_failures = 0
        return True

    def _publish_state_snapshot(self, *, reason: str, changed_fields=None) -> bool:
        try:
            document = dict(self.runtime_snapshot())
        except Exception:
            # Keep standalone SDK tests and older hosts functional.  A Runtime
            # host exposes the canonical document and therefore does not use
            # this fallback in production.
            with self._state_lock:
                fields = dict(self._state)
            document = {"schema_version": "1.0", "components": [],
                        "unscoped_fields": dict(fields), "fields": fields}
        fields = dict(document.get("fields") or {})
        battery_level = fields.pop("battery_level", None)
        # The Runtime document is authoritative.  Keep every component entry
        # belonging to this MQTT worker's device so every component ID on the
        # device is visible without leaking another device's state.
        components = [dict(item) for item in (document.get("components") or [])
                      if isinstance(item, Mapping) and
                      (not item.get("device_id") or
                       str(item.get("device_id")) == self.default_device_id)]
        if isinstance(battery_level, (int, float)) and not isinstance(battery_level, bool):
            # Battery is device-wide, but expose it in every component entry
            # so consumers that parse components independently can display it.
            for component in components:
                component_fields = dict(component.get("fields") or {})
                component_fields["battery_level"] = float(battery_level)
                component["fields"] = component_fields
        return self._publish_legacy({
            "msg_type": "state_snapshot", "msg_version": "1.0",
            "device_id": self.default_device_id, "timestamp_ns": now_ns(),
            "reason": reason, "changed_fields": list(changed_fields or []),
            "schema_version": str(document.get("schema_version") or "1.0"),
            "components": components,
            "unscoped_fields": dict(document.get("unscoped_fields") or {}),
            "fields": fields,
        })

    def _heartbeat_loop(self) -> None:
        while not self._heartbeat_stop.wait(self.heartbeat_interval_s):
            if not self.client or not self.client.is_connected():
                continue
            self._publish_legacy({
                "msg_type": "heartbeat", "msg_version": "1.0",
                "device_id": self.default_device_id, "timestamp_ns": now_ns(),
            })
            self._publish_state_snapshot(reason="periodic")

    def invoke(self, request: Mapping[str, Any]) -> Mapping[str, Any]:
        message_type = request.get("message_type")
        payload = dict(request.get("payload") or {})
        if message_type == "uran.protocol.mqtt.decode.v1":
            result = self.codec.decode(payload)
        elif message_type == "uran.protocol.mqtt.publish.v1":
            if not self.client or not self.client.publish_uplink(payload):
                raise RuntimeError("MQTT transport is unavailable or publish failed")
            result = {"published": True}
        else:
            raise ValueError("unsupported message_type")
        return {"status": "succeeded", "operation_id": request["operation_id"],
                "payload": result}


def worker_factory() -> Worker:
    return MqttWorker()


def main() -> int:
    return WorkerServer(worker_factory()).serve()
