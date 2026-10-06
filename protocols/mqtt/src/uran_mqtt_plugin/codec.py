"""Strict boundary conversion for the legacy 1.0 cloud protocol."""

import json
import math
import time
from typing import Any, Dict, Mapping, Optional

from uran_interfaces import ComponentTarget
from uran_worker_sdk import now_ns


class DecodeError(ValueError):
    """A cloud payload cannot be converted without guessing its meaning."""


class CloudCodec:
    SUPPORTED = {
        "control_switch",
        "move_cmd",
        "task_ctrl",
        "media_ctrl",
        "frpc_ctrl",
        "param_update",
        "state_query",
    }

    @staticmethod
    def _text(payload: Mapping[str, Any], key: str, default: str = "") -> str:
        value = payload.get(key, default)
        return default if value is None else str(value)

    @staticmethod
    def _number(payload: Mapping[str, Any], key: str, default: float = 0.0) -> float:
        value = payload.get(key, default)
        if value in (None, ""):
            return default
        try:
            result = float(value)
            if not math.isfinite(result):
                raise ValueError("non-finite")
            return result
        except (TypeError, ValueError) as exc:
            raise DecodeError(f"{key} must be numeric") from exc

    @staticmethod
    def timestamp_ns(payload: Mapping[str, Any]) -> int:
        for key, multiplier in (("timestamp_ns", 1), ("timestamp_ms", 1_000_000)):
            value = payload.get(key)
            if value not in (None, ""):
                try:
                    if isinstance(value, bool):
                        raise ValueError("boolean")
                    if isinstance(value, int):
                        parsed = value
                    elif isinstance(value, float):
                        # JavaScript clients commonly serialize large integer
                        # timestamps as JSON numbers (and some serializers use
                        # exponent notation).  Preserve them when they are
                        # finite integral values; fractional timestamps remain
                        # invalid.
                        if not math.isfinite(value) or not value.is_integer():
                            raise ValueError("not an integer")
                        parsed = int(value)
                    elif isinstance(value, str):
                        text = value.strip()
                        if not text:
                            raise ValueError("not an integer")
                        try:
                            parsed = int(text, 10)
                        except ValueError:
                            # Accept exponent/decimal spellings only when the
                            # represented value is an exact integer.
                            numeric = float(text)
                            if not math.isfinite(numeric) or not numeric.is_integer():
                                raise ValueError("not an integer")
                            parsed = int(numeric)
                    else:
                        raise ValueError("not an integer")
                    if parsed < 0:
                        raise ValueError("negative")
                    return parsed * multiplier
                except (TypeError, ValueError) as exc:
                    raise DecodeError(f"{key} must be an integer timestamp") from exc
        return 0

    def decode(self, payload: Mapping[str, Any]) -> Dict[str, Any]:
        if not isinstance(payload, Mapping):
            raise DecodeError("downlink payload must be an object")
        message_type = self._text(payload, "msg_type").strip()
        if message_type not in self.SUPPORTED:
            raise DecodeError(f"unsupported msg_type: {message_type or '<empty>'}")

        target = ComponentTarget(
            device_id=self._text(payload, "device_id"),
            component_id=self._text(payload, "component_id"),
            instance_id=self._text(payload, "instance_id"),
        ).to_dict()
        body: Dict[str, Any] = {
            "msg_version": self._text(payload, "msg_version", "1.0"),
            "timestamp_ns": self.timestamp_ns(payload),
        }
        if message_type == "move_cmd":
            body.update({
                "controller": self._text(payload, "controller"),
                "linear_vel_x": self._number(payload, "linear_vel_x"),
                "linear_vel_y": self._number(payload, "linear_vel_y"),
                "linear_vel_z": self._number(payload, "linear_vel_z"),
                "angular_vel_z": self._number(payload, "angular_vel_z"),
                "target_roll": self._number(payload, "target_roll"),
                "target_pitch": self._number(payload, "target_pitch"),
                "target_yaw": self._number(payload, "target_yaw"),
                "action": self._text(payload, "action"),
                "extra": self._json_value(payload.get("extra_json", {})),
            })
        elif message_type == "task_ctrl":
            params = self._json_value(payload.get("task_params_json", {}))
            task_type = self._text(payload, "task_type").strip()
            if isinstance(params, dict) and (not task_type or task_type == "default_task"):
                task_type = str(params.get("task_type") or params.get("type") or task_type)
            body.update({
                "task_id": self._text(payload, "task_id"),
                "action": self._text(payload, "action"),
                "task_type": task_type,
                "task_params": params,
            })
        elif message_type == "control_switch":
            switch = payload.get("switch", {})
            if not isinstance(switch, Mapping):
                raise DecodeError("switch must be an object")
            body["switch"] = dict(switch)
        elif message_type == "media_ctrl":
            # Existing clients use signal_json, while newer cloud paths may
            # carry the same object directly as signal.  Accept both forms
            # at the protocol boundary without changing the wire schema.
            signal_value = payload.get("signal_json", payload.get("signal", {}))
            body.update({
                "action": self._text(payload, "action"),
                "protocol": self._text(payload, "protocol"),
                "channel_id": self._text(payload, "channel_id"),
                "signal": self._json_value(signal_value),
            })
        elif message_type == "state_query":
            fields = payload.get("field_names", [])
            if not isinstance(fields, list):
                raise DecodeError("field_names must be an array")
            body["field_names"] = [str(item) for item in fields]
        else:
            body.update({key: value for key, value in payload.items()
                         if key not in {"msg_type", "msg_version", "device_id",
                                       "component_id", "instance_id"}})
        return {"message_type": message_type, "target": target, "payload": body}

    def decode_public(
        self,
        payload: Mapping[str, Any],
        *,
        operation_id: str,
        sequence: int,
        default_device_id: str,
        default_component_id: str,
        validity_window_ns: int = 1_000_000_000,
        received_at_ns: Optional[int] = None,
    ) -> Dict[str, Any]:
        """Convert a legacy cloud command into a routed platform message."""
        decoded = self.decode(payload)
        legacy_type = decoded["message_type"]
        legacy_body = decoded["payload"]
        target = dict(decoded["target"])
        target["device_id"] = target.get("device_id") or default_device_id
        target["component_id"] = target.get("component_id") or default_component_id
        target = {key: value for key, value in target.items() if value}
        if not target.get("device_id") or not target.get("component_id"):
            raise DecodeError("device_id and component_id are required for routing")

        timestamp_ns = int(legacy_body.get("timestamp_ns") or received_at_ns or now_ns())
        if legacy_type == "move_cmd":
            action = str(legacy_body.get("action") or "").strip().lower()
            if action:
                motion_payload: Dict[str, Any] = {
                    "kind": "safety_action",
                    "action": action,
                    "reason": "legacy_mqtt_command",
                }
            else:
                motion_payload = {
                    "kind": "base_velocity",
                    "frame_id": "base_link",
                    "linear_mps": {
                        "x": legacy_body["linear_vel_x"],
                        "y": legacy_body["linear_vel_y"],
                        "z": legacy_body["linear_vel_z"],
                    },
                    "angular_rps": {"x": 0.0, "y": 0.0,
                                    "z": legacy_body["angular_vel_z"]},
                    "posture_rad": {
                        "roll": legacy_body["target_roll"],
                        "pitch": legacy_body["target_pitch"],
                        "yaw": legacy_body["target_yaw"],
                    },
                }
            extra = legacy_body.get("extra")
            metadata: Dict[str, Any] = {
                "source": "legacy_mqtt",
                "motion_id": (extra or {}).get("motion_id") if isinstance(extra, Mapping) else None,
                "legacy_extra": extra,
            }
            metadata = {key: value for key, value in metadata.items() if value is not None}
            document = {
                "schema_version": "1.0",
                "operation_id": operation_id,
                "target": target,
                "controller": str(legacy_body.get("controller") or "cloud"),
                "sequence": int(sequence),
                "issued_at_ns": timestamp_ns,
                "valid_until_ns": timestamp_ns + max(1, int(validity_window_ns)),
                "payload": motion_payload,
                "metadata": metadata,
            }
            message_type = "uran.motion.command.v1"
        elif legacy_type == "task_ctrl":
            document = {
                "schema_version": "1.0",
                "operation_id": operation_id,
                "task_id": str(legacy_body.get("task_id") or operation_id),
                "target": target,
                "action": str(legacy_body.get("action") or "").strip().lower(),
                "task_type": str(legacy_body.get("task_type") or "legacy_task"),
                "parameters": (dict(legacy_body.get("task_params") or {})
                               if isinstance(legacy_body.get("task_params"), Mapping) else {}),
                "timestamp_ns": timestamp_ns,
            }
            message_type = "uran.tasks.command.v1"
        elif legacy_type == "media_ctrl":
            legacy_action = str(legacy_body.get("action") or "").strip().lower()
            signal = legacy_body.get("signal")
            if legacy_action in {"offer", "answer", "candidate", "signal"}:
                action = "switch"
            elif legacy_action == "record":
                action = "record_start"
            else:
                action = legacy_action
            channel_id = str(legacy_body.get("channel_id") or "default")
            document = {
                "schema_version": "1.0",
                "operation_id": operation_id,
                "target": target,
                "session_id": str(payload.get("session_id") or channel_id),
                "action": action,
                "protocol": str(legacy_body.get("protocol") or "webrtc").lower(),
                "channel_id": channel_id,
                "timestamp_ns": timestamp_ns,
            }
            if isinstance(signal, Mapping) and signal:
                document["signal"] = dict(signal)
            message_type = "uran.media.session-command.v1"
        else:
            document = {
                "schema_version": "1.0",
                "operation_id": operation_id,
                "target": target,
                "legacy_type": legacy_type,
                **legacy_body,
            }
            message_type = f"uran.legacy.{legacy_type.replace('_', '-')}.v1"

        return {
            "message_type": message_type,
            "payload": document,
            "correlation_id": operation_id,
        }

    @staticmethod
    def _json_value(value: Any) -> Any:
        if not isinstance(value, str):
            return value
        if not value:
            return {}
        try:
            return json.loads(value)
        except json.JSONDecodeError as exc:
            raise DecodeError("embedded JSON field is invalid") from exc

    @staticmethod
    def encode_uplink(*, device_id: str, source: str, data_type: str,
                      timestamp_ns: int, payload: Any,
                      preferred_protocol: str = "mqtt") -> Dict[str, Any]:
        return {
            "msg_type": "uplink_data",
            "msg_version": "1.0",
            "device_id": device_id,
            "source_pkg": source,
            "data_type": data_type,
            "timestamp_ns": int(timestamp_ns),
            "payload": payload,
            "payload_json": json.dumps(payload, ensure_ascii=False,
                                       separators=(",", ":")),
            "preferred_protocol": preferred_protocol,
        }
