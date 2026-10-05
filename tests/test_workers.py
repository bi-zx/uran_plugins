import io
import json
import time

from uran_worker_sdk import WorkerServer

from uran_autotask_plugin.worker import worker_factory as autotask_factory
from uran_cyberdog2_plugin.worker import worker_factory as cyberdog_factory
from uran_media_plugin.worker import worker_factory as media_factory
from uran_mqtt_plugin.worker import MqttWorker, worker_factory as mqtt_factory
from uran_px4_plugin.worker import worker_factory as px4_factory


def _run(factory, frames):
    source = io.BytesIO(b"".join(
        json.dumps(frame, separators=(",", ":")).encode() + b"\n" for frame in frames))
    output = io.BytesIO()
    assert WorkerServer(factory(), source, output).serve() == 0
    return [json.loads(line) for line in output.getvalue().splitlines()]


def _lifecycle(config, invoke):
    return [
        {"kind": "request", "request_id": "h", "op": "hello",
         "protocol": "uran.worker.jsonl.v1", "identity": {}},
        {"kind": "request", "request_id": "c", "op": "configure", "config": config},
        {"kind": "request", "request_id": "a", "op": "activate"},
        {"kind": "request", "request_id": "i", "op": "invoke",
         "operation_id": "op-1", **invoke},
        {"kind": "request", "request_id": "d", "op": "deactivate"},
        {"kind": "request", "request_id": "s", "op": "shutdown"},
    ]


def test_cyberdog_worker_executes_typed_simulation_motion():
    replies = _run(cyberdog_factory, _lifecycle({"backend": "simulation", "state_reporting": False}, {
        "message_type": "uran.motion.command.v1",
        "payload": {"schema_version": "1.0", "operation_id": "op-1",
                    "target": {"device_id": "dog", "component_id": "base"},
                    "controller": "cloud", "sequence": 9, "issued_at_ns": time.time_ns(),
                    "valid_until_ns": time.time_ns() + 1_000_000_000,
                    "payload": {"kind": "safety_action", "action": "stop"}},
    }))
    assert replies[3]["ok"] is True
    assert replies[3]["status"] == "succeeded"
    assert replies[3]["payload"]["backend"]["simulation"] is True


def test_media_worker_requires_explicit_simulator_or_real_backend():
    replies = _run(media_factory, _lifecycle({"state_reporting": False}, {
        "message_type": "uran.media.session-command.v1",
        "payload": {"schema_version": "1.0", "operation_id": "op-1",
                    "target": {"device_id": "dog", "component_id": "camera"},
                    "session_id": "s1", "channel_id": "front",
                    "action": "start", "protocol": "webrtc", "timestamp_ns": 1},
    }))
    assert replies[3]["status"] == "failed"
    assert replies[3]["payload"]["code"] == "E_BACKEND_UNAVAILABLE"


def test_autotask_worker_accepts_and_pauses_waypoint_task():
    replies = _run(autotask_factory, _lifecycle({"backend": "simulation", "allow_unarbitrated_simulation": True, "state_reporting": False}, {
        "message_type": "uran.tasks.command.v1",
        "payload": {"schema_version": "1.0", "operation_id": "op-1",
                    "target": {"device_id": "dog", "component_id": "base"},
                    "timestamp_ns": 1, "task_id": "t1", "action": "start", "task_type": "waypoint",
                    "parameters": {"waypoints": [{"lat": 39, "lon": 116}]}},
    }))
    # Without a configured geo reference the mission state machine pauses
    # safely instead of pretending that navigation has started.
    assert replies[3]["payload"]["stage"] == "paused"


def test_mqtt_worker_decodes_control_state_param_and_media_boundaries():
    for cloud in (
        {"msg_type": "control_switch", "switch": {"control_mode": "auto"}},
        {"msg_type": "state_query", "field_names": ["battery_level"]},
        {"msg_type": "param_update", "params_json": {"linear_vel_limit": 0.3}},
        {"msg_type": "media_ctrl", "action": "start", "protocol": "webrtc",
         "channel_id": "front"},
    ):
        replies = _run(mqtt_factory, _lifecycle({"transport_enabled": False}, {
            "message_type": "uran.protocol.mqtt.decode.v1", "payload": cloud,
        }))
        assert replies[3]["ok"] is True
        assert replies[3]["payload"]["message_type"] == cloud["msg_type"]


def test_mqtt_state_snapshot_preserves_all_runtime_components():
    worker = MqttWorker()
    worker.configure({"transport_enabled": False, "device_id": "robot"})

    published = []

    class Client:
        def publish_uplink(self, payload):
            published.append(payload)
            return True

    worker.client = Client()
    worker.runtime_snapshot = lambda: {
        "schema_version": "1.0",
        "device_id": "robot",
        "components": [
            {"device_id": "robot", "component_id": "base", "instances": [], "fields": {}},
            {"device_id": "robot", "component_id": "arm", "instances": [], "fields": {}},
        ],
        "unscoped_fields": {},
        "fields": {},
    }

    assert worker._publish_state_snapshot(reason="periodic") is True
    assert [item["component_id"] for item in published[0]["components"]] == ["base", "arm"]


def test_px4_worker_simulation_requires_trigger_before_motion():
    now = time.time_ns()
    command = {"schema_version": "1.0", "operation_id": "op-1",
               "target": {"device_id": "drone", "component_id": "aerial"},
               "controller": "cloud", "sequence": 1, "issued_at_ns": now,
               "valid_until_ns": now + 1_000_000_000,
               "payload": {"kind": "base_velocity", "frame_id": "base_link",
                            "linear_mps": {"x": 0.2, "y": 0, "z": 0},
                            "angular_rps": {"x": 0, "y": 0, "z": 0}}}
    replies = _run(px4_factory, _lifecycle({"backend": "simulation",
                                             "device_id": "drone",
                                             "component_id": "aerial"}, {
        "message_type": "uran.motion.command.v1", "payload": command,
    }))
    assert replies[3]["status"] == "failed"
    assert replies[3]["payload"]["code"] == "E_NOT_READY"
