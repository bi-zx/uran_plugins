import json
import os
import time

from uran_worker_sdk import WorkerServer
from uran_so101_plugin.calibration import (apply_safety_range, empty_calibration,
                                           is_complete, validate_raw_targets,
                                           values_to_raw)
from uran_so101_plugin.kinematics import SO101Kinematics
from uran_so101_plugin.worker import worker_factory


ROOT = os.path.dirname(os.path.dirname(__file__))
URDF = os.path.join(ROOT, "config", "so101_chain.urdf")
RANGE = os.path.join(ROOT, "config", "so101_follower_com9.json")


def _frame(op, request_id, **kwargs):
    return {"kind": "request", "request_id": request_id, "op": op, **kwargs}


def test_fk_ik_round_trip():
    kin = SO101Kinematics(URDF)
    joints = [0.0, 15.0, -25.0, 30.0, 10.0]
    target = kin.forward(joints)
    result = kin.inverse_position([target[0][3] * 1000, target[1][3] * 1000, target[2][3] * 1000], joints)
    assert result["success"]
    assert result["position_error_mm"] < 1.5


def test_safety_range_is_complete_and_rejects_outside():
    data = apply_safety_range(empty_calibration(), RANGE)
    assert is_complete(data)
    safe = [0.0] * 5 + [50.0]
    values_to_raw(data, safe)
    try:
        validate_raw_targets(data, [99999] * 6)
    except ValueError:
        pass
    else:
        raise AssertionError("out-of-range joint was accepted")


def test_simulation_worker_arm_command():
    now = time.time_ns()
    command = {"message_type": "uran.motion.command.v1", "payload": {
        "schema_version": "1.0", "operation_id": "op-1",
        "target": {"device_id": "dog", "component_id": "arm"},
        "controller": "test", "sequence": 1, "issued_at_ns": now,
        "valid_until_ns": now + 1000000000,
        "payload": {"kind": "safety_action", "action": "stop"}}}
    import io
    source = io.BytesIO((json.dumps(_frame("hello", "h", protocol="uran.worker.jsonl.v1", identity={})) + "\n" +
                         json.dumps(_frame("configure", "c", config={"backend": "simulation", "device_id": "dog", "component_id": "arm", "state_reporting": False})) + "\n" +
                         json.dumps(_frame("activate", "a")) + "\n" +
                         json.dumps(_frame("invoke", "i", operation_id="op-1", **command)) + "\n" +
                         json.dumps(_frame("shutdown", "s")) + "\n").encode())
    output = io.BytesIO()
    assert WorkerServer(worker_factory(), source, output).serve() == 0
    replies = [json.loads(line) for line in output.getvalue().splitlines()]
    assert replies[3]["status"] == "succeeded"
