"""Installed native Runtime + actual plugin workers; no sibling source imports."""
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

import pytest
from uran_interfaces import ContractValidator


def plugin(instance, component, module, types, capabilities, config):
    return {"instance_id": instance, "device_id": "robot", "component_id": component,
            "class_type": "uran_worker_adapter/WorkerProcessPlugin",
            "config": {"command": [sys.executable, "-m", "uran_worker_sdk",
                                   module + ":worker_factory"],
                       "message_types": types, "capabilities": capabilities,
                       "request_timeout_ms": 1500, "worker_config": config}}


class Runtime:
    def __init__(self, prefix, path):
        self.events = []
        self.lines = queue.Queue()
        self.validator = ContractValidator()
        self.process = subprocess.Popen(
            [str(prefix / "bin/uran_runtime"), "--config", str(path), "--manifest",
             str(prefix / "share/uran_framework/plugins/worker/plugin.json")],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1)
        def read():
            for line in self.process.stdout:
                self.lines.put(json.loads(line))
            self.lines.put(None)
        threading.Thread(target=read, daemon=True).start()

    def request(self, **frame):
        self.process.stdin.write(json.dumps(frame) + "\n")
        self.process.stdin.flush()
        while True:
            value = self.lines.get(timeout=8)
            assert value is not None, self.process.stderr.read()
            if "ok" in value:
                return value
            self.events.append(value)

    def dispatch(self, kind, operation_id, payload, instance, capability=""):
        return self.request(op="dispatch", message_type=kind, correlation_id=operation_id,
                            payload=payload, target={"instance_id": instance,
                                                     "capability_id": capability})

    def result(self, operation_id, statuses=("succeeded", "failed", "rejected", "cancelled")):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            self.request(op="status")
            for event in self.events:
                result = event.get("payload", {})
                if (event.get("message_type") == "uran.operations.state.v1" and
                        result.get("operation_id") == operation_id and
                        result.get("status") in statuses):
                    self.validator.validate("operations/state.schema.json", result)
                    return result
            time.sleep(0.02)
        raise AssertionError(f"no result for {operation_id}: {self.events}")


@pytest.fixture
def runtime(tmp_path):
    configured = os.environ.get("URAN_RUNTIME_PREFIX")
    if not configured:
        pytest.skip("set URAN_RUNTIME_PREFIX to an installed framework artifact")
    prefix = Path(configured)
    assert (prefix / "bin/uran_runtime").is_file()
    plugins = [
        plugin("simulation", "base", "uran_simulation_plugin.worker",
               ["uran.motion.command.v1"], ["motion.base.velocity", "motion.base.safety"],
               {"required_arbiter_instance": "arbiter", "components": [
                   {"device_id": "robot", "component_id": "base", "kind": "base"}]}),
        plugin("arbiter", "control", "uran_control_arbiter_plugin.worker",
               ["uran.motion.command.v1", "uran.control-lease.acquire.v1",
                "uran.control-lease.release.v1", "uran.control-lease.query.v1"],
               ["motion.control.acquire", "motion.control.release", "motion.control.gateway"],
               {"backend_instances": {"base": "simulation"}}),
        plugin("task", "tasks", "uran_autotask_plugin.worker",
               ["uran.tasks.command.v1", "uran.operations.cancel.v1"], ["tasks.waypoint"],
               {"backend": "simulation", "arbiter_instance": "arbiter"}),
    ]
    path = tmp_path / "runtime.json"
    path.write_text(json.dumps({"plugins": plugins}))  # JSON is valid YAML.
    host = Runtime(prefix, path)
    try:
        status = host.request(op="status")
        assert status["result"]["ready"]
        providers = {item["instance_id"]: item["capabilities"] for item in status["result"]["plugins"]}
        assert "motion.base.velocity" in providers["simulation"]
        assert "tasks.waypoint" in providers["task"]
        yield host
        assert host.request(op="shutdown")["ok"]
        host.process.stdin.close()
        assert host.process.wait(timeout=8) == 0, host.process.stderr.read()
    finally:
        if host.process.poll() is None:
            host.process.terminate()
            host.process.wait(timeout=8)


def lease(runtime, duration=20):
    now = time.time_ns()
    doc = {"schema_version": "1.0", "lease_id": "lease", "owner": "client", "priority": 50,
           "resource": {"device_id": "robot", "component_id": "base", "resource_group": "base-motion"},
           "acquired_at_ns": now, "expires_at_ns": now + int(duration * 1e9)}
    assert runtime.dispatch("uran.control-lease.acquire.v1", "acquire", doc, "arbiter")["ok"]
    assert runtime.result("acquire")["status"] == "succeeded"
    return doc


def motion(operation_id, kind=None):
    now = time.time_ns()
    return {"schema_version": "1.0", "operation_id": operation_id,
            "target": {"device_id": "robot", "component_id": "base"}, "controller": "client",
            "lease_id": "lease", "sequence": now, "issued_at_ns": now,
            "valid_until_ns": now + 2_000_000_000, "payload": kind or {
                "kind": "base_velocity", "frame_id": "base_link",
                "linear_mps": {"x": 0.2, "y": 0, "z": 0}, "angular_rps": {"x": 0, "y": 0, "z": 0}}}


def test_backend_result_and_rejection_return_through_arbiter(runtime):
    lease(runtime)
    assert runtime.dispatch("uran.motion.command.v1", "move", motion("move"), "arbiter")["ok"]
    result = runtime.result("move")
    assert result["status"] == "succeeded"
    assert result["payload"]["simulation"] is True
    # A stale command is accepted by the gateway and rejected by the backend;
    # the terminal failure must return without killing the arbiter.
    stale = motion("stale")
    stale["sequence"] = 0
    assert runtime.dispatch("uran.motion.command.v1", "stale", stale, "arbiter")["ok"]
    assert runtime.result("stale")["status"] == "rejected"
    assert runtime.dispatch("uran.motion.command.v1", "again", motion("again"), "arbiter")["ok"]
    assert runtime.result("again")["status"] == "succeeded"


def test_task_calls_arbiter_and_backend_then_cancels(runtime):
    command = {"schema_version": "1.0", "operation_id": "task-start", "task_id": "route",
               "target": {"device_id": "robot", "component_id": "base"},
               "action": "start", "task_type": "waypoint", "timestamp_ns": time.time_ns(),
               "parameters": {"waypoints": [{"lat": 39, "lon": 116}]}}
    assert runtime.dispatch("uran.tasks.command.v1", "task-start", command, "task")["ok"]
    assert runtime.result("task-start", ("running",))["payload"]["stage"] == "paused"
    snapshot = runtime.request(op="snapshot")["result"]
    assert snapshot["plugin.simulation.last_command"]["applied"]["action"] == "stop"
    assert runtime.dispatch("uran.operations.cancel.v1", "cancel-request",
                            {"operation_id": "task-start"}, "task")["ok"]
    assert runtime.result("task-start")["status"] == "cancelled"


def test_expired_lease_stops_backend(runtime):
    lease(runtime, duration=0.2)
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        state = runtime.request(op="snapshot")["result"].get("plugin.simulation.last_command", {})
        if state.get("applied", {}).get("reason") == "lease_expired":
            break
        time.sleep(0.02)
    assert state["applied"]["action"] == "stop"
