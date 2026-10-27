from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Iterable, Tuple


class MultiComponentRobot:
    def __init__(self, components: Iterable[Dict[str, Any]]):
        self._components = {}
        for item in components:
            key = (str(item["device_id"]), str(item["component_id"]))
            if key in self._components:
                raise ValueError(f"duplicate component: {key}")
            self._components[key] = {"kind": str(item["kind"]), "state": {}}

    def execute(self, target: Dict[str, Any], payload: Dict[str, Any]):
        key = (str(target.get("device_id", "")), str(target.get("component_id", "")))
        component = self._components.get(key)
        if component is None:
            raise KeyError(f"unknown component: {key}")
        kind = payload.get("kind")
        if component["kind"] == "base" and kind in {"base_velocity", "safety_action"}:
            component["state"] = deepcopy(payload)
        elif component["kind"] == "arm" and kind in {"arm_pose", "safety_action"}:
            component["state"] = deepcopy(payload)
        elif component["kind"] == "gripper" and kind in {"gripper_action", "safety_action"}:
            component["state"] = deepcopy(payload)
        else:
            raise ValueError(f"{kind} is not supported by {component['kind']}")
        return {"target": dict(target), "applied": deepcopy(payload), "simulation": True}

    def snapshot(self):
        return {f"{device}/{component}": deepcopy(value)
                for (device, component), value in self._components.items()}
