import pytest

from uran_simulation_plugin import MultiComponentRobot


def test_dual_arm_and_gripper_state_is_isolated():
    robot = MultiComponentRobot([
        {"device_id": "robot", "component_id": "arm_left", "kind": "arm"},
        {"device_id": "robot", "component_id": "arm_right", "kind": "arm"},
        {"device_id": "robot", "component_id": "gripper", "kind": "gripper"},
    ])
    robot.execute({"device_id": "robot", "component_id": "arm_left"},
                  {"kind": "arm_pose", "position_m": [0.2, 0.0, 0.25]})
    robot.execute({"device_id": "robot", "component_id": "arm_right"},
                  {"kind": "arm_pose", "position_m": [0.1, 0.0, 0.2]})
    state = robot.snapshot()
    assert state["robot/arm_left"]["state"]["position_m"] == [0.2, 0.0, 0.25]
    assert state["robot/arm_right"]["state"]["position_m"] == [0.1, 0.0, 0.2]
    assert state["robot/gripper"]["state"] == {}


def test_command_for_wrong_component_type_fails():
    robot = MultiComponentRobot([
        {"device_id": "robot", "component_id": "gripper", "kind": "gripper"}])
    with pytest.raises(ValueError):
        robot.execute({"device_id": "robot", "component_id": "gripper"},
                      {"kind": "base_velocity"})


def test_gripper_toggle_is_single_action():
    robot = MultiComponentRobot([
        {"device_id": "robot", "component_id": "gripper", "kind": "gripper"}])
    result = robot.execute({"device_id": "robot", "component_id": "gripper"},
                           {"kind": "gripper_action", "action": "toggle"})
    assert result["applied"]["action"] == "toggle"
