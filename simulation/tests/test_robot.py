import pytest

from uran_simulation_plugin import MultiComponentRobot


def test_dual_arm_and_gripper_state_is_isolated():
    robot = MultiComponentRobot([
        {"device_id": "robot", "component_id": "arm_left", "kind": "arm"},
        {"device_id": "robot", "component_id": "arm_right", "kind": "arm"},
        {"device_id": "robot", "component_id": "gripper", "kind": "gripper"},
    ])
    robot.execute({"device_id": "robot", "component_id": "arm_left"},
                  {"kind": "arm_joints", "positions_rad": [0.5]})
    robot.execute({"device_id": "robot", "component_id": "arm_right"},
                  {"kind": "arm_joints", "positions_rad": [-0.5]})
    state = robot.snapshot()
    assert state["robot/arm_left"]["state"]["positions_rad"] == [0.5]
    assert state["robot/arm_right"]["state"]["positions_rad"] == [-0.5]
    assert state["robot/gripper"]["state"] == {}


def test_command_for_wrong_component_type_fails():
    robot = MultiComponentRobot([
        {"device_id": "robot", "component_id": "gripper", "kind": "gripper"}])
    with pytest.raises(ValueError):
        robot.execute({"device_id": "robot", "component_id": "gripper"},
                      {"kind": "base_velocity"})
