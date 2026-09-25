import math

import pytest

from uran_px4_plugin import MotionCommand, Px4Controller


class Transport:
    def __init__(self):
        self.positions = []
        self.actions = []

    def publish_position(self, command): self.positions.append(command)
    def publish_takeoff_land(self, action): self.actions.append(action)


def test_body_velocity_is_rotated_into_world_setpoint():
    transport = Transport()
    controller = Px4Controller(transport, clock=lambda: 1.0, linear_limit=3.0)
    controller.on_traj_start(0.0, 0.0, 2.0, math.pi / 2)
    result = controller.execute(MotionCommand(linear_x=1.0), now=2.0)
    assert result.accepted
    assert result.applied["position"]["x"] == 0.0
    assert result.applied["position"]["y"] == 0.0
    assert result.applied["velocity"]["x"] == pytest.approx(0.0)
    assert result.applied["velocity"]["y"] == pytest.approx(1.0)


def test_takeoff_requires_safe_rc_and_velocity_waits_for_trigger():
    transport = Transport()
    controller = Px4Controller(transport)
    assert not controller.execute(MotionCommand(action="takeoff")).accepted
    controller.update_rc([0, 0, 1500, 0, 2000, 2000])
    assert controller.execute(MotionCommand(action="takeoff")).accepted
    assert transport.actions == ["takeoff"]
    assert controller.execute(MotionCommand(linear_x=1.0)).code == "E_NOT_READY"


def test_keepalive_reverts_to_zero_after_command_timeout():
    now = [0.0]
    transport = Transport()
    controller = Px4Controller(transport, clock=lambda: now[0], cmd_timeout_s=0.5)
    controller.on_traj_start(0.0, 0.0, 1.0, 0.0)
    controller.execute(MotionCommand(linear_x=1.0), now=0.0)
    now[0] = 1.0
    result = controller.tick()
    assert result and result.applied["velocity"]["x"] == 0.0


def test_land_is_delayed_until_position_command_quiet_period():
    now = [0.0]
    transport = Transport()
    controller = Px4Controller(transport, clock=lambda: now[0], land_retry_delay_s=0.6)
    controller.on_traj_start(0.0, 0.0, 1.0, 0.0)
    controller.execute(MotionCommand(action="land"), now=0.0)
    assert transport.actions == []
    now[0] = 0.7
    controller.tick()
    assert transport.actions == ["land"]
