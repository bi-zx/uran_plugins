import pytest

from uran_cyberdog2_plugin import MotionCommand, MotionController, SimulatedBackend


def test_stop_bypasses_failsafe_mode_and_control_lock_and_zeros_velocity():
    backend = SimulatedBackend()
    controller = MotionController(backend)
    controller.failsafe_active = True
    controller.control_mode = "manual"
    controller.lock = {"active": True, "owner": "task"}
    result = controller.execute(MotionCommand(action="stop", controller="auto",
                                              linear_vel_x=0.8, command_seq=8))
    assert result.accepted
    assert result.applied == {"linear_x": 0.0, "linear_y": 0.0,
                              "linear_z": 0.0, "angular_z": 0.0}
    assert controller.last_executed_seq == 8


def test_velocity_is_rejected_during_failsafe():
    controller = MotionController(SimulatedBackend())
    controller.failsafe_active = True
    result = controller.execute(MotionCommand(linear_vel_x=0.4, command_seq=7))
    assert not result.accepted
    assert result.code == "E_FAILSAFE_ACTIVE"
    assert controller.last_rejected_seq == 7


def test_non_finite_velocity_is_rejected_before_backend():
    backend = SimulatedBackend()
    controller = MotionController(backend)
    result = controller.execute(MotionCommand(linear_vel_x=float("nan")))
    assert not result.accepted
    assert result.code == "E_INVALID_NUMBER"
    assert backend.commands == []


def test_velocity_vector_is_clamped_without_changing_direction():
    backend = SimulatedBackend()
    controller = MotionController(backend, linear_limit=1.0, angular_limit=0.5)
    result = controller.execute(MotionCommand(linear_vel_x=3.0, linear_vel_y=4.0,
                                              angular_vel_z=2.0))
    assert result.applied == pytest.approx({"linear_x": 0.6, "linear_y": 0.8,
                                           "linear_z": 0.0, "angular_z": 0.5})


def test_failsafe_stops_after_timeout_and_recovers_after_delay():
    backend = SimulatedBackend()
    controller = MotionController(backend, failsafe_timeout_s=2, recover_delay_s=1)
    controller.heartbeat(False, now=10)
    assert controller.tick(now=11) is None
    assert controller.tick(now=12).accepted
    assert controller.failsafe_active
    controller.heartbeat(True, now=13)
    controller.tick(now=14)
    assert not controller.failsafe_active
