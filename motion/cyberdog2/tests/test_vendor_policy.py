from uran_cyberdog2_plugin.controller import MotionCommand, MotionResult
from uran_cyberdog2_plugin.vendor_policy import (
    CyberDogPolicyBackend, MOTION_RECOVERY_STAND, SWITCH_CHARGING,
    SWITCH_ESTOP, SWITCH_NORMAL,
)


class Transport:
    def __init__(self):
        self.servo = []
        self.motions = []
        self.result = MotionResult(True, "OK")
        self.on_motion = None

    def publish_servo(self, **values):
        self.servo.append(values)

    def motion_result(self, motion_id):
        self.motions.append(motion_id)
        if self.on_motion:
            self.on_motion(motion_id)
        return self.result


def test_timeout_loop_publishes_zero_velocity_at_keepalive_tick():
    now = [1.0]
    transport = Transport()
    backend = CyberDogPolicyBackend(transport, cmd_timeout_s=0.5, clock=lambda: now[0])
    backend.execute(MotionCommand(linear_vel_x=0.4))
    now[0] = 1.6
    backend.tick()
    assert transport.servo[-1]["vx"] == 0.0


def test_idle_zero_stands_once_and_retries_only_after_cooldown():
    now = [1.0]
    transport = Transport()
    backend = CyberDogPolicyBackend(
        transport, idle_to_stand_timeout_s=2.0, auto_stand_retry_interval_s=10.0,
        clock=lambda: now[0])
    backend.execute(MotionCommand())
    transport.result = MotionResult(False, "E_REJECTED")
    now[0] = 3.0
    backend.tick()
    assert transport.motions == [MOTION_RECOVERY_STAND]
    now[0] = 5.0
    backend.tick()
    assert transport.motions == [MOTION_RECOVERY_STAND]
    now[0] = 13.0
    backend.tick()
    assert transport.motions == [MOTION_RECOVERY_STAND, MOTION_RECOVERY_STAND]


def test_stale_charging_status_recovers_after_bms_disconnect():
    now = [1.0]
    backend = CyberDogPolicyBackend(Transport(), charging_recover_timeout_s=1.0,
                                    clock=lambda: now[0])
    backend.update_motion_status(SWITCH_CHARGING)
    backend.update_bms(connected=False)
    now[0] = 2.0
    backend.tick()
    assert backend.switch_status == SWITCH_NORMAL


def test_estop_recovery_waits_until_normal_before_velocity():
    transport = Transport()
    backend = CyberDogPolicyBackend(transport)
    backend.switch_status = SWITCH_ESTOP
    transport.on_motion = lambda motion_id: backend.update_motion_status(SWITCH_NORMAL)
    result = backend.execute(MotionCommand(linear_vel_x=0.2))
    assert result.accepted
    assert transport.motions == [MOTION_RECOVERY_STAND]
    assert transport.servo[-1]["vx"] == 0.2


def test_control_suspend_sends_zero_and_blocks_until_resume():
    transport = Transport()
    backend = CyberDogPolicyBackend(transport)
    backend.suspend()
    assert transport.servo[-1]["vx"] == 0.0
    assert not backend.execute(MotionCommand(linear_vel_x=0.2)).accepted
    assert backend.execute(MotionCommand(action="emergency_stop")).accepted
    assert transport.servo[-1]["vx"] == 0.0
    backend.resume()
    assert backend.execute(MotionCommand(linear_vel_x=0.2)).accepted
