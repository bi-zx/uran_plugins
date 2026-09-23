import math
from types import SimpleNamespace

from uran_navigation_plugin.obstacle_avoidance import GapAvoidancePlanner
from uran_navigation_plugin.route_follower import RouteFollower
from uran_navigation_plugin.safety_gate import MotionSafetyGate
from uran_navigation_plugin.straight_drive_controller import StraightDriveController
from uran_navigation_plugin.velocity_shaper import VelocityShaper
from uran_autotask_plugin.mission.mission_manager import MissionManager


class _Scan:
    def __init__(self, ranges=None):
        self.angle_min = -math.pi / 2.0
        self.angle_increment = math.pi / 180.0
        self.range_min = 0.05
        self.range_max = 10.0
        self.ranges = list(ranges if ranges is not None else [10.0] * 181)


class _Gateway:
    def __init__(self):
        self.commands = []

    def velocity(self, **command):
        self.commands.append(dict(command))


class _Logger:
    def warning(self, _message):
        pass


class _YawAligner:
    def state_snapshot(self):
        return {'aligned': False}


def _controller(clock, pose, gateway, **overrides):
    config = {
        'control_hz': 20.0,
        'target_tolerance_m': 3.0,
        'target_tolerance_relax_after_s': 30.0,
        'target_tolerance_relaxed_m': 5.0,
        'target_tolerance_final_m': 10.0,
        'goal_confirm_frames': 3,
        'goal_latch_s': 0.0,
        'scan_timeout_s': 0.35,
        'control_pose_timeout_s': 0.3,
        'min_scan_valid_ratio': 0.5,
        'min_front_valid_ratio': 0.5,
    }
    config.update(overrides)
    return StraightDriveController(
        config=config,
        move_gateway=gateway,
        pose_getter=lambda: dict(pose),
        map_pose_getter=lambda: dict(pose),
        now_ns_getter=lambda: 1_000_000_000,
        logger=_Logger(),
        monotonic_getter=lambda: clock[0],
    )


def test_velocity_shaper_limits_acceleration_and_forces_linear_stop():
    shaper = VelocityShaper({
        'max_speed_mps': 1.0,
        'max_angular_speed_radps': 1.0,
        'max_linear_accel_mps2': 0.5,
        'max_linear_decel_mps2': 1.0,
        'max_angular_accel_radps2': 1.0,
    })

    vx, wz = shaper.shape(1.0, 1.0, dt_s=0.2)
    assert abs(vx - 0.1) < 1e-9
    assert abs(wz - 0.2) < 1e-9

    vx, wz = shaper.shape(
        1.0,
        0.5,
        dt_s=0.2,
        force_zero_linear=True,
    )
    assert vx == 0.0
    assert wz > 0.0


def test_safety_gate_blocks_failsafe_and_stale_pose():
    gate = MotionSafetyGate({'control_pose_timeout_s': 0.3})

    failsafe = gate.check(failsafe_active=True)
    assert failsafe['allowed'] is False
    assert failsafe['code'] == 'E_FAILSAFE_ACTIVE'

    stale = gate.check(pose_age_s=0.31)
    assert stale['allowed'] is False
    assert stale['code'] == 'E_POSE_STALE'


def test_route_follower_detects_transit_point_passage():
    follower = RouteFollower({'goal_pass_margin_m': 0.3})
    follower.set_segment(
        start_x=0.0,
        start_y=0.0,
        goal_x=10.0,
        goal_y=0.0,
        goal_kind='transit',
        requires_stop=False,
    )

    result = follower.update({'x': 10.4, 'y': 0.1, 'yaw_deg': 0.0})
    assert result['passed_goal'] is True
    assert result['requires_stop'] is False


def test_route_follower_looks_into_next_transit_segment():
    follower = RouteFollower({
        'lookahead_min_m': 1.0,
        'lookahead_max_m': 1.0,
    })
    follower.set_segment(
        start_x=0.0,
        start_y=0.0,
        goal_x=10.0,
        goal_y=0.0,
        next_x=10.0,
        next_y=10.0,
        goal_kind='transit',
        requires_stop=False,
    )

    result = follower.update({'x': 9.5, 'y': 0.0, 'yaw_deg': 0.0})
    assert result['reference_x'] == 10.0
    assert result['reference_y'] > 0.0


def test_route_follower_keeps_next_segment_reference_after_crossing_transit_point():
    follower = RouteFollower({
        'lookahead_min_m': 1.0,
        'lookahead_max_m': 1.0,
    })
    follower.set_segment(
        start_x=0.0,
        start_y=0.0,
        goal_x=10.0,
        goal_y=0.0,
        next_x=10.0,
        next_y=10.0,
        goal_kind='transit',
        requires_stop=False,
    )

    result = follower.update({'x': 10.1, 'y': 0.0, 'yaw_deg': 0.0})
    assert result['passed_goal'] is True
    assert result['reference_x'] == 10.0
    assert result['reference_y'] > 0.0


def test_obstacle_planner_rejects_invalid_front_scan():
    planner = GapAvoidancePlanner({
        'min_scan_valid_ratio': 0.5,
        'min_front_valid_ratio': 0.5,
    })
    scan = _Scan([float('nan')] * 181)

    decision = planner.plan(scan, now_s=0.0)
    assert decision.state == 'blocked'
    assert decision.scan_quality == 'invalid'


def test_cyberdog_zero_ranges_are_clear_when_scan_has_healthy_returns():
    planner = GapAvoidancePlanner({
        'zero_range_mode': 'clear_if_scan_healthy',
        'min_positive_return_count': 20,
        'min_positive_return_ratio': 0.05,
        'min_scan_valid_ratio': 0.5,
        'min_front_valid_ratio': 0.5,
    })
    ranges = [0.0] * 181
    ranges[:30] = [10.0] * 30

    decision = planner.plan(_Scan(ranges), now_s=0.0)

    assert decision.state == 'clear'
    assert decision.scan_quality == 'good'
    assert decision.front_valid_ratio == 1.0
    assert decision.front_positive_return_ratio == 0.0
    assert decision.zero_ranges_treated_as_clear is True


def test_cyberdog_nearly_all_zero_scan_remains_invalid():
    planner = GapAvoidancePlanner({
        'zero_range_mode': 'clear_if_scan_healthy',
        'min_positive_return_count': 20,
        'min_positive_return_ratio': 0.05,
        'min_scan_valid_ratio': 0.5,
        'min_front_valid_ratio': 0.5,
    })
    ranges = [0.0] * 181
    ranges[:10] = [10.0] * 10

    decision = planner.plan(_Scan(ranges), now_s=0.0)

    assert decision.state == 'blocked'
    assert decision.scan_quality == 'invalid'
    assert decision.zero_ranges_treated_as_clear is False


def test_cyberdog_zero_compatibility_keeps_real_front_obstacle():
    planner = GapAvoidancePlanner({
        'zero_range_mode': 'clear_if_scan_healthy',
        'min_positive_return_count': 20,
        'min_positive_return_ratio': 0.05,
        'min_scan_valid_ratio': 0.5,
        'min_front_valid_ratio': 0.5,
        'min_stop_distance_m': 0.65,
    })
    ranges = [0.0] * 181
    ranges[:30] = [10.0] * 30
    ranges[88:93] = [0.4] * 5

    decision = planner.plan(_Scan(ranges), now_s=0.0)

    assert decision.state in {'stop', 'blocked'}
    assert decision.front_clearance_m == 0.4
    assert decision.zero_ranges_treated_as_clear is True


def test_obstacle_planner_reports_blocked_when_no_gap_exists():
    planner = GapAvoidancePlanner({
        'min_scan_valid_ratio': 0.5,
        'min_front_valid_ratio': 0.5,
        'min_stop_distance_m': 0.65,
    })

    decision = planner.plan(_Scan([0.2] * 181), now_s=0.0)
    assert decision.state == 'blocked'
    assert decision.choice is None


def test_goal_confirmation_resets_after_leaving_tolerance():
    clock = [0.0]
    pose = {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0, 'age_s': 0.0}
    gateway = _Gateway()
    controller = _controller(clock, pose, gateway)
    assert controller.start(task_id='goal-test', target_x=10.0, target_y=0.0)['accepted']
    controller.update_scan(_Scan(), monotonic_s=clock[0])

    pose['x'] = 8.0
    first = controller.tick(monotonic_s=clock[0])
    assert first['state'] == 'goal_settling'
    assert first['goal_inside_count'] == 1

    clock[0] += 0.05
    pose['x'] = 6.0
    controller.update_scan(_Scan(), monotonic_s=clock[0])
    outside = controller.tick(monotonic_s=clock[0])
    assert outside['state'] != 'completed'
    assert controller.snapshot()['goal_inside_count'] == 0

    pose['x'] = 8.0
    states = []
    for _ in range(3):
        clock[0] += 0.05
        controller.update_scan(_Scan(), monotonic_s=clock[0])
        states.append(controller.tick(monotonic_s=clock[0])['state'])
    assert states == ['goal_settling', 'goal_settling', 'completed']


def test_target_tolerance_uses_three_five_ten_meter_ladder():
    clock = [0.0]
    pose = {'x': 91.0, 'y': 0.0, 'yaw_deg': 0.0, 'age_s': 0.0}
    controller = _controller(clock, pose, _Gateway())
    assert controller.start(task_id='ladder', target_x=100.0, target_y=0.0)['accepted']
    controller.update_scan(_Scan(), monotonic_s=0.0)
    controller.tick(monotonic_s=0.0)

    assert controller._effective_target_tolerance_m(29.9) == 3.0
    assert controller._effective_target_tolerance_m(30.0) == 5.0
    assert controller._effective_target_tolerance_m(60.0) == 10.0


def test_target_tolerance_does_not_relax_during_long_approach():
    clock = [0.0]
    pose = {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0, 'age_s': 0.0}
    controller = _controller(clock, pose, _Gateway())
    assert controller.start(task_id='long-leg', target_x=100.0, target_y=0.0)['accepted']
    controller.update_scan(_Scan(), monotonic_s=0.0)
    controller.tick(monotonic_s=0.0)

    assert controller._effective_target_tolerance_m(120.0) == 3.0
    assert controller.snapshot()['goal_relaxation_age_s'] is None


def test_transit_point_does_not_use_three_meter_stop_tolerance():
    clock = [0.0]
    pose = {'x': 8.0, 'y': 0.0, 'yaw_deg': 0.0, 'age_s': 0.0}
    controller = _controller(clock, pose, _Gateway())
    assert controller.start(
        task_id='transit',
        target_x=10.0,
        target_y=0.0,
        route_start_x=0.0,
        route_start_y=0.0,
        goal_kind='transit',
        requires_stop=False,
    )['accepted']
    controller.update_scan(_Scan(), monotonic_s=0.0)

    result = controller.tick(monotonic_s=0.0)
    assert result['active'] is True
    assert result['state'] != 'completed'
    assert result['desired_vx'] > controller.goal_settle_speed_mps


def test_stop_point_approach_speed_uses_braking_distance():
    clock = [0.0]
    pose = {'x': 4.0, 'y': 0.0, 'yaw_deg': 0.0, 'age_s': 0.0}
    controller = _controller(
        clock,
        pose,
        _Gateway(),
        default_speed_mps=0.8,
        max_speed_mps=0.8,
        brake_decel_mps2=0.5,
    )
    assert controller.start(
        task_id='approach',
        target_x=10.0,
        target_y=0.0,
        requires_stop=True,
    )['accepted']
    controller.update_scan(_Scan(), monotonic_s=0.0)

    far = controller.tick(monotonic_s=0.0)
    assert far['desired_vx'] == 0.8

    clock[0] += 0.05
    pose['x'] = 9.0
    controller.update_scan(_Scan(), monotonic_s=clock[0])
    near = controller.tick(monotonic_s=clock[0])
    assert near['state'] == 'goal_settling'
    assert controller.snapshot()['last_vx'] == 0.0


def test_distance_command_decelerates_before_completion():
    clock = [0.0]
    pose = {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0, 'age_s': 0.0}
    controller = _controller(
        clock,
        pose,
        _Gateway(),
        default_speed_mps=0.8,
        max_speed_mps=0.8,
        brake_decel_mps2=0.5,
        max_pose_step_m=10.0,
    )
    assert controller.start(task_id='distance', distance_m=4.0)['accepted']
    controller.update_scan(_Scan(), monotonic_s=0.0)
    controller.tick(monotonic_s=0.0)

    clock[0] += 0.05
    pose['x'] = 3.7
    controller.update_scan(_Scan(), monotonic_s=clock[0])
    result = controller.tick(monotonic_s=clock[0])

    assert result['state'] == 'clear'
    assert controller.goal_settle_speed_mps < result['desired_vx'] < 0.8
    assert abs(controller.distance_target_remaining_m() - 0.3) < 1e-9


def test_zero_monotonic_timestamp_is_a_valid_scan_time():
    clock = [0.0]
    pose = {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0, 'age_s': 0.0}
    controller = _controller(clock, pose, _Gateway())
    controller.update_scan(_Scan(), monotonic_s=0.0)

    clock[0] = 0.2
    assert abs(controller.scan_age_s() - 0.2) < 1e-9


def _initial_yaw_manager():
    manager = MissionManager.__new__(MissionManager)
    manager._gps_vo_yaw_aligner = _YawAligner()
    manager._initial_yaw_calibration_active = True
    manager._active_local_goal_kind = 'initial_yaw_calibration'
    manager._paused_local_context = {}

    def pause_with_error(**_error):
        manager._capture_paused_local_context()
        manager._initial_yaw_calibration_active = False
        manager._active_local_goal_kind = ''

    manager._pause_with_error = pause_with_error
    manager._publish_periodic_progress = lambda **_kwargs: None
    return manager


def test_initial_yaw_completion_failure_preserves_resume_context():
    manager = _initial_yaw_manager()

    manager._handle_initial_yaw_control_result(
        result={'state': 'completed'},
        monotonic_s=1.0,
    )

    assert manager._paused_local_context == {'kind': 'initial_yaw_calibration'}


def test_initial_yaw_terminal_failure_preserves_resume_context():
    manager = _initial_yaw_manager()

    def handle_terminal(result, **_kwargs):
        assert result['terminal'] is True
        manager._pause_with_error()
        return True

    manager._handle_terminal_local_control_result = handle_terminal
    manager._handle_initial_yaw_control_result(
        result={'state': 'blocked', 'terminal': True},
        monotonic_s=1.0,
    )

    assert manager._paused_local_context == {'kind': 'initial_yaw_calibration'}


def test_straight_drive_resume_uses_remaining_distance_and_duration():
    class Controller:
        def __init__(self):
            self.started = None

        def snapshot(self):
            return {
                'target_distance_m': 10.0,
                'distance_traveled_m': 4.0,
                'target_duration_s': 20.0,
                'elapsed_s': 8.0,
            }

        def start(self, **kwargs):
            self.started = kwargs
            return {'accepted': True}

    controller = Controller()
    manager = MissionManager.__new__(MissionManager)
    manager._task = None
    manager._outdoor_mission = None
    manager._straight_drive_task = {
        'distance_m': 10.0,
        'duration_s': 20.0,
        'speed_mps': 0.8,
        'target_x': None,
        'target_y': None,
        'target_tolerance_m': None,
    }
    manager._straight_drive_controller = controller
    manager._initial_yaw_calibration_active = False
    manager._active_home_calibration_index = None
    manager._active_outdoor_point_index = None
    manager._active_waypoint_index = None
    manager._paused_local_context = {}
    manager._capture_paused_local_context()
    manager._runtime = SimpleNamespace(
        task_id='resume-straight',
        stage='paused',
        status='exception',
        event='task_paused',
        error={},
    )
    manager._require_auto_mode = False
    manager._control_mode_getter = lambda: 'auto'
    manager._acquire_motion_control_lock = lambda **_kwargs: None
    manager._publish_task_progress = lambda: None
    manager._write_state_fields = lambda: None

    manager._handle_resume()

    assert controller.started['distance_m'] == 6.0
    assert controller.started['duration_s'] == 12.0
    assert manager._runtime.stage == 'executing'
