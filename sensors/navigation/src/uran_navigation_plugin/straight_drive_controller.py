import math
import time
from typing import Any, Dict, Optional

from .obstacle_avoidance import GapAvoidancePlanner
from .route_follower import RouteFollower
from .safety_gate import MotionSafetyGate
from .velocity_shaper import VelocityShaper


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _finite_float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


class StraightDriveController:
    """Run one metric-frame route segment with reactive LiDAR control.

    This class owns the high-rate local control state, but it does not know
    anything about ROS messages.  The mission manager supplies a target and a
    move gateway supplies the resulting planar velocity command.
    """

    def __init__(
        self,
        *,
        config: Optional[Dict[str, Any]] = None,
        move_gateway,
        pose_getter,
        map_pose_getter=None,
        now_ns_getter,
        logger,
        safety_gate=None,
        control_state_getter=None,
        monotonic_getter=None,
    ):
        cfg = dict(config or {})
        self.enabled = bool(cfg.get('enabled', True))
        self.control_hz = max(5.0, float(cfg.get('control_hz', 20.0)))
        self.default_speed_mps = max(0.0, float(cfg.get('default_speed_mps', 0.18)))
        self.max_speed_mps = max(0.0, float(cfg.get('max_speed_mps', 0.25)))
        self.min_avoid_speed_mps = max(0.0, float(cfg.get('min_avoid_speed_mps', 0.06)))
        self.max_angular_speed_radps = max(
            0.0,
            float(cfg.get('max_angular_speed_radps', 0.7)),
        )
        self.heading_kp = max(0.0, float(cfg.get('heading_kp', 1.4)))
        self.slowdown_heading_rad = math.radians(
            float(cfg.get('slowdown_heading_deg', 55.0))
        )
        self.turn_in_place_heading_rad = math.radians(
            float(cfg.get('turn_in_place_heading_deg', 85.0))
        )
        self.scan_timeout_s = max(0.0, float(cfg.get('scan_timeout_s', 0.35)))
        self.blocked_timeout_s = max(0.0, float(cfg.get('blocked_timeout_s', 4.0)))
        self.fault_pause_timeout_s = max(
            0.0,
            float(cfg.get('fault_pause_timeout_s', self.blocked_timeout_s)),
        )
        self.recover_heading_tolerance_rad = math.radians(
            float(cfg.get('recover_heading_tolerance_deg', 10.0))
        )
        self.distance_tolerance_m = max(0.0, float(cfg.get('distance_tolerance_m', 0.15)))

        # The first tolerance is deliberately modest.  The later values are
        # only a timeout fallback for noisy outdoor localization.
        self.target_tolerance_m = max(0.05, float(cfg.get('target_tolerance_m', 0.8)))
        self.target_tolerance_relax_after_s = max(
            0.0,
            float(cfg.get('target_tolerance_relax_after_s', 30.0)),
        )
        self.target_tolerance_relaxed_m = max(
            self.target_tolerance_m,
            float(cfg.get('target_tolerance_relaxed_m', 5.0)),
        )
        self.target_tolerance_final_m = max(
            self.target_tolerance_relaxed_m,
            float(cfg.get('target_tolerance_final_m', 10.0)),
        )
        self.goal_confirm_frames = max(1, int(cfg.get('goal_confirm_frames', 5)))
        self.goal_settle_speed_mps = max(
            0.0,
            float(cfg.get('goal_settle_speed_mps', 0.15)),
        )
        self.goal_settle_angular_radps = max(
            0.0,
            float(cfg.get('goal_settle_angular_radps', 0.15)),
        )
        self.goal_latch_s = max(0.0, float(cfg.get('goal_latch_s', 0.5)))
        self.goal_pass_margin_m = max(0.0, float(cfg.get('goal_pass_margin_m', 0.35)))

        self.command_latency_s = max(0.0, float(cfg.get('command_latency_s', 0.15)))
        self.brake_decel_mps2 = max(0.01, float(cfg.get('brake_decel_mps2', 0.5)))
        self.stop_margin_m = max(0.0, float(cfg.get('stop_margin_m', 0.25)))
        self.max_pose_step_m = max(0.0, float(cfg.get('max_pose_step_m', 2.0)))
        self.command_source = str(cfg.get('command_source', 'straight_drive'))

        self._planner = GapAvoidancePlanner(cfg)
        self._route_follower = RouteFollower(cfg)
        self._velocity_shaper = VelocityShaper(cfg)
        self._safety_gate = safety_gate or MotionSafetyGate(cfg)
        self._move_gateway = move_gateway
        self._pose_getter = pose_getter
        self._map_pose_getter = map_pose_getter
        self._now_ns_getter = now_ns_getter
        self._control_state_getter = control_state_getter
        self._monotonic_getter = monotonic_getter or time.monotonic
        self._logger = logger

        self._latest_scan = None
        self._latest_scan_monotonic_s: Optional[float] = None
        self._active = False
        self._task_id = ''
        self._started_monotonic_s: Optional[float] = None
        self._last_control_monotonic_s: Optional[float] = None
        self._last_command_monotonic_s: Optional[float] = None
        self._blocked_since_s: Optional[float] = None
        self._fault_since_s: Optional[float] = None
        self._last_safety_code = ''
        self._yaw_ref_rad: Optional[float] = None
        self._start_pose: Optional[Dict[str, Any]] = None
        self._last_path_pose: Optional[Dict[str, Any]] = None
        self._path_length_m = 0.0
        self._target_distance_m: Optional[float] = None
        self._target_duration_s: Optional[float] = None
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._target_tolerance_base_m = self.target_tolerance_m
        self._target_tolerance_max_m = self.target_tolerance_final_m
        self._pose_source = 'local'
        self._active_command_source = self.command_source
        self._goal_kind = 'inspection'
        self._requires_stop = True
        self._speed_mps = self.default_speed_mps
        self._last_vx = 0.0
        self._last_wz = 0.0
        self._goal_inside_count = 0
        self._goal_inside_since_s: Optional[float] = None
        self._goal_relaxation_started_s: Optional[float] = None
        self._goal_latched = False
        self._goal_latched_until_s: Optional[float] = None
        self._route_state: Dict[str, Any] = {}
        self._last_safety: Dict[str, Any] = {}
        self._last_decision: Dict[str, Any] = {
            'active': False,
            'state': 'idle',
            'reason': 'not started',
        }

    def update_scan(self, scan, *, monotonic_s: Optional[float] = None):
        self._latest_scan = scan
        self._latest_scan_monotonic_s = (
            self._monotonic_getter() if monotonic_s is None else float(monotonic_s)
        )

    def start(
        self,
        *,
        task_id: str,
        distance_m: Optional[float] = None,
        duration_s: Optional[float] = None,
        speed_mps: Optional[float] = None,
        target_x: Optional[float] = None,
        target_y: Optional[float] = None,
        target_tolerance_m: Optional[float] = None,
        command_source: Optional[str] = None,
        route_start_x: Optional[float] = None,
        route_start_y: Optional[float] = None,
        route_next_x: Optional[float] = None,
        route_next_y: Optional[float] = None,
        goal_kind: str = 'inspection',
        requires_stop: Optional[bool] = None,
        monotonic_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        if not self.enabled:
            return {
                'accepted': False,
                'code': 'E_STRAIGHT_DRIVE_DISABLED',
                'message': 'straight drive controller is disabled',
            }
        if self._move_gateway is None:
            return {
                'accepted': False,
                'code': 'E_MOVE_GATEWAY_UNAVAILABLE',
                'message': 'uran_move gateway is unavailable',
            }

        distance = self._optional_positive_or_none(distance_m)
        duration = self._optional_positive_or_none(duration_s)
        target_x_value = _finite_float(target_x) if target_x is not None else None
        target_y_value = _finite_float(target_y) if target_y is not None else None
        if (target_x is not None) != (target_y is not None):
            return {
                'accepted': False,
                'code': 'E_STRAIGHT_DRIVE_TARGET_INVALID',
                'message': 'target_x and target_y must be provided together',
            }
        has_target = target_x_value is not None and target_y_value is not None
        if target_x is not None and not has_target:
            return {
                'accepted': False,
                'code': 'E_STRAIGHT_DRIVE_TARGET_INVALID',
                'message': 'target_x and target_y must be finite numbers',
            }
        if distance is None and duration is None and not has_target:
            return {
                'accepted': False,
                'code': 'E_STRAIGHT_DRIVE_TARGET_EMPTY',
                'message': 'distance_m, duration_s, or target_x/target_y must be provided',
            }

        if self._active:
            self.stop(reason='replaced_by_new_local_goal')

        now = self._monotonic_getter() if monotonic_s is None else float(monotonic_s)
        pose_source = 'map' if has_target else 'local'
        pose = self._current_pose(source=pose_source)
        needs_pose = (distance is not None) or has_target
        if needs_pose and not self._pose_has_xy(pose):
            required = 'map pose' if has_target else 'local odometry pose'
            return {
                'accepted': False,
                'code': 'E_STRAIGHT_DRIVE_POSE_UNAVAILABLE',
                'message': f'{required} is required for this straight drive command',
            }

        requested_speed = (
            self.default_speed_mps if speed_mps is None else _finite_float(speed_mps)
        )
        if requested_speed is None:
            return {
                'accepted': False,
                'code': 'E_STRAIGHT_DRIVE_SPEED_INVALID',
                'message': 'speed_mps must be a finite number',
            }
        if self.max_speed_mps <= 0.0:
            return {
                'accepted': False,
                'code': 'E_STRAIGHT_DRIVE_SPEED_DISABLED',
                'message': 'max_speed_mps is zero',
            }

        yaw = self._pose_yaw_rad(pose)
        self._active = True
        self._task_id = str(task_id or '')
        self._started_monotonic_s = now
        self._last_control_monotonic_s = None
        self._last_command_monotonic_s = None
        self._blocked_since_s = None
        self._fault_since_s = None
        self._last_safety_code = ''
        self._yaw_ref_rad = yaw
        self._start_pose = dict(pose or {}) if pose else None
        self._last_path_pose = dict(pose or {}) if pose else None
        self._path_length_m = 0.0
        self._target_distance_m = distance
        self._target_duration_s = duration
        self._target_x = target_x_value if has_target else None
        self._target_y = target_y_value if has_target else None
        self._target_tolerance_base_m = self.target_tolerance_m
        requested_tolerance = self.target_tolerance_m
        if target_tolerance_m is not None:
            parsed_tolerance = _finite_float(target_tolerance_m)
            if parsed_tolerance is not None:
                requested_tolerance = max(0.05, parsed_tolerance)
        # Include the complete fallback ladder.  Otherwise a requested point
        # tolerance of 0.8 m would accidentally prevent the 5/10 m fallback.
        self._target_tolerance_max_m = max(
            self._target_tolerance_base_m,
            requested_tolerance,
            self.target_tolerance_relaxed_m,
            self.target_tolerance_final_m,
        )
        self._pose_source = pose_source
        self._active_command_source = str(command_source or self.command_source)
        self._goal_kind = str(goal_kind or 'inspection').strip().lower() or 'inspection'
        self._requires_stop = (
            bool(requires_stop)
            if requires_stop is not None
            else self._goal_kind not in {'transit'}
        )
        self._speed_mps = _clamp(abs(float(requested_speed)), 0.02, self.max_speed_mps)
        self._goal_inside_count = 0
        self._goal_inside_since_s = None
        self._goal_relaxation_started_s = None
        self._goal_latched = False
        self._goal_latched_until_s = None
        self._route_state = {}
        self._last_safety = {}
        self._planner.reset()
        self._route_follower.reset()
        self._velocity_shaper.reset()

        if has_target:
            start_x = route_start_x
            start_y = route_start_y
            if start_x is None or start_y is None:
                start_x = float(pose.get('x', target_x_value))
                start_y = float(pose.get('y', target_y_value))
            start_x = _finite_float(start_x)
            start_y = _finite_float(start_y)
            if start_x is None or start_y is None:
                start_x = float(pose.get('x', target_x_value))
                start_y = float(pose.get('y', target_y_value))
            next_x_value = _finite_float(route_next_x) if route_next_x is not None else None
            next_y_value = _finite_float(route_next_y) if route_next_y is not None else None
            self._route_follower.set_segment(
                start_x=start_x,
                start_y=start_y,
                goal_x=target_x_value,
                goal_y=target_y_value,
                next_x=next_x_value,
                next_y=next_y_value,
                goal_kind=self._goal_kind,
                requires_stop=self._requires_stop,
            )

        self._last_vx = 0.0
        self._last_wz = 0.0
        self._last_decision = {
            'active': True,
            'state': 'starting',
            'reason': 'straight drive accepted',
            'task_id': self._task_id,
            'target_distance_m': self._target_distance_m,
            'target_duration_s': self._target_duration_s,
            'target_x': self._target_x,
            'target_y': self._target_y,
            'target_tolerance_m': self._effective_target_tolerance_m(now) if has_target else None,
            'target_tolerance_base_m': self._target_tolerance_base_m if has_target else None,
            'target_tolerance_max_m': self._target_tolerance_max_m if has_target else None,
            'pose_source': self._pose_source,
            'command_source': self._active_command_source,
            'goal_kind': self._goal_kind,
            'requires_stop': self._requires_stop,
            'speed_mps': self._speed_mps,
            'yaw_ref_deg': math.degrees(yaw) if yaw is not None else None,
        }
        return {'accepted': True}

    def stop(self, *, reason: str = '', monotonic_s: Optional[float] = None):
        if self._move_gateway is not None:
            self._publish_velocity(
                0.0,
                0.0,
                reason=reason or 'straight_drive_stop',
                hard_stop=True,
                monotonic_s=monotonic_s,
            )
        self._active = False
        self._blocked_since_s = None
        self._fault_since_s = None
        self._goal_latched = False
        self._goal_latched_until_s = None
        self._last_decision = {
            'active': False,
            'state': 'idle',
            'reason': reason or 'stopped',
            'task_id': self._task_id,
            'last_vx': self._last_vx,
            'last_wz': self._last_wz,
        }

    def hard_stop(self, *, reason: str = '', monotonic_s: Optional[float] = None):
        """Publish an immediate zero command while retaining task state."""
        if self._move_gateway is not None:
            self._publish_velocity(
                0.0,
                0.0,
                reason=reason or 'straight_drive_hard_stop',
                hard_stop=True,
                monotonic_s=monotonic_s,
            )
        self._last_decision = dict(self._last_decision)
        self._last_decision.update({
            'active': self._active,
            'state': 'safety_blocked',
            'reason': reason or 'hard stop',
        })

    def tick(self, *, monotonic_s: Optional[float] = None) -> Dict[str, Any]:
        now = self._monotonic_getter() if monotonic_s is None else float(monotonic_s)
        if not self._active:
            return {'state': 'idle', 'active': False}

        if self._last_control_monotonic_s is not None:
            min_interval = 1.0 / self.control_hz
            if now - self._last_control_monotonic_s + 1e-9 < min_interval:
                return {
                    'state': 'running',
                    'active': True,
                    'throttled': True,
                }
        self._last_control_monotonic_s = now

        pose = self._current_pose(source=self._pose_source)
        pose_available = self._pose_has_xy(pose)
        pose_age_s = self._pose_age_s(pose)
        if pose_available:
            self._update_path_distance(pose)

        scan_age = self._scan_age_at(now)
        if self._latest_scan is None:
            return self._handle_safety_block(
                now,
                code='E_SCAN_UNAVAILABLE',
                reason='laser scan is unavailable',
                pose_available=pose_available,
                pose_age_s=pose_age_s,
                scan_age_s=None,
                scan_quality='invalid',
            )

        # Never declare a goal reached from an unavailable or stale pose.  The
        # previous implementation checked completion first, which could use a
        # frozen position sample and silently finish a route.
        if not pose_available:
            return self._handle_safety_block(
                now,
                code='E_POSE_UNAVAILABLE',
                reason='control pose is unavailable',
                pose_available=False,
                pose_age_s=pose_age_s,
                scan_age_s=scan_age,
                scan_quality='unknown',
            )
        if (
            pose_age_s is not None and
            self._safety_gate.pose_timeout_s > 0.0 and
            pose_age_s > self._safety_gate.pose_timeout_s
        ):
            return self._handle_safety_block(
                now,
                code='E_POSE_STALE',
                reason='control pose is stale',
                pose_available=True,
                pose_age_s=pose_age_s,
                scan_age_s=scan_age,
                scan_quality='unknown',
            )

        if self.scan_timeout_s > 0.0 and scan_age is not None and scan_age > self.scan_timeout_s:
            return self._handle_safety_block(
                now,
                code='E_SCAN_STALE',
                reason='laser scan is stale',
                pose_available=pose_available,
                pose_age_s=pose_age_s,
                scan_age_s=scan_age,
                scan_quality='invalid',
            )

        theta_goal = self._theta_goal(pose)
        decision = self._planner.plan(
            self._latest_scan,
            theta_goal=theta_goal,
            stop_distance_m=self._dynamic_stop_distance_m(),
            now_s=now,
        )
        control_state = self._control_state()
        safety = self._safety_gate.check(
            control_mode=control_state.get('control_mode', 'auto'),
            controller=control_state.get('controller', 'auto'),
            pose_available=pose_available,
            pose_age_s=pose_age_s,
            scan_available=True,
            scan_age_s=scan_age,
            scan_quality=decision.scan_quality,
            scan_valid_ratio=decision.valid_ratio,
            front_valid_ratio=decision.front_valid_ratio,
            bottom_status=control_state.get('bottom_status'),
            battery_level=control_state.get('battery_level'),
            failsafe_active=control_state.get('failsafe_active', False),
        )
        self._last_safety = dict(safety)
        if not safety.get('allowed', False):
            scan_diagnostics = {
                'valid_ratio': decision.valid_ratio,
                'front_valid_ratio': decision.front_valid_ratio,
                'positive_return_count': decision.positive_return_count,
                'positive_return_ratio': decision.positive_return_ratio,
                'zero_range_ratio': decision.zero_range_ratio,
                'front_positive_return_ratio': decision.front_positive_return_ratio,
                'front_zero_range_ratio': decision.front_zero_range_ratio,
                'zero_ranges_treated_as_clear': decision.zero_ranges_treated_as_clear,
            }
            safety_code = str(safety.get('code') or 'E_CONTROL_SAFETY')
            safety_reason = str(
                safety.get('reason') or 'motion safety gate blocked control'
            )
            if safety_code in {'E_SCAN_QUALITY', 'E_SCAN_FRONT_QUALITY'}:
                safety_reason = (
                    f'{decision.reason}; valid_ratio={decision.valid_ratio:.3f}, '
                    f'front_valid_ratio={decision.front_valid_ratio:.3f}, '
                    f'positive_return_ratio={decision.positive_return_ratio:.3f}, '
                    f'zero_range_ratio={decision.zero_range_ratio:.3f}'
                )
            return self._handle_safety_block(
                now,
                code=safety_code,
                reason=safety_reason,
                pose_available=pose_available,
                pose_age_s=pose_age_s,
                scan_age_s=scan_age,
                scan_quality=decision.scan_quality,
                safety=safety,
                scan_diagnostics=scan_diagnostics,
            )

        self._fault_since_s = None
        self._last_safety_code = ''

        completion = self._completion_reason(now, pose=pose)
        if completion is not None:
            return self._handle_completion(completion, now)
        self._reset_goal_confirmation()

        desired_vx, desired_wz = self._velocity_from_decision(
            decision,
            pose=pose,
            now=now,
            speed_cap_mps=safety.get('speed_cap_mps'),
        )

        if decision.state in {'blocked', 'stop'}:
            self._mark_blocked(now)
        if decision.state == 'blocked':
            vx, wz = self._publish_velocity(
                0.0,
                0.0,
                reason='obstacle_blocked',
                hard_stop=True,
                monotonic_s=now,
            )
        else:
            if decision.state != 'stop':
                self._blocked_since_s = None
            vx, wz = self._publish_velocity(
                desired_vx,
                desired_wz,
                reason=decision.state,
                force_zero_linear=decision.state == 'stop',
                speed_cap_mps=safety.get('speed_cap_mps'),
                monotonic_s=now,
            )

        result = {
            'state': decision.state,
            'active': True,
            'task_id': self._task_id,
            'theta_goal_deg': math.degrees(theta_goal),
            'vx': vx,
            'wz': wz,
            'desired_vx': desired_vx,
            'desired_wz': desired_wz,
            'distance_traveled_m': self.distance_traveled_m(),
            'target_remaining_m': self.target_remaining_m(pose=pose),
            'elapsed_s': self.elapsed_s(now),
            'decision': decision.to_dict(),
            'safety': dict(safety),
            'route': dict(self._route_state),
        }

        if self._blocked_since_s is not None:
            blocked_for = max(0.0, now - self._blocked_since_s)
            result['blocked_for_s'] = blocked_for
            if blocked_for >= self.blocked_timeout_s:
                self._active = False
                result['active'] = False
                result['terminal'] = True
                result['reason'] = 'obstacle blocked timeout'
        self._last_decision = dict(result)
        return result

    def snapshot(self) -> Dict[str, Any]:
        data = dict(self._last_decision)
        data.setdefault('active', self._active)
        data['scan_age_s'] = self.scan_age_s()
        data['distance_traveled_m'] = self.distance_traveled_m()
        data['path_length_m'] = self._path_length_m
        data['target_distance_m'] = self._target_distance_m
        data['target_duration_s'] = self._target_duration_s
        data['distance_target_remaining_m'] = self.distance_target_remaining_m()
        data['duration_target_remaining_s'] = self.duration_target_remaining_s()
        data['target_x'] = self._target_x
        data['target_y'] = self._target_y
        data['target_tolerance_m'] = (
            self._effective_target_tolerance_m() if self._target_x is not None else None
        )
        data['target_tolerance_base_m'] = (
            self._target_tolerance_base_m if self._target_x is not None else None
        )
        data['target_tolerance_max_m'] = (
            self._target_tolerance_max_m if self._target_x is not None else None
        )
        data['target_remaining_m'] = self.target_remaining_m()
        data['pose_source'] = self._pose_source
        data['command_source'] = self._active_command_source
        data['goal_kind'] = self._goal_kind
        data['requires_stop'] = self._requires_stop
        data['goal_inside_count'] = self._goal_inside_count
        data['goal_confirm_frames'] = self.goal_confirm_frames
        data['goal_latched'] = self._goal_latched
        data['goal_relaxation_age_s'] = (
            max(0.0, self._monotonic_getter() - self._goal_relaxation_started_s)
            if self._goal_relaxation_started_s is not None else None
        )
        data['elapsed_s'] = self.elapsed_s()
        data['control_hz'] = self.control_hz
        data['last_vx'] = self._last_vx
        data['last_wz'] = self._last_wz
        data['route_follower'] = self._route_follower.snapshot()
        data['velocity_shaper'] = self._velocity_shaper.snapshot()
        data['last_safety'] = dict(self._last_safety)
        return data

    def is_active(self) -> bool:
        return self._active

    def elapsed_s(self, now: Optional[float] = None) -> float:
        if self._started_monotonic_s is None:
            return 0.0
        now = self._monotonic_getter() if now is None else float(now)
        return max(0.0, now - self._started_monotonic_s)

    def scan_age_s(self, now: Optional[float] = None) -> Optional[float]:
        if self._latest_scan_monotonic_s is None:
            return None
        now = self._monotonic_getter() if now is None else float(now)
        return max(0.0, now - self._latest_scan_monotonic_s)

    def distance_traveled_m(self) -> float:
        return max(0.0, float(self._path_length_m))

    def target_remaining_m(self, *, pose: Optional[Dict[str, Any]] = None) -> Optional[float]:
        if self._target_x is None or self._target_y is None:
            return None
        pose = self._current_pose(source=self._pose_source) if pose is None else pose
        if not self._pose_has_xy(pose):
            return None
        return math.hypot(
            float(self._target_x) - float(pose.get('x', 0.0)),
            float(self._target_y) - float(pose.get('y', 0.0)),
        )

    def distance_target_remaining_m(self) -> Optional[float]:
        if self._target_distance_m is None:
            return None
        return max(0.0, self._target_distance_m - self.distance_traveled_m())

    def duration_target_remaining_s(self, now: Optional[float] = None) -> Optional[float]:
        if self._target_duration_s is None:
            return None
        return max(0.0, self._target_duration_s - self.elapsed_s(now))

    def _completion_reason(
        self,
        now: float,
        *,
        pose: Optional[Dict[str, Any]] = None,
    ) -> Optional[str]:
        if self._target_duration_s is not None and self.elapsed_s(now) >= self._target_duration_s:
            return 'duration reached'
        if (
            self._target_distance_m is not None and
            self.distance_traveled_m() >= max(
                0.0,
                self._target_distance_m - self.distance_tolerance_m,
            )
        ):
            return 'distance reached'

        if self._target_x is None or self._target_y is None:
            return None
        pose = self._current_pose(source=self._pose_source) if pose is None else pose
        if not self._pose_has_xy(pose):
            return None
        self._route_state = self._route_follower.update(
            pose,
            speed_mps=self._speed_mps,
        )
        remaining = self.target_remaining_m(pose=pose)
        self._update_goal_relaxation(now, remaining)
        if not self._requires_stop and (
            bool(self._route_state.get('passed_goal', False)) or
            (remaining is not None and remaining <= self.goal_pass_margin_m)
        ):
            return 'route point passed'
        if (
            self._requires_stop and
            remaining is not None and
            remaining <= self._effective_target_tolerance_m(now)
        ):
            return 'target reached'
        return None

    # Kept as a compatibility alias for callers that used the old private
    # method in diagnostics or tests.
    def _completion_state(self, now: float) -> Optional[str]:
        return self._completion_reason(now)

    def _handle_completion(self, reason: str, now: float) -> Dict[str, Any]:
        if self._requires_stop:
            if not self._goal_latched:
                self._goal_latched = True
                self._goal_inside_since_s = now
                self._goal_latched_until_s = now + self.goal_latch_s
            self._goal_inside_count += 1
            self._publish_velocity(
                0.0,
                0.0,
                reason='goal_settle',
                hard_stop=True,
                monotonic_s=now,
            )
            latch_finished = (
                self._goal_latched_until_s is None or
                now >= self._goal_latched_until_s
            )
            if self._goal_inside_count < self.goal_confirm_frames or not latch_finished:
                result = {
                    'state': 'goal_settling',
                    'active': True,
                    'task_id': self._task_id,
                    'reason': reason,
                    'goal_inside_count': self._goal_inside_count,
                    'goal_confirm_frames': self.goal_confirm_frames,
                    'goal_inside_for_s': max(0.0, now - self._goal_inside_since_s),
                    'goal_latch_remaining_s': (
                        max(0.0, self._goal_latched_until_s - now)
                        if self._goal_latched_until_s is not None else 0.0
                    ),
                    'target_remaining_m': self.target_remaining_m(),
                    'elapsed_s': self.elapsed_s(now),
                }
                self._last_decision = dict(result)
                return result

        self._active = False
        self._blocked_since_s = None
        self._fault_since_s = None
        result = {
            'state': 'completed',
            'active': False,
            'terminal': True,
            'reason': reason,
            'task_id': self._task_id,
            'distance_traveled_m': self.distance_traveled_m(),
            'target_remaining_m': self.target_remaining_m(),
            'elapsed_s': self.elapsed_s(now),
            'goal_inside_count': self._goal_inside_count,
        }
        self._last_decision = dict(result)
        return result

    def _reset_goal_confirmation(self):
        if not self._goal_latched and self._goal_inside_count == 0:
            return
        self._goal_inside_count = 0
        self._goal_inside_since_s = None
        self._goal_latched = False
        self._goal_latched_until_s = None

    def _effective_target_tolerance_m(self, now: Optional[float] = None) -> float:
        if self._target_x is None or self._target_y is None:
            return self._target_tolerance_base_m
        if self.target_tolerance_relax_after_s <= 0.0:
            return self._target_tolerance_max_m
        if self._goal_relaxation_started_s is None:
            return self._target_tolerance_base_m

        now = self._monotonic_getter() if now is None else float(now)
        elapsed = max(0.0, now - self._goal_relaxation_started_s)
        tolerance = self._target_tolerance_base_m
        if elapsed >= self.target_tolerance_relax_after_s:
            tolerance = max(tolerance, self.target_tolerance_relaxed_m)
        if elapsed >= 2.0 * self.target_tolerance_relax_after_s:
            tolerance = max(tolerance, self.target_tolerance_final_m)
        return min(tolerance, self._target_tolerance_max_m)

    def _update_goal_relaxation(
        self,
        now: float,
        remaining_m: Optional[float],
    ):
        if not self._requires_stop or self._goal_relaxation_started_s is not None:
            return
        if remaining_m is None:
            return
        if (
            remaining_m > self._target_tolerance_base_m and
            remaining_m <= self._target_tolerance_max_m
        ):
            self._goal_relaxation_started_s = float(now)

    def _velocity_from_decision(
        self,
        decision,
        *,
        pose: Optional[Dict[str, Any]] = None,
        now: Optional[float] = None,
        speed_cap_mps: Optional[float] = None,
    ) -> tuple:
        theta = _normalize_angle(float(decision.theta_target))
        abs_theta = abs(theta)
        wz = _clamp(
            self.heading_kp * theta,
            -self.max_angular_speed_radps,
            self.max_angular_speed_radps,
        )

        if decision.state == 'blocked':
            return 0.0, 0.0
        if decision.state == 'stop':
            # A selected gap is usable for turning, but it is not a reason to
            # drive forward while the front safety distance is exceeded.
            if decision.choice is None:
                return 0.0, 0.0
            return 0.0, _clamp(
                wz * 0.75,
                -self.max_angular_speed_radps,
                self.max_angular_speed_radps,
            )
        if self._target_x is not None and abs_theta >= self.turn_in_place_heading_rad:
            return 0.0, wz

        heading_scale = 1.0 - _clamp(
            abs_theta / max(self.slowdown_heading_rad, 0.1),
            0.0,
            0.85,
        )
        clearance_scale = _clamp(
            (float(decision.front_clearance_m) - self._planner.min_stop_distance_m) /
            max(
                0.1,
                self._planner.clear_path_distance_m - self._planner.min_stop_distance_m,
            ),
            0.0,
            1.0,
        )
        desired_speed = self._speed_mps * min(
            heading_scale,
            max(0.25, clearance_scale),
        )
        if decision.state in {'avoid', 'recover'}:
            desired_speed *= 0.75 if decision.state == 'avoid' else 0.6
        if self._target_x is not None and self._requires_stop:
            remaining = self.target_remaining_m(pose=pose)
            if remaining is not None:
                stopping_boundary = self._effective_target_tolerance_m(now)
                distance_to_boundary = max(0.0, remaining - stopping_boundary)
                approach_speed = math.sqrt(
                    2.0 * self.brake_decel_mps2 * distance_to_boundary
                )
                if distance_to_boundary > 0.0:
                    approach_speed = max(self.goal_settle_speed_mps, approach_speed)
                desired_speed = min(desired_speed, approach_speed)
        if self._target_distance_m is not None and self._requires_stop:
            distance_to_boundary = max(
                0.0,
                self._target_distance_m -
                self.distance_tolerance_m -
                self.distance_traveled_m(),
            )
            approach_speed = math.sqrt(
                2.0 * self.brake_decel_mps2 * distance_to_boundary
            )
            if distance_to_boundary > 0.0:
                approach_speed = max(self.goal_settle_speed_mps, approach_speed)
            desired_speed = min(desired_speed, approach_speed)
        if self._target_duration_s is not None and self._requires_stop:
            remaining_time = self.duration_target_remaining_s(now)
            if remaining_time is not None:
                time_approach_speed = self.brake_decel_mps2 * remaining_time
                if remaining_time > 0.0:
                    time_approach_speed = max(
                        self.goal_settle_speed_mps,
                        time_approach_speed,
                    )
                desired_speed = min(desired_speed, time_approach_speed)
        if speed_cap_mps is not None:
            desired_speed = min(desired_speed, max(0.0, float(speed_cap_mps)))
        return _clamp(desired_speed, 0.0, self.max_speed_mps), wz

    def _theta_goal(self, pose: Optional[Dict[str, Any]]) -> float:
        if self._target_x is not None and self._target_y is not None:
            if self._pose_has_xy(pose):
                self._route_state = self._route_follower.update(
                    pose,
                    speed_mps=self._speed_mps,
                )
                if self._route_state.get('available', False):
                    return _normalize_angle(
                        float(self._route_state.get('theta_route_rad', 0.0))
                    )
                current_yaw = self._pose_yaw_rad(pose)
                if current_yaw is not None:
                    target_yaw = math.atan2(
                        float(self._target_y) - float(pose.get('y', 0.0)),
                        float(self._target_x) - float(pose.get('x', 0.0)),
                    )
                    return _normalize_angle(target_yaw - current_yaw)
            return 0.0

        current_yaw = self._pose_yaw_rad(pose)
        if self._yaw_ref_rad is None or current_yaw is None:
            return 0.0
        return _normalize_angle(self._yaw_ref_rad - current_yaw)

    def _current_pose(self, *, source: str = '') -> Dict[str, Any]:
        getter = self._pose_getter
        if source == 'map' and self._map_pose_getter is not None:
            getter = self._map_pose_getter
        if getter is None:
            return {}
        try:
            pose = getter() or {}
        except Exception:
            pose = {}
        return dict(pose)

    def _pose_yaw_rad(self, pose: Optional[Dict[str, Any]]) -> Optional[float]:
        if not pose:
            return None
        value = _finite_float(pose.get('yaw_deg'))
        if value is None:
            value = _finite_float(pose.get('yaw_rad'))
            return value
        return math.radians(value)

    def _pose_has_xy(self, pose: Optional[Dict[str, Any]]) -> bool:
        if not pose:
            return False
        return (
            _finite_float(pose.get('x')) is not None and
            _finite_float(pose.get('y')) is not None
        )

    def _pose_age_s(self, pose: Optional[Dict[str, Any]]) -> Optional[float]:
        if not pose:
            return None
        explicit_age = _finite_float(pose.get('age_s'))
        if explicit_age is not None and explicit_age >= 0.0:
            return explicit_age
        timestamp_ns = pose.get('timestamp_ns')
        if timestamp_ns in (None, ''):
            timestamp_ns = (
                int(pose.get('stamp_sec', 0) or 0) * 1_000_000_000 +
                int(pose.get('stamp_nanosec', 0) or 0)
            )
        try:
            timestamp_ns = int(timestamp_ns or 0)
            now_ns = int(self._now_ns_getter() or 0)
        except Exception:
            return None
        if timestamp_ns <= 0 or now_ns <= 0:
            return None
        return max(0.0, float(now_ns - timestamp_ns) / 1_000_000_000.0)

    def _update_path_distance(self, pose: Dict[str, Any]):
        if self._last_path_pose is None:
            self._last_path_pose = dict(pose)
            return
        try:
            dx = float(pose['x']) - float(self._last_path_pose['x'])
            dy = float(pose['y']) - float(self._last_path_pose['y'])
        except (KeyError, TypeError, ValueError):
            return
        step = math.hypot(dx, dy)
        if self.max_pose_step_m > 0.0 and step > self.max_pose_step_m:
            # Ignore an implausible localization jump, but use the new sample
            # as the next reference so one bad point does not add forever.
            self._last_path_pose = dict(pose)
            return
        self._path_length_m += step
        self._last_path_pose = dict(pose)

    def _mark_blocked(self, now: float):
        if self._blocked_since_s is None:
            self._blocked_since_s = now

    def _scan_age_at(self, now: float) -> Optional[float]:
        if self._latest_scan_monotonic_s is None:
            return None
        return max(0.0, now - self._latest_scan_monotonic_s)

    def _dynamic_stop_distance_m(self) -> float:
        speed = max(0.0, abs(self._last_vx))
        braking_distance = speed * speed / (2.0 * self.brake_decel_mps2)
        return max(
            self._planner.min_stop_distance_m,
            self._planner.min_stop_distance_m +
            speed * self.command_latency_s +
            braking_distance +
            self.stop_margin_m,
        )

    def _control_state(self) -> Dict[str, Any]:
        if self._control_state_getter is None:
            return {
                'control_mode': 'auto',
                'controller': 'auto',
                'bottom_status': '',
                'battery_level': None,
            }
        try:
            state = self._control_state_getter() or {}
        except Exception:
            state = {}
        if not isinstance(state, dict):
            state = {}
        return state

    def _handle_safety_block(
        self,
        now: float,
        *,
        code: str,
        reason: str,
        pose_available: bool,
        pose_age_s: Optional[float],
        scan_age_s: Optional[float],
        scan_quality: str,
        safety: Optional[Dict[str, Any]] = None,
        scan_diagnostics: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        new_fault = self._fault_since_s is None or self._last_safety_code != code
        if new_fault:
            self._fault_since_s = now
            self._last_safety_code = code
            if self._logger is not None:
                try:
                    self._logger.warning(f'local control safety stop: {code}: {reason}')
                except Exception:
                    pass
            try:
                self._move_gateway.stop(reason=code)
            except Exception:
                pass
        self._publish_velocity(
            0.0,
            0.0,
            reason=code,
            hard_stop=True,
            monotonic_s=now,
        )
        blocked_for = max(0.0, now - self._fault_since_s)
        result = {
            'state': 'safety_blocked',
            'active': True,
            'task_id': self._task_id,
            'reason': reason,
            'code': code,
            'blocked_for_s': blocked_for,
            'pose_available': pose_available,
            'pose_age_s': pose_age_s,
            'scan_age_s': scan_age_s,
            'scan_quality': scan_quality,
            'safety': dict(safety or self._last_safety),
        }
        result.update(dict(scan_diagnostics or {}))
        timeout = self.fault_pause_timeout_s
        if timeout > 0.0 and blocked_for >= timeout:
            self._active = False
            result['active'] = False
            result['terminal'] = True
        self._last_decision = dict(result)
        return result

    def _publish_velocity(
        self,
        vx: float,
        wz: float,
        *,
        reason: str,
        hard_stop: bool = False,
        force_zero_linear: bool = False,
        speed_cap_mps: Optional[float] = None,
        monotonic_s: Optional[float] = None,
    ) -> tuple:
        now = self._monotonic_getter() if monotonic_s is None else float(monotonic_s)
        if self._last_command_monotonic_s is not None:
            dt = now - self._last_command_monotonic_s
        else:
            dt = 1.0 / self.control_hz
        shaped_vx, shaped_wz = self._velocity_shaper.shape(
            vx,
            wz,
            dt_s=dt,
            hard_stop=hard_stop,
            force_zero_linear=force_zero_linear,
            speed_cap_mps=speed_cap_mps,
        )
        self._last_vx = float(shaped_vx)
        self._last_wz = float(shaped_wz)
        self._last_command_monotonic_s = now
        self._move_gateway.velocity(
            linear_x=self._last_vx,
            angular_z=self._last_wz,
            source=self._active_command_source,
            reason=reason,
            task_id=self._task_id,
        )
        return self._last_vx, self._last_wz

    def _optional_positive_or_none(self, value: Any) -> Optional[float]:
        if value in (None, ''):
            return None
        number = _finite_float(value)
        if number is None or number <= 0.0:
            return None
        return number
