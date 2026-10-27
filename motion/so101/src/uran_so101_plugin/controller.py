"""Deterministic fixed-period Cartesian controller, independent of transport."""
import copy
import math

from .calibration import joint_limits, gripper_limits, values_to_raw, validate_raw_targets, raw_to_values
from .kinematics import (_mat_mul, _axis_rotation, _matrix_to_rpy, _rpy_rotation,
                         _norm, pose_error, quaternion_transform)


class CartesianController(object):
    def __init__(self, kinematics, calibration, guard, config):
        self.kin = kinematics
        self.calibration = calibration
        self.guard = guard
        self.calibrated_limit_joints = config.get('calibrated_limit_joints', [])
        self.limits = kinematics.joint_bounds(joint_limits(calibration), self.calibrated_limit_joints)
        self.gripper_limits = gripper_limits(calibration)
        def positive(name, default, maximum):
            value = float(config.get(name, default))
            if not math.isfinite(value) or not 0 < value <= maximum:
                raise ValueError('invalid ' + name)
            return value
        self.hz = positive('control_hz', 20.0, 100.0)
        self.timeout = positive('command_timeout_s', 0.2, 1.0)
        self.filter_tau = float(config.get('velocity_filter_tau_s', 0.07))
        if not math.isfinite(self.filter_tau) or not 0 <= self.filter_tau <= 0.5:
            raise ValueError('invalid velocity filter time constant')
        self.joint_speed = positive('joint_speed_deg_s', 60.0, 180.0)
        self.frame_step = positive('max_joint_step_degrees', 3.0, 3.0)
        self.gripper_speed = positive('gripper_speed_percent_s', 100.0, 200.0)
        self.linear_speed = positive('max_linear_mps', 0.05, 0.1)
        self.angular_speed = positive('max_angular_rps', 0.5, 1.0)
        self.tracking_limit = positive('max_tracking_error_m', 0.02, 0.05)
        self.orientation_weight = positive('orientation_weight', 0.005, 1.0)
        self.damping = positive('ik_damping', 0.01, 1.0)
        self.iterations = int(positive('ik_max_iterations', 12, 100))
        self.workspace = config.get('workspace_m', [[-0.35, 0.35], [-0.35, 0.35], [-0.2, 0.45]])
        if len(self.workspace) != 3 or any(len(b) != 2 or not all(math.isfinite(v) for v in b) or b[0] >= b[1] for b in self.workspace):
            raise ValueError('invalid Cartesian workspace')
        self.mode = 'idle'
        self.q = None
        self.target = None
        self.last = {}
        self.deadline = 0
        self.previous_time = None

    def fork(self):
        """A control frame may compute without mutating the live controller."""
        other = copy.copy(self)
        for name in ('q', 'target', 'last', 'filtered', 'body', 'linear', 'orientation'):
            if hasattr(self, name):
                setattr(other, name, copy.deepcopy(getattr(self, name)))
        other.guard = self.guard.fork()
        return other

    def reset(self, measured, now):
        self.q = list(measured)
        self.target = self.kin.forward(measured[:5])
        self.gripper_target = measured[5]
        self.mode = 'idle'
        self.last = {'mode': 'idle'}
        self.filtered = [0.0] * 4
        self.previous_time = now

    def validate_start(self, measured):
        if len(measured) != 6 or not all(math.isfinite(v) for v in measured):
            raise ValueError('six finite measured positions are required')
        validate_raw_targets(self.calibration, values_to_raw(self.calibration, measured))
        for index, joint in enumerate(self.kin.active):
            low, high = self.limits[index]
            if not low <= measured[index] <= high:
                raise ValueError('{} measured {:.2f} deg outside [{:.2f}, {:.2f}]; restore a valid starting pose'.format(
                    joint['name'], measured[index], low, high))

    def command(self, body, measured, now, deadline):
        kind = body['kind']
        if kind == 'arm_pose':
            if body.get('frame_id', 'base') != 'base':
                raise ValueError('SO101 pose requires frame_id=base')
            target = quaternion_transform(body['position_m'], body['orientation_xyzw'])
            if any(not b[0] <= target[i][3] <= b[1] for i, b in enumerate(self.workspace)):
                raise ValueError('pose outside Cartesian workspace')
        elif kind == 'arm_cartesian_velocity':
            if body['frame_id'] != 'base':
                raise ValueError('SO101 velocity requires frame_id=base')
            if body['angular_rps']['x'] != 0 or body['angular_rps']['y'] != 0:
                raise ValueError('SO101 supports base-Z angular velocity only')
            numbers = list(body['linear_mps'].values()) + list(body['angular_rps'].values()) + list((body.get('orientation_rad') or {}).values())
            if not all(math.isfinite(v) for v in numbers):
                raise ValueError('non-finite Cartesian command')
        elif kind != 'gripper_action' or body.get('action') != 'toggle':
            raise ValueError('unsupported SO101 payload')
        self.validate_start(measured)
        previous_mode = self.mode
        same_velocity = (previous_mode == 'velocity' and now < self.deadline
                         and kind == 'arm_cartesian_velocity' and body == self.body)
        if (self.q is None or previous_mode != 'velocity' or kind != 'arm_cartesian_velocity'
                or now >= self.deadline):
            self.reset(measured, now)
        self.deadline = deadline
        if kind == 'arm_pose':
            self.mode, self.target = 'pose', target
        elif kind == 'gripper_action':
            self.mode = 'gripper'
            self.gripper_target = self.gripper_limits[0] if measured[5] >= 50 else self.gripper_limits[1]
        else:
            self.mode = 'velocity'
            self.body = copy.deepcopy(body)
            self.deadline = min(deadline, now + self.timeout)
            self.linear = [max(-self.linear_speed, min(self.linear_speed, body['linear_mps'][a])) for a in ('x', 'y', 'z')]
            self.angular = max(-self.angular_speed, min(self.angular_speed, body['angular_rps']['z']))
            self.orientation = dict(body.get('orientation_rad') or {})
            # A zero packet cancels accumulated travel immediately. If an
            # absolute orientation is supplied, that remains a real target.
            if not any(self.linear) and not self.angular and not self.orientation:
                self.reset(measured, now)
        if not same_velocity:
            self.last = {'mode': self.mode, 'reason': 'zero_velocity' if self.mode == 'idle' else 'awaiting_control_frame'}

    def step(self, now, measured, checkpoint=None):
        dt = min(1.0 / self.hz, max(0.0, now - self.previous_time))
        self.previous_time = now
        if self.mode == 'idle':
            # Preserve the last stop/failure diagnostics until a new command.
            # Terminal notification is emitted once by the completing frame.
            self.last.pop('terminal', None)
            return None
        if now >= self.deadline:
            mode = self.mode
            blocked = self.last.get('reason') == 'self_collision'
            collision = self.last.get('collision')
            self.reset(measured, now)
            self.last = {'mode': 'idle', 'reason': 'watchdog' if mode == 'velocity' else 'deadline',
                         'terminal': 'succeeded' if mode == 'velocity' and not blocked else 'failed',
                         'blocked_before_timeout': blocked}
            if blocked:
                self.last['collision'] = collision
            return list(measured)  # cancel the outstanding servo goal
        if dt <= 0:
            return None
        if max(abs(a-b) for a, b in zip(measured[:5], self.q[:5])) > 10.0:
            self.reset(measured, now)
            self.last = {'mode': 'idle', 'reason': 'feedback_tracking_error', 'terminal': 'failed'}
            return list(measured)
        snapshot = copy.deepcopy(self.target)
        rejected = None
        filtered = list(self.filtered)
        if self.mode == 'velocity':
            alpha = 1.0 - math.exp(-dt/self.filter_tau) if self.filter_tau else 1.0
            filtered = [a + alpha*(b-a) for a,b in zip(filtered, self.linear+[self.angular])]
        if checkpoint:
            checkpoint()
        # All backtracking candidates start at the same measured pose. Reuse
        # only its distances, never the per-path monotonic recovery bounds.
        start_distances = self.guard.distances(measured)
        actual = self.kin.forward(measured[:5])
        for scale in (1.0, 0.5, 0.25, 0.125, 0.0625):
            if checkpoint:
                checkpoint()
            target = copy.deepcopy(snapshot)
            if self.mode == 'velocity':
                delta = [target[i][3] + filtered[i]*dt*scale - actual[i][3] for i in range(3)]
                norm = _norm(delta)
                if norm > self.tracking_limit:
                    delta = [v*self.tracking_limit/norm for v in delta]
                for i, bounds in enumerate(self.workspace):
                    # If measured pose starts outside the workspace, only allow
                    # commanded recovery; clamping must not cause an unsolicited jump.
                    low, high = min(bounds[0], actual[i][3]), max(bounds[1], actual[i][3])
                    target[i][3] = max(low, min(high, actual[i][3] + delta[i]))
                rotation = _mat_mul(_axis_rotation([0, 0, 1], filtered[3]*dt*scale), target)
                for i in range(3):
                    target[i][:3] = rotation[i][:3]
                if self.orientation:
                    rpy = list(_matrix_to_rpy(target))
                    for i, name in enumerate(('roll', 'pitch', 'yaw')):
                        if name in self.orientation:
                            rpy[i] = self.orientation[name]
                    rotation = _rpy_rotation(*rpy)
                    for i in range(3):
                        target[i][:3] = rotation[i]
                # Bound orientation wind-up as well as translational wind-up.
                from .kinematics import rotation_error
                error = rotation_error(target, actual)
                angle = _norm(error)
                if angle > 0.35:
                    bounded = _mat_mul(_axis_rotation([v/angle for v in error], 0.35), actual)
                    for i in range(3):
                        target[i][:3] = bounded[i][:3]
            if self.mode == 'gripper':
                candidate = list(self.q)
                delta = max(-self.gripper_speed*dt*scale, min(self.gripper_speed*dt*scale, self.gripper_target - self.q[5]))
                candidate[5] += delta
                result = {'position_error_mm': 0.0, 'orientation_error_rad': 0.0}
            else:
                result = self.kin.inverse_pose(target, self.q[:5], self.limits,
                    max_step_degrees=min(self.frame_step, self.joint_speed*dt)*scale,
                    max_iterations=self.iterations, damping=self.damping,
                    orientation_weight=self.orientation_weight,
                    calibrated_limit_joints=self.calibrated_limit_joints,
                    checkpoint=checkpoint)
                candidate = result['joints'] + [self.q[5]]
            raw = values_to_raw(self.calibration, candidate)
            previous_raw = values_to_raw(self.calibration, self.q)
            # Bound the writable encoder delta too: rounding two consecutive
            # angles can otherwise turn a 3-degree bound into 35 ticks (>3 deg).
            ticks = int(math.floor(min(self.frame_step, self.joint_speed*dt)*scale*4095.0/360.0))
            for index in range(5):
                limited = max(previous_raw[index]-ticks, min(previous_raw[index]+ticks, raw[index]))
                if limited != raw[index]:
                    raw[index] = limited
                    candidate[index] = raw_to_values(self.calibration, dict(enumerate(raw, 1)))[index]
            validate_raw_targets(self.calibration, raw)
            quantized = raw_to_values(self.calibration, dict(enumerate(raw, 1)))
            ok, detail = self.guard.check_path(measured, quantized, checkpoint, start_distances)
            if not ok:
                rejected = detail
                continue
            self.filtered = filtered
            self.q = list(candidate)
            self.target = target
            self.last = dict(result, mode=self.mode, collision=detail, backtrack_scale=scale)
            error = pose_error(self.target, self.kin.forward(measured[:5]))
            reached = (self.mode == 'pose' and _norm(error[:3]) < 0.0015 and _norm(error[3:]) < math.radians(5))
            reached = reached or (self.mode == 'gripper' and abs(measured[5] - self.gripper_target) < 1.0)
            if reached:
                self.mode = 'idle'
                self.last['terminal'] = 'succeeded'
            return candidate
        # All tentative controller state is discarded on collision failure.
        self.last = {'mode': self.mode, 'reason': 'self_collision', 'collision': rejected}
        return None
