"""Dependency-free SO101 forward and inverse kinematics.

The implementation follows the LeRobot SO101 chain: URDF-derived FK, an
analytic geometric Jacobian, and damped least-squares position IK.
"""
import math
import xml.etree.ElementTree as ET

def _identity():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat_mul(first, second):
    # All callers multiply rigid homogeneous transforms. Explicit affine
    # arithmetic avoids generator overhead on Python 3.6/aarch64.
    result = []
    for a, b, c, d in first[:3]:
        result.append([a*second[0][0]+b*second[1][0]+c*second[2][0],
                       a*second[0][1]+b*second[1][1]+c*second[2][1],
                       a*second[0][2]+b*second[1][2]+c*second[2][2],
                       a*second[0][3]+b*second[1][3]+c*second[2][3]+d])
    result.append([0.0, 0.0, 0.0, 1.0])
    return result


def _rpy_rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _origin_transform(xyz, rpy):
    result = _identity()
    rotation = _rpy_rotation(rpy[0], rpy[1], rpy[2])
    for row in range(3):
        for column in range(3):
            result[row][column] = rotation[row][column]
    result[0][3], result[1][3], result[2][3] = xyz
    return result


def _axis_rotation(axis, angle):
    x, y, z = axis
    sine = math.sin(angle)
    cosine = math.cos(angle)
    result = _identity()
    result[0][0] = cosine + x * x * (1.0 - cosine)
    result[0][1] = x * y * (1.0 - cosine) - z * sine
    result[0][2] = x * z * (1.0 - cosine) + y * sine
    result[1][0] = y * x * (1.0 - cosine) + z * sine
    result[1][1] = cosine + y * y * (1.0 - cosine)
    result[1][2] = y * z * (1.0 - cosine) - x * sine
    result[2][0] = z * x * (1.0 - cosine) - y * sine
    result[2][1] = z * y * (1.0 - cosine) + x * sine
    result[2][2] = cosine + z * z * (1.0 - cosine)
    return result


def _position(matrix):
    return [matrix[0][3], matrix[1][3], matrix[2][3]]


def _norm(values):
    return math.sqrt(sum(value * value for value in values))


def _cross(first, second):
    """Return the 3-D cross product without requiring NumPy."""
    return [
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    ]


def _rotate_vector(matrix, vector):
    """Rotate a vector by the rotational part of a homogeneous matrix."""
    return [sum(matrix[row][column] * vector[column] for column in range(3))
            for row in range(3)]


def _solve_3x3(matrix, vector):
    augmented = [list(matrix[row]) + [float(vector[row])] for row in range(3)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) <= 1e-12:
            raise ValueError('singular IK matrix')
        if pivot != column:
            augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        for item in range(column, 4):
            augmented[column][item] /= divisor
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            for item in range(column, 4):
                augmented[row][item] -= factor * augmented[column][item]
    return [augmented[row][3] for row in range(3)]


def _vector(text, fallback):
    if not text:
        return list(fallback)
    values = [float(item) for item in text.split()]
    if len(values) != 3:
        raise ValueError('URDF vector must have three values: {}'.format(text))
    return values


def _matrix_to_rpy(matrix):
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def _matrix_to_quaternion(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale
    return x, y, z, w


class SO101Kinematics(object):
    def __init__(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        by_child = {}
        for element in root.findall('joint'):
            origin = element.find('origin')
            axis = element.find('axis')
            parent = element.find('parent')
            child = element.find('child')
            origin_xyz = _vector(origin.get('xyz') if origin is not None else None,
                                 (0.0, 0.0, 0.0))
            origin_rpy = _vector(origin.get('rpy') if origin is not None else None,
                                 (0.0, 0.0, 0.0))
            axis_values = _vector(axis.get('xyz') if axis is not None else None,
                                  (0.0, 0.0, 0.0))
            axis_norm = _norm(axis_values)
            if axis_norm > 1e-12:
                axis_values = [value / axis_norm for value in axis_values]
            limit = element.find('limit')
            by_child[child.get('link')] = {
                'name': element.get('name'),
                'type': element.get('type'),
                'parent': parent.get('link'),
                'child': child.get('link'),
                'origin': _origin_transform(origin_xyz, origin_rpy),
                'axis': axis_values,
                'lower': float(limit.get('lower')) if limit is not None and limit.get('lower') else None,
                'upper': float(limit.get('upper')) if limit is not None and limit.get('upper') else None,
            }
        chain = []
        current = 'gripper_frame_link'
        while current != 'base_link':
            joint = by_child.get(current)
            if joint is None:
                raise ValueError('URDF chain is incomplete at {}'.format(current))
            chain.append(joint)
            current = joint['parent']
        chain.reverse()
        self.by_child = by_child
        self.chain = chain
        self.active = [joint for joint in chain if joint['type'] != 'fixed']

    def forward(self, joint_degrees):
        transform = _identity()
        active_index = 0
        for joint in self.chain:
            transform = _mat_mul(transform, joint['origin'])
            if joint['type'] != 'fixed':
                transform = _mat_mul(
                    transform,
                    _axis_rotation(joint['axis'], math.radians(joint_degrees[active_index])))
                active_index += 1
        return transform

    def forward_with_jacobian(self, joint_degrees, spatial=False):
        """Return FK and the geometric position Jacobian.

        The Jacobian is analytic (axis cross end-point vector), rather than a
        finite difference approximation.  This is the same geometric
        Jacobian used by mature KDL/TRAC-IK style solvers and is both faster
        and less noisy on the CyberDog's Python 3.6 image.
        """
        transform = _identity()
        active_index = 0
        joint_positions = []
        joint_axes = []
        for joint in self.chain:
            transform = _mat_mul(transform, joint['origin'])
            if joint['type'] != 'fixed':
                joint_positions.append(_position(transform))
                joint_axes.append(_rotate_vector(transform, joint['axis']))
                transform = _mat_mul(
                    transform,
                    _axis_rotation(joint['axis'],
                                   math.radians(joint_degrees[active_index])))
                active_index += 1
        end_position = _position(transform)
        jacobian = []
        for axis, position in zip(joint_axes, joint_positions):
            column = _cross(axis, [end_position[index] - position[index]
                                   for index in range(3)])
            jacobian.append(column + axis if spatial else column)
        # Internally the solver uses columns; expose a 3 x N matrix.
        return transform, [[jacobian[column][row]
                            for column in range(len(jacobian))]
                           for row in range(6 if spatial else 3)]

    def inverse_position(self, target_mm, seed_degrees, max_iterations=100,
                         joint_limits_degrees=None, damping=0.02,
                         max_step_degrees=8.0, tolerance_mm=1.5,
                         acceptance_mm=5.0):
        """Solve position IK with damped least-squares and line search.

        This is a dependency-free Levenberg-Marquardt/DLS solver.  It follows
        the well-tested KDL/TRAC-IK approach while using the SO-101 chain
        loaded from URDF, which avoids adding unsupported native packages to
        ROS2 Galactic on the robot.
        """
        target = [float(value) / 1000.0 for value in target_mm]
        joints = [float(value) for value in seed_degrees]
        lower = [math.degrees(joint['lower']) for joint in self.active]
        upper = [math.degrees(joint['upper']) for joint in self.active]
        if joint_limits_degrees is not None:
            for index, bounds in enumerate(joint_limits_degrees):
                lower[index] = max(lower[index], float(bounds[0]))
                upper[index] = min(upper[index], float(bounds[1]))
        joints = [min(upper[index], max(lower[index], joints[index]))
                  for index in range(5)]
        damping = max(1e-5, float(damping))
        max_step = math.radians(max(0.1, float(max_step_degrees)))
        tolerance = max(0.1, float(tolerance_mm))
        acceptance = max(tolerance, float(acceptance_mm))
        for iteration in range(max_iterations):
            matrix, jacobian = self.forward_with_jacobian(joints)
            current = _position(matrix)
            error = [target[index] - current[index] for index in range(3)]
            error_mm = _norm(error) * 1000.0
            if error_mm <= tolerance:
                return {'success': True, 'iterations': iteration,
                        'position_error_mm': error_mm, 'joints': list(joints)}
            normal = [[0.0 for _ in range(3)] for _ in range(3)]
            for row in range(3):
                for column in range(3):
                    normal[row][column] = sum(
                        jacobian[row][item] * jacobian[column][item]
                        for item in range(5))
                normal[row][row] += damping * damping
            correction = _solve_3x3(normal, error)
            delta = [sum(jacobian[row][index] * correction[row] for row in range(3))
                     for index in range(5)]
            delta_norm = _norm(delta)
            if delta_norm > max_step:
                delta = [value * max_step / delta_norm for value in delta]

            # Backtracking keeps the solver from accepting a step that makes
            # the Cartesian error worse near a joint limit or singularity.
            best = list(joints)
            best_error = error_mm
            for scale in (1.0, 0.5, 0.25, 0.1):
                candidate = [min(upper[index], max(
                    lower[index], joints[index] +
                    math.degrees(delta[index]) * scale))
                    for index in range(5)]
                candidate_error = _norm([
                    target[index] - _position(self.forward(candidate))[index]
                    for index in range(3)]) * 1000.0
                if candidate_error < best_error:
                    best, best_error = candidate, candidate_error
                    break
            joints = best
            if best_error >= error_mm:
                # Increase damping on a stalled iteration; this is the
                # Levenberg-Marquardt safeguard used by practical solvers.
                damping = min(1.0, damping * 2.0)
        final_position = _position(self.forward(joints))
        final_error = _norm([target[index] - final_position[index]
                             for index in range(3)]) * 1000.0
        return {'success': final_error <= acceptance, 'iterations': max_iterations,
                'position_error_mm': final_error, 'joints': list(joints)}



    def link_transforms(self, degrees, gripper_percent=50.0):
        """Full URDF tree, including the moving jaw (normalized 0..100%)."""
        angles = dict((j['name'], math.radians(v)) for j, v in zip(self.active, degrees))
        result = {'base_link': _identity()}
        pending = list(self.by_child.values())
        while pending:
            progress = False
            for joint in list(pending):
                if joint['parent'] not in result:
                    continue
                t = _mat_mul(result[joint['parent']], joint['origin'])
                if joint['type'] != 'fixed':
                    if joint['name'] == 'gripper':
                        angle = joint['lower'] + gripper_percent / 100.0 * (joint['upper'] - joint['lower'])
                    else:
                        angle = angles[joint['name']]
                    t = _mat_mul(t, _axis_rotation(joint['axis'], angle))
                result[joint['child']] = t
                pending.remove(joint)
                progress = True
            if not progress:
                raise ValueError('disconnected URDF tree')
        return result

    def joint_bounds(self, joint_limits_degrees=None, calibrated_limit_joints=()):
        """Resolve named calibration overrides without changing URDF geometry.

        Unselected joints use the intersection with the reference URDF.
        Selected joints require finite calibrated bounds; limits never vanish.
        """
        if (not isinstance(calibrated_limit_joints, (list, tuple)) or
                any(not isinstance(name, str) for name in calibrated_limit_joints)):
            raise ValueError('calibrated_limit_joints must be a list of joint names')
        names = {joint['name'] for joint in self.active}
        selected = set(calibrated_limit_joints)
        if not selected.issubset(names) or len(selected) != len(calibrated_limit_joints):
            raise ValueError('calibrated_limit_joints contains unknown or duplicate joints')
        if selected and joint_limits_degrees is None:
            raise ValueError('calibrated joint bounds are required for limit overrides')
        if joint_limits_degrees is not None and len(joint_limits_degrees) != len(self.active):
            raise ValueError('one calibrated range per arm joint is required')
        result = []
        for index, joint in enumerate(self.active):
            low, high = math.degrees(joint['lower']), math.degrees(joint['upper'])
            if joint_limits_degrees is not None:
                bounds = joint_limits_degrees[index]
                if len(bounds) != 2 or not all(math.isfinite(v) for v in bounds) or bounds[0] >= bounds[1]:
                    raise ValueError('invalid calibrated range for ' + joint['name'])
                if joint['name'] in selected:
                    low, high = bounds
                else:
                    low, high = max(low, bounds[0]), min(high, bounds[1])
            if low >= high:
                raise ValueError('empty effective range for ' + joint['name'])
            result.append((low, high))
        return result

    def inverse_pose(self, target, seed, joint_limits_degrees=None,
                     max_step_degrees=3.0, max_iterations=12, damping=0.01,
                     orientation_weight=0.005, calibrated_limit_joints=(), checkpoint=None):
        """Weighted spatial DLS. Bound TOTAL change relative to the input seed.

        Position residual is metres; orientation residual is the base-frame
        SO(3) rotation vector. Orientation weight multiplies its squared cost.
        A five-axis arm returns residuals, never pretends arbitrary 6D success.
        """
        if len(seed) != 5 or not all(math.isfinite(float(x)) for x in seed):
            raise ValueError('IK requires five finite joint angles')
        if not all(math.isfinite(float(x)) for row in target for x in row):
            raise ValueError('non-finite pose')
        if not (max_step_degrees > 0 and damping > 0 and orientation_weight >= 0):
            raise ValueError('invalid IK parameters')
        bounds = []
        for i, (low, high) in enumerate(self.joint_bounds(joint_limits_degrees, calibrated_limit_joints)):
            low, high = max(low, seed[i] - max_step_degrees), min(high, seed[i] + max_step_degrees)
            if low > high or seed[i] < low - 1e-7 or seed[i] > high + 1e-7:
                raise ValueError('measured joint outside effective URDF/calibration limits')
            bounds.append((low, high))
        joints = list(seed)
        weights = [1.0] * 3 + [orientation_weight] * 3
        lam = damping
        initial_cost = None
        for iteration in range(max_iterations):
            if checkpoint:
                checkpoint()
            matrix, jac = self.forward_with_jacobian(joints, spatial=True)
            error = pose_error(target, matrix)
            cost = sum(w * e * e for w, e in zip(weights, error))
            if initial_cost is None:
                initial_cost = cost
            if cost < 1e-14:
                break
            normal = [[sum(weights[r] * jac[r][i] * jac[r][j] for r in range(6))
                       + (lam * lam if i == j else 0.0) for j in range(5)] for i in range(5)]
            rhs = [sum(weights[r] * jac[r][i] * error[r] for r in range(6)) for i in range(5)]
            delta = solve_linear(normal, rhs)
            improved = False
            for scale in (1.0, 0.5, 0.25, 0.125):
                candidate = [max(bounds[i][0], min(bounds[i][1], joints[i] + math.degrees(delta[i]) * scale)) for i in range(5)]
                e = pose_error(target, self.forward(candidate))
                new_cost = sum(w * v * v for w, v in zip(weights, e))
                if new_cost < cost - 1e-16:
                    joints, improved = candidate, True
                    lam = max(damping, lam * 0.7)
                    break
            if not improved:
                lam = min(1.0, lam * 3.0)
                if lam >= 1.0:
                    break
        error = pose_error(target, self.forward(joints))
        return {'joints': joints, 'position_error_mm': _norm(error[:3]) * 1000.0,
                'orientation_error_rad': _norm(error[3:]), 'iterations': iteration + 1,
                'cost': sum(w * v * v for w, v in zip(weights, error)),
                'initial_cost': initial_cost}


def solve_linear(matrix, vector):
    n = len(vector)
    a = [list(row) + [float(v)] for row, v in zip(matrix, vector)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(a[row][col]))
        if abs(a[pivot][col]) < 1e-14:
            raise ValueError('singular IK matrix')
        a[col], a[pivot] = a[pivot], a[col]
        divisor = a[col][col]
        a[col] = [v / divisor for v in a[col]]
        for row in range(n):
            if row != col:
                factor = a[row][col]
                a[row] = [v - factor * w for v, w in zip(a[row], a[col])]
    return [row[-1] for row in a]


def quaternion_transform(position, xyzw):
    if len(position) != 3 or len(xyzw) != 4:
        raise ValueError('invalid pose dimensions')
    vals = list(position) + list(xyzw)
    if not all(math.isfinite(float(v)) for v in vals):
        raise ValueError('pose must be finite')
    norm = _norm(xyzw)
    if norm < 1e-8:
        raise ValueError('zero quaternion')
    x, y, z, w = [v / norm for v in xyzw]
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), position[0]],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), position[1]],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), position[2]], [0, 0, 0, 1]]


def rotation_error(target, current):
    delta = [[sum(target[i][k] * current[j][k] for k in range(3))
              for j in range(3)] for i in range(3)]
    x, y, z, w = _matrix_to_quaternion(delta)
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    sine = _norm((x, y, z))
    if sine < 1e-10:
        return [2*x, 2*y, 2*z]
    scale = 2.0 * math.atan2(sine, max(0.0, w)) / sine
    return [scale*x, scale*y, scale*z]


def pose_error(target, current):
    return [target[i][3] - current[i][3] for i in range(3)] + rotation_error(target, current)
