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
    return [[sum(first[row][item] * second[item][column] for item in range(4))
             for column in range(4)] for row in range(4)]


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

    def forward_with_jacobian(self, joint_degrees):
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
            jacobian.append(column)
        # Internally the solver uses columns; expose a 3 x N matrix.
        return transform, [[jacobian[column][row]
                            for column in range(len(jacobian))]
                           for row in range(3)]

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


