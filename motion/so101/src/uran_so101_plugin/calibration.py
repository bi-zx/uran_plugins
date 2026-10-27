"""SO-101 raw-count calibration helpers.

The calibration file intentionally contains no guessed mechanical limits.  A
file is considered usable only after every joint has raw_min/raw_max values.
"""

from __future__ import print_function

import math
import json
import os

import yaml


JOINT_NAMES = (
    'shoulder_pan', 'shoulder_lift', 'elbow_flex',
    'wrist_flex', 'wrist_roll', 'gripper',
)


def empty_calibration():
    joints = {}
    for index, name in enumerate(JOINT_NAMES):
        joints[name] = {
            'id': index + 1,
            'raw_center': None,
            'raw_min': None,
            'raw_max': None,
            'direction': 1,
        }
    return {
        'version': 1,
        'enabled': False,
        'joints': joints,
    }


def load_calibration(path):
    data = empty_calibration()
    if not path or not os.path.isfile(path):
        return data
    with open(path, 'r') as stream:
        loaded = yaml.safe_load(stream) or {}
    if not isinstance(loaded, dict):
        return data
    for key in ('version', 'enabled'):
        if key in loaded:
            data[key] = loaded[key]
    loaded_joints = loaded.get('joints')
    # Present_Position already includes the servo homing offset. Match
    # RealSO101: calibrated range midpoint for arm joints, drive_mode only
    # reverses the normalized gripper coordinate.
    if not isinstance(loaded_joints, dict) and all(isinstance(loaded.get(name), dict) for name in JOINT_NAMES):
        loaded_joints = loaded
        data['enabled'] = True
        for name in JOINT_NAMES:
            values = loaded_joints[name]
            data['joints'][name].update({
                'id': int(values.get('id', data['joints'][name]['id'])),
                'raw_min': int(values['range_min']),
                'raw_max': int(values['range_max']),
                'raw_center': (int(values['range_min']) + int(values['range_max'])) / 2.0,
                'direction': -1 if name == 'gripper' and int(values.get('drive_mode', 0)) else 1,
                'drive_mode': int(values.get('drive_mode', 0)),
                'homing_offset': int(values.get('homing_offset', 0)),
            })
        return data
    if isinstance(loaded_joints, dict):
        for name in JOINT_NAMES:
            values = loaded_joints.get(name)
            if isinstance(values, dict):
                data['joints'][name].update(values)
    return data


def load_safety_range(path):
    """Load a LeRobot SO-101 calibration JSON as raw-position limits."""
    if not path or not os.path.isfile(path):
        raise IOError('safety range file not found: {}'.format(path))
    with open(path, 'r') as stream:
        loaded = json.load(stream)
    if not isinstance(loaded, dict):
        raise ValueError('safety range JSON must contain an object')

    result = {}
    for index, name in enumerate(JOINT_NAMES, 1):
        values = loaded.get(name)
        if not isinstance(values, dict):
            raise ValueError('safety range missing joint {}'.format(name))
        try:
            motor_id = int(values['id'])
            range_min = int(values['range_min'])
            range_max = int(values['range_max'])
            drive_mode = int(values.get('drive_mode', 0))
            homing_offset = int(values.get('homing_offset', 0))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('invalid safety range for {}: {}'.format(name, exc))
        if motor_id != index:
            raise ValueError('{} safety range id must be {}'.format(name, index))
        if not (0 <= range_min < range_max <= 4095):
            raise ValueError('{} safety range must satisfy 0 <= min < max <= 4095'.format(name))
        if drive_mode not in (0, 1):
            raise ValueError('{} drive_mode must be 0 or 1'.format(name))
        result[name] = {
            'id': motor_id,
            'drive_mode': drive_mode,
            'homing_offset': homing_offset,
            'range_min': range_min,
            'range_max': range_max,
        }
    return result


def apply_safety_range(data, path):
    """Intersect explicit limits; never manufacture or change calibration."""
    if not is_complete(data):
        raise ValueError('a complete calibration is required before safety limits')
    profile = load_safety_range(path)
    for name in JOINT_NAMES:
        current = data['joints'][name]
        safety = profile[name]
        low = max(current.get('safe_min', current['raw_min']), safety['range_min'])
        high = min(current.get('safe_max', current['raw_max']), safety['range_max'])
        if low >= high:
            raise ValueError('{} calibration and safety ranges do not overlap'.format(name))
        current.update(safe_min=low, safe_max=high)
    data['safety_range_file'] = path
    return data


def narrow_safety_range(data, scale=1.0):
    """Shrink effective raw limits around each calibrated center.

    ``scale=1`` preserves the LeRobot profile.  A value such as ``0.82``
    keeps 82 percent of the available travel on each side of the calibrated
    zero, preventing edge-of-range IK solutions while retaining the user's
    recorded center.  The original JSON limits remain available as
    ``safety_range_min/max`` for diagnostics.
    """
    try:
        scale = float(scale)
    except (TypeError, ValueError):
        raise ValueError('safety_range_scale must be a finite number')
    if not math.isfinite(scale) or not 0.0 < scale <= 1.0:
        raise ValueError('safety_range_scale must be in (0, 1]')
    for name in JOINT_NAMES:
        values = _joint_config(data, name)
        try:
            raw_min = float(values['raw_min'])
            raw_max = float(values['raw_max'])
            center = float(values['raw_center'])
        except (KeyError, TypeError, ValueError):
            raise ValueError('{} has incomplete raw safety range'.format(name))
        if not (0.0 <= raw_min < center < raw_max <= 4095.0):
            raise ValueError('{} has invalid raw center/range'.format(name))
        # Shrink each side independently so an off-centre SO-101 calibration
        # is preserved instead of being moved to the numerical midpoint.
        narrowed_min = int(math.ceil(center - (center - raw_min) * scale))
        narrowed_max = int(math.floor(center + (raw_max - center) * scale))
        narrowed_min = max(int(math.ceil(raw_min)), narrowed_min)
        narrowed_max = min(int(math.floor(raw_max)), narrowed_max)
        if narrowed_min >= narrowed_max:
            raise ValueError('{} narrowed safety range is empty'.format(name))
        values['safe_min'] = max(values.get('safe_min', raw_min), narrowed_min)
        values['safe_max'] = min(values.get('safe_max', raw_max), narrowed_max)
    data['safety_range_scale'] = scale
    return data


def validate_raw_targets(data, targets):
    """Reject raw commands outside the effective calibrated safety bounds."""
    if len(targets) != len(JOINT_NAMES):
        raise ValueError('six raw joint targets are required')
    for name, value in zip(JOINT_NAMES, targets):
        values = _joint_config(data, name)
        raw_min = float(values.get('safe_min', values['raw_min']))
        raw_max = float(values.get('safe_max', values['raw_max']))
        if not math.isfinite(float(value)) or value < raw_min or value > raw_max:
            raise ValueError(
                '{} raw target {} is outside safe range [{}, {}]'.format(
                    name, value, raw_min, raw_max))


def save_calibration(path, data):
    parent = os.path.dirname(path)
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    with open(path, 'w') as stream:
        # PyYAML on the CyberDog image may predate the sort_keys argument.
        yaml.safe_dump(data, stream, default_flow_style=False)


def _joint_config(data, name):
    joints = data.get('joints') if isinstance(data, dict) else None
    values = joints.get(name) if isinstance(joints, dict) else None
    return values if isinstance(values, dict) else {}


def is_complete(data):
    if not isinstance(data, dict) or not bool(data.get('enabled', False)):
        return False
    for index, name in enumerate(JOINT_NAMES, 1):
        values = _joint_config(data, name)
        try:
            low, high, center, direction = _validated_joint(data, name)
            if int(values['id']) != index:
                return False
            safe_low = float(values.get('safe_min', low))
            safe_high = float(values.get('safe_max', high))
            if not low <= safe_low < safe_high <= high:
                return False
        except (KeyError, TypeError, ValueError, OverflowError):
            return False
    return True


def _validated_joint(data, name):
    values = _joint_config(data, name)
    raw_min = float(values['raw_min'])
    raw_max = float(values['raw_max'])
    direction = int(values.get('direction', 1))
    raw_zero = values.get('raw_center')
    raw_zero = float(raw_zero)
    if not (0 <= raw_min < raw_max <= 4095):
        raise ValueError('{} has invalid raw limits'.format(name))
    if not (raw_min <= raw_zero <= raw_max):
        raise ValueError('{} raw_center is outside raw limits'.format(name))
    if direction not in (-1, 1):
        raise ValueError('{} direction must be 1 or -1'.format(name))
    return raw_min, raw_max, raw_zero, direction


def raw_to_values(data, positions):
    """Return first five joint values in degrees and gripper in percent."""
    if not is_complete(data):
        raise ValueError('calibration is incomplete or disabled')
    result = []
    for index, name in enumerate(JOINT_NAMES):
        raw_min, raw_max, raw_zero, direction = _validated_joint(data, name)
        raw = float(positions[index + 1])
        if not math.isfinite(raw):
            raise ValueError('non-finite measured position')
        if name == 'gripper':
            value = (raw - raw_min) * 100.0 / (raw_max - raw_min)
            value = 100.0 - value if direction < 0 else value
        else:
            value = direction * (raw - raw_zero) * 360.0 / 4095.0
        result.append(value)
    return result


def values_to_raw(data, values):
    """Convert degrees (five joints) and gripper percent to raw positions."""
    if not is_complete(data):
        raise ValueError('calibration is incomplete or disabled')
    if len(values) != len(JOINT_NAMES):
        raise ValueError('six calibrated joint values are required')
    result = []
    for name, value in zip(JOINT_NAMES, values):
        raw_min, raw_max, raw_zero, direction = _validated_joint(data, name)
        value = float(value)
        if not math.isfinite(value):
            raise ValueError('{} command is not finite'.format(name))
        if name == 'gripper':
            if not 0.0 <= value <= 100.0:
                raise ValueError('gripper percent outside [0, 100]')
            value = 100.0 - value if direction < 0 else value
            raw = raw_min + value * (raw_max - raw_min) / 100.0
        else:
            raw = raw_zero + direction * value * 4095.0 / 360.0
        if raw < raw_min - 1e-8 or raw > raw_max + 1e-8:
            raise ValueError('{} target exceeds calibration range'.format(name))
        result.append(int(round(raw)))
    return result


def joint_limits(data):
    """Effective angular limits without changing the calibrated zero."""
    if not is_complete(data):
        raise ValueError('calibration is incomplete or disabled')
    result = []
    for name in JOINT_NAMES[:5]:
        low, high, center, direction = _validated_joint(data, name)
        item = data['joints'][name]
        # Whole encoder ticks are the actual writable boundary.
        low = math.ceil(item.get('safe_min', low))
        high = math.floor(item.get('safe_max', high))
        result.append(tuple(sorted(direction * (v - center) * 360.0 / 4095.0
                                   for v in (low, high))))
    return result


def gripper_limits(data):
    item = data['joints']['gripper']
    raw = {i: data['joints'][n]['raw_center'] for i, n in enumerate(JOINT_NAMES, 1)}
    result = []
    for v in (item.get('safe_min', item['raw_min']), item.get('safe_max', item['raw_max'])):
        raw[6] = v
        result.append(raw_to_values(data, raw)[5])
    return tuple(sorted(result))
