#!/usr/bin/env python3
"""Python 3.6 diagnostic/benchmark; optional hardware access is READ ONLY.

Never sends Goal_Position, torque, EEPROM, calibration or MQTT commands.
Stop all other serial owners before using --read-port.
"""
import argparse
import json
import os
import platform
import statistics
import time

from uran_so101_plugin.calibration import load_calibration, raw_to_values, values_to_raw, joint_limits
from uran_so101_plugin.kinematics import SO101Kinematics
from uran_so101_plugin.collision import CapsuleGuard
from uran_so101_plugin.controller import CartesianController


def verify(config_dir, calibration_path, read_port=None, frames=100, calibrated_limit_joints=()):
    calibration = load_calibration(calibration_path)
    urdf = os.path.join(config_dir, 'so101_collision.urdf')
    kin = SO101Kinematics(urdf)
    guard = CapsuleGuard(kin, os.path.join(config_dir, 'collision_capsules.json'), urdf)
    limits = kin.joint_bounds(joint_limits(calibration), calibrated_limit_joints)
    output = {'python': platform.python_version(), 'machine': platform.machine(),
              'hardware_writes': 0, 'frames': frames, 'joint_limits_deg': limits,
              'calibrated_limit_joints': list(calibrated_limit_joints)}
    if read_port:
        from uran_so101_plugin.feetech_bus import FeetechBus
        bus = FeetechBus(read_port, 1000000, .08)
        try:
            bus.open()
            start = time.monotonic()
            raw = bus.read_positions(range(1,7))
            read_ms = (time.monotonic()-start)*1000
        finally:
            bus.close()
        values = raw_to_values(calibration, raw)
        transform = kin.forward(values[:5])
        output['hardware_read'] = {'raw': raw, 'values': values, 'read_ms': read_ms,
            'position_m': [transform[i][3] for i in range(3)],
            'calibration_roundtrip_ticks': max(abs(values_to_raw(calibration, values)[i-1]-v) for i,v in raw.items()),
            'minimum_envelope_clearance_m': min(guard.distances(values).values())}
        output['hardware_read']['within_joint_limits'] = all(
            limits[i][0] <= v <= limits[i][1] for i,v in enumerate(values[:5]))
    c = CartesianController(kin, calibration, guard, {'calibrated_limit_joints': calibrated_limit_joints})
    q = [0,-30,0,30,0,50]
    c.reset(q, 0)
    x0 = kin.forward(q[:5])[0][3]
    timings = []
    blocked = 0
    for i in range(frames):
        body = {'kind': 'arm_cartesian_velocity', 'frame_id': 'base',
                'linear_mps': {'x': .005, 'y': 0., 'z': 0.},
                'angular_rps': {'x': 0., 'y': 0., 'z': 0.}}
        c.command(body, q, i*.05, i*.05+1)
        start = time.monotonic()
        candidate = c.step((i+1)*.05, q)
        timings.append((time.monotonic()-start)*1000)
        if candidate is not None:
            raw = values_to_raw(calibration, candidate)
            q = raw_to_values(calibration, dict(enumerate(raw,1)))
        else:
            blocked += 1
    ordered = sorted(timings)
    output['simulation'] = {'mean_ms': statistics.mean(timings), 'p95_ms': ordered[int(.95*(len(ordered)-1))],
        'max_ms': max(timings), 'blocked_frames': blocked,
        'delta_x_m': kin.forward(q[:5])[0][3] - x0, 'final': c.last}
    assert output['simulation']['delta_x_m'] > .0005, 'simulated velocity failed to move'
    c.step(c.deadline+.01, q)
    assert c.mode == 'idle' and c.last['reason'] == 'watchdog'
    output['watchdog'] = 'passed'
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-dir', default=os.path.join(os.path.dirname(__file__), '../config'))
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--read-port')
    parser.add_argument('--frames', type=int, default=100)
    parser.add_argument('--calibrated-limit-joint', action='append', default=[],
                        help='Use calibrated bounds for this joint, matching the deployed profile')
    args = parser.parse_args()
    print(json.dumps(verify(args.config_dir, args.calibration, args.read_port, args.frames,
                           args.calibrated_limit_joint), indent=2))
