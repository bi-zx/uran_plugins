"""Calibration, geometry, closed-loop control and hardware-write regressions."""
import copy
import io
import importlib.util
import json
import math
import os
import threading
import time

import pytest

from uran_worker_sdk import WorkerServer, now_ns
from uran_so101_plugin.calibration import (JOINT_NAMES, apply_safety_range, empty_calibration,
    is_complete, load_calibration, narrow_safety_range, raw_to_values, joint_limits,
    validate_raw_targets, values_to_raw)
from uran_so101_plugin.kinematics import (SO101Kinematics, rotation_error, _axis_rotation,
    _identity, _mat_mul, _matrix_to_quaternion)
from uran_so101_plugin.collision import CapsuleGuard, segment_distance, box_clearance
from uran_so101_plugin.controller import CartesianController
from uran_so101_plugin.feetech_bus import (FeetechBus, FeetechError, FeetechStatusError,
    FeetechTorqueError, is_voltage_alarm)
from uran_so101_plugin.worker import worker_factory, SimulatedArmBackend, FeetechArmBackend

ROOT = os.path.dirname(os.path.dirname(__file__))
URDF = os.path.join(ROOT, 'config', 'so101_collision.urdf')
RANGE = os.path.join(ROOT, 'config', 'so101_follower_com9.json')
MODEL = os.path.join(ROOT, 'config', 'collision_capsules.json')


def geometry():
    kin = SO101Kinematics(URDF)
    return kin, CapsuleGuard(kin, MODEL, URDF)


def calibration():
    return load_calibration(RANGE)


def velocity(x=0.01, yaw=0.0):
    return {'kind': 'arm_cartesian_velocity', 'frame_id': 'base',
            'linear_mps': {'x': x, 'y': 0.0, 'z': 0.0},
            'angular_rps': {'x': 0.0, 'y': 0.0, 'z': yaw}}


def request(body, sequence=1, controller='test', duration=1.0, operation=None):
    now = now_ns()
    op = operation or 'op-{}'.format(sequence)
    return {'message_type': 'uran.motion.command.v1', 'operation_id': op,
        'payload': {'schema_version': '1.0', 'operation_id': op,
        'target': {'device_id': 'dog', 'component_id': 'arm'}, 'controller': controller,
        'sequence': sequence, 'issued_at_ns': now, 'valid_until_ns': now + int(duration*1e9),
        'payload': body}}


def worker(**config):
    w = worker_factory()
    w.configure(dict({'backend': 'simulation', 'device_id': 'dog', 'component_id': 'arm',
        'calibration_file': RANGE, 'state_reporting': False, 'manual_tick': True}, **config))
    w.activate()
    return w


def test_lerobot_conversion_matches_reference_and_roundtrips(tmp_path):
    original = json.load(open(RANGE))
    original['gripper']['drive_mode'] = 1
    # homing offsets and drive modes must NOT shift/reverse the five arm joints.
    for name in JOINT_NAMES[:5]:
        original[name]['drive_mode'] = 1
        original[name]['homing_offset'] = -1700
    path = tmp_path / 'calibration.json'
    path.write_text(json.dumps(original))
    data = load_calibration(str(path))
    for fraction in (0.0, 0.21, 0.5, 0.9, 1.0):
        raw = {i: round(original[n]['range_min'] + fraction*(original[n]['range_max']-original[n]['range_min']))
               for i, n in enumerate(JOINT_NAMES, 1)}
        values = raw_to_values(data, raw)
        for i, n in enumerate(JOINT_NAMES[:5]):
            mid = (original[n]['range_min'] + original[n]['range_max'])/2
            assert values[i] == pytest.approx((raw[i+1]-mid)*360/4095)
        grip = original['gripper']
        assert values[5] == pytest.approx(100 - (raw[6]-grip['range_min'])*100/(grip['range_max']-grip['range_min']))
        assert values_to_raw(data, values) == list(raw.values())


def test_limits_never_create_calibration_or_change_coordinates():
    with pytest.raises(ValueError):
        apply_safety_range(empty_calibration(), RANGE)
    data = calibration()
    raw = values_to_raw(data, [0]*5+[30])
    values = raw_to_values(data, dict(enumerate(raw, 1)))
    narrow_safety_range(data, 0.8)
    assert raw_to_values(data, dict(enumerate(raw, 1))) == values
    assert values_to_raw(data, values) == raw
    with pytest.raises(ValueError):
        values_to_raw(data, [10000]*5+[50])
    with pytest.raises(ValueError):
        validate_raw_targets(data, [99999]*6)
    data['joints']['wrist_roll']['raw_center'] = float('nan')
    assert not is_complete(data)


def test_spatial_jacobian_against_finite_differences():
    kin, _ = geometry()
    q = [12, -18, 25, 15, 35]
    transform, jac = kin.forward_with_jacobian(q, spatial=True)
    eps = 1e-6
    for i in range(5):
        changed = list(q)
        changed[i] += math.degrees(eps)
        other = kin.forward(changed)
        derivative = [(other[j][3]-transform[j][3])/eps for j in range(3)] + [v/eps for v in rotation_error(other, transform)]
        assert [jac[j][i] for j in range(6)] == pytest.approx(derivative, abs=3e-7)
    old = SO101Kinematics(os.path.join(ROOT, 'config', 'so101_chain.urdf'))
    for row_a, row_b in zip(old.forward(q), transform):
        assert row_a == pytest.approx(row_b, abs=1e-12)


def test_pose_solver_tracks_achievable_orientation_without_overwriting_joint():
    kin, _ = geometry()
    seed = [0, 15, -25, 30, 10]
    goal = [1, 16, -26, 31, 12]
    target = kin.forward(goal)
    result = kin.inverse_pose(target, seed, max_iterations=40)
    assert result['position_error_mm'] < 0.1
    assert result['orientation_error_rad'] < math.radians(0.1)
    assert max(abs(a-b) for a, b in zip(seed, result['joints'])) <= 3.0
    # Large unreachable six-dimensional request remains finite and frame bounded.
    target[0][3] = 5
    result = kin.inverse_pose(target, seed)
    assert result['position_error_mm'] > 1000
    assert all(math.isfinite(v) and abs(v-seed[i]) <= 3 for i, v in enumerate(result['joints']))
    assert math.sqrt(sum(v*v for v in rotation_error(_axis_rotation([0, 1, 0], math.pi), _identity()))) == pytest.approx(math.pi)


def test_segment_and_box_geometry_degenerate_parallel_intersecting():
    assert segment_distance([0,0,0], [0,0,0], [0,3,4], [0,3,4]) == 5
    assert segment_distance([0,0,0], [1,0,0], [0,2,0], [1,2,0]) == 2
    assert segment_distance([-1,0,0], [1,0,0], [0,-1,0], [0,1,0]) == 0
    axes = [[1,0,0], [0,1,0], [0,0,1]]
    a = ([0,0,0], axes, [1,1,1])
    assert box_clearance(a, ([3,0,0], axes, [1,1,1])) == pytest.approx(1)
    assert box_clearance(a, ([1,0,0], axes, [1,1,1])) == pytest.approx(-1)


def test_generated_envelopes_contain_reference_vertices():
    # Source checkout-only provenance check; remote runtime tests omit it when
    # the reference assets aren't present alongside the repositories.
    assets = os.path.abspath(os.path.join(ROOT, '../../../..', 'lerobot-so101-ik/assets'))
    if not os.path.isdir(assets):
        pytest.skip('offline reference STL assets not installed')
    spec = importlib.util.spec_from_file_location('generator', os.path.join(ROOT, 'tools/generate_collision_model.py'))
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    model = json.load(open(MODEL))
    for c in model['capsules']:
        points, digest = generator.vertices(os.path.join(assets, c['mesh']))
        assert digest == c['sha256']
        for p in points:
            assert segment_distance(p, p, c['a'], c['b']) <= c['radius'] + 1e-9
            assert all(abs(p[i]-c['box_center'][i]) <= c['box_half'][i] for i in range(3))


def test_collision_safe_folded_and_intermediate_path():
    _, guard = geometry()
    safe = [0,0,0,0,0,50]
    folded = [0,-60,90,70,0,50]
    assert min(guard.distances(safe).values()) > 0
    assert min(guard.distances(folded).values()) < -0.01
    assert guard.check_path(safe, [0,1,-1,0,0,50])[0]
    assert not guard.check_path(safe, folded)[0]
    # Endpoints alone are insufficient: inject a crossing at the midpoint.
    guard.pairs = [(1, 0)]
    guard.recovery_bounds = None
    guard.distances = lambda q: {(1, 0): -0.01 if 0.2 < q[0] < 0.8 else 0.01}
    assert not guard.check_path(safe, [1,0,0,0,0,50])[0]


def controller():
    kin, guard = geometry()
    c = CartesianController(kin, calibration(), guard, {'velocity_filter_tau_s': 0.0})
    c.reset([0,-30,0,30,0,50], 0)
    return c


def test_velocity_accumulates_sub_tolerance_targets_and_watchdog_holds():
    c = controller()
    measured = list(c.q)
    x0 = c.target[0][3]
    c.command(velocity(0.005), measured, 0, 1)
    for stamp in (0.05, 0.10, 0.15):
        q = c.step(stamp, measured)
        assert q is not None
        assert max(abs(a-b) for a,b in zip(c.q[:5], measured[:5])) <= 3
        measured = q
    assert c.target[0][3] - x0 == pytest.approx(0.00075)
    assert c.kin.forward(measured[:5])[0][3] - x0 > 0.0006
    assert c.step(0.201, measured) == measured
    assert c.mode == 'idle'
    assert c.last['reason'] == 'watchdog'
    assert c.step(0.25, measured) is None


def test_receive_frequency_does_not_integrate_motion():
    slow, fast = controller(), controller()
    qa, qb = list(slow.q), list(fast.q)
    for tick in range(1, 9):
        start = (tick-1)*0.05
        slow.command(velocity(), qa, start, 2)
        for sub in range(5):
            fast.command(velocity(), qb, start + sub*.009, 2)
        qa = slow.step(tick*.05, qa)
        qb = fast.step(tick*.05, qb)
    assert qa == pytest.approx(qb, abs=1e-10)


def test_zero_and_collision_restore_integrator_state():
    c = controller()
    measured = list(c.q)
    c.command(velocity(), measured, 0, 1)
    q0, t0 = list(c.q), copy.deepcopy(c.target)
    c.guard.check_path = lambda *args: (False, {'blocked': True})
    assert c.step(.05, measured) is None
    assert c.q == q0 and c.target == t0
    c.command(velocity(0), measured, .06, 1)
    assert c.mode == 'idle'
    assert c.step(.1, measured) is None


def test_orientation_velocity_uses_base_z_frame():
    c = controller()
    q = list(c.q)
    initial = c.kin.forward(q[:5])
    c.command(velocity(0, .2), q, 0, 1)
    candidate = c.step(.05, q)
    expected = _mat_mul(_axis_rotation([0,0,1], .01), initial)
    assert rotation_error(expected, c.target) == pytest.approx([0,0,0], abs=1e-9)
    assert candidate is not None


class RecordingBackend(SimulatedArmBackend):
    def __init__(self, positions):
        super(RecordingBackend, self).__init__(positions)
        self.calls = []
    def write_positions(self, targets):
        self.calls.append(('write', list(targets)))
        super(RecordingBackend, self).write_positions(targets)
    def set_torque(self, enabled):
        self.calls.append(('torque', enabled))
        super(RecordingBackend, self).set_torque(enabled)


def test_worker_hold_before_torque_async_feedback_and_replay():
    w = worker()
    w.backend = RecordingBackend(w.last_raw)
    try:
        r = request(velocity(-0.01))
        assert w.invoke(r)['status'] == 'accepted'
        measured = list(w.last_raw)
        w.tick(w.controller.previous_time + .05)
        assert w.backend.calls[0] == ('write', measured)
        assert w.backend.calls[1] == ('torque', True)
        assert w.last_raw == measured
        assert w.last_targets != measured
        with pytest.raises(ValueError):
            w.invoke(r)
        w.invoke(request({'kind': 'safety_action', 'action': 'emergency_stop'}, 2))
        assert w.estop and not w.torque_enabled
        with pytest.raises(ValueError):
            w.invoke(request(velocity(), 3))
    finally:
        w.shutdown()


def test_dry_run_and_idle_do_not_write_goals_or_torque():
    w = worker(allow_motion=False)
    w.backend = RecordingBackend(w.last_raw)
    try:
        w.tick()
        w.invoke(request(velocity()))
        w.tick(w.controller.previous_time + .05)
        w.invoke(request({'kind': 'safety_action', 'action': 'emergency_stop'}, 2))
        w.shutdown()
        assert w.backend.calls == []
    finally:
        w.shutdown()


def test_toggle_is_rate_limited_and_uses_measured_position():
    w = worker()
    try:
        w.backend.positions = values_to_raw(w.calibration, [0]*5+[30])
        r = request({'kind': 'gripper_action', 'action': 'toggle'})
        assert w.invoke(r)['status'] == 'accepted'
        assert w.controller.gripper_target == 100
        previous = w.last_values[5]
        w.tick(w.controller.previous_time + .05)
        assert 0 < w.controller.q[5] - previous <= 5.00001
        with pytest.raises(ValueError):
            w.invoke(request({'kind': 'gripper_action', 'action': 'toggle'}, 2, operation='op-1'))
        with pytest.raises(ValueError):
            w.invoke(request({'kind': 'arm_joints', 'joint_names': ['gripper'], 'positions_rad': [0]}, 2))
    finally:
        w.shutdown()


def test_pose_deadline_does_not_claim_arrival():
    w = worker()
    try:
        target = w.kinematics.forward([0,-30,0,30,0])
        body = {'kind': 'arm_pose', 'frame_id': 'base', 'position_m': [target[i][3] for i in range(3)],
                'orientation_xyzw': list(_matrix_to_quaternion(target))}
        assert w.invoke(request(body))['status'] == 'accepted'
        w.tick(w.controller.deadline + .01)
        assert w.operations['op-1'] == 'failed'
        assert not w.torque_enabled
    finally:
        w.shutdown()


def test_backend_probe_failure_closes_serial(monkeypatch):
    import uran_so101_plugin.worker as module
    calls = []
    class Bus(object):
        def __init__(self, *args): pass
        def open(self): pass
        def close(self): calls.append('close')
        def read_positions(self, ids): raise IOError('unplugged')
    monkeypatch.setattr(module, 'FeetechBus', Bus)
    backend = FeetechArmBackend('/dev/test', 1000000, .03)
    with pytest.raises(IOError):
        backend.open()
    assert calls == ['close'] and backend.bus is None


def test_worker_server_lifecycle_without_calibration():
    import io
    frames = [{'kind': 'request', 'request_id': str(i), 'op': op, **fields} for i, (op, fields) in enumerate([
        ('hello', {'protocol': 'uran.worker.jsonl.v1', 'identity': {}}),
        ('configure', {'config': {'backend': 'simulation', 'device_id': 'dog', 'component_id': 'arm', 'state_reporting': False}}),
        ('activate', {}), ('invoke', request({'kind': 'safety_action', 'action': 'stop'})), ('shutdown', {})])]
    source = io.BytesIO(('\n'.join(json.dumps(f) for f in frames)+'\n').encode())
    output = io.BytesIO()
    assert WorkerServer(worker_factory(), source, output).serve() == 0
    assert json.loads(output.getvalue().splitlines()[3])['status'] == 'succeeded'


def test_workspace_outside_pose_does_not_jump_on_zero_velocity():
    c = controller()
    measured = [0,0,0,0,0,50]  # x exceeds the default workspace limit
    c.reset(measured, 0)
    target = copy.deepcopy(c.target)
    c.command(velocity(0, .01), measured, 0, 1)
    c.step(.05, measured)
    assert [row[3] for row in c.target] == [row[3] for row in target]


def test_torque_disable_attempts_every_motor_after_failure():
    from uran_so101_plugin.feetech_bus import FeetechBus, FeetechError
    bus = object.__new__(FeetechBus)
    calls = []
    def write(motor, *args, **kwargs):
        calls.append(motor)
        if motor == 2:
            raise IOError('unplugged')
    bus.write = write
    with pytest.raises(FeetechError):
        bus.set_torque(range(1, 7), False)
    assert calls == list(range(1, 7))


def test_read_failure_latches_worker_and_attempts_torque_disable():
    w = worker()
    w.backend = RecordingBackend(w.last_raw)
    try:
        w.invoke(request(velocity(-.01)))
        w.tick(w.controller.previous_time + .05)
        def fail():
            raise IOError('serial disconnected')
        w.backend.read_positions = fail
        w.tick(w.controller.previous_time + .05)
        assert w.faulted and not w.connected
        assert w.backend.calls[-1] == ('torque', False)
        assert w.operations['op-1'] == 'failed'
        with pytest.raises(ValueError):
            w.invoke(request(velocity(), 2))
    finally:
        w.shutdown()


def test_collision_recovery_does_not_rebase_to_deeper_overlap():
    _, guard = geometry()
    guard.pairs = [(1,0)]
    guard.distances = lambda q: {(1,0): -.01-q[0]*.001}
    q = [0,0,0,0,0,50]
    assert guard.check_path(q, q)[0]
    # Simulate external disturbance: it must not establish a deeper baseline.
    deeper = [1,0,0,0,0,50]
    assert not guard.check_path(deeper, deeper)[0]


def test_filter_rolls_back_when_collision_blocks_and_resets_on_stop():
    kin, guard = geometry()
    c = CartesianController(kin, calibration(), guard, {})
    q = [0,-30,0,30,0,50]
    c.reset(q, 0)
    c.command(velocity(), q, 0, 1)
    c.step(.05, q)
    assert 0 < c.filtered[0] < .01
    before = list(c.filtered)
    c.guard.check_path = lambda *args: (False, {'blocked': True})
    assert c.step(.1, q) is None
    assert c.filtered == before
    c.command(velocity(0), q, .11, 1)
    assert c.filtered == [0.0]*4


def test_thread_lifecycle_stops_and_reactivates():
    w = worker(manual_tick=False)
    try:
        first = w.thread
        w.deactivate()
        assert not first.is_alive() and not w.active
        w.activate()
        assert w.thread is not first and w.thread.is_alive()
    finally:
        w.shutdown()
    assert not w.thread.is_alive()


def test_invalid_start_rejects_without_faulting_or_torque():
    w = worker()
    w.backend.positions = values_to_raw(w.calibration, [0,-110,0,30,0,50])
    w.backend = RecordingBackend(w.backend.positions)
    try:
        with pytest.raises(ValueError, match='shoulder_lift'):
            w.invoke(request(velocity(-.01)))
        assert not w.faulted and w.controller.mode == 'idle'
        assert w.backend.calls == []
    finally:
        w.shutdown()


def test_successful_reactivation_clears_transient_serial_error():
    w = worker()
    try:
        w.last_error = 'Write timeout'
        w.deactivate()
        w.activate()
        assert w.last_error == ''
    finally:
        w.shutdown()


def test_encoder_quantization_preserves_actual_frame_speed_bound():
    c = controller()
    q = list(c.q)
    target = c.kin.forward([5,-15,-10,35,10])
    c.command({'kind': 'arm_pose', 'frame_id': 'base',
               'position_m': [target[i][3] for i in range(3)],
               'orientation_xyzw': list(_matrix_to_quaternion(target))}, q, 0, 5)
    previous = values_to_raw(c.calibration, q)
    for frame in range(1, 8):
        candidate = c.step(frame*.05, q)
        if candidate is None:
            continue
        raw = values_to_raw(c.calibration, candidate)
        assert max(abs(a-b)*360/4095 for a,b in zip(raw[:5],previous[:5])) <= 3.0
        previous = raw
        q = raw_to_values(c.calibration, dict(enumerate(raw,1)))


def test_calibrated_shoulder_range_applies_to_readiness_and_ik():
    kin, guard = geometry()
    data = calibration()
    data['joints']['shoulder_lift'].update(raw_min=769, raw_max=3380, raw_center=2074.5)
    q = [0, -113.63, 90, 70, 0, 50]
    reference = CartesianController(kin, data, guard, {})
    with pytest.raises(ValueError, match='shoulder_lift'):
        reference.validate_start(q)
    expanded = CartesianController(kin, data, guard, {'calibrated_limit_joints': ['shoulder_lift']})
    expanded.validate_start(q)
    assert expanded.limits[1] == pytest.approx((-114.769230769, 114.769230769))
    assert [b for i, b in enumerate(expanded.limits) if i != 1] == [b for i, b in enumerate(reference.limits) if i != 1]
    target = kin.forward([0, -112.63, 90, 70, 0])
    result = kin.inverse_pose(target, q[:5], expanded.limits,
                              calibrated_limit_joints=['shoulder_lift'])
    assert result['joints'][1] < -100
    assert result['cost'] < result['initial_cost']
    assert all(lo <= value <= hi for value, (lo, hi) in zip(result['joints'], expanded.limits))
    assert max(abs(a-b) for a, b in zip(result['joints'], q)) <= 3
    # Crossing the new bounds is still forbidden, including in the IK solver.
    for angle in (-115, 115):
        outside = list(q)
        outside[1] = angle
        with pytest.raises(ValueError):
            expanded.validate_start(outside)
        with pytest.raises(ValueError):
            kin.inverse_pose(target, outside[:5], expanded.limits,
                             calibrated_limit_joints=['shoulder_lift'])
    # Narrower effective calibration bounds continue to take precedence.
    narrow_safety_range(data, 0.8)
    narrowed = CartesianController(kin, data, guard, {'calibrated_limit_joints': ['shoulder_lift']})
    assert narrowed.limits[1][1] < 100
    with pytest.raises(ValueError):
        narrowed.validate_start(q)


@pytest.mark.parametrize('override', ['shoulder_lift', None, ['typo'], ['gripper'], ['shoulder_lift', 'shoulder_lift']])
def test_invalid_calibrated_limit_override_is_rejected(override):
    kin, guard = geometry()
    with pytest.raises(ValueError):
        CartesianController(kin, calibration(), guard, {'calibrated_limit_joints': override})


def test_relaxed_shoulder_still_checks_collision_before_writing():
    w = worker(calibrated_limit_joints=['shoulder_lift'])
    w.backend = RecordingBackend(values_to_raw(w.calibration, [0,-113,90,70,0,50]))
    checks = []
    def blocked(measured, candidate, *args):
        checks.append((measured, candidate))
        return False, {'blocked': True, 'pair': ['lower_arm_link', 'shoulder_link']}
    w.guard.check_path = blocked
    try:
        assert w.invoke(request(velocity(-.005)))['status'] == 'accepted'
        w.tick(w.controller.previous_time + .05)
        assert checks
        assert not w.faulted
        assert w.controller.last['reason'] == 'self_collision'
        assert w.backend.calls == []
    finally:
        w.shutdown()

# Voltage alarm recovery: typed wire errors, stable feedback, no replay.


class AlarmBackend(RecordingBackend):
    def __init__(self, positions):
        super(AlarmBackend, self).__init__(positions)
        self.error = None
        self.reads = 0
        self.torque_error = None

    def read_positions(self):
        self.reads += 1
        if self.error is not None:
            raise self.error
        return super(AlarmBackend, self).read_positions()

    def set_torque(self, enabled):
        super(AlarmBackend, self).set_torque(enabled)
        if self.torque_error:
            raise self.torque_error


def attach(w):
    w.backend = AlarmBackend(w.last_raw)
    return w.backend


def alarm(w, bits=1):
    w.backend.error = FeetechStatusError(6, bits)
    w.tick(10.0)


def sample(w):
    w.tick(w.recovery_next + 0.001)


def recover(w):
    w.backend.error = None
    for _ in range(w.recovery_required):
        sample(w)


@pytest.mark.parametrize('bits,eligible', [(1, True), (5, False), (4, False), (32, False)])
def test_wire_status_error_preserves_motor_and_bitmask(bits, eligible):
    bus = object.__new__(FeetechBus)
    bus.port, bus.timeout = '/fake', .01
    body = bytes([6, 4, bits, 0, 8])
    bus._ser = io.BytesIO(b'\xff\xff' + body + bytes([FeetechBus._checksum(body)]))
    bus._require_open = lambda: None
    with pytest.raises(FeetechStatusError) as error:
        bus._read_status(expected_id=6, min_params=2)
    assert error.value.motor_id == 6
    assert error.value.error_bits == bits
    assert is_voltage_alarm(error.value) == eligible
    assert not is_voltage_alarm(FeetechError(str(error.value)))  # no string matching


def test_idle_voltage_recovers_after_three_spaced_samples_without_writes():
    w = worker()
    b = attach(w)
    try:
        alarm(w)
        assert w.faulted and w.recovery_pending and not w.connected
        b.error = None
        before = b.reads
        w.tick(w.recovery_next - .001)
        assert b.reads == before
        sample(w)
        sample(w)
        assert w.faulted and w.recovery_healthy == 2
        with pytest.raises(ValueError, match='recovery in progress'):
            w.invoke(request(velocity(-.01)))
        sample(w)
        assert w.connected and not w.faulted and not w.recovery_pending
        assert w.last_error == '' and w.recovery_count == 1
        assert w.controller.mode == 'idle' and w.last_targets == []
        assert w.recovery_last_error == 'motor 6 returned error 0x01 (voltage)'
        assert b.calls == []
        assert w.invoke(request(velocity(-.01), 2))['status'] == 'accepted'
    finally:
        w.shutdown()


def test_flapping_voltage_resets_consecutive_sample_counter():
    w = worker()
    b = attach(w)
    try:
        alarm(w)
        b.error = None
        sample(w)
        sample(w)
        b.error = FeetechStatusError(6, 1)
        sample(w)
        assert w.recovery_healthy == 0 and w.recovery_pending
        assert w.voltage_alarm_count == 2
        recover(w)
        assert w.recovery_count == 1
    finally:
        w.shutdown()


def test_active_operation_fails_and_recovery_reseeds_without_replaying():
    w = worker()
    b = attach(w)
    try:
        old = request(velocity(-.01))
        w.invoke(old)
        w.tick(w.controller.previous_time + .05)
        assert w.torque_enabled
        alarm(w)
        assert b.calls[-1] == ('torque', False)
        assert w.operations['op-1'] == 'failed' and w.pending_operation is None
        b.positions = values_to_raw(w.calibration, [0,-30,0,30,0,40])
        writes = list(b.calls)
        recover(w)
        assert w.controller.q == w.last_values
        assert w.controller.mode == 'idle' and not w.torque_enabled
        w.tick(w.recovery_next + .05)
        assert b.calls == writes
        with pytest.raises(ValueError, match='duplicated'):
            w.invoke(old)
        w.invoke(request(velocity(.005), 2))
        measured = list(w.last_raw)
        w.tick(w.controller.previous_time + .05)
        assert b.calls[len(writes)] == ('write', measured)
        assert b.calls[len(writes)+1] == ('torque', True)
    finally:
        w.shutdown()


def test_voltage_during_invoke_uses_recovery_path():
    w = worker()
    b = attach(w)
    try:
        b.error = FeetechStatusError(6, 1)
        with pytest.raises(FeetechStatusError):
            w.invoke(request(velocity()))
        assert w.recovery_pending
        recover(w)
        assert not w.faulted
        assert b.calls == []
    finally:
        w.shutdown()


def test_voltage_at_activation_recovers_without_reactivation():
    w = worker()
    b = attach(w)
    try:
        w.deactivate()
        b.error = FeetechStatusError(6, 1)
        w.activate()
        assert w.active and w.faulted and w.recovery_pending
        recover(w)
        assert w.connected and not w.faulted
        assert b.calls == []
    finally:
        w.shutdown()


@pytest.mark.parametrize('error', [FeetechStatusError(6, 5), FeetechStatusError(6, 32), IOError('unplugged')])
def test_non_voltage_fault_remains_latched(error):
    w = worker()
    b = attach(w)
    try:
        b.error = error
        w.tick(10)
        assert w.faulted and not w.recovery_pending
        b.error = None
        count = b.reads
        for stamp in (11,12,13):
            w.tick(stamp)
        assert b.reads == count and w.faulted
    finally:
        w.shutdown()


def test_more_serious_error_during_recovery_cancels_auto_recovery():
    w = worker()
    b = attach(w)
    try:
        alarm(w)
        b.error = FeetechStatusError(6, 5)
        sample(w)
        assert not w.recovery_pending and w.faulted
    finally:
        w.shutdown()


def test_emergency_stop_cancels_recovery_and_cannot_auto_clear():
    w = worker()
    b = attach(w)
    try:
        alarm(w)
        w.invoke(request({'kind': 'safety_action', 'action': 'emergency_stop'}))
        assert w.estop and not w.recovery_pending
        count = b.reads
        recover(w)
        assert w.estop and w.faulted and b.reads == count
        with pytest.raises(ValueError, match='latched'):
            w.invoke(request(velocity(), 2))
    finally:
        w.shutdown()


def test_partial_torque_enable_alarm_verifies_disabled_registers_before_recovery():
    w = worker()
    b = attach(w)
    try:
        b.torque_error = FeetechStatusError(6, 1)
        w.invoke(request(velocity(-.01)))
        w.tick(w.controller.previous_time + .05)
        assert w.recovery_pending and w.recovery_verify_torque
        assert ('torque', True) in b.calls and b.calls[-1] == ('torque', False)
        b.torque_error = None
        # An unacknowledged torque-disable can leave any motor energized.
        b.torque = True
        sample(w)
        assert b.calls[-1] == ('torque', False) and w.recovery_healthy == 0
        recover(w)
        assert not w.torque_enabled and not w.torque_may_be_enabled and not w.faulted
        assert w.operations['op-1'] == 'failed'
    finally:
        b.torque_error = None
        w.shutdown()


def test_non_voltage_torque_failure_is_not_hidden_by_voltage_recovery():
    w = worker()
    b = attach(w)
    try:
        w.invoke(request(velocity(-.01)))
        w.tick(w.controller.previous_time+.05)
        b.torque_error = FeetechTorqueError([(2, IOError('unplugged'))])
        alarm(w)
        assert not w.recovery_pending and w.faulted
    finally:
        b.torque_error = None
        w.shutdown()


def test_dry_run_recovery_never_writes_torque_or_goals():
    w = worker(allow_motion=False)
    b = attach(w)
    try:
        alarm(w)
        recover(w)
        assert not w.faulted and b.calls == []
    finally:
        w.shutdown()


def test_auto_recovery_can_be_disabled_and_deactivation_stops_retrying():
    w = worker(voltage_auto_recovery=False)
    attach(w)
    try:
        alarm(w)
        recover(w)
        assert w.faulted and not w.recovery_pending
    finally:
        w.shutdown()
    w = worker()
    b = attach(w)
    alarm(w)
    w.deactivate()
    count = b.reads
    recover(w)
    assert b.reads == count and w.faulted
    w.shutdown()


def test_inflight_command_issued_before_recovery_is_not_executed_after_recovery():
    w = worker()
    b = attach(w)
    try:
        delayed = request(velocity(-.01), duration=30)
        alarm(w)
        recover(w)
        assert not w.faulted
        with pytest.raises(ValueError, match='predates voltage recovery'):
            w.invoke(delayed)
        assert w.controller.mode == 'idle' and b.calls == []
        assert w.invoke(request(velocity(-.01), 2))['status'] == 'accepted'
    finally:
        w.shutdown()


def test_controller_trial_isolates_mutable_state_and_collision_baselines():
    c = controller()
    measured = list(c.q)
    c.command(velocity(), measured, 0, 1)
    before = copy.deepcopy(c.__dict__)
    trial = c.fork()
    assert trial.step(.05, measured) is not None
    assert c.q == before['q'] and c.target == before['target']
    assert c.filtered == before['filtered'] and c.guard.recovery_bounds is None
    assert trial.guard.recovery_bounds is not None
    assert trial.kin is c.kin and trial.calibration is c.calibration


@pytest.mark.parametrize('interruption', ['stop', 'emergency_stop', 'zero', 'change', 'cancel', 'deactivate', 'fault', 'feedback'])
def test_slow_frame_does_not_block_commands_or_write_superseded_goal(monkeypatch, interruption):
    w = worker()
    w.backend = RecordingBackend(w.last_raw)
    started, release, completed = threading.Event(), threading.Event(), threading.Event()
    original = CartesianController.step
    failures = []
    def slow_step(c, *args, **kwargs):
        started.set()
        assert release.wait(3)
        return original(c, *args, **kwargs)
    monkeypatch.setattr(CartesianController, 'step', slow_step)
    w.invoke(request(velocity(-.01)))
    frame = threading.Thread(target=lambda: w.tick(w.controller.previous_time + .05))
    def interrupt():
        try:
            if interruption == 'cancel':
                w.cancel('op-1')
            elif interruption == 'deactivate':
                w.deactivate()
            elif interruption == 'fault':
                with w.lock:
                    w._enter_fault(FeetechError('injected'), time.monotonic())
            elif interruption == 'feedback':
                w.backend.positions[0] += 1
                w.invoke(request(velocity(-.01), 2))
            else:
                body = (velocity(0 if interruption == 'zero' else .01) if interruption in ('zero', 'change')
                        else {'kind': 'safety_action', 'action': interruption})
                w.invoke(request(body, 2))
        except Exception as exc:
            failures.append(exc)
        finally:
            completed.set()
    command = threading.Thread(target=interrupt)
    try:
        frame.start()
        assert started.wait(1)
        command.start()
        assert completed.wait(1), 'command waited for IK/collision lock'
        assert not failures
        before = list(w.backend.calls)
        release.set()
        frame.join(2)
        assert not frame.is_alive()
        assert w.backend.calls == before
        assert not any(name == 'write' or value is True for name, value in before)
        assert w.loop_stats['discarded_frames'] == 1
    finally:
        release.set()
        frame.join(3)
        if command.ident is not None:
            command.join(3)
        w.shutdown()


def test_identical_velocity_renewal_allows_inflight_frame_to_commit(monkeypatch):
    w = worker()
    w.backend = RecordingBackend(w.last_raw)
    started, release = threading.Event(), threading.Event()
    original = CartesianController.step
    def slow_step(c, *args, **kwargs):
        started.set()
        assert release.wait(3)
        return original(c, *args, **kwargs)
    monkeypatch.setattr(CartesianController, 'step', slow_step)
    w.invoke(request(velocity(-.01)))
    revision = w.command_revision
    frame = threading.Thread(target=lambda: w.tick(w.controller.previous_time + .05))
    try:
        frame.start()
        assert started.wait(1)
        assert w.invoke(request(velocity(-.01), 2))['status'] == 'accepted'
        deadline = w.controller.deadline
        assert w.command_revision == revision
        release.set()
        frame.join(2)
        assert not frame.is_alive()
        assert w.last_targets and w.last_targets != w.last_raw
        assert w.pending_operation == 'op-2' and w.operations['op-1'] == 'cancelled'
        assert w.controller.deadline == deadline
        assert w.controller.guard is w.guard and w.guard.recovery_bounds is not None
    finally:
        release.set()
        frame.join(3)
        w.shutdown()


def test_renewal_after_watchdog_starts_at_feedback_and_invalidates_old_frame():
    w = worker()
    try:
        w.invoke(request(velocity(-.01)))
        revision = w.command_revision
        w.controller.target[0][3] += .01
        w.controller.deadline = time.monotonic() - .001
        w.invoke(request(velocity(-.01), 2))
        assert w.command_revision > revision
        assert w.controller.target == w.kinematics.forward(w.last_values[:5])
        assert w.controller.filtered == [0.0]*4
    finally:
        w.shutdown()


@pytest.mark.parametrize('candidate, reason', [(None, 'self_collision'),
    ([0.0]*5+[50.0], 'self_collision'), (None, 'watchdog')])
def test_deadline_during_computation_holds_even_when_collision_has_no_candidate(monkeypatch, candidate, reason):
    w = worker()
    w.backend = RecordingBackend(w.last_raw)
    try:
        w.invoke(request(velocity(-.01)))
        w.tick(w.controller.previous_time + .05)
        assert w.torque_enabled
        w.invoke(request(velocity(-.01), 2))
        def expire(c, *args, **kwargs):
            w.controller.deadline = time.monotonic() - 1
            c.last = {'reason': reason, 'collision': {'pair': ['upper_arm', 'base']}}
            return candidate
        monkeypatch.setattr(CartesianController, 'step', expire)
        w.manual_tick = False  # enable real monotonic deadline check, no background thread
        before = len(w.backend.calls)
        w.tick()
        assert w.controller.mode == 'idle'
        assert w.operations['op-2'] == 'failed'
        if reason == 'self_collision':
            assert w.controller.last['blocked_before_timeout']
            assert w.controller.last['collision']['pair'] == ['upper_arm', 'base']
            before_status = copy.deepcopy(w.controller.last)
            monkeypatch.undo()
            w.tick()
            assert w.controller.last == before_status
        assert w.backend.calls[before:] == [('write', w.last_raw)]
        assert not w.faulted
    finally:
        w.shutdown()


def test_expired_command_reports_age_and_lock_wait_without_writing():
    w = worker()
    w.backend = RecordingBackend(w.last_raw)
    try:
        r = request(velocity())
        r['payload']['issued_at_ns'] -= 2000000000
        r['payload']['valid_until_ns'] -= 2000000000
        with pytest.raises(ValueError, match='expired .*age at invoke.*lock wait'):
            w.invoke(r)
        assert w.last_command_timing['remaining_ms'] < 0
        assert w.last_command_timing['age_at_invoke_ms'] >= 2000
        assert w.backend.calls == []
    finally:
        w.shutdown()


def test_optimized_box_clearance_matches_normalized_sat():
    import random
    from uran_so101_plugin.kinematics import _cross, _norm
    rng = random.Random(8127)
    dot = lambda a, b: sum(x*y for x, y in zip(a, b))
    for _ in range(80):
        def box():
            axis = [rng.uniform(-1, 1) for i in range(3)]
            length = _norm(axis)
            rotation = _axis_rotation([v/length for v in axis], rng.uniform(-math.pi, math.pi))
            return ([rng.uniform(-.1, .1) for i in range(3)],
                    [[rotation[i][j] for i in range(3)] for j in range(3)],
                    [rng.uniform(.001, .1) for i in range(3)])
        a, b = box(), box()
        axes = a[1] + b[1] + [_cross(x, y) for x in a[1] for y in b[1]]
        gaps = []
        for axis in axes:
            length = _norm(axis)
            if length < 1e-8:
                continue
            unit = [v/length for v in axis]
            ra = sum(h*abs(dot(unit, v)) for h, v in zip(a[2], a[1]))
            rb = sum(h*abs(dot(unit, v)) for h, v in zip(b[2], b[1]))
            gaps.append(abs(dot([y-x for x, y in zip(a[0], b[0])], unit))-ra-rb)
        assert box_clearance(a, b) == pytest.approx(max(gaps), abs=1e-12)


@pytest.mark.parametrize('bits', [0, 1, 5])
def test_read_confirms_valid_reply_after_transmit_timeout_without_resending(bits):
    from types import SimpleNamespace
    class WriteTimeout(Exception):
        pass
    bus = object.__new__(FeetechBus)
    bus.port, bus.timeout = '/fake', .003
    bus._serial = SimpleNamespace(SerialTimeoutException=WriteTimeout)
    bus.read_write_timeouts = bus.read_replies_after_write_timeout = 0
    body = bytes([6, 4, bits, 0, 8])
    bus._ser = io.BytesIO(b'\xff\xff' + body + bytes([FeetechBus._checksum(body)]))
    bus._require_open = lambda: None
    writes = []
    def timed_out(packet):
        writes.append(packet)
        raise WriteTimeout('Write timeout after os.write accepted all bytes')
    bus._write_packet = timed_out
    if bits:
        with pytest.raises(FeetechStatusError) as exc:
            bus.read(6, 56, 2)
        assert exc.value.error_bits == bits
    else:
        assert bus.read(6, 56, 2) == 2048
    assert len(writes) == 1 and writes[0][4] == 2  # READ only
    assert bus.read_write_timeouts == bus.read_replies_after_write_timeout == 1


@pytest.mark.parametrize('reply_kind', ['absent', 'checksum', 'wrong_id', 'short'])
def test_read_transmit_timeout_without_valid_reply_still_fails(reply_kind):
    from types import SimpleNamespace
    class WriteTimeout(Exception):
        pass
    bus = object.__new__(FeetechBus)
    bus.port, bus.timeout = '/fake', .003
    bus._serial = SimpleNamespace(SerialTimeoutException=WriteTimeout)
    bus.read_write_timeouts = bus.read_replies_after_write_timeout = 0
    body = bytes([5 if reply_kind == 'wrong_id' else 6, 4, 0, 0, 8])
    if reply_kind == 'short':
        body = bytes([6, 3, 0, 0])
    checksum = FeetechBus._checksum(body) ^ (1 if reply_kind == 'checksum' else 0)
    bus._ser = io.BytesIO(b'' if reply_kind == 'absent' else b'\xff\xff' + body + bytes([checksum]))
    bus._require_open = lambda: None
    writes = []
    def timed_out(packet):
        writes.append(packet)
        raise WriteTimeout('Write timeout')
    bus._write_packet = timed_out
    with pytest.raises(FeetechError, match='transmit timeout without valid reply'):
        bus.read(6, 56, 2)
    assert len(writes) == 1
    assert bus.read_write_timeouts == 1 and bus.read_replies_after_write_timeout == 0


@pytest.mark.parametrize('kind', ['torque', 'goal', 'read_disconnected'])
def test_motion_writes_and_non_timeout_read_failures_are_never_replayed(kind):
    from types import SimpleNamespace
    class WriteTimeout(Exception):
        pass
    bus = object.__new__(FeetechBus)
    bus._serial = SimpleNamespace(SerialTimeoutException=WriteTimeout)
    writes, reads = [], []
    def fail(packet):
        writes.append(packet)
        raise IOError('disconnected') if kind == 'read_disconnected' else WriteTimeout('timeout')
    bus._write_packet = fail
    bus._read_status = lambda **kwargs: reads.append(kwargs)
    with pytest.raises((WriteTimeout, IOError)):
        if kind == 'torque':
            bus.write(1, 40, 1, expect_status=True)
        elif kind == 'goal':
            bus.sync_write(42, 2, {1: 2050})
        else:
            bus.read(1, 56, 2)
    assert len(writes) == 1 and reads == []


def test_blocked_operation_reports_collision_before_replacement_and_timeout(monkeypatch):
    w = worker()
    events = []
    w._server = object()
    w.emit_operation = lambda op, seq, status, payload: events.append((op, seq, status, payload))
    monkeypatch.setattr(CapsuleGuard, 'check_path', lambda *args: (False, {'blocked': True, 'pair': ['upper_arm', 'base']}))
    try:
        w.invoke(request(velocity(-.01)))
        assert w.controller.last['mode'] == 'velocity'
        assert w.controller.last['reason'] == 'awaiting_control_frame'
        w.tick(w.controller.previous_time + .05)
        assert events[-1][0:3] == ('op-1', 1, 'running')
        assert events[-1][3]['reason'] == 'self_collision'
        count = len(events)
        w.tick(w.controller.previous_time + .05)
        assert len(events) == count  # No duplicate blockage notification per operation.
        w.invoke(request(velocity(-.01), 2))
        assert events[-1][0:3] == ('op-1', 2, 'cancelled')
        assert events[-1][3]['superseded_by'] == 'op-2'
        assert events[-1][3]['continuous_velocity']
        assert events[-1][3]['controller']['reason'] == 'self_collision'
        w.tick(w.controller.previous_time + .05)
        assert events[-1][0:3] == ('op-2', 1, 'running')
        w.tick(w.controller.deadline + .01)
        assert events[-1][0:3] == ('op-2', 2, 'failed')
        assert events[-1][3]['controller']['blocked_before_timeout']
        assert events[-1][3]['controller']['collision']['pair'] == ['upper_arm', 'base']
    finally:
        w.shutdown()


def test_superseded_reports_original_mode_and_does_not_report_new_target(monkeypatch):
    w = worker()
    events = []
    w._server = object()
    w.emit_operation = lambda op, seq, status, payload: events.append(payload)
    try:
        w.invoke(request(velocity(-.01)))
        w.invoke(request(velocity(0), 2))
        assert events[-1]['controller']['mode'] == 'velocity'
        assert events[-1]['superseded_by'] == 'op-2'
        assert not events[-1]['continuous_velocity']
        assert w.controller.mode == 'idle'
        assert w.controller.last['reason'] == 'zero_velocity'
        w._enter_fault(FeetechError('read request timeout'), time.monotonic())
        with pytest.raises(ValueError, match='latched.*read request timeout'):
            w.invoke(request(velocity(-.01), 3))
    finally:
        w.shutdown()
