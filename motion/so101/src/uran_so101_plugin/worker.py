"""SO101 arm worker.

The worker owns one Feetech bus and exposes the generic URAN arm motion
contract.  It keeps the transport behind a small backend so the complete IK
and safety path can run in simulation on CI and on CyberDog2 with pyserial.
"""
import json
import collections
import copy
import math
import os
import threading
import time
from typing import Any, Dict, Mapping

from uran_interfaces import ContractValidator
from uran_worker_sdk import Worker, WorkerServer, now_ns

from .calibration import (JOINT_NAMES, apply_safety_range, is_complete,
                          load_calibration, raw_to_values, validate_raw_targets,
                          values_to_raw)
from .feetech_bus import FeetechBus, FeetechError, discover_port, is_voltage_alarm
from .kinematics import SO101Kinematics, _matrix_to_quaternion


class ControlFrameCancelled(Exception):
    """A superseded or expired calculation must never produce a servo write."""


class SimulatedArmBackend(object):
    def __init__(self, positions=None):
        self.positions = list(positions or [2048] * 6)
        self.torque = False

    def open(self):
        return

    def close(self):
        return

    def set_torque(self, enabled):
        self.torque = bool(enabled)

    def read_positions(self):
        return {index + 1: value for index, value in enumerate(self.positions)}

    def read_torque_enabled(self):
        return {index: self.torque for index in range(1, 7)}

    def write_positions(self, targets):
        self.positions = [int(value) for value in targets]


class FeetechArmBackend(object):
    def __init__(self, port, baudrate, timeout):
        self.port = str(port or "auto")
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.bus = None
        self.resolved_port = ""
        self.torque = False
        self.lock = threading.RLock()

    def open(self):
        with self.lock:
            if self.bus is not None:
                return
            port = self.port
            if port == "auto":
                port = discover_port(self.baudrate)
            if not port:
                raise FeetechError("SO101 serial port was not found")
            bus = FeetechBus(port, self.baudrate, self.timeout)
            bus.open()
            # Probe the complete chain before accepting a hardware backend.
            try:
                found = bus.read_positions(range(1, 7))
                if len(found) != 6:
                    raise FeetechError("SO101 requires motors 1..6")
            except Exception:
                bus.close()
                raise
            self.bus = bus
            self.resolved_port = str(port)

    def close(self):
        with self.lock:
            if self.bus is not None:
                self.bus.close()
            self.bus = None
            self.torque = False

    def set_torque(self, enabled):
        self.open()
        self.bus.set_torque(range(1, 7), bool(enabled))
        self.torque = bool(enabled)

    def read_positions(self):
        self.open()
        return self.bus.read_positions(range(1, 7))

    def read_torque_enabled(self):
        self.open()
        return {index: bool(self.bus.read(index, 40, 1)) for index in range(1, 7)}

    def write_positions(self, targets):
        self.open()
        self.bus.sync_write(42, 2, {index + 1: int(value) for index, value in enumerate(targets)})


class SO101Worker(Worker):
    plugin_id = "uran-so101-arm"
    plugin_version = "0.2.0"
    capabilities = ("motion.arm.cartesian_velocity", "motion.arm.pose", "motion.gripper.action", "motion.arm.safety")

    def configure(self, config):
        from .calibration import narrow_safety_range
        from .collision import CapsuleGuard
        from .controller import CartesianController
        self.validator = ContractValidator()
        self.expected_device_id = str(config.get("device_id") or self.identity.get("device_id") or "")
        self.expected_component_id = str(config.get("component_id") or self.identity.get("component_id") or "arm")
        self.lock = threading.RLock()
        self.frame_lock = threading.Lock()
        self.command_revision = 0
        self.command_owner = None
        self.last_command_timing = {}
        self.sequences = {}
        self.operations = collections.OrderedDict()
        self.pending_operation = None
        self.pending_event_sequence = 0
        self.pending_block_reported = False
        self.stop_event = threading.Event()
        self.thread = None
        self.telemetry_thread = None
        self.active = False
        self.estop = False
        self.faulted = False
        self.state_reporting = bool(config.get("state_reporting", True))
        self.allow_motion = bool(config.get("allow_motion", True))
        self.voltage_recovery_enabled = config.get('voltage_auto_recovery', True)
        self.recovery_interval = float(config.get('voltage_recovery_interval_s', 0.5))
        self.recovery_required = config.get('voltage_recovery_healthy_samples', 3)
        if (not isinstance(self.voltage_recovery_enabled, bool) or
                not math.isfinite(self.recovery_interval) or not 0.1 <= self.recovery_interval <= 10.0 or
                isinstance(self.recovery_required, bool) or not isinstance(self.recovery_required, int) or
                not 2 <= self.recovery_required <= 100):
            raise ValueError('invalid voltage recovery configuration')
        self.recovery_pending = False
        self.recovery_next = 0.0
        self.recovery_healthy = 0
        self.recovery_attempts = 0
        self.recovery_count = 0
        self.voltage_alarm_count = 0
        self.recovery_last_error = ''
        self.recovery_verify_torque = False
        self.recovered_at_ns = 0
        self.manual_tick = bool(config.get("manual_tick", False))
        if self.manual_tick and config.get('backend') != 'simulation':
            raise ValueError('manual_tick is restricted to simulation')
        if config.get('enable_torque_on_start', False):
            raise ValueError('torque on startup is disabled; first valid command prepares a measured hold')
        if any(config.get('joint_offsets_deg', [])):
            raise ValueError('joint offsets are not supported; use the calibrated range midpoint')
        package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        default_config = os.environ.get("URAN_SO101_CONFIG_DIR", os.path.join(package_root, "config"))
        self.calibration_file = str(config.get("calibration_file") or os.path.join(default_config, "uran_arm_calibration.yaml"))
        self.urdf_path = str(config.get("urdf_path") or os.path.join(default_config, "so101_collision.urdf"))
        self.calibration = load_calibration(self.calibration_file)
        if config.get('safety_range_file'):
            self.calibration = apply_safety_range(self.calibration, config['safety_range_file'])
        if is_complete(self.calibration):
            self.calibration = narrow_safety_range(self.calibration, config.get('safety_range_scale', 1.0))
        self.kinematics = SO101Kinematics(self.urdf_path)
        self.guard = CapsuleGuard(self.kinematics,
            config.get('collision_model') or os.path.join(default_config, 'collision_capsules.json'),
            self.urdf_path, config.get('collision_margin_m', 0.0))
        self.controller = CartesianController(self.kinematics, self.calibration, self.guard, config) if is_complete(self.calibration) else None
        backend_name = str(config.get("backend", "so101_feetech"))
        if backend_name == "simulation":
            positions = values_to_raw(self.calibration, [0.0]*5 + [50.0]) if self.controller else None
            self.backend = SimulatedArmBackend(positions)
        elif backend_name in ("so101_feetech", "feetech"):
            self.backend = FeetechArmBackend(config.get("serial_port", "auto"), config.get("baudrate", 1000000), config.get("serial_timeout_s", 0.03))
        else:
            raise ValueError("unsupported SO101 backend: {}".format(backend_name))
        self.last_raw = [0] * 6
        self.last_values = [0.0] * 6
        self.last_targets = []
        self.last_error = ""
        self.connected = False
        self.torque_enabled = False
        self.torque_may_be_enabled = False
        self.loop_stats = {'ticks': 0, 'overruns': 0, 'last_duration_ms': 0.0,
                           'discarded_frames': 0}

    def activate(self):
        self._register_states()
        with self.lock:
            self.command_revision += 1
            self.stop_event.clear()
            self.active = True
            self.estop = False
            self.faulted = False
            self.recovery_pending = False
            self.recovery_healthy = 0
            try:
                self.backend.open()
                self._refresh()
                self.connected = True
                # A successful reconnect clears a previous transient serial
                # error; persistent readiness/fault state remains explicit.
                self.last_error = ""
                if self.controller:
                    self.controller.reset(self.last_values, time.monotonic())
            except Exception as exc:
                self._enter_fault(exc, time.monotonic())
        self._publish_states()
        if not self.manual_tick:
            self.thread = threading.Thread(target=self._run, name='so101-control', daemon=True)
            self.thread.start()
        if self.state_reporting:
            self.telemetry_thread = threading.Thread(target=self._telemetry, name='so101-state', daemon=True)
            self.telemetry_thread.start()

    def _register_states(self):
        if not self.state_reporting:
            return
        prefix = "device.{}.component.{}.".format(self.expected_device_id, self.expected_component_id)
        for name, kind in (("connection.connected", "bool"), ("torque.enabled", "bool"),
                           ("calibration.complete", "bool"), ("position.raw", "json"),
                           ("position.values", "json"), ("pose.measured", "json"),
                           ("control.status", "json")):
            self.register_state(prefix + name, kind, persistent=False)

    def _refresh(self):
        positions = self.backend.read_positions()
        self.last_raw = [int(positions[index + 1]) for index in range(6)]
        if is_complete(self.calibration):
            self.last_values = raw_to_values(self.calibration, positions)

    def _publish_states(self):
        if not self.state_reporting:
            return
        # Host state requests can block; they never run in the servo loop or
        # while holding the serial/controller lock.
        with self.lock:
            transform = self.kinematics.forward(self.last_values[:5])
            readiness_error = ''
            try:
                if not self.controller:
                    raise ValueError('calibration incomplete')
                self.controller.validate_start(self.last_values)
            except ValueError as exc:
                readiness_error = str(exc)
            states = {"connection.connected": self.connected, "torque.enabled": self.torque_enabled,
                      "calibration.complete": is_complete(self.calibration),
                      "position.raw": list(self.last_raw), "position.values": list(self.last_values),
                      "pose.measured": {'position_m': [transform[i][3] for i in range(3)],
                                        'orientation_xyzw': list(_matrix_to_quaternion(transform))},
                      "control.status": {'estop_latched': self.estop, 'faulted': self.faulted,
                        'dry_run': not self.allow_motion, 'error': self.last_error,
                        'motion_ready': bool(self.connected and not self.estop and not self.faulted and not readiness_error),
                        'readiness_error': readiness_error,
                        'joint_limits_deg': {j['name']: list(bounds) for j, bounds in
                            zip(self.kinematics.active, self.controller.limits)} if self.controller else {},
                        'calibrated_limit_joints': list(self.controller.calibrated_limit_joints) if self.controller else [],
                        'voltage_recovery': {
                            'enabled': self.voltage_recovery_enabled, 'pending': self.recovery_pending,
                            'healthy_samples': self.recovery_healthy, 'required_samples': self.recovery_required,
                            'retry_interval_s': self.recovery_interval, 'attempts': self.recovery_attempts,
                            'recoveries': self.recovery_count, 'alarms': self.voltage_alarm_count,
                            'last_error': self.recovery_last_error},
                        'target_raw': list(self.last_targets), 'loop': dict(self.loop_stats),
                        'last_command_timing': dict(self.last_command_timing),
                        'serial': {
                            'read_write_timeouts': getattr(getattr(self.backend, 'bus', None), 'read_write_timeouts', 0),
                            'read_replies_after_write_timeout': getattr(getattr(self.backend, 'bus', None), 'read_replies_after_write_timeout', 0)},
                        'controller': copy.deepcopy(self.controller.last) if self.controller else {}}}
        prefix = "device.{}.component.{}.".format(self.expected_device_id, self.expected_component_id)
        for suffix, value in states.items():
            if self.stop_event.is_set():
                break
            self.set_state(prefix + suffix, value, persistent=False)

    def _telemetry(self):
        while not self.stop_event.wait(0.5):
            try:
                self._publish_states()
            except Exception as exc:
                with self.lock:
                    self.last_error = 'state reporting: ' + str(exc)

    def _finish(self, status, reason, details=None, controller_state=None):
        operation = self.pending_operation
        if operation is None:
            return
        self.pending_operation = None
        self.operations[operation] = status
        if self._server:
            payload = {'reason': reason, 'controller': controller_state if controller_state is not None else
                       (copy.deepcopy(self.controller.last) if self.controller else {})}
            payload.update(details or {})
            self.emit_operation(operation, self.pending_event_sequence + 1, status, payload)

    def _report_blocked(self):
        if self.pending_operation is None or self.pending_block_reported:
            return
        self.pending_block_reported = True
        self.pending_event_sequence += 1
        if self._server:
            self.emit_operation(self.pending_operation, self.pending_event_sequence, 'running',
                                {'reason': 'self_collision', 'blocked': True,
                                 'controller': copy.deepcopy(self.controller.last)})

    def _hold(self):
        if self.allow_motion and self.torque_enabled and self.connected:
            self.backend.write_positions(self.last_raw)
            self.last_targets = list(self.last_raw)

    def _disable_torque(self):
        if self.allow_motion:
            self.backend.set_torque(False)
        self.torque_enabled = False
        self.torque_may_be_enabled = False

    def _enter_fault(self, exc, stamp):
        self.command_revision += 1
        self.last_error = str(exc)
        self.faulted = True
        self.connected = False
        self.recovery_healthy = 0
        voltage = is_voltage_alarm(exc)
        if voltage:
            self.voltage_alarm_count += 1
        self.recovery_pending = bool(voltage and self.voltage_recovery_enabled and not self.estop)
        self.recovery_last_error = self.last_error
        self.recovery_next = stamp + self.recovery_interval
        self.recovery_verify_torque = self.allow_motion and (self.torque_enabled or self.torque_may_be_enabled)
        if self.controller:
            self.controller.reset(self.last_values, stamp)
            self.controller.last = {'mode': 'idle', 'reason': 'fault', 'error': self.last_error}
        self._finish('failed', self.last_error)
        if self.torque_enabled or self.torque_may_be_enabled:
            try:
                self._disable_torque()
            except Exception as torque_exc:
                self.last_error += '; torque disable failed: ' + str(torque_exc)
                # Do not let a voltage alarm hide an unrelated failure while
                # disabling a partially enabled chain.
                if not is_voltage_alarm(torque_exc):
                    self.recovery_pending = False
                self.recovery_last_error = self.last_error

    def _recover_voltage(self, stamp):
        if not self.recovery_pending or self.estop or stamp < self.recovery_next:
            return
        self.recovery_next = stamp + self.recovery_interval
        self.recovery_attempts += 1
        try:
            self._refresh()  # Every successful sample must include all six motors.
            if self.recovery_verify_torque:
                # An alarm ACK cannot confirm a torque write. Check registers
                # before recovery, including after partial torque enabling.
                torque = self.backend.read_torque_enabled()
                if set(torque) != set(range(1, 7)):
                    raise FeetechError('incomplete torque feedback during recovery')
                if any(torque.values()):
                    self._disable_torque()
                    self.recovery_healthy = 0
                    return
                self.torque_enabled = False
                self.torque_may_be_enabled = False
            self.recovery_healthy += 1
            if self.recovery_healthy < self.recovery_required:
                return
        except Exception as exc:
            self.last_error = str(exc)
            self.recovery_last_error = self.last_error
            self.recovery_healthy = 0
            if is_voltage_alarm(exc):
                self.voltage_alarm_count += 1
            else:
                self.recovery_pending = False
            return
        self.connected = True
        self.faulted = False
        self.last_error = ''
        self.recovery_pending = False
        self.recovery_verify_torque = False
        self.recovery_count += 1
        self.recovered_at_ns = now_ns()
        self.last_targets = []
        if self.controller:
            self.controller.reset(self.last_values, stamp)
            self.controller.last = {'mode': 'idle', 'reason': 'voltage_recovered_waiting_for_command'}

    def _command_io(self, function, *args):
        """Apply the same fault handling to serial I/O in invoke/cancel."""
        try:
            return function(*args)
        except Exception as exc:
            self._enter_fault(exc, time.monotonic())
            raise

    def _write_values(self, values):
        targets = values_to_raw(self.calibration, values)
        validate_raw_targets(self.calibration, targets)
        if self.allow_motion:
            if not self.torque_enabled:
                # Stale Goal_Position must never be active when enabling torque.
                validate_raw_targets(self.calibration, self.last_raw)
                self.backend.write_positions(self.last_raw)
                self.torque_may_be_enabled = True
                self.backend.set_torque(True)
                self.torque_enabled = True
            if not self.manual_tick and time.monotonic() >= self.controller.deadline:
                self._hold()
                self.controller.reset(self.last_values, time.monotonic())
                self._finish('failed', 'deadline_before_goal_write')
                return
            self.backend.write_positions(targets)
        self.last_targets = list(targets)
        # last_values/raw always remain measured feedback, never a target ACK.

    def _frame_checkpoint(self, revision):
        with self.lock:
            if (revision != self.command_revision or not self.active or self.estop
                    or self.faulted or not self.connected or self.stop_event.is_set()):
                raise ControlFrameCancelled('superseded')
            if not self.manual_tick and time.monotonic() >= self.controller.deadline:
                raise ControlFrameCancelled('deadline')

    def tick(self, stamp=None):
        # One computation at a time, but invoke/stop/telemetry only need the
        # short state/serial lock. Never hold it across IK or collision search.
        with self.frame_lock:
            with self.lock:
                if not self.active or self.stop_event.is_set():
                    return
                stamp = time.monotonic() if stamp is None else stamp
                if self.faulted:
                    self._recover_voltage(stamp)
                    return
                if not self.connected:
                    return
                try:
                    self._refresh()
                except Exception as exc:
                    self._enter_fault(exc, stamp)
                    return
                if not self.controller or self.estop:
                    return
                frame = self.controller.fork()
                previous_mode = frame.mode
                revision = self.command_revision
                measured, raw = list(self.last_values), list(self.last_raw)
            cancelled, failure, values = False, None, None
            try:
                values = frame.step(stamp, measured, lambda: self._frame_checkpoint(revision))
            except ControlFrameCancelled:
                cancelled = True
            except Exception as exc:
                failure = exc
            with self.lock:
                if (revision != self.command_revision or not self.active or self.estop
                        or self.faulted or not self.connected or self.stop_event.is_set()):
                    self.loop_stats['discarded_frames'] += 1
                    return
                try:
                    if failure is not None:
                        raise failure
                    # Check even when collision returned no candidate. An old
                    # outstanding servo goal must not outlive a blocked frame.
                    hold_frame = (values is not None and frame.mode == 'idle' and
                                  frame.last.get('reason') in ('watchdog', 'deadline', 'feedback_tracking_error'))
                    if (previous_mode != 'idle' and not self.manual_tick
                            and not hold_frame
                            and time.monotonic() >= self.controller.deadline):
                        self._refresh()
                        previous = frame.last if frame.last.get('reason') == 'self_collision' else self.controller.last
                        blocked = previous.get('reason') == 'self_collision'
                        collision = copy.deepcopy(previous.get('collision'))
                        self.controller.reset(self.last_values, time.monotonic())
                        self.controller.last = {'mode': 'idle', 'reason': 'deadline_during_control_frame',
                                                'blocked_before_timeout': blocked}
                        if blocked:
                            self.controller.last['collision'] = collision
                        self._hold()
                        self._finish('failed', 'deadline_during_control_frame')
                        return
                    if cancelled or raw != self.last_raw:
                        # A command may have read newer feedback while solving.
                        # Recompute from it instead of committing a stale path.
                        self.loop_stats['discarded_frames'] += 1
                        return
                    # Identical live velocity updates renew the lease without
                    # invalidating a frame or integrating any extra movement.
                    frame.deadline = self.controller.deadline
                    self.controller = frame
                    self.guard = frame.guard
                    if values is not None:
                        if frame.last.get('reason') in ('watchdog', 'deadline', 'feedback_tracking_error'):
                            self._hold()
                        elif previous_mode != 'idle':
                            self._write_values(values)
                    terminal = frame.last.get('terminal')
                    if terminal:
                        self._finish(terminal, frame.last.get('reason', 'measured_target_reached'))
                    elif frame.last.get('reason') == 'self_collision':
                        self._report_blocked()
                except Exception as exc:
                    self._enter_fault(exc, time.monotonic())

    def _run(self):
        period = 1.0 / (self.controller.hz if self.controller else 5.0)
        next_tick = time.monotonic() + period
        while not self.stop_event.wait(max(0.0, next_tick - time.monotonic())):
            start = time.monotonic()
            self.tick(start)
            duration = time.monotonic() - start
            with self.lock:
                self.loop_stats['ticks'] += 1
                self.loop_stats['last_duration_ms'] = duration * 1000
                if duration > period:
                    self.loop_stats['overruns'] += 1
            # Never catch up by sending a burst of frames after an overrun.
            next_tick = max(next_tick + period, time.monotonic() + 0.001)

    def invoke(self, request):
        entered_at = time.monotonic()
        entered_ns = now_ns()
        if request.get("message_type") != "uran.motion.command.v1":
            raise ValueError("unsupported message_type")
        document = dict(request.get("payload") or {})
        self.validator.validate("motion/command.schema.json", document)
        before_lock = time.monotonic()
        with self.lock:
            received_ns = now_ns()
            self.last_command_timing = {
                'operation_id': str(request['operation_id']), 'invoke_at_ns': entered_ns,
                'issued_at_ns': int(document['issued_at_ns']),
                'valid_until_ns': int(document['valid_until_ns']),
                'age_at_invoke_ms': (entered_ns - int(document['issued_at_ns'])) / 1e6,
                'lock_wait_ms': (time.monotonic() - before_lock) * 1000,
                'remaining_ms': (int(document['valid_until_ns']) - received_ns) / 1e6}
            target = document["target"]
            if target.get("device_id") != self.expected_device_id or target.get("component_id") != self.expected_component_id:
                raise ValueError("motion command targets another component")
            if not self.active:
                raise ValueError('arm is inactive')
            remaining = (int(document['valid_until_ns']) - received_ns) / 1e9
            if remaining <= 0:
                raise ValueError('motion command is expired (late {:.1f} ms, age at invoke {:.1f} ms, lock wait {:.1f} ms)'.format(
                    -remaining * 1000, self.last_command_timing['age_at_invoke_ms'],
                    self.last_command_timing['lock_wait_ms']))
            operation = str(request['operation_id'])
            if operation != document['operation_id']:
                raise ValueError('operation id mismatch')
            if operation in self.operations:
                raise ValueError('operation is duplicated')
            key = document['controller']
            sequence = int(document['sequence'])
            if sequence <= self.sequences.get(key, -1):
                raise ValueError("motion command sequence is stale or duplicated")
            body = dict(document['payload'])
            stamp = time.monotonic()
            if body['kind'] == 'safety_action':
                action = body['action']
                if action not in ('stop', 'emergency_stop'):
                    raise ValueError('unsupported arm safety action')
                self.command_revision += 1
                self._finish('cancelled', action)
                if self.controller:
                    self.controller.reset(self.last_values, stamp)
                if action == 'emergency_stop':
                    self.estop = True  # explicit lifecycle reactivate is required
                    self.recovery_pending = False
                    self._command_io(self._disable_torque)
                elif self.connected:
                    self._command_io(self._refresh)
                    self._command_io(self._hold)
                status = 'succeeded'
            else:
                if self.estop or self.faulted:
                    if self.recovery_pending:
                        raise ValueError('voltage alarm recovery in progress; send a new command after recovery')
                    raise ValueError('arm is latched; repair fault and reactivate component: ' +
                                     (self.last_error or 'emergency stop'))
                if not self.connected or not self.controller:
                    raise ValueError('arm is disconnected or calibration incomplete')
                if int(document['issued_at_ns']) < self.recovered_at_ns:
                    raise ValueError('command predates voltage recovery; send a new command')
                self._command_io(self._refresh)
                stamp = time.monotonic()
                remaining = (int(document['valid_until_ns']) - now_ns()) / 1e9
                if remaining <= 0:
                    raise ValueError('motion command is expired after feedback read')
                renewal = (self.controller.mode == 'velocity' and stamp < self.controller.deadline
                           and body['kind'] == 'arm_cartesian_velocity'
                           and body == self.controller.body and key == self.command_owner)
                previous_state = copy.deepcopy(self.controller.last)
                previous_state['mode'] = self.controller.mode
                self.controller.command(body, self.last_values, stamp, stamp + remaining)
                if not renewal:
                    self.command_revision += 1
                self.command_owner = key
                self._finish('cancelled', 'superseded',
                             {'superseded_by': operation, 'continuous_velocity': renewal}, previous_state)
                if self.controller.mode == 'idle':
                    self._command_io(self._hold)
                    status = 'succeeded'
                else:
                    self.pending_operation = operation
                    self.pending_event_sequence = 0
                    self.pending_block_reported = False
                    status = 'accepted'
            self.sequences[key] = sequence
            self.operations[operation] = status
            while len(self.operations) > 2048:
                self.operations.popitem(last=False)
            self.last_command_timing['invoke_duration_ms'] = (time.monotonic() - entered_at) * 1000
            return {'status': status, 'operation_id': operation,
                    'payload': {'accepted': True, 'dry_run': not self.allow_motion}}

    def cancel(self, operation_id):
        with self.lock:
            if operation_id == self.pending_operation:
                self.command_revision += 1
                self._command_io(self._refresh)
                self.controller.reset(self.last_values, time.monotonic())
                self._command_io(self._hold)
                self._finish('cancelled', 'cancel_requested')
            return {'status': 'cancelled', 'operation_id': operation_id}

    def deactivate(self):
        self.stop_event.set()
        # Invalidate in-flight computations before waiting for their threads.
        with self.lock:
            self.active = False
            self.command_revision += 1
            self._finish('cancelled', 'deactivated')
            if self.torque_enabled or self.torque_may_be_enabled:
                self._disable_torque()
        # Never join while holding the lock used by the control/telemetry thread.
        for thread in (self.thread, self.telemetry_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=6.0)
                if thread.is_alive():
                    raise RuntimeError('SO101 thread did not stop')

    def shutdown(self):
        if hasattr(self, 'stop_event'):
            self.deactivate()
        if hasattr(self, 'backend'):
            self.backend.close()
        self.connected = False


def worker_factory():
    return SO101Worker()


def main():
    return WorkerServer(worker_factory()).serve()
