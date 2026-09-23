import json
import math
import time
from typing import Any, Dict, Optional

from .waypoint_action_runner import WaypointActionRunner
from .waypoint_dispatcher import WaypointDispatcher
from ..task_models import MissionTask, TaskRuntime, TERMINAL_TASK_STAGES
from ..task_parser import parse_task_definition
from ..geo_utils import EARTH_RADIUS_M
from uran_localization_plugin import GpsVoYawAligner, OutdoorPoseAligner
from ..outdoor import (
    GpsVoGate,
    GoalResolution,
    OutdoorMissionPlan,
    OutdoorGoalResolver,
    OutdoorStartAligner,
    is_outdoor_task_payload,
    parse_outdoor_task_definition,
)


class MissionManager:
    def __init__(
        self,
        *,
        logger,
        backend,
        camera_capture,
        projector,
        closed_loop_manager,
        pose_registry,
        control_mode_getter,
        controller_getter,
        battery_level_getter,
        now_ns_getter,
        position_getter=None,
        stamp_getter,
        publish_uplink,
        write_state,
        write_motion_control_lock,
        publish_media_control,
        set_map_pose_reporting_enabled,
        move_gateway=None,
        straight_drive_controller=None,
        extra_status_getter=None,
        mission_defaults,
        outdoor_goal_resolver_cfg: Optional[Dict[str, Any]] = None,
        outdoor_pose_aligner_cfg: Optional[Dict[str, Any]] = None,
        gps_vo_yaw_aligner_cfg: Optional[Dict[str, Any]] = None,
        outdoor_start_alignment_cfg: Optional[Dict[str, Any]] = None,
        progress_report_interval_s: float,
        require_auto_mode: bool,
        media_actions_cfg: Optional[Dict[str, Any]] = None,
    ):
        self._logger = logger
        self._backend = backend
        self._projector = projector
        self._closed_loop_manager = closed_loop_manager
        self._pose_registry = pose_registry
        self._control_mode_getter = control_mode_getter
        self._controller_getter = controller_getter
        self._battery_level_getter = battery_level_getter
        self._now_ns_getter = now_ns_getter
        self._position_getter = position_getter or self._pose_registry.effective_position
        self._publish_uplink = publish_uplink
        self._write_state = write_state
        self._write_motion_control_lock = write_motion_control_lock
        self._publish_media_control = publish_media_control
        self._set_map_pose_reporting_enabled = set_map_pose_reporting_enabled
        self._move_gateway = move_gateway
        self._straight_drive_controller = straight_drive_controller
        self._extra_status_getter = extra_status_getter
        self._mission_defaults = dict(mission_defaults or {})
        self._outdoor_goal_resolver = OutdoorGoalResolver(outdoor_goal_resolver_cfg)
        self._outdoor_start_aligner = OutdoorStartAligner(outdoor_start_alignment_cfg)
        self._outdoor_pose_aligner_cfg = dict(outdoor_pose_aligner_cfg or {})
        self._outdoor_pose_aligner = OutdoorPoseAligner(self._outdoor_pose_aligner_cfg)
        self._gps_vo_yaw_aligner = GpsVoYawAligner(gps_vo_yaw_aligner_cfg)
        self._progress_report_interval_s = float(progress_report_interval_s)
        self._require_auto_mode = bool(require_auto_mode)
        self._media_actions_cfg = dict(media_actions_cfg or {})

        self._dispatcher = WaypointDispatcher(
            logger=logger,
            backend=backend,
            projector=projector,
            stamp_getter=stamp_getter,
        )
        self._action_runner = WaypointActionRunner(
            camera_capture=camera_capture,
            backend_snapshot_getter=self._dispatcher.backend_snapshot,
            battery_level_getter=self._battery_level_getter,
            position_getter=self._position_getter,
            gps_monitor_state_getter=self._closed_loop_manager.gps_state_snapshot,
            now_ns_getter=self._now_ns_getter,
            publish_event=self._publish_task_action_event,
            set_runtime_event=self._set_runtime_event,
        )

        self._task: Optional[MissionTask] = None
        self._outdoor_mission: Optional[OutdoorMissionPlan] = None
        self._runtime = TaskRuntime()
        self._active_waypoint_index: Optional[int] = None
        self._active_outdoor_point_index: Optional[int] = None
        self._active_outdoor_goal_resolution: Optional[GoalResolution] = None
        self._active_outdoor_candidate_index: Optional[int] = None
        self._active_local_goal_kind = ''
        self._pending_first_inspection_index: Optional[int] = None
        self._pending_home_calibration_index: Optional[int] = None
        self._active_home_calibration_index: Optional[int] = None
        self._hover_until: Optional[float] = None
        self._last_periodic_report_ts = 0.0
        self._recording_active = False
        self._outdoor_calibration_records = []
        self._outdoor_goal_records = []
        self._outdoor_gps_vo_gate = GpsVoGate()
        self._outdoor_start_alignment: Dict[str, Any] = {}
        self._motion_control_locked = False
        self._straight_drive_task: Optional[Dict[str, Any]] = None
        self._initial_yaw_calibration_active = False
        self._paused_local_context: Dict[str, Any] = {}

    @property
    def task(self) -> Optional[MissionTask]:
        return self._task

    @property
    def runtime(self) -> TaskRuntime:
        return self._runtime

    def has_active_task(self) -> bool:
        if self._task is None and self._outdoor_mission is None and self._straight_drive_task is None:
            return False
        return self._runtime.stage not in TERMINAL_TASK_STAGES

    def handle_task_command(
        self,
        *,
        action: str,
        task_id: str = '',
        task_type: str = '',
        task_params_json: str = '',
    ):
        normalized_action = str(action or '').strip().lower()
        if normalized_action == 'start':
            self._handle_start(
                task_id=task_id,
                task_type=task_type,
                task_params_json=task_params_json,
            )
            return
        if normalized_action == 'pause':
            self.pause_current_operation(reason='paused_by_command')
            return
        if normalized_action == 'resume':
            self.resume_current_operation()
            return
        if normalized_action == 'stop':
            self.stop_current_operation(reason='stopped_by_command')
            return
        if normalized_action == 'update_params':
            self._logger.warning('update_params is reserved but not implemented yet')
            self._set_runtime_event('update_params_ignored', publish_now=True)
            return
        self._logger.warning(f'unknown task action: {action}')

    def pause_current_operation(self, *, reason: str = 'paused_by_command') -> Dict[str, Any]:
        if self.has_active_task():
            self._handle_pause(reason=reason)
            return {'accepted': True, 'message': 'task paused'}
        return {'accepted': False, 'message': 'no active task'}

    def resume_current_operation(self) -> Dict[str, Any]:
        if self.has_active_task() and self._runtime.stage == 'paused':
            self._handle_resume()
            return {'accepted': True, 'message': 'task resumed'}

        return {'accepted': False, 'message': 'no paused task'}

    def stop_current_operation(self, *, reason: str = 'stopped_by_command') -> Dict[str, Any]:
        if self.has_active_task():
            self._handle_stop(reason=reason)
            return {'accepted': True, 'message': 'task stopped'}

        return {'accepted': False, 'message': 'no active task'}

    def tick(self, *, monotonic_s: Optional[float] = None):
        monotonic_s = time.monotonic() if monotonic_s is None else float(monotonic_s)

        if (
            self._backend is not None and
            self.has_active_task() and
            not self._uses_local_velocity_navigation()
        ):
            self._backend.refresh_status()

        backend_event = (
            self._backend.take_terminal_event()
            if self._backend is not None and not self._uses_local_velocity_navigation()
            else None
        )

        if self.has_active_task():
            self._tick_task(monotonic_s=monotonic_s, backend_event=backend_event)
            return

    def tick_control(self, *, monotonic_s: Optional[float] = None) -> Dict[str, Any]:
        """Run only the high-frequency local velocity loop.

        Task parsing, waypoint dispatch, media actions, and progress reporting
        stay in ``tick``.  This prevents the 20 Hz control timer from running
        the complete mission state machine a second time.
        """
        monotonic_s = time.monotonic() if monotonic_s is None else float(monotonic_s)
        controller = self._straight_drive_controller
        if controller is None or not controller.is_active():
            return {'state': 'idle', 'active': False}
        if self._runtime.stage == 'paused' or self._runtime.stage in TERMINAL_TASK_STAGES:
            controller.hard_stop(reason='task_not_executable', monotonic_s=monotonic_s)
            return {'state': 'idle', 'active': False}

        result = controller.tick(monotonic_s=monotonic_s)
        if self._straight_drive_task is not None:
            self._handle_straight_drive_control_result(
                result=result,
                monotonic_s=monotonic_s,
            )
        elif self._initial_yaw_calibration_active:
            self._handle_initial_yaw_control_result(
                result=result,
                monotonic_s=monotonic_s,
            )
        elif self._active_home_calibration_index is not None:
            self._handle_home_calibration_control_result(
                result=result,
                monotonic_s=monotonic_s,
            )
        elif self._active_outdoor_point_index is not None:
            self._handle_outdoor_control_result(
                result=result,
                monotonic_s=monotonic_s,
            )
        elif self._active_waypoint_index is not None:
            self._handle_waypoint_control_result(
                result=result,
                monotonic_s=monotonic_s,
            )
        else:
            controller.hard_stop(reason='local_goal_context_missing', monotonic_s=monotonic_s)
        return result

    def handle_control_state_change(self) -> None:
        """Stop and pause an active task immediately after leaving auto mode."""
        if not self.has_active_task() or self._runtime.stage == 'paused':
            return
        if not self._require_auto_mode or self._control_mode_getter() == 'auto':
            return
        if self._straight_drive_controller is not None:
            self._straight_drive_controller.hard_stop(reason='control_mode_changed')
        self._pause_with_error(
            code='E_TASK_CONFLICT',
            description='current control_mode is not auto',
            suggested_action='switch_to_auto_mode',
        )

    def build_status_payload(self) -> Dict[str, Any]:
        payload = self._runtime.to_status_dict(
            task=self._task,
            position=self._position_getter(),
            battery_level=self._battery_level_getter(),
            control_mode=self._control_mode_getter(),
            controller=self._controller_getter(),
            backend_state=self._dispatcher.backend_snapshot(),
        )
        payload['timestamp_ns'] = self._now_ns_getter()
        payload['geo_reference'] = self._projector.reference_summary()
        payload['gps_monitor'] = self._closed_loop_manager.gps_state_snapshot()
        payload['closed_loop'] = self._closed_loop_manager.state_snapshot()
        payload['recording_active'] = self._recording_active
        payload['map_pose'] = self._pose_registry.map_pose_dict()
        payload['visual_pose'] = self._pose_registry.visual_pose_dict()
        payload['outdoor_navigation'] = self._outdoor_status_snapshot()
        payload['straight_drive'] = self._straight_drive_status_snapshot()
        if self._extra_status_getter is not None:
            payload.update(dict(self._extra_status_getter() or {}))
        return payload

    def outdoor_pose_alignment_state(self) -> Dict[str, Any]:
        return self._outdoor_pose_aligner.state_snapshot()

    def local_navigation_pose(self) -> Dict[str, Any]:
        map_pose = self._pose_registry.latest_map_pose()
        if map_pose:
            pose = dict(map_pose)
            pose['source'] = 'map_pose'
            return pose

        gps_pose = self._gps_projected_local_pose()
        if gps_pose:
            return gps_pose
        return {}

    def _gps_projected_local_pose(self) -> Dict[str, Any]:
        gps = self._pose_registry.gps_position()
        if not gps or 'lat' not in gps or 'lon' not in gps:
            return {}

        mission = self._outdoor_mission
        if mission is not None and int(gps.get('fix_type', 0) or 0) < int(mission.min_gps_fix_type):
            return {}

        origin = self._active_projection_origin()
        if not origin or 'lat' not in origin or 'lon' not in origin:
            return {}

        try:
            ref_lat = float(origin['lat'])
            ref_lon = float(origin['lon'])
            lat = float(gps['lat'])
            lon = float(gps['lon'])
        except Exception:
            return {}

        ref_lat_rad = math.radians(ref_lat)
        x = math.radians(lon - ref_lon) * EARTH_RADIUS_M * math.cos(ref_lat_rad)
        y = math.radians(lat - ref_lat) * EARTH_RADIUS_M

        visual_pose = self._pose_registry.latest_visual_pose() or {}
        yaw_deg = visual_pose.get('yaw_deg')
        yaw_offset_deg = self._gps_vo_yaw_aligner.yaw_offset_deg
        if yaw_deg not in (None, '') and yaw_offset_deg is not None:
            yaw_deg = float(yaw_deg) + float(yaw_offset_deg)
        if yaw_deg in (None, ''):
            yaw_deg = self._active_initial_heading_deg()
        if yaw_deg in (None, ''):
            yaw_deg = 0.0

        return {
            'frame_id': 'mission_planner_local_xz',
            'timestamp_ns': int(gps.get('timestamp_ns', 0) or self._now_ns_getter()),
            'x': float(x),
            'y': float(y),
            'z': float(gps.get('alt', 0.0) or 0.0),
            'yaw_deg': float(yaw_deg),
            'source': 'gps_projected_with_local_yaw',
            'gps_fix_type': int(gps.get('fix_type', 0) or 0),
            'gps_num_sv': int(gps.get('num_sv', 0) or 0),
        }

    def _active_projection_origin(self) -> Dict[str, Any]:
        mission = self._outdoor_mission
        if mission is not None:
            origin = dict(mission.projection_origin or {})
            if origin:
                return origin
            coordinate_system = dict(mission.coordinate_system or {})
            local = coordinate_system.get('local') if isinstance(coordinate_system.get('local'), dict) else {}
            if isinstance(local.get('projection_origin'), dict):
                return dict(local['projection_origin'])
        return {}

    def _active_initial_heading_deg(self) -> Optional[float]:
        mission = self._outdoor_mission
        if mission is None or not mission.execution_points:
            return None
        yaw = mission.execution_points[0].yaw_deg
        if yaw not in (None, ''):
            return float(yaw)
        if len(mission.execution_points) >= 2:
            first = mission.execution_points[0]
            second = mission.execution_points[1]
            dx = float(second.x) - float(first.x)
            dy = float(second.y) - float(first.y)
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                return math.degrees(math.atan2(dy, dx))
        return None

    def _update_gps_vo_yaw_alignment(self):
        if self._outdoor_mission is None:
            return
        self._gps_vo_yaw_aligner.update(
            gps_position=self._pose_registry.gps_position(),
            visual_pose=self._pose_registry.latest_visual_pose(),
            timestamp_ns=self._now_ns_getter(),
        )

    def _should_run_initial_yaw_calibration(self) -> bool:
        if self._outdoor_mission is None:
            return False
        if not self._gps_vo_yaw_aligner.enabled:
            return False
        if self._gps_vo_yaw_aligner.yaw_offset_deg is not None:
            return False
        return True

    def _outdoor_status_snapshot(self) -> Dict[str, Any]:
        if self._outdoor_mission is None:
            return {'active': False}

        current_point = None
        if 0 <= self._runtime.current_waypoint_index < len(self._outdoor_mission.execution_points):
            current_point = self._outdoor_mission.execution_points[
                self._runtime.current_waypoint_index
            ].to_dict()

        active_goal = None
        if (
            self._active_outdoor_point_index is not None and
            0 <= self._active_outdoor_point_index < len(self._outdoor_mission.execution_points)
        ):
            active_goal = self._outdoor_mission.execution_points[
                self._active_outdoor_point_index
            ].to_dict()

        active_candidate = None
        if (
            self._active_outdoor_goal_resolution is not None and
            self._active_outdoor_candidate_index is not None
        ):
            candidate = self._active_outdoor_goal_resolution.candidate(
                self._active_outdoor_candidate_index
            )
            active_candidate = candidate.to_dict() if candidate is not None else None

        return {
            'active': self._runtime.stage not in TERMINAL_TASK_STAGES,
            'mission': self._outdoor_mission.to_status_dict(),
            'current_point': current_point,
            'active_goal': active_goal,
            'active_candidate': active_candidate,
            'navigation_mode': 'local_lidar_velocity',
            'active_local_goal_kind': self._active_local_goal_kind,
            'initial_yaw_calibration_active': self._initial_yaw_calibration_active,
            'pending_first_inspection_index': self._pending_first_inspection_index,
            'pending_home_calibration_index': self._pending_home_calibration_index,
            'active_home_calibration_index': self._active_home_calibration_index,
            'local_navigation_pose': self.local_navigation_pose(),
            'local_nav_yaw_offset_deg': self._gps_vo_yaw_aligner.yaw_offset_deg,
            'gps_vo_yaw_alignment': self._gps_vo_yaw_aligner.state_snapshot(),
            'active_resolution': (
                self._active_outdoor_goal_resolution.to_dict()
                if self._active_outdoor_goal_resolution is not None else None
            ),
            'goal_resolver': self._outdoor_goal_resolver.config_snapshot(),
            'start_alignment': dict(self._outdoor_start_alignment),
            'start_aligner': self._outdoor_start_aligner.config_snapshot(),
            'goal_records': list(self._outdoor_goal_records[-20:]),
            'calibration_records': list(self._outdoor_calibration_records[-20:]),
            'gps_vo_gate': self._outdoor_gps_vo_gate.state_snapshot(),
            'pose_alignment': self._outdoor_pose_aligner.state_snapshot(),
        }

    def _straight_drive_status_snapshot(self) -> Dict[str, Any]:
        if self._straight_drive_controller is None:
            return {'active': False, 'available': False}
        snapshot = dict(self._straight_drive_controller.snapshot())
        snapshot['available'] = True
        snapshot['task'] = dict(self._straight_drive_task or {})
        return snapshot

    def status_json(self, *, task_id: str = '') -> str:
        if task_id and task_id != self._runtime.task_id:
            return json.dumps(
                {
                    'task_id': task_id,
                    'stage': 'not_found',
                    'status': 'unknown',
                },
                ensure_ascii=False,
            )
        return json.dumps(self.build_status_payload(), ensure_ascii=False)

    def _uses_local_velocity_navigation(self) -> bool:
        return self._straight_drive_task is not None or self._outdoor_mission is not None or self._task is not None

    def destroy(self):
        self._stop_straight_drive_if_needed(reason='autotask_destroyed')
        self._release_motion_control_lock(reason='autotask_destroyed')
        self._stop_recording_if_needed()
        self._set_map_pose_reporting_enabled(False)

    def _tick_task(self, *, monotonic_s: float, backend_event):
        if self._straight_drive_task is not None:
            if self._require_auto_mode and self._control_mode_getter() != 'auto':
                self._pause_with_error(
                    code='E_TASK_CONFLICT',
                    description='current control_mode is not auto',
                    suggested_action='switch_to_auto_mode',
                )
                return
            self._publish_periodic_progress(monotonic_s=monotonic_s)
            return

        if self._outdoor_mission is not None:
            self._tick_outdoor_task(monotonic_s=monotonic_s, backend_event=backend_event)
            return

        if self._require_auto_mode and self._control_mode_getter() != 'auto':
            self._pause_with_error(
                code='E_TASK_CONFLICT',
                description='current control_mode is not auto',
                suggested_action='switch_to_auto_mode',
            )
            return

        if (
            self._task is not None and
            self._task.abort_on_low_battery and
            self._battery_level_getter() is not None and
            self._battery_level_getter() <= self._task.low_battery_threshold
        ):
            self._abort_task(
                code='E_LOW_BATTERY',
                description=(
                    f'battery level {self._battery_level_getter():.1f} is below threshold '
                    f'{self._task.low_battery_threshold:.1f}'
                ),
                suggested_action=self._task.abort_action or 'stop',
            )
            return

        if self._runtime.stage == 'executing':
            violation = self._closed_loop_manager.evaluate(
                projector=self._projector,
                pose_registry=self._pose_registry,
                now_ns=self._now_ns_getter(),
                monotonic_s=monotonic_s,
            )
            if violation is not None:
                if violation.get('action') == 'abort':
                    self._abort_task(
                        code=violation['code'],
                        description=violation['description'],
                        suggested_action=violation['suggested_action'],
                    )
                else:
                    self._pause_with_error(
                        code=violation['code'],
                        description=violation['description'],
                        suggested_action=violation['suggested_action'],
                    )
                return

        if backend_event is not None:
            self._handle_backend_event(backend_event)

        if self._runtime.stage == 'paused':
            self._publish_periodic_progress(monotonic_s=monotonic_s)
            return

        if self._active_waypoint_index is not None:
            self._publish_periodic_progress(monotonic_s=monotonic_s)
            return

        if self._hover_until is not None:
            if monotonic_s < self._hover_until:
                self._publish_periodic_progress(monotonic_s=monotonic_s)
                return
            self._hover_until = None
            if self._task is not None and self._runtime.current_waypoint_index >= 0:
                completed_waypoint = self._task.waypoints[self._runtime.current_waypoint_index]
                self._action_runner.queue(completed_waypoint, task_id=self._runtime.task_id)

        if self._action_runner.has_pending():
            if not self._action_runner.tick():
                self._publish_periodic_progress(monotonic_s=monotonic_s)
                return

        if self._active_waypoint_index is None and self._hover_until is None:
            self._dispatch_next_waypoint_if_needed()

        self._publish_periodic_progress(monotonic_s=monotonic_s)

    def _handle_start(self, *, task_id: str, task_type: str, task_params_json: str):
        effective_task_type = self._effective_task_type(
            task_type=task_type,
            task_params_json=task_params_json,
        )

        if self._is_straight_drive_task(task_type=effective_task_type, task_params_json=task_params_json):
            self._handle_straight_drive_start(
                task_id=task_id,
                task_type=effective_task_type,
                task_params_json=task_params_json,
            )
            return

        if is_outdoor_task_payload(task_type=effective_task_type, task_params_json=task_params_json):
            self._handle_outdoor_start(
                task_id=task_id,
                task_type=effective_task_type,
                task_params_json=task_params_json,
            )
            return

        try:
            task = parse_task_definition(
                task_id=task_id,
                task_type=effective_task_type,
                task_params_json=task_params_json,
                defaults=self._mission_defaults,
            )
        except Exception as exc:
            self._runtime = TaskRuntime(
                task_id=task_id,
                stage='aborted',
                status='exception',
                error=self._make_error(
                    code='E_TASK_PARSE',
                    description=f'failed to parse task params: {exc}',
                    suggested_action='fix_task_payload',
                ),
            )
            self._task = None
            self._outdoor_mission = None
            self._publish_task_progress()
            self._write_state_fields()
            return

        self._task = task
        self._outdoor_mission = None
        self._paused_local_context = {}
        self._runtime = TaskRuntime(
            task_id=task.task_id,
            stage='preparing',
            status='normal',
            current_waypoint_index=-1,
            current_waypoint_seq=-1,
            total_waypoints=len(task.waypoints),
            progress_percent=0.0,
            event='task_received',
            error=None,
        )
        self._active_waypoint_index = None
        self._hover_until = None
        self._action_runner.reset()
        self._closed_loop_manager.reset()
        self._acquire_motion_control_lock(task_id=task.task_id, reason='task_received')

        if self._require_auto_mode and self._control_mode_getter() != 'auto':
            self._pause_with_error(
                code='E_TASK_CONFLICT',
                description='current control_mode is not auto; switch to auto before executing task',
                suggested_action='switch_to_auto_mode',
            )
            return

        if not self._projector.is_ready():
            self._pause_with_error(
                code='E_GEOREF_NOT_READY',
                description='geo reference is disabled or not configured',
                suggested_action='configure_geo_reference',
            )
            return

        self._set_map_pose_reporting_enabled(True)
        self._start_recording_if_needed()
        self._publish_task_progress()
        self._write_state_fields()

    def _effective_task_type(self, *, task_type: str, task_params_json: str) -> str:
        normalized_type = str(task_type or '').strip()
        try:
            payload = json.loads(task_params_json or '{}')
        except Exception:
            return normalized_type
        if not isinstance(payload, dict):
            return normalized_type

        payload_type = str(payload.get('task_type') or payload.get('type') or '').strip()
        if payload_type:
            return payload_type
        if is_outdoor_task_payload(task_type='', task_params_json=task_params_json):
            return 'mission_planner_route'
        return normalized_type

    def _is_straight_drive_task(self, *, task_type: str, task_params_json: str) -> bool:
        normalized_type = str(task_type or '').strip().lower()
        if normalized_type in {'straight_drive', 'drive_straight', 'forward_drive'}:
            return True
        try:
            payload = json.loads(task_params_json or '{}')
        except Exception:
            return False
        if not isinstance(payload, dict):
            return False
        payload_type = str(payload.get('task_type') or payload.get('type') or '').strip().lower()
        return payload_type in {'straight_drive', 'drive_straight', 'forward_drive'}

    def _optional_float(self, value) -> Optional[float]:
        if value in (None, ''):
            return None
        try:
            return float(value)
        except Exception as exc:
            raise ValueError(f'{value!r} is not a number') from exc

    def _handle_straight_drive_start(self, *, task_id: str, task_type: str, task_params_json: str):
        if self._straight_drive_controller is None:
            self._runtime = TaskRuntime(
                task_id=task_id,
                stage='aborted',
                status='exception',
                error=self._make_error(
                    code='E_STRAIGHT_DRIVE_UNAVAILABLE',
                    description='straight drive controller is unavailable',
                    suggested_action='check_autotask_config',
                ),
            )
            self._publish_task_progress()
            self._write_state_fields()
            return

        try:
            payload = json.loads(task_params_json or '{}')
            if not isinstance(payload, dict):
                payload = {}
        except Exception as exc:
            self._runtime = TaskRuntime(
                task_id=task_id,
                stage='aborted',
                status='exception',
                error=self._make_error(
                    code='E_STRAIGHT_DRIVE_PARSE',
                    description=f'failed to parse straight drive params: {exc}',
                    suggested_action='fix_task_payload',
                ),
            )
            self._publish_task_progress()
            self._write_state_fields()
            return

        try:
            distance_m = self._optional_float(payload.get('distance_m', payload.get('distance')))
            duration_s = self._optional_float(payload.get('duration_s', payload.get('duration')))
            speed_mps = self._optional_float(payload.get('speed_mps', payload.get('speed')))
            target_x = self._optional_float(payload.get('target_x'))
            target_y = self._optional_float(payload.get('target_y'))
            target_tolerance_m = self._optional_float(payload.get('target_tolerance_m'))
        except ValueError as exc:
            self._runtime = TaskRuntime(
                task_id=task_id,
                stage='aborted',
                status='exception',
                error=self._make_error(
                    code='E_STRAIGHT_DRIVE_PARSE',
                    description=f'failed to parse straight drive numeric params: {exc}',
                    suggested_action='fix_task_payload',
                ),
            )
            self._task = None
            self._outdoor_mission = None
            self._straight_drive_task = None
            self._publish_task_progress()
            self._write_state_fields()
            return
        self._task = None
        self._outdoor_mission = None
        self._paused_local_context = {}
        self._straight_drive_task = {
            'task_id': str(task_id or ''),
            'task_type': str(task_type or payload.get('task_type') or 'straight_drive'),
            'distance_m': distance_m,
            'duration_s': duration_s,
            'speed_mps': speed_mps,
            'target_x': target_x,
            'target_y': target_y,
            'target_tolerance_m': target_tolerance_m,
            'params': payload,
        }
        self._runtime = TaskRuntime(
            task_id=str(task_id or ''),
            stage='preparing',
            status='normal',
            current_waypoint_index=0,
            current_waypoint_seq=0,
            total_waypoints=1,
            progress_percent=0.0,
            event='straight_drive_received',
            error=None,
        )
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._active_local_goal_kind = ''
        self._pending_first_inspection_index = None
        self._pending_home_calibration_index = None
        self._active_home_calibration_index = None
        self._hover_until = None
        self._action_runner.reset()
        self._closed_loop_manager.reset()
        self._acquire_motion_control_lock(task_id=self._runtime.task_id, reason='straight_drive_received')

        if self._require_auto_mode and self._control_mode_getter() != 'auto':
            self._pause_with_error(
                code='E_TASK_CONFLICT',
                description='current control_mode is not auto; switch to auto before straight drive',
                suggested_action='switch_to_auto_mode',
            )
            return

        result = self._straight_drive_controller.start(
            task_id=self._runtime.task_id,
            distance_m=self._straight_drive_task.get('distance_m'),
            duration_s=self._straight_drive_task.get('duration_s'),
            speed_mps=self._straight_drive_task.get('speed_mps'),
            target_x=self._straight_drive_task.get('target_x'),
            target_y=self._straight_drive_task.get('target_y'),
            target_tolerance_m=self._straight_drive_task.get('target_tolerance_m'),
            goal_kind='inspection',
            requires_stop=True,
        )
        if not result.get('accepted', False):
            self._pause_with_error(
                code=result.get('code', 'E_STRAIGHT_DRIVE_REJECTED'),
                description=result.get('message', 'straight drive command was rejected'),
                suggested_action='check_straight_drive_params',
            )
            return

        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._runtime.event = 'straight_drive_started'
        self._publish_task_progress()
        self._write_state_fields()

    def _handle_outdoor_start(self, *, task_id: str, task_type: str, task_params_json: str):
        try:
            outdoor_mission = parse_outdoor_task_definition(
                task_id=task_id,
                task_type=task_type,
                task_params_json=task_params_json,
                defaults=self._mission_defaults,
            )
        except Exception as exc:
            self._runtime = TaskRuntime(
                task_id=task_id,
                stage='aborted',
                status='exception',
                error=self._make_error(
                    code='E_OUTDOOR_TASK_PARSE',
                    description=f'failed to parse outdoor task params: {exc}',
                    suggested_action='fix_mission_planner_payload',
                ),
            )
            self._task = None
            self._outdoor_mission = None
            self._publish_task_progress()
            self._write_state_fields()
            return

        self._task = None
        self._outdoor_mission = outdoor_mission
        self._paused_local_context = {}
        self._runtime = TaskRuntime(
            task_id=outdoor_mission.task_id,
            stage='preparing',
            status='normal',
            current_waypoint_index=-1,
            current_waypoint_seq=-1,
            total_waypoints=len(outdoor_mission.execution_points),
            progress_percent=0.0,
            event='outdoor_task_received',
            error=None,
        )
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._active_local_goal_kind = ''
        self._hover_until = None
        self._action_runner.reset()
        self._closed_loop_manager.reset()
        self._outdoor_calibration_records = []
        self._outdoor_goal_records = []
        self._outdoor_start_alignment = {}
        self._gps_vo_yaw_aligner.reset()
        self._pending_first_inspection_index = None
        self._pending_home_calibration_index = None
        self._active_home_calibration_index = None
        self._initial_yaw_calibration_active = False
        self._outdoor_gps_vo_gate = GpsVoGate(
            stable_required_count=outdoor_mission.stable_offset_required_count
        )
        self._outdoor_pose_aligner.reset()
        self._acquire_motion_control_lock(
            task_id=outdoor_mission.task_id,
            reason='outdoor_task_received',
        )

        if self._require_auto_mode and self._control_mode_getter() != 'auto':
            self._pause_with_error(
                code='E_TASK_CONFLICT',
                description='current control_mode is not auto; switch to auto before executing outdoor task',
                suggested_action='switch_to_auto_mode',
            )
            return

        self._outdoor_start_alignment = self._outdoor_start_aligner.evaluate_and_apply(
            mission=outdoor_mission,
            current_pose=self.local_navigation_pose(),
            now_ns=self._now_ns_getter(),
        )
        alignment_status = str(self._outdoor_start_alignment.get('status', ''))
        alignment_action = str(self._outdoor_start_alignment.get('action', ''))
        if alignment_action == 'reject':
            self._abort_task(
                code='E_OUTDOOR_START_MISMATCH',
                description=self._outdoor_start_alignment.get(
                    'message',
                    'current pose does not match planned outdoor start',
                ),
                suggested_action='replan_from_current_pose',
            )
            return
        if alignment_action == 'wait':
            self._pause_with_error(
                code='E_OUTDOOR_START_POSE_UNAVAILABLE',
                description=self._outdoor_start_alignment.get(
                    'message',
                    'current local navigation pose is unavailable',
                ),
                suggested_action='wait_for_gps_or_localization',
            )
            return
        if alignment_action == 'run_home_calibration':
            self._pending_first_inspection_index = int(
                self._outdoor_start_alignment.get('first_inspection_index', -1)
            )
            home_index = int(self._outdoor_start_alignment.get('home_calibration_index', -1))
            self._pending_home_calibration_index = home_index
            if self._should_run_initial_yaw_calibration():
                if not self._dispatch_initial_yaw_calibration():
                    return
                self._publish_task_progress()
                self._write_state_fields()
                return
            if not self._dispatch_home_calibration(home_index=home_index):
                return
            self._publish_task_progress()
            self._write_state_fields()
            return
        if 'next_current_waypoint_index' in self._outdoor_start_alignment:
            self._runtime.current_waypoint_index = int(
                self._outdoor_start_alignment.get('next_current_waypoint_index', -1)
            )
            self._runtime.current_waypoint_seq = (
                outdoor_mission.execution_points[self._runtime.current_waypoint_index].seq
                if 0 <= self._runtime.current_waypoint_index < len(outdoor_mission.execution_points)
                else -1
            )
            self._runtime.progress_percent = (
                100.0 * (self._runtime.current_waypoint_index + 1) /
                max(1, len(outdoor_mission.execution_points))
                if self._runtime.current_waypoint_index >= 0 else 0.0
            )

        if self._should_run_initial_yaw_calibration():
            if not self._dispatch_initial_yaw_calibration():
                return
            self._publish_task_progress()
            self._write_state_fields()
            return
        self._runtime.event = alignment_status or 'outdoor_task_received'

        self._set_map_pose_reporting_enabled(True)
        self._publish_task_progress()
        self._write_state_fields()

    def _handle_pause(self, *, reason: str):
        if (
            self._task is None and
            self._outdoor_mission is None and
            self._straight_drive_task is None
        ) or self._runtime.stage in TERMINAL_TASK_STAGES:
            return

        self._capture_paused_local_context()
        self._stop_straight_drive_if_needed(reason=reason)
        self._stop_backend_navigation_if_needed()
        self._publish_move_stop(reason=reason)
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._active_home_calibration_index = None
        self._initial_yaw_calibration_active = False
        self._hover_until = None
        self._action_runner.reset()
        self._runtime.stage = 'paused'
        self._runtime.status = 'paused'
        self._runtime.event = reason
        self._release_motion_control_lock(reason=reason)
        self._publish_task_progress()
        self._write_state_fields()

    def _handle_resume(self):
        if self._task is None and self._outdoor_mission is None and self._straight_drive_task is None:
            return
        if self._runtime.stage != 'paused':
            return
        if self._require_auto_mode and self._control_mode_getter() != 'auto':
            self._pause_with_error(
                code='E_TASK_CONFLICT',
                description='cannot resume task because control_mode is not auto',
                suggested_action='switch_to_auto_mode',
            )
            return
        paused_context = dict(self._paused_local_context)
        self._acquire_motion_control_lock(task_id=self._runtime.task_id, reason='task_resumed')
        if self._straight_drive_task is not None and self._straight_drive_controller is not None:
            distance_m = self._straight_drive_task.get('distance_m')
            duration_s = self._straight_drive_task.get('duration_s')
            if str(paused_context.get('kind') or '') == 'straight_drive':
                if paused_context.get('remaining_distance_m') is not None:
                    distance_m = float(paused_context['remaining_distance_m'])
                if paused_context.get('remaining_duration_s') is not None:
                    duration_s = float(paused_context['remaining_duration_s'])
            if (
                (distance_m is not None and float(distance_m) <= 0.0) or
                (duration_s is not None and float(duration_s) <= 0.0)
            ):
                self._complete_task()
                return
            result = self._straight_drive_controller.start(
                task_id=self._runtime.task_id,
                distance_m=distance_m,
                duration_s=duration_s,
                speed_mps=self._straight_drive_task.get('speed_mps'),
                target_x=self._straight_drive_task.get('target_x'),
                target_y=self._straight_drive_task.get('target_y'),
                target_tolerance_m=self._straight_drive_task.get('target_tolerance_m'),
                goal_kind='inspection',
                requires_stop=True,
            )
            if not result.get('accepted', False):
                self._pause_with_error(
                    code=result.get('code', 'E_STRAIGHT_DRIVE_REJECTED'),
                    description=result.get('message', 'straight drive command was rejected'),
                    suggested_action='check_straight_drive_params',
                )
                return
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._runtime.event = 'task_resumed'
        self._runtime.error = None
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._active_local_goal_kind = ''
        self._active_home_calibration_index = None
        context_kind = str(paused_context.get('kind') or '')
        if context_kind == 'home_calibration':
            home_index = int(paused_context.get('index', -1))
            if not self._dispatch_home_calibration(home_index=home_index):
                return
        elif context_kind == 'initial_yaw_calibration':
            if not self._dispatch_initial_yaw_calibration():
                return
        self._paused_local_context = {}
        self._publish_task_progress()
        self._write_state_fields()

    def _handle_stop(self, *, reason: str):
        self._stop_straight_drive_if_needed(reason=reason)
        self._stop_backend_navigation_if_needed()
        self._publish_move_stop(reason=reason)
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._active_local_goal_kind = ''
        self._pending_first_inspection_index = None
        self._pending_home_calibration_index = None
        self._active_home_calibration_index = None
        self._initial_yaw_calibration_active = False
        self._gps_vo_yaw_aligner.reset()
        self._straight_drive_task = None
        self._paused_local_context = {}
        self._hover_until = None
        self._action_runner.reset()
        self._runtime.stage = 'aborted'
        self._runtime.status = 'exception'
        self._runtime.event = reason
        self._runtime.error = self._make_error(
            code='E_TASK_STOPPED',
            description='task stopped by external command',
            suggested_action='restart_task',
            severity='info',
        )
        self._release_motion_control_lock(reason=reason)
        self._stop_recording_if_needed()
        self._set_map_pose_reporting_enabled(False)
        self._publish_task_progress()
        self._write_state_fields()

    def _handle_straight_drive_control_result(
        self,
        *,
        result: Dict[str, Any],
        monotonic_s: float,
    ):
        state = str(result.get('state') or '')
        if state == 'completed':
            self._runtime.current_waypoint_index = 0
            self._runtime.current_waypoint_seq = 0
            self._runtime.progress_percent = 100.0
            self._complete_task()
            return

        if self._handle_terminal_local_control_result(
            result,
            default_code='E_STRAIGHT_DRIVE_BLOCKED',
            default_description='straight drive is blocked',
        ):
            return

        if not result.get('throttled', False):
            self._runtime.stage = 'executing'
            self._runtime.status = 'normal'
            self._runtime.event = f'straight_drive_{state or "running"}'
            if self._straight_drive_task.get('distance_m'):
                traveled = float(result.get('distance_traveled_m', 0.0) or 0.0)
                target = max(0.01, float(self._straight_drive_task.get('distance_m') or 0.01))
                self._runtime.progress_percent = max(0.0, min(99.0, 100.0 * traveled / target))
            elif self._straight_drive_task.get('duration_s'):
                elapsed = float(result.get('elapsed_s', 0.0) or 0.0)
                target = max(0.01, float(self._straight_drive_task.get('duration_s') or 0.01))
                self._runtime.progress_percent = max(0.0, min(99.0, 100.0 * elapsed / target))

        self._publish_periodic_progress(monotonic_s=monotonic_s)

    def _handle_terminal_local_control_result(
        self,
        result: Dict[str, Any],
        *,
        default_code: str,
        default_description: str,
    ) -> bool:
        if not bool(result.get('terminal', False)):
            return False
        code = str(result.get('code') or default_code)
        suggested_actions = {
            'E_CONTROL_MODE': 'switch_to_auto_mode',
            'E_CONTROLLER_MODE': 'switch_to_auto_controller',
            'E_FAILSAFE_ACTIVE': 'restore_link_and_clear_failsafe',
            'E_POSE_UNAVAILABLE': 'wait_for_localization',
            'E_POSE_STALE': 'check_localization_input',
            'E_SCAN_UNAVAILABLE': 'check_lidar_topic',
            'E_SCAN_STALE': 'check_lidar_topic',
            'E_SCAN_QUALITY': 'check_lidar_data',
            'E_SCAN_FRONT_QUALITY': 'check_lidar_data',
            'E_BOTTOM_STATUS': 'check_cyberdog_motion_status',
            'E_LOW_BATTERY': 'charge_robot',
        }
        self._pause_with_error(
            code=code,
            description=str(result.get('reason') or default_description),
            suggested_action=suggested_actions.get(
                code,
                'clear_obstacle_or_send_stop',
            ),
        )
        return True

    def _dispatch_next_waypoint_if_needed(self):
        if self._task is None:
            return

        next_index = self._runtime.current_waypoint_index + 1
        if next_index >= len(self._task.waypoints):
            self._complete_task()
            return

        waypoint = self._task.waypoints[next_index]
        if not self._projector.is_ready():
            self._pause_with_error(
                code='E_GEOREF_NOT_READY',
                description='geo reference is required to project waypoint task into map frame',
                suggested_action='configure_geo_reference',
            )
            return

        try:
            map_x, map_y = self._projector.latlon_to_map_xy(waypoint.lat, waypoint.lon)
        except Exception as exc:
            self._pause_with_error(
                code='E_GEOREF_PROJECT_FAILED',
                description=f'failed to project waypoint into map frame: {exc}',
                suggested_action='check_geo_reference_and_waypoint',
            )
            return

        route_start = self._waypoint_map_xy(next_index - 1)
        if route_start is None:
            current_pose = self.local_navigation_pose()
            if 'x' in current_pose and 'y' in current_pose:
                route_start = (float(current_pose['x']), float(current_pose['y']))
        route_next = self._waypoint_map_xy(next_index + 1)
        result = self._start_local_map_goal(
            task_id=self._runtime.task_id,
            target_x=map_x,
            target_y=map_y,
            speed_mps=waypoint.speed_mps,
            source='waypoint',
            target_tolerance_m=None,
            route_start_x=route_start[0] if route_start is not None else None,
            route_start_y=route_start[1] if route_start is not None else None,
            route_next_x=route_next[0] if route_next is not None else None,
            route_next_y=route_next[1] if route_next is not None else None,
            goal_kind='inspection',
            requires_stop=True,
        )
        if not result.get('success', False):
            self._pause_with_error(
                code=result.get('error_code', 'E_MOVE_FAIL'),
                description=result.get('description', 'failed to dispatch waypoint'),
                suggested_action=result.get('suggested_action', 'check_local_velocity_navigation'),
            )
            return

        self._active_waypoint_index = next_index
        self._active_local_goal_kind = 'waypoint'
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._runtime.event = 'waypoint_local_goal_started'
        self._runtime.current_waypoint_seq = waypoint.seq
        self._runtime.last_goal_map_pose = {
            'x': float(map_x),
            'y': float(map_y),
            'z': float(waypoint.alt),
            'navigation_mode': 'local_lidar_velocity',
            'coordinate_frame': 'map_or_mission_planner_local',
        }
        self._publish_task_progress()
        self._write_state_fields()

    def _waypoint_map_xy(self, index: int):
        if self._task is None or index < 0 or index >= len(self._task.waypoints):
            return None
        waypoint = self._task.waypoints[index]
        try:
            return self._projector.latlon_to_map_xy(waypoint.lat, waypoint.lon)
        except Exception:
            return None

    def _handle_waypoint_control_result(
        self,
        *,
        result: Dict[str, Any],
        monotonic_s: float,
    ):
        state = str(result.get('state') or '')
        if state == 'completed':
            self._handle_waypoint_success()
            return

        if self._handle_terminal_local_control_result(
            result,
            default_code='E_LOCAL_NAV_BLOCKED',
            default_description='local LiDAR navigation is blocked',
        ):
            return

        if not result.get('throttled', False):
            self._runtime.stage = 'executing'
            self._runtime.status = 'normal'
            self._runtime.event = f'waypoint_local_goal_{state or "running"}'

        self._publish_periodic_progress(monotonic_s=monotonic_s)

    def _tick_outdoor_task(self, *, monotonic_s: float, backend_event):
        if self._outdoor_mission is None:
            return

        if self._require_auto_mode and self._control_mode_getter() != 'auto':
            self._pause_with_error(
                code='E_TASK_CONFLICT',
                description='current control_mode is not auto',
                suggested_action='switch_to_auto_mode',
            )
            return

        self._update_gps_vo_yaw_alignment()

        if backend_event is not None:
            self._handle_outdoor_backend_event(backend_event)

        if self._runtime.stage == 'paused':
            self._publish_periodic_progress(monotonic_s=monotonic_s)
            return

        if self._initial_yaw_calibration_active:
            self._publish_periodic_progress(monotonic_s=monotonic_s)
            return

        if self._active_home_calibration_index is not None:
            self._publish_periodic_progress(monotonic_s=monotonic_s)
            return

        if self._active_outdoor_point_index is not None:
            self._publish_periodic_progress(monotonic_s=monotonic_s)
            return

        if self._active_outdoor_point_index is None:
            self._dispatch_next_outdoor_point_if_needed()

        self._publish_periodic_progress(monotonic_s=monotonic_s)

    def _dispatch_next_outdoor_point_if_needed(self):
        mission = self._outdoor_mission
        if mission is None:
            return

        next_index = self._runtime.current_waypoint_index + 1
        if next_index >= len(mission.execution_points):
            self._complete_task()
            return

        point = mission.execution_points[next_index]
        resolution = self._outdoor_goal_resolver.resolve(
            point,
            previous_point=self._outdoor_point_at(next_index - 1),
            next_point=self._outdoor_point_at(next_index + 1),
            current_pose=self._pose_registry.map_pose_dict(),
        )
        if not resolution.candidates:
            self._pause_with_error(
                code='E_OUTDOOR_GOAL_RESOLVE_EMPTY',
                description='outdoor goal resolver produced no candidate target',
                suggested_action='check_goal_resolver_config',
            )
            return

        self._active_outdoor_goal_resolution = resolution
        self._active_outdoor_candidate_index = 0
        if not self._dispatch_outdoor_candidate(next_index=next_index, candidate_index=0):
            self._active_outdoor_goal_resolution = None
            self._active_outdoor_candidate_index = None

    def _dispatch_initial_yaw_calibration(self) -> bool:
        mission = self._outdoor_mission
        if mission is None:
            return False
        if self._straight_drive_controller is None:
            self._pause_with_error(
                code='E_LOCAL_NAV_UNAVAILABLE',
                description='local LiDAR velocity navigation controller is unavailable',
                suggested_action='check_autotask_config',
            )
            return False

        # A resumed calibration must not combine displacement collected before
        # and after a pause into one heading estimate.
        self._gps_vo_yaw_aligner.reset()
        result = self._straight_drive_controller.start(
            task_id=mission.task_id,
            distance_m=self._gps_vo_yaw_aligner.calibration_drive_distance_m,
            speed_mps=None,
            command_source='initial_yaw_calibration',
            goal_kind='calibration',
            requires_stop=True,
        )
        if not result.get('accepted', False):
            self._pause_with_error(
                code=result.get('code', 'E_INITIAL_YAW_CALIBRATION_REJECTED'),
                description=result.get('message', 'failed to start initial yaw calibration'),
                suggested_action='check_gps_visual_odom_scan_and_move_gateway',
            )
            return False

        self._initial_yaw_calibration_active = True
        self._active_local_goal_kind = 'initial_yaw_calibration'
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._runtime.event = 'initial_yaw_calibration_started'
        self._runtime.last_goal_map_pose = {
            'navigation_mode': 'local_lidar_velocity',
            'calibration_role': 'initial_yaw',
            'distance_m': self._gps_vo_yaw_aligner.calibration_drive_distance_m,
            'required_gps_displacement_m': self._gps_vo_yaw_aligner.min_gps_displacement_m,
            'required_visual_displacement_m': self._gps_vo_yaw_aligner.min_visual_displacement_m,
        }
        return True

    def _handle_initial_yaw_control_result(
        self,
        *,
        result: Dict[str, Any],
        monotonic_s: float,
    ):
        state = str(result.get('state') or '')
        alignment = self._gps_vo_yaw_aligner.state_snapshot()
        if alignment.get('aligned', False):
            self._straight_drive_controller.stop(reason='initial_yaw_calibration_completed')
            self._initial_yaw_calibration_active = False
            self._active_local_goal_kind = ''
            pending_home_index = self._pending_home_calibration_index
            if pending_home_index is not None:
                if not self._dispatch_home_calibration(home_index=pending_home_index):
                    return
                self._publish_task_progress()
                self._write_state_fields()
                return
            self._runtime.stage = 'executing'
            self._runtime.status = 'normal'
            self._runtime.event = 'initial_yaw_calibration_completed'
            self._runtime.error = None
            self._publish_task_progress()
            self._write_state_fields()
            return

        if state == 'completed':
            self._pause_with_error(
                code='E_INITIAL_YAW_CALIBRATION_FAILED',
                description='initial yaw calibration finished but gps/visual displacement was not reliable',
                suggested_action='retry_in_open_area_or_increase_calibration_distance',
            )
            return

        if result.get('terminal'):
            self._handle_terminal_local_control_result(
                result,
                default_code='E_INITIAL_YAW_CALIBRATION_BLOCKED',
                default_description='initial yaw calibration is blocked',
            )
            return

        if not result.get('throttled', False):
            self._runtime.stage = 'executing'
            self._runtime.status = 'normal'
            self._runtime.event = f'initial_yaw_calibration_{state or "running"}'

        self._publish_periodic_progress(monotonic_s=monotonic_s)

    def _dispatch_home_calibration(self, *, home_index: int) -> bool:
        mission = self._outdoor_mission
        if mission is None or home_index < 0 or home_index >= len(mission.execution_points):
            self._abort_task(
                code='E_OUTDOOR_HOME_UNAVAILABLE',
                description='home calibration is required but home point is unavailable',
                suggested_action='fix_mission_planner_payload',
            )
            return False

        home = mission.execution_points[home_index]
        current_pose = self.local_navigation_pose()
        next_point = self._outdoor_point_at(
            self._pending_first_inspection_index
            if self._pending_first_inspection_index is not None else -1
        )
        result = self._start_local_map_goal(
            task_id=mission.task_id,
            target_x=float(home.x),
            target_y=float(home.y),
            speed_mps=None,
            source='home_calibration',
            target_tolerance_m=self._local_goal_target_tolerance(home),
            route_start_x=current_pose.get('x'),
            route_start_y=current_pose.get('y'),
            route_next_x=next_point.x if next_point is not None else None,
            route_next_y=next_point.y if next_point is not None else None,
            goal_kind='home',
            requires_stop=True,
        )
        if not result.get('success', False):
            self._pause_with_error(
                code=result.get('error_code', 'E_HOME_CALIBRATION_REJECTED'),
                description=result.get('description', 'failed to start home calibration'),
                suggested_action=result.get('suggested_action', 'check_local_velocity_navigation'),
            )
            return False

        self._pending_home_calibration_index = None
        self._active_home_calibration_index = home_index
        self._active_local_goal_kind = 'home_calibration'
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._runtime.event = 'home_calibration_started'
        self._runtime.current_waypoint_seq = home.seq
        self._runtime.last_goal_map_pose = {
            'x': float(home.x),
            'y': float(home.y),
            'z': float(home.z),
            'navigation_mode': 'local_lidar_velocity',
            'coordinate_frame': 'map_or_mission_planner_local',
            'calibration_role': 'home',
        }
        return True

    def _handle_home_calibration_control_result(
        self,
        *,
        result: Dict[str, Any],
        monotonic_s: float,
    ):
        state = str(result.get('state') or '')
        if state == 'completed':
            self._handle_home_calibration_reached()
            return

        if self._handle_terminal_local_control_result(
            result,
            default_code='E_HOME_CALIBRATION_BLOCKED',
            default_description='home calibration is blocked',
        ):
            return

        if not result.get('throttled', False):
            self._runtime.stage = 'executing'
            self._runtime.status = 'normal'
            self._runtime.event = f'home_calibration_{state or "running"}'

        self._publish_periodic_progress(monotonic_s=monotonic_s)

    def _handle_home_calibration_reached(self):
        mission = self._outdoor_mission
        if mission is None:
            return

        self._active_home_calibration_index = None
        self._active_local_goal_kind = ''
        first_index = self._pending_first_inspection_index
        current_pose = self.local_navigation_pose()
        alignment = self._outdoor_start_aligner.evaluate_and_apply(
            mission=mission,
            current_pose=current_pose,
            now_ns=self._now_ns_getter(),
        )
        self._outdoor_start_alignment = alignment
        action = str(alignment.get('action') or '')
        if action == 'reject':
            self._abort_task(
                code='E_OUTDOOR_HOME_CALIBRATION_FAILED',
                description=alignment.get('message', 'home calibration failed'),
                suggested_action='replan_from_current_pose',
            )
            return
        if action == 'wait':
            self._pause_with_error(
                code='E_OUTDOOR_START_POSE_UNAVAILABLE',
                description=alignment.get('message', 'current local navigation pose is unavailable'),
                suggested_action='wait_for_gps_or_localization',
            )
            return
        if action == 'run_home_calibration':
            home_distance_m = float(alignment.get('home_distance_m', float('inf')))
            reject_distance_m = float(alignment.get('home_calibration_reject_distance_m', 10.0))
            if home_distance_m > reject_distance_m:
                self._abort_task(
                    code='E_OUTDOOR_HOME_CALIBRATION_FAILED',
                    description=alignment.get('message', 'home calibration distance is still too large'),
                    suggested_action='replan_from_current_pose',
                )
                return
            self._pause_with_error(
                code='E_OUTDOOR_HOME_CALIBRATION_INCOMPLETE',
                description='home calibration reached but first inspection is still too far',
                suggested_action='check_gps_or_replan',
            )
            return

        if first_index is None or first_index < 0:
            first_index = int(alignment.get('first_inspection_index', -1))
        self._pending_first_inspection_index = None
        self._runtime.current_waypoint_index = int(first_index) - 1
        self._runtime.current_waypoint_seq = (
            mission.execution_points[self._runtime.current_waypoint_index].seq
            if 0 <= self._runtime.current_waypoint_index < len(mission.execution_points)
            else -1
        )
        self._runtime.progress_percent = (
            100.0 * (self._runtime.current_waypoint_index + 1) /
            max(1, len(mission.execution_points))
            if self._runtime.current_waypoint_index >= 0 else 0.0
        )
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._runtime.event = 'home_calibration_completed'
        self._runtime.error = None
        self._publish_task_progress()
        self._write_state_fields()

    def _outdoor_point_at(self, index: int):
        mission = self._outdoor_mission
        if mission is None:
            return None
        if index < 0 or index >= len(mission.execution_points):
            return None
        return mission.execution_points[index]

    def _dispatch_outdoor_candidate(self, *, next_index: int, candidate_index: int) -> bool:
        mission = self._outdoor_mission
        resolution = self._active_outdoor_goal_resolution
        if mission is None or resolution is None:
            return False

        candidate = resolution.candidate(candidate_index)
        if candidate is None:
            self._pause_with_error(
                code='E_OUTDOOR_GOAL_CANDIDATE_EMPTY',
                description='outdoor goal resolver candidate index is out of range',
                suggested_action='check_goal_resolver_config',
            )
            return False

        point = mission.execution_points[next_index]
        goal_correction = self._current_outdoor_goal_correction()
        correction_x = float(goal_correction.get('dx', 0.0))
        correction_y = float(goal_correction.get('dy', 0.0))
        corrected_x = float(candidate.x) + correction_x
        corrected_y = float(candidate.y) + correction_y
        previous_point = self._outdoor_point_at(next_index - 1)
        next_point = self._outdoor_point_at(next_index + 1)
        route_start = None
        if previous_point is not None:
            route_start = (
                float(previous_point.x) + correction_x,
                float(previous_point.y) + correction_y,
            )
        if route_start is None:
            current_pose = self.local_navigation_pose()
            if 'x' in current_pose and 'y' in current_pose:
                route_start = (float(current_pose['x']), float(current_pose['y']))
        route_next = (
            (float(next_point.x) + correction_x, float(next_point.y) + correction_y)
            if next_point is not None else None
        )
        point_kind = str(point.kind or '').strip().lower() or 'inspection'
        result = self._start_local_map_goal(
            task_id=mission.task_id,
            target_x=corrected_x,
            target_y=corrected_y,
            speed_mps=None,
            source='mission_planner_route',
            target_tolerance_m=self._local_goal_target_tolerance(point),
            route_start_x=route_start[0] if route_start is not None else None,
            route_start_y=route_start[1] if route_start is not None else None,
            route_next_x=route_next[0] if route_next is not None else None,
            route_next_y=route_next[1] if route_next is not None else None,
            goal_kind=point_kind,
            requires_stop=point_kind != 'transit',
        )
        if not result.get('success', False):
            self._pause_with_error(
                code=result.get('error_code', 'E_MOVE_FAIL'),
                description=result.get('description', 'failed to dispatch outdoor route point'),
                suggested_action=result.get('suggested_action', 'check_local_velocity_navigation'),
            )
            return False

        self._active_outdoor_point_index = next_index
        self._active_outdoor_candidate_index = candidate_index
        self._active_local_goal_kind = 'outdoor'
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._runtime.event = (
            'outdoor_point_local_goal_started'
            if candidate_index == 0 else
            'outdoor_point_retry_local_goal_started'
        )
        self._runtime.current_waypoint_seq = point.seq
        self._runtime.last_goal_map_pose = {
            'x': corrected_x,
            'y': corrected_y,
            'z': float(point.z),
            'original_x': float(point.x),
            'original_y': float(point.y),
            'candidate_x': float(candidate.x),
            'candidate_y': float(candidate.y),
            'goal_correction': goal_correction,
            'candidate_index': int(candidate_index),
            'candidate_source': candidate.source,
            'offset_kind': candidate.offset_kind,
            'offset_distance_m': candidate.offset_distance_m,
            'navigation_mode': 'local_lidar_velocity',
            'coordinate_frame': 'map_or_mission_planner_local',
        }
        self._publish_task_progress()
        self._write_state_fields()
        return True

    def _start_local_map_goal(
        self,
        *,
        task_id: str,
        target_x: float,
        target_y: float,
        speed_mps: Optional[float],
        source: str,
        target_tolerance_m: Optional[float] = None,
        route_start_x: Optional[float] = None,
        route_start_y: Optional[float] = None,
        route_next_x: Optional[float] = None,
        route_next_y: Optional[float] = None,
        goal_kind: str = 'inspection',
        requires_stop: Optional[bool] = None,
    ) -> Dict[str, Any]:
        if self._straight_drive_controller is None:
            return {
                'success': False,
                'error_code': 'E_LOCAL_NAV_UNAVAILABLE',
                'description': 'local LiDAR velocity navigation controller is unavailable',
                'suggested_action': 'check_autotask_config',
            }
        if not self.local_navigation_pose():
            return {
                'success': False,
                'error_code': 'E_LOCAL_NAV_POSE_UNAVAILABLE',
                'description': 'map pose or GPS projected local pose is required for local route navigation',
                'suggested_action': 'wait_for_gps_or_localization',
            }

        result = self._straight_drive_controller.start(
            task_id=task_id,
            speed_mps=speed_mps,
            target_x=float(target_x),
            target_y=float(target_y),
            target_tolerance_m=target_tolerance_m,
            command_source=source,
            route_start_x=route_start_x,
            route_start_y=route_start_y,
            route_next_x=route_next_x,
            route_next_y=route_next_y,
            goal_kind=goal_kind,
            requires_stop=requires_stop,
        )
        if not result.get('accepted', False):
            return {
                'success': False,
                'error_code': result.get('code', 'E_LOCAL_NAV_REJECTED'),
                'description': result.get('message', 'local LiDAR velocity navigation rejected target'),
                'suggested_action': 'check_scan_map_pose_and_move_gateway',
            }
        return {
            'success': True,
            'mode': 'local_lidar_velocity',
            'source': str(source),
        }

    def _local_goal_target_tolerance(self, point) -> Optional[float]:
        point_kind = str(getattr(point, 'kind', '') or '').strip().lower()
        if point_kind == 'transit':
            return None
        tolerance_m = getattr(point, 'tolerance_m', None)
        if tolerance_m in (None, ''):
            return None
        try:
            return float(tolerance_m)
        except (TypeError, ValueError):
            return None

    def _handle_outdoor_control_result(
        self,
        *,
        result: Dict[str, Any],
        monotonic_s: float,
    ):
        state = str(result.get('state') or '')
        if state == 'completed':
            self._handle_outdoor_point_success()
            return

        if result.get('terminal') and state == 'blocked':
            self._record_active_outdoor_goal(
                status='local_navigation_blocked',
                backend_event={'state': state, 'reason': result.get('reason', '')},
            )
        if self._handle_terminal_local_control_result(
            result,
            default_code='E_LOCAL_NAV_BLOCKED',
            default_description='local LiDAR navigation is blocked',
        ):
            return

        if not result.get('throttled', False):
            self._runtime.stage = 'executing'
            self._runtime.status = 'normal'
            self._runtime.event = f'outdoor_local_goal_{state or "running"}'

        self._publish_periodic_progress(monotonic_s=monotonic_s)

    def _current_outdoor_goal_correction(self) -> Dict[str, Any]:
        alignment_state = self._outdoor_pose_aligner.state_snapshot()
        correction = dict((alignment_state.get('last_result') or {}).get('goal_correction') or {})
        if not correction.get('enabled', False):
            return {
                'enabled': False,
                'applied': False,
                'dx': 0.0,
                'dy': 0.0,
                'reason': correction.get('reason', 'goal correction is disabled'),
            }
        if not correction.get('applied', False):
            return {
                'enabled': True,
                'applied': False,
                'dx': 0.0,
                'dy': 0.0,
                'reason': correction.get('reason', 'alignment estimate is not stable'),
            }
        return {
            'enabled': True,
            'applied': True,
            'dx': float(correction.get('dx', 0.0)),
            'dy': float(correction.get('dy', 0.0)),
            'reason': correction.get('reason', 'stable gps/map offset correction'),
            'source': 'gps_map_alignment',
        }

    def _handle_outdoor_backend_event(self, backend_event: Dict[str, Any]):
        state = backend_event.get('state')
        if state == 'succeeded':
            self._handle_outdoor_point_success()
            return

        if state in {'stopped', 'canceled'}:
            if self._runtime.stage == 'paused':
                self._runtime.event = 'navigation_stopped'
                self._publish_task_progress()
            return

        if self._retry_next_outdoor_candidate(backend_event):
            return

        if self._skip_current_outdoor_point_if_allowed(backend_event):
            return

        self._record_active_outdoor_goal(status='failed_exhausted', backend_event=backend_event)
        self._pause_with_error(
            code='E_MOVE_FAIL',
            description=(
                'legacy navigation backend returned terminal state '
                f'{state}, feedback={backend_event.get("feedback_msg", "")}'
            ),
            suggested_action='check_local_velocity_navigation_or_disable_legacy_backend',
        )

    def _retry_next_outdoor_candidate(self, backend_event: Dict[str, Any]) -> bool:
        if (
            self._active_outdoor_point_index is None or
            self._active_outdoor_goal_resolution is None or
            self._active_outdoor_candidate_index is None
        ):
            return False

        current_index = int(self._active_outdoor_candidate_index)
        if not self._active_outdoor_goal_resolution.has_next_after(current_index):
            return False

        self._record_active_outdoor_goal(status='candidate_failed', backend_event=backend_event)
        next_candidate_index = current_index + 1
        self._runtime.event = 'outdoor_point_retrying_with_resolved_candidate'
        dispatched = self._dispatch_outdoor_candidate(
            next_index=self._active_outdoor_point_index,
            candidate_index=next_candidate_index,
        )
        return True if dispatched or self._runtime.stage == 'paused' else False

    def _skip_current_outdoor_point_if_allowed(self, backend_event: Dict[str, Any]) -> bool:
        mission = self._outdoor_mission
        if mission is None or self._active_outdoor_point_index is None:
            return False

        point = mission.execution_points[self._active_outdoor_point_index]
        if not self._outdoor_goal_resolver.should_skip_after_exhausted(point.kind):
            return False

        self._record_active_outdoor_goal(status='skipped_unreachable', backend_event=backend_event)
        self._runtime.current_waypoint_index = self._active_outdoor_point_index
        self._runtime.current_waypoint_seq = point.seq
        self._runtime.progress_percent = (
            100.0 * (self._runtime.current_waypoint_index + 1) /
            max(1, len(mission.execution_points))
        )
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._active_local_goal_kind = ''
        self._runtime.event = 'outdoor_transit_point_skipped_unreachable'
        self._runtime.error = None
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        self._publish_task_progress()
        self._write_state_fields()
        return True

    def _handle_outdoor_point_success(self):
        mission = self._outdoor_mission
        if mission is None or self._active_outdoor_point_index is None:
            return

        point = mission.execution_points[self._active_outdoor_point_index]
        self._record_active_outdoor_goal(status='reached', backend_event=None)
        self._runtime.current_waypoint_index = self._active_outdoor_point_index
        self._runtime.current_waypoint_seq = point.seq
        self._runtime.progress_percent = (
            100.0 * (self._runtime.current_waypoint_index + 1) /
            max(1, len(mission.execution_points))
        )
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._active_local_goal_kind = ''
        self._runtime.event = 'outdoor_point_reached'
        self._runtime.error = None
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'

        if mission.calibrate_at_nav_points and point.kind in {'calibration', 'inspection', 'home'}:
            calibration = self._record_outdoor_calibration(point)
            if calibration.get('status') == 'failed':
                self._pause_with_error(
                    code='E_OUTDOOR_CALIBRATION_ERROR',
                    description=calibration.get('message', 'outdoor calibration failed'),
                    suggested_action='check_gps_or_relocalize',
                )
                return
            self._runtime.event = (
                'outdoor_calibration_checked'
                if calibration.get('status') == 'passed' else
                'outdoor_calibration_observing'
            )

        if str(point.kind or '').strip().lower() == 'transit':
            # The controller intentionally does not send zero speed for a
            # transit point.  Dispatch the next segment in the same control
            # callback so the bottom layer receives a continuous command
            # stream instead of waiting for the slower mission timer.
            self._dispatch_next_outdoor_point_if_needed()
            return

        self._publish_task_progress()
        self._write_state_fields()

    def _record_active_outdoor_goal(
        self,
        *,
        status: str,
        backend_event: Optional[Dict[str, Any]],
    ):
        mission = self._outdoor_mission
        if (
            mission is None or
            self._active_outdoor_point_index is None or
            self._active_outdoor_goal_resolution is None or
            self._active_outdoor_candidate_index is None
        ):
            return

        point = mission.execution_points[self._active_outdoor_point_index]
        candidate = self._active_outdoor_goal_resolution.candidate(
            self._active_outdoor_candidate_index
        )
        record = {
            'timestamp_ns': self._now_ns_getter(),
            'status': str(status),
            'point': point.to_dict(),
            'candidate': candidate.to_dict() if candidate is not None else None,
            'candidate_index': int(self._active_outdoor_candidate_index),
            'candidate_count': len(self._active_outdoor_goal_resolution.candidates),
            'resolution': {
                'strategy': self._active_outdoor_goal_resolution.strategy,
                'validator_status': self._active_outdoor_goal_resolution.validator_status,
            },
            'backend_event': dict(backend_event or {}),
        }
        self._outdoor_goal_records.append(record)

    def _record_outdoor_calibration(self, point) -> Dict[str, Any]:
        mission = self._outdoor_mission
        tolerance_m = float(point.tolerance_m or (mission.position_tolerance_m if mission else 10.0))
        gps_position = self._pose_registry.gps_position()
        visual_pose = self._pose_registry.latest_visual_pose()
        record = {
            'seq': point.seq,
            'kind': point.kind,
            'nav_point_id': point.nav_point_id,
            'name': point.name,
            'target': point.to_dict(),
            'gps_position': gps_position,
            'visual_pose': visual_pose or {},
            'tolerance_m': tolerance_m,
            'timestamp_ns': self._now_ns_getter(),
            'status': 'skipped',
            'message': '',
        }

        if point.lat is None or point.lon is None:
            record['message'] = 'target nav point has no lat/lon; map-frame navigation accepted'
            self._outdoor_calibration_records.append(record)
            return record

        if not gps_position or 'lat' not in gps_position or 'lon' not in gps_position:
            record['status'] = 'pending_gps'
            record['message'] = 'gps position is unavailable at calibration point'
            self._outdoor_calibration_records.append(record)
            return record

        error_m = self._latlon_distance_m(
            float(point.lat),
            float(point.lon),
            float(gps_position['lat']),
            float(gps_position['lon']),
        )
        record['error_m'] = error_m
        record['gps_fix_type'] = self._pose_registry.gps_fix_type()
        record['gps_num_sv'] = self._pose_registry.gps_num_sv()

        gps_delta_from_prev_m = None
        visual_delta_from_prev_m = None
        if self._outdoor_calibration_records:
            prev = self._outdoor_calibration_records[-1].get('gps_position') or {}
            if 'lat' in prev and 'lon' in prev:
                gps_delta_from_prev_m = self._latlon_distance_m(
                    float(prev['lat']),
                    float(prev['lon']),
                    float(gps_position['lat']),
                    float(gps_position['lon']),
                )
                record['gps_delta_from_prev_m'] = gps_delta_from_prev_m
            prev_visual = self._outdoor_calibration_records[-1].get('visual_pose') or {}
            if visual_pose is not None and 'x' in prev_visual and 'y' in prev_visual:
                visual_delta_from_prev_m = self._xy_distance_m(
                    float(prev_visual['x']),
                    float(prev_visual['y']),
                    float(visual_pose['x']),
                    float(visual_pose['y']),
                )
                record['visual_delta_from_prev_m'] = visual_delta_from_prev_m

        gate = self._outdoor_gps_vo_gate.evaluate(
            gps_error_m=error_m,
            tolerance_m=tolerance_m,
            jump_reject_m=mission.gps_jump_reject_m if mission is not None else 15.0,
            gps_delta_from_prev_m=gps_delta_from_prev_m,
            visual_delta_from_prev_m=visual_delta_from_prev_m,
            gps_fix_type=self._pose_registry.gps_fix_type(),
            gps_num_sv=self._pose_registry.gps_num_sv(),
            min_fix_type=mission.min_gps_fix_type if mission is not None else 2,
        )
        record['gate'] = gate
        alignment = self._record_outdoor_pose_alignment(
            point=point,
            gps_position=gps_position,
            visual_pose=visual_pose,
        )
        if alignment:
            record['alignment'] = alignment
        record['status'] = gate.get('status', 'pending')
        record['message'] = gate.get('reason', '')
        if record['status'] == 'passed':
            record['message'] = f'gps error {error_m:.2f}m within tolerance {tolerance_m:.2f}m'
        elif record['status'] == 'failed':
            record['message'] = f'gps error {error_m:.2f}m exceeds tolerance {tolerance_m:.2f}m'

        self._outdoor_calibration_records.append(record)
        return record

    def _record_outdoor_pose_alignment(
        self,
        *,
        point,
        gps_position: Dict[str, Any],
        visual_pose: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        if point.lat is None or point.lon is None:
            return {}
        if not gps_position or 'lat' not in gps_position or 'lon' not in gps_position:
            return {}

        try:
            alignment = self._outdoor_pose_aligner.evaluate(
                projector=self._projector,
                target_lat=float(point.lat),
                target_lon=float(point.lon),
                target_x=float(point.x),
                target_y=float(point.y),
                gps_position=gps_position,
                map_pose=self._pose_registry.latest_map_pose(),
                visual_pose=visual_pose,
                gps_fix_type=self._pose_registry.gps_fix_type(),
                gps_num_sv=self._pose_registry.gps_num_sv(),
                timestamp_ns=self._now_ns_getter(),
            )
        except Exception as exc:
            alignment = {
                'status': 'error',
                'message': f'failed to evaluate outdoor pose alignment: {exc}',
                'stable': False,
            }
        return alignment

    def _xy_distance_m(self, x_a: float, y_a: float, x_b: float, y_b: float) -> float:
        return math.hypot(float(x_b) - float(x_a), float(y_b) - float(y_a))

    def _latlon_distance_m(self, lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
        lat_a_rad = math.radians(lat_a)
        lat_b_rad = math.radians(lat_b)
        dlat = lat_b_rad - lat_a_rad
        dlon = math.radians(lon_b - lon_a)
        a = (
            math.sin(dlat / 2.0) ** 2 +
            math.cos(lat_a_rad) * math.cos(lat_b_rad) * math.sin(dlon / 2.0) ** 2
        )
        return 2.0 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))

    def _handle_backend_event(self, backend_event: Dict[str, Any]):
        if self._task is None:
            return

        state = backend_event.get('state')
        if state == 'succeeded':
            self._handle_waypoint_success()
            return

        if state in {'stopped', 'canceled'}:
            if self._runtime.stage == 'paused':
                self._runtime.event = 'navigation_stopped'
                self._publish_task_progress()
            return

        self._pause_with_error(
            code='E_MOVE_FAIL',
            description=(
                'legacy navigation backend returned terminal state '
                f'{state}, feedback={backend_event.get("feedback_msg", "")}'
            ),
            suggested_action='check_local_velocity_navigation_or_disable_legacy_backend',
        )

    def _handle_waypoint_success(self):
        if self._task is None or self._active_waypoint_index is None:
            return

        waypoint = self._task.waypoints[self._active_waypoint_index]
        self._runtime.current_waypoint_index = self._active_waypoint_index
        self._runtime.current_waypoint_seq = waypoint.seq
        self._runtime.progress_percent = (
            100.0 * (self._runtime.current_waypoint_index + 1) / max(1, len(self._task.waypoints))
        )
        self._active_waypoint_index = None
        self._active_local_goal_kind = ''
        self._runtime.event = 'waypoint_reached'
        self._runtime.error = None
        self._runtime.stage = 'executing'
        self._runtime.status = 'normal'
        if waypoint.hover_time_s > 0.0:
            self._hover_until = time.monotonic() + waypoint.hover_time_s
        else:
            self._action_runner.queue(waypoint, task_id=self._runtime.task_id)
        self._publish_task_progress()
        self._write_state_fields()

    def _complete_task(self):
        self._stop_straight_drive_if_needed(reason='task_completed')
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._active_local_goal_kind = ''
        self._active_waypoint_index = None
        self._pending_first_inspection_index = None
        self._pending_home_calibration_index = None
        self._active_home_calibration_index = None
        self._initial_yaw_calibration_active = False
        self._gps_vo_yaw_aligner.reset()
        self._straight_drive_task = None
        self._paused_local_context = {}
        self._runtime.stage = 'completed'
        self._runtime.status = 'normal'
        self._runtime.progress_percent = 100.0
        self._runtime.event = 'task_completed'
        self._runtime.error = None
        self._action_runner.reset()
        self._release_motion_control_lock(reason='task_completed')
        self._stop_recording_if_needed()
        self._set_map_pose_reporting_enabled(False)
        self._publish_task_progress()
        self._write_state_fields()

    def _abort_task(self, *, code: str, description: str, suggested_action: str):
        self._stop_straight_drive_if_needed(reason=code)
        self._stop_backend_navigation_if_needed()
        self._publish_move_stop(reason='task_aborted')
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._active_local_goal_kind = ''
        self._pending_first_inspection_index = None
        self._pending_home_calibration_index = None
        self._active_home_calibration_index = None
        self._initial_yaw_calibration_active = False
        self._gps_vo_yaw_aligner.reset()
        self._straight_drive_task = None
        self._paused_local_context = {}
        self._hover_until = None
        self._action_runner.reset()
        self._runtime.stage = 'aborted'
        self._runtime.status = 'exception'
        self._runtime.event = 'task_aborted'
        self._runtime.error = self._make_error(
            code=code,
            description=description,
            suggested_action=suggested_action,
        )
        self._release_motion_control_lock(reason=code)
        self._stop_recording_if_needed()
        self._set_map_pose_reporting_enabled(False)
        self._publish_task_progress()
        self._write_state_fields()

    def _pause_with_error(self, *, code: str, description: str, suggested_action: str):
        self._capture_paused_local_context()
        self._stop_straight_drive_if_needed(reason=code)
        self._stop_backend_navigation_if_needed()
        self._publish_move_stop(reason=code)
        self._active_waypoint_index = None
        self._active_outdoor_point_index = None
        self._active_outdoor_goal_resolution = None
        self._active_outdoor_candidate_index = None
        self._active_local_goal_kind = ''
        self._active_home_calibration_index = None
        self._initial_yaw_calibration_active = False
        self._hover_until = None
        self._action_runner.reset()
        self._runtime.stage = 'paused'
        self._runtime.status = 'exception'
        self._runtime.event = 'task_paused'
        self._runtime.error = self._make_error(
            code=code,
            description=description,
            suggested_action=suggested_action,
        )
        self._release_motion_control_lock(reason=code)
        self._publish_task_progress()
        self._write_state_fields()

    def _capture_paused_local_context(self):
        if self._initial_yaw_calibration_active:
            self._paused_local_context = {'kind': 'initial_yaw_calibration'}
        elif self._active_home_calibration_index is not None:
            self._paused_local_context = {
                'kind': 'home_calibration',
                'index': int(self._active_home_calibration_index),
            }
        elif self._active_outdoor_point_index is not None:
            self._paused_local_context = {
                'kind': 'outdoor',
                'index': int(self._active_outdoor_point_index),
            }
        elif self._active_waypoint_index is not None:
            self._paused_local_context = {
                'kind': 'waypoint',
                'index': int(self._active_waypoint_index),
            }
        elif self._straight_drive_task is not None:
            context = {'kind': 'straight_drive'}
            if self._straight_drive_controller is not None:
                try:
                    control = self._straight_drive_controller.snapshot()
                except Exception:
                    control = {}
                target_distance = control.get('target_distance_m')
                if target_distance is not None:
                    context['remaining_distance_m'] = max(
                        0.0,
                        float(target_distance) - float(control.get('distance_traveled_m', 0.0)),
                    )
                target_duration = control.get('target_duration_s')
                if target_duration is not None:
                    context['remaining_duration_s'] = max(
                        0.0,
                        float(target_duration) - float(control.get('elapsed_s', 0.0)),
                    )
            self._paused_local_context = context

    def _stop_backend_navigation_if_needed(self):
        if self._task is None and self._outdoor_mission is None:
            return
        if not self._dispatcher.has_active_goal():
            return
        map_name = self._task.map_name if self._task is not None else self._outdoor_mission.map_name
        self._dispatcher.stop_all_tasks(map_name=map_name)

    def _stop_straight_drive_if_needed(self, *, reason: str):
        if self._straight_drive_controller is None:
            return
        if not self._straight_drive_controller.is_active():
            return
        try:
            self._straight_drive_controller.stop(reason=reason)
        except Exception as exc:
            self._logger.warning(f'failed to stop straight drive controller: {exc}')

    def _publish_move_stop(self, *, reason: str):
        if self._move_gateway is None:
            return
        try:
            self._move_gateway.stop(reason=reason)
        except Exception as exc:
            self._logger.warning(f'failed to publish uran_move stop action: {exc}')

    def _publish_task_action_event(self, action_name: str, payload: Dict[str, Any]):
        self._publish_uplink(
            data_type='task_action_result',
            payload=payload,
            urgent=not bool(payload.get('success', False)),
        )

    def _start_recording_if_needed(self):
        if self._task is None or not self._task.media_record or self._recording_active:
            return
        channel_id = str(self._media_actions_cfg.get('record_channel_id', 'cyberdog_main'))
        self._recording_active = True
        self._publish_media_control(
            action='record_start',
            channel_id=channel_id,
        )

    def _stop_recording_if_needed(self):
        if not self._recording_active:
            return
        self._recording_active = False
        channel_id = str(self._media_actions_cfg.get('record_channel_id', 'cyberdog_main'))
        self._publish_media_control(
            action='record_stop',
            channel_id=channel_id,
        )

    def _set_runtime_event(self, event: str, *, publish_now: bool):
        self._runtime.event = event
        if publish_now:
            self._publish_task_progress()
            self._write_state_fields()

    def _make_error(
        self,
        *,
        code: str,
        description: str,
        suggested_action: str,
        severity: str = 'warning',
    ) -> Dict[str, Any]:
        return {
            'code': code,
            'severity': severity,
            'description': description,
            'suggested_action': suggested_action,
        }

    def _publish_periodic_progress(self, *, monotonic_s: float):
        if monotonic_s - self._last_periodic_report_ts < self._progress_report_interval_s:
            return
        self._publish_task_progress()

    def _publish_task_progress(self):
        self._publish_uplink(
            data_type='task_progress',
            payload=self.build_status_payload(),
            urgent=(
                self._runtime.stage in TERMINAL_TASK_STAGES or
                self._runtime.status == 'exception'
            ),
        )
        self._last_periodic_report_ts = time.monotonic()

    def _write_state_fields(self):
        task_id = self._runtime.task_id if self._runtime.task_id else ''
        self._write_state('task_id', task_id)
        self._write_state('task_stage', self._runtime.stage)

    def _acquire_motion_control_lock(self, *, task_id: str, reason: str):
        self._motion_control_locked = True
        self._write_motion_control_lock(
            active=True,
            task_id=task_id,
            reason=reason,
        )

    def _release_motion_control_lock(self, *, reason: str):
        if not self._motion_control_locked:
            return
        self._motion_control_locked = False
        self._write_motion_control_lock(
            active=False,
            task_id=self._runtime.task_id,
            reason=reason,
        )
