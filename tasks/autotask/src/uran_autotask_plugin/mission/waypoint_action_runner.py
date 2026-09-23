from typing import Any, Dict, Optional


class WaypointActionRunner:
    def __init__(
        self,
        *,
        camera_capture,
        backend_snapshot_getter,
        battery_level_getter,
        position_getter,
        gps_monitor_state_getter,
        now_ns_getter,
        publish_event,
        set_runtime_event,
    ):
        self._camera_capture = camera_capture
        self._backend_snapshot_getter = backend_snapshot_getter
        self._battery_level_getter = battery_level_getter
        self._position_getter = position_getter
        self._gps_monitor_state_getter = gps_monitor_state_getter
        self._now_ns_getter = now_ns_getter
        self._publish_event = publish_event
        self._set_runtime_event = set_runtime_event

        self._pending_waypoint = None
        self._pending_waypoint_actions = []
        self._pending_action_name: Optional[str] = None
        self._task_id = ''

    def reset(self):
        self._pending_waypoint = None
        self._pending_waypoint_actions = []
        self._pending_action_name = None
        self._task_id = ''
        if self._camera_capture is not None:
            self._camera_capture.cancel_pending()

    def has_pending(self) -> bool:
        return self._pending_waypoint is not None

    def queue(self, waypoint, *, task_id: str):
        if not getattr(waypoint, 'actions', None):
            return
        self._pending_waypoint = waypoint
        self._pending_waypoint_actions = list(waypoint.actions)
        self._pending_action_name = None
        self._task_id = task_id
        self._set_runtime_event('waypoint_actions_started', publish_now=True)

    def tick(self) -> bool:
        waypoint = self._pending_waypoint
        if waypoint is None:
            return True

        while True:
            if self._pending_action_name == 'capture_image':
                result = self._poll_capture_image_action(waypoint)
                if result is None:
                    return False
                self._publish_action_event('capture_image', result)
                self._pending_action_name = None
                continue

            if not self._pending_waypoint_actions:
                self._pending_waypoint = None
                self._set_runtime_event('waypoint_actions_finished', publish_now=True)
                return True

            action_key = str(self._pending_waypoint_actions.pop(0)).strip().lower()
            if action_key == 'capture_image':
                self._pending_action_name = action_key
                result = self._start_capture_image_action(waypoint)
                if result is None:
                    return False
                self._publish_action_event('capture_image', result)
                self._pending_action_name = None
                continue

            if action_key == 'sensor_report':
                result = self._run_sensor_report_action(waypoint)
                self._publish_action_event('sensor_report', result)
                continue

            self._publish_action_event(
                action_key,
                {
                    'success': False,
                    'message': f'unsupported waypoint action: {action_key}',
                    'action': action_key,
                    'waypoint_seq': waypoint.seq,
                },
            )

    def _start_capture_image_action(self, waypoint) -> Optional[Dict[str, Any]]:
        if self._camera_capture is None:
            return {
                'success': False,
                'action': 'capture_image',
                'waypoint_seq': waypoint.seq,
                'message': 'camera capture adapter is unavailable',
            }

        started = self._camera_capture.start_capture()
        if not started.get('accepted', False):
            result = dict(started['result'])
            result['action'] = 'capture_image'
            result['waypoint_seq'] = waypoint.seq
            return result

        self._set_runtime_event('capture_image_started', publish_now=True)
        return self._poll_capture_image_action(waypoint)

    def _poll_capture_image_action(self, waypoint) -> Optional[Dict[str, Any]]:
        if self._camera_capture is None:
            return {
                'success': False,
                'action': 'capture_image',
                'waypoint_seq': waypoint.seq,
                'message': 'camera capture adapter is unavailable',
            }

        result = self._camera_capture.poll_capture()
        if result is None:
            self._set_runtime_event('capture_image_pending', publish_now=False)
            return None
        result['action'] = 'capture_image'
        result['waypoint_seq'] = waypoint.seq
        return result

    def _run_sensor_report_action(self, waypoint) -> Dict[str, Any]:
        return {
            'success': True,
            'action': 'sensor_report',
            'waypoint_seq': waypoint.seq,
            'battery_level': self._battery_level_getter(),
            'position': self._position_getter(),
            'gps_monitor': self._gps_monitor_state_getter(),
            'backend_state': self._backend_snapshot_getter(),
            'timestamp_ns': self._now_ns_getter(),
        }

    def _publish_action_event(self, action_name: str, payload: Dict[str, Any]):
        event_payload = dict(payload)
        event_payload['task_id'] = self._task_id
        event_payload['timestamp_ns'] = self._now_ns_getter()
        self._publish_event(action_name, event_payload)
