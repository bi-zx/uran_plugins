from typing import Any, Dict, Optional


class WaypointDispatcher:
    def __init__(self, *, logger, backend, projector, stamp_getter):
        self._logger = logger
        self._backend = backend
        self._projector = projector
        self._stamp_getter = stamp_getter

    def has_active_goal(self) -> bool:
        if self._backend is None:
            return False
        return self._backend.has_active_goal()

    def backend_snapshot(self) -> Dict[str, Any]:
        if self._backend is None:
            return {}
        return self._backend.get_execution_snapshot()

    def stop_all_tasks(self, *, map_name: str = '') -> bool:
        if self._backend is None:
            return False
        return self._backend.stop_all_tasks(map_name=map_name)

    def dispatch_task_waypoint(self, *, task, waypoint) -> Dict[str, Any]:
        if not self._projector.is_ready():
            return {
                'success': False,
                'error_code': 'E_GEOREF_NOT_READY',
                'description': 'geo reference is not ready while dispatching waypoint',
                'suggested_action': 'configure_geo_reference',
            }
        if self._backend is None:
            return {
                'success': False,
                'error_code': 'E_NAV_BACKEND_UNAVAILABLE',
                'description': 'navigation backend is unavailable',
                'suggested_action': 'configure_navigation_backend',
            }

        pose = self._projector.waypoint_to_pose(
            waypoint,
            timestamp_ns=int(self._stamp_getter()),
            heading_mode=task.heading_mode,
        )
        if not self._backend.navigate_to_pose(
            pose,
            map_name=task.map_name,
            outdoor=task.outdoor,
        ):
            return {
                'success': False,
                'error_code': 'E_MOVE_FAIL',
                'description': 'failed to dispatch waypoint to navigation backend',
                'suggested_action': 'check_navigation_backend',
            }

        return {
            'success': True,
            'pose': pose,
        }

    def dispatch_map_goal(
        self,
        *,
        map_x: float,
        map_y: float,
        map_name: str = '',
        outdoor: bool = False,
        frame_id: str = 'map',
        yaw_deg: Optional[float] = None,
    ) -> Dict[str, Any]:
        if self._backend is None:
            return {
                'success': False,
                'error_code': 'E_NAV_BACKEND_UNAVAILABLE',
                'description': 'navigation backend is unavailable',
                'suggested_action': 'configure_navigation_backend',
            }

        pose = {
            'timestamp_ns': int(self._stamp_getter()),
            'frame_id': str(frame_id or 'map'),
            'x': float(map_x),
            'y': float(map_y),
            'z': 0.0,
            'yaw_deg': 0.0 if yaw_deg is None else float(yaw_deg),
        }

        if not self._backend.navigate_to_pose(
            pose,
            map_name=map_name or '',
            outdoor=bool(outdoor),
        ):
            return {
                'success': False,
                'error_code': 'E_MOVE_FAIL',
                'description': 'failed to dispatch map goal to navigation backend',
                'suggested_action': 'check_navigation_backend',
            }

        return {
            'success': True,
            'pose': pose,
        }

    def send_raw_navigation_goal(
        self,
        *,
        nav_type: int,
        poses=None,
        map_name: str = '',
        outdoor: bool = False,
    ) -> Dict[str, Any]:
        if self._backend is None:
            return {
                'success': False,
                'error_code': 'E_NAV_BACKEND_UNAVAILABLE',
                'description': 'navigation backend is unavailable',
                'suggested_action': 'configure_navigation_backend',
            }
        if not self._backend.send_raw_navigation_goal(
            nav_type=int(nav_type),
            poses=poses or [],
            map_name=map_name or '',
            outdoor=bool(outdoor),
        ):
            return {
                'success': False,
                'error_code': 'E_MOVE_FAIL',
                'description': 'failed to dispatch raw navigation goal to navigation backend',
                'suggested_action': 'check_navigation_backend',
            }
        return {'success': True}
