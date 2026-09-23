import json
from typing import Any, Dict, Optional

from .mission_contract import OutdoorMissionPlan
from .route_sampler import build_execution_points


OUTDOOR_TASK_TYPES = {'outdoor_route', 'outdoor_patrol', 'mission_planner_route'}


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y', 'on'}
    return bool(value)


def _as_float(value: Any, default: float) -> float:
    if value in (None, ''):
        return default
    return float(value)


def _load_payload(task_params_json: str) -> Dict[str, Any]:
    payload = json.loads(task_params_json or '{}')
    if not isinstance(payload, dict):
        raise ValueError('task_params_json must decode to a JSON object')
    return payload


def _planner_payload(payload: Dict[str, Any]) -> Dict[str, Any]:
    for key in ('planner_result', 'plan_result', 'mission_plan'):
        candidate = payload.get(key)
        if isinstance(candidate, dict):
            return candidate
    return payload


def _looks_like_route_payload(payload: Dict[str, Any]) -> bool:
    if any(key in payload for key in ('planner_result', 'plan_result', 'mission_plan')):
        return True
    route = payload.get('route')
    if isinstance(route, dict):
        if isinstance(route.get('points'), list):
            return True
        if any(key in route for key in ('route_nav_points', 'display_route_local_m', 'legs')):
            return True
    if all(key in payload for key in ('projection_origin', 'home_pose', 'start_pose')):
        return True
    if any(key in payload for key in ('route_nav_points', 'display_route_local_m', 'legs')):
        return True
    return False


def _select_robot_route(plan: Dict[str, Any], robot_id: str = '') -> Dict[str, Any]:
    route = plan.get('route')
    if isinstance(route, dict):
        return route

    robots = plan.get('robots')
    if not isinstance(robots, list) or not robots:
        if any(key in plan for key in ('route_nav_points', 'display_route_local_m', 'legs')):
            return plan
        raise ValueError('outdoor mission requires route or robots[] in planner result')

    normalized_robot_id = str(robot_id or '').strip()
    if normalized_robot_id:
        for robot in robots:
            if not isinstance(robot, dict):
                continue
            aliases = {
                str(robot.get('robot_id') or ''),
                str(robot.get('hardware_id') or ''),
                str(robot.get('planning_slot_id') or ''),
            }
            if normalized_robot_id in aliases:
                return robot

    if len(robots) == 1:
        return robots[0]

    raise ValueError('multiple robot routes found; robot_id/hardware_id/planning_slot_id is required')


def _projection_origin(payload: Dict[str, Any], plan: Dict[str, Any]) -> Dict[str, Any]:
    coordinate_system = payload.get('coordinate_system')
    if isinstance(coordinate_system, dict):
        local_system = coordinate_system.get('local')
        if isinstance(local_system, dict) and isinstance(local_system.get('projection_origin'), dict):
            return dict(local_system['projection_origin'])
        if isinstance(coordinate_system.get('projection_origin'), dict):
            return dict(coordinate_system['projection_origin'])
    for candidate in (
        payload.get('projection_origin'),
        plan.get('projection_origin'),
        plan.get('metadata', {}).get('projection_origin') if isinstance(plan.get('metadata'), dict) else None,
    ):
        if isinstance(candidate, dict):
            return dict(candidate)
    return {}


def _planner_summary_value(payload: Dict[str, Any], key: str, default: float = 0.0) -> Any:
    planner_summary = payload.get('planner_summary')
    if isinstance(planner_summary, dict) and planner_summary.get(key) not in (None, ''):
        return planner_summary.get(key)
    return default


def is_outdoor_task_payload(*, task_type: str, task_params_json: str) -> bool:
    normalized_type = str(task_type or '').strip()
    if normalized_type in OUTDOOR_TASK_TYPES:
        return True
    try:
        payload = _load_payload(task_params_json)
    except Exception:
        return False
    payload_type = str(payload.get('task_type') or '').strip()
    if payload_type in OUTDOOR_TASK_TYPES:
        return True
    return _looks_like_route_payload(payload)


def parse_outdoor_task_definition(
    *,
    task_id: str,
    task_type: str,
    task_params_json: str,
    defaults: Optional[Dict[str, Any]] = None,
) -> OutdoorMissionPlan:
    defaults = defaults or {}
    payload = _load_payload(task_params_json)
    plan = _planner_payload(payload)
    execution_cfg = dict(payload.get('execution') or {})
    robot_meta = payload.get('robot') if isinstance(payload.get('robot'), dict) else {}

    robot_id = str(
        payload.get('robot_id') or
        payload.get('hardware_id') or
        payload.get('planning_slot_id') or
        robot_meta.get('hardware_id') or
        robot_meta.get('planning_slot_id') or
        defaults.get('robot_id', '')
    )
    route = _select_robot_route(plan, robot_id=robot_id)
    if not robot_id:
        robot_id = str(route.get('hardware_id') or route.get('planning_slot_id') or route.get('robot_id') or '')

    position_tolerance_m = _as_float(
        execution_cfg.get('position_tolerance_m'),
        _as_float(payload.get('position_tolerance_m'), 10.0),
    )
    route_nav_point_ids = route.get('route_nav_point_ids') or payload.get('route_nav_point_ids') or []
    route_nav_points = route.get('route_nav_points') or payload.get('route_nav_points') or []

    route_payload = dict(route)
    if route_nav_point_ids:
        route_payload['route_nav_point_ids'] = list(route_nav_point_ids)
    if route_nav_points:
        route_payload['route_nav_points'] = list(route_nav_points)

    execution_points = build_execution_points(
        route_payload,
        position_tolerance_m=position_tolerance_m,
        min_segment_m=_as_float(execution_cfg.get('min_segment_m'), 2.0),
        max_segment_m=_as_float(execution_cfg.get('max_segment_m'), 12.0),
        calibration_snap_radius_m=_as_float(execution_cfg.get('calibration_snap_radius_m'), 12.0),
    )
    if not execution_points:
        raise ValueError('outdoor route produced no execution points')

    scene_name = str(payload.get('scene_name') or plan.get('scene_name') or payload.get('scene') or '')
    planner_result_id = str(
        payload.get('planner_result_id') or
        payload.get('plan_id') or
        plan.get('plan_id') or
        ''
    )
    coordinate_system = dict(payload.get('coordinate_system') or plan.get('coordinate_system') or {})

    return OutdoorMissionPlan(
        task_id=str(payload.get('task_id') or task_id),
        task_type=str(payload.get('task_type') or task_type or 'outdoor_route'),
        scene_name=scene_name,
        planner_result_id=planner_result_id,
        robot_id=robot_id,
        map_name=str(payload.get('map_name') or defaults.get('map_name', '')),
        frame_id=str(payload.get('frame_id') or defaults.get('frame_id', 'map')),
        coordinate_system=coordinate_system,
        projection_origin=_projection_origin(payload, plan),
        route_nav_point_ids=[str(item) for item in route_nav_point_ids],
        route_nav_points=list(route_nav_points) if isinstance(route_nav_points, list) else [],
        execution_points=execution_points,
        position_tolerance_m=position_tolerance_m,
        gps_jump_reject_m=_as_float(execution_cfg.get('gps_jump_reject_m'), 15.0),
        gps_vo_blend_window_s=_as_float(execution_cfg.get('gps_vo_blend_window_s'), 3.0),
        stable_offset_required_count=int(_as_float(execution_cfg.get('stable_offset_required_count'), 2)),
        min_gps_fix_type=int(_as_float(execution_cfg.get('min_gps_fix_type'), 2)),
        calibrate_at_nav_points=_as_bool(execution_cfg.get('calibrate_at_nav_points'), True),
        raw_route_summary={
            'planning_slot_id': route.get('planning_slot_id', robot_meta.get('planning_slot_id', '')),
            'hardware_id': route.get('hardware_id', robot_meta.get('hardware_id', '')),
            'total_distance_with_home_m': route.get('total_distance_with_home_m') or _planner_summary_value(
                payload,
                'total_distance_with_home_m',
                0.0,
            ),
            'estimated_time_with_home_s': route.get('estimated_time_with_home_s') or _planner_summary_value(
                payload,
                'estimated_time_with_home_s',
                0.0,
            ),
            'leg_count': len(route.get('legs') or []),
        },
    )
