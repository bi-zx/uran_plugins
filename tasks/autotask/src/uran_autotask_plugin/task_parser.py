import json
from typing import Any, Dict, Iterable, List, Optional

from .task_models import GeoWaypoint, MissionTask


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y', 'on'}
    return bool(value)


def _as_float(value: Any, default: float = 0.0) -> float:
    if value in (None, ''):
        return default
    return float(value)


def _as_int(value: Any, default: int = 0) -> int:
    if value in (None, ''):
        return default
    return int(value)


def _as_list_of_strings(values: Optional[Iterable[Any]]) -> List[str]:
    if values is None:
        return []
    result: List[str] = []
    for value in values:
        if value is None:
            continue
        result.append(str(value))
    return result


def parse_task_definition(
    *,
    task_id: str,
    task_type: str,
    task_params_json: str,
    defaults: Optional[Dict[str, Any]] = None,
) -> MissionTask:
    defaults = defaults or {}
    payload = json.loads(task_params_json or '{}')
    if not isinstance(payload, dict):
        raise ValueError('task_params_json must decode to a JSON object')

    normalized_task_type = str(payload.get('task_type') or task_type or 'waypoint')
    if normalized_task_type == 'custom' and isinstance(payload.get('waypoints'), list):
        normalized_task_type = 'waypoint'
    if normalized_task_type not in {'waypoint', 'patrol'}:
        raise ValueError(f'unsupported task_type: {normalized_task_type}')

    raw_waypoints = payload.get('waypoints')
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError('waypoint/patrol task requires a non-empty waypoints array')

    parsed_waypoints: List[GeoWaypoint] = []
    for index, raw_waypoint in enumerate(raw_waypoints):
        if not isinstance(raw_waypoint, dict):
            raise ValueError(f'waypoint #{index} must be an object')
        parsed_waypoints.append(
            GeoWaypoint(
                seq=_as_int(raw_waypoint.get('seq'), index),
                lat=_as_float(raw_waypoint.get('lat')),
                lon=_as_float(raw_waypoint.get('lon')),
                alt=_as_float(raw_waypoint.get('alt'), 0.0),
                heading_deg=(
                    None if raw_waypoint.get('heading_deg') in (None, '')
                    else _as_float(raw_waypoint.get('heading_deg'))
                ),
                speed_mps=_as_float(raw_waypoint.get('speed_mps'), 0.5),
                hover_time_s=_as_float(raw_waypoint.get('hover_time_s'), 0.0),
                actions=_as_list_of_strings(raw_waypoint.get('actions')),
            )
        )
    parsed_waypoints.sort(key=lambda item: item.seq)

    known_keys = {
        'task_id',
        'task_type',
        'map_name',
        'outdoor',
        'localization_source',
        'loop_closure_source',
        'heading_mode',
        'waypoints',
        'abort_on_low_battery',
        'low_battery_threshold',
        'abort_action',
        'media_record',
        'sensor_report_interval_ms',
    }
    extra = {key: value for key, value in payload.items() if key not in known_keys}

    return MissionTask(
        task_id=str(payload.get('task_id') or task_id),
        task_type=normalized_task_type,
        map_name=str(payload.get('map_name') or defaults.get('map_name', '')),
        outdoor=_as_bool(payload.get('outdoor'), _as_bool(defaults.get('outdoor'), False)),
        localization_source=str(
            payload.get('localization_source') or defaults.get('localization_source', 'laser')
        ),
        loop_closure_source=str(
            payload.get('loop_closure_source') or defaults.get('loop_closure_source', 'gps')
        ),
        heading_mode=str(payload.get('heading_mode') or defaults.get('heading_mode', 'true_north_deg')),
        abort_on_low_battery=_as_bool(
            payload.get('abort_on_low_battery'),
            _as_bool(defaults.get('abort_on_low_battery'), False),
        ),
        low_battery_threshold=_as_float(
            payload.get('low_battery_threshold'),
            _as_float(defaults.get('low_battery_threshold'), 20.0),
        ),
        abort_action=str(payload.get('abort_action') or defaults.get('abort_action', 'stop')),
        media_record=_as_bool(
            payload.get('media_record'),
            _as_bool(defaults.get('media_record'), False),
        ),
        sensor_report_interval_ms=_as_int(
            payload.get('sensor_report_interval_ms'),
            _as_int(defaults.get('sensor_report_interval_ms'), 2000),
        ),
        waypoints=parsed_waypoints,
        extra=extra,
    )
