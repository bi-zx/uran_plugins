import math
from typing import Any, Dict, List, Sequence

from .mission_contract import OutdoorExecutionPoint


def _as_float(value: Any, default: float = 0.0) -> float:
    if value in (None, ''):
        return default
    return float(value)


def _distance(a: Dict[str, float], b: Dict[str, float]) -> float:
    return math.hypot(float(a['x']) - float(b['x']), float(a['y']) - float(b['y']))


def _dedupe(points: Sequence[Dict[str, float]], *, epsilon: float = 1e-6) -> List[Dict[str, float]]:
    deduped: List[Dict[str, float]] = []
    for point in points:
        normalized = {'x': float(point['x']), 'y': float(point['y'])}
        if deduped and _distance(deduped[-1], normalized) <= epsilon:
            continue
        deduped.append(normalized)
    return deduped


def _resample_polyline(
    points: Sequence[Dict[str, float]],
    *,
    min_segment_m: float,
    max_segment_m: float,
) -> List[Dict[str, float]]:
    points = _dedupe(points)
    if len(points) <= 1:
        return list(points)

    sampled = [points[0]]
    for start, end in zip(points, points[1:]):
        distance_m = _distance(start, end)
        if distance_m <= min_segment_m:
            continue
        split_count = max(1, int(math.ceil(distance_m / max_segment_m)))
        for index in range(1, split_count + 1):
            ratio = index / split_count
            sampled.append({
                'x': start['x'] + (end['x'] - start['x']) * ratio,
                'y': start['y'] + (end['y'] - start['y']) * ratio,
            })
    return _dedupe(sampled)


def _polyline_from_legs(legs: Sequence[Dict[str, Any]]) -> List[Dict[str, float]]:
    merged: List[Dict[str, float]] = []
    for leg in legs:
        raw_points = leg.get('polyline_local_m') or []
        if not isinstance(raw_points, list):
            continue
        for raw in raw_points:
            if not isinstance(raw, dict):
                continue
            if 'x' not in raw or 'z' not in raw:
                continue
            merged.append({'x': _as_float(raw.get('x')), 'y': _as_float(raw.get('z'))})
    return _dedupe(merged)


def _polyline_from_route(route: Dict[str, Any]) -> List[Dict[str, float]]:
    raw_points = route.get('display_route_local_m') or route.get('polyline_local_m') or []
    points: List[Dict[str, float]] = []
    if isinstance(raw_points, list):
        for raw in raw_points:
            if not isinstance(raw, dict):
                continue
            if 'x' not in raw or 'z' not in raw:
                continue
            points.append({'x': _as_float(raw.get('x')), 'y': _as_float(raw.get('z'))})
    if points:
        return _dedupe(points)
    return _polyline_from_legs(route.get('legs') or [])


def _explicit_route_points(route: Dict[str, Any]) -> List[OutdoorExecutionPoint]:
    raw_points = route.get('points') or []
    if not isinstance(raw_points, list):
        return []

    points: List[OutdoorExecutionPoint] = []
    for raw_index, raw in enumerate(raw_points):
        if not isinstance(raw, dict):
            continue
        local = raw.get('local') if isinstance(raw.get('local'), dict) else {}
        map_pose = raw.get('map') if isinstance(raw.get('map'), dict) else {}
        geo = raw.get('geo') if isinstance(raw.get('geo'), dict) else {}
        source = raw.get('source') if isinstance(raw.get('source'), dict) else {}

        if 'x' in map_pose and 'y' in map_pose:
            x = _as_float(map_pose.get('x'))
            y = _as_float(map_pose.get('y'))
        elif 'x' in local and 'z' in local:
            x = _as_float(local.get('x'))
            y = _as_float(local.get('z'))
        elif 'x' in raw and ('y' in raw or 'z' in raw):
            x = _as_float(raw.get('x'))
            y = _as_float(raw.get('y', raw.get('z')))
        else:
            continue

        point_id = str(raw.get('point_id') or '')
        source_type = str(source.get('type') or '')
        nav_point_id = point_id if source_type == 'nav_point' or str(raw.get('kind') or '') in {'calibration', 'inspection'} else ''

        points.append(OutdoorExecutionPoint(
            seq=raw_index,
            kind=str(raw.get('kind') or 'transit'),
            x=x,
            y=y,
            z=_as_float(map_pose.get('z'), 0.0),
            yaw_deg=None if raw.get('yaw_deg') in (None, '') else _as_float(raw.get('yaw_deg')),
            lat=None if geo.get('lat') in (None, '') else _as_float(geo.get('lat')),
            lon=None if geo.get('lon') in (None, '') else _as_float(geo.get('lon')),
            nav_point_id=nav_point_id,
            name=str(raw.get('name') or point_id),
            source=source_type or str(raw.get('source') or 'mission_planner_route_point'),
            tolerance_m=None if raw.get('tolerance_m') in (None, '') else _as_float(raw.get('tolerance_m')),
        ))

    for index, point in enumerate(points):
        point.seq = index
    return points


def _nav_point_to_local(nav_point: Dict[str, Any]) -> Dict[str, float]:
    return {
        'x': _as_float(nav_point.get('local_x')),
        'y': _as_float(nav_point.get('local_z')),
    }


def _nearest_point_index(points: Sequence[OutdoorExecutionPoint], target: Dict[str, float]) -> int:
    if not points:
        return -1
    best_index = 0
    best_distance = float('inf')
    for index, point in enumerate(points):
        distance_m = math.hypot(point.x - target['x'], point.y - target['y'])
        if distance_m < best_distance:
            best_index = index
            best_distance = distance_m
    return best_index


def build_execution_points(
    route: Dict[str, Any],
    *,
    position_tolerance_m: float,
    min_segment_m: float = 2.0,
    max_segment_m: float = 12.0,
    calibration_snap_radius_m: float = 12.0,
) -> List[OutdoorExecutionPoint]:
    explicit_points = _explicit_route_points(route)
    if explicit_points:
        return explicit_points

    route_nav_points = route.get('route_nav_points') or []
    if not isinstance(route_nav_points, list):
        route_nav_points = []

    polyline = _polyline_from_route(route)
    if not polyline:
        for nav_point in route_nav_points:
            if isinstance(nav_point, dict) and 'local_x' in nav_point and 'local_z' in nav_point:
                polyline.append(_nav_point_to_local(nav_point))

    sampled = _resample_polyline(
        polyline,
        min_segment_m=float(min_segment_m),
        max_segment_m=max(float(max_segment_m), 0.1),
    )

    points = [
        OutdoorExecutionPoint(
            seq=index,
            kind='transit',
            x=point['x'],
            y=point['y'],
            source='planner_polyline',
            tolerance_m=position_tolerance_m,
        )
        for index, point in enumerate(sampled)
    ]

    for nav_point in route_nav_points:
        if not isinstance(nav_point, dict) or 'local_x' not in nav_point or 'local_z' not in nav_point:
            continue
        target = _nav_point_to_local(nav_point)
        nearest = _nearest_point_index(points, target)
        nav_distance = (
            float('inf') if nearest < 0 else
            math.hypot(points[nearest].x - target['x'], points[nearest].y - target['y'])
        )
        point = OutdoorExecutionPoint(
            seq=0,
            kind='inspection' if str(nav_point.get('action') or '').strip() else 'calibration',
            x=target['x'],
            y=target['y'],
            yaw_deg=(
                None if nav_point.get('yaw') in (None, '') else _as_float(nav_point.get('yaw'))
            ),
            lat=None if nav_point.get('lat') in (None, '') else _as_float(nav_point.get('lat')),
            lon=None if nav_point.get('lon') in (None, '') else _as_float(nav_point.get('lon')),
            nav_point_id=str(nav_point.get('id') or ''),
            name=str(nav_point.get('name') or ''),
            source='route_nav_point',
            tolerance_m=position_tolerance_m,
        )
        if nearest >= 0 and nav_distance <= calibration_snap_radius_m:
            point.seq = points[nearest].seq
            points[nearest] = point
        else:
            insert_at = nearest + 1 if nearest >= 0 else len(points)
            points.insert(insert_at, point)

    for index, point in enumerate(points):
        point.seq = index
    return points
