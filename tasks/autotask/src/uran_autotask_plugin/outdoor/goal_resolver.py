import math
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional, Sequence

from .mission_contract import OutdoorExecutionPoint


@dataclass
class GoalCandidate:
    attempt_index: int
    x: float
    y: float
    z: float
    yaw_deg: Optional[float]
    source: str
    offset_kind: str
    offset_distance_m: float
    original_x: float
    original_y: float

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class GoalResolution:
    point_seq: int
    point_kind: str
    nav_point_id: str
    original_x: float
    original_y: float
    candidates: List[GoalCandidate]
    strategy: str
    validator_status: str

    def candidate(self, index: int) -> Optional[GoalCandidate]:
        if index < 0 or index >= len(self.candidates):
            return None
        return self.candidates[index]

    def has_next_after(self, index: int) -> bool:
        return index + 1 < len(self.candidates)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'point_seq': self.point_seq,
            'point_kind': self.point_kind,
            'nav_point_id': self.nav_point_id,
            'original_x': self.original_x,
            'original_y': self.original_y,
            'strategy': self.strategy,
            'validator_status': self.validator_status,
            'candidate_count': len(self.candidates),
            'candidates': [candidate.to_dict() for candidate in self.candidates],
        }


class OutdoorGoalResolver:
    """Resolve planner points into retryable Nav2 goals.

    The first version does not pretend to know where walls are. It creates a
    bounded set of nearby candidate goals; later we can plug in a map/costmap
    validator and filter these candidates before dispatching them.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        config = dict(config or {})
        self._enabled = _as_bool(config.get('enabled'), True)
        self._max_attempts = max(1, int(_as_float(config.get('max_attempts_per_point'), 7.0)))
        self._search_radii = _parse_radii(config.get('search_radii_m'), [0.0, 2.0, 5.0, 10.0])
        self._dedupe_epsilon_m = max(0.01, _as_float(config.get('dedupe_epsilon_m'), 0.1))
        self._skip_unreachable_transit = _as_bool(config.get('skip_unreachable_transit'), True)
        self._skip_unreachable_nav_points = _as_bool(
            config.get('skip_unreachable_nav_points'),
            False,
        )
        self._strategy = str(config.get('strategy') or 'route_tangent_offsets')

    def config_snapshot(self) -> Dict[str, Any]:
        return {
            'enabled': self._enabled,
            'strategy': self._strategy,
            'max_attempts_per_point': self._max_attempts,
            'search_radii_m': list(self._search_radii),
            'dedupe_epsilon_m': self._dedupe_epsilon_m,
            'skip_unreachable_transit': self._skip_unreachable_transit,
            'skip_unreachable_nav_points': self._skip_unreachable_nav_points,
        }

    def resolve(
        self,
        point: OutdoorExecutionPoint,
        *,
        previous_point: Optional[OutdoorExecutionPoint] = None,
        next_point: Optional[OutdoorExecutionPoint] = None,
        current_pose: Optional[Dict[str, Any]] = None,
    ) -> GoalResolution:
        if not self._enabled:
            return self._build_resolution(
                point,
                candidates=[
                    self._candidate(point, 0, point.x, point.y, 'original', 0.0)
                ],
            )

        direction = self._route_direction(
            point,
            previous_point=previous_point,
            next_point=next_point,
            current_pose=current_pose,
        )
        normal = (-direction[1], direction[0])

        candidates: List[GoalCandidate] = [
            self._candidate(point, 0, point.x, point.y, 'original', 0.0)
        ]

        offset_templates = []
        for radius in self._search_radii:
            if radius <= 0.0:
                continue
            offset_templates.extend((
                ('left', normal[0] * radius, normal[1] * radius, radius),
                ('right', -normal[0] * radius, -normal[1] * radius, radius),
            ))
        for radius in self._search_radii:
            if radius <= 0.0:
                continue
            offset_templates.extend((
                ('backward', -direction[0] * radius, -direction[1] * radius, radius),
                ('forward', direction[0] * radius, direction[1] * radius, radius),
            ))

        for offset_kind, dx, dy, radius in offset_templates:
            if len(candidates) >= self._max_attempts:
                break
            candidate = self._candidate(
                point,
                len(candidates),
                point.x + dx,
                point.y + dy,
                offset_kind,
                radius,
            )
            if self._is_duplicate(candidate, candidates):
                continue
            candidates.append(candidate)

        return self._build_resolution(point, candidates=candidates)

    def should_skip_after_exhausted(self, point_kind: str) -> bool:
        normalized = str(point_kind or '').strip().lower()
        if normalized == 'transit':
            return self._skip_unreachable_transit
        return self._skip_unreachable_nav_points

    def _build_resolution(
        self,
        point: OutdoorExecutionPoint,
        *,
        candidates: List[GoalCandidate],
    ) -> GoalResolution:
        return GoalResolution(
            point_seq=point.seq,
            point_kind=point.kind,
            nav_point_id=point.nav_point_id,
            original_x=float(point.x),
            original_y=float(point.y),
            candidates=candidates[:self._max_attempts],
            strategy=self._strategy,
            validator_status='not_configured',
        )

    def _candidate(
        self,
        point: OutdoorExecutionPoint,
        attempt_index: int,
        x: float,
        y: float,
        offset_kind: str,
        offset_distance_m: float,
    ) -> GoalCandidate:
        return GoalCandidate(
            attempt_index=int(attempt_index),
            x=float(x),
            y=float(y),
            z=float(point.z),
            yaw_deg=point.yaw_deg,
            source='planner_original' if offset_kind == 'original' else 'resolver_offset',
            offset_kind=offset_kind,
            offset_distance_m=float(offset_distance_m),
            original_x=float(point.x),
            original_y=float(point.y),
        )

    def _is_duplicate(self, candidate: GoalCandidate, candidates: Sequence[GoalCandidate]) -> bool:
        for item in candidates:
            if math.hypot(candidate.x - item.x, candidate.y - item.y) <= self._dedupe_epsilon_m:
                return True
        return False

    def _route_direction(
        self,
        point: OutdoorExecutionPoint,
        *,
        previous_point: Optional[OutdoorExecutionPoint],
        next_point: Optional[OutdoorExecutionPoint],
        current_pose: Optional[Dict[str, Any]],
    ) -> tuple:
        dx = dy = 0.0
        if previous_point is not None and next_point is not None:
            dx = float(next_point.x) - float(previous_point.x)
            dy = float(next_point.y) - float(previous_point.y)
        elif previous_point is not None:
            dx = float(point.x) - float(previous_point.x)
            dy = float(point.y) - float(previous_point.y)
        elif next_point is not None:
            dx = float(next_point.x) - float(point.x)
            dy = float(next_point.y) - float(point.y)
        elif current_pose and 'x' in current_pose and 'y' in current_pose:
            dx = float(point.x) - float(current_pose['x'])
            dy = float(point.y) - float(current_pose['y'])

        norm = math.hypot(dx, dy)
        if norm <= 1e-6:
            return (1.0, 0.0)
        return (dx / norm, dy / norm)


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


def _parse_radii(value: Any, default: Sequence[float]) -> List[float]:
    raw = value if isinstance(value, list) else list(default)
    radii: List[float] = []
    for item in raw:
        try:
            radius = max(0.0, float(item))
        except Exception:
            continue
        if radius not in radii:
            radii.append(radius)
    if 0.0 not in radii:
        radii.insert(0, 0.0)
    return sorted(radii)
