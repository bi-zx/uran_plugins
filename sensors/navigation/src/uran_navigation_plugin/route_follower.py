import math
from typing import Any, Dict, Optional


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class RouteFollower:
    """Geometry-only local route follower.

    The class never publishes a command.  It projects the current pose onto
    one route segment and returns a look-ahead reference for the velocity
    controller.  Coordinates are expected to be in one common metric frame.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        cfg = dict(config or {})
        self.lookahead_min_m = max(0.1, float(cfg.get('lookahead_min_m', 0.8)))
        self.lookahead_max_m = max(
            self.lookahead_min_m,
            float(cfg.get('lookahead_max_m', 2.0)),
        )
        self.lookahead_time_s = max(0.0, float(cfg.get('lookahead_time_s', 0.8)))
        self.goal_pass_margin_m = max(0.0, float(cfg.get('goal_pass_margin_m', 0.35)))
        self.corridor_half_width_m = max(
            0.1,
            float(cfg.get('route_corridor_half_width_m', 1.5)),
        )
        self._route_start = None
        self._goal = None
        self._next_point = None
        self._goal_kind = 'inspection'
        self._requires_stop = True

    def reset(self):
        self._route_start = None
        self._goal = None
        self._next_point = None
        self._goal_kind = 'inspection'
        self._requires_stop = True

    def set_segment(
        self,
        *,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
        next_x: Optional[float] = None,
        next_y: Optional[float] = None,
        goal_kind: str = 'inspection',
        requires_stop: Optional[bool] = None,
    ):
        self._route_start = (float(start_x), float(start_y))
        self._goal = (float(goal_x), float(goal_y))
        if next_x is not None and next_y is not None:
            self._next_point = (float(next_x), float(next_y))
        else:
            self._next_point = None
        self._goal_kind = str(goal_kind or 'inspection')
        self._requires_stop = (
            bool(requires_stop)
            if requires_stop is not None
            else self._goal_kind not in {'transit'}
        )

    def snapshot(self) -> Dict[str, Any]:
        return {
            'route_start': self._route_start,
            'goal': self._goal,
            'next_point': self._next_point,
            'goal_kind': self._goal_kind,
            'requires_stop': self._requires_stop,
            'lookahead_min_m': self.lookahead_min_m,
            'lookahead_max_m': self.lookahead_max_m,
            'lookahead_time_s': self.lookahead_time_s,
            'corridor_half_width_m': self.corridor_half_width_m,
        }

    def update(
        self,
        pose: Dict[str, Any],
        *,
        speed_mps: float = 0.0,
        lookahead_override_m: Optional[float] = None,
    ) -> Dict[str, Any]:
        if not pose or self._route_start is None or self._goal is None:
            return {
                'available': False,
                'reason': 'route segment or pose is unavailable',
            }

        try:
            px = float(pose['x'])
            py = float(pose['y'])
            yaw = math.radians(float(pose.get('yaw_deg', 0.0)))
        except (TypeError, ValueError, KeyError):
            return {
                'available': False,
                'reason': 'pose has invalid x/y/yaw',
            }

        sx, sy = self._route_start
        gx, gy = self._goal
        dx = gx - sx
        dy = gy - sy
        segment_length = math.hypot(dx, dy)
        goal_distance = math.hypot(gx - px, gy - py)
        if segment_length < 1e-6:
            return {
                'available': True,
                'segment_length_m': 0.0,
                'along_track_m': 0.0,
                'cross_track_error_m': goal_distance,
                'goal_distance_m': goal_distance,
                'route_heading_rad': yaw,
                'theta_route_rad': 0.0,
                'curvature_radpm': 0.0,
                'lookahead_m': 0.0,
                'passed_goal': goal_distance <= self.goal_pass_margin_m,
                'requires_stop': self._requires_stop,
                'goal_kind': self._goal_kind,
            }

        ux = dx / segment_length
        uy = dy / segment_length
        rel_x = px - sx
        rel_y = py - sy
        along = rel_x * ux + rel_y * uy
        cross = rel_x * (-uy) + rel_y * ux

        lookahead = (
            self.lookahead_min_m + max(0.0, abs(float(speed_mps))) * self.lookahead_time_s
            if lookahead_override_m is None else float(lookahead_override_m)
        )
        lookahead = _clamp(lookahead, self.lookahead_min_m, self.lookahead_max_m)
        reference_s = _clamp(max(0.0, along) + lookahead, 0.0, segment_length)
        ref_x = sx + ux * reference_s
        ref_y = sy + uy * reference_s

        if not self._requires_stop and self._next_point is not None:
            remaining_along = max(0.0, segment_length - max(0.0, along))
            next_lookahead = max(0.0, lookahead - remaining_along)
            next_dx = self._next_point[0] - gx
            next_dy = self._next_point[1] - gy
            next_length = math.hypot(next_dx, next_dy)
            if next_lookahead > 0.0 and next_length > 1e-6:
                next_s = min(next_lookahead, next_length)
                ref_x = gx + next_dx / next_length * next_s
                ref_y = gy + next_dy / next_length * next_s

        # A stop point must still pull the robot back toward its exact target.
        # A transit point with a following segment keeps looking forward;
        # otherwise crossing the switching line would place the reference
        # behind the robot and provoke an unnecessary turn-around.
        if along > segment_length and (self._requires_stop or self._next_point is None):
            ref_x = gx
            ref_y = gy

        delta_x = ref_x - px
        delta_y = ref_y - py
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_x = cos_yaw * delta_x + sin_yaw * delta_y
        body_y = -sin_yaw * delta_x + cos_yaw * delta_y
        reference_distance = math.hypot(body_x, body_y)
        theta_route = math.atan2(body_y, body_x) if reference_distance > 1e-6 else 0.0
        curvature = (
            2.0 * body_y / max(reference_distance * reference_distance, 1e-6)
        )
        route_heading = math.atan2(dy, dx)
        passed_goal = (
            along >= segment_length and
            abs(cross) <= self.corridor_half_width_m
        )

        return {
            'available': True,
            'segment_length_m': segment_length,
            'along_track_m': along,
            'cross_track_error_m': cross,
            'goal_distance_m': goal_distance,
            'route_heading_rad': route_heading,
            'theta_route_rad': _normalize_angle(theta_route),
            'curvature_radpm': curvature,
            'lookahead_m': lookahead,
            'reference_x': ref_x,
            'reference_y': ref_y,
            'reference_body_x': body_x,
            'reference_body_y': body_y,
            'passed_goal': passed_goal,
            'requires_stop': self._requires_stop,
            'goal_kind': self._goal_kind,
            'next_point': self._next_point,
        }
