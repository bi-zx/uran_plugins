import math
import statistics
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class GapChoice:
    theta_target: float
    gap_start: float
    gap_end: float
    gap_width_rad: float
    score: float
    front_clearance_m: float
    nearest_obstacle_m: float
    blocked_ratio: float
    physical_width_m: float = 0.0
    direction: str = ''

    def to_dict(self) -> Dict[str, Any]:
        return {
            'theta_target_rad': self.theta_target,
            'theta_target_deg': math.degrees(self.theta_target),
            'gap_start_deg': math.degrees(self.gap_start),
            'gap_end_deg': math.degrees(self.gap_end),
            'gap_width_deg': math.degrees(self.gap_width_rad),
            'score': self.score,
            'front_clearance_m': self.front_clearance_m,
            'nearest_obstacle_m': self.nearest_obstacle_m,
            'blocked_ratio': self.blocked_ratio,
            'physical_width_m': self.physical_width_m,
            'direction': self.direction,
        }


@dataclass
class AvoidanceDecision:
    state: str
    theta_target: float
    front_clearance_m: float
    nearest_obstacle_m: float
    blocked_ratio: float
    choice: Optional[GapChoice] = None
    reason: str = ''
    scan_quality: str = 'good'
    valid_ratio: float = 1.0
    front_valid_ratio: float = 1.0
    positive_return_count: int = 0
    positive_return_ratio: float = 1.0
    zero_range_ratio: float = 0.0
    front_positive_return_ratio: float = 1.0
    front_zero_range_ratio: float = 0.0
    zero_ranges_treated_as_clear: bool = False
    direction: str = ''

    def to_dict(self) -> Dict[str, Any]:
        return {
            'state': self.state,
            'theta_target_rad': self.theta_target,
            'theta_target_deg': math.degrees(self.theta_target),
            'front_clearance_m': self.front_clearance_m,
            'nearest_obstacle_m': self.nearest_obstacle_m,
            'blocked_ratio': self.blocked_ratio,
            'reason': self.reason,
            'choice': self.choice.to_dict() if self.choice is not None else None,
            'scan_quality': self.scan_quality,
            'valid_ratio': self.valid_ratio,
            'front_valid_ratio': self.front_valid_ratio,
            'positive_return_count': self.positive_return_count,
            'positive_return_ratio': self.positive_return_ratio,
            'zero_range_ratio': self.zero_range_ratio,
            'front_positive_return_ratio': self.front_positive_return_ratio,
            'front_zero_range_ratio': self.front_zero_range_ratio,
            'zero_ranges_treated_as_clear': self.zero_ranges_treated_as_clear,
            'direction': self.direction,
        }


class GapAvoidancePlanner:
    """Small Follow-the-Gap style planner for straight-drive obstacle avoidance."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        cfg = dict(config or {})
        self.usable_fov_rad = math.radians(float(cfg.get('usable_fov_deg', 180.0)))
        self.angle_offset_rad = math.radians(float(cfg.get('angle_offset_deg', 0.0)))
        self.obstacle_consider_range_m = float(cfg.get('obstacle_consider_range_m', 3.0))
        self.robot_radius_m = float(cfg.get('robot_radius_m', 0.35))
        self.safety_margin_m = float(cfg.get('safety_margin_m', 0.20))
        self.inflation_radius_m = float(
            cfg.get('inflation_radius_m', self.robot_radius_m + self.safety_margin_m)
        )
        self.min_gap_width_rad = math.radians(float(cfg.get('min_gap_width_deg', 28.0)))
        self.max_bubble_angle_rad = math.radians(float(cfg.get('max_bubble_angle_deg', 35.0)))
        self.front_sector_rad = math.radians(float(cfg.get('front_sector_deg', 18.0)))
        self.min_stop_distance_m = float(cfg.get('min_stop_distance_m', 0.65))
        self.clear_path_distance_m = float(cfg.get('clear_path_distance_m', 1.4))
        self.median_window = max(1, int(cfg.get('median_window', 5)))
        if self.median_window % 2 == 0:
            self.median_window += 1
        self.goal_weight = float(cfg.get('goal_weight', 2.0))
        self.clearance_weight = float(cfg.get('clearance_weight', 1.0))
        self.width_weight = float(cfg.get('width_weight', 0.25))
        self.center_bias_weight = float(cfg.get('center_bias_weight', 0.4))
        self.direction_consistency_weight = float(
            cfg.get('direction_consistency_weight', 1.0)
        )
        self.robot_width_m = max(0.1, float(cfg.get('robot_width_m', 2.0 * self.robot_radius_m)))
        self.min_gap_physical_width_m = max(
            self.robot_width_m + 2.0 * self.safety_margin_m,
            float(cfg.get('min_gap_physical_width_m', self.robot_width_m + 2.0 * self.safety_margin_m)),
        )
        self.min_valid_ratio = _clamp(float(cfg.get('min_scan_valid_ratio', 0.50)), 0.0, 1.0)
        self.min_front_valid_ratio = _clamp(
            float(cfg.get('min_front_valid_ratio', 0.50)),
            0.0,
            1.0,
        )
        self.degraded_valid_ratio = _clamp(
            float(cfg.get('degraded_scan_valid_ratio', 0.75)),
            self.min_valid_ratio,
            1.0,
        )
        self.degraded_front_valid_ratio = _clamp(
            float(cfg.get('degraded_front_valid_ratio', 0.75)),
            self.min_front_valid_ratio,
            1.0,
        )
        zero_range_mode = str(cfg.get('zero_range_mode', 'invalid')).strip().lower()
        self.zero_range_mode = (
            zero_range_mode
            if zero_range_mode in {'invalid', 'clear_if_scan_healthy'}
            else 'invalid'
        )
        self.min_positive_return_count = max(
            1,
            int(cfg.get('min_positive_return_count', 20)),
        )
        self.min_positive_return_ratio = _clamp(
            float(cfg.get('min_positive_return_ratio', 0.05)),
            0.0,
            1.0,
        )
        self.direction_lock_s = max(0.0, float(cfg.get('avoid_direction_lock_s', 1.5)))
        self.clear_confirm_s = max(0.0, float(cfg.get('clear_confirm_s', 0.8)))
        self.recover_confirm_s = max(0.0, float(cfg.get('recover_confirm_s', 0.8)))
        self._last_beam_valid: List[bool] = []
        self._last_valid_ratio = 0.0
        self._last_front_valid_ratio = 0.0
        self._last_positive_return_count = 0
        self._last_positive_return_ratio = 0.0
        self._last_zero_range_ratio = 0.0
        self._last_front_positive_return_ratio = 0.0
        self._last_front_zero_range_ratio = 0.0
        self._last_zero_ranges_treated_as_clear = False
        self._last_scan_quality = 'invalid'
        self._avoid_direction = ''
        self._avoid_direction_since: Optional[float] = None
        self._clear_since: Optional[float] = None
        self._last_state = 'idle'

    def reset(self):
        self._last_beam_valid = []
        self._last_valid_ratio = 0.0
        self._last_front_valid_ratio = 0.0
        self._last_positive_return_count = 0
        self._last_positive_return_ratio = 0.0
        self._last_zero_range_ratio = 0.0
        self._last_front_positive_return_ratio = 0.0
        self._last_front_zero_range_ratio = 0.0
        self._last_zero_ranges_treated_as_clear = False
        self._last_scan_quality = 'invalid'
        self._avoid_direction = ''
        self._avoid_direction_since = None
        self._clear_since = None
        self._last_state = 'idle'

    def quality_snapshot(self) -> Dict[str, Any]:
        return {
            'scan_quality': self._last_scan_quality,
            'valid_ratio': self._last_valid_ratio,
            'front_valid_ratio': self._last_front_valid_ratio,
            'positive_return_count': self._last_positive_return_count,
            'positive_return_ratio': self._last_positive_return_ratio,
            'zero_range_ratio': self._last_zero_range_ratio,
            'front_positive_return_ratio': self._last_front_positive_return_ratio,
            'front_zero_range_ratio': self._last_front_zero_range_ratio,
            'zero_ranges_treated_as_clear': self._last_zero_ranges_treated_as_clear,
            'avoid_direction': self._avoid_direction,
            'avoid_direction_age_s': (
                max(0.0, time.monotonic() - self._avoid_direction_since)
                if self._avoid_direction_since is not None else None
            ),
            'clear_age_s': (
                max(0.0, time.monotonic() - self._clear_since)
                if self._clear_since is not None else None
            ),
        }

    def plan(
        self,
        scan,
        *,
        theta_goal: float = 0.0,
        stop_distance_m: Optional[float] = None,
        now_s: Optional[float] = None,
    ) -> AvoidanceDecision:
        now = time.monotonic() if now_s is None else float(now_s)
        beams = self._preprocess(scan)
        if not beams:
            self._last_scan_quality = 'invalid'
            self._last_state = 'blocked'
            return self._decision(
                state='blocked',
                theta_target=0.0,
                front_clearance_m=0.0,
                nearest_obstacle_m=0.0,
                blocked_ratio=1.0,
                reason='no usable laser beams',
            )

        front_ranges = [r for theta, r in beams if abs(theta) <= self.front_sector_rad]
        front_clearance = min(front_ranges) if front_ranges else min(r for _, r in beams)
        nearest_obstacle = min(r for _, r in beams)
        valid_ratio = self._last_valid_ratio
        front_valid_ratio = self._last_front_valid_ratio
        if (
            valid_ratio < self.min_valid_ratio or
            front_valid_ratio < self.min_front_valid_ratio
        ):
            self._last_scan_quality = 'invalid'
            self._last_state = 'blocked'
            return self._decision(
                state='blocked',
                theta_target=0.0,
                front_clearance_m=front_clearance,
                nearest_obstacle_m=nearest_obstacle,
                blocked_ratio=1.0,
                reason='laser scan quality is insufficient',
            )
        if (
            valid_ratio < self.degraded_valid_ratio or
            front_valid_ratio < self.degraded_front_valid_ratio
        ):
            self._last_scan_quality = 'degraded'
        else:
            self._last_scan_quality = 'good'

        blocked = self._build_blocked_mask(beams)
        blocked_ratio = float(sum(1 for item in blocked if item)) / float(len(blocked))
        stop_distance = self.min_stop_distance_m if stop_distance_m is None else max(
            self.min_stop_distance_m,
            float(stop_distance_m),
        )

        if front_clearance <= stop_distance:
            choice = self._choose_gap(
                beams,
                blocked,
                theta_goal=theta_goal,
                now_s=now,
            )
            self._remember_direction(choice, now)
            self._clear_since = None
            if choice is None:
                self._last_state = 'blocked'
                return self._decision(
                    state='blocked',
                    theta_target=0.0,
                    front_clearance_m=front_clearance,
                    nearest_obstacle_m=nearest_obstacle,
                    blocked_ratio=blocked_ratio,
                    reason='front obstacle is too close and no safe gap was found',
                )
            self._last_state = 'stop'
            return self._decision(
                state='stop',
                theta_target=choice.theta_target if choice is not None else 0.0,
                front_clearance_m=front_clearance,
                nearest_obstacle_m=nearest_obstacle,
                blocked_ratio=blocked_ratio,
                choice=choice,
                reason='front obstacle inside stop distance',
            )

        if self._has_clear_goal_corridor(beams, blocked, theta_goal):
            if self._avoid_direction:
                if self._clear_since is None:
                    self._clear_since = now
                clear_for = now - self._clear_since
                if clear_for < self.recover_confirm_s:
                    self._last_state = 'recover'
                    return self._decision(
                        state='recover',
                        theta_target=_normalize_angle(theta_goal),
                        front_clearance_m=front_clearance,
                        nearest_obstacle_m=nearest_obstacle,
                        blocked_ratio=blocked_ratio,
                        reason='goal corridor is clear, recovering route direction',
                    )
                self._avoid_direction = ''
                self._avoid_direction_since = None
            self._clear_since = self._clear_since or now
            if now - self._clear_since >= self.clear_confirm_s:
                self._clear_since = None
            self._last_state = 'clear'
            return self._decision(
                state='clear',
                theta_target=_normalize_angle(theta_goal),
                front_clearance_m=front_clearance,
                nearest_obstacle_m=nearest_obstacle,
                blocked_ratio=blocked_ratio,
                reason='goal corridor is clear',
            )

        self._clear_since = None
        choice = self._choose_gap(
            beams,
            blocked,
            theta_goal=theta_goal,
            now_s=now,
        )
        if choice is None:
            self._last_state = 'blocked'
            return self._decision(
                state='blocked',
                theta_target=0.0,
                front_clearance_m=front_clearance,
                nearest_obstacle_m=nearest_obstacle,
                blocked_ratio=blocked_ratio,
                reason='no safe gap found',
            )

        self._remember_direction(choice, now)
        self._last_state = 'avoid'
        return self._decision(
            state='avoid',
            theta_target=choice.theta_target,
            front_clearance_m=front_clearance,
            nearest_obstacle_m=nearest_obstacle,
            blocked_ratio=blocked_ratio,
            choice=choice,
            reason='selected safest local gap',
        )

    def _decision(self, **kwargs) -> AvoidanceDecision:
        return AvoidanceDecision(
            scan_quality=self._last_scan_quality,
            valid_ratio=self._last_valid_ratio,
            front_valid_ratio=self._last_front_valid_ratio,
            positive_return_count=self._last_positive_return_count,
            positive_return_ratio=self._last_positive_return_ratio,
            zero_range_ratio=self._last_zero_range_ratio,
            front_positive_return_ratio=self._last_front_positive_return_ratio,
            front_zero_range_ratio=self._last_front_zero_range_ratio,
            zero_ranges_treated_as_clear=self._last_zero_ranges_treated_as_clear,
            direction=self._avoid_direction,
            **kwargs,
        )

    def _remember_direction(self, choice: Optional[GapChoice], now: float):
        if choice is None or not choice.direction:
            return
        if self._avoid_direction == choice.direction:
            return
        if (
            self._avoid_direction and
            self._avoid_direction_since is not None and
            now - self._avoid_direction_since < self.direction_lock_s
        ):
            return
        self._avoid_direction = choice.direction
        self._avoid_direction_since = now

    def _preprocess(self, scan) -> List[Tuple[float, float]]:
        self._last_beam_valid = []
        self._last_valid_ratio = 0.0
        self._last_front_valid_ratio = 0.0
        self._last_positive_return_count = 0
        self._last_positive_return_ratio = 0.0
        self._last_zero_range_ratio = 0.0
        self._last_front_positive_return_ratio = 0.0
        self._last_front_zero_range_ratio = 0.0
        self._last_zero_ranges_treated_as_clear = False
        ranges = list(getattr(scan, 'ranges', []) or [])
        if not ranges:
            return []

        angle_min = float(getattr(scan, 'angle_min', 0.0))
        angle_increment = float(getattr(scan, 'angle_increment', 0.0))
        if not math.isfinite(angle_increment) or abs(angle_increment) < 1e-9:
            return []

        range_min = float(getattr(scan, 'range_min', 0.05) or 0.05)
        range_max = float(getattr(scan, 'range_max', 10.0) or 10.0)
        if not math.isfinite(range_max) or range_max <= range_min:
            range_max = max(range_min + 1.0, self.obstacle_consider_range_m)

        half_fov = max(0.0, self.usable_fov_rad / 2.0)
        normalized_ranges: List[float] = []
        valid_flags: List[bool] = []
        zero_flags: List[bool] = []
        positive_return_flags: List[bool] = []
        for raw in ranges:
            converted = True
            try:
                value = float(raw)
            except Exception:
                converted = False
                value = range_max
            valid = True
            zero_range = value == 0.0
            positive_return = converted and math.isfinite(value) and value > 0.0
            if not converted:
                valid = False
            if math.isnan(value) or value <= 0.0:
                valid = False
                value = range_max
            elif math.isinf(value):
                value = range_max
            value = _clamp(value, range_min, range_max)
            normalized_ranges.append(value)
            valid_flags.append(valid)
            zero_flags.append(zero_range)
            positive_return_flags.append(positive_return)

        filtered_ranges = self._median_filter(normalized_ranges)
        beams_with_validity = []
        for index, value in enumerate(filtered_ranges):
            theta = angle_min + float(index) * angle_increment + self.angle_offset_rad
            theta = _normalize_angle(theta)
            if -half_fov <= theta <= half_fov:
                beams_with_validity.append((
                    theta,
                    value,
                    valid_flags[index],
                    zero_flags[index],
                    positive_return_flags[index],
                ))
        beams_with_validity.sort(key=lambda item: item[0])
        beam_count = len(beams_with_validity)
        positive_return_count = sum(
            1 for _, _, _, _, positive in beams_with_validity if positive
        )
        positive_return_ratio = (
            float(positive_return_count) / float(beam_count)
            if beam_count else 0.0
        )
        scan_has_healthy_returns = (
            positive_return_count >= self.min_positive_return_count and
            positive_return_ratio >= self.min_positive_return_ratio
        )
        treat_zero_as_clear = (
            self.zero_range_mode == 'clear_if_scan_healthy' and
            scan_has_healthy_returns
        )

        beams = [(theta, value) for theta, value, _, _, _ in beams_with_validity]
        self._last_beam_valid = [
            valid or (zero_range and treat_zero_as_clear)
            for _, _, valid, zero_range, _ in beams_with_validity
        ]
        self._last_positive_return_count = positive_return_count
        self._last_positive_return_ratio = positive_return_ratio
        self._last_zero_range_ratio = (
            float(sum(1 for _, _, _, zero_range, _ in beams_with_validity if zero_range)) /
            float(beam_count)
            if beam_count else 0.0
        )
        self._last_zero_ranges_treated_as_clear = bool(
            treat_zero_as_clear and self._last_zero_range_ratio > 0.0
        )
        self._last_valid_ratio = (
            float(sum(1 for valid in self._last_beam_valid if valid)) /
            float(len(self._last_beam_valid))
            if self._last_beam_valid else 0.0
        )
        front_flags = [
            valid for (theta, _), valid in zip(beams, self._last_beam_valid)
            if abs(theta) <= self.front_sector_rad
        ]
        front_raw_flags = [
            (zero_range, positive)
            for theta, _, _, zero_range, positive in beams_with_validity
            if abs(theta) <= self.front_sector_rad
        ]
        self._last_front_valid_ratio = (
            float(sum(1 for valid in front_flags if valid)) / float(len(front_flags))
            if front_flags else 0.0
        )
        self._last_front_positive_return_ratio = (
            float(sum(1 for _, positive in front_raw_flags if positive)) /
            float(len(front_raw_flags))
            if front_raw_flags else 0.0
        )
        self._last_front_zero_range_ratio = (
            float(sum(1 for zero_range, _ in front_raw_flags if zero_range)) /
            float(len(front_raw_flags))
            if front_raw_flags else 0.0
        )
        return beams

    def _median_filter(self, values: Sequence[float]) -> List[float]:
        if self.median_window <= 1 or len(values) <= 2:
            return list(values)
        radius = self.median_window // 2
        result: List[float] = []
        for index in range(len(values)):
            start = max(0, index - radius)
            end = min(len(values), index + radius + 1)
            result.append(float(statistics.median(values[start:end])))
        return result

    def _build_blocked_mask(self, beams: Sequence[Tuple[float, float]]) -> List[bool]:
        blocked = [False] * len(beams)
        if not beams:
            return blocked

        angle_increment = self._estimate_angle_increment(beams)
        for index, (_, distance) in enumerate(beams):
            if distance > self.obstacle_consider_range_m:
                continue
            ratio = _clamp(self.inflation_radius_m / max(distance, 0.05), 0.0, 1.0)
            bubble = min(math.asin(ratio), self.max_bubble_angle_rad)
            k = max(0, int(math.ceil(bubble / max(angle_increment, 1e-6))))
            start = max(0, index - k)
            end = min(len(beams), index + k + 1)
            for beam_index in range(start, end):
                blocked[beam_index] = True
        return blocked

    def _estimate_angle_increment(self, beams: Sequence[Tuple[float, float]]) -> float:
        if len(beams) < 2:
            return math.radians(1.0)
        diffs = [
            abs(float(beams[index + 1][0]) - float(beams[index][0]))
            for index in range(len(beams) - 1)
        ]
        diffs = [item for item in diffs if item > 1e-9 and math.isfinite(item)]
        if not diffs:
            return math.radians(1.0)
        return float(statistics.median(diffs))

    def _has_clear_goal_corridor(
        self,
        beams: Sequence[Tuple[float, float]],
        blocked: Sequence[bool],
        theta_goal: float,
    ) -> bool:
        theta_goal = _normalize_angle(theta_goal)
        corridor_half_width = max(self.min_gap_width_rad / 2.0, self.front_sector_rad)
        matched = False
        for (theta, distance), is_blocked in zip(beams, blocked):
            if abs(_normalize_angle(theta - theta_goal)) > corridor_half_width:
                continue
            matched = True
            if is_blocked or distance < self.clear_path_distance_m:
                return False
        return matched

    def _choose_gap(
        self,
        beams: Sequence[Tuple[float, float]],
        blocked: Sequence[bool],
        *,
        theta_goal: float,
        now_s: Optional[float] = None,
    ) -> Optional[GapChoice]:
        gaps = self._find_gaps(beams, blocked)
        if not gaps:
            return None

        theta_goal = _normalize_angle(theta_goal)
        best: Optional[GapChoice] = None
        front_ranges = [r for theta, r in beams if abs(theta) <= self.front_sector_rad]
        front_clearance = min(front_ranges) if front_ranges else min(r for _, r in beams)
        nearest_obstacle = min(r for _, r in beams)
        blocked_ratio = float(sum(1 for item in blocked if item)) / float(len(blocked))

        choices = []
        for start, end in gaps:
            start_theta = beams[start][0]
            end_theta = beams[end][0]
            width = end_theta - start_theta
            if width < self.min_gap_width_rad:
                continue
            target = _clamp(theta_goal, start_theta, end_theta)
            center = (start_theta + end_theta) / 2.0
            gap_clearance = min(beams[index][1] for index in range(start, end + 1))
            physical_width = 2.0 * gap_clearance * math.sin(max(0.0, width) / 2.0)
            if physical_width < self.min_gap_physical_width_m:
                continue
            direction = 'left' if center > 1e-6 else 'right' if center < -1e-6 else ''
            score = (
                -self.goal_weight * abs(_normalize_angle(target - theta_goal)) +
                self.clearance_weight * min(gap_clearance, self.obstacle_consider_range_m) +
                self.width_weight * width -
                self.center_bias_weight * abs(center) +
                (
                    self.direction_consistency_weight
                    if direction and direction == self._avoid_direction else 0.0
                )
            )
            choice = GapChoice(
                theta_target=target,
                gap_start=start_theta,
                gap_end=end_theta,
                gap_width_rad=width,
                score=score,
                front_clearance_m=front_clearance,
                nearest_obstacle_m=nearest_obstacle,
                blocked_ratio=blocked_ratio,
                physical_width_m=physical_width,
                direction=direction,
            )
            choices.append(choice)

        locked_direction = self._avoid_direction
        if (
            locked_direction and
            self._avoid_direction_since is not None and
            now_s is not None and
            float(now_s) - self._avoid_direction_since < self.direction_lock_s
        ):
            locked_choices = [
                choice for choice in choices if choice.direction == locked_direction
            ]
            if locked_choices:
                choices = locked_choices

        for choice in choices:
            if best is None or choice.score > best.score:
                best = choice
        return best

    def _find_gaps(
        self,
        beams: Sequence[Tuple[float, float]],
        blocked: Sequence[bool],
    ) -> List[Tuple[int, int]]:
        gaps: List[Tuple[int, int]] = []
        start = None
        for index, is_blocked in enumerate(blocked):
            if not is_blocked and start is None:
                start = index
            elif is_blocked and start is not None:
                gaps.append((start, index - 1))
                start = None
        if start is not None:
            gaps.append((start, len(blocked) - 1))
        return gaps
