from typing import Any, Dict, Optional


class GpsVoGate:
    def __init__(self, *, stable_required_count: int = 2):
        self._stable_required_count = max(1, int(stable_required_count))
        self._stable_offset_count = 0
        self._last_result: Dict[str, Any] = {}

    def reset(self):
        self._stable_offset_count = 0
        self._last_result = {}

    def state_snapshot(self) -> Dict[str, Any]:
        return {
            'stable_offset_count': self._stable_offset_count,
            'stable_required_count': self._stable_required_count,
            'last_result': dict(self._last_result),
        }

    def evaluate(
        self,
        *,
        gps_error_m: float,
        tolerance_m: float,
        jump_reject_m: float,
        gps_delta_from_prev_m: Optional[float],
        visual_delta_from_prev_m: Optional[float],
        gps_fix_type: int,
        gps_num_sv: int,
        min_fix_type: int = 2,
    ) -> Dict[str, Any]:
        gps_error_m = float(gps_error_m)
        tolerance_m = float(tolerance_m)
        jump_reject_m = float(jump_reject_m)

        result = {
            'decision': 'observe',
            'status': 'pending',
            'gps_error_m': gps_error_m,
            'tolerance_m': tolerance_m,
            'jump_reject_m': jump_reject_m,
            'gps_delta_from_prev_m': gps_delta_from_prev_m,
            'visual_delta_from_prev_m': visual_delta_from_prev_m,
            'gps_fix_type': int(gps_fix_type),
            'gps_num_sv': int(gps_num_sv),
            'stable_offset_count': self._stable_offset_count,
            'reason': '',
        }

        if int(gps_fix_type) < int(min_fix_type):
            self._stable_offset_count = 0
            result.update({
                'decision': 'observe',
                'status': 'pending',
                'stable_offset_count': self._stable_offset_count,
                'reason': 'gps fix type is lower than required',
            })
            self._last_result = dict(result)
            return result

        if gps_error_m <= tolerance_m:
            self._stable_offset_count = 0
            result.update({
                'decision': 'pass',
                'status': 'passed',
                'stable_offset_count': self._stable_offset_count,
                'reason': 'gps error is within tolerance',
            })
            self._last_result = dict(result)
            return result

        gps_jump = (
            gps_delta_from_prev_m is not None and
            float(gps_delta_from_prev_m) > jump_reject_m
        )
        visual_supports_jump = (
            visual_delta_from_prev_m is not None and
            float(visual_delta_from_prev_m) > jump_reject_m * 0.5
        )

        if gps_jump and not visual_supports_jump:
            self._stable_offset_count = 0
            result.update({
                'decision': 'reject_gps_jump',
                'status': 'pending',
                'stable_offset_count': self._stable_offset_count,
                'reason': 'gps jumped but local odometry did not support the same motion',
            })
            self._last_result = dict(result)
            return result

        self._stable_offset_count += 1
        result['stable_offset_count'] = self._stable_offset_count
        if self._stable_offset_count >= self._stable_required_count:
            result.update({
                'decision': 'fail_stable_offset',
                'status': 'failed',
                'reason': 'gps offset is stable and still exceeds tolerance',
            })
        else:
            result.update({
                'decision': 'observe_stable_offset',
                'status': 'pending',
                'reason': 'gps offset exceeds tolerance; waiting for another stable sample',
            })
        self._last_result = dict(result)
        return result
