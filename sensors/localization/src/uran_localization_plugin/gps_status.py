import json
from typing import Any, Dict, Optional


class GpsStatusTracker:
    """Normalize external GPS status into a stable robot_pose field."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        config = dict(config or {})
        self._min_fix_type = int(_as_float(config.get('min_fix_type'), 2.0))
        self._min_num_sv = int(_as_float(config.get('min_num_sv'), 0.0))
        self._status_stale_timeout_s = max(
            0.1,
            _as_float(config.get('status_stale_timeout_s'), 5.0),
        )
        self._status: Dict[str, Any] = {}
        self._last_parse_error = ''
        self._received_timestamp_ns = 0

    def update_from_json(self, data: str, *, received_timestamp_ns: int = 0) -> Dict[str, Any]:
        try:
            parsed = json.loads(data or '{}')
            if not isinstance(parsed, dict):
                raise ValueError('gps status payload is not a JSON object')
        except Exception as exc:
            self._last_parse_error = str(exc)
            self._received_timestamp_ns = int(received_timestamp_ns or 0)
            return self.snapshot(now_ns=received_timestamp_ns)

        self._status = dict(parsed)
        self._last_parse_error = ''
        self._received_timestamp_ns = int(received_timestamp_ns or 0)
        return self.snapshot(now_ns=received_timestamp_ns)

    def snapshot(self, *, now_ns: int = 0) -> Dict[str, Any]:
        if not self._status:
            state = 'invalid_status' if self._last_parse_error else 'unknown'
            message = (
                'gps status payload is invalid'
                if self._last_parse_error else
                'gps status has not been received'
            )
            return {
                'available': False,
                'state': state,
                'message': message,
                'connected': False,
                'stale': True,
                'valid_fix': False,
                'fix_type': 0,
                'num_sv': 0,
                'active_port': '',
                'last_error': '',
                'parse_error': self._last_parse_error,
                'received_timestamp_ns': self._received_timestamp_ns,
                'status_age_s': _age_s(now_ns, self._received_timestamp_ns),
            }

        connected = _as_bool(self._status.get('connected'), False)
        stale = _as_bool(self._status.get('stale'), True)
        valid_fix = _as_bool(self._status.get('valid_fix'), False)
        fix_type = int(_as_float(self._status.get('fix_type'), 0.0))
        num_sv = int(_as_float(self._status.get('num_sv'), 0.0))
        last_error = str(self._status.get('last_error') or '')
        status_age_s = _age_s(now_ns, self._received_timestamp_ns)

        if status_age_s is not None and status_age_s > self._status_stale_timeout_s:
            state = 'status_stale'
            message = 'gps status has not been updated recently'
        elif not connected:
            state = 'disconnected'
            message = last_error or 'external gps is disconnected'
        elif stale:
            state = 'stale'
            message = 'gps serial data is stale'
        elif not valid_fix and num_sv <= 0:
            state = 'no_signal'
            message = 'gps is connected but has no usable satellite signal'
        elif not valid_fix:
            state = 'no_fix'
            message = 'gps is connected but has no valid fix'
        elif fix_type < self._min_fix_type:
            state = 'weak_fix'
            message = 'gps fix type is lower than required'
        elif self._min_num_sv > 0 and num_sv < self._min_num_sv:
            state = 'weak_fix'
            message = 'gps satellite count is lower than required'
        else:
            state = 'valid_fix'
            message = 'gps fix is valid'

        available = state == 'valid_fix'
        status_timestamp_ns = int(_as_float(self._status.get('timestamp_ns'), 0.0))
        return {
            'available': available,
            'state': state,
            'message': message,
            'connected': connected,
            'stale': stale,
            'valid_fix': valid_fix,
            'fix_type': fix_type,
            'num_sv': num_sv,
            'lat': _optional_float(self._status.get('lat')),
            'lon': _optional_float(self._status.get('lon')),
            'alt': _optional_float(self._status.get('alt')),
            'active_port': str(self._status.get('active_port') or ''),
            'last_error': last_error,
            'last_rx_age_s': _optional_float(self._status.get('last_rx_age_s')),
            'last_publish_age_s': _optional_float(self._status.get('last_publish_age_s')),
            'parsed_nav_pvt_frames': int(_as_float(self._status.get('parsed_nav_pvt_frames'), 0.0)),
            'published_messages': int(_as_float(self._status.get('published_messages'), 0.0)),
            'read_error_count': int(_as_float(self._status.get('read_error_count'), 0.0)),
            'reconnect_count': int(_as_float(self._status.get('reconnect_count'), 0.0)),
            'status_timestamp_ns': status_timestamp_ns,
            'received_timestamp_ns': self._received_timestamp_ns,
            'status_age_s': status_age_s,
            'parse_error': self._last_parse_error,
        }


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {'1', 'true', 'yes', 'y', 'on'}:
            return True
        if normalized in {'0', 'false', 'no', 'n', 'off'}:
            return False
        return default
    return bool(value)


def _as_float(value: Any, default: float = 0.0) -> float:
    if value in (None, ''):
        return default
    try:
        return float(value)
    except Exception:
        return default


def _optional_float(value: Any):
    if value in (None, ''):
        return None
    try:
        return float(value)
    except Exception:
        return None


def _age_s(now_ns: int, then_ns: int):
    try:
        now_ns = int(now_ns or 0)
        then_ns = int(then_ns or 0)
    except Exception:
        return None
    if now_ns <= 0 or then_ns <= 0 or now_ns < then_ns:
        return None
    return float(now_ns - then_ns) / 1_000_000_000.0
