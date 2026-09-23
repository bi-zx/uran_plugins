from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


TERMINAL_TASK_STAGES = {'completed', 'aborted'}


@dataclass
class GeoWaypoint:
    seq: int
    lat: float
    lon: float
    alt: float = 0.0
    heading_deg: Optional[float] = None
    speed_mps: float = 0.5
    hover_time_s: float = 0.0
    actions: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MissionTask:
    task_id: str
    task_type: str
    map_name: str
    outdoor: bool
    localization_source: str
    loop_closure_source: str
    heading_mode: str
    abort_on_low_battery: bool
    low_battery_threshold: float
    abort_action: str
    media_record: bool
    sensor_report_interval_ms: int
    waypoints: List[GeoWaypoint] = field(default_factory=list)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data['waypoints'] = [waypoint.to_dict() for waypoint in self.waypoints]
        return data


@dataclass
class TaskRuntime:
    task_id: str = ''
    stage: str = 'idle'
    status: str = 'idle'
    current_waypoint_index: int = -1
    current_waypoint_seq: int = -1
    total_waypoints: int = 0
    progress_percent: float = 0.0
    event: Optional[str] = None
    error: Optional[Dict[str, Any]] = None
    last_goal_map_pose: Optional[Dict[str, float]] = None

    def to_status_dict(
        self,
        *,
        task: Optional[MissionTask] = None,
        position: Optional[Dict[str, Any]] = None,
        battery_level: Optional[float] = None,
        control_mode: str = '',
        controller: str = '',
        backend_state: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        payload = {
            'task_id': self.task_id,
            'stage': self.stage,
            'status': self.status,
            'current_waypoint_index': self.current_waypoint_index,
            'current_waypoint_seq': self.current_waypoint_seq,
            'total_waypoints': self.total_waypoints,
            'progress_percent': round(float(self.progress_percent), 3),
            'event': self.event,
            'error': self.error,
            'last_goal_map_pose': self.last_goal_map_pose,
            'control_mode': control_mode,
            'controller': controller,
            'position': position or {},
            'battery_level': battery_level,
            'backend_state': backend_state or {},
        }
        if task is not None:
            payload['task'] = task.to_dict()
        return payload
