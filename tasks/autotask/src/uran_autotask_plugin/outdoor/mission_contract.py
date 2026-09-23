from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class OutdoorExecutionPoint:
    seq: int
    kind: str
    x: float
    y: float
    z: float = 0.0
    yaw_deg: Optional[float] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    nav_point_id: str = ''
    name: str = ''
    source: str = ''
    tolerance_m: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class OutdoorMissionPlan:
    task_id: str
    task_type: str
    scene_name: str
    planner_result_id: str
    robot_id: str
    map_name: str
    frame_id: str
    coordinate_system: Dict[str, Any] = field(default_factory=dict)
    projection_origin: Dict[str, Any] = field(default_factory=dict)
    route_nav_point_ids: List[str] = field(default_factory=list)
    route_nav_points: List[Dict[str, Any]] = field(default_factory=list)
    execution_points: List[OutdoorExecutionPoint] = field(default_factory=list)
    position_tolerance_m: float = 10.0
    gps_jump_reject_m: float = 15.0
    gps_vo_blend_window_s: float = 3.0
    stable_offset_required_count: int = 2
    min_gps_fix_type: int = 2
    calibrate_at_nav_points: bool = True
    raw_route_summary: Dict[str, Any] = field(default_factory=dict)

    def to_status_dict(self) -> Dict[str, Any]:
        return {
            'task_id': self.task_id,
            'task_type': self.task_type,
            'scene_name': self.scene_name,
            'planner_result_id': self.planner_result_id,
            'robot_id': self.robot_id,
            'map_name': self.map_name,
            'frame_id': self.frame_id,
            'position_tolerance_m': self.position_tolerance_m,
            'gps_jump_reject_m': self.gps_jump_reject_m,
            'gps_vo_blend_window_s': self.gps_vo_blend_window_s,
            'stable_offset_required_count': self.stable_offset_required_count,
            'min_gps_fix_type': self.min_gps_fix_type,
            'calibrate_at_nav_points': self.calibrate_at_nav_points,
            'route_nav_point_ids': list(self.route_nav_point_ids),
            'total_execution_points': len(self.execution_points),
            'coordinate_system': dict(self.coordinate_system),
            'projection_origin': dict(self.projection_origin),
            'raw_route_summary': dict(self.raw_route_summary),
        }
