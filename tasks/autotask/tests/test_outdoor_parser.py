import json
from pathlib import Path

from uran_autotask_plugin.outdoor import (
    GpsVoGate,
    OutdoorExecutionPoint,
    OutdoorGoalResolver,
    is_outdoor_task_payload,
    parse_outdoor_task_definition,
)


def test_parse_mission_planner_plan_result():
    plan_path = Path('/home/techno/mission_planner/data/outputs/interactive_plan_1859b62319/plan_result.json')
    if not plan_path.exists():
        return

    plan = json.loads(plan_path.read_text())
    payload = {
        'task_type': 'outdoor_route',
        'task_id': 'demo_outdoor',
        'robot_id': 'cyberdog2_01',
        'scene_name': plan.get('scene_name', ''),
        'planner_result_id': plan_path.parent.name,
        'plan_result': plan,
        'execution': {'position_tolerance_m': 10.0, 'max_segment_m': 12.0},
    }

    mission = parse_outdoor_task_definition(
        task_id='demo_outdoor',
        task_type='outdoor_route',
        task_params_json=json.dumps(payload),
        defaults={'map_name': '', 'frame_id': 'map'},
    )

    assert mission.robot_id == 'cyberdog2_01'
    assert mission.execution_points
    assert any(point.kind in {'calibration', 'inspection'} for point in mission.execution_points)


def test_parse_mission_planner_route_payload_v1_1_points():
    payload = {
        'schema_version': '1.1.0',
        'task_type': 'mission_planner_route',
        'task_id': 'mp_demo_cyberdog2_01',
        'planner_result_id': 'interactive_plan_demo',
        'scene_name': 'NCEPU',
        'robot': {
            'planning_slot_id': 'slot_01',
            'hardware_id': 'cyberdog2_01',
            'display_name': '机器狗 1',
        },
        'coordinate_system': {
            'local': {
                'frame_id': 'mission_planner_local_xz',
                'projection_origin': {'lat': 38.888235, 'lon': 115.508},
            },
            'ros_map': {'frame_id': 'map', 'x_from': 'local.x', 'y_from': 'local.z'},
        },
        'execution': {
            'position_tolerance_m': 10.0,
            'gps_jump_reject_m': 15.0,
            'min_gps_fix_type': 2,
        },
        'route': {
            'route_nav_point_ids': ['NP_001'],
            'route_nav_points': [],
            'points': [
                {
                    'seq': 0,
                    'point_id': 'start_slot_01',
                    'kind': 'start',
                    'source': {'type': 'robot_start', 'id': 'start_slot_01'},
                    'local': {'x': 0.0, 'z': 0.0},
                    'map': {'frame_id': 'map', 'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'geo': {'lat': 38.888235, 'lon': 115.508, 'alt': 0.0},
                    'tolerance_m': 10.0,
                },
                {
                    'seq': 1,
                    'point_id': 'n_demo',
                    'kind': 'transit',
                    'source': {'type': 'route_graph_node', 'id': 'n_demo'},
                    'local': {'x': 5.0, 'z': 2.0},
                    'map': {'frame_id': 'map', 'x': 5.0, 'y': 2.0, 'z': 0.0},
                    'geo': {'lat': 38.88825, 'lon': 115.50805, 'alt': 0.0},
                    'tolerance_m': 10.0,
                },
                {
                    'seq': 2,
                    'point_id': 'NP_001',
                    'kind': 'calibration',
                    'source': {'type': 'nav_point', 'id': 'NP_001'},
                    'local': {'x': 10.0, 'z': 3.0},
                    'map': {'frame_id': 'map', 'x': 10.0, 'y': 3.0, 'z': 0.0},
                    'geo': {'lat': 38.88826, 'lon': 115.5081, 'alt': 0.0},
                    'tolerance_m': 10.0,
                },
            ],
        },
        'planner_summary': {
            'planning_slot_id': 'slot_01',
            'hardware_id': 'cyberdog2_01',
            'total_distance_with_home_m': 12.0,
            'estimated_time_with_home_s': 20.0,
        },
    }

    mission = parse_outdoor_task_definition(
        task_id='fallback',
        task_type='mission_planner_route',
        task_params_json=json.dumps(payload, ensure_ascii=False),
        defaults={'map_name': '', 'frame_id': 'map'},
    )

    assert mission.task_id == 'mp_demo_cyberdog2_01'
    assert mission.robot_id == 'cyberdog2_01'
    assert mission.projection_origin == {'lat': 38.888235, 'lon': 115.508}
    assert len(mission.execution_points) == 3
    assert mission.execution_points[1].x == 5.0
    assert mission.execution_points[1].y == 2.0
    assert mission.execution_points[2].kind == 'calibration'
    assert mission.execution_points[2].nav_point_id == 'NP_001'
    assert mission.execution_points[2].lat == 38.88826
    assert mission.raw_route_summary['hardware_id'] == 'cyberdog2_01'


def test_default_task_with_mission_planner_payload_is_outdoor_task():
    payload = {
        'schema_version': '1.2.0',
        'task_type': 'mission_planner_route',
        'task_id': 'mp_demo_cyberdog2_01',
        'projection_origin': {'lat': 38.888235, 'lon': 115.508, 'alt': 0.0},
        'home_pose': {'lat': 38.888235, 'lon': 115.508, 'alt': 0.0},
        'start_pose': {'lat': 38.888235, 'lon': 115.508, 'alt': 0.0},
        'route': {
            'points': [
                {
                    'seq': 1,
                    'point_id': 'NP_001',
                    'kind': 'inspection',
                    'map': {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'z': 0.0},
                    'geo': {'lat': 38.888236, 'lon': 115.508001, 'alt': 0.0},
                },
            ],
        },
    }

    assert is_outdoor_task_payload(
        task_type='default_task',
        task_params_json=json.dumps(payload),
    )

    mission = parse_outdoor_task_definition(
        task_id='fallback',
        task_type='default_task',
        task_params_json=json.dumps(payload),
        defaults={'map_name': '', 'frame_id': 'map'},
    )
    assert mission.task_type == 'mission_planner_route'
    assert len(mission.execution_points) == 1


def test_gps_vo_gate_accepts_in_tolerance():
    gate = GpsVoGate(stable_required_count=2)
    result = gate.evaluate(
        gps_error_m=6.0,
        tolerance_m=10.0,
        jump_reject_m=15.0,
        gps_delta_from_prev_m=None,
        visual_delta_from_prev_m=None,
        gps_fix_type=2,
        gps_num_sv=10,
    )
    assert result['status'] == 'passed'
    assert result['decision'] == 'pass'


def test_gps_vo_gate_rejects_unsupported_jump_then_fails_stable_offset():
    gate = GpsVoGate(stable_required_count=2)
    first = gate.evaluate(
        gps_error_m=20.0,
        tolerance_m=10.0,
        jump_reject_m=15.0,
        gps_delta_from_prev_m=40.0,
        visual_delta_from_prev_m=1.0,
        gps_fix_type=2,
        gps_num_sv=10,
    )
    assert first['status'] == 'pending'
    assert first['decision'] == 'reject_gps_jump'

    second = gate.evaluate(
        gps_error_m=18.0,
        tolerance_m=10.0,
        jump_reject_m=15.0,
        gps_delta_from_prev_m=3.0,
        visual_delta_from_prev_m=2.0,
        gps_fix_type=2,
        gps_num_sv=10,
    )
    third = gate.evaluate(
        gps_error_m=17.0,
        tolerance_m=10.0,
        jump_reject_m=15.0,
        gps_delta_from_prev_m=2.0,
        visual_delta_from_prev_m=2.0,
        gps_fix_type=2,
        gps_num_sv=10,
    )
    assert second['status'] == 'pending'
    assert third['status'] == 'failed'


def test_outdoor_goal_resolver_generates_route_tangent_candidates():
    resolver = OutdoorGoalResolver({
        'enabled': True,
        'max_attempts_per_point': 5,
        'search_radii_m': [0.0, 2.0],
    })
    previous = OutdoorExecutionPoint(seq=0, kind='transit', x=0.0, y=0.0)
    point = OutdoorExecutionPoint(seq=1, kind='inspection', x=10.0, y=0.0)
    next_point = OutdoorExecutionPoint(seq=2, kind='transit', x=20.0, y=0.0)

    resolution = resolver.resolve(point, previous_point=previous, next_point=next_point)

    assert resolution.candidate(0).offset_kind == 'original'
    assert resolution.candidate(1).offset_kind == 'left'
    assert resolution.candidate(1).x == 10.0
    assert resolution.candidate(1).y == 2.0
    assert resolution.candidate(2).offset_kind == 'right'
    assert resolution.candidate(2).y == -2.0
    assert len(resolution.candidates) == 5


def test_outdoor_goal_resolver_skip_policy_keeps_nav_points_strict_by_default():
    resolver = OutdoorGoalResolver({
        'skip_unreachable_transit': True,
        'skip_unreachable_nav_points': False,
    })

    assert resolver.should_skip_after_exhausted('transit') is True
    assert resolver.should_skip_after_exhausted('inspection') is False
    assert resolver.should_skip_after_exhausted('calibration') is False
