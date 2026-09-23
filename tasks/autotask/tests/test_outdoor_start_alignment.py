from uran_autotask_plugin.outdoor import OutdoorExecutionPoint, OutdoorMissionPlan, OutdoorStartAligner


def _mission():
    return OutdoorMissionPlan(
        task_id='task_start_alignment',
        task_type='mission_planner_route',
        scene_name='demo',
        planner_result_id='plan_demo',
        robot_id='dog_1',
        map_name='',
        frame_id='map',
        execution_points=[
            OutdoorExecutionPoint(seq=0, kind='start', x=0.0, y=0.0),
            OutdoorExecutionPoint(seq=1, kind='transit', x=5.0, y=0.0),
            OutdoorExecutionPoint(seq=2, kind='inspection', x=10.0, y=1.0),
            OutdoorExecutionPoint(seq=3, kind='home', x=0.0, y=0.0),
        ],
    )


def test_start_aligner_goes_directly_to_first_inspection_when_close():
    aligner = OutdoorStartAligner({
        'first_inspection_direct_distance_m': 5.0,
        'home_calibration_accept_distance_m': 5.0,
        'home_calibration_reject_distance_m': 10.0,
    })

    result = aligner.evaluate_and_apply(
        mission=_mission(),
        current_pose={
            'x': 7.0,
            'y': 1.0,
            'z': 0.0,
            'frame_id': 'map',
            'timestamp_ns': 1_000_000_000,
        },
        now_ns=1_100_000_000,
    )

    assert result['status'] == 'direct_first_inspection'
    assert result['action'] == 'go_first_inspection'
    assert result['first_inspection_index'] == 2
    assert result['next_current_waypoint_index'] == 1


def test_start_aligner_accepts_home_calibration_when_home_is_close():
    aligner = OutdoorStartAligner({
        'first_inspection_direct_distance_m': 5.0,
        'home_calibration_accept_distance_m': 5.0,
        'home_calibration_reject_distance_m': 10.0,
    })

    result = aligner.evaluate_and_apply(
        mission=_mission(),
        current_pose={
            'x': 3.0,
            'y': 0.0,
            'z': 0.0,
            'frame_id': 'map',
            'timestamp_ns': 1_000_000_000,
        },
        now_ns=1_100_000_000,
    )

    assert result['status'] == 'home_calibration_passed'
    assert result['action'] == 'go_first_inspection'
    assert result['home_distance_m'] == 3.0
    assert result['next_current_waypoint_index'] == 1


def test_start_aligner_requires_home_calibration_when_home_is_within_reject_distance():
    aligner = OutdoorStartAligner({
        'first_inspection_direct_distance_m': 5.0,
        'home_calibration_accept_distance_m': 5.0,
        'home_calibration_reject_distance_m': 10.0,
    })

    result = aligner.evaluate_and_apply(
        mission=_mission(),
        current_pose={
            'x': 0.0,
            'y': 8.0,
            'z': 0.0,
            'frame_id': 'map',
            'timestamp_ns': 1_000_000_000,
        },
        now_ns=1_100_000_000,
    )

    assert result['status'] == 'home_calibration_required'
    assert result['action'] == 'run_home_calibration'
    assert result['home_calibration_index'] == 3
    assert result['first_inspection_index'] == 2


def test_start_aligner_rejects_when_home_calibration_is_too_far():
    aligner = OutdoorStartAligner({
        'first_inspection_direct_distance_m': 5.0,
        'home_calibration_accept_distance_m': 5.0,
        'home_calibration_reject_distance_m': 10.0,
    })

    result = aligner.evaluate_and_apply(
        mission=_mission(),
        current_pose={
            'x': 0.0,
            'y': 11.0,
            'z': 0.0,
            'frame_id': 'map',
            'timestamp_ns': 1_000_000_000,
        },
        now_ns=1_100_000_000,
    )

    assert result['status'] == 'failed'
    assert result['action'] == 'reject'


def test_start_aligner_rejects_when_far_from_first_inspection_and_home():
    aligner = OutdoorStartAligner({
        'first_inspection_direct_distance_m': 5.0,
        'home_calibration_accept_distance_m': 5.0,
        'home_calibration_reject_distance_m': 10.0,
    })

    result = aligner.evaluate_and_apply(
        mission=_mission(),
        current_pose={
            'x': 30.0,
            'y': 0.0,
            'z': 0.0,
            'frame_id': 'map',
            'timestamp_ns': 1_000_000_000,
        },
        now_ns=1_100_000_000,
    )

    assert result['status'] == 'failed'
    assert result['action'] == 'reject'


def test_start_aligner_waits_for_fresh_pose():
    aligner = OutdoorStartAligner({'max_pose_age_s': 3.0})

    result = aligner.evaluate_and_apply(
        mission=_mission(),
        current_pose={'x': 7.0, 'y': 1.0, 'timestamp_ns': 1_000_000_000},
        now_ns=5_000_000_000,
    )

    assert result['status'] == 'stale_pose'
    assert result['action'] == 'wait'
