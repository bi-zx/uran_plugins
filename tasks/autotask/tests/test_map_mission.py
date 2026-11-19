import math
import time

from uran_autotask_plugin.worker import AutotaskWorker
from uran_localization_plugin import GeoReferenceConfig, MapProjector

DEVICE = {"device_id": "dog", "component_id": "base"}
REF = dict(enabled=True, frame_id="map", reference_lat=39.0, reference_lon=116.0)
PROJECT = MapProjector(GeoReferenceConfig.from_dict(REF))


def polygon(x0, y0, x1, y1):
    ring = []
    for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)):
        lat,lon = PROJECT.map_xy_to_latlon(x,y)
        ring.append([lon, lat])
    return {"type": "Polygon", "coordinates": [ring]}


def task(tmp_path, kind, *, resurvey=False, points=(), aligned=True, fence=None, order="given", manage=False, reference_mode="geographic", boundary_m=None, straight_drive=None):
    w = AutotaskWorker()
    w.configure({"device_id":"dog", "backend":"simulation", "allow_unarbitrated_simulation":True,
                 "state_reporting":False, "mapping":{"map_database":str(tmp_path/"map.sqlite3"), "fence_clearance_m":0.1,"manage_slam":manage},
                 "straight_drive":{"goal_confirm_frames":1,"goal_latch_s":0.0,**(straight_drive or {})}})
    w.emit_operation = lambda *args: None
    params = {"map_id":"quad","frame_id":"map","resolution_m":1.0,
              "geofence":fence or polygon(0,0,5,4),"resurvey":resurvey,
              "visit_order":order, "waypoints":[{"waypoint_id":name,**(xy if isinstance(xy,dict) else {"position_m":xy})} for name,xy in points]}
    if reference_mode == "local":
        params.pop("geofence")
        params["reference_mode"] = "local"
        if boundary_m:
            params["boundary_m"] = boundary_m
    elif aligned:
        params["georeference"] = REF
    cmd = {"schema_version":"1.0","operation_id":"run","task_id":"mission","target":DEVICE,
           "timestamp_ns":time.time_ns(),"action":"start","task_type":kind,"parameters":params}
    w.invoke({"operation_id":"request","message_type":"uran.tasks.command.v1","payload":cmd})
    return w,w.contexts["run"].manager


def observe(w, kind, value, stamp=None):
    timestamp = time.time_ns() if stamp is None else stamp
    frame = w.body_frame if kind == "laser_scan" else ("wgs84" if kind == "gps_fix" else "map")
    document = {"schema_version":"1.0","source":DEVICE,"timestamp_ns":timestamp,
                "observation_type":kind,"value":value,"frame_id":frame}
    return w.invoke({"operation_id":"observation","message_type":"uran.sensors.observation.v1","payload":document})


def scan(w, stamp=None):
    observe(w,"laser_scan",{"angle_min":-math.pi,"angle_max":math.pi,"angle_increment":math.pi/180,
                            "range_min":0.03,"range_max":10,"ranges":[5.0]*361},stamp)


def pose(w,x,y,stamp=None):
    observe(w,"map_pose",{"map_id":"quad","x":x,"y":y,"yaw_deg":0,"odometry_fused":True},stamp)


def terrain(w, clear=(), walls=(), occupied=(), stamp=None):
    if w.contexts["run"].manager.pose is None:
        pose(w,1.5,0.5)
    rows = [{"x_m":x+0.5,"y_m":y+0.5,"ground_continuous":True,"clearance_ok":True,
             "step_ok":True,"slope_ok":True} for x,y in clear]
    rows.extend({"x_m":x+0.5,"y_m":y+0.5,"fixed_obstacle":True} for x,y in walls)
    rows.extend({"x_m":x+0.5,"y_m":y+0.5,"occupied":True} for x,y in occupied)
    observe(w,"terrain_cells",{"map_id":"quad","cells":rows},stamp)


def test_margin_return_and_geofence_from_timestamped_fixes(tmp_path):
    w,m = task(tmp_path,"area_explore",aligned=False,fence=polygon(0,0,8,4))
    pose(w,-0.5,0.5)
    scan(w)
    cells=[(x,y) for x in range(-4,5) for y in range(3)]
    for _ in range(2): terrain(w,clear=cells)
    m.tick()
    assert m.phase=="startup"
    assert m._allowed((-1,0),"startup") and not m._allowed((-4,0),"startup")
    for x,y in [(-0.5,0.5),(1.2,0.5),(2.7,0.5)]:
        stamp=time.time_ns()
        pose(w,x,y,stamp)
        lat,lon=PROJECT.map_xy_to_latlon(x,y)
        observe(w,"gps_fix",{"latitude_deg":lat,"longitude_deg":lon,"fix_type":3},stamp)
    assert m.alignment.projector is not None
    # The latest pose can be just outside when the georeference becomes available.
    pose(w,-0.5,0.5)
    m.tick()
    assert m.phase=="return" and m.goal_xy==(0.5,0.5)
    assert all(x>=0 for x,y in m.grid.cells)
    pose(w,0.5,0.5)
    m.tick()
    assert m.phase=="explore" and not m._allowed((-1,0),"explore")


def test_resurvey_changes_static_map_in_both_directions_and_patrol_is_read_only(tmp_path):
    w,m=task(tmp_path,"area_explore")
    for _ in range(2): terrain(w,clear=[(0,0),(1,0)],walls=[(2,0)])
    assert m.grid.cells=={(0,0):1,(1,0):1,(2,0):-1}
    w,m=task(tmp_path,"area_explore",resurvey=True,fence=polygon(1,0,4,2))
    stamp=time.time_ns()
    terrain(w,walls=[(0,0),(1,0)],clear=[(2,0)],stamp=stamp)
    terrain(w,walls=[(0,0),(1,0)],clear=[(2,0)],stamp=stamp)
    assert m.grid.cells[(1,0)]==1
    terrain(w,walls=[(0,0),(1,0)],clear=[(2,0)])
    assert m.grid.cells=={(0,0):1,(1,0):-1,(2,0):1}
    w,patrol=task(tmp_path,"map_patrol",points=[('a',[0.5,0.5])])
    for _ in range(2): terrain(w,walls=[(0,0)],clear=[(1,0)],occupied=[(2,0)])
    assert (0,0) in patrol.closed and patrol.grid.cells==m.grid.cells
    _,reopened=task(tmp_path,"area_explore")
    assert reopened.grid.cells==m.grid.cells


def test_patrol_skips_unreachable_but_completes_reachable_waypoint(tmp_path):
    w,_=task(tmp_path,"area_explore")
    for _ in range(2): terrain(w,clear=[(0,0),(1,0),(2,0)],walls=[(1,1)])
    w,m=task(tmp_path,"map_patrol",points=[("behind_wall",[1.5,1.5]),("visit",[2.5,0.5])])
    pose(w,0.5,0.5)
    scan(w)
    terrain(w,clear=[(0,0)])
    m.tick()
    assert m.missed==[{"waypoint_id":"behind_wall","reason":"no_green_route"}]
    assert m.goal_xy==(2.5,0.5)
    pose(w,2.5,0.5)
    m.tick_control()
    m.tick()
    assert m.build_status_payload()["outcome"]=="partial"
    assert m.visited==["visit"] and not m.pending
    assert not m.lease_active


def test_pose_outage_and_duplicate_frame_do_not_skip_points(tmp_path):
    w,_=task(tmp_path,"area_explore")
    for _ in range(2): terrain(w,clear=[(0,0)])
    w,m=task(tmp_path,"map_patrol",points=[("work",[0.5,0.5])])
    pose(w,0.5,0.5,time.time_ns()-5_000_000_000)
    m.tick()
    assert not m.missed and m.runtime.event=="waiting_for_slam_pose"
    m.pause_current_operation(reason="manual")
    assert m.resume_current_operation()["accepted"] and m.start_xy==(0.5,0.5)
    assert m.stop_current_operation(reason="operator")["accepted"]


def test_exploration_returns_two_masks_with_unknown_and_hole(tmp_path):
    fence = polygon(0,0,4,3)
    fence["coordinates"].append(polygon(1,1,2,2)["coordinates"][0])
    w,m = task(tmp_path,"area_explore",fence=fence)
    pose(w,0.5,0.5)
    scan(w)
    for _ in range(4):
        terrain(w,clear=[(0,0)],walls=[(0,1),(1,0)])
        m.tick()
    report = m.build_status_payload()
    assert report["outcome"] == "survey_complete"
    grid = report["map"]
    def decode(name):
        return {(x,y) for y,first,last in grid[name] for x in range(first,last+1)}
    green, red = decode("green"), decode("red")
    assert green == {(0,0)}
    area = {(x,y) for x in range(4) for y in range(3)} - {(1,1)}
    assert red == area - green
    assert decode("fixed_obstacles") == {(0,1),(1,0)}
    assert grid["unverified_area_m2"] == 8
    assert decode("unverified") == red - decode("fixed_obstacles")
    assert grid["boundary"] == fence


def test_patrol_during_sensor_outage_then_all_missed(tmp_path):
    w,_ = task(tmp_path,"area_explore")
    for _ in range(2): terrain(w,clear=[(0,0)],walls=[(1,0)])
    w,m = task(tmp_path,"map_patrol",points=[("wall",[1.5,0.5]),("unknown",[2.5,1.5])])
    pose(w,0.5,0.5)
    terrain(w,clear=[(0,0)])
    scan(w,time.time_ns()-2_000_000_000)
    m.tick()
    assert not m.missed and len(m.pending)==2 and m.runtime.event=="waiting_for_laser_scan"
    scan(w)
    m.tick()
    assert m.outcome=="no_waypoint_reached" and m.runtime.stage=="aborted"
    assert [wp["waypoint_id"] for wp in m.missed]==["wall","unknown"]
    assert not m.lease_active


def test_live_obstruction_reroutes_then_remaining_waypoints_continue(tmp_path):
    w,survey=task(tmp_path,"area_explore")
    cells=[(x,y) for x in range(3) for y in range(2)]
    for _ in range(2): terrain(w,clear=cells)
    w,m=task(tmp_path,"map_patrol",points=[("first",[2.5,0.5]),("isolated",[4.5,3.5]),("last",[2.5,1.5])])
    pose(w,0.5,0.5)
    scan(w)
    terrain(w,clear=[(0,0)])
    m.tick()
    assert m.goal_xy == (2.5,0.5)
    terrain(w,occupied=[(1,0)])
    m.tick()
    for target in [(0.5,1.5),(2.5,1.5),(2.5,0.5),(2.5,1.5)]:
        assert m.goal_xy == target
        pose(w,*target)
        m.tick_control()
        m.tick()
    assert m.outcome=="partial" and m.visited==["first","last"]
    assert m.missed==[{"waypoint_id":"isolated","reason":"no_green_route"}]
    assert m.grid.cells==survey.grid.cells


def test_gnss_outlier_does_not_move_map_or_control_pose(tmp_path):
    w,m = task(tmp_path,"area_explore",aligned=False)
    for x,y,err in [(0,0,0),(1.5,0,0),(3.5,0,0),(5,0,40),(5,2,0)]:
        stamp=time.time_ns()
        pose(w,x,y,stamp)
        lat,lon=PROJECT.map_xy_to_latlon(x+err,y-err)
        observe(w,"gps_fix",{"latitude_deg":lat,"longitude_deg":lon,"fix_type":3},stamp)
    assert m.alignment.inliers==4
    assert math.dist(m.alignment.projector.map_xy_to_latlon(5,2), PROJECT.map_xy_to_latlon(5,2))<1e-7
    assert m.pose["x"]==5 and m.pose["y"]==2


def test_pointcloud_needs_ground_and_airspace_and_marks_a_drop(tmp_path):
    w,m = task(tmp_path,"area_explore")
    pose(w,0.5,0.5)
    floor = [[x/10+0.05,y/10+0.05,0.0] for x in range(-19,29) for y in range(-19,29)]
    def depth(points, height):
        pose(w,0.5,0.5)
        observe(w,"depth_rays",{"map_id":"quad","ground_xyz":[0.5,0.5,0.0],
                                  "beams":[{"origin":[0.5,0.5,height],"points":points}]})
    for _ in range(2): depth(floor,0.25)
    assert (0,0) not in m.grid.cells
    for _ in range(2): depth(floor,1.4)
    assert m.grid.cells[(0,0)]==1
    floor.append([1.55,0.55,-0.8])
    for _ in range(2): depth(floor,1.4)
    assert m.grid.cells[(1,0)]==-1


def test_patrol_waits_for_fresh_ground_before_driving(tmp_path):
    w,_ = task(tmp_path,"area_explore")
    for _ in range(2): terrain(w,clear=[(0,0),(1,0)])
    w,m = task(tmp_path,"map_patrol",points=[("gate",[1.5,0.5])])
    pose(w,0.5,0.5)
    scan(w)
    terrain(w,clear=[(1,0)])
    m.tick()
    assert m.goal_xy==(1.5,0.5)
    assert not m.motion_allowed(0.2,0) and m.waiting_for_clearance
    m.tick_control()
    assert not m.missed and m.runtime.event=="waiting_for_ground_clearance"
    for _ in range(2): terrain(w,clear=[(0,0),(1,0)])
    m.tick()
    assert m.motion_allowed(0.2,0) and m.pending[0]["waypoint_id"]=="gate"


def test_frontier_turns_towards_unseen_ground(tmp_path):
    w,m = task(tmp_path,"area_explore")
    pose(w,1.5,1.5)
    scan(w)
    for _ in range(2): terrain(w,clear=[(1,1)],walls=[(2,1),(0,1),(1,0)])
    m.tick()
    assert m.runtime.event=="turning_to_observe"
    now = time.monotonic()
    m.controller._monotonic_getter = lambda: now
    m.tick_control()
    assert m.controller.is_active() and not m.waiting_for_clearance
    observe(w,"map_pose",{"map_id":"quad","x":1.5,"y":1.5,"yaw_deg":90})
    now += 0.15
    m.tick_control()
    assert not m.controller.is_active() and not m.leg
    m.tick()
    assert m.runtime.event=="observing_terrain" and m.view_wait[:2]==((1,1),90)


def test_nearest_waypoint_uses_green_route_and_geographic_coordinates(tmp_path):
    w,_ = task(tmp_path,"area_explore")
    # The point across the wall is nearer in Euclidean distance but takes six steps.
    cells = [(0,0),(0,1),(0,2),(1,2),(2,2),(2,1),(2,0),(0,3)]
    for _ in range(2): terrain(w,clear=cells,walls=[(1,0),(1,1)])
    lat,lon = PROJECT.map_xy_to_latlon(0.5,3.5)
    w,m = task(tmp_path,"map_patrol",points=[("around_wall",[2.5,0.5]),("on_lane",{"lat":lat,"lon":lon})],order="nearest")
    pose(w,0.5,0.5)
    scan(w)
    terrain(w,clear=[(0,0)])
    m.tick()
    assert m.inspection=="on_lane" and math.dist(m.goal_xy,(0.5,3.5))<1e-6


def test_refused_save_keeps_the_new_grid_out_of_patrol(tmp_path):
    w,m=task(tmp_path,"area_explore",manage=True)
    def status(state,reason=""):
        observe(w,"slam_session_state",{"map_id":"quad","operation_id":"run","state":state,"reason":reason})
    pose(w,0.5,0.5)
    assert m.pose is None and not m.slam_ready
    status("ready")
    pose(w,0.5,0.5)
    scan(w)
    for _ in range(3):
        terrain(w,clear=[(0,0)],walls=[(0,1),(1,0)])
        m.tick()
    assert m.slam_command=="save" and m.has_active_task()
    assert not m.lease_active and "map" not in m.build_status_payload()
    _,reopened=task(tmp_path,"area_explore",manage=True)
    assert reopened.slam_mode=="mapping" and not reopened.grid.cells
    status("failed","storage unavailable")
    assert m.runtime.stage=="aborted" and m.outcome=="slam_failed" and m.detail=="storage unavailable"


def test_indoor_mapping_and_patrol_need_no_gps(tmp_path):
    import pytest
    boundary={"type":"Polygon","coordinates":[[[0,0],[3,0],[3,2],[0,2],[0,0]]]}
    w,m=task(tmp_path,"area_explore",reference_mode="local",boundary_m=boundary)
    pose(w,0.5,0.5)
    scan(w)
    for _ in range(3):
        terrain(w,clear=[(0,0)],walls=[(1,0),(0,1)])
        m.tick()
    result=m.build_status_payload()
    assert result["outcome"]=="survey_complete" and not result["georeferenced"]
    assert result["map"]["boundary_m"]==boundary and "georeference" not in result["map"]
    w,p=task(tmp_path,"map_patrol",reference_mode="local",points=[("desk",[0.5,0.5]),("wall",[1.5,0.5])])
    observe(w,"gps_fix",{"latitude_deg":39.0,"longitude_deg":116.0,"fix_type":3})
    observe(w,"map_pose",{"map_id":"quad","x":0.5,"y":0.5,"yaw_deg":0,"odometry_fused":False})
    assert p.pose is None and p.alignment is None
    pose(w,0.5,0.5)
    scan(w)
    for _ in range(2): terrain(w,clear=[(0,0)])
    p.tick()
    p.tick_control()
    p.tick()
    assert p.phase=="patrol" and p.visited==["desk"] and p.outcome=="partial"
    with pytest.raises(ValueError,match="inconsistent"):
        task(tmp_path,"area_explore",reference_mode="geographic")


def test_indoor_without_boundary_reports_observed_extent(tmp_path):
    w,m=task(tmp_path,"area_explore",reference_mode="local")
    pose(w,-1.5,-0.5)
    scan(w)
    for _ in range(3):
        terrain(w,clear=[(-2,-1)],walls=[(-1,-1),(-3,-1),(-2,0),(-2,-2)])
        m.tick()
    grid=m.build_status_payload()["map"]
    assert grid["boundary_m"]["coordinates"]==[[[-3,-2],[0,-2],[0,1],[-3,1],[-3,-2]]]
    assert grid["green"]==[[-1,-2,-2]] and grid["unverified_area_m2"]==4



def test_two_footprints_share_laser_map_but_survey_their_own_cells(tmp_path):
    import pytest
    from uran_autotask_plugin.survey_grid import SurveyGrid, CLEAR, BLOCKED

    args = (tmp_path / "maps.sqlite3", "shelf", "floor", 0.25)
    small, large = {"body_radius_m": 0.2}, {"body_radius_m": 0.6}
    def layer(terrain, patrol=False):
        return SurveyGrid(*args, patrol=patrol, reference_mode="local", terrain_config=terrain)

    first = layer(small)
    assert not first.has_saved_map
    first.save({(0, 0): CLEAR}, None)
    second = layer(large)
    assert second.has_saved_map and not second.cells
    with pytest.raises(ValueError, match="terrain profile"):
        layer(large, patrol=True)
    second.save({(0, 0): BLOCKED}, None)
    assert layer(small, patrol=True).cells == {(0, 0): CLEAR}
    assert layer(large, patrol=True).cells == {(0, 0): BLOCKED}


def test_pre_profile_sqlite_can_be_read_and_migrated_with_known_dimensions(tmp_path):
    import sqlite3
    import pytest
    from uran_autotask_plugin.survey_grid import SurveyGrid

    path = tmp_path / "old.sqlite3"
    with sqlite3.connect(path) as db:
        db.execute("CREATE TABLE maps(name TEXT PRIMARY KEY, frame TEXT, resolution REAL, reference TEXT, environment TEXT DEFAULT 'outdoor')")
        db.execute("INSERT INTO maps VALUES ('floor', 'site', 0.5, NULL, 'indoor')")
        db.execute("CREATE TABLE cells(name TEXT, x INTEGER, y INTEGER, state INTEGER, PRIMARY KEY(name,x,y))")
        db.execute("INSERT INTO cells VALUES ('floor', 1, -2, 1)")
    def open_map(terrain, patrol, legacy=None):
        return SurveyGrid(path, "floor", "site", 0.5, patrol=patrol, reference_mode="local", terrain_config=terrain, legacy_terrain=legacy)

    old = {"body_radius_m": 0.2}
    assert open_map(old, True, old).cells == {(1, -2): 1}
    with sqlite3.connect(path) as db:
        assert "terrain" not in [v[1] for v in db.execute("PRAGMA table_info(cells)")]
    with pytest.raises(ValueError, match="terrain profile"):
        open_map({}, True, old)
    assert open_map({}, False, old).has_saved_map
    assert open_map(old, True).cells == {(1, -2): 1}
    assert not open_map({}, False).cells



def test_crossing_into_a_new_fence_waits_for_ground_at_the_current_pose(tmp_path):
    w,m = task(tmp_path, "area_explore", fence=polygon(2,0,3,1))
    pose(w,.5,.5)
    scan(w)
    for _ in range(2): terrain(w,clear=[(0,0)])
    pose(w,2.5,.5)
    m.tick()
    assert m.has_active_task() and m.runtime.event == "waiting_for_ground_clearance"
    for _ in range(2): terrain(w,clear=[(2,0)])
    m.tick()
    assert m.outcome == "survey_complete" and m.build_status_payload()["map"]["green"] == [[0,2,2]]


def test_observation_radius_follows_body_and_preserves_previous_cells(tmp_path):
    w,m = task(tmp_path,"area_explore",reference_mode="local")
    pose(w,.5,.5)
    for _ in range(2): terrain(w,clear=[(0,0),(5,0),(5,1)],walls=[(6,0)])
    assert m.grid.cells == {(0,0):1,(5,0):1}
    assert m.seen_bounds == (0,0,5,0)
    pose(w,6.5,.5)
    for _ in range(2): terrain(w,clear=[(5,1)],walls=[(6,0),(0,0)])
    assert m.grid.cells == {(0,0):1,(5,0):1,(5,1):1,(6,0):-1}
    pos = m.build_status_payload()["position"]
    assert pos["x"] == 6.5 and pos["fresh"] and "latitude_deg" not in pos


def test_delayed_georeference_reports_coordinates_from_map(tmp_path):
    w,m = task(tmp_path,"area_explore",aligned=False)
    for x in (0,2,4):
        stamp=time.time_ns()
        pose(w,x,1.0,stamp)
        if x == 0:
            assert "latitude_deg" not in m.build_status_payload()["position"]
        lat,lon = PROJECT.map_xy_to_latlon(x,1.0)
        observe(w,"gps_fix",{"latitude_deg":lat,"longitude_deg":lon,"fix_type":3},stamp)
    pose(w,5.0,2.0)
    pos = m.build_status_payload()["position"]
    assert math.dist((pos["latitude_deg"],pos["longitude_deg"]),PROJECT.map_xy_to_latlon(5,2)) < 1e-7


def test_dev_profile_turn_uses_recent_terrain_outside_the_certified_body_radius(tmp_path):
    dims = dict(robot_length_m=.6, robot_width_m=.4, safety_margin_m=.15, turn_radius_m=.5)
    w, m = task(tmp_path,"area_explore",reference_mode="local",straight_drive=dims)
    pose(w,0.95,0.5)
    m.phase, m.leg, m.runtime.stage = "explore", [(0,0)], "executing"
    stamp = time.time_ns()
    # Near the right grid edge a 50 cm disk extends past the padding of the centre cell.
    m.live_clear[(0,0)] = stamp
    assert not m.motion_allowed(0,0.2) and m.waiting_for_clearance
    m.live_clear[(1,0)] = stamp
    assert m.motion_allowed(0,0.2)
    m.closed.add((1,0))
    assert not m.motion_allowed(0,0.2)
    m.closed.clear()
    m.live_clear[(1,0)] = stamp - 2_000_000_000
    assert not m.motion_allowed(0,0.2)
    # From the middle the centre cell alone already certifies the whole disk.
    pose(w,0.5,0.5)
    assert m.motion_allowed(0,0.2)
