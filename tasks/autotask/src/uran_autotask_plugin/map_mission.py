"""Task policy for bounded exploration, explicit re-survey and mapped patrol."""
import json
import math
import time

from uran_navigation_plugin.terrain import MAPPING_RADIUS_M
from uran_navigation_plugin.swept_terrain import SweptTerrain
from uran_localization_plugin import GnssMapAlignment, PoseRegistry
from .survey_grid import BLOCKED, CLEAR, UNKNOWN, Fence, SurveyGrid, first_straight_leg, neighbors, path_to, reachable_paths
from .task_models import TERMINAL_TASK_STAGES, TaskRuntime


class MapMission:
    def __init__(self, command, config, database, controller, lease, progress, control_mode):
        p = command.get("parameters") or {}
        self.patrol = command["task_type"] == "map_patrol"
        self.resurvey = bool(p.get("resurvey", False))
        if self.patrol and self.resurvey:
            raise ValueError("resurvey belongs to area_explore")
        self.reference_mode = p.get("reference_mode","geographic" if "geofence" in p else "local")
        if self.reference_mode not in ("local","geographic"):
            raise ValueError("reference_mode must be local or geographic")
        self.local = self.reference_mode == "local"
        if self.local and "geofence" in p or not self.local and "boundary_m" in p:
            raise ValueError("use boundary_m for map coordinates and geofence for WGS84")
        self.fence = Fence(p.get("boundary_m") if self.local else p.get("geofence"), local=self.local)
        resolution = float(p.get("resolution_m", config.get("resolution_m", 0.5)))
        self.terrain_config = dict(config.get("terrain") or {})
        self.grid = SurveyGrid(database,p["map_id"],p.get("frame_id","map"),resolution,patrol=self.patrol,
                               reference_mode=self.reference_mode, terrain_config=self.terrain_config, legacy_terrain=config.get("legacy_terrain"))
        self.manage_slam = bool(config.get("manage_slam",False))
        self.slam_mode = "localization" if self.patrol or self.grid.has_saved_map else "mapping"
        self.slam_command = "start" if self.manage_slam else ""
        self.slam_ready = not self.manage_slam
        self.deferred_outcome = ""
        self.last_slam_request = 0.0
        saved_ref = json.loads(self.grid.reference_json) if self.grid.reference_json else None
        self.alignment = None if self.local else GnssMapAlignment(self.grid.frame_id,self.fence.geographic[0][0],
                                                                     saved_ref or p.get("georeference"),config)
        self.seen_bounds = None
        if self.alignment and self.alignment.projector:
            self.fence.align(self.alignment.projector)
        self.startup_radius = float(p.get("startup_radius_m", config.get("startup_radius_m", 2.0)))
        self.margin = float(config.get("fence_clearance_m", self.grid.terrain["body_radius_m"] + 0.05))
        self.confirmations = max(2, int(config.get("confirmation_frames", 2)))
        if self.startup_radius < 0 or self.margin < 0:
            raise ValueError("boundary distances must be nonnegative")
        self.operation_id = command["operation_id"]
        self.controller, self.lease, self.progress, self.control_mode = controller, lease, progress, control_mode
        self.runtime = TaskRuntime(task_id=command["task_id"], stage="preparing", status="normal")
        self.pending = list(p.get("waypoints", []))
        self.free_order = p.get("visit_order", "nearest") == "nearest"
        if self.patrol and (not self.pending or len({p["waypoint_id"] for p in self.pending}) != len(self.pending)):
            raise ValueError("patrol requires unique waypoint IDs")
        for wp in self.pending:
            if "position_m" in wp:
                xy = wp["position_m"]
                if len(xy) < 2 or not all(math.isfinite(v) for v in xy[:2]):
                    raise ValueError("waypoint position_m must contain finite map coordinates")
            elif self.local:
                raise ValueError("local patrol waypoints require position_m in the saved SLAM map")
            elif not (-90 <= wp["lat"] <= 90 and -180 <= wp["lon"] <= 180):
                raise ValueError("waypoint lat/lon must be WGS84 degrees")
        self.visited, self.missed = [], []
        self.poses = PoseRegistry()
        self.pose = self.start_xy = None
        self.localization_revision = None
        self.revision_start = 0
        self.surfaces = SweptTerrain(self.grid.terrain) if controller.half_length_m is not None else None
        self.supported = False
        self.forward_view = None
        self.forward_motion = False
        self.support_stamp = None
        self.external_closed = set()
        self.probing = False
        self.probe_views = set()
        self.next_pose_report = 0.0
        self.report_interval = float(config.get("progress_report_interval_s", 1.0))
        self.pose_timeout = float(config.get("pose_timeout_s", 1.0))
        self.max_pair_delta = float(config.get("pose_pair_tolerance_s", 0.3))
        self.terrain_timeout = float(config.get("terrain_timeout_s", 1.0))
        self.last_terrain_stamp = self.last_scan_stamp = self.observation_count = 0
        self.scan_usable = False
        self.terrain_frame_count = 0
        self.votes, self.observed, self.closed = {}, {}, set()
        self.survey_evidence = {}
        self.original_states = {}
        self.live_clear = {}
        self.wall_views = {}
        self.waiting_for_clearance = False
        self.edges, self.scanned_views = set(), set()
        self.probed = set()
        self.view_headings = set()
        self.view_wait = None
        self.return_limit = None
        self.phase, self.leg = "", []
        self.goal_xy = self.inspection = None
        self.lease_active = False
        self.outcome, self.detail = "", ""
        self.clearances = {}
        self.motion_denied = False
        self.inputs_missing = False
        self.hold_since = self.hold_cell = None

    @property
    def map_located(self):
        return self.local or self.alignment.projector is not None

    def _waypoint_xy(self, point):
        if "position_m" in point:
            return tuple(point["position_m"][:2])
        return self.alignment.projector.latlon_to_map_xy(point["lat"],point["lon"])

    def has_active_task(self):
        return self.runtime.stage not in TERMINAL_TASK_STAGES

    def handle_task_command(self, **_):
        self.lease(True, "map_task_started")
        self.lease_active = True
        self._event("starting_laser_"+self.slam_mode if self.manage_slam else "waiting_for_slam_pose")

    def _event(self, name):
        if name != self.runtime.event:
            self.runtime.event = name
            self.progress()

    def _halt(self, reason):
        self.forward_motion = False
        if self.controller.is_active():
            self.controller.stop(reason=reason)
        self.leg = []
        self.probing = False
        self.goal_xy = self.inspection = None
        self.motion_denied = self.waiting_for_clearance = False
        self.inputs_missing = False
        self.hold_since = self.hold_cell = None

    def _hold(self, reason):
        if self.controller.is_active():
            if self.hold_since is None:
                self.hold_since = time.monotonic()
                self.hold_cell = self.grid.cell((self.pose["x"], self.pose["y"]))
            timeout = self.controller.fault_pause_timeout_s
            if timeout and time.monotonic() - self.hold_since >= timeout:
                self._halt(reason)
            else:
                self.controller.hard_stop(reason=reason)
        self._event(reason)

    def _finish(self, outcome):
        self._halt(outcome)
        if self.lease_active:
            self.lease(False, outcome)
            self.lease_active = False
        if self.manage_slam and outcome != "slam_failed" and self.slam_command != "closed":
            self.slam_ready = False
            self.deferred_outcome = outcome
            self.slam_command = "save" if self.slam_mode == "mapping" and outcome == "survey_complete" else "stop"
            self._event("saving_laser_map" if self.slam_command == "save" else "ending_laser_session")
            return
        self.outcome = outcome
        self.runtime.stage = "aborted" if outcome in ("cancelled", "no_waypoint_reached","slam_failed") else "completed"
        self.runtime.status = "exception" if outcome in ("no_waypoint_reached","slam_failed") else "normal"
        self._event(outcome)

    def update_slam(self, value):
        state = value["state"]
        if state == "failed":
            self.detail = value.get("reason","laser SLAM failed")
            self.slam_command = "closed"
            self._finish("slam_failed")
        elif state == "ready" and self.slam_command == "start":
            self.slam_command = ""
            self.slam_ready = True
            self._event("waiting_for_slam_pose")
        elif state == "closed" and self.slam_command in ("save","stop"):
            if self.slam_command == "save":
                self.grid.save(self.grid.cells.copy(), self.grid.reference_json, map_saved=True)
            self.slam_command = "closed"
            self._finish(self.deferred_outcome)

    def pause_current_operation(self, *, reason):
        if not self.has_active_task() or self.deferred_outcome:
            return {"accepted": False, "message": "task is ending or complete"}
        self._halt(reason)
        self.runtime.stage = "paused"
        if self.lease_active:
            self.lease(False, reason)
            self.lease_active = False
        self._event(reason)
        return {"accepted": True}

    def resume_current_operation(self):
        if self.runtime.stage != "paused" or self.deferred_outcome:
            return {"accepted": False, "message": "task is not paused"}
        self.lease(True, "resumed")
        self.lease_active = True
        self.runtime.stage = "executing"
        self._event("resumed")
        return {"accepted": True}

    def stop_current_operation(self, *, reason):
        if not self.has_active_task():
            return {"accepted": False, "message": "task is complete"}
        if self.deferred_outcome:
            return {"accepted": False, "message":"task is already ending"}
        self.detail = reason
        self._finish("cancelled")
        return {"accepted": True}

    def build_status_payload(self):
        result = {"task_id": self.runtime.task_id, "stage": self.runtime.stage, "event": self.runtime.event,
                  "map_id": self.grid.map_id, "phase": self.phase, "outcome": self.outcome, "detail": self.detail,
                  "visited": list(self.visited), "missed": list(self.missed),
                  "pending": [p["waypoint_id"] for p in self.pending],
                  "reference_mode": self.reference_mode, "frame_id": self.grid.frame_id,
                  "georeferenced": bool(self.alignment and self.alignment.projector)}
        if self.pose:
            position = dict(self.pose, fresh=self.slam_ready and abs(time.time_ns()-self.pose["timestamp_ns"]) <= self.pose_timeout*1e9)
            if self.alignment and self.alignment.projector:
                position["latitude_deg"],position["longitude_deg"] = self.alignment.projector.map_xy_to_latlon(position["x"],position["y"])
            result["position"] = position
        if self.manage_slam and self.slam_command in ("start","save","stop"):
            result["slam"] = {"operation_id":self.operation_id,"command":self.slam_command,"mode":self.slam_mode}
        if not self.patrol and self.outcome == "survey_complete":
            extent = self.fence
            if self.fence.unbounded:
                cells = list(self.grid.cells) or [self.grid.cell((self.pose["x"],self.pose["y"]))]
                if self.seen_bounds:
                    left,bottom,right,top = self.seen_bounds
                    cells.extend(((left,bottom),(right,top)))
                xs,ys = zip(*cells)
                r = self.grid.resolution
                x0,y0,x1,y1 = min(xs)*r,min(ys)*r,(max(xs)+1)*r,(max(ys)+1)*r
                extent = Fence({"type":"Polygon","coordinates":[[[x0,y0],[x1,y0],[x1,y1],[x0,y1],[x0,y0]]]},local=True)
            result["map"] = self.grid.export(extent)
            result["map"]["reference_mode"] = self.reference_mode
            if self.local:
                result["map"]["boundary_m"] = {"type":"Polygon","coordinates":extent.rings}
            else:
                result["map"]["boundary"] = {"type":"Polygon","coordinates":self.fence.geographic}
                result["map"]["georeference"] = self.alignment.projector.reference_summary()
        return result

    def update_pose(self, value, stamp):
        if self.local and value.get("odometry_fused") is not True:
            return
        pose = {key: float(value[key]) for key in ("x", "y", "yaw_deg")}
        if not all(map(math.isfinite, pose.values())) or (self.pose and stamp <= self.pose["timestamp_ns"]):
            return
        revision = value.get("localization_revision",0)
        if self.localization_revision is None and revision:
            self.revision_start = stamp
        if self.localization_revision is not None and revision != self.localization_revision:
            self._halt("map_relocalized")
            self.revision_start = stamp
            if self.surfaces is not None:
                self.surfaces = SweptTerrain(self.grid.terrain)
            self.supported, self.support_stamp = False, None
            self.forward_view = None
            self.external_closed.clear()
            self.probe_views.clear()
            self.wall_views.clear()
            self.probed.clear()
            self.seen_bounds = None
            self.observed.clear()
            self.survey_evidence.clear()
            if self.original_states:
                if self.manage_slam and self.slam_mode == "mapping":
                    self.grid.apply(self.original_states,self.grid.reference_json)
                else:
                    self.grid.save(self.original_states,self.grid.reference_json)
                self.original_states.clear()
            self.poses.clear_map_poses()
            if self.alignment:
                self.alignment.clear_samples()
            self.closed.clear()
            self.edges.clear()
            self.votes.clear()
            self.live_clear.clear()
            self.view_wait = None
            self.scanned_views.clear()
            self.view_headings.clear()
            self.observation_count = self.last_terrain_stamp = 0
            self._event("map_relocalized")
        self.localization_revision = revision
        if "support_state" in value:
            support_stamp = value["support_stamp_ns"]
            if self.support_stamp is None or support_stamp >= self.support_stamp:
                self.support_stamp = support_stamp
                state = value["support_state"]
                if abs(stamp-support_stamp) <= self.pose_timeout*1e9:
                    self.supported = state == "standing" or (state == "walking" and self.supported)
        self.pose = {**pose, "timestamp_ns": stamp}
        self.poses.update_map_pose(self.pose)
        if self.start_xy is None:
            self.start_xy = (pose["x"], pose["y"])
        now = time.monotonic()
        if now >= self.next_pose_report:
            self.next_pose_report = now + self.report_interval
            self.progress()

    def update_gps(self, gps, stamp):
        if self.local or (self.patrol and self.alignment.projector):
            return
        pose = self.poses.nearest_map_pose(timestamp_ns=stamp, max_delta_s=self.max_pair_delta)
        if pose is None:
            return
        if self.alignment.update(gps, pose, stamp):
            self.fence.align(self.alignment.projector)
            self.clearances.clear()
            self._commit(self.survey_evidence, realigned=True)

    def _clearance(self, cell):
        if cell not in self.clearances:
            self.clearances[cell] = self.fence.clearance(*self.grid.center(cell))
        return self.clearances[cell]

    def _commit(self, observations, realigned=False):
        if self.patrol or not self.map_located:
            return
        changed = {}
        for cell, (label, source_xy) in observations.items():
            old = self.grid.cells.get(cell, UNKNOWN)
            if self._clearance(cell) >= 0 and self.fence.clearance(*source_xy) >= 0:
                if old != label and (old != CLEAR or self.resurvey):
                    self.original_states.setdefault(cell,old)
                    changed[cell] = label
            elif realigned and cell in self.original_states:
                changed[cell] = self.original_states.pop(cell)
        reference = None if self.local else json.dumps(self.alignment.projector.reference_summary(), sort_keys=True)
        if self.manage_slam and self.slam_mode == "mapping":
            self.grid.apply(changed,reference)
        else:
            self.grid.save(changed,reference)

    def update_terrain(self, values, stamp, frames=None, groups=()):
        if frames and self.surfaces is not None:
            for frame in sorted(frames,key=lambda f:f.stamp):
                if frame.stamp >= self.revision_start:
                    self.surfaces.merge(frame)
            self._record_forward_view(groups, stamp)
        if stamp <= self.last_terrain_stamp:
            return
        pose = self.poses.nearest_map_pose(timestamp_ns=stamp, max_delta_s=self.max_pair_delta)
        if pose is None:
            return
        xy = (pose["x"],pose["y"])
        values = [item for item in values if math.dist((item["x_m"],item["y_m"]),xy) <= MAPPING_RADIUS_M
                  and math.dist(self.grid.center(self.grid.cell((item["x_m"],item["y_m"]))),xy) <= MAPPING_RADIUS_M]
        labels = {}
        for item in values:
            cell = self.grid.cell((item["x_m"], item["y_m"]))
            if self.fence.unbounded:
                x,y = cell
                l,b,r,t = self.seen_bounds or (x,y,x,y)
                self.seen_bounds = min(l,x),min(b,y),max(r,x),max(t,y)
            fixed = item.get("fixed_obstacle") is True or item.get("step_ok") is False or item.get("slope_ok") is False
            if self.resurvey and item.get("vertical_surface") is True:
                xy = (pose["x"],pose["y"])
                first_stamp, first_xy = self.wall_views.setdefault(cell,(stamp,xy))
                if stamp-first_stamp >= 1e9 and math.dist(first_xy,xy) >= self.grid.resolution:
                    fixed = True
            elif item.get("ground_continuous") is True and item.get("clearance_ok") is True:
                self.wall_views.pop(cell,None)
            if (frames and not fixed and not item.get("occupied") and not item.get("clearance_ok")
                    and self.surfaces.covers_cell(cell,self.grid.resolution,self.grid.terrain["body_radius_m"])):
                item = dict(item,ground_continuous=True,clearance_ok=True,step_ok=True,slope_ok=True)
            clear = all(item.get(k) is True for k in ("ground_continuous", "clearance_ok", "step_ok", "slope_ok"))
            if fixed:
                labels[cell] = BLOCKED
            elif any(item.get(k) is False for k in ("ground_continuous", "clearance_ok")) or item.get("occupied") is True:
                labels.setdefault(cell, UNKNOWN)
                if labels[cell] == CLEAR:
                    labels[cell] = UNKNOWN
            elif clear:
                labels.setdefault(cell, CLEAR)
        if not labels and not any(item.get("observed") is True for item in values):
            return
        self.last_terrain_stamp = stamp
        self.terrain_frame_count += 1
        if not labels:
            return
        confirmed = {}
        for cell, label in labels.items():
            if frames is None and label != CLEAR:
                self.external_closed.add(cell)
            if label != CLEAR:
                self.closed.add(cell)
                self.live_clear.pop(cell,None)
            old_vote, count, previous = self.votes.get(cell, (UNKNOWN, 0, 0))
            count = count+1 if label == old_vote and stamp-previous <= self.terrain_timeout*1e9 else 1
            self.votes[cell] = (label, count, stamp)
            if label == UNKNOWN or count < self.confirmations:
                continue
            if label == CLEAR:
                self.closed.discard(cell)
                self.external_closed.discard(cell)
                self.live_clear[cell] = stamp
            self.observed[cell] = confirmed[cell] = label
        self.observation_count += 1
        source_xy = (pose["x"],pose["y"])
        if not self.patrol and (not self.map_located or self.fence.clearance(*source_xy) >= 0):
            evidence = {cell:(label,source_xy) for cell,label in confirmed.items()}
            self.survey_evidence.update(evidence)
            self._commit(evidence)

    def _record_forward_view(self, groups, stamp):
        # Age the sensor at acquisition, and the stream at reception. Only a timely
        # registered return ahead of the body can provide the latter timestamp.
        now = time.time_ns()
        for group in groups:
            acquired = group.get("timestamp_ns",stamp)
            if not 0 <= now-acquired <= self.terrain_timeout*1e9 or (self.forward_view and acquired <= self.forward_view[0]):
                continue
            pose = self.poses.nearest_map_pose(timestamp_ns=acquired,max_delta_s=self.max_pair_delta)
            if pose is None:
                continue
            x,y,yaw = pose["x"],pose["y"],math.radians(pose["yaw_deg"])
            c,s = math.cos(yaw),math.sin(yaw)
            for px,py,_ in group["points"]:
                dx,dy = px-x,py-y
                if (dx*c+dy*s >= self.controller.half_length_m and abs(dy*c-dx*s) <= self.controller.corridor_half_width_m
                        and math.hypot(dx,dy) <= self.surfaces.range):
                    self.forward_view = (acquired,now,x,y,yaw)
                    break

    def _can_continue_straight(self, now):
        if not self.forward_motion or not self.forward_view or not self._use_sweep():
            return False
        acquired,received,x,y,yaw = self.forward_view
        if not (acquired <= received <= now <= received+self.terrain_timeout*1e9):
            return False
        px,py,angle = self._metric_pose()
        length,width = self.controller.half_length_m,self.controller.corridor_half_width_m
        speed = self.controller.max_speed_mps
        braking = speed*self._braking_time(speed)
        dx,dy = px-x,py-y
        along = dx*math.cos(yaw)+dy*math.sin(yaw)
        across = dy*math.cos(yaw)-dx*math.sin(yaw)
        turn = (angle-yaw+math.pi)%(2*math.pi)-math.pi
        corner_shift = 2*math.hypot(length,width)*abs(math.sin(turn/2))
        if math.hypot(dx,dy)+corner_shift <= braking:
            return True
        if abs(across)+corner_shift > braking or not 0 <= along <= speed*(now-acquired)*1e-9+braking:
            return False
        # Check successive braking strips between the observed and current pose.
        # The command itself checks the final strip. Never bridge sideways or turn.
        steps = math.ceil(abs(along)/braking)
        for i in range(steps):
            t = i/steps
            pose = x+t*dx,y+t*dy,yaw+t*turn
            if self.surfaces.straight(pose,length,width,braking,acquired) != "clear":
                return False
        return True

    def _use_sweep(self):
        return (self.surfaces is not None and self.supported and self.surfaces.measured and
                abs(time.time_ns()-self.support_stamp) <= self.pose_timeout*1e9)

    def _metric_pose(self):
        return self.pose["x"],self.pose["y"],math.radians(self.pose["yaw_deg"])

    def _braking_time(self, speed):
        return abs(speed)/(2*self.controller.brake_decel_mps2) + self.controller.command_latency_s + 1/self.controller.control_hz

    def _straight_clearance(self, vx, wz):
        # Use the cruise-speed horizon during acceleration as well. A millimetre-wide
        # first command cannot give a representative ground sample across the corridor.
        speed = vx if wz else math.copysign(self.controller.max_speed_mps,vx)
        recent = time.time_ns()-int(self.terrain_timeout*1e9)
        if self.forward_view and self.last_terrain_stamp < recent:
            recent = min(recent,self.forward_view[0])
        return self.surfaces.sweep(self._metric_pose(),self.controller.half_length_m,
                                   self.controller.corridor_half_width_m,speed,wz,self._braking_time(speed),recent)

    def _walkable(self, cell, phase):
        if cell in self.closed:
            return False
        saved = self.grid.cells.get(cell, UNKNOWN)
        if self.patrol and phase == "patrol":
            return saved == CLEAR
        if self.patrol and self.map_located and self._clearance(cell) >= 0:
            return saved == CLEAR
        return self.observed.get(cell, saved) == CLEAR

    def _allowed(self, cell, phase):
        if not self._walkable(cell, phase):
            return False
        if phase == "startup":
            return math.dist(self.grid.center(cell), self.start_xy) <= self.startup_radius
        clearance = self._clearance(cell)
        if phase == "return":
            return clearance >= self.return_limit
        return clearance >= self.margin

    def turn_clearance(self, radius):
        """Confirm the turn disk with recent footprint-expanded terrain observations."""
        x, y = self.pose["x"], self.pose["y"]
        if self.phase in ("explore", "patrol") and self.fence.clearance(x, y) < max(radius, self.margin):
            return "turning disk crosses the mission boundary"
        # Every green terrain cell certifies its whole area plus body_radius_m.
        extra = max(0.0, radius - self.grid.terrain["body_radius_m"])
        size = self.grid.resolution
        left, bottom = self.grid.cell((x-extra, y-extra))
        right, top = self.grid.cell((x+extra, y+extra))
        oldest = time.time_ns() - self.terrain_timeout*1e9
        unknown = False
        for ix in range(left, right+1):
            for iy in range(bottom, top+1):
                dx = max(ix*size-x, 0.0, x-(ix+1)*size)
                dy = max(iy*size-y, 0.0, y-(iy+1)*size)
                if dx*dx + dy*dy > extra*extra:
                    continue
                cell = (ix, iy)
                if cell in (self.external_closed if self._use_sweep() else self.closed) or self.patrol and self.map_located and self._clearance(cell) >= 0 and self.grid.cells.get(cell) != CLEAR:
                    return "turning disk intersects blocked terrain"
                if self.live_clear.get(cell, 0) < oldest:
                    unknown = True
        if self._use_sweep():
            state = self.surfaces.turn(self._metric_pose(),radius,self.controller.half_length_m,
                                       self.controller.body_half_width_m)
            if state == "blocked":
                return "turning disk intersects blocked terrain"
            unknown = state != "clear"
        if unknown:
            return "turning disk needs fresh ground clearance"
        return ""

    def motion_allowed(self, vx, wz):
        """Check the commanded arc through the next update and its braking distance."""
        if vx == 0 and wz == 0:
            self.forward_motion = False
            return True
        if not self.pose or not self.leg or self.runtime.stage != "executing":
            return False
        now = time.time_ns()
        if self.last_terrain_stamp and self.terrain_timeout > 0 and now-self.last_terrain_stamp > self.terrain_timeout*1e9:
            if vx <= 0 or wz or not self._can_continue_straight(now):
                self.waiting_for_clearance = True
                return False
        if vx == 0 and self.controller.turn_radius_m is not None:
            blocker = self.turn_clearance(self.controller.turn_radius_m)
            if blocker:
                self.waiting_for_clearance = blocker == "turning disk needs fresh ground clearance"
                return False
        x, y = self.pose["x"], self.pose["y"]
        start = self.grid.cell((x,y))
        yaw = math.radians(self.pose["yaw_deg"])
        fine = self._use_sweep()
        if self.probing and not fine:
            self.waiting_for_clearance = True
            return False
        if fine and vx:
            state = self._straight_clearance(vx,wz)
            if state != "clear":
                self.waiting_for_clearance = state == "unknown"
                return False
        stop_s = self._braking_time(vx)
        steps = max(1, math.ceil(abs(vx)*stop_s / (self.grid.resolution/5)))
        dt = stop_s / steps
        for _ in range(steps):
            x += vx*dt*math.cos(yaw + wz*dt/2)
            y += vx*dt*math.sin(yaw + wz*dt/2)
            yaw += wz*dt
            cell = self.grid.cell((x,y))
            if self.patrol and self.map_located and self._clearance(cell) >= 0 and self.grid.cells.get(cell) != CLEAR:
                return False
            if (cell in (self.external_closed if fine else self.closed) or
                    (cell != start and not self._allowed(cell,self.phase) and
                     not (self.probing and self.grid.cells.get(cell,UNKNOWN) == UNKNOWN))):
                return False
            if not fine and now-self.live_clear.get(cell,0) > self.terrain_timeout*1e9:
                self.waiting_for_clearance = True
                return False
            if self.phase == "startup" and math.dist((x,y), self.start_xy) > self.startup_radius:
                return False
            if self.phase != "startup":
                minimum = self.return_limit if self.phase == "return" else self.margin
                if self.fence.clearance(x,y) < minimum:
                    return False
        self.forward_motion = vx > 0 and wz == 0
        return True

    def _next_phase(self, xy):
        if not self.map_located:
            return "startup"
        distance = self.fence.clearance(*xy)
        if distance < self.margin:
            if self.phase != "return":
                self.return_limit = distance - self.startup_radius
            return "return"
        return "patrol" if self.patrol else "explore"

    def _unknown_headings(self, cell):
        for adj, yaw in zip(neighbors(cell), (0,90,180,-90)):
            if self.map_located:
                if self._clearance(adj) < self.margin:
                    continue
            elif math.dist(self.grid.center(adj), self.start_xy) > self.startup_radius:
                continue
            if adj not in self.observed and (self.resurvey or adj not in self.grid.cells):
                yield yaw

    def _start_forward_probe(self, cell, headings):
        # A frontier is a direction to inspect. Only the measured braking strip is authorized
        # on each control tick. Discovering a frontier never colours the underlying survey cell.
        x, y, yaw = self._metric_pose()
        heading = (int(math.floor((math.degrees(yaw)+45)/90))*90+180)%360-180
        key = cell,heading
        if (self.phase != "explore" or heading%360 not in {h%360 for h in headings} or key in self.probe_views or
                self._straight_clearance(self.controller.max_speed_mps,0) != "clear"):
            return False
        distance = 0.
        step = min(self.grid.resolution/2,self.surfaces.size)
        for i in range(1,math.floor(self.surfaces.range/step)+1):
            d = i*step
            point = x+d*math.cos(yaw), y+d*math.sin(yaw)
            target = self.grid.cell(point)
            if (self.fence.clearance(*point) < self.margin or target in self.external_closed or
                    (target != cell and self.grid.cells.get(target) == BLOCKED)):
                break
            distance = d
        if distance <= self.controller.target_tolerance_m:
            return False
        goal = x+distance*math.cos(yaw), y+distance*math.sin(yaw)
        result = self.controller.start(task_id=self.runtime.task_id,target_x=goal[0],target_y=goal[1],
                    route_start_x=x,route_start_y=y,command_source="mapping",goal_kind="frontier",requires_stop=False)
        if result["accepted"]:
            self.probing = True
            self.leg, self.goal_xy = [cell],goal
            self._event("following_frontier")
            return True
        return False

    def _observe_from(self, cell):
        headings = list(self._unknown_headings(cell))
        if headings and self._use_sweep() and self._start_forward_probe(cell,headings):
            return
        headings = headings or [0]
        heading = next((yaw for yaw in headings if (cell,yaw) not in self.view_headings),None)
        if heading is None:
            self.scanned_views.add(cell)
            return
        delta = (heading-self.pose["yaw_deg"]+180)%360-180
        if abs(math.radians(delta)) > self.controller.turn_tolerance_rad:
            if (self.turn_clearance(self.controller.turn_radius_m) if self._use_sweep() else
                    time.time_ns()-self.live_clear.get(cell,0) > self.terrain_timeout*1e9):
                self._event("waiting_for_ground_clearance")
                return
            self.view_wait = None
            result = self.controller.start(task_id=self.runtime.task_id, turn_yaw_deg=heading, goal_kind="view", requires_stop=True, command_source="mapping")
            if result["accepted"]:
                self.leg = [cell]
                self._event("turning_to_observe")
            return
        if self.view_wait is None or self.view_wait[:2] != (cell,heading):
            self.view_wait = (cell,heading,self.terrain_frame_count)
        elif self.terrain_frame_count-self.view_wait[2] >= self.confirmations:
            self.view_headings.add((cell,heading))
            self.view_wait = None
        self._event("observing_terrain")

    def _exploration_target(self, parents):
        for cell in parents:
            if cell in self.scanned_views:
                continue
            if self.resurvey and self.grid.cells.get(cell) == CLEAR and cell not in self.observed:
                return cell
            if next(self._unknown_headings(cell),None) is not None:
                return cell
        return None

    def tick(self):
        if not self.has_active_task():
            return
        if self.slam_command in ("start","save","stop"):
            now = time.monotonic()
            if now-self.last_slam_request >= 1.0:
                self.last_slam_request = now
                self.progress()
            return
        if self.runtime.stage == "paused":
            return
        if self.control_mode() != "auto":
            self.pause_current_operation(reason="waiting_for_auto_mode")
            return
        now = time.time_ns()
        for stamp, timeout, reason in ((self.pose["timestamp_ns"] if self.pose else 0, self.pose_timeout, "waiting_for_slam_pose"),
                                       (self.last_terrain_stamp, self.terrain_timeout, "waiting_for_terrain"),
                                       (self.last_scan_stamp if self.scan_usable else 0, self.controller.scan_timeout_s, "waiting_for_laser_scan")):
            if not stamp or (timeout > 0 and abs(now-stamp) > timeout*1e9):
                if reason == "waiting_for_terrain" and self._can_continue_straight(now):
                    continue
                self._hold(reason)
                self.inputs_missing = True
                return
        if self.support_stamp is not None and (not self.supported or abs(now-self.support_stamp) > self.pose_timeout*1e9):
            self._hold("waiting_for_base_support")
            self.inputs_missing = True
            return
        self.inputs_missing = False
        xy = (self.pose["x"], self.pose["y"])
        start = self.grid.cell(xy)
        if self.hold_since is not None:
            timeout = self.controller.fault_pause_timeout_s
            if (timeout and time.monotonic()-self.hold_since >= timeout) or (start != self.hold_cell and start not in self.leg):
                self._halt("replan_after_sensor_gap")
        phase = self._next_phase(xy)
        if phase != self.phase:
            self._halt("phase_changed")
            self.phase = phase
            self.view_wait = None
            self.scanned_views.clear()
            self.view_headings.clear()
            self.edges.clear()
        self.runtime.stage = "executing"
        if start in self.leg and self.leg[-1] != start:
            self.leg = self.leg[self.leg.index(start)+1:]
        if self.probing:
            self.leg = [start]
        observing_here = self.goal_xy is None and self.leg == [start] and self._use_sweep()
        if self.leg and not (self.probing or observing_here) and any(not self._allowed(c,phase) for c in self.leg):
            self._halt("replan")
        if self.controller.is_active() or self.leg:
            return
        if phase == "patrol" and self.grid.cells.get(start) != CLEAR:
            self._event("waiting_for_green_start")
            return
        if start in (self.external_closed if self._use_sweep() else self.closed):
            self._event("current_footprint_obstructed")
            return
        if not self.observation_count and not self._use_sweep():
            self._event("waiting_for_terrain_coverage")
            return
        parents = reachable_paths(start, lambda c: self._allowed(c,phase), self.edges)
        target_xy = None
        if phase == "return":
            goal = next((c for c in parents if self._clearance(c) >= self.margin and self._walkable(c,phase)), None)
            if goal is None:
                self._event("waiting_for_reentry_path")
                return
        elif phase == "patrol":
            by_cell = {c: i for i,c in enumerate(parents)}
            if self.free_order:
                self.pending.sort(key=lambda wp: by_cell.get(self.grid.cell(self._waypoint_xy(wp)), math.inf))
            while self.pending:
                wp = self.pending[0]
                target_xy = self._waypoint_xy(wp)
                goal = self.grid.cell(target_xy)
                if goal in parents and self._allowed(goal,phase) and self.fence.clearance(*target_xy) >= self.margin:
                    self.inspection = wp["waypoint_id"]
                    break
                self.missed.append({"waypoint_id": wp["waypoint_id"], "reason": "no_green_route"})
                self.pending.pop(0)
                self.progress()
                if self.edges:
                    self.edges.clear()
                    return
            if not self.pending:
                self._finish("all_reached" if not self.missed else ("partial" if self.visited else "no_waypoint_reached"))
                return
        else:
            goal = self._exploration_target(parents)
            samples = self.alignment.sample_positions if goal is None and phase == "startup" else ()
            if samples:
                # A fully visible start area may still need motion for GNSS heading.
                self.probed.add(start)
                goal = max((c for c in parents if c not in self.probed), default=None,
                           key=lambda c: min(math.dist(self.grid.center(c), xy) for xy in samples))
            if goal is None:
                if phase == "startup":
                    self._event("waiting_for_georeference")
                elif now-self.live_clear.get(start,0) > self.terrain_timeout*1e9 or self.grid.cells.get(start) != CLEAR:
                    # Evidence collected before fence alignment may be outside this survey.
                    self._event("waiting_for_ground_clearance")
                else:
                    self._finish("survey_complete")
                return
            if goal == start:
                self._observe_from(start)
                return
        path = path_to(parents, goal)
        leg = first_straight_leg(path)
        end = leg[-1] if leg else goal
        self.leg = leg or [goal]
        self.goal_xy = target_xy if target_xy is not None and end == goal else self.grid.center(end)
        if end != goal:
            self.inspection = None
        result = self.controller.start(task_id=self.runtime.task_id, target_x=self.goal_xy[0], target_y=self.goal_xy[1],
            route_start_x=xy[0], route_start_y=xy[1], command_source="mapping", target_tolerance_m=min(0.15,self.grid.resolution*0.3),
            goal_kind="inspection" if self.inspection else "transit", requires_stop=bool(self.inspection))
        if not result["accepted"]:
            self._halt("controller_wait")
            self._event(result.get("code", "controller_wait"))
            return
        self._event("returning_inside" if phase == "return" else "following_path")

    def tick_control(self):
        if self.inputs_missing or not self.leg or not self.controller.is_active():
            return
        self.waiting_for_clearance = self.motion_denied = False
        result = self.controller.tick()
        if self.waiting_for_clearance or result.get("decision", {}).get("reason") == "turning disk needs fresh ground clearance":
            self._hold("waiting_for_ground_clearance")
            return
        if self.motion_denied or result.get("reason") == "obstacle blocked timeout":
            if self.probing and self._straight_clearance(self.controller.max_speed_mps,0) != "blocked":
                heading = (int(math.floor((self.pose["yaw_deg"]+45)/90))*90+180)%360-180
                self.probe_views.add((self.leg[0],heading))
            current = self.grid.cell((self.pose["x"], self.pose["y"]))
            steps = [c for c in self.leg if c in neighbors(current)]
            if steps:
                self.edges.update(((current,steps[0]), (steps[0],current)))
            elif self.inspection and current == self.leg[-1]:
                self.missed.append({"waypoint_id": self.inspection, "reason": "approach_blocked"})
                self.pending.pop(0)
                self.edges.clear()
            self._halt("route_blocked")
            self._event("replanning")
        elif result.get("state") == "completed":
            self.probing = False
            self.hold_since = self.hold_cell = None
            if self.inspection:
                self.visited.append(self.inspection)
                self.pending.pop(0)
                self.edges.clear()
            self.leg = []
            self.inspection = self.goal_xy = None
            self._event("leg_reached")
        elif result.get("code") in {"E_SCAN_QUALITY", "E_SCAN_FRONT_QUALITY", "E_SCAN_UNAVAILABLE", "E_SCAN_STALE", "E_POSE_UNAVAILABLE", "E_POSE_STALE"}:
            self._event(result["code"])
            if result.get("terminal"):
                self._halt(result["code"])
        elif result.get("terminal"):
            self._halt("controller_fault")
            self.pause_current_operation(reason=result.get("code") or result.get("reason") or "controller_fault")
        elif self.hold_since is not None and (result.get("vx",0) or result.get("wz",0)):
            self.hold_since = self.hold_cell = None
            self._event("turning_to_observe" if self.goal_xy is None else
                        ("following_frontier" if self.probing else "returning_inside" if self.phase == "return" else "following_path"))
