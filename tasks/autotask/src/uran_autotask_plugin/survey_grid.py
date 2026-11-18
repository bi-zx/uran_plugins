"""Metric survey grid, local or geographic boundary and durable traversability observations."""
import json
import math
from bisect import bisect_left, bisect_right
from collections import deque
from contextlib import closing
from pathlib import Path
import sqlite3

from uran_localization_plugin import MapProjector
from uran_navigation_plugin.terrain import terrain_parameters

UNKNOWN, CLEAR, BLOCKED = 0, 1, -1


def neighbors(cell):
    x, y = cell
    return ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1))


def reachable_paths(start, permitted, closed_edges):
    """Four connected cells have equal cost. Parents are in increasing travel distance."""
    parents = {start: None}
    queue = deque([start])
    while queue:
        current = queue.popleft()
        for step in neighbors(current):
            if step not in parents and (current, step) not in closed_edges and permitted(step):
                parents[step] = current
                queue.append(step)
    return parents


def path_to(parents, goal):
    path = []
    while goal is not None:
        path.append(goal)
        goal = parents[goal]
    return path[::-1]


def first_straight_leg(path):
    for i in range(2, len(path)):
        if path[i][0] != path[0][0] and path[i][1] != path[0][1]:
            return path[1:i]
    return path[1:]


class Fence:
    """Signed clearance from an XY position to a task polygon, including holes."""

    def __init__(self, polygon, *, local=False):
        self.unbounded = local and polygon is None
        coords = []
        if not self.unbounded:
            if not polygon or polygon.get("type") != "Polygon":
                raise ValueError("expected a closed Polygon in the task coordinate system")
            coords = polygon["coordinates"]
            if not coords or any(len(r) < 4 or r[0] != r[-1] for r in coords):
                raise ValueError("polygon rings must contain at least three vertices and close")
            if any(len(v) != 2 or not all(map(math.isfinite,v)) for r in coords for v in r):
                raise ValueError("polygon vertices must contain two finite coordinates")
        self.geographic = None if local else coords
        self.rings = coords if local else []

    def align(self, projector: MapProjector):
        self.rings = [[projector.latlon_to_map_xy(lat, lon) for lon, lat in ring] for ring in self.geographic]

    def rows(self, resolution):
        """Yield bounded rows of cell centers; inner rings leave gaps in a row."""
        low = min(y for x,y in self.rings[0])
        high = max(y for x,y in self.rings[0])
        for row in range(math.ceil(low/resolution - 0.5), math.ceil(high/resolution - 0.5)):
            y = (row + 0.5) * resolution
            cuts = sorted(ax + (y-ay)*(bx-ax)/(by-ay) for ring in self.rings
                          for (ax,ay),(bx,by) in zip(ring,ring[1:]) if (ay > y) != (by > y))
            for left,right in zip(cuts[::2],cuts[1::2]):
                first = math.ceil(left/resolution-0.5)
                last = math.ceil(right/resolution-0.5) - 1
                if first <= last:
                    yield row, first, last

    def clearance(self, x, y):
        if self.unbounded:
            return math.inf
        distance = math.inf
        membership = []
        for ring in self.rings:
            inside = False
            for (ax, ay), (bx, by) in zip(ring, ring[1:]):
                dx, dy = bx - ax, by - ay
                norm = dx * dx + dy * dy
                at = min(1.0, max(0.0, ((x - ax) * dx + (y - ay) * dy) / norm)) if norm else 0.0
                distance = min(distance, math.hypot(x - ax - at * dx, y - ay - at * dy))
                if (ay > y) != (by > y) and x < ax + (y - ay) * dx / dy:
                    inside = not inside
            membership.append(inside)
        return distance if membership and membership[0] and not any(membership[1:]) else -distance


class SurveyGrid:
    """A task edits only surveyed cells. A patrol opens the same map as a snapshot."""

    def __init__(self, path, map_id, frame_id, resolution, *, patrol, reference_mode, terrain_config, legacy_terrain=None):
        self.path = Path(path)
        self.map_id, self.frame_id, self.resolution = map_id, frame_id, resolution
        self.patrol = patrol
        self.terrain = terrain_parameters(terrain_config)
        self.layer = json.dumps(self.terrain, sort_keys=True, separators=(",", ":"))
        legacy_layer = None if legacy_terrain is None else json.dumps(terrain_parameters(legacy_terrain), sort_keys=True, separators=(",", ":"))
        if not map_id or Path(map_id).name != map_id or map_id in (".", "..") or not frame_id or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError("map_id, frame_id and a positive resolution_m are required")
        if patrol and not self.path.exists():
            raise ValueError(f"no surveyed map for {map_id}")
        if not patrol:
            self.path.parent.mkdir(parents=True, exist_ok=True)
        uri = self.path.resolve().as_uri() + ("?mode=ro" if patrol else "?mode=rwc")
        with closing(sqlite3.connect(uri, uri=True)) as db, db:
            # A patrol takes one consistent read snapshot, including on the legacy schema.
            db.execute("BEGIN" if patrol else "BEGIN IMMEDIATE")
            if not patrol:
                db.execute("CREATE TABLE IF NOT EXISTS maps (name TEXT PRIMARY KEY, frame TEXT, resolution REAL, reference TEXT, reference_mode TEXT, saved INTEGER DEFAULT 0)")
            fields = {col[1] for col in db.execute("PRAGMA table_info(maps)")}
            cell_fields = {col[1] for col in db.execute("PRAGMA table_info(cells)")}
            legacy_mode = "CASE environment WHEN 'indoor' THEN 'local' ELSE 'geographic' END" if "environment" in fields else "'geographic'"
            if not patrol:
                if "reference_mode" not in fields:
                    db.execute("ALTER TABLE maps ADD COLUMN reference_mode TEXT")
                    db.execute("UPDATE maps SET reference_mode=" + legacy_mode)
                if "saved" not in fields:
                    db.execute("ALTER TABLE maps ADD COLUMN saved INTEGER DEFAULT 0")
                    db.execute("UPDATE maps SET saved=EXISTS(SELECT 1 FROM cells WHERE cells.name=maps.name)")
                if cell_fields and "terrain" not in cell_fields:
                    db.execute("ALTER TABLE cells RENAME TO legacy_cells")
                db.execute("CREATE TABLE IF NOT EXISTS cells (name TEXT, terrain TEXT, x INTEGER, y INTEGER, state INTEGER, PRIMARY KEY(name,terrain,x,y))")
                if cell_fields and "terrain" not in cell_fields:
                    db.execute("INSERT INTO cells SELECT name,'',x,y,state FROM legacy_cells")
                    db.execute("DROP TABLE legacy_cells")
                db.execute("INSERT OR IGNORE INTO maps(name,frame,resolution,reference_mode) VALUES (?,?,?,?)", (map_id, frame_id, resolution, reference_mode))
                if legacy_layer is not None:
                    # Rows with an empty profile predate recorded dimensions. Newer observations win.
                    db.execute("INSERT OR IGNORE INTO cells SELECT name,?,x,y,state FROM cells WHERE name=? AND terrain=''", (legacy_layer, map_id))
                    db.execute("DELETE FROM cells WHERE name=? AND terrain=''", (map_id,))
                fields.update(("reference_mode", "saved"))
                cell_fields.add("terrain")
            mode_expr = "reference_mode" if "reference_mode" in fields else legacy_mode
            saved_expr = "saved" if "saved" in fields else "EXISTS(SELECT 1 FROM cells WHERE cells.name=maps.name)"
            row = db.execute(f"SELECT frame,resolution,reference,{mode_expr},{saved_expr} FROM maps WHERE name=?", (map_id,)).fetchone() if fields else None
            if not row or row[:2] != (frame_id, resolution) or row[3] != reference_mode:
                raise ValueError(f"unknown map or inconsistent reference mode, frame or resolution for {map_id}")
            self.reference_json, self.has_saved_map = row[2], bool(row[4])
            self.cells = {}
            if "terrain" in cell_fields:
                if legacy_layer == self.layer:
                    self.cells.update({(x,y): state for x,y,state in db.execute("SELECT x,y,state FROM cells WHERE name=? AND terrain=''", (map_id,))})
                rows = db.execute("SELECT x,y,state FROM cells WHERE name=? AND terrain=?", (map_id, self.layer))
            elif legacy_layer == self.layer:
                rows = db.execute("SELECT x,y,state FROM cells WHERE name=?", (map_id,))
            else:
                rows = ()
            self.cells.update({(x,y): state for x,y,state in rows})
        if patrol and not self.cells:
            raise ValueError(f"no surveyed cells for this terrain profile on {map_id}")

    def cell(self, xy):
        return math.floor(xy[0] / self.resolution), math.floor(xy[1] / self.resolution)

    def center(self, cell):
        return (cell[0] + 0.5) * self.resolution, (cell[1] + 0.5) * self.resolution

    def save(self, changes, reference, *, map_saved=False):
        if self.patrol or (not map_saved and not changes and self.reference_json == reference):
            return
        with closing(sqlite3.connect(self.path)) as db, db:
            db.executemany("INSERT OR REPLACE INTO cells(name,terrain,x,y,state) VALUES (?,?,?,?,?)",
                           [(self.map_id, self.layer, x, y, state) for (x,y),state in changes.items() if state != UNKNOWN])
            db.executemany("DELETE FROM cells WHERE name=? AND terrain=? AND x=? AND y=?",
                           [(self.map_id,self.layer,x,y) for (x,y),state in changes.items() if state == UNKNOWN])
            db.execute("UPDATE maps SET reference=?, saved=MAX(saved,?) WHERE name=?", (reference, int(map_saved or bool(changes)), self.map_id))
        self.has_saved_map |= map_saved or bool(changes)
        self.apply(changes,reference)

    def apply(self, changes, reference):
        for cell,state in changes.items():
            if state == UNKNOWN:
                self.cells.pop(cell,None)
            else:
                self.cells[cell] = state
        self.reference_json = reference

    def export(self, fence):
        by_row = {}
        for x,y in self.cells:
            by_row.setdefault(y, []).append(x)
        for columns in by_row.values():
            columns.sort()
        green, red, fixed = [], [], []
        unknown = 0
        for y, first, last in fence.rows(self.resolution):
            columns = by_row.get(y, [])
            known = columns[bisect_left(columns, first):bisect_right(columns, last)]
            unknown += last-first+1 - len(known)
            red_start = first
            for x in known:
                runs = green if self.cells[(x,y)] == CLEAR else fixed
                if runs and runs[-1][0] == y and runs[-1][2]+1 == x:
                    runs[-1][2] = x
                else:
                    runs.append([y,x,x])
                if self.cells[(x,y)] == CLEAR:
                    if red_start < x:
                        red.append([y,red_start,x-1])
                    red_start = x+1
            if red_start <= last:
                red.append([y,red_start,last])
        return {"map_id": self.map_id, "frame_id": self.frame_id, "resolution_m": self.resolution,
                "terrain": self.terrain, "encoding": "row_x_first_x_last", "green": green, "red": red, "fixed_obstacles": fixed,
                "unverified_area_m2": unknown * self.resolution**2}
