"""Evaluate measured ground and swept body clearance in a gravity-aligned map."""
import math
from collections import defaultdict


MAPPING_RADIUS_M = 5.0

TERRAIN_DEFAULTS = {
    "body_radius_m": 0.40, "body_height_m": 0.45, "max_step_m": 0.06,
    "max_slope_deg": 15.0, "terrain_range_m": 3.0, "terrain_sample_m": 0.10,
}


def terrain_parameters(config):
    params = {key: float(config.get(key, default)) for key, default in TERRAIN_DEFAULTS.items()}
    if not all(math.isfinite(value) and value > 0 for value in params.values()):
        raise ValueError("terrain dimensions and limits must be finite and positive")
    if params["max_step_m"] >= params["body_height_m"] or params["max_slope_deg"] >= 90:
        raise ValueError("invalid step height or slope for the terrain profile")
    return params


def terrain_cells(beams, ground_xyz, resolution, config, body_xy=None):
    """Each beam group has one optical origin and finite measured XYZ endpoints."""
    params = terrain_parameters(config)
    body_radius, body_height = params["body_radius_m"], params["body_height_m"]
    max_step = params["max_step_m"]
    slope = math.tan(math.radians(params["max_slope_deg"]))
    max_range, sample = params["terrain_range_m"], params["terrain_sample_m"]
    tiles_per_cell = math.ceil(resolution/sample)
    size = resolution / tiles_per_cell
    padding = math.ceil(body_radius / size)
    ground_x, ground_y, ground_z = ground_xyz
    surface = defaultdict(list)
    free = set()
    candidates = set()

    for group in beams:
        ox, oy, oz = group["origin"]
        bx, by = group.get("body_xy", body_xy if body_xy is not None else ground_xyz[:2])
        for x, y, z in group["points"]:
            if math.hypot(x-bx, y-by) > MAPPING_RADIUS_M:
                continue
            dx, dy, dz = x-ox, y-oy, z-oz
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            if not 0 < distance <= max_range:
                continue
            tx, ty, tz = math.floor(x/size), math.floor(y/size), math.floor(z/size)
            surface[(tx,ty)].append(z)
            for cx in range((tx-padding)//tiles_per_cell, (tx+padding)//tiles_per_cell+1):
                for cy in range((ty-padding)//tiles_per_cell, (ty+padding)//tiles_per_cell+1):
                    candidates.add((cx,cy))
            # Stop before the return. A missing or saturated return contributes no free space.
            steps = math.ceil(2*distance/size)
            for i in range(1,steps):
                t = i/steps
                voxel = (math.floor((ox+dx*t)/size), math.floor((oy+dy*t)/size), math.floor((oz+dz*t)/size))
                if voxel != (tx,ty,tz):
                    free.add(voxel)

    heights, drops, occupied, tall_surface = {}, set(), set(), set()
    for (tx,ty), hits in surface.items():
        low = min(hits)
        horizontal = math.hypot((tx+0.5)*size-ground_x,(ty+0.5)*size-ground_y)
        tolerance = max_step + slope*horizontal
        if max(hits)-low >= body_height/2:
            tall_surface.add((tx,ty))
        if low < ground_z - tolerance:
            drops.add((tx,ty))
        elif low <= ground_z + tolerance:
            heights[(tx,ty)] = low
            if any(max_step < z-low <= body_height for z in hits):
                occupied.add((tx,ty))
        elif low <= ground_z + tolerance + body_height:
            occupied.add((tx,ty))

    results = []
    for cx,cy in sorted(candidates):
        tiles = {(x,y) for x in range(cx*tiles_per_cell-padding, (cx+1)*tiles_per_cell+padding)
                       for y in range(cy*tiles_per_cell-padding, (cy+1)*tiles_per_cell+padding)}
        supported = tiles.intersection(heights)
        row = {"x_m": (cx+0.5)*resolution, "y_m": (cy+0.5)*resolution, "observed": True}
        jumps = [abs(heights[(x,y)]-heights[n]) for x,y in supported
                 for n in ((x+1,y),(x,y+1)) if n in supported]
        if drops.intersection(tiles) or any(d > max_step for d in jumps):
            row["step_ok"] = False
        elif occupied.intersection(tiles):
            row["occupied"] = True
            if len(tiles.intersection(tall_surface)) >= 3:
                row["vertical_surface"] = True
        elif len(supported) == len(tiles):
            row["ground_continuous"] = True
            row["step_ok"] = True
            row["slope_ok"] = all(d <= slope*size for d in jumps)
            if row["slope_ok"] and all((x,y,k) in free for x,y in tiles for k in
                    range(max(math.floor(heights[(x,y)]/size)+1, math.floor((heights[(x,y)]+max_step)/size)),
                          math.floor((heights[(x,y)]+body_height)/size)+1)):
                row["clearance_ok"] = True
        results.append(row)
    return results
