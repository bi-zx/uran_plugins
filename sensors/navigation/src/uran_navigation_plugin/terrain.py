"""Evaluate measured ground and swept body clearance in a gravity-aligned map."""
import math
from . import _terrain_rays


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
    return _terrain_rays.classify(beams,ground_xyz,ground_xyz[:2] if body_xy is None else body_xy,resolution,
                                  params["body_radius_m"],params["body_height_m"],params["max_step_m"],
                                  math.tan(math.radians(params["max_slope_deg"])),params["terrain_range_m"],params["terrain_sample_m"])
