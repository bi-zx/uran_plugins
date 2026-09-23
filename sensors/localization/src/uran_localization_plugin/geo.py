"""ROS-independent geographic projection primitives."""

from dataclasses import dataclass
import math
from typing import Any, Dict, Optional, Tuple

EARTH_RADIUS_M = 6378137.0


@dataclass
class GeoReferenceConfig:
    enabled: bool = False
    frame_id: str = "map"
    reference_lat: float = 0.0
    reference_lon: float = 0.0
    map_origin_x: float = 0.0
    map_origin_y: float = 0.0
    map_yaw_deg: float = 0.0
    scale: float = 1.0

    @classmethod
    def from_dict(cls, payload: Optional[Dict[str, Any]]) -> "GeoReferenceConfig":
        payload = payload or {}
        return cls(**{name: payload.get(name, field.default)
                      for name, field in cls.__dataclass_fields__.items()})


class MapProjector:
    def __init__(self, config: GeoReferenceConfig):
        self._config = config

    @property
    def config(self) -> GeoReferenceConfig:
        return self._config

    def is_ready(self) -> bool:
        return self._config.enabled

    def latlon_to_enu(self, lat: float, lon: float) -> Tuple[float, float]:
        ref_lat = math.radians(self._config.reference_lat)
        east = math.radians(lat * 0 + lon - self._config.reference_lon) * EARTH_RADIUS_M * math.cos(ref_lat)
        north = math.radians(lat - self._config.reference_lat) * EARTH_RADIUS_M
        return east, north

    def latlon_to_map_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        if not self.is_ready():
            raise RuntimeError("geo reference is not enabled")
        east, north = self.latlon_to_enu(lat, lon)
        yaw = math.radians(self._config.map_yaw_deg)
        return (
            self._config.map_origin_x + (east * math.cos(yaw) + north * math.sin(yaw)) * self._config.scale,
            self._config.map_origin_y + (-east * math.sin(yaw) + north * math.cos(yaw)) * self._config.scale,
        )

    def map_xy_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        if not self.is_ready():
            raise RuntimeError("geo reference is not enabled")
        if abs(self._config.scale) <= 1e-9:
            raise ValueError("geo reference scale cannot be zero")
        local_x = (x - self._config.map_origin_x) / self._config.scale
        local_y = (y - self._config.map_origin_y) / self._config.scale
        yaw = math.radians(self._config.map_yaw_deg)
        east = local_x * math.cos(yaw) - local_y * math.sin(yaw)
        north = local_x * math.sin(yaw) + local_y * math.cos(yaw)
        lat = self._config.reference_lat + math.degrees(north / EARTH_RADIUS_M)
        cos_lat = math.cos(math.radians(self._config.reference_lat))
        if abs(cos_lat) <= 1e-12:
            raise ValueError("geo projection is undefined at the poles")
        lon = self._config.reference_lon + math.degrees(east / (EARTH_RADIUS_M * cos_lat))
        return lat, lon

    def heading_deg_to_map_yaw(self, heading_deg: Optional[float], heading_mode: str) -> float:
        if heading_deg is None:
            return 0.0
        if heading_mode != "true_north_deg":
            return math.radians(heading_deg)
        return math.radians(90.0 - heading_deg - self._config.map_yaw_deg)

    def waypoint_to_pose(self, waypoint: Any, *, timestamp_ns: int = 0,
                         heading_mode: str = "true_north_deg") -> Dict[str, Any]:
        x, y = self.latlon_to_map_xy(waypoint.lat, waypoint.lon)
        return {
            "frame_id": self._config.frame_id,
            "timestamp_ns": int(timestamp_ns),
            "x": x,
            "y": y,
            "z": float(waypoint.alt),
            "yaw_rad": self.heading_deg_to_map_yaw(waypoint.heading_deg, heading_mode),
        }

    def reference_summary(self) -> Dict[str, Any]:
        return dict(vars(self._config))
