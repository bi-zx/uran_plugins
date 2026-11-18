"""Fit paired SLAM positions and GNSS fixes to one metric map frame."""
import math
from collections import deque
from itertools import combinations

from .geo import GeoReferenceConfig, MapProjector


class GnssMapAlignment:
    """Fit WGS84 to map XY. The ENU origin is a (longitude, latitude) pair."""

    def __init__(self, frame_id, origin, reference, config):
        self.frame_id = frame_id
        self.enu = MapProjector(GeoReferenceConfig(enabled=True, reference_lat=origin[1], reference_lon=origin[0]))
        self.projector = None
        if reference:
            self.projector = MapProjector(GeoReferenceConfig.from_dict({**reference, "frame_id": frame_id, "enabled": True}))
        self.baseline = float(config.get("alignment_baseline_m", 3.0))
        self.gate = float(config.get("gnss_residual_m", 1.5))
        self.refresh_ns = float(config.get("gnss_refresh_s", 2.0)) * 1e9
        self._pairs = deque(maxlen=40)
        self.last_stamp = 0
        self.inliers = 0

    @property
    def sample_positions(self):
        return [(sample[2], sample[3]) for sample in self._pairs]

    def clear_samples(self):
        self._pairs.clear()
        self.last_stamp = self.inliers = 0

    def update(self, gps, pose, stamp):
        if pose is None or not (gps.get("horizontal_fix_valid") or int(gps.get("fix_type",0)) >= 3) or stamp <= self.last_stamp:
            return False
        self.last_stamp = stamp
        east, north = self.enu.latlon_to_enu(float(gps["lat"]), float(gps["lon"]))
        x, y = pose["x"], pose["y"]
        if not all(map(math.isfinite, (east, north, x, y))):
            return False
        # Limit the weight of one location, while allowing a later fix to replace an early outlier.
        if self._pairs:
            nearest = min(range(len(self._pairs)), key=lambda i: math.hypot(x-self._pairs[i][2], y-self._pairs[i][3]))
            previous = self._pairs[nearest]
            if math.hypot(x-previous[2], y-previous[3]) < 0.4:
                if stamp-previous[4] < self.refresh_ns:
                    return False
                del self._pairs[nearest]
        self._pairs.append((east, north, x, y, stamp))
        if len(self._pairs) < 3:
            return False
        pairs = [sample[:4] for sample in self._pairs]
        best = []
        best_error = math.inf
        for a, b in combinations(pairs, 2):
            de, dn, dx, dy = (v-u for u,v in zip(a,b))
            if min(math.hypot(de,dn), math.hypot(dx,dy)) < self.baseline:
                continue
            if abs(math.hypot(de,dn) - math.hypot(dx,dy)) > self.gate:
                continue
            angle = math.atan2(dy,dx) - math.atan2(dn,de)
            c, s = math.cos(angle), math.sin(angle)
            support = []
            error = 0.0
            for point in pairs:
                pe, pn, px, py = (v-u for u,v in zip(a,point))
                residual = math.hypot(px - (c*pe-s*pn), py - (s*pe+c*pn))
                if residual <= self.gate:
                    support.append(point)
                    error += residual * residual
            if len(support) > len(best) or (len(support) == len(best) and error < best_error):
                best, best_error = support, error
        if len(best) < max(3, math.ceil(len(pairs) * 0.6)):
            return False
        e0, n0, x0, y0 = [sum(col) / len(best) for col in zip(*best)]
        dot = sum((e-e0)*(x-x0) + (n-n0)*(y-y0) for e,n,x,y in best)
        cross = sum((e-e0)*(y-y0) - (n-n0)*(x-x0) for e,n,x,y in best)
        angle = math.atan2(cross, dot)
        c, s = math.cos(angle), math.sin(angle)
        self.projector = MapProjector(GeoReferenceConfig(enabled=True, frame_id=self.frame_id,
            reference_lat=self.enu.config.reference_lat, reference_lon=self.enu.config.reference_lon,
            map_origin_x=x0-c*e0+s*n0, map_origin_y=y0-s*e0-c*n0, map_yaw_deg=-math.degrees(angle)))
        self.inliers = len(best)
        return True
