"""Map-frame terrain observations and footprints swept by a mobile base."""
import math
from typing import NamedTuple

from . import _terrain_rays
from .terrain import MAPPING_RADIUS_M


class TerrainFrame(NamedTuple):
    stamp: int
    data: object


def _clip(poly, axis, limit, side):
    """Clip a convex polygon against one coordinate half-plane."""
    out = []
    prev = poly[-1]
    for p in poly:
        a, b = side*(prev[axis]-limit), side*(p[axis]-limit)
        if (a < 0) != (b < 0):
            t = a/(a-b)
            out.append((prev[0]+t*(p[0]-prev[0]), prev[1]+t*(p[1]-prev[1])))
        if b >= 0:
            out.append(p)
        prev = p
    return out


class SweptTerrain:
    def __init__(self, params):
        self.size = params["terrain_sample_m"]
        self.range = min(MAPPING_RADIUS_M,params["terrain_range_m"])
        self.data = _terrain_rays.create(self.size,params["max_step_m"],params["body_height_m"],
                                         self.range,math.tan(math.radians(params["max_slope_deg"])))
        self.measured = False

    def project(self, group, ground, body_xy, stamp):
        body = group.get("body_xy",body_xy)
        data = _terrain_rays.project(self.data,tuple(group["origin"]),tuple(ground),*body,
                                     group["points"],group.get("free_ends",()),stamp)
        return TerrainFrame(stamp,data)

    def merge(self, frame):
        self.measured = _terrain_rays.merge(self.data,frame.data)

    def covers_cell(self, cell, resolution, padding):
        return _terrain_rays.cell(self.data,*cell,resolution,padding)

    def area(self, tiles, hits_only=False):
        return _terrain_rays.area(self.data,tiles,hits_only)

    def straight(self, pose, length, width, distance, recent=0):
        return _terrain_rays.straight(self.data,*pose,length,width,distance,recent)

    def polygon_tiles(self, poly):
        """All map squares with positive-area intersection, including along a thin braking strip."""
        size = self.size
        rows = set()
        low, high = min(y for x,y in poly), max(y for x,y in poly)
        for iy in range(math.floor(low/size+1e-9), math.ceil(high/size-1e-9)):
            strip = _clip(poly,1,iy*size,1)
            if strip:
                strip = _clip(strip,1,(iy+1)*size,-1)
            if strip:
                x0, x1 = min(x for x,y in strip), max(x for x,y in strip)
                rows.update((ix,iy) for ix in range(math.floor(x0/size+1e-9), math.ceil(x1/size-1e-9)))
        return rows

    def sweep(self, pose, length, width, vx, wz, duration, recent=0):
        """Check the new area of the padded footprint. Any measured hit in the full sweep vetoes."""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        if not wz:
            return self.straight(pose,length,width,vx*duration,recent)
        corners = [(length,width),(-length,width),(-length,-width),(length,-width)]
        def world(poly):
            return [(x+c*u-s*v,y+s*u+c*v) for u,v in poly]
        swept = set()
        whole = self.polygon_tiles(world(corners))
        # Follow both sides of the actual arc. Each swept face contributes only its
        # portion outside the starting rectangle, including newly exposed side strips.
        steps = max(1, math.ceil((abs(vx)+abs(wz)*math.hypot(length,width))*duration/(self.size/4)))
        previous = corners
        for i in range(1,steps+1):
            t = duration*i/steps
            a = wz*t
            ca, sa = math.cos(a), math.sin(a)
            ox = vx*math.sin(a)/wz if wz else vx*t
            oy = vx*2*math.sin(a/2)**2/wz if wz else 0.
            current = [(ox+ca*u-sa*v,oy+sa*u+ca*v) for u,v in corners]
            for j in range(4):
                # Two triangles also cover a face whose corners move in opposite directions.
                for tri in ([previous[j-1],previous[j],current[j]], [previous[j-1],current[j],current[j-1]]):
                    for axis,limit,side in ((0,length,1),(0,-length,-1),(1,width,1),(1,-width,-1)):
                        if max(side*(p[axis]-limit) for p in tri) <= 1e-10:
                            continue
                        swept.update(self.polygon_tiles(world(_clip(tri,axis,limit,side))))
            previous = current
        if self.area(whole | swept,True) == "blocked":
            return "blocked"
        return self.area(swept)

    def turn(self, pose, radius, length, body_width):
        x, y, yaw = pose
        c, s, size = math.cos(yaw), math.sin(yaw), self.size
        disk, outside = set(), set()
        for ix in range(math.floor((x-radius)/size),math.ceil((x+radius)/size)):
            for iy in range(math.floor((y-radius)/size),math.ceil((y+radius)/size)):
                x0, y0 = ix*size-x, iy*size-y
                if max(x0,0.,-x0-size)**2 + max(y0,0.,-y0-size)**2 >= radius**2:
                    continue
                disk.add((ix,iy))
                if any(abs(u*c+v*s)>length+1e-9 or abs(v*c-u*s)>body_width+1e-9
                       for u in (x0,x0+size) for v in (y0,y0+size)):
                    outside.add((ix,iy))
        return "blocked" if self.area(disk,True) == "blocked" else self.area(outside)
