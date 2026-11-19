"""Planar clearance from a scan expressed at the centre of a rectangular base."""
import math


class FootprintScan:
    def __init__(self, scan, angle_offset, half_length, half_width):
        self.half_length, self.half_width = half_length, half_width
        self.range_max = float(scan.range_max)
        self.hits = []
        self.valid_count = 0
        self.origin = scan.angle_min + angle_offset
        increment = scan.angle_increment
        count = max(1, round(2*math.pi/abs(increment)))
        self.step = 2*math.pi/count
        self.observed = bytearray(count)
        for i, value in enumerate(scan.ranges):
            if value is None:
                continue
            r = float(value)
            if not (r == math.inf or 0 < r <= self.range_max and r >= scan.range_min):
                continue
            self.valid_count += 1
            angle = self.origin + i*increment
            self.observed[round(i*increment/self.step) % count] = 1
            if math.isfinite(r):
                self.hits.append((r*math.cos(angle), r*math.sin(angle), r))
        self.nearest = min((r for _, _, r in self.hits), default=self.range_max)

    def corridor(self, heading, stop_distance, min_valid_ratio):
        """Return longitudinal clearance and coverage at the front and both front corners."""
        c, s = math.cos(heading), math.sin(heading)
        clearance = self.range_max
        for x, y, _ in self.hits:
            along, lateral = x*c + y*s, -x*s + y*c
            if along >= -self.half_length and abs(lateral) < self.half_width - 1e-8:
                clearance = min(clearance, max(0.0, along))
        half_angle = math.atan2(self.half_width, self.half_length)
        count = len(self.observed)
        covered = self.range_max >= math.hypot(stop_distance, self.half_width)
        # Check left, centre and right separately so a blind side cannot borrow centre rays.
        edges = [round((heading - self.origin + half_angle*(2*i/3 - 1))/self.step) for i in range(4)]
        for start, end in zip(edges, edges[1:]):
            indices = range(start, max(start+1, end))
            valid = sum(self.observed[i % count] for i in indices)
            covered = covered and valid > 0 and valid >= min_valid_ratio*len(indices)
        return clearance, covered

    def arc_blocked(self, vx, wz, duration):
        """Sweep the padded rectangle along the commanded arc up to braking."""
        corner = math.hypot(self.half_length, self.half_width)
        nearby = [(x,y) for x,y,r in self.hits if r <= corner + vx*duration]
        if not nearby:
            return False
        travel = (vx + abs(wz)*corner)*duration
        steps = max(1, math.ceil(travel/0.02))
        # The closest sampled pose is at most half a step from any point in the sweep.
        pad = travel/(2*steps)
        radius = vx/wz
        for i in range(steps+1):
            angle = wz*duration*i/steps
            c, s = math.cos(angle), math.sin(angle)
            ox, oy = radius*s, 2*radius*math.sin(angle/2)**2
            for x,y in nearby:
                dx, dy = x-ox, y-oy
                if abs(dx*c+dy*s) < self.half_length+pad and abs(dy*c-dx*s) < self.half_width+pad:
                    return True
        return False

    def turn_blocker(self, radius, map_clearance):
        if self.nearest < radius - 1e-8:
            return 'obstacle inside turning radius'
        if map_clearance is not None:
            return map_clearance(radius)
        if self.range_max < radius or not all(self.observed):
            return 'turning disk has unobserved sectors'
        return ''
