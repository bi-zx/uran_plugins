"""Dependency-free conservative SO101 capsule self-collision guard."""
import copy
import hashlib
import json
import math

from .kinematics import _mat_mul, _origin_transform, _cross, _norm


def _sub(a, b):
    return [x-y for x, y in zip(a, b)]


def _dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def segment_distance(p1, q1, p2, q2):
    """Closest Euclidean distance between two closed line segments."""
    d1, d2, r = _sub(q1, p1), _sub(q2, p2), _sub(p1, p2)
    a, e, f = _dot(d1, d1), _dot(d2, d2), _dot(d2, r)
    clip = lambda v: max(0.0, min(1.0, v))
    if a < 1e-16 and e < 1e-16:
        return math.sqrt(_dot(r, r))
    if a < 1e-16:
        s, t = 0.0, clip(f / e)
    else:
        c = _dot(d1, r)
        if e < 1e-16:
            t, s = 0.0, clip(-c / a)
        else:
            b = _dot(d1, d2)
            denom = a*e-b*b
            s = clip((b*f-c*e)/denom) if denom > 1e-16 else 0.0
            t = (b*s+f)/e
            if t < 0:
                t, s = 0.0, clip(-c/a)
            elif t > 1:
                t, s = 1.0, clip((b-c)/a)
    delta = [r[i] + s*d1[i] - t*d2[i] for i in range(3)]
    return math.sqrt(max(0.0, _dot(delta, delta)))


def transform_point(t, p):
    return [sum(t[i][j]*p[j] for j in range(3)) + t[i][3] for i in range(3)]


def box_clearance(first, second):
    """Maximum normalized separating-axis gap of two OBBs (SAT, 15 axes).

    Positive means separated, negative is the minimum translation penetration
    along a separating axis. Not a mesh signed distance.
    """
    ca, axes_a, ha = first
    cb, axes_b, hb = second
    delta = _sub(cb, ca)
    axes = axes_a + axes_b + [_cross(a, b) for a in axes_a for b in axes_b]
    maximum = -float('inf')
    for axis in axes:
        length = math.sqrt(_dot(axis, axis))
        if length < 1e-8:
            continue
        # Work on the unnormalized axis; normalize the final gap once. Avoid
        # nested generators in this hot path on the robot's Python 3.6 CPU.
        ra = (ha[0]*abs(_dot(axis, axes_a[0])) + ha[1]*abs(_dot(axis, axes_a[1]))
              + ha[2]*abs(_dot(axis, axes_a[2])))
        rb = (hb[0]*abs(_dot(axis, axes_b[0])) + hb[1]*abs(_dot(axis, axes_b[1]))
              + hb[2]*abs(_dot(axis, axes_b[2])))
        maximum = max(maximum, (abs(_dot(delta, axis))-ra-rb)/length)
    return maximum


class CapsuleGuard(object):
    def __init__(self, kinematics, model_path, urdf_path, margin_m=0.0, sample_degrees=0.5):
        self.kinematics = kinematics
        with open(model_path) as stream:
            model = json.load(stream)
        with open(urdf_path, 'rb') as stream:
            if hashlib.sha256(stream.read()).hexdigest() != model['chain_sha256']:
                raise ValueError('collision model/URDF hash mismatch')
        if model['version'] != 1 or not model['capsules']:
            raise ValueError('invalid collision model')
        self.margin = float(margin_m)
        self.sample_degrees = float(sample_degrees)
        if not math.isfinite(self.margin) or self.margin < 0 or not 0 < self.sample_degrees <= 1:
            raise ValueError('invalid collision margin/sample size')
        self.capsules = model['capsules']
        for c in self.capsules:
            if c['link'] not in kinematics.by_child and c['link'] != 'base_link':
                raise ValueError('unknown collision link')
            if not all(math.isfinite(v) for v in c['a'] + c['b'] + [c['radius']]) or c['radius'] <= 0:
                raise ValueError('invalid collision capsule')
            c['origin'] = _origin_transform(c['xyz'], c['rpy'])
        ignored = {frozenset(p) for p in model['ignored_pairs']}
        # Assert the model cannot smuggle nonadjacent link exemptions.
        adjacent = {frozenset((j['parent'], j['child'])) for j in kinematics.by_child.values()}
        if not ignored.issubset(adjacent):
            raise ValueError('collision model exempts nonadjacent links')
        self.pairs = [(i, j) for i, a in enumerate(self.capsules)
                      for j, b in enumerate(self.capsules[:i])
                      if a['link'] != b['link'] and frozenset((a['link'], b['link'])) not in ignored]
        self.last = {}
        self.recovery_bounds = None

    def fork(self):
        """Share immutable geometry, isolate trial results and recovery bounds."""
        other = copy.copy(self)
        other.last = copy.deepcopy(self.last)
        other.recovery_bounds = None if self.recovery_bounds is None else dict(self.recovery_bounds)
        return other

    def distances(self, values):
        transforms = self.kinematics.link_transforms(values[:5], values[5])
        world = []
        for c in self.capsules:
            t = _mat_mul(transforms[c['link']], c['origin'])
            world.append((transform_point(t, c['a']), transform_point(t, c['b']), c['radius'],
                          (transform_point(t, c['box_center']), [[t[i][j] for i in range(3)] for j in range(3)], c['box_half'])))
        result = {}
        for i, j in self.pairs:
            capsule = segment_distance(world[i][0], world[i][1], world[j][0], world[j][1]) - world[i][2] - world[j][2]
            result[(i, j)] = capsule if capsule > self.margin else max(capsule, box_clearance(world[i][3], world[j][3]))
        return result

    def check_path(self, measured, candidate, checkpoint=None, start_distances=None):
        """Sample the whole joint path. Existing overlap may only improve.

        Enclosing capsules may overlap in valid physical poses. The current
        measured pose is a recovery bound, with NO additional penetration
        allowance. No pair is permanently ignored or rebased to a worse goal.
        This is sampled checking, not a proof of continuous collision freedom.
        """
        start = self.distances(measured) if start_distances is None else start_distances
        steps = max(1, int(math.ceil(max(abs(a-b) / (2.0 if i == 5 else self.sample_degrees)
                                       for i, (a, b) in enumerate(zip(measured, candidate))))))
        worst = float('inf')
        overlaps = sum(d < self.margin for d in start.values())
        # Each sample must not deepen an existing penetration; once separated,
        # margin applies for the rest of the path.
        if self.recovery_bounds is None:
            self.recovery_bounds = {p: min(self.margin, d) for p, d in start.items()}
        # Persistent measured-only bounds prevent repeated rebasing from
        # allowing slow penetration creep across successive control frames.
        self.recovery_bounds = {p: max(self.recovery_bounds.get(p, -float('inf')), min(self.margin, d))
                                for p, d in start.items()}
        bounds = dict(self.recovery_bounds)
        for step in range(1, steps + 1):
            if checkpoint:
                checkpoint()
            q = [a + (b-a)*step/steps for a, b in zip(measured, candidate)]
            distances = self.distances(q)
            worst = min(worst, min(distances.values()))
            for pair, d in distances.items():
                if d < bounds[pair] - 1e-7:
                    names = [self.capsules[i]['name'] for i in pair]
                    self.last = {'blocked': True, 'pair': names, 'distance_m': d,
                                 'bound_m': bounds[pair], 'startup_overlaps': overlaps}
                    return False, dict(self.last)
                bounds[pair] = max(bounds[pair], min(self.margin, d))
        self.last = {'blocked': False, 'minimum_distance_m': worst, 'startup_overlaps': overlaps,
                     'recovery_only': overlaps > 0, 'samples': steps}
        return True, dict(self.last)
