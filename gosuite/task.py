"""Generic task-success rules evaluated on a payload pose sequence.

Each task file declares its own rules under ``task``; nothing about a particular geometry is
hard-coded here.  Rules, checked in this order:

1. ``keep_in``  - list of (restricted box R, allowed box A): every part of the payload that lies
   inside R must also lie inside A, at every state.  Tested exactly on 17 payload line segments
   (12 box edges, 4 face-centre lines, the axis) by clipping each segment to R and testing the
   clipped endpoints against the convex box A.  This is what forbids, e.g., entering a
   neighbouring rack bay or a neighbouring rebar window.
2. ``portals``  - ordered openings; a reference point of the payload (its leading end or centre)
   must cross each opening plane in the positive direction, inside the opening rectangle, in
   order, and must still be beyond the plane at the final state.
3. ``goal``     - final payload position / orientation tolerance around the nominal goal pose.
4. ``placement`` (optional) - the payload centreline must lie above each named support with a
   bounded pre-release gap.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import apply, inverse


def payload_segments(size):
    hx, hy, hz = np.asarray(size, dtype=float) / 2
    corners = {(sx, sy, sz): np.array([sx * hx, sy * hy, sz * hz]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)}
    segs = []
    for a, b in [(a, b) for a in corners for b in corners if a < b and sum(i != j for i, j in zip(a, b)) == 1]:
        segs.append([corners[a], corners[b]])
    for y, z in [(0, hz), (0, -hz), (hy, 0), (-hy, 0), (0, 0)]:
        segs.append([[-hx, y, z], [hx, y, z]])
    return np.array(segs, dtype=float)


def world_segments(poses, segments):
    """(T,4,4) poses and (S,2,3) local segments -> (T,S,2,3) world segments."""
    return np.einsum('tij,skj->tski', poses[:, :3, :3], segments) + poses[:, None, None, :3, 3]


def clip_to_box(a, b, half):
    """Slab clipping of segments a->b (box frame, (...,3)) against |x|<=half. Returns t0, t1, valid."""
    d = b - a
    t0 = np.zeros(a.shape[:-1])
    t1 = np.ones(a.shape[:-1])
    for k in range(3):
        da, aa, h = d[..., k], a[..., k], half[k]
        parallel = np.abs(da) < 1e-12
        with np.errstate(divide='ignore', invalid='ignore'):
            te = (-h - aa) / da
            tx = (h - aa) / da
        t0 = np.maximum(t0, np.where(parallel, -np.inf, np.minimum(te, tx)))
        t1 = np.minimum(t1, np.where(parallel, np.inf, np.maximum(te, tx)))
        t1 = np.where(parallel & (np.abs(aa) > h), -1., t1)
    return t0, t1, t0 <= t1


def keep_in_violation(rule, segments_world):
    """First state index violating the keep-in rule, or None."""
    r = np.asarray(rule['restricted']['pose'], dtype=float)
    half_r = np.asarray(rule['restricted']['size'], dtype=float) / 2
    a_pose = np.asarray(rule['allowed']['pose'], dtype=float)
    half_a = np.asarray(rule['allowed']['size'], dtype=float) / 2 + rule.get('tolerance_m', 1e-3)
    inv_r = inverse(r)
    a = apply(inv_r, segments_world[..., 0, :])
    b = apply(inv_r, segments_world[..., 1, :])
    t0, t1, valid = clip_to_box(a, b, half_r)
    to_a = inverse(a_pose) @ r
    p0 = apply(to_a, a + (b - a) * t0[..., None])
    p1 = apply(to_a, a + (b - a) * t1[..., None])
    outside = valid & (np.any(np.abs(p0) > half_a, axis=-1) | np.any(np.abs(p1) > half_a, axis=-1))
    bad = np.flatnonzero(outside.any(axis=1))
    return int(bad[0]) if len(bad) else None


def reference_points(poses, size, which):
    local = np.array([size[0] / 2, 0., 0.]) if which == 'front' else np.zeros(3)
    return np.einsum('tij,j->ti', poses[:, :3, :3], local) + poses[:, :3, 3]


def portal_violation(task, size, poses):
    last = 0
    for portal in task.get('portals', []):
        pts = apply(inverse(np.asarray(portal['pose'], dtype=float)), reference_points(poses, size, portal.get('point', 'front')))
        x = pts[:, 0]
        width, height = portal['size']
        crossing = None
        for i in np.flatnonzero((x[:-1] <= 0) & (x[1:] > 0)):
            if i < last:
                continue
            f = -x[i] / (x[i + 1] - x[i])
            y, z = pts[i, 1:] + f * (pts[i + 1, 1:] - pts[i, 1:])
            if abs(y) <= width / 2 and abs(z) <= height / 2:
                crossing = int(i)
                break
        if crossing is None:
            return 'portal_missed:' + portal['name']
        if x[-1] <= 0:
            return 'portal_backed_out:' + portal['name']
        last = crossing
    return None


def placement_violation(scene, final):
    rule = scene['task'].get('placement')
    if not rule:
        return None
    size = np.asarray(scene['payload']['size'], dtype=float)
    axis, center = final[:3, 0], final[:3, 3]
    obstacles = {o['name']: o for o in scene['obstacles']}
    half_vertical = sum(abs(final[2, j]) * size[j] / 2 for j in (1, 2))
    for name in rule['supports']:
        sup = obstacles[name]
        pose = np.asarray(sup['pose'], dtype=float)
        top = pose[2, 3] + sup['size'][2] / 2
        u = float(np.dot(pose[:3, 3] - center, axis))
        if abs(u) > size[0] / 2 - rule['edge_margin_m']:
            return 'placement_coverage:' + name
        line = center + u * axis
        if np.linalg.norm((line - pose[:3, 3])[:2]) > rule['lateral_tolerance_m']:
            return 'placement_lateral:' + name
        gap = line[2] - half_vertical - top
        if not rule['gap_range_m'][0] <= gap <= rule['gap_range_m'][1]:
            return 'placement_height:' + name
    return None


class TaskChecker:
    """Caches per-task geometry so repeated checks (Monte Carlo episodes) stay cheap."""

    def __init__(self, scene):
        self.scene = scene
        self.task = scene['task']
        self.size = np.asarray(scene['payload']['size'], dtype=float)
        self.segments = payload_segments(self.size)
        goal = self.task['goal']
        self.goal = np.asarray(goal['object_pose'], dtype=float)
        self.goal_position_tol = goal['position_tolerance_m']
        self.goal_angle_tol = goal['orientation_tolerance_rad']

    def state_violation(self, pose):
        """Keep-in rules are state constraints: name of the rule a single payload pose violates, else None."""
        rules = self.task.get('keep_in', [])
        if not rules:
            return None
        segs = world_segments(np.asarray(pose, dtype=float)[None], self.segments)
        for rule in rules:
            if keep_in_violation(rule, segs) is not None:
                return rule['name']
        return None

    def __call__(self, poses):
        """Returns (ok, reason, state). `poses`: (T,4,4) payload poses of the whole execution."""
        poses = np.asarray(poses, dtype=float)
        rules = self.task.get('keep_in', [])
        if rules:
            segs = world_segments(poses, self.segments)
            for rule in rules:
                state = keep_in_violation(rule, segs)
                if state is not None:
                    return False, 'keep_in:' + rule['name'], state
        reason = portal_violation(self.task, self.size, poses)
        if reason:
            return False, reason, None
        final = poses[-1]
        if np.linalg.norm(final[:3, 3] - self.goal[:3, 3]) > self.goal_position_tol:
            return False, 'goal_position', None
        if Rotation.from_matrix(self.goal[:3, :3].T @ final[:3, :3]).magnitude() > self.goal_angle_tol:
            return False, 'goal_orientation', None
        reason = placement_violation(self.scene, final)
        if reason:
            return False, reason, None
        return True, None, None
