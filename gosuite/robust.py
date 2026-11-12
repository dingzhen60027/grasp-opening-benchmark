"""Risk-aware witness optimisation for the v0.4 generator.

A v0.4 family marks one segment of its scripted TCP knots as the insertion (pre-entry -> goal line)
and declares which deviations a robust path may use: lateral / vertical offsets and yaw / pitch about
the TCP, in an insertion frame A (x along the insertion).  The optimiser perturbs the knots of that
segment with a cross-entropy search and scores each candidate by the fraction of PLANNING-stream
grasp draws that make the payload touch the environment (or leave a keep-in region) anywhere along
the segment.  The evaluation draws are never used, so evaluating the selected path is unbiased.

Under a fixed in-hand error the payload deviates by about dt + phi x p at payload point p, i.e. in
proportion to the lever arm.  The optimiser therefore learns to keep the payload tip (largest lever)
away from obstacles while it passes them and to converge to the goal line only when short-lever
points are next to the obstacles.  The resulting witness is generator-side oracle evidence of what a
risk-aware path can achieve; planners never receive it.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from .evaluate import planning_errors
from .geometry import from_rt, grasp_delta, interpolate_pose, inverse, pose_distance

DIMENSIONS = ('dy', 'dz', 'yaw', 'pitch')
GENERATOR_PLANNING_SEED = 9001


def perturb(pose, frame, offset):
    """Apply (dy, dz, yaw, pitch) in insertion frame `frame` to a TCP pose, rotating about the TCP."""
    dy, dz, yaw, pitch = offset
    r = frame[:3, :3]
    rot = Rotation.from_rotvec(r[:, 2] * yaw) * Rotation.from_rotvec(r[:, 1] * pitch)
    out = pose.copy()
    out[:3, :3] = rot.as_matrix() @ pose[:3, :3]
    out[:3, 3] = pose[:3, 3] + r[:, 1] * dy + r[:, 2] * dz
    return out


class InsertionSpace:
    """Maps a parameter vector to the TCP knots of a perturbed insertion segment.

    Two parameterisations:
    * TCP-relative (default): per knot, (dy, dz, yaw, pitch) applied at the TCP;
    * line-through-control-depths (`control_depths=(x1, x2)` along the frame x axis): per knot and per
      active plane (lateral 'y' and/or vertical 'z'), the payload line's offset from the goal line at
      the two control depths.  Clearances at obstacles located at those depths are then direct,
      decoupled parameters, which makes the risk landscape far better conditioned for the search.
    """

    def __init__(self, start, end, frame, active=('dy', 'yaw'), max_offset=.04, max_angle=np.deg2rad(6.), fractions=(0., .3, .55, .8),
                 control_depths=None, planes=('y',)):
        self.start, self.end, self.frame = start, end, frame
        self.fractions = fractions
        self.control = None if control_depths is None else np.asarray(control_depths, dtype=float)
        if self.control is None:
            self.active = [DIMENSIONS.index(a) for a in active]
            limits = np.array([max_offset, max_offset, max_angle, max_angle])
            self.per_knot = len(self.active)
            self.bounds = np.tile(limits[self.active], len(fractions))
        else:
            self.planes = [{'y': 1, 'z': 2}[p] for p in planes]
            self.per_knot = 2 * len(self.planes)
            self.bounds = np.full(self.per_knot * len(fractions), max_offset)
        self.dim = len(self.bounds)

    def _line_pose(self, pose, row):
        """TCP pose whose tool axis is the line through the two offset control points (per plane)."""
        r = self.frame[:3, :3]
        x_tcp = float(np.dot(pose[:3, 3] - self.frame[:3, 3], r[:, 0]))
        x1, x2 = self.control
        angles, shift = {}, np.zeros(3)
        for k, axis in enumerate(self.planes):
            o1, o2 = row[2 * k], row[2 * k + 1]
            slope = (o2 - o1) / (x2 - x1)
            shift += r[:, axis] * (o1 + slope * (x_tcp - x1))
            angles[axis] = np.arctan(slope)
        rot = Rotation.from_rotvec(r[:, 2] * angles.get(1, 0.)) * Rotation.from_rotvec(-r[:, 1] * angles.get(2, 0.))
        out = pose.copy()
        out[:3, :3] = rot.as_matrix() @ pose[:3, :3]
        out[:3, 3] = pose[:3, 3] + shift
        return out

    def knots(self, theta):
        theta = np.clip(np.asarray(theta, dtype=float), -self.bounds, self.bounds).reshape(len(self.fractions), self.per_knot)
        out = []
        for u, row in zip(self.fractions, theta):
            nominal = interpolate_pose(self.start, self.end, u)
            if self.control is None:
                offset = np.zeros(4)
                offset[self.active] = row
                out.append(perturb(nominal, self.frame, offset))
            else:
                out.append(self._line_pose(nominal, row))
        if self.fractions[-1] < 1:
            out.append(self.end.copy())
        return out


def payload_poses(tcp_knots, T_EO, linear_step=.004, angular_step=.012):
    poses = []
    for a, b in zip(tcp_knots[:-1], tcp_knots[1:]):
        d, ang = pose_distance(a, b)
        n = max(2, int(np.ceil(d / linear_step)), int(np.ceil(ang / angular_step)))
        poses.extend(interpolate_pose(a, b, u) @ T_EO for u in np.linspace(0, 1, n, endpoint=False))
    poses.append(tcp_knots[-1] @ T_EO)
    return poses


def segment_risk(world, checker, poses, right_deltas):
    """(nominal_ok, risk) for a payload pose sequence (payload-environment + keep-in).

    If the nominal sequence itself collides, `risk` is 1 + the fraction of colliding poses, a graded
    penalty that lets the search move out of collision instead of seeing a flat wall.
    """
    bad = 0
    for pose in poses:
        world.place_payload_pose(pose)
        if world.payload_environment_hit() or checker.state_violation(pose):
            bad += 1
    if bad:
        return False, 1. + bad / len(poses)
    fails = 0
    for delta in right_deltas:
        for pose in poses:
            t = pose @ delta
            world.place_payload_pose(t)
            if world.payload_environment_hit() or checker.state_violation(t):
                fails += 1
                break
    return True, fails / len(right_deltas)


def tip_fractions(start, end, lever, axis, depths, extra=(0.,)):
    """Insertion fractions at which the payload tip reaches each given depth along `axis` (plus extras)."""
    x0, x1 = float(np.dot(start[:3, 3], axis)), float(np.dot(end[:3, 3], axis))
    us = [(d - lever - x0) / (x1 - x0) for d in depths]
    mids = [(a + b) / 2 for a, b in zip(sorted(us), sorted(us)[1:] + [1.])]
    return tuple(sorted({round(float(np.clip(u, 0., .95)), 3) for u in list(extra) + us + mids}))


def planning_deltas(scene, T_EO, samples=80):
    """Payload-frame perturbations from the planning stream reserved for the generator (seed 9001)."""
    return [inverse(T_EO) @ grasp_delta(xi) @ T_EO for xi in planning_errors(scene, samples, GENERATOR_PLANNING_SEED)]


def straight_risk(world, checker, scene, space, samples=80):
    """Planning-draw risk of the unperturbed (straight) insertion segment."""
    ok, risk = segment_risk(world, checker, payload_poses(space.knots(np.zeros(space.dim)), world.T_EO),
                            planning_deltas(scene, world.T_EO, samples))
    return risk if ok else None


def optimise(world, checker, scene, space, samples=80, iterations=8, population=32, elite=7, seed=0, regularisation=.02):
    """Cross-entropy search. Returns candidates [(cost, risk, theta)] sorted best first (theta=0 included)."""
    rng = np.random.default_rng(seed)
    T_EO = world.T_EO
    deltas = planning_deltas(scene, T_EO, samples)
    cache = []

    def cost(theta):
        ok, risk = segment_risk(world, checker, payload_poses(space.knots(theta), T_EO), deltas)
        c = risk + regularisation * float(np.mean((np.asarray(theta) / space.bounds) ** 2))
        cache.append((c, risk if ok else None, np.asarray(theta, dtype=float)))
        return c

    cost(np.zeros(space.dim))
    mean, std = np.zeros(space.dim), space.bounds / 3
    for _ in range(iterations):
        thetas = np.clip(mean + rng.normal(size=(population, space.dim)) * std, -space.bounds, space.bounds)
        scored = sorted(((cost(t), i) for i, t in enumerate(thetas)), key=lambda x: x[0])
        best = thetas[[i for _, i in scored[:elite]]]
        mean, std = best.mean(axis=0), best.std(axis=0) + 1e-3 * space.bounds
    cost(mean)
    return sorted((c for c in cache if c[1] is not None), key=lambda c: c[0])
