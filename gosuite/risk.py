"""Vectorised trajectory-level risk under a fixed in-hand grasp error.

Model
-----
Every execution holds one grasp error xi = (dt, phi) for the whole motion, so a payload pose sequence
P_1..P_T fails for xi iff the perturbed payload P_t D(xi) touches the environment (or leaves a keep-in
region) at ANY state t, where D(xi) = T_EO^-1 [Exp(phi), dt] T_EO is the error expressed in the payload
frame.  The failure probability of a path is therefore the measure of a UNION over states,

    p_fail(P) = Pr_xi[ min_t min_o g(P_t D(xi), B_o) <= 0 ],

with g the separation between the payload box and obstacle box B_o.  It is estimated on planning-stream
samples xi_1..xi_N with common random numbers, so candidate paths are compared on the same draws.

Every obstacle of the suite is a box or a compound of boxes, and the payload is a box (a hollow tube is
represented by its outer box, which is exact for collisions with the environment because no obstacle
enters the bore).  The separation g is the separating-axis margin of the two oriented boxes: positive
when they are apart (a lower bound of their distance, exact for face contacts) and minus the smallest
overlap when they intersect.  The sign is exact, so the estimated failure indicator equals the
simulator's payload-environment test up to contact tolerance (checked in tests/test_trio.py).

What the model deliberately leaves out: payload-arm contact (the arm does not depend on xi and the
payload is held in front of the hand), and the arm itself (checked nominally by the planner).
The model reads only obstacle geometry, the payload, the grasp and the keep-in rules of the task file.
"""
import numpy as np
from scipy.stats import norm
from .task import clip_to_box, payload_segments, world_segments
from .geometry import apply, grasp_delta, inverse

EPS = 1e-9
CHUNK = 200_000   # exact (pair, sample) box tests per vectorised block


def obstacle_boxes(scene):
    """(R, c, half) arrays of every obstacle box (compound parts expanded) in the world frame."""
    rots, centres, halves = [], [], []
    for item in scene['obstacles']:
        pose = np.asarray(item['pose'], dtype=float)
        for part in item.get('parts') or [dict(size=item['size'], center=[0., 0., 0.])]:
            rots.append(pose[:3, :3])
            centres.append(pose[:3, :3] @ np.asarray(part.get('center', [0, 0, 0]), dtype=float) + pose[:3, 3])
            halves.append(np.asarray(part['size'], dtype=float) / 2)
    return np.array(rots), np.array(centres), np.array(halves)


def box_separation(RA, cA, a, RB, cB, b):
    """Separating-axis margin of oriented boxes (broadcast over leading dims).

    RA, RB (...,3,3) rotations, cA, cB (...,3) centres, a, b (...,3) half extents.  Returns (...) margins:
    > 0 separated (lower bound of the distance), <= 0 overlapping (minus the smallest overlap)."""
    R = np.einsum('...ji,...jk->...ik', RA, RB)
    T = np.einsum('...ji,...j->...i', RA, cB - cA)
    AR = np.abs(R) + EPS
    a = np.broadcast_to(a, T.shape)
    b = np.broadcast_to(b, T.shape)
    seps = [np.abs(T) - (a + np.einsum('...ij,...j->...i', AR, b)),
            np.abs(np.einsum('...ij,...i->...j', R, T)) - (np.einsum('...ij,...i->...j', AR, a) + b)]
    cross = []
    for i in range(3):
        i1, i2 = (i + 1) % 3, (i + 2) % 3
        for j in range(3):
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            ra = a[..., i1] * AR[..., i2, j] + a[..., i2] * AR[..., i1, j]
            rb = b[..., j1] * AR[..., i, j2] + b[..., j2] * AR[..., i, j1]
            t = np.abs(T[..., i2] * R[..., i1, j] - T[..., i1] * R[..., i2, j])
            length = np.sqrt(np.maximum(1 - R[..., i, j] ** 2, 0.))
            cross.append(np.where(length > 1e-6, (t - ra - rb) / np.maximum(length, 1e-6), -np.inf))
    return np.maximum(np.max(np.concatenate(seps, axis=-1), axis=-1), np.max(np.stack(cross, axis=-1), axis=-1))


def forbidden_boxes(rule):
    """Boxes whose union is R minus (A grown by the rule tolerance), or None if A is not aligned with R."""
    R = np.asarray(rule['restricted']['pose'], dtype=float)
    A = np.asarray(rule['allowed']['pose'], dtype=float)
    rel = inverse(R) @ A
    if not np.allclose(rel[:3, :3], np.eye(3), atol=1e-9):
        return None
    hr = np.asarray(rule['restricted']['size'], dtype=float) / 2
    ha = np.asarray(rule['allowed']['size'], dtype=float) / 2 + rule.get('tolerance_m', 1e-3)
    lo_a, hi_a = rel[:3, 3] - ha, rel[:3, 3] + ha
    pieces = []
    for j in range(3):
        for lo, hi in ((hi_a[j], hr[j]), (-hr[j], lo_a[j])):
            if hi - lo <= 1e-9:
                continue
            centre, half = np.zeros(3), hr.copy()
            centre[j], half[j] = (lo + hi) / 2, (hi - lo) / 2
            pieces.append((R[:3, :3], R[:3, :3] @ centre + R[:3, 3], half))
    return pieces


class RiskModel:
    """Trajectory-level grasp-error risk of payload pose sequences for one task."""

    def __init__(self, scene, errors, slack=.005, ignore=('floor',)):
        self.scene = scene
        payload = scene['payload']
        self.half = np.asarray(payload['size'], dtype=float) / 2
        self.T_EO = np.asarray(payload['T_EO'], dtype=float)
        names = [o['name'] for o in scene['obstacles'] for _ in (o.get('parts') or [0])]
        keep = np.array([n not in ignore for n in names])
        R, c, h = obstacle_boxes(scene)
        self.R, self.c, self.h = R[keep], c[keep], h[keep]
        self.names = [n for n, k in zip(names, keep) if k]
        self.slack = slack
        to_o = inverse(self.T_EO)
        self.errors = np.asarray(errors, dtype=float)
        # payload-frame perturbations D(xi) = T_EO^-1 Delta(xi) T_EO; sample 0 is the nominal grasp
        self.D = np.array([np.eye(4)] + [to_o @ grasp_delta(x) @ self.T_EO for x in self.errors])
        # Largest displacement of any payload point over the samples: |dt| + |phi| * (largest lever arm).
        # A (state, box) pair whose NOMINAL separation exceeds it cannot touch for any sample (exact pruning,
        # since the separating-axis margin never exceeds the true distance).
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * self.half
        lever = np.linalg.norm(corners @ self.T_EO[:3, :3].T + self.T_EO[:3, 3], axis=1).max()
        self.displacement = np.r_[0., [np.linalg.norm(x[:3]) + np.linalg.norm(x[3:]) * lever for x in self.errors]] + 1e-4
        self.reach = float(self.displacement.max())
        # Keep-in rules: the forbidden set R \\ A of an allowed box A aligned with its restricted box R is a
        # union of at most six boxes, which are added as virtual obstacles (exact, and pruned like obstacles).
        # Rules whose boxes are not aligned fall back to the checker's 17-segment test.
        self.keep_in = []
        for rule in scene['task'].get('keep_in', []):
            pieces = forbidden_boxes(rule)
            if pieces is None:
                r, a = rule['restricted'], rule['allowed']
                self.keep_in.append((inverse(np.asarray(r['pose'], dtype=float)), np.asarray(r['size'], dtype=float) / 2,
                                     inverse(np.asarray(a['pose'], dtype=float)) @ np.asarray(r['pose'], dtype=float),
                                     np.asarray(a['size'], dtype=float) / 2 + rule.get('tolerance_m', 1e-3)))
                continue
            for k, (rot, centre, half) in enumerate(pieces):
                self.R = np.r_[self.R, rot[None]]
                self.c = np.r_[self.c, centre[None]]
                self.h = np.r_[self.h, half[None]]
                self.names.append(f"keep_in:{rule['name']}:{k}")
        self.radius = np.linalg.norm(self.h, axis=1)
        self.segments = payload_segments(2 * self.half)

    @property
    def samples(self):
        return len(self.errors)

    def perturbed(self, poses):
        """(T,4,4) payload poses -> (T,N+1,4,4) perturbed poses (index 0 nominal)."""
        return np.einsum('tij,njk->tnik', poses, self.D)

    def pairs(self, poses):
        """(state, box) pairs that can touch for some sample, with their nominal separation.

        Broad phase: box bounding sphere against the payload box grown by its sphere radius and `slack`;
        narrow phase: nominal separating-axis margin below the largest sample displacement `reach`."""
        local = np.einsum('tji,tmj->tmi', poses[:, :3, :3], self.c[None] - poses[:, None, :3, 3])
        outside = np.maximum(np.abs(local) - self.half, 0.)
        ti, oi = np.nonzero(np.linalg.norm(outside, axis=2) < self.radius[None] + self.reach + self.slack)
        nominal = box_separation(poses[ti, :3, :3], poses[ti, :3, 3], self.half, self.R[oi], self.c[oi], self.h[oi])
        near = nominal < self.reach
        return ti[near], oi[near], nominal[near]

    def separations(self, poses):
        """(T, N+1) minimum payload-environment separation per state and sample (inf if nothing is near).

        Only (pair, sample) combinations whose largest displacement reaches the pair's nominal separation
        are tested exactly; for the others the separation is bounded below by nominal - displacement > 0,
        which is what they contribute (so the failure indicator is exact and margins stay informative)."""
        poses = np.asarray(poses, dtype=float)
        ti, oi, nominal = self.pairs(poses)
        out = np.full((len(poses), len(self.D)), np.inf)
        if len(ti):
            bound = nominal[:, None] - self.displacement[None]
            bound[:, 0] = nominal
            k, i = np.nonzero(bound[:, 1:] <= 0)
            i = i + 1
            for a in range(0, len(k), CHUNK):
                kk, ii = k[a:a + CHUNK], i[a:a + CHUNK]
                P = np.einsum('kij,kjl->kil', poses[ti[kk]], self.D[ii])
                bound[kk, ii] = box_separation(P[:, :3, :3], P[:, :3, 3], self.half, self.R[oi[kk]], self.c[oi[kk]], self.h[oi[kk]])
            np.minimum.at(out, ti, bound)
        if self.keep_in:
            states = self.keep_in_states(poses)
            if len(states):
                bad = self.keep_in_violation(self.perturbed(poses[states]))
                out[states] = np.where(bad, np.minimum(out[states], -1e-3), out[states])
        return out

    def keep_in_states(self, poses):
        """States at which some sample could violate a keep-in rule (exact pruning with `reach`).

        A state is skipped when the nominal payload stays `reach` away from every restricted box, or lies
        `reach` inside the allowed box: no sample moves any payload point farther than `reach`."""
        ends = world_segments(poses, self.segments)[:, :, :, :]          # (T,S,2,3)
        pts = ends.reshape(len(poses), -1, 3)
        need = np.zeros(len(poses), bool)
        for inv_r, half_r, to_a, half_a in self.keep_in:
            in_r = apply(inv_r, pts)
            lo, hi = in_r.min(axis=1), in_r.max(axis=1)
            near_r = np.all(hi >= -half_r - self.reach, axis=1) & np.all(lo <= half_r + self.reach, axis=1)
            in_a = apply(to_a, in_r)
            deep = np.all(np.abs(in_a) <= half_a - self.reach, axis=(1, 2))
            need |= near_r & ~deep
        return np.flatnonzero(need)

    def keep_in_violation(self, P):
        """(T, N+1) bool: the perturbed payload leaves an allowed box while inside its restricted box."""
        shape = P.shape[:2]
        segs = world_segments(P.reshape(-1, 4, 4), self.segments)
        bad = np.zeros(len(segs), bool)
        for inv_r, half_r, to_a, half_a in self.keep_in:
            a = apply(inv_r, segs[..., 0, :])
            b = apply(inv_r, segs[..., 1, :])
            t0, t1, valid = clip_to_box(a, b, half_r)
            p0 = apply(to_a, a + (b - a) * t0[..., None])
            p1 = apply(to_a, a + (b - a) * t1[..., None])
            bad |= (valid & (np.any(np.abs(p0) > half_a, axis=-1) | np.any(np.abs(p1) > half_a, axis=-1))).any(axis=1)
        return bad.reshape(shape)

    def final_margins(self, final):
        """(N+1,) margin of the final-state rules for every sample of a nominal final payload pose.

        Goal position / orientation tolerance and the optional placement rule (coverage, lateral offset,
        pre-release gap), each as a signed distance in metres (> 0 satisfied); the orientation margin is
        converted to metres at the payload end (half its length)."""
        task = self.scene['task']
        goal = np.asarray(task['goal']['object_pose'], dtype=float)
        P = np.einsum('ij,njk->nik', final, self.D)
        dp = np.linalg.norm(P[:, :3, 3] - goal[:3, 3], axis=1)
        cos = np.clip((np.einsum('ij,nij->n', goal[:3, :3], P[:, :3, :3]) - 1) / 2, -1, 1)
        margins = [task['goal']['position_tolerance_m'] - dp,
                   (task['goal']['orientation_tolerance_rad'] - np.arccos(cos)) * self.half[0]]
        rule = task.get('placement')
        if rule:
            obstacles = {o['name']: o for o in self.scene['obstacles']}
            size = 2 * self.half
            axis, centre = P[:, :3, 0], P[:, :3, 3]
            half_vertical = sum(np.abs(P[:, 2, j]) * size[j] / 2 for j in (1, 2))
            for name in rule['supports']:
                sup = obstacles[name]
                pose = np.asarray(sup['pose'], dtype=float)
                top = pose[2, 3] + sup['size'][2] / 2
                u = np.einsum('ni,ni->n', pose[:3, 3] - centre, axis)
                line = centre + u[:, None] * axis
                gap = line[:, 2] - half_vertical - top
                margins += [size[0] / 2 - rule['edge_margin_m'] - np.abs(u),
                            rule['lateral_tolerance_m'] - np.linalg.norm((line - pose[:3, 3])[:, :2], axis=1),
                            gap - rule['gap_range_m'][0], rule['gap_range_m'][1] - gap]
        return np.min(margins, axis=0)

    def evaluate(self, poses, final_rules=True):
        """Summary of a payload pose sequence (the last pose is the final state).

        nominal_margin  smallest margin of the nominal payload (collision and, with final_rules, goal rules)
        margins         (N,) per-sample smallest margin over the whole sequence (collision, keep-in, final rules)
        failures        (N,) bool: the sample fails somewhere (trajectory-level event)
        risk            empirical failure probability (trajectory level)
        statewise_risk  max over states of the per-state failure probability (what a state-wise
                        chance constraint bounds; never larger than `risk`)"""
        sep = self.separations(poses)
        if final_rules:
            sep[-1] = np.minimum(sep[-1], self.final_margins(np.asarray(poses)[-1]))
        margins = sep[:, 1:].min(axis=0)
        fail = margins <= 0
        return dict(nominal_margin=float(sep[:, 0].min()), margins=margins, failures=fail, risk=float(fail.mean()),
                    statewise_risk=float((sep[:, 1:] <= 0).mean(axis=1).max()), separations=sep)

    def linearised(self, poses, std, h=1e-4):
        """(T,) first-order Gaussian collision risk per state, summed over nearby boxes (Boole's inequality).

        For each (state, box) pair the separation d(xi) is linearised at xi = 0 by central differences,
        d(xi) ~ d0 + g . xi, so Pr[d <= 0] = Phi(-d0 / sqrt(sum_j g_j^2 std_j^2)).  This is the per-waypoint
        risk estimate of linear-Gaussian chance-constrained planners; it is used here as a comparator only."""
        poses = np.asarray(poses, dtype=float)
        ti, oi, d0 = self.pairs(poses)
        out = np.zeros(len(poses))
        if not len(ti):
            return out
        to_o = inverse(self.T_EO)
        grads = []
        for j in range(6):
            e = np.zeros(6)
            e[j] = h
            seps = []
            for sign in (1., -1.):
                P = poses[ti] @ (to_o @ grasp_delta(sign * e) @ self.T_EO)
                seps.append(box_separation(P[:, :3, :3], P[:, :3, 3], self.half, self.R[oi], self.c[oi], self.h[oi]))
            grads.append((seps[0] - seps[1]) / (2 * h))
        sigma = np.sqrt(np.sum((np.array(grads).T * np.asarray(std)) ** 2, axis=1)) + 1e-12
        np.add.at(out, ti, norm.sf(d0 / sigma))
        return out
