"""PyBullet world for one task: UR5e + Robotiq 2F-85, static scene obstacles and a rigid payload.

Collision semantics (identical for witnesses, planners and evaluation):

* robot-environment and payload-environment: any closest point at distance <= margin;
* robot self-collision: penetration deeper than 1e-6 m between links that are neither rigidly
  connected nor adjacent;
* payload-robot: penetration deeper than 1e-6 m between the payload and an ARM link.  Links
  rigidly attached to ``wrist_3_link`` (flange, coupling, gripper) are excluded: the grasp
  perturbation is the payload pose inside the hand and is assumed physically realisable there.

Every narrow-phase query is preceded by a conservative AABB broad phase; tests check that the
broad phase never drops a contact found by brute force.
"""
from pathlib import Path
import itertools
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from .geometry import from_rt, grasp_delta

ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = ROOT / 'assets' / 'ur5e_robotiq'
HOME = np.array([0., -1.5, 1.5, -1.5, -1.57, 0.])
PAD = 0.002  # extra broad-phase padding (m)


class SuiteWorld:
    def __init__(self, scene=None, model_file='ur5e.urdf'):
        self.cid = p.connect(p.DIRECT)
        self.robot = p.loadURDF(str(ASSET_ROOT / model_file), useFixedBase=True,
                                flags=p.URDF_USE_SELF_COLLISION, physicsClientId=self.cid)
        count = p.getNumJoints(self.robot, physicsClientId=self.cid)
        infos = [p.getJointInfo(self.robot, j, physicsClientId=self.cid) for j in range(count)]
        self.link_names = {i[0]: i[12].decode() for i in infos}
        self.link_names[-1] = p.getBodyInfo(self.robot, physicsClientId=self.cid)[0].decode()
        self.arm = [i[0] for i in infos if i[2] == p.JOINT_REVOLUTE]
        if len(self.arm) != 6:
            raise RuntimeError('Expected six revolute arm joints')
        self.tcp = next(i[0] for i in infos if i[12] == b'benchmark_tcp')
        self.lower = np.array([infos[j][8] for j in self.arm])
        self.upper = np.array([infos[j][9] for j in self.arm])
        parents = {i[0]: i[16] for i in infos}
        components = [{i} for i in range(-1, count)]
        for i in infos:
            if i[2] == p.JOINT_FIXED:
                ca = next(c for c in components if i[0] in c)
                cb = next(c for c in components if parents[i[0]] in c)
                if ca is not cb:
                    ca.update(cb)
                    components.remove(cb)
        self.ignored_self = set()
        for a, b in itertools.combinations(range(-1, count), 2):
            ca = next(c for c in components if a in c)
            cb = next(c for c in components if b in c)
            if ca is cb or any(parents.get(x) in cb for x in ca) or any(parents.get(x) in ca for x in cb):
                self.ignored_self.add((a, b))
        wrist3 = next(i[0] for i in infos if i[12] == b'wrist_3_link')
        self.hand_links = set(next(c for c in components if wrist3 in c))
        self.collision_links = [l for l in range(-1, count)
                                if p.getCollisionShapeData(self.robot, l, physicsClientId=self.cid)]
        self.arm_collision_links = [l for l in self.collision_links if l not in self.hand_links]
        self.obstacles, self.names, self.roles = [], {}, {}
        self.payload = None
        self.T_EO = None
        self.payload_T = None
        self._links = None
        self.set_q(HOME)
        if scene is not None:
            self.load_scene(scene)

    # ------------------------------------------------------------------ scene
    def close(self):
        p.disconnect(self.cid)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _body(self, parts, pose):
        shapes = [p.GEOM_BOX] * len(parts)
        halves = [np.asarray(x['size'], dtype=float) / 2 for x in parts]
        centers = [x.get('center', [0, 0, 0]) for x in parts]
        colors = [x.get('color', [.7, .72, .75, 1]) for x in parts]
        col = p.createCollisionShapeArray(shapeTypes=shapes, halfExtents=halves, collisionFramePositions=centers,
                                          physicsClientId=self.cid)
        vis = p.createVisualShapeArray(shapeTypes=shapes, halfExtents=halves, visualFramePositions=centers,
                                       rgbaColors=colors, physicsClientId=self.cid)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                 basePosition=pose[:3, 3], baseOrientation=Rotation.from_matrix(pose[:3, :3]).as_quat(),
                                 physicsClientId=self.cid)

    def load_scene(self, scene):
        self.scene = scene
        for item in scene['obstacles']:
            parts = item.get('parts') or [dict(size=item['size'], center=[0, 0, 0], color=item.get('color'))]
            body = self._body(parts, np.asarray(item['pose'], dtype=float))
            self.obstacles.append(body)
            self.names[body] = item['name']
            self.roles[body] = item.get('role')
        boxes = np.array([p.getAABB(b, physicsClientId=self.cid) for b in self.obstacles]) if self.obstacles else np.zeros((0, 2, 3))
        self.ob_lo, self.ob_hi = boxes[:, 0], boxes[:, 1]
        payload = scene['payload']
        self.payload = self._body(payload['parts'], np.eye(4))
        self.T_EO = np.asarray(payload['T_EO'], dtype=float)
        self.place_payload()

    # -------------------------------------------------------------- kinematics
    def set_q(self, q):
        for j, v in zip(self.arm, q):
            p.resetJointState(self.robot, j, float(v), physicsClientId=self.cid)
        self._links = None

    def ee_pose(self):
        s = p.getLinkState(self.robot, self.tcp, computeForwardKinematics=True, physicsClientId=self.cid)
        return from_rt(Rotation.from_quat(s[5]).as_matrix(), s[4])

    def place_payload(self, xi=None):
        if self.payload is None:
            return None
        delta = np.eye(4) if xi is None else grasp_delta(xi)
        return self.place_payload_pose(self.ee_pose() @ delta @ self.T_EO)

    def place_payload_pose(self, t):
        p.resetBasePositionAndOrientation(self.payload, t[:3, 3], Rotation.from_matrix(t[:3, :3]).as_quat(),
                                          physicsClientId=self.cid)
        self.payload_T = t
        return t

    def ik(self, pose, seed, position_tolerance=5e-4, angle_tolerance=2e-3):
        """Damped-least-squares IK from `seed`; returns the 2*pi-equivalent solution nearest the seed."""
        self.set_q(seed)
        q = np.array(p.calculateInverseKinematics(
            self.robot, self.tcp, pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_quat(),
            maxNumIterations=1200, residualThreshold=0, physicsClientId=self.cid)[:6])
        for j in range(6):
            options = [q[j] + k * 2 * np.pi for k in range(-2, 3) if self.lower[j] <= q[j] + k * 2 * np.pi <= self.upper[j]]
            if not options:
                return None
            q[j] = min(options, key=lambda v: abs(v - seed[j]))
        self.set_q(q)
        actual = self.ee_pose()
        if np.linalg.norm(actual[:3, 3] - pose[:3, 3]) > position_tolerance:
            return None
        if Rotation.from_matrix(actual[:3, :3].T @ pose[:3, :3]).magnitude() > angle_tolerance:
            return None
        return q

    # --------------------------------------------------------------- collision
    def link_aabbs(self):
        if self._links is None:
            boxes = np.array([p.getAABB(self.robot, l, physicsClientId=self.cid) for l in self.collision_links])
            self._links = (boxes[:, 0], boxes[:, 1])
        return self._links

    def robot_environment_hit(self, margin=0.):
        if not self.obstacles:
            return None
        lo, hi = self.link_aabbs()
        m = margin + PAD
        overlap = np.all(hi[:, None, :] >= self.ob_lo[None] - m, axis=2) & np.all(lo[:, None, :] <= self.ob_hi[None] + m, axis=2)
        for li, oi in zip(*np.nonzero(overlap)):
            link, body = self.collision_links[li], self.obstacles[oi]
            if p.getClosestPoints(self.robot, body, margin, linkIndexA=link, physicsClientId=self.cid):
                return dict(reason='robot_environment', obstacle=self.names[body], link=self.link_names[link])
        return None

    def self_hit(self):
        for c in p.getClosestPoints(self.robot, self.robot, 0, physicsClientId=self.cid):
            a, b = sorted((c[3], c[4]))
            if a != b and (a, b) not in self.ignored_self and c[8] < -1e-6:
                return dict(reason='robot_self', links=[self.link_names[a], self.link_names[b]])
        return None

    def payload_environment_hit(self, margin=0.):
        if not self.obstacles:
            return None
        lo, hi = np.array(p.getAABB(self.payload, physicsClientId=self.cid))
        m = margin + PAD
        for oi in np.flatnonzero(np.all(self.ob_hi >= lo - m, axis=1) & np.all(self.ob_lo <= hi + m, axis=1)):
            body = self.obstacles[oi]
            if p.getClosestPoints(self.payload, body, margin, physicsClientId=self.cid):
                return dict(reason='payload_environment', obstacle=self.names[body])
        return None

    def payload_robot_hit(self):
        lo, hi = np.array(p.getAABB(self.payload, physicsClientId=self.cid))
        llo, lhi = self.link_aabbs()
        for li in np.flatnonzero(np.all(lhi >= lo - PAD, axis=1) & np.all(llo <= hi + PAD, axis=1)):
            link = self.collision_links[li]
            if link in self.hand_links:
                continue
            for c in p.getClosestPoints(self.robot, self.payload, 0, linkIndexA=link, physicsClientId=self.cid):
                if c[8] < -1e-6:
                    return dict(reason='payload_robot', link=self.link_names[link])
        return None

    def check(self, q, xi=None, margin=0., scenarios=None, robot_margin=0.):
        """Full state validity. Returns None if valid, else a dict describing the first violation.

        `xi` is the grasp perturbation of the nominal check; `scenarios` (optional, Nx6) adds
        further perturbations that must ALSO be collision-free (payload checks only, since the
        arm does not depend on the grasp).  `margin` inflates payload-environment checks.
        """
        q = np.asarray(q, dtype=float)
        if not np.isfinite(q).all() or np.any(q < self.lower - 1e-9) or np.any(q > self.upper + 1e-9):
            return dict(reason='joint_limits')
        self.set_q(q)
        hit = self.robot_environment_hit(robot_margin) or self.self_hit()
        if hit:
            self.place_payload(xi)
            return hit
        perturbations = [xi] + ([] if scenarios is None else list(scenarios))
        for k, e in enumerate(perturbations):
            self.place_payload(e)
            hit = self.payload_environment_hit(margin) or self.payload_robot_hit()
            if hit:
                if k:
                    hit['scenario'] = k - 1
                    self.place_payload(xi)
                return hit
        if len(perturbations) > 1:
            self.place_payload(xi)
        return None

    def layout_conflict(self, mount_margin=.05):
        """Exact check that station fixtures neither intersect the source fixtures nor crowd the robot mount."""
        groups = {b: o.get('group') for b, o in zip(self.obstacles, self.scene['obstacles'])}
        station = [i for i, b in enumerate(self.obstacles) if groups[b] == 'station']
        for i, b in enumerate(self.obstacles):
            if groups[b] not in ('source', 'cell') or self.names[b] == 'floor':
                continue
            m = mount_margin if groups[b] == 'cell' else 0.
            for j in station:
                if np.all(self.ob_hi[j] >= self.ob_lo[i] - m) and np.all(self.ob_lo[j] <= self.ob_hi[i] + m):
                    if p.getClosestPoints(self.obstacles[j], b, m, physicsClientId=self.cid):
                        return f'layout_overlap:{self.names[self.obstacles[j]]}/{self.names[b]}'
        return None

    def payload_distances(self, cutoff=0.05):
        """{obstacle name: distance} for every obstacle within `cutoff` of the payload."""
        lo, hi = np.array(p.getAABB(self.payload, physicsClientId=self.cid))
        m = cutoff + PAD
        out = {}
        for oi in np.flatnonzero(np.all(self.ob_hi >= lo - m, axis=1) & np.all(self.ob_lo <= hi + m, axis=1)):
            body = self.obstacles[oi]
            pts = p.getClosestPoints(self.payload, body, cutoff, physicsClientId=self.cid)
            if pts:
                out[self.names[body]] = float(min(c[8] for c in pts))
        return out

    def payload_clearance(self, cutoff=0.05):
        """Minimum payload-environment distance (capped at `cutoff`) and the closest obstacle name."""
        d = self.payload_distances(cutoff)
        if not d:
            return float(cutoff), None
        name = min(d, key=d.get)
        return d[name], name
