from pathlib import Path
import hashlib
import itertools
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation


def transform(position, quaternion):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    t[:3, 3] = position
    return t


def grasp_delta(xi):
    """Translation in EE metres and rotation vector in EE radians, about EE origin.

    This is [Exp_SO3(phi), delta_t], NOT the SE(3) exponential of a twist.
    Left multiplication of T_EO rotates its offset (the grasp lever arm).
    """
    return transform(np.asarray(xi)[:3], Rotation.from_rotvec(np.asarray(xi)[3:]).as_quat())


def densify(path, step=0.012):
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 6 or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError("Expected a finite Nx6 joint path with at least two states")
    if step <= 0:
        raise ValueError("step must be positive")
    out = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, int(np.ceil(np.max(np.abs(b-a)) / step)))
        out.extend(a+(b-a)*i/n for i in range(1, n+1))
    return np.array(out)


class World:
    def __init__(self, scene=None, asset_root=None, contact_link_names=None, model_file='ur5e.urdf'):
        self.cid = p.connect(p.DIRECT)
        self.asset_root=Path(asset_root) if asset_root else Path(__file__).resolve().parents[1]/'assets/ur5e'
        self.robot = p.loadURDF(str(self.asset_root/model_file), useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION, physicsClientId=self.cid)
        count=p.getNumJoints(self.robot,physicsClientId=self.cid)
        infos=[p.getJointInfo(self.robot,j,physicsClientId=self.cid) for j in range(count)]
        self.arm=[i[0] for i in infos if i[2]==p.JOINT_REVOLUTE]
        assert len(self.arm)==6
        self.ee=next(i[0] for i in infos if i[12]==b'benchmark_tcp')
        self.lower = np.array([p.getJointInfo(self.robot,j,physicsClientId=self.cid)[8] for j in self.arm])
        self.upper = np.array([p.getJointInfo(self.robot,j,physicsClientId=self.cid)[9] for j in self.arm])
        self.home = np.array([0,-1.5,1.5,-1.5,-1.57,0])
        self.obstacles = []
        self.object = None
        # Ignore only rigidly connected / directly adjacent robot links.
        parents = {j:p.getJointInfo(self.robot,j,physicsClientId=self.cid)[16] for j in range(count)}
        fixed_edges = [(j,parents[j]) for j in range(count)
            if p.getJointInfo(self.robot,j,physicsClientId=self.cid)[2] == p.JOINT_FIXED]
        components = [{i} for i in range(-1,count)]
        for a,b in fixed_edges:
            ca=next(c for c in components if a in c); cb=next(c for c in components if b in c)
            if ca is not cb:
                ca.update(cb); components.remove(cb)
        self.ignored_self = set()
        for a,b in itertools.combinations(range(-1,count),2):
            ca=next(c for c in components if a in c); cb=next(c for c in components if b in c)
            if ca is cb or any(parents.get(x) in cb for x in ca) or any(parents.get(x) in ca for x in cb):
                self.ignored_self.add((a,b))
        # Synthetic fixture contacts payload by design; other arm-object contacts count.
        contact_link_names = contact_link_names or [b'benchmark_gripper',b'benchmark_tcp']
        self.grasp_contact_links = {i[0] for i in infos if i[12] in contact_link_names}
        self.set_q(self.home)
        if scene is not None:
            self.load_scene(scene)

    def close(self):
        p.disconnect(self.cid)

    def box(self, size, pose, color):
        col=p.createCollisionShape(p.GEOM_BOX, halfExtents=np.array(size)/2, physicsClientId=self.cid)
        vis=p.createVisualShape(p.GEOM_BOX, halfExtents=np.array(size)/2, rgbaColor=color, physicsClientId=self.cid)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
            basePosition=pose[:3,3],baseOrientation=Rotation.from_matrix(pose[:3,:3]).as_quat(), physicsClientId=self.cid)

    def load_scene(self, scene):
        self.scene=scene
        for o in scene['obstacles']:
            self.obstacles.append(self.box(o['size'],np.array(o['pose']),[.45,.55,.65,1]))
        obj=scene['object']
        self.object=self.box(obj['size'],np.eye(4),[.95,.58,.08,1])
        self.grasp=np.array(obj['T_EO'])
        self.update_object()

    def set_q(self,q):
        for j,v in zip(self.arm,q):
            p.resetJointState(self.robot,j,float(v),physicsClientId=self.cid)

    def ee_pose(self):
        s=p.getLinkState(self.robot,self.ee,computeForwardKinematics=True,physicsClientId=self.cid)
        return transform(s[4],s[5])

    def update_object(self,xi=None):
        if self.object is None:
            return None
        t=self.ee_pose() @ grasp_delta(np.zeros(6) if xi is None else xi) @ self.grasp
        p.resetBasePositionAndOrientation(self.object,t[:3,3],Rotation.from_matrix(t[:3,:3]).as_quat(),physicsClientId=self.cid)
        return t

    def ik(self,pose,seed):
        self.set_q(seed)
        q=np.array(p.calculateInverseKinematics(self.robot,self.ee,pose[:3,3],Rotation.from_matrix(pose[:3,:3]).as_quat(),
            maxNumIterations=1000,residualThreshold=1e-8,physicsClientId=self.cid)[:6])
        self.set_q(q)
        actual=self.ee_pose()
        pe=np.linalg.norm(actual[:3,3]-pose[:3,3])
        re=Rotation.from_matrix(actual[:3,:3].T @ pose[:3,:3]).magnitude()
        if np.any(q<self.lower) or np.any(q>self.upper) or pe>5e-4 or re>2e-3:
            return None
        return q

    def collision(self,q,xi=None,margin=0.):
        if not np.isfinite(q).all() or np.any(q<self.lower) or np.any(q>self.upper):
            return 'joint_limits'
        self.set_q(q); self.update_object(xi)
        for ob in self.obstacles:
            if p.getClosestPoints(self.robot,ob,margin,physicsClientId=self.cid):
                return 'robot_environment'
            if p.getClosestPoints(self.object,ob,margin,physicsClientId=self.cid):
                return 'object_environment'
        for c in p.getClosestPoints(self.robot,self.robot,0,physicsClientId=self.cid):
            a,b=sorted((c[3],c[4]))
            if a!=b and (a,b) not in self.ignored_self and c[8]<-1e-6:
                return 'robot_self'
        for c in p.getClosestPoints(self.robot,self.object,0,physicsClientId=self.cid):
            if c[3] not in self.grasp_contact_links and c[8]<-1e-6:
                return 'object_robot'
        return None

    def asset_fingerprint(self):
        root=self.asset_root
        files={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest()
            for f in sorted(root.rglob('*')) if f.is_file()}
        return dict(source='assets/ur5e/provenance.json',license='assets/ur5e/LICENSE',files=files)
