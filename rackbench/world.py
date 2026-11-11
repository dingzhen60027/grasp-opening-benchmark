from pathlib import Path
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from gobench.world import World, transform


class RackWorld(World):
    def __init__(self,scene=None,jaw_state='closed'):
        super().__init__(asset_root=Path(__file__).resolve().parents[1]/'assets/ur5e_robotiq',
            model_file='ur5e_open.urdf' if jaw_state=='open' else 'ur5e.urdf',
            contact_link_names=[b'robotiq_85_left_finger_tip_link',b'robotiq_85_right_finger_tip_link'])
        self.names={};self.fixed_object_pose=None
        if jaw_state=='open':self.grasp_contact_links=set()
        if scene is not None:self.load_scene(scene)

    def compound(self,parts,pose):
        """Hollow profiles are unions of walls, not convex hulls closing the holes."""
        shapes=[p.GEOM_BOX]*len(parts)
        halves=[np.asarray(x['size'])/2 for x in parts]
        positions=[x.get('center',[0,0,0]) for x in parts]
        colors=[x.get('color',[.72,.75,.79,1]) for x in parts]
        c=p.createCollisionShapeArray(shapeTypes=shapes,halfExtents=halves,collisionFramePositions=positions,physicsClientId=self.cid)
        v=p.createVisualShapeArray(shapeTypes=shapes,halfExtents=halves,visualFramePositions=positions,rgbaColors=colors,physicsClientId=self.cid)
        return p.createMultiBody(baseMass=0,baseCollisionShapeIndex=c,baseVisualShapeIndex=v,
            basePosition=pose[:3,3],baseOrientation=Rotation.from_matrix(pose[:3,:3]).as_quat(),physicsClientId=self.cid)

    def load_scene(self,scene):
        self.scene=scene
        for item in scene['obstacles']:
            pose=np.array(item['pose'])
            body=self.compound(item['parts'],pose) if 'parts' in item else self.box(item['size'],pose,item['color'])
            self.obstacles.append(body);self.names[body]=item['name']
        self.object=self.compound(scene['object']['parts'],np.eye(4));self.grasp=np.array(scene['object']['T_EO'])
        self.update_object()

    def update_object(self,xi=None):
        if self.fixed_object_pose is None:return super().update_object(xi)
        t=self.fixed_object_pose
        p.resetBasePositionAndOrientation(self.object,t[:3,3],Rotation.from_matrix(t[:3,:3]).as_quat(),physicsClientId=self.cid)
        return t

    def closest_environment(self,q):
        self.set_q(q);self.update_object();best=(float('inf'),None,None)
        for body in [self.robot,self.object]:
            for ob in self.obstacles:
                for c in p.getClosestPoints(body,ob,.1,physicsClientId=self.cid):
                    if c[8]<best[0]:best=(c[8],self.names[ob],'robot' if body==self.robot else 'payload')
        return best

    def ik(self,pose,seed):
        self.set_q(seed)
        # PyBullet's position-only residual can terminate too early for a pure
        # rotation waypoint. Disable that early exit, then check position AND angle.
        q=np.array(p.calculateInverseKinematics(self.robot,self.ee,pose[:3,3],Rotation.from_matrix(pose[:3,:3]).as_quat(),
            maxNumIterations=1200,residualThreshold=0,physicsClientId=self.cid)[:6])
        # Equivalent 2*pi wraps are chosen near the previous state, within UR limits.
        for j in range(6):
            candidates=[q[j]+k*2*np.pi for k in range(-2,3) if self.lower[j]<=q[j]+k*2*np.pi<=self.upper[j]]
            if not candidates:return None
            q[j]=min(candidates,key=lambda v:abs(v-seed[j]))
        self.set_q(q);actual=self.ee_pose()
        if np.linalg.norm(actual[:3,3]-pose[:3,3])>5e-4 or Rotation.from_matrix(actual[:3,:3].T@pose[:3,:3]).magnitude()>2e-3:return None
        return q

    def collision_detail(self,q,xi=None):
        reason=self.collision(q,xi)
        if reason in ['object_environment','robot_environment']:
            body=self.object if reason=='object_environment' else self.robot
            for ob in self.obstacles:
                c=p.getClosestPoints(body,ob,0,physicsClientId=self.cid)
                if c:return dict(reason=reason,obstacle=self.names[ob],depth_m=min(x[8] for x in c))
        if reason=='object_robot':
            for c in p.getClosestPoints(self.robot,self.object,0,physicsClientId=self.cid):
                if c[3] not in self.grasp_contact_links and c[8]<-1e-6:
                    return dict(reason=reason,link=p.getJointInfo(self.robot,c[3],physicsClientId=self.cid)[12].decode(),depth_m=c[8])
        if reason=='robot_self':
            for c in p.getClosestPoints(self.robot,self.robot,0,physicsClientId=self.cid):
                a,b=sorted((c[3],c[4]))
                if a!=b and (a,b) not in self.ignored_self and c[8]<-1e-6:
                    return dict(reason=reason,links=[p.getJointInfo(self.robot,j,physicsClientId=self.cid)[12].decode() if j>=0 else 'base' for j in [a,b]],depth_m=c[8])
        return dict(reason=reason) if reason else None
