import copy,json
from pathlib import Path
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from rackbench.world import RackWorld
from rackbench.generate import tube_parts,ee_pose
from rackbench.evaluate import task_success,validate_nominal
from gobench.world import transform,grasp_delta

ROOT=Path(__file__).resolve().parents[1]

def load(name='central_bay_store'):
    s=json.loads((ROOT/'data/rack_v0.2/tasks'/f'{name}.json').read_text())
    qs=np.load(ROOT/'data/rack_v0.2/reference_witnesses'/f'{name}.npy')
    return s,qs

def test_actual_six_joint_model_and_narrow_contact_exemption():
    w=RackWorld()
    try:
        assert len(w.arm)==6
        names={p.getJointInfo(w.robot,j,physicsClientId=w.cid)[12].decode() for j in w.grasp_contact_links}
        assert names=={'robotiq_85_left_finger_tip_link','robotiq_85_right_finger_tip_link'}
        assert p.getNumJoints(w.robot,physicsClientId=w.cid)>20
    finally:w.close()

def test_hollow_tube_has_open_bore_and_colliding_walls():
    w=RackWorld()
    try:
        tube=w.compound(tube_parts(),np.eye(4))
        probe=w.box([.01,.01,.01],np.eye(4),[1,0,0,1])
        assert not p.getClosestPoints(tube,probe,0,physicsClientId=w.cid)
        p.resetBasePositionAndOrientation(probe,[0,.018,0],[0,0,0,1],physicsClientId=w.cid)
        assert p.getClosestPoints(tube,probe,0,physicsClientId=w.cid)
    finally:w.close()

def test_pure_rotation_ik_checks_orientation_as_well_as_position():
    s,qs=load();w=RackWorld(s)
    try:
        w.set_q(qs[0]);target=w.ee_pose();target[:3,:3]=Rotation.from_euler('z',-.03).as_matrix()@target[:3,:3]
        q=w.ik(target,qs[0]);assert q is not None
        assert Rotation.from_matrix(target[:3,:3].T@w.ee_pose()[:3,:3]).magnitude()<.002
    finally:w.close()

def test_reverse_tasks_share_geometry_and_are_not_independent():
    for name in ['central_bay','side_bay','deep_bay']:
        a,qa=load(name+'_store');b,qb=load(name+'_retrieve')
        assert a['geometry_id']==b['geometry_id'] and a['obstacles']==b['obstacles']
        np.testing.assert_array_equal(qa,qb[::-1])

def test_goal_requires_requested_bay_and_supports():
    s,qs=load();w=RackWorld(s)
    try:
        poses=[]
        for q in qs[::8]:w.set_q(q);poses.append(w.update_object())
        w.set_q(qs[-1]);poses.append(w.update_object());poses=np.array(poses)
        assert task_success(s,poses)==(True,None)
        wrong=poses.copy();wrong[-1,1,3]+=.26
        assert task_success(s,wrong)==(False,'wrong_bay')
        hovering=poses.copy();hovering[-1,2,3]+=.03
        relaxed=copy.deepcopy(s);relaxed['task']['goal_position_tolerance_m']=.1
        assert task_success(relaxed,hovering)==(False,'pre_release_height')
    finally:w.close()

def test_aabb_broad_phase_never_drops_detected_pairs():
    s,qs=load();w=RackWorld(s);rng=np.random.default_rng(42)
    try:
        bounds=np.array([p.getAABB(ob,physicsClientId=w.cid) for ob in w.obstacles])
        for q in qs[::60]:
            w.set_q(q);w.update_object(rng.normal(size=6)*np.r_[[.008]*3,[.06]*3])
            lo,hi=np.array(p.getAABB(w.object,physicsClientId=w.cid))
            selected=set(np.flatnonzero(np.all(bounds[:,1]>=lo-.002,axis=1)&np.all(bounds[:,0]<=hi+.002,axis=1)))
            actual={i for i,ob in enumerate(w.obstacles) if p.getClosestPoints(w.object,ob,0,physicsClientId=w.cid)}
            assert actual<=selected
    finally:w.close()

def test_goal_region_accepts_a_valid_nonreference_joint_endpoint():
    s,qs=load();w=RackWorld(s)
    try:
        w.set_q(qs[-1]);goal=w.ee_pose();goal[2,3]+=.001
        q=w.ik(goal,qs[-1]);assert q is not None
        assert np.max(abs(q-np.array(s['q_goal'])))>1e-5
        path=np.vstack([qs,q]);ok,reason,_=validate_nominal(w,s,path)
        assert ok,reason
    finally:w.close()
