"""Whole-transfer validation with bay occupancy, support coverage and grasp errors."""
import argparse,hashlib,itertools,json,time
from collections import Counter
from pathlib import Path
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from gobench.world import densify,grasp_delta
from gobench.evaluate import draw_errors,interval
from gobench.generate import write_json
from .world import RackWorld

def vertices(scene,poses):
    corners=np.array(list(itertools.product([-1,1],repeat=3)))*np.array(scene['object']['size'])/2
    return np.einsum('tij,kj->tki',poses[:,:3,:3],corners)+poses[:,:3,3][:,None,:]

def task_success(scene,poses):
    task=scene['task'];rack=scene['rack'];v=vertices(scene,poses);store=task['operation']=='store'
    outside=v[0] if store else v[-1];inside=v[-1] if store else v[0]
    if outside[:,0].max()>rack['front_x']-.015:return False,'not_fully_withdrawn'
    if inside[:,0].min()<rack['front_x']+task['min_final_rear_depth_m']:return False,'not_fully_stored'
    if inside[:,0].max()>rack['back_x']-.01:return False,'beyond_back_stop'
    if abs(inside[:,1]-rack['target_bay_y']).max()>rack['target_bay_half_width_m']:return False,'wrong_bay'
    leading=np.max(v[:,:,0],axis=1);inside_indices=np.flatnonzero(leading>rack['front_x'])
    if not len(inside_indices):return False,'never_entered_rack'
    cross=inside_indices[0] if store else inside_indices[-1];cv=v[cross]
    if np.max(abs(cv[:,1]))>.527 or cv[:,2].min()<.169 or cv[:,2].max()>.471:return False,'wrong_facade_crossing'
    goal=np.array(task['nominal_object_goal_pose']);actual=poses[-1]
    if np.linalg.norm(actual[:3,3]-goal[:3,3])>task['goal_position_tolerance_m']:return False,'goal_position'
    if Rotation.from_matrix(goal[:3,:3].T@actual[:3,:3]).magnitude()>task['goal_orientation_tolerance_rad']:return False,'goal_orientation'
    # Tube centerline above each saddle: 2--20 mm bottom clearance before release.
    if store:
        axis=actual[:3,0];length=scene['object']['size'][0]
        for ob in scene['obstacles']:
            if ob['role']!='target_support':continue
            center=np.array(ob['pose'])[:3,3];u=(center[0]-actual[0,3])/axis[0]
            line=actual[:3,3]+u*axis
            if abs(u)>length/2-.025 or abs(line[1]-center[1])>.025:return False,'missed_support'
            half_vertical=sum(abs(actual[2,j])*scene['object']['size'][j]/2 for j in [1,2])
            gap=line[2]-half_vertical-(center[2]+ob['size'][2]/2)
            if not .002<=gap<=.020:return False,'pre_release_height'
    return True,None

def validate_nominal(w,scene,path):
    if np.max(abs(path[0]-scene['q_start']))>1e-5:return False,'wrong_start',None
    poses=[]
    for q in path:
        detail=w.collision_detail(q)
        if detail:return False,detail,None
        poses.append(w.update_object())
    poses=np.array(poses);ok,reason=task_success(scene,poses)
    return ok,reason,poses

def evaluate(scene,path,episodes=128,joint_step=.0025,scale=1.,mode='combined'):
    if episodes<=0 or scale<0:raise ValueError('Invalid sample count or uncertainty scale')
    begin=time.perf_counter();path=densify(path,joint_step);w=RackWorld(scene)
    try:
        valid,reason,nominal=validate_nominal(w,scene,path)
        if not valid:return dict(status='invalid_nominal_path',reason=reason)
        errors=draw_errors(scene,episodes,scene['evaluation_seed'],scale,mode)
        ee=nominal@np.linalg.inv(w.grasp);outcomes=[]
        bounds=np.array([p.getAABB(ob,physicsClientId=w.cid) for ob in w.obstacles])
        for xi in errors:
            poses=ee@grasp_delta(xi)@w.grasp;quats=Rotation.from_matrix(poses[:,:3,:3]).as_quat()
            result=dict(collision=False,success=False,reason=None,obstacle=None,first_state=None)
            for index,q in enumerate(path):
                w.set_q(q);p.resetBasePositionAndOrientation(w.object,poses[index,:3,3],quats[index],physicsClientId=w.cid)
                lo,hi=np.array(p.getAABB(w.object,physicsClientId=w.cid))
                # Conservative AABB broad phase. Every surviving pair still goes
                # through Bullet's original narrow-phase collision query.
                selected=np.flatnonzero(np.all(bounds[:,1]>=lo-.002,axis=1)&np.all(bounds[:,0]<=hi+.002,axis=1))
                for ob in (w.obstacles[j] for j in selected):
                    if p.getClosestPoints(w.object,ob,0,physicsClientId=w.cid):
                        result.update(collision=True,reason='object_environment',obstacle=w.names[ob]);break
                if not result['collision']:
                    for c in p.getClosestPoints(w.robot,w.object,0,physicsClientId=w.cid):
                        if c[3] not in w.grasp_contact_links and c[8]<-1e-6:
                            result.update(collision=True,reason='object_robot',obstacle=p.getJointInfo(w.robot,c[3],physicsClientId=w.cid)[12].decode());break
                if result['collision']:result['first_state']=index;break
            if not result['collision']:result['success'],result['reason']=task_success(scene,poses)
            outcomes.append(result)
        nc=sum(r['collision'] for r in outcomes);ns=sum(r['success'] for r in outcomes)
        return dict(status='evaluated',task_id=scene['id'],geometry_id=scene['geometry_id'],operation=scene['task']['operation'],
            episodes=episodes,seed=scene['evaluation_seed'],mode=mode,scale=scale,joint_step_max_rad=joint_step,trajectory_states=len(path),
            collision_count=nc,collision_rate=nc/episodes,collision_ci95=interval(nc,episodes),success_count=ns,success_rate=ns/episodes,success_ci95=interval(ns,episodes),
            failures_by_reason=dict(Counter(r['reason'] for r in outcomes if not r['success'])),
            colliding_obstacles=dict(Counter(r['obstacle'] for r in outcomes if r['collision'])),
            error_samples=errors.tolist(),outcomes=outcomes,wall_seconds=time.perf_counter()-begin,
            qualification='Discrete kinematic evaluation of fixed grasp error; no contact dynamics or robustness certificate')
    finally:w.close()

def audit_retreat(scene,joint_step=.0025):
    """Open-hand exit AFTER an assumed lowering/release; not a release simulation."""
    w=RackWorld(scene,jaw_state='open')
    try:
        t=np.array(scene['task']['nominal_object_goal_pose']).copy();t[2,3]-=.008;w.fixed_object_pose=t
        last=np.array(scene['q_goal']);w.set_q(last);target=w.ee_pose();poses=[];seed=last
        for x in np.linspace(target[0,3],.60,80):
            desired=target.copy();desired[0,3]=x;q=w.ik(desired,seed)
            if q is None:return dict(status='failed',reason='ik'),None
            poses.append(q);seed=q
        path=densify(poses,joint_step)
        supports={ob for ob in w.obstacles if w.names[ob].startswith('target_saddle_')}
        for i,q in enumerate(path):
            w.set_q(q);w.update_object()
            for ob in w.obstacles:
                if p.getClosestPoints(w.robot,ob,0,physicsClientId=w.cid):return dict(status='failed',reason='robot_environment',obstacle=w.names[ob],state=i),None
                if ob not in supports and p.getClosestPoints(w.object,ob,0,physicsClientId=w.cid):return dict(status='failed',reason='placed_payload_environment',obstacle=w.names[ob],state=i),None
            if any(c[8]<-1e-6 for c in p.getClosestPoints(w.robot,w.object,0,physicsClientId=w.cid)):
                return dict(status='failed',reason='open_gripper_payload',state=i),None
            for c in p.getClosestPoints(w.robot,w.robot,0,physicsClientId=w.cid):
                a,b=sorted([c[3],c[4]])
                if a!=b and (a,b) not in w.ignored_self and c[8]<-1e-6:return dict(status='failed',reason='robot_self',state=i),None
        gripper_links=[j for j in range(p.getNumJoints(w.robot,physicsClientId=w.cid))
            if p.getJointInfo(w.robot,j,physicsClientId=w.cid)[12].startswith(b'robotiq_')]
        max_x=max(p.getAABB(w.robot,j,physicsClientId=w.cid)[1][0] for j in gripper_links)
        if max_x>scene['rack']['front_x']-.02:return dict(status='failed',reason='gripper_not_fully_out'),None
        return dict(status='passed',states=len(path),joint_step_max_rad=joint_step,gripper_exit_clearance_m=scene['rack']['front_x']-max_x,
            scope='Open-gripper retreat after assumed 8 mm lowering; release transition/dynamics not simulated'),path
    finally:w.close()

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--path',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--episodes',type=int,default=128);ap.add_argument('--joint-step',type=float,default=.0025)
    ap.add_argument('--scale',type=float,default=1.);ap.add_argument('--mode',choices=['combined','translation_only','rotation_only'],default='combined');a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    result=evaluate(json.loads(a.scene.read_text()),np.load(a.path),a.episodes,a.joint_step,a.scale,a.mode)
    result['scene_sha256']=hashlib.sha256(a.scene.read_bytes()).hexdigest();result['path_sha256']=hashlib.sha256(a.path.read_bytes()).hexdigest()
    write_json(a.output,result);print({k:v for k,v in result.items() if k not in ['error_samples','outcomes']})
