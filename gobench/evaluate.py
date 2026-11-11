"""Full-path grasp perturbation evaluation; samples are constant across trajectory."""
import argparse, itertools, json, time, hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import beta
from .world import World, densify, grasp_delta, p
from .generate import write_json


def interval(k,n,confidence=.95):
    """Two-sided Clopper-Pearson interval for an episode Bernoulli rate."""
    a=(1-confidence)/2
    return [float(beta.ppf(a,k,n-k+1)) if k else 0.,float(beta.ppf(1-a,k+1,n-k)) if k<n else 1.]


def task_success(scene,poses):
    """Require full payload initially before and finally beyond ALL finite-thickness gates.
    Each payload center must cross the aperture in the specified order; an outside detour fails.
    Goal is actual payload pose, not just the TCP pose.
    """
    size=np.array(scene['object']['size'])
    corners=np.array(list(itertools.product([-1,1],repeat=3)))*size/2
    vertices=np.einsum('tij,kj->tki',poses[:,:3,:3],corners)+poses[:,:3,3,None].transpose(0,2,1)
    last=-1
    for opening in scene['openings']:
        gate=np.array(opening['pose']); inv=np.linalg.inv(gate)
        centers=(poses[:,:3,3]-gate[:3,3]) @ gate[:3,:3]
        verts=(vertices-gate[:3,3]) @ gate[:3,:3]
        if verts[0,:,0].max()>=-opening['thickness']/2 or verts[-1,:,0].min()<=opening['thickness']/2:
            return False,'incomplete_transfer'
        crossing=np.flatnonzero((centers[:-1,0]<=0)&(centers[1:,0]>0))
        good=[i for i in crossing if i>last and abs(centers[i,1])<opening['width']/2
            and opening['z_min']<centers[i,2]<opening['z_max']]
        if not good:return False,'missed_opening'
        last=int(good[0])
    goal=np.array(scene['task']['nominal_object_goal_pose'])
    if np.linalg.norm(poses[-1,:3,3]-goal[:3,3])>scene['task']['goal_position_tolerance_m']:
        return False,'goal_position'
    if Rotation.from_matrix(goal[:3,:3].T @ poses[-1,:3,:3]).magnitude()>scene['task']['goal_orientation_tolerance_rad']:
        return False,'goal_orientation'
    return True,None


def validate_nominal(w,scene,path):
    if np.max(np.abs(path[0]-scene['q_start']))>1e-5 or np.max(np.abs(path[-1]-scene['q_goal']))>1e-5:
        return False,'wrong_endpoints',None
    poses=[]
    for q in path:
        c=w.collision(q)
        if c:return False,c,None
        poses.append(w.update_object())
    ok,reason=task_success(scene,np.array(poses))
    return ok,reason,np.array(poses)


def draw_errors(scene,n,seed,scale=1.,mode='combined'):
    std=np.r_[scene['uncertainty']['translation_std_m'],scene['uncertainty']['rotation_std_rad']]*scale
    if mode=='translation_only':std[3:]=0
    elif mode=='rotation_only':std[:3]=0
    elif mode!='combined':raise ValueError(mode)
    return np.random.default_rng(seed).normal(size=(n,6))*std


def evaluate(scene,path,episodes=128,scale=1.,mode='combined',seed=None,joint_step=.00125):
    if episodes<=0 or scale<0:raise ValueError('episodes must be positive; scale nonnegative')
    start=time.perf_counter();path=densify(path,joint_step);w=World(scene)
    try:
        ok,reason,nominal_poses=validate_nominal(w,scene,path)
        if not ok:return dict(status='invalid_nominal_path',reason=reason,task_id=scene['id'])
        seed=scene['evaluation_seed'] if seed is None else seed
        errors=draw_errors(scene,episodes,seed,scale,mode)
        ee_poses=nominal_poses @ np.linalg.inv(w.grasp)
        # The nonzero transform is sampled ONCE per episode, retained for all states.
        outcomes=[]
        for xi in errors:
            poses=ee_poses @ grasp_delta(xi) @ w.grasp
            quaternions=Rotation.from_matrix(poses[:,:3,:3]).as_quat()
            reason=None;collision=False;collision_state=None
            for index,q in enumerate(path):
                w.set_q(q)
                p.resetBasePositionAndOrientation(w.object,poses[index,:3,3],quaternions[index],physicsClientId=w.cid)
                # Arm/environment and self contacts are deterministic and prechecked above.
                for ob in w.obstacles:
                    if p.getClosestPoints(w.object,ob,0,physicsClientId=w.cid):
                        reason='object_environment';collision=True;break
                if not collision:
                    for c in p.getClosestPoints(w.robot,w.object,0,physicsClientId=w.cid):
                        if c[3] not in w.grasp_contact_links and c[8]<-1e-6:
                            reason='object_robot';collision=True;break
                if collision:collision_state=index;break
            success=False
            if not collision:success,reason=task_success(scene,np.array(poses))
            outcomes.append(dict(success=bool(success),collision=collision,reason=reason,first_collision_state=collision_state))
        collisions=sum(x['collision'] for x in outcomes);successes=sum(x['success'] for x in outcomes)
        return dict(status='evaluated',task_id=scene['id'],family=scene['family'],difficulty=scene['difficulty'],split=scene['split'],
            episodes=episodes,mode=mode,uncertainty_scale=scale,seed=seed,trajectory_states=len(path),joint_step_max_rad=joint_step,
            collision_count=collisions,collision_rate=collisions/episodes,collision_rate_ci95=interval(collisions,episodes),
            task_success_count=successes,task_success_rate=successes/episodes,task_success_ci95=interval(successes,episodes),
            path_joint_length_rad=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),
            error_samples=errors.tolist(),outcomes=outcomes,wall_seconds=time.perf_counter()-start,
            qualification='Empirical discrete kinematic evaluation; not a continuous or worst-case robustness certificate')
    finally:w.close()

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--path',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--episodes',type=int,default=128);ap.add_argument('--scale',type=float,default=1)
    ap.add_argument('--mode',choices=['combined','translation_only','rotation_only'],default='combined')
    ap.add_argument('--joint-step',type=float,default=.00125);args=ap.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    r=evaluate(json.loads(args.scene.read_text()),np.load(args.path),args.episodes,args.scale,args.mode,joint_step=args.joint_step)
    r['scene_sha256']=hashlib.sha256(args.scene.read_bytes()).hexdigest();r['trajectory_sha256']=hashlib.sha256(args.path.read_bytes()).hexdigest()
    write_json(args.output,r);print({k:v for k,v in r.items() if k not in ['error_samples','outcomes']})
