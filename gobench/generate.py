"""Procedural tasks. Save only independently collision-checked IK witness paths."""
import argparse, hashlib, json, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .world import World, transform, densify

FAMILIES=['single_opening','deep_tunnel','offset_pair','shelf_entry','turning_pair']
LEVELS={'easy':.025,'medium':.012,'hard':.006}

def write_json(p, obj):
    p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')

def gate(obstacles, x, y, zbottom, width, thickness, yaw=0.):
    """Four solid walls, no infinitely thin or collision-disabled decorative geometry."""
    top=1.08; outer_width=1.5; outer_height=1.45
    pose=transform([x,y,0],Rotation.from_euler('z',yaw).as_quat())
    for name,center,size in [
      ('bottom',[0,0,zbottom/2],[thickness,outer_width,zbottom]),
      ('top',[0,0,(top+outer_height)/2],[thickness,outer_width,outer_height-top]),
      ('left',[0,-(width+outer_width)/4,(zbottom+top)/2],[thickness,(outer_width-width)/2,top-zbottom]),
      ('right',[0,(width+outer_width)/4,(zbottom+top)/2],[thickness,(outer_width-width)/2,top-zbottom])]:
        t=pose.copy();t[:3,3]=(pose@np.r_[center,1])[:3]
        obstacles.append(dict(name=f'gate{len(obstacles)//4}_{name}',shape='box',size=size,pose=t.tolist()))
    return dict(pose=pose.tolist(),width=width,z_min=zbottom,z_max=top,thickness=thickness)

def candidate(family,level,seed):
    rng=np.random.default_rng(seed); gap=LEVELS[level]
    length=float(rng.uniform(.32,.44)); z=float(rng.uniform(.43,.49)); y=float(rng.uniform(-.04,.04))
    x=float(rng.uniform(.45,.48)); thickness=.035
    size=[.035,length,.045];grasp=np.eye(4)
    # Translation of grasp along the long axis changes the uncertainty lever arm.
    grasp[1,3]=float(rng.uniform(-.035,.035))
    object_y=-grasp[1,3]
    bottom=z-size[2]/2-gap; width=length+2*gap
    obstacles=[];gates=[]
    xs=[x];ys=[y];yaws=[0.]
    if family=='deep_tunnel':thickness=float(rng.uniform(.13,.19))
    if family=='offset_pair':xs=[x-.055,x+.07];ys=[y-.025,y+.025];yaws=[0.,0.]
    if family=='turning_pair':xs=[x-.075,x+.085];ys=[y,y+.02];yaws=[-.10,.10]
    for gx,gy,angle in zip(xs,ys,yaws):
        gates.append(gate(obstacles,gx,gy,bottom,width,thickness,angle))
    if family=='shelf_entry':
        # Enclosed destination bay; back wall remains beyond the goal payload.
        for name,center,box in [('back',[.88,y,.60],[.035,1.0,1.2]),('ceiling',[.70,y,1.095],[.38,1.,.03])]:
            obstacles.append(dict(name=name,shape='box',size=box,pose=transform(center,[0,0,0,1]).tolist()))
    # Generator-only Cartesian witness crosses gates at their centers.
    xp=[.29];yp=[ys[0]];angles=[yaws[0]]
    for gx,gy,angle in zip(xs,ys,yaws):
        pad=thickness/2+size[0]/2+abs(np.sin(angle))*length/2+.004
        xp.extend([gx-pad,gx,gx+pad]);yp.extend([gy]*3);angles.extend([angle]*3)
    xp.append(.65);yp.append(ys[-1]);angles.append(yaws[-1])
    knots=[]
    for px,py,angle in zip(xp,yp,angles):
        rot=Rotation.from_euler('z',angle)*Rotation.from_euler('x',np.pi)
        ee_y=py-object_y*np.cos(angle)
        ee_x=px+object_y*np.sin(angle)
        knots.append(transform([ee_x,ee_y,z],rot.as_quat()))
    scene=dict(schema_version='0.1.0',family=family,difficulty=level,geometry_seed=seed,robot='ur5e',
        units=dict(length='m',angle='rad'),object=dict(shape='box',size=size,T_EO=grasp.tolist()),
        obstacles=obstacles,openings=gates,nominal_gap_m=gap,
        uncertainty=dict(convention='T_WO = T_WE @ [Exp_SO3(phi), delta_t] @ T_EO; EE-frame perturbation; fixed per episode',
            distribution='independent_zero_mean_gaussian',translation_std_m=[.003]*3,rotation_std_rad=[float(np.deg2rad(2))]*3),
        task=dict(type='rigid_payload_transfer',require_all_openings_in_order=True,goal_position_tolerance_m=.02,goal_orientation_tolerance_rad=float(np.deg2rad(10))))
    return scene,knots

def solve_witness(world,knots):
    seed=world.home;path=[]
    for i in range(len(knots)-1):
        a,b=knots[i:i+2];slerp=Slerp([0,1],Rotation.from_matrix([a[:3,:3],b[:3,:3]]))
        n=max(8,int(np.linalg.norm(b[:3,3]-a[:3,3])/.006))
        for u in np.linspace(0,1,n,endpoint=i==len(knots)-2):
            t=transform((1-u)*a[:3,3]+u*b[:3,3],slerp(u).as_quat())
            q=world.ik(t,seed)
            if q is None:return None,'ik'
            if path and np.max(np.abs(q-seed))>.3:return None,'ik_branch_jump'
            path.append(q);seed=q
    path=densify(path,.01)
    for q in path:
        c=world.collision(q)
        if c:return None,c
    return path,None

def generate(out,per_cell=4,master_seed=251105052):
    if not 1<=per_cell<=90:raise ValueError('per_cell must be between 1 and 90 to preserve disjoint seed ranges')
    out=Path(out)
    if out.exists():raise FileExistsError(f'{out} exists; choose a fresh directory')
    out.mkdir(parents=True);entries=[];rejects=[];start=time.perf_counter()
    for fi,family in enumerate(FAMILIES):
      for li,level in enumerate(LEVELS):
       for k in range(per_cell):
        split='dev' if k<max(1,per_cell//2) else 'test'
        task_id=f'{family}_{level}_{k:03d}'
        for attempt in range(40):
            seed=master_seed+fi*100000+li*10000+k*100+attempt
            scene,knots=candidate(family,level,seed);w=World(scene)
            path,reason=solve_witness(w,knots)
            if reason:
                rejects.append(dict(task_id=task_id,seed=seed,reason=reason));w.close();continue
            scene['id']=task_id;scene['split']=split
            scene['q_start']=path[0].tolist();scene['q_goal']=path[-1].tolist()
            w.set_q(path[-1]);scene['task']['nominal_object_goal_pose']=w.update_object().tolist()
            scene['validation']=dict(nominal_witness='passed',states=len(path),joint_step_max_rad=.01,
                method='discrete full arm/environment, payload/environment, nonadjacent self, payload/arm except grasp fixture',
                robust_feasibility='unknown',continuous_collision_certificate=False)
            scene['evaluation_seed']=seed+10000000
            file=out/'tasks'/split/(task_id+'.json');write_json(file,scene)
            witness=out/'reference_witnesses'/split/(task_id+'.npy');witness.parent.mkdir(parents=True,exist_ok=True);np.save(witness,path)
            entries.append(dict(id=task_id,family=family,difficulty=level,split=split,seed=seed,
                file=str(file.relative_to(out)),sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                witness_sha256=hashlib.sha256(witness.read_bytes()).hexdigest(),states=len(path)))
            if len(entries)==1:write_json(out/'robot_assets.json',w.asset_fingerprint())
            w.close();print(task_id,'accepted',len(path),'attempt',attempt,flush=True);break
        else:
            write_json(out/'rejections.json',rejects);raise RuntimeError(f'No nominal witness for {task_id}; no invalid fallback saved')
    write_json(out/'rejections.json',rejects)
    write_json(out/'manifest.json',dict(version='0.1.0',status='synthetic pilot benchmark; not a paper result',robot='UR5e',
        master_seed=master_seed,tasks=entries,counts=dict(tasks=len(entries),rejected_candidates=len(rejects)),
        generation_wall_seconds=time.perf_counter()-start,reference_policy='Witnesses are generator feasibility evidence; excluded from planner input',
        selection_policy='Nominal IK and collision feasibility only; no filtering on uncertain outcomes or planner success'))
    return entries

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--per-cell',type=int,default=4);ap.add_argument('--seed',type=int,default=251105052);args=ap.parse_args()
    generate(args.output,args.per_cell,args.seed)
