"""Engineering-scale short-stock rack; explicit task semantics and nominal witnesses."""
import argparse, copy, hashlib, json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from gobench.world import transform,densify
from gobench.generate import write_json
from .world import RackWorld

BLUE=[.075,.21,.36,1];ORANGE=[.94,.38,.07,1];STEEL=[.55,.61,.64,1]
BASE=Path(__file__).resolve().parents[1]

def box(name,center,size,color=STEEL,role='rack_structure'):
    return dict(name=name,size=size,color=color,pose=transform(center,[0,0,0,1]).tolist(),role=role)

def tube_parts(length=.5,width=.04,height=.04,wall=.003,color=None):
    c=color or [.8,.82,.85,1]
    return [dict(size=[length,width,wall],center=[0,0,z],color=c) for z in [-(height-wall)/2,(height-wall)/2]]+[
        dict(size=[length,wall,height-2*wall],center=[0,y,0],color=c) for y in [-(width-wall)/2,(width-wall)/2]]

def ee_pose(pos,yaw=0.):
    # TCP +Z is insertion direction; jaw closing axis +X maps to world +Y.
    r=np.array([[0,0,1],[1,0,0],[0,1,0]])
    r=Rotation.from_euler('z',yaw).as_matrix()@r
    return transform(pos,Rotation.from_matrix(r).as_quat())

def candidate(index=0):
    # Each cell is an independently named engineering configuration, not a difficulty claim.
    cases=[('central_bay',0.,.50,.0),('side_bay',.26,.46,.0),('deep_bay',-.26,.56,.02)]
    name,y,length,offset=cases[index]
    front=.60;back=1.23;floor=-.75;obs=[]
    obs.append(box('floor',[.25,0,floor-.025],[2.6,2.8,.05],[.84,.85,.83,1],'floor'))
    obs.append(box('robot_pedestal',[0,0,-.39],[.23,.25,.70],[.27,.29,.31,1],'robot_mount'))
    obs.append(box('robot_base_plate',[0,0,-.025],[.24,.24,.04],STEEL,'robot_mount'))
    for x in [front,back]:
      for yy in [-.55,.55]:
        obs.append(box(f'upright_{x}_{yy}',[x,yy,-.02],[.045,.045,1.42],BLUE))
        obs.append(box(f'foot_{x}_{yy}',[x,yy,floor+.012],[.12,.11,.024],BLUE))
    # Three usable tiers; all shelf decks, beams and rear stops are collision geometry.
    decks=[-.43,-.12,.16,.48]
    for level,z in enumerate(decks):
        obs.append(box(f'deck_{level}',[(front+back)/2,0,z],[back-front,1.06,.018],[.64,.68,.7,1]))
        for x in [front,back]:obs.append(box(f'beam_{level}_{x}',[x,0,z-.025],[.045,1.1,.06],ORANGE))
    obs.append(box('rear_guard_panel',[back+.025,0,.02],[.012,1.07,1.34],[.27,.34,.37,1],'back_guard'))
    # Dividers constrain lateral sweeping at the occupied tier.
    for yy in [-.40,-.13,.13,.40]:
        obs.append(box(f'bin_divider_{yy}',[.94,yy,.255],[.52,.012,.18],BLUE,'bin_divider'))
    # Adjacent inventory blocks the tempting side sweep, including a protruding end.
    for lev,z in enumerate([-.12,.16]):
      for col,yy in enumerate([-.26,0.,.26]):
        if lev==1 and abs(yy-y)<.02:continue
        for lane,dy in enumerate([-.034,.034]):
            obs.append(dict(name=f'stock_{lev}_{col}_{lane}',parts=tube_parts(.48 if col!=0 else .54,color=[.48,.52,.56,1]),
                pose=transform([.90,yy+dy,z+.029],[0,0,0,1]).tolist(),role='neighbor_stock'))
    # Two physical nests and side-guide ears at the requested destination.
    support_z=.181
    for k,x in enumerate([.77+offset,1.06+offset]):
        obs.append(box(f'target_saddle_{k}',[x,y,support_z-.006],[.05,.10,.012],[.16,.18,.21,1],'target_support'))
        for s in [-1,1]:
            obs.append(box(f'target_guide_{k}_{s}',[x,y+s*.039,.211],[.035,.014,.06],ORANGE,'target_guide'))
    # Source pallet / transfer stand outside the rack, including stock waiting to be picked.
    obs.append(box('source_table',[.27,-.39,.05],[.60,.52,.035],[.35,.39,.41,1],'source_fixture'))
    for x in [.02,.52]:
      for yy in [-.61,-.17]:obs.append(box(f'source_leg_{x}_{yy}',[x,yy,-.35875],[.035,.035,.7825],STEEL,'source_fixture'))
    # The carried tube is already grasped, 80 mm above this source nest.
    for d in [.13,.36]:obs.append(box(f'source_cradle_{d}',[.12+d*np.cos(.8),-.47+d*np.sin(.8),.114],[.10,.075,.035],ORANGE,'source_fixture'))
    for xx in [-.035,.27]:
        t=transform([xx+.10,-.43,.09],Rotation.from_euler('z',.8).as_quat())
        obs.append(dict(name=f'source_waiting_stock_{xx}',parts=tube_parts(.36),pose=t.tolist(),role='source_stock'))
    # Keep the front of a long-stock rack outside the arm's folded wrist region.
    for item in obs:
        if item['role'] not in ['floor','robot_mount','source_fixture','source_stock']:
            item['pose'][0][3]+=.15
    front+=.15;back+=.15
    # Reference grasp: only 20 mm of stock behind the TCP, long cantilever ahead.
    grasp=np.eye(4);grasp[:3,:3]=ee_pose([0,0,0])[:3,:3].T
    grasp[2,3]=length/2-.020
    # Source->raise->turn in aisle->align->insert->lower to 8 mm pre-release clearance.
    pre_x=front-length+.020-.025
    goal_x=.83 if index!=2 else .80
    knots=[ee_pose([.12,-.47,.23],.8),ee_pose([.12,-.47,.34],.8),
        ee_pose([pre_x,-.47,.34],.8),ee_pose([pre_x,-.47,.34],0),
        ee_pose([.43,-.47,.38],0),ee_pose([.43,y,.38],0),
        ee_pose([.43,y,.235],0),ee_pose([goal_x,y,.235],0),ee_pose([goal_x,y,.209],0)]
    scene=dict(schema_version='0.2.0',id=name,family='warehouse_short_stock_rack',split='development',robot='UR5e + Robotiq 2F-85',
        units={'length':'m','angle':'rad'},obstacles=obs,
        object=dict(kind='hollow_square_tube',material='aluminium (nominal)',parts=tube_parts(length),size=[length,.04,.04],
            wall_thickness_m=.003,estimated_mass_kg=float((.04**2-.034**2)*length*2700),T_EO=grasp.tolist()),
        rack=dict(front_x=front,back_x=back,width_m=1.1,depth_m=back-front,height_m=1.42,
            target_bay_y=y,target_support_top_z=support_z,target_bay_half_width_m=.124),
        grasp=dict(model='frozen manufacturer parallel grasp',rear_overhang_m=.020,front_lever_arm_m=length-.020,
            contact_exclusions=['robotiq_85_left_finger_tip_link','robotiq_85_right_finger_tip_link']),
        uncertainty=dict(convention='T_WO=T_WE @ [Exp_SO3(phi),dt] @ T_EO; EE frame; one fixed draw per transfer',
            translation_std_m=[.002]*3,rotation_std_rad=[float(np.deg2rad(1))]*3,calibrated=False),
        task=dict(operation='store',start_phase='post_grasp_above_source_nest',goal_phase='pre_release_above_destination_supports',
            goal_representation='object_pose_region',q_goal_role='reference IK solution only; planners may end elsewhere inside the goal region',
            goal_position_tolerance_m=.012,goal_orientation_tolerance_rad=float(np.deg2rad(5)),
            min_final_rear_depth_m=.015,require_destination_support_coverage=True),
        evaluation_seed=202611100+index,qualification='Engineered synthetic short-stock rack, not a measured warehouse digital twin')
    return scene,knots

def solve(w,knots):
    seed=w.home;path=[];phases=[]
    for seg,(a,b) in enumerate(zip(knots[:-1],knots[1:])):
        r=Slerp([0,1],Rotation.from_matrix([a[:3,:3],b[:3,:3]]))
        n=max(12,int(np.linalg.norm(b[:3,3]-a[:3,3])/.004),int(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()/.025))
        for u in np.linspace(0,1,n,endpoint=True):
            pose=transform(a[:3,3]*(1-u)+b[:3,3]*u,r(u).as_quat());q=w.ik(pose,seed)
            if q is None:return None,dict(reason='ik',segment=seg,u=float(u),pose=pose.tolist())
            if path and np.max(abs(q-seed))>.25:return None,dict(reason='ik_jump',segment=seg,u=float(u))
            detail=w.collision_detail(q)
            if detail:return None,dict(**detail,segment=seg,u=float(u),q=q.tolist())
            path.append(q);seed=q
        phases.append(len(path)-1)
    path=densify(path,.005)
    for i,q in enumerate(path):
        detail=w.collision_detail(q)
        if detail:return None,dict(**detail,state=i)
    return path,None

def generate(output):
    out=Path(output)
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True);entries=[]
    for i in range(3):
        scene,knots=candidate(i);w=RackWorld(scene)
        try:
            path,reason=solve(w,knots)
            if reason:
                write_json(out/(scene['id']+'_rejected.json'),dict(scene=scene,reason=reason,knots=[t.tolist() for t in knots]))
                print(scene['id'],'REJECTED',reason,flush=True);continue
            scene['q_start']=path[0].tolist();scene['q_goal']=path[-1].tolist()
            w.set_q(path[0]);scene['task']['nominal_object_start_pose']=w.update_object().tolist()
            w.set_q(path[-1]);scene['task']['nominal_object_goal_pose']=w.update_object().tolist()
            scene['validation']=dict(witness_states=len(path),joint_step_max_rad=.005,nominal_collision_free=True,continuous_certificate=False)
            geometry_id=scene['id']
            for operation in ['store','retrieve']:
                task=copy.deepcopy(scene);task['geometry_id']=geometry_id;task['id']=geometry_id+'_'+operation
                task['task']['operation']=operation;traj=path if operation=='store' else path[::-1].copy()
                if operation=='retrieve':
                    task['q_start'],task['q_goal']=scene['q_goal'],scene['q_start']
                    task['task']['nominal_object_start_pose'],task['task']['nominal_object_goal_pose']=scene['task']['nominal_object_goal_pose'],scene['task']['nominal_object_start_pose']
                    task['task']['start_phase']='post_grasp_above_destination_supports';task['task']['goal_phase']='transfer_pose_above_source_nest'
                f=out/'tasks'/(task['id']+'.json');write_json(f,task)
                pth=out/'reference_witnesses'/(task['id']+'.npy');pth.parent.mkdir(exist_ok=True);np.save(pth,traj)
                entries.append(dict(id=task['id'],geometry_id=geometry_id,operation=operation,file=str(f.relative_to(out)),sha256=hashlib.sha256(f.read_bytes()).hexdigest(),
                    witness_file=str(pth.relative_to(out)),witness_sha256=hashlib.sha256(pth.read_bytes()).hexdigest()))
                print(task['id'],'ACCEPTED',len(traj),flush=True)
        finally:w.close()
    write_json(out/'manifest.json',dict(version='0.2.0',status='development engineering scenes',tasks=entries,
        selection='nominal feasibility only; no filtering on uncertain evaluations',test_set='not frozen',
        independent_geometry_count=len(set(e['geometry_id'] for e in entries)),
        reverse_task_policy='Retrieval and storage share geometry and reversed feasibility witnesses; do not count as independent scenes',
        robot_provenance=json.loads((BASE/'assets/ur5e_robotiq/provenance.json').read_text())))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);a=ap.parse_args();generate(a.output)
