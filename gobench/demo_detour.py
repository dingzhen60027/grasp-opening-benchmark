"""Scene-informed deterministic detour for a SINGLE gate; an illustrative control, not a new robust planner.
No generator witness or sampled execution errors are read. Start/goal remain identical.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .world import World,transform
from .generate import solve_witness,write_json
from .evaluate import validate_nominal

def detour(scene):
    if len(scene['openings'])!=1:raise ValueError('Single-gate demo only')
    w=World(scene)
    try:
        w.set_q(scene['q_start']);start=w.ee_pose();w.set_q(scene['q_goal']);end=w.ee_pose()
        gate=np.array(scene['openings'][0]['pose']);gx,gy=gate[:2,3]
        target_R=Rotation.from_euler('z',.45)*Rotation.from_euler('x',np.pi)
        z=start[2,3]+.045
        grasp=np.array(scene['object']['T_EO'])
        # Rotate and lift before entering the slab, maintain pose until the entire box exits.
        pad=.12+scene['openings'][0]['thickness']/2
        knots=[start]
        for x in [gx-pad,gx,gx+pad]:
            obj=transform([x,gy,z],target_R.as_quat());knots.append(obj@np.linalg.inv(grasp))
        knots.append(end)
        path,reason=solve_witness(w,knots)
        if path is None:raise RuntimeError(reason)
        if np.max(abs(path[0]-scene['q_start']))>1e-3 or np.max(abs(path[-1]-scene['q_goal']))>1e-3:
            raise RuntimeError('IK changed endpoint branch')
        path[0]=scene['q_start'];path[-1]=scene['q_goal']
        from .world import densify
        path=densify(path,.005)
        ok,reason,_=validate_nominal(w,scene,path)
        if not ok:raise RuntimeError(reason)
        return path
    finally:w.close()

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    scene=json.loads(a.scene.read_text());path=detour(scene);a.output.mkdir(parents=True);np.save(a.output/'trajectory.npy',path)
    write_json(a.output/'result.json',dict(method='fixed_lift_yaw_single_gate_control',status='nominal_valid',task_id=scene['id'],
        lift_m=.045,yaw_rad=.45,witness_used=False,robust_guarantee=False,path_states=len(path)))
