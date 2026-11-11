"""Revalidate every saved witness; uncertainty pilot uses DEVELOPMENT instances only."""
import argparse,json,hashlib,time
from pathlib import Path
import numpy as np
from .world import World,densify
from .evaluate import evaluate,validate_nominal
from .generate import write_json

def audit(data,out,episodes,joint_step=.005):
    data=Path(data);out=Path(out)
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True);manifest=json.loads((data/'manifest.json').read_text());records=[];evaluations=[]
    for entry in manifest['tasks']:
        sf=data/entry['file'];wf=data/'reference_witnesses'/entry['split']/(entry['id']+'.npy')
        assert hashlib.sha256(sf.read_bytes()).hexdigest()==entry['sha256']
        assert hashlib.sha256(wf.read_bytes()).hexdigest()==entry['witness_sha256']
        scene=json.loads(sf.read_text());path=densify(np.load(wf),.005);w=World(scene)
        try:ok,reason,_=validate_nominal(w,scene,path)
        finally:w.close()
        records.append(dict(id=scene['id'],nominal_valid=bool(ok),reason=reason,states=len(path)))
        print(scene['id'],'nominal',ok,reason,flush=True)
        if not ok:continue
        if entry['split']=='dev':
            r=evaluate(scene,np.load(wf),episodes=episodes,joint_step=joint_step)
            r['source']='generator nominal feasibility witness; not a competitive planner result'
            r['scene_sha256']=entry['sha256'];r['trajectory_sha256']=entry['witness_sha256']
            write_json(out/'uncertainty'/f"{scene['id']}.json",r)
            evaluations.append({k:v for k,v in r.items() if k not in ['error_samples','outcomes']})
            print('  uncertain collision',r.get('collision_rate'),flush=True)
    write_json(out/'summary.json',dict(nominal=records,uncertainty=evaluations,
        counts=dict(nominal_total=len(records),nominal_valid=sum(r['nominal_valid'] for r in records),uncertainty_dev_tasks=len(evaluations)),
        test_policy='Test set only checked for nominal construction feasibility; no uncertain test outcomes used during development'))
    if not all(r['nominal_valid'] for r in records):raise RuntimeError('Nominal audit failed; see summary')

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--data',required=True);ap.add_argument('--output',required=True);ap.add_argument('--episodes',type=int,default=256);ap.add_argument('--joint-step',type=float,default=.005);a=ap.parse_args();audit(a.data,a.output,a.episodes,a.joint_step)
