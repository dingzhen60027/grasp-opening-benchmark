"""Reproducible nominal and open-hand-exit audit of all public rack tasks."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from gobench.generate import write_json
from gobench.world import densify
from .world import RackWorld
from .evaluate import validate_nominal,audit_retreat

def audit(data,output):
    data=Path(data);out=Path(output)
    if (out/'nominal_audit.json').exists():raise FileExistsError(out/'nominal_audit.json')
    out.mkdir(parents=True,exist_ok=True);report={}
    manifest=json.loads((data/'manifest.json').read_text())
    for e in manifest['tasks']:
        f=data/e['file'];pf=data/e['witness_file']
        assert hashlib.sha256(f.read_bytes()).hexdigest()==e['sha256']
        assert hashlib.sha256(pf.read_bytes()).hexdigest()==e['witness_sha256']
        s=json.loads(f.read_text());path=densify(np.load(pf),.0025);w=RackWorld(s)
        try:ok,reason,poses=validate_nominal(w,s,path)
        finally:w.close()
        r=dict(nominal_valid=ok,reason=reason,states=len(path),joint_step_max_rad=.0025,scene_sha256=e['sha256'],witness_sha256=e['witness_sha256'])
        if s['task']['operation']=='store':
            retreat,traj=audit_retreat(s);r['open_hand_retreat']=retreat
            if traj is not None:
                (out/'retreats').mkdir(exist_ok=True);np.save(out/'retreats'/(s['id']+'.npy'),traj)
        report[s['id']]=r;print(s['id'],r,flush=True)
    write_json(out/'nominal_audit.json',report)
    if not all(v['nominal_valid'] and v.get('open_hand_retreat',{'status':'passed'})['status']=='passed' for v in report.values()):
        raise RuntimeError('Audit failed; see report')

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--data',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();audit(a.data,a.output)
