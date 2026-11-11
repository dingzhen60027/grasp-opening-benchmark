"""RRT-Connect adapter. Reads task JSON only; never reads reference witnesses."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from gobench.plan import rrt_connect
from gobench.generate import write_json
from .world import RackWorld
from .evaluate import validate_nominal

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--seed',type=int,default=0);ap.add_argument('--budget',type=float,default=30);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    a.output.mkdir(parents=True)
    path,result=rrt_connect(json.loads(a.scene.read_text()),seed=a.seed,budget=a.budget,resolution=.015,
        world_factory=RackWorld,validator=validate_nominal)
    result['scene_sha256']=hashlib.sha256(a.scene.read_bytes()).hexdigest()
    if path is not None:np.save(a.output/'trajectory.npy',path)
    write_json(a.output/'result.json',result);print(result)
