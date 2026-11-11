"""Seeded joint-space RRT-Connect baseline. Does not read generator witnesses."""
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np
from .world import World,densify
from .evaluate import validate_nominal
from .generate import write_json

def rrt_connect(scene,seed=0,budget=20.,margin=0.,step=.22,resolution=.035,world_factory=World,validator=validate_nominal):
    if budget<=0 or margin<0:raise ValueError('budget must be positive and margin nonnegative')
    w=world_factory(scene);rng=np.random.default_rng(seed);start=time.perf_counter();deadline=start+budget;checks=0
    def valid(q):
        nonlocal checks
        checks+=1
        return w.collision(q,margin=margin) is None
    def edge(a,b):
        n=max(1,int(np.ceil(np.max(np.abs(b-a))/resolution)))
        for u in np.linspace(0,1,n+1)[1:]:
            if time.perf_counter()>deadline or not valid(a+(b-a)*u):return False
        return True
    q0=np.array(scene['q_start']);q1=np.array(scene['q_goal']);path=None;status='timeout'
    try:
        if not valid(q0) or not valid(q1):status='invalid_endpoint_for_margin'
        elif edge(q0,q1):path=np.array([q0,q1]);status='solved'
        else:
            trees=[([q0],[-1]),([q1],[-1])]
            side=0
            def extend(tree,target):
                nodes,parents=tree
                j=int(np.argmin(np.linalg.norm(np.array(nodes)-target,axis=1)))
                d=target-nodes[j];length=np.linalg.norm(d)
                new=target if length<=step else nodes[j]+d*step/length
                if not edge(nodes[j],new):return None,False
                nodes.append(new);parents.append(j)
                return len(nodes)-1,length<=step
            def trace(tree,j):
                result=[]
                while j!=-1:result.append(tree[0][j]);j=tree[1][j]
                return result[::-1]
            while time.perf_counter()<deadline:
                a,b=trees[side],trees[1-side]
                sample=b[0][0] if rng.random()<.1 else rng.uniform(w.lower,w.upper)
                ja,_=extend(a,sample)
                if ja is not None:
                    while time.perf_counter()<deadline:
                        jb,reached=extend(b,a[0][ja])
                        if jb is None:break
                        if reached:
                            pa=trace(a,ja);pb=trace(b,jb)
                            path=np.array(pa+pb[-2::-1])
                            if side==1:path=path[::-1]
                            status='solved';break
                    if path is not None:break
                side=1-side
        if path is not None:
            path=densify(path,.005)
            ok,reason,_=validator(w,scene,path)
            if ok and margin>0:
                ok=all(valid(q) for q in path)
                if not ok:reason='margin_not_preserved'
            if not ok:status='post_validation_failed:'+str(reason);path=None
        result=dict(method='rrt_connect' if margin==0 else 'uniform_margin_rrt_connect',status=status,
            scene_id=scene['id'],seed=seed,budget_seconds=budget,margin_m=margin,
            collision_checks=checks,wall_seconds=time.perf_counter()-start,
            path_states=0 if path is None else len(path),witness_used=False)
        return path,result
    finally:w.close()

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--seed',type=int,default=0);ap.add_argument('--budget',type=float,default=20);ap.add_argument('--margin',type=float,default=0)
    args=ap.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    path,r=rrt_connect(json.loads(args.scene.read_text()),args.seed,args.budget,args.margin)
    r['scene_sha256']=hashlib.sha256(args.scene.read_bytes()).hexdigest()
    if path is not None:np.save(args.output/'trajectory.npy',path)
    write_json(args.output/'result.json',r);print(r)
