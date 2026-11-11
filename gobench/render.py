"""Render actual benchmark geometry and URDF in PyBullet's software renderer."""
import argparse,json
from pathlib import Path
import numpy as np
import pybullet as p
import imageio.v2 as imageio
from .world import World


def frame(w,q,xi=None,width=960,height=720,camera=(-.55,-1.25,1.15)):
    w.set_q(q);w.update_object(xi)
    view=p.computeViewMatrix(camera,[.46,0,.52],[0,0,1])
    proj=p.computeProjectionMatrixFOV(48,width/height,.03,5)
    rgba=p.getCameraImage(width,height,view,proj,renderer=p.ER_TINY_RENDERER,
        lightDirection=[-2,-3,5],shadow=1,physicsClientId=w.cid)[2]
    return np.array(rgba,dtype=np.uint8)[:,:,:3]


def render(scene,path,out,video=False):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);w=World(scene)
    try:
        x=scene['openings'][0]['pose'][0][3]
        wps=[]
        for q in path:w.set_q(q);wps.append(w.update_object()[0,3])
        idx=int(np.argmin(abs(np.array(wps)-x)))
        imageio.imwrite(out/'scene.png',frame(w,path[idx]))
        if video:
            d=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
            # Visualization only: uniform joint arc length, not a timed control trajectory.
            samples=np.linspace(0,d[-1],120)
            qs=np.array([np.interp(samples,d,path[:,j]) for j in range(6)]).T
            with imageio.get_writer(out/'replay.mp4',fps=20,codec='libx264',quality=7) as writer:
                for q in qs:writer.append_data(frame(w,q,width=800,height=600))
    finally:w.close()

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--path',type=Path,required=True);ap.add_argument('--output',required=True);ap.add_argument('--video',action='store_true');a=ap.parse_args()
    render(json.loads(a.scene.read_text()),np.load(a.path),a.output,a.video)
