"""Render the actual collision scene, with task phases and a close-up."""
import argparse,json
from pathlib import Path
import numpy as np
import pybullet as p
import imageio.v2 as imageio
from PIL import Image,ImageDraw,ImageFont
from .world import RackWorld

def frame(w,q,width=1280,height=960,detail=False):
    w.set_q(q);w.update_object()
    y=w.scene['rack']['target_bay_y']
    camera=([.22,y-.80,.78] if detail else [-1.35,-2.05,1.36])
    target=([1.02,y,.25] if detail else [.55,-.03,-.02])
    view=p.computeViewMatrix(camera,target,[0,0,1]);proj=p.computeProjectionMatrixFOV(45,width/height,.02,6)
    rgba=p.getCameraImage(width,height,view,proj,renderer=p.ER_TINY_RENDERER,lightDirection=[-3,-4,7],
        shadow=1,lightAmbientCoeff=.55,lightDiffuseCoeff=.55,physicsClientId=w.cid)[2]
    return np.array(rgba,dtype=np.uint8)[:,:,:3]

def font(size):
    try:return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',size)
    except OSError:return ImageFont.load_default(size=size)

def annotated(w,q,label,detail=False,width=1000,height=760):
    img=Image.fromarray(frame(w,q,width,height,detail));d=ImageDraw.Draw(img)
    d.rectangle([0,0,width,68],fill='#102431');d.text((24,19),label,font=font(22),fill='white')
    d.rectangle([0,height-35,width,height],fill='#eef2f3');d.text((20,height-27),'UR5e + Robotiq 2F-85 | actual simulator geometry | dimensions in metres',font=font(14),fill='#354853')
    return img

def render(scene,path,out,video=False):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);w=RackWorld(scene)
    try:
        # Use positions to select meaningful phases, instead of frame-count quartiles.
        ee=[]
        for q in path:w.set_q(q);ee.append(w.ee_pose()[:3,3])
        ee=np.array(ee);y=scene['rack']['target_bay_y']
        alignment=int(np.argmin(np.linalg.norm(ee-[.43,y,.38],axis=1)))
        entering=int(np.argmin(np.linalg.norm(ee-[.69,y,.235],axis=1)))
        pics=[annotated(w,path[0],'1  Picked stock above source nest'),
            annotated(w,path[alignment],'2  Turn and align with the selected bay'),
            annotated(w,path[entering],'3  Insert past neighboring stock',True),
            annotated(w,path[-1],'4  Pre-release pose over two supports',True)]
        board=Image.new('RGB',(2000,1520),'white')
        for i,img in enumerate(pics):board.paste(img,((i%2)*1000,(i//2)*760))
        board.save(out/'workflow.png');pics[1].save(out/'overview.png');pics[-1].save(out/'placement_detail.png')
        if video:
            d=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
            ss=np.linspace(0,d[-1],180);qs=np.array([np.interp(ss,d,path[:,j]) for j in range(6)]).T
            with imageio.get_writer(out/'replay.mp4',fps=24,codec='libx264',quality=8) as writer:
                for i,q in enumerate(qs):
                    main=annotated(w,q,f"{scene['id']}  |  kinematic replay {i+1}/180",width=960,height=736)
                    inset=Image.fromarray(frame(w,q,384,288,True));main.paste(inset,(560,76));writer.append_data(np.array(main))
    finally:w.close()

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--path',type=Path,required=True)
    ap.add_argument('--output',required=True);ap.add_argument('--video',action='store_true');a=ap.parse_args()
    render(json.loads(a.scene.read_text()),np.load(a.path),a.output,a.video)
