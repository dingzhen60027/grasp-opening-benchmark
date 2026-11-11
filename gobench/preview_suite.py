"""Rebuild overview from actual task JSON and mesh renderings, including plan-view slices."""
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Rectangle
import imageio.v2 as imageio
from .generate import FAMILIES
from .render import render
from .world import World

def main():
    fig,axes=plt.subplots(2,3,figsize=(17,11),layout='constrained',facecolor='#f5f7fa')
    labels=['Single opening','Deep tunnel','Offset openings','Shelf entry','Turning openings']
    root=Path('data/v0.1')
    for ax,f,label in zip(axes.flat,FAMILIES,labels):
        name=f+'_hard_000';scene=json.loads((root/'tasks/dev'/f'{name}.json').read_text());path=np.load(root/'reference_witnesses/dev'/f'{name}.npy')
        render(scene,path,'previews/'+f)
        ax.imshow(imageio.imread('previews/'+f+'/scene.png'));ax.set_title(label,fontsize=17,fontweight='bold',pad=14);ax.axis('off')
        inset=ax.inset_axes([.60,.53,.39,.44]);inset.set_facecolor('#ffffff')
        z=scene['task']['nominal_object_goal_pose'][2][3]
        for o in scene['obstacles']:
            t=np.array(o['pose']);sx,sy,sz=np.array(o['size'])/2
            if abs(t[2,3]-z)>sz:continue
            xy=np.array([[-sx,-sy,0],[sx,-sy,0],[sx,sy,0],[-sx,sy,0]])@t[:3,:3].T+t[:3,3]
            inset.add_patch(Polygon(xy[:,:2],facecolor='#65788b',alpha=.8))
        w=World(scene);xy=[]
        for q in path:w.set_q(q);xy.append(w.update_object()[:2,3])
        w.close();xy=np.array(xy);inset.plot(xy[:,0],xy[:,1],color='#db8800',linewidth=2)
        inset.scatter(*xy[0],c='#078b69',s=20);inset.scatter(*xy[-1],c='#c73c42',s=20)
        inset.set_xlim(.22,.93);inset.set_ylim(-.32,.32);inset.set_aspect('equal');inset.set_xticks([]);inset.set_yticks([]);inset.set_title('Top slice at payload height',fontsize=8,pad=3)
    ax=axes.flat[5];ax.axis('off')
    ax.text(.04,.96,'UR5e / 6 DOF\n\n5 families × 3 gaps × 4 instances\n= 60 validated nominal tasks\n\nClearance: 25 / 12 / 6 mm\n30 development + 30 test\n\nOrange: rigid payload\nBlue: UR5e collision mesh\nGrey: solid environment\n\nInsets: top-view geometric slices\nGreen / red: start / goal\n\nFixed grasp error per execution',va='top',fontsize=16,color='#223344',linespacing=1.5)
    fig.suptitle('Grasp Opening Benchmark v0.1',fontsize=26,fontweight='bold')
    fig.savefig('previews/overview.png',dpi=140);plt.close(fig)
if __name__=='__main__':main()
