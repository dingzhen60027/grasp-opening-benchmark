from pathlib import Path
import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from gobench.world import World,densify,grasp_delta,transform
from gobench.evaluate import draw_errors,task_success,evaluate,interval
from gobench.generate import candidate,solve_witness

ROOT=Path(__file__).resolve().parents[1]
SCENE=ROOT/'data/v0.1/tasks/dev/single_opening_hard_000.json'
PATH=ROOT/'data/v0.1/reference_witnesses/dev/single_opening_hard_000.npy'

def test_rotation_uses_grasp_pivot_and_metric_units():
    attachment=np.eye(4);attachment[0,3]=.5
    xi=np.array([.003,0,0,0,0,np.deg2rad(10)])
    actual=grasp_delta(xi)@attachment
    np.testing.assert_allclose(actual[:3,3],[.003+.5*np.cos(xi[-1]),.5*np.sin(xi[-1]),0],atol=1e-12)

def test_resampling_preserves_endpoints_and_rejects_nan():
    path=np.array([[0]*6,[.12,-.03,0,0,0,.2]])
    dense=densify(path,.01)
    np.testing.assert_array_equal(dense[[0,-1]],path)
    assert np.max(abs(np.diff(dense,axis=0)))<=.010000001
    with pytest.raises(ValueError):densify([[0]*6,[np.nan]*6])
    with pytest.raises(ValueError):densify(path,0)

def test_zero_error_is_exact_nominal_and_full_transfer():
    scene=json.loads(SCENE.read_text());path=np.load(PATH)
    r=evaluate(scene,path,episodes=3,scale=0)
    assert r['status']=='evaluated' and r['collision_count']==0 and r['task_success_count']==3
    bad=path.copy();bad[-1]=path[0]
    assert evaluate(scene,bad,episodes=1)['reason']=='wrong_endpoints'

def test_collision_reacts_to_grasp_error_not_just_arm():
    scene=json.loads(SCENE.read_text());path=np.load(PATH);w=World(scene)
    try:
        distances=[]
        for q in path:w.set_q(q);distances.append(abs(w.update_object()[0,3]-scene['openings'][0]['pose'][0][3]))
        q=path[np.argmin(distances)]
        assert w.collision(q) is None
        assert w.collision(q,[0,0,.03,0,0,0])=='object_environment'
    finally:w.close()

def test_episode_seeds_reproducible_and_ablation_axes_zero():
    s=json.loads(SCENE.read_text())
    np.testing.assert_array_equal(draw_errors(s,8,10),draw_errors(s,8,10))
    assert not np.array_equal(draw_errors(s,8,10),draw_errors(s,8,11))
    assert np.all(draw_errors(s,8,10,mode='translation_only')[:,3:]==0)
    assert np.all(draw_errors(s,8,10,mode='rotation_only')[:,:3]==0)

def test_going_around_wall_is_not_task_completion():
    s=json.loads(SCENE.read_text());g=np.array(s['task']['nominal_object_goal_pose'])
    a=g.copy();a[0,3]=.28
    b=g.copy();b[0,3]=.4;b[1,3]=2
    c=b.copy();c[0,3]=.6
    poses=np.array([a,b,c,g])
    assert task_success(s,poses)==(False,'missed_opening')

def test_joint_model_is_6d_and_ik_fk_agree():
    s,ks=candidate('single_opening','easy',1234);w=World(s)
    try:
        q=w.ik(ks[0],w.home);assert q is not None and q.shape==(6,)
        w.set_q(q);a=w.ee_pose()
        assert np.linalg.norm(a[:3,3]-ks[0][:3,3])<5e-4
        assert Rotation.from_matrix(a[:3,:3].T@ks[0][:3,:3]).magnitude()<2e-3
    finally:w.close()

def test_split_instances_and_seeds_are_disjoint():
    m=json.loads((ROOT/'data/v0.1/manifest.json').read_text())
    entries=m['tasks'];assert len(entries)==60
    dev={e['seed'] for e in entries if e['split']=='dev'};test={e['seed'] for e in entries if e['split']=='test'}
    assert len(dev)==len(test)==30 and not dev&test
    assert interval(0,256)[1]>0 # zero observed events cannot establish zero risk

def test_cached_evaluator_agrees_with_full_collision_checker():
    s=json.loads(SCENE.read_text());path=densify(np.load(PATH),.01)
    r=evaluate(s,path,episodes=16,joint_step=.01)
    w=World(s)
    try:
        expected=[any(w.collision(q,xi) is not None for q in path) for xi in r['error_samples']]
    finally:w.close()
    assert expected==[x['collision'] for x in r['outcomes']]
