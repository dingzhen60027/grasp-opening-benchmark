"""Regression tests for the v0.4 suite (planner-controllable risk)."""
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest
from gosuite.evaluate import validate_path
from gosuite.experiment import run_one
from gosuite.families import FAMILIES_V04
from gosuite.world import SuiteWorld

ROOT = Path(__file__).resolve().parents[1]
SUITE = ROOT / 'data' / 'suite_v0.4'
pytestmark = pytest.mark.skipif(not (SUITE / 'manifest.json').exists(), reason='v0.4 suite not generated')
MANIFEST = json.loads((SUITE / 'manifest.json').read_text()) if (SUITE / 'manifest.json').exists() else {'tasks': []}


def load(task_id):
    entry = next(e for e in MANIFEST['tasks'] if e['id'] == task_id)
    return json.loads((SUITE / entry['file']).read_text()), entry


def test_manifest_is_balanced_and_both_witnesses_are_hashed():
    tasks = MANIFEST['tasks']
    assert MANIFEST['suite_version'] == 'v0.4' and len(tasks) == 96 and not MANIFEST['failed']
    for fam in FAMILIES_V04:
        for level in ('easy', 'medium', 'hard'):
            cell = [t for t in tasks if t['family'] == fam and t['level'] == level]
            assert sorted(t['split'] for t in cell) == ['dev'] * 2 + ['test'] * 6
    for t in tasks:
        assert hashlib.sha256((SUITE / t['file']).read_bytes()).hexdigest() == t['sha256']
        assert hashlib.sha256((SUITE / t['witness_file']).read_bytes()).hexdigest() == t['witness_sha256']
        assert hashlib.sha256((SUITE / t['witness_nominal_file']).read_bytes()).hexdigest() == t['witness_nominal_sha256']


@pytest.mark.parametrize('family', sorted(FAMILIES_V04))
def test_both_witnesses_are_valid_and_reach_the_same_goal(family):
    scene, entry = load(f'{family}_medium_dev_00')
    finals = []
    with SuiteWorld(scene) as w:
        for key in ('witness_file', 'witness_nominal_file'):
            path = np.load(SUITE / entry[key])
            report = validate_path(w, scene, path)[0]
            assert report == dict(valid=True, reason=None, state=None), (key, report)
            w.set_q(path[-1])
            finals.append(w.place_payload())
    assert np.linalg.norm(finals[0][:3, 3] - finals[1][:3, 3]) < .002


@pytest.mark.parametrize('family', sorted(FAMILIES_V04))
def test_risk_aware_witness_deviates_from_the_straight_one(family):
    scene, entry = load(f'{family}_hard_dev_00')
    tips = {}
    with SuiteWorld(scene) as w:
        for key in ('witness_file', 'witness_nominal_file'):
            pts = []
            for q in np.load(SUITE / entry[key]):
                w.set_q(q)
                pose = w.place_payload()
                pts.append(pose[:3, :3] @ [scene['payload']['size'][0] / 2, 0, 0] + pose[:3, 3])
            tips[key] = np.array(pts)
    frame = np.asarray(scene['design']['insertion_frame'])
    goal = np.asarray(scene['task']['goal']['object_pose'])[:3, 3]
    axes = frame[:3, [{'y': 1, 'z': 2}[a] for a in scene['design']['deviation_planes']]]   # planes the optimiser may use
    lateral = lambda p: np.abs((p - goal) @ axes).max(axis=1)
    near = lambda p: (p @ frame[:3, 0]) > min(scene['design']['control_depths']) - .05
    robust, straight = tips['witness_file'], tips['witness_nominal_file']
    assert lateral(robust[near(robust)]).max() > lateral(straight[near(straight)]).max() + .005


def test_risk_aware_witness_never_loses_to_the_straight_one_on_planning_draws():
    for t in MANIFEST['tasks']:
        d = t['descriptors']
        if d['planning_risk_nominal'] == 0:   # straight path already risk-free: it is also the risk-aware witness
            assert d['planning_risk_robust'] == 0 and d['robust_rank'] == -1, t['id']
        else:
            assert d['planning_risk_robust'] < d['planning_risk_nominal'], t['id']
    hard = [t for t in MANIFEST['tasks'] if t['level'] == 'hard']
    assert all(t['descriptors']['planning_risk_nominal'] > 0 for t in hard)


def test_harness_runs_the_straight_witness_oracle(tmp_path):
    entry = next(e for e in MANIFEST['tasks'] if e['id'] == 'rack_easy_dev_00')
    cfg = dict(budget_seconds=5, episodes=4, uncertainty_scale=1., mode='combined', joint_step=.0025, _seed=0)
    dest, rec = run_one((entry, cfg, SUITE, dict(label='Straight', planner='reference_nominal', params={}), tmp_path))
    assert rec['nominal_valid'] and rec['trajectory_sha256'] == entry['witness_nominal_sha256']


# ---- reporting and comparison ------------------------------------------------------------------

def test_report_never_borrows_another_suites_goal_flags(tmp_path):
    from gosuite.report import find_feasibility
    out = tmp_path / 'suite_vX' / 'my_method'
    (out / 'runs').mkdir(parents=True)
    assert find_feasibility(out) == (None, None)   # used to fall back to results/suite_v0.3 (same task ids)
    with pytest.raises(SystemExit):
        find_feasibility(out, tmp_path / 'missing.json')
    flags = out.parent / 'validation' / 'goal_feasibility.json'
    flags.parent.mkdir()
    flags.write_text(json.dumps({'t': '01'}))
    assert find_feasibility(out) == ({'t': '01'}, flags)


def test_paired_comparison_averages_seeds_before_differencing():
    from gosuite.compare import metrics, paired
    row = lambda task, seed, s, valid=True: dict(task_id=task, seed=seed, task_success_rate=s, nominal_valid=valid, collision_rate=1 - s,
                                                 feasible_episode_success=s, level='easy', family='rack')
    a = [row('t1', 0, .9), row('t1', 1, .7), row('t2', 0, .5), row('t2', 1, .5)]   # task means .8 and .5
    b = [row('t1', 0, .6), row('t2', 0, .5)]
    d = paired(a, b)
    assert d['tasks'] == 2 and abs(d['mean'] - .1) < 1e-12 and (d['a_higher'], d['b_higher']) == (1, 0)
    m = metrics(a + [row('t3', 0, 0., valid=False)])
    assert abs(m['task_success'] - (.8 + .5 + 0.) / 3) < 1e-12 and abs(m['valid_path'] - .8) < 1e-12
    assert abs(m['collision_given_valid'] - np.mean([.1, .3, .5, .5])) < 1e-12   # invalid runs excluded


def test_committed_comparison_is_reproducible():
    from gosuite.compare import compare
    cfg = json.loads((ROOT / 'configs' / 'comparison_v0.4.json').read_text())
    committed = ROOT / cfg['output'] / 'comparison.json'
    if not committed.exists():
        pytest.skip('comparison not generated')
    cfg['results'] = [str(ROOT / d) for d in cfg['results']]
    res, old = compare(cfg), json.loads(committed.read_text())
    for key in ('methods', 'pairs', 'by_family', 'seeds', 'episodes'):
        assert json.dumps(res[key], sort_keys=True) == json.dumps(old[key], sort_keys=True), key
