"""Regression tests for the v0.3 multi-environment suite (gosuite)."""
import copy
import hashlib
import json
from pathlib import Path
import numpy as np
import pybullet as p
import pytest
from gosuite.evaluate import evaluate_path, evaluation_errors, planning_errors, validate_path
from gosuite.experiment import run_one
from gosuite.families import FAMILIES, rack
from gosuite.geometry import densify, from_rt, tool_horizontal
from gosuite.planning import PlanningProblem
from gosuite.task import TaskChecker, clip_to_box, keep_in_violation, payload_segments, world_segments
from gosuite.witness import follow_knots
from gosuite.world import SuiteWorld

ROOT = Path(__file__).resolve().parents[1]
SUITE = ROOT / 'data' / 'suite_v0.3'
MANIFEST = json.loads((SUITE / 'manifest.json').read_text())


def load(task_id):
    entry = next(e for e in MANIFEST['tasks'] if e['id'] == task_id)
    return json.loads((SUITE / entry['file']).read_text()), np.load(SUITE / entry['witness_file']), entry


def test_manifest_has_balanced_geometry_level_splits_and_valid_hashes():
    tasks = MANIFEST['tasks']
    assert len(tasks) == 96 and not MANIFEST['failed']
    assert len({t['id'] for t in tasks}) == 96 and len({t['seed'] for t in tasks}) == 96
    for fam in FAMILIES:
        for level in ('easy', 'medium', 'hard'):
            cell = [t for t in tasks if t['family'] == fam and t['level'] == level]
            assert sorted(t['split'] for t in cell) == ['dev'] * 2 + ['test'] * 6
    for t in tasks:
        assert hashlib.sha256((SUITE / t['file']).read_bytes()).hexdigest() == t['sha256']
        assert hashlib.sha256((SUITE / t['witness_file']).read_bytes()).hexdigest() == t['witness_sha256']


@pytest.mark.parametrize('family', sorted(FAMILIES))
def test_each_family_witness_passes_full_nominal_validation(family):
    scene, path, _ = load(f'{family}_medium_dev_00')
    with SuiteWorld(scene) as w:
        report = validate_path(w, scene, path)[0]
    assert report == dict(valid=True, reason=None, state=None)
    assert scene['design']['clearance_per_side_m'] == .010


def test_wrong_bay_entry_is_rejected_although_collision_free():
    """The v0.2 bug: entering the outer bay and sliding sideways over the dividers must fail the keep-in rule."""
    rejected = 0
    for entry in [e for e in MANIFEST['tasks'] if e['family'] == 'rack' and e['split'] == 'dev']:
        scene, _, _ = load(entry['id'])
        _, knots, seeds = rack.build(scene['design']['params'])
        k = list(knots)
        x_pre, z = k[4][0, 3], k[4][2, 3]
        k[3] = tool_horizontal([x_pre, -.47, z], 0.)
        k.insert(4, tool_horizontal([x_pre + .10, -.47, z], 0.))   # tube tip 0.075 m inside the outer bay
        k[5] = tool_horizontal([x_pre + .10, k[5][1, 3], z], 0.)   # slide sideways to the target bay
        with SuiteWorld(scene) as w:
            path, failure = follow_knots(w, k, seeds)
            if failure:  # this layout physically blocks the sweep (e.g. a beam); nothing to test here
                continue
            report = validate_path(w, scene, densify(path, .005))[0]
        assert report['reason'] == 'keep_in:rack_target_bay', entry['id']
        rejected += 1
    assert rejected >= 3


def test_keep_in_clipping_is_exact_on_a_simple_case():
    rule = dict(name='r', restricted=dict(pose=np.eye(4).tolist(), size=[1, 1, 1]),
                allowed=dict(pose=from_rt(np.eye(3), [0, .3, 0]).tolist(), size=[1, .2, 1]), tolerance_m=0)
    segs = payload_segments([.2, .02, .02])
    inside_allowed = from_rt(np.eye(3), [0, .3, 0])
    outside_allowed = from_rt(np.eye(3), [0, 0, 0])
    beyond_restricted = from_rt(np.eye(3), [0, .9, 0])
    poses = np.array([inside_allowed, beyond_restricted, outside_allowed])
    assert keep_in_violation(rule, world_segments(poses[:2], segs)) is None
    assert keep_in_violation(rule, world_segments(poses, segs)) == 2
    t0, t1, ok = clip_to_box(np.array([[-2., 0, 0]]), np.array([[2., 0, 0]]), np.array([.5, .5, .5]))
    assert ok[0] and np.isclose(t0[0], .375) and np.isclose(t1[0], .625)


def test_portals_must_be_crossed_in_order():
    scene, path, _ = load('rebar_medium_dev_00')
    checker = TaskChecker(scene)
    with SuiteWorld(scene) as w:
        poses = []
        for q in densify(path, .005):
            w.set_q(q)
            poses.append(w.place_payload())
    poses = np.array(poses)
    assert checker(poses)[0]
    assert checker(poses[::-1])[1].startswith('portal')


def test_broad_phase_never_drops_a_brute_force_contact():
    rng = np.random.default_rng(3)
    for family in sorted(FAMILIES):
        scene, path, _ = load(f'{family}_hard_dev_00')
        with SuiteWorld(scene) as w:
            for q in path[::max(1, len(path) // 25)]:
                q = q + rng.normal(scale=.03, size=6)
                xi = rng.normal(size=6) * np.r_[[.006] * 3, [.05] * 3]
                if np.any(q < w.lower) or np.any(q > w.upper):
                    continue
                w.set_q(q)
                w.place_payload(xi)
                brute_payload = any(p.getClosestPoints(w.payload, b, 0, physicsClientId=w.cid) for b in w.obstacles)
                brute_robot = any(p.getClosestPoints(w.robot, b, 0, physicsClientId=w.cid) for b in w.obstacles)
                assert (w.payload_environment_hit() is not None) == brute_payload
                assert (w.robot_environment_hit() is not None) == brute_robot


def test_common_random_numbers_and_disjoint_planning_stream():
    scene, _, _ = load('tote_easy_dev_00')
    a = evaluation_errors(scene, 64)
    np.testing.assert_array_equal(a, evaluation_errors(scene, 64))
    np.testing.assert_array_equal(a[:16], evaluation_errors(scene, 16))
    assert not np.isclose(planning_errors(scene, 64, seed=0)[:, None], a[None]).all(axis=2).any()
    assert np.all(evaluation_errors(scene, 8, mode='translation_only')[:, 3:] == 0)
    assert np.all(evaluation_errors(scene, 8, mode='rotation_only')[:, :3] == 0)


def test_zero_uncertainty_execution_reproduces_nominal_success():
    scene, path, _ = load('lathe_hard_dev_00')
    r = evaluate_path(scene, path, episodes=3, scale=0., metrics=False)
    assert r['status'] == 'evaluated' and r['success_count'] == 3 and r['collision_count'] == 0


def test_planning_problem_rejects_wrong_start_and_flags_collisions():
    scene, path, _ = load('rack_easy_dev_00')
    prob = PlanningProblem(scene)
    try:
        assert prob.is_valid(prob.q_start) and prob.in_goal_region(prob.q_goal_reference)
        bad = path.copy()
        bad[0] = bad[0] + .1
        assert prob.validate(bad)[0]['reason'] == 'wrong_start'
        assert prob.check(prob.q_start + 7.)['reason'] == 'joint_limits'
        mid = path[len(path) // 2]
        assert prob.check(mid, margin=.5)['reason'] == 'payload_environment'  # a 0.5 m margin cannot hold mid-task
    finally:
        prob.close()


def test_scenario_check_counts_colliding_grasp_samples_against_the_risk_bound():
    scene, _, _ = load('tote_hard_dev_00')
    prob = PlanningProblem(scene)
    try:
        goal = prob.q_goal_reference
        samples = prob.planning_errors(16, seed=0) * 3.  # 6 mm / 3 deg: the rod cannot stay clear of 5 mm cell walls
        hit = prob.check(goal, scenarios=samples, risk=0.)
        assert hit['reason'] == 'scenario_risk' and hit['colliding'] == 1
        assert prob.check(goal, scenarios=samples, risk=1.) is None
        assert prob.check(goal) is None  # nominal goal state stays valid, payload restored to nominal
        np.testing.assert_allclose(prob.world.payload_T, prob.world.ee_pose() @ prob.world.T_EO)
    finally:
        prob.close()


def test_layout_conflict_is_detected():
    scene, _, _ = load('tote_medium_dev_00')
    s = copy.deepcopy(scene)
    s['obstacles'].append(dict(name='intruder', role='tote', group='station', size=[.1, .1, .1], color=[1, 0, 0, 1],
                               pose=from_rt(np.eye(3), [.27, -.39, .05]).tolist()))
    with SuiteWorld(s) as w:
        assert w.layout_conflict().startswith('layout_overlap:intruder')


def test_harness_runs_reference_oracle_end_to_end(tmp_path):
    entry = next(e for e in MANIFEST['tasks'] if e['id'] == 'rebar_easy_dev_00')
    cfg = dict(budget_seconds=5, episodes=4, uncertainty_scale=1., mode='combined', joint_step=.0025, _seed=0)
    spec = dict(label='Reference', planner='reference', params={})
    dest, rec = run_one((entry, cfg, SUITE, spec, tmp_path))
    assert dest.exists() and rec['nominal_valid'] and rec['execution']['episodes'] == 4
    assert len(rec['execution']['episode_success']) == 4
