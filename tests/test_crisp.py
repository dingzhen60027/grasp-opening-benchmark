"""Tests of the proposed method (CRISP) and its exact grasp-error risk model."""
import copy
import json
from pathlib import Path
import numpy as np
import pytest
from gosuite.evaluate import evaluation_errors, planning_errors
from gosuite.geometry import densify, grasp_delta
from gosuite.risk import RiskModel, box_separation
from gosuite.task import TaskChecker
from gosuite.world import SuiteWorld

ROOT = Path(__file__).resolve().parents[1]
SUITE = ROOT / 'data' / 'suite_v0.4'
pytestmark = pytest.mark.skipif(not (SUITE / 'manifest.json').exists(), reason='v0.4 suite not generated')
MANIFEST = json.loads((SUITE / 'manifest.json').read_text()) if (SUITE / 'manifest.json').exists() else {'tasks': []}


def load(task_id):
    entry = next(e for e in MANIFEST['tasks'] if e['id'] == task_id)
    return json.loads((SUITE / entry['file']).read_text()), entry


def payload_path(world, path):
    poses, ee = [], []
    for q in path:
        world.set_q(q)
        ee.append(world.ee_pose())
        poses.append(world.place_payload())
    return np.array(poses), np.array(ee)


def test_box_separation_signs_on_simple_cases():
    I = np.eye(3)
    assert box_separation(I, np.zeros(3), np.ones(3), I, np.array([2.5, 0, 0]), np.ones(3)) == pytest.approx(.5)
    assert box_separation(I, np.zeros(3), np.ones(3), I, np.array([1.5, 0, 0]), np.ones(3)) == pytest.approx(-.5)
    r = np.array([[np.cos(.3), -np.sin(.3), 0], [np.sin(.3), np.cos(.3), 0], [0, 0, 1]])
    assert box_separation(I, np.zeros(3), np.ones(3), r, np.array([0, 3.5, 0]), np.ones(3)) > 0


@pytest.mark.parametrize('task_id', ['rack_hard_dev_00', 'rebar_medium_dev_00', 'tote_hard_dev_01'])
def test_risk_model_agrees_with_the_simulator_and_is_only_conservative(task_id):
    """Per (state, draw): the box model flags every contact/keep-in violation the simulator finds."""
    scene, entry = load(task_id)
    errors = planning_errors(scene, 24, seed=3)
    model = RiskModel(scene, errors)
    with SuiteWorld(scene) as w:
        checker = TaskChecker(scene)
        poses, ee = payload_path(w, densify(np.load(SUITE / entry['witness_nominal_file']), .02))
        sep = model.separations(poses)
        missed = extra = total = 0
        for t in np.flatnonzero(np.isfinite(sep[:, 0])):
            w.set_q(densify(np.load(SUITE / entry['witness_nominal_file']), .02)[t])
            for i, xi in enumerate(errors):
                pose = w.place_payload_pose(ee[t] @ grasp_delta(xi) @ w.T_EO)
                truth = bool(w.payload_environment_hit() or checker.state_violation(pose))
                missed += truth and sep[t, i + 1] > 0
                extra += (not truth) and sep[t, i + 1] <= 0
                total += 1
    assert missed == 0
    assert extra <= .002 * total


def test_pruning_is_exact():
    scene, entry = load('lathe_hard_dev_00')
    model = RiskModel(scene, planning_errors(scene, 32, seed=1))
    with SuiteWorld(scene) as w:
        poses, _ = payload_path(w, densify(np.load(SUITE / entry['witness_file']), .02))
    pruned = model.separations(poses)
    model.displacement[:] = 10.   # every (pair, draw) is tested exactly
    model.reach = 10.
    exact = model.separations(poses)
    near = np.isfinite(pruned)
    assert np.array_equal(pruned <= 0, exact <= 0)                 # identical failure indicators,
    assert np.allclose(pruned[pruned <= 0], exact[pruned <= 0])    # every failing margin is exact,
    assert np.all(exact[~near] > 0)                                # and 'inf' (no pair within reach) never hides a contact


def test_final_state_rules_match_the_suite_goal_flags():
    flags_file = ROOT / 'results' / 'suite_v0.4' / 'validation' / 'goal_feasibility.json'
    if not flags_file.exists():
        pytest.skip('validation not generated')
    flags = json.loads(flags_file.read_text())
    disagree = total = 0
    for task_id in ['rack_hard_dev_00', 'lathe_medium_dev_01', 'tote_hard_dev_00']:
        scene, _ = load(task_id)
        model = RiskModel(scene, evaluation_errors(scene, 256))
        with SuiteWorld(scene) as w:
            w.set_q(scene['q_goal_reference'])
            fail = model.evaluate(w.place_payload()[None])['failures']
        disagree += sum(f != (c == '1') for f, c in zip(fail, flags[task_id]))
        total += len(fail)
    assert disagree <= .005 * total


def strip_generator_metadata(scene):
    clean = copy.deepcopy(scene)
    for key in ('design', 'descriptors', 'validation', 'generator', 'description'):
        clean.pop(key, None)
    for k, item in enumerate(clean['obstacles']):
        item['role'] = 'obstacle'
    return clean


def _plan(scene):
    from gosuite.planning import PlanningProblem
    from gosuite.planners import crisp
    problem = PlanningProblem(scene)
    try:
        return crisp.plan(problem, seed=0, budget=60, samples=16, popsize=8, max_iterations=2, goal_iterations=2, certify=50).path
    finally:
        problem.close()


def test_crisp_uses_only_the_planner_interface():
    """Removing the generator's design block, descriptors and obstacle roles does not change the plan."""
    from multiprocessing import get_context
    scene, _ = load('rebar_hard_dev_00')
    # OMPL (the transfer) is only imported in worker processes, as in the harness
    with get_context('fork').Pool(2, maxtasksperchild=1) as pool:
        paths = pool.map(_plan, [scene, strip_generator_metadata(scene)])
    assert paths[0] is not None and np.array_equal(paths[0], paths[1])


def test_ccvpsto_padding_reproduces_the_published_thresholds():
    """k_binom(beta=0.05, N=100, eta) gives eta_binom = 0.01 / 0.04 / 0.13 / 0.31 (Brudermueller et al., Table 2)."""
    from gosuite.planners.ccvpsto import k_binom
    assert [k_binom(.05, 100, eta) for eta in (.05, .1, .2, .4)] == [1, 4, 13, 31]


@pytest.mark.parametrize('config', ['comparison_crisp_v0.4.json', 'comparison_crisp_ablation_v0.4.json'])
def test_committed_crisp_comparisons_are_reproducible(config):
    from gosuite.compare import compare
    cfg = json.loads((ROOT / 'configs' / config).read_text())
    committed = ROOT / cfg['output'] / 'comparison.json'
    if not committed.exists():
        pytest.skip('comparison not generated')
    cfg['results'] = [str(ROOT / d) for d in cfg['results']]
    res, old = compare(cfg), json.loads(committed.read_text())
    for key in ('methods', 'pairs', 'seeds', 'episodes'):
        assert json.dumps(res[key], sort_keys=True) == json.dumps(old[key], sort_keys=True), key


def test_robustness_counts_tasks_without_a_path_as_zero(tmp_path):
    from gosuite.robustness import summarise
    tasks = [e['id'] for e in MANIFEST['tasks'] if e['split'] == 'test']
    cfg = dict(suite=str(SUITE), split='test', methods=[dict(name='m')], conditions=[dict(scale=1., mode='combined')])
    rows = [dict(method='m', task=t, scale=1., mode='combined', success=1.) for t in tasks[1:]]   # one task has no path
    table = summarise(cfg, rows)
    assert abs(table['m']['combined, s = 1']['mean'] - (len(tasks) - 1) / len(tasks)) < 1e-12


def test_paper_tables_are_generated_from_committed_outputs(tmp_path):
    if not (ROOT / 'results' / 'suite_v0.4' / 'comparison_crisp_test' / 'comparison.json').exists():
        pytest.skip('comparison not generated')
    import subprocess, sys
    subprocess.run([sys.executable, '-m', 'gosuite.paper_tables', '--output', str(tmp_path)], cwd=ROOT, check=True, capture_output=True)
    text = (tmp_path / 'main_results.tex').read_text()
    assert r'\textbf{CRISP (proposed)}' in text and r'\toprule' in text
