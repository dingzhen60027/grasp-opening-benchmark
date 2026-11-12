"""Validity analyses of a generated suite (run after the reference evaluation).

    python -m gosuite.validate --suite data/suite_v0.3 --reference results/suite_v0.3/reference \
        --output results/suite_v0.3/validation

1. Headroom: for every task, the fraction of the common evaluation draws for which the GOAL state
   itself fails (collision, keep-in, goal tolerance or placement).  Such episodes are lost by any
   path that ends at q_goal_reference, so 1 - p_goal is an upper bound for those paths, and
   bound - reference success is the part of the reference failure that a better path could avoid.
2. Step convergence: the reference paths re-evaluated at half the joint step.
3. Sensitivity: uncertainty-scale sweep and translation-only / rotation-only ablations (dev split).
4. Descriptor validity: rank correlation of difficulty descriptors with reference success.
5. Variance components for planning the number of tasks, seeds and episodes.
"""
import argparse
import json
from collections import Counter, defaultdict
from multiprocessing import Pool
from pathlib import Path
import numpy as np
from scipy.stats import spearmanr
from .evaluate import evaluate_path, evaluation_errors
from .geometry import grasp_delta
from .task import TaskChecker, placement_violation
from .world import SuiteWorld

LEVELS = ['easy', 'medium', 'hard']
FAMILIES = ['rack', 'rebar', 'lathe', 'tote']
SCALES = [0., .5, 1., 1.5, 2., 3.]


def goal_state_failures(scene, errors, q=None, world=None):
    """Per-draw failure of a single (goal) configuration: contact, keep-in, goal tolerance, placement."""
    own = world is None
    world = world or SuiteWorld(scene)
    checker = TaskChecker(scene)
    try:
        world.set_q(np.asarray(scene['q_goal_reference'] if q is None else q))
        ee = world.ee_pose()
        reasons = []
        for xi in errors:
            pose = world.place_payload_pose(ee @ grasp_delta(xi) @ world.T_EO)
            hit = world.payload_environment_hit() or world.payload_robot_hit()
            if hit:
                reasons.append('contact:' + str(hit.get('obstacle', hit.get('link'))).split('_')[0])
                continue
            if checker.state_violation(pose):
                reasons.append('keep_in')
                continue
            dp = np.linalg.norm(pose[:3, 3] - checker.goal[:3, 3])
            ang = np.arccos(np.clip((np.trace(checker.goal[:3, :3].T @ pose[:3, :3]) - 1) / 2, -1, 1))
            if dp > checker.goal_position_tol or ang > checker.goal_angle_tol:
                reasons.append('goal_tolerance')
                continue
            if placement_violation(scene, pose):
                reasons.append('placement')
                continue
            reasons.append(None)
        return reasons
    finally:
        if own:
            world.close()


GOAL_OFFSETS_MM = [(dx, dy, dz) for dx in (-8, 0, 8) for dy in (-4, 0, 4) for dz in (-4, -2, 0, 2, 4, 6)]


def headroom_job(job):
    """Goal-state failure at q_goal_reference, plus the best static goal pose found inside the goal region."""
    suite, entry, episodes = job
    scene = json.loads((suite / entry['file']).read_text())
    errors = evaluation_errors(scene, episodes)
    world = SuiteWorld(scene)
    try:
        reasons = goal_state_failures(scene, errors, world=world)
        goal = np.asarray(scene['task']['goal']['object_pose'])
        tol = scene['task']['goal']['position_tolerance_m']
        to_tcp = np.linalg.inv(world.T_EO)
        axis = goal[:3, 0]
        lateral = np.cross([0, 0, 1.], axis)
        lateral = lateral / np.linalg.norm(lateral) if np.linalg.norm(lateral) > 1e-6 else goal[:3, 1]
        best = (sum(r is None for r in reasons) / len(reasons), (0, 0, 0))
        for dx, dy, dz in GOAL_OFFSETS_MM:
            shift = (dx * axis + dy * lateral + dz * np.array([0, 0, 1.])) / 1000
            if (dx, dy, dz) == (0, 0, 0) or np.linalg.norm(shift) > .8 * tol:
                continue
            pose = goal.copy()
            pose[:3, 3] += shift
            q = world.ik(pose @ to_tcp, np.asarray(scene['q_goal_reference']))
            if q is None or world.check(q):
                continue
            alt = goal_state_failures(scene, errors, q=q, world=world)
            ok = sum(r is None for r in alt) / len(alt)
            if ok > best[0]:
                best = (ok, (dx, dy, dz))
    finally:
        world.close()
    return dict(id=entry['id'], goal_fail=''.join('0' if r is None else '1' for r in reasons),
                goal_reasons=dict(Counter(r for r in reasons if r)), region_bound=best[0], region_offset_mm=best[1])


def eval_job(job):
    suite, entry, episodes, scale, mode, step = job
    scene = json.loads((suite / entry['file']).read_text())
    r = evaluate_path(scene, np.load(suite / entry['witness_file']), episodes, scale, mode, step, keep_outcomes=True, metrics=False)
    bits = ''.join('1' if o['success'] else '0' for o in r['outcomes'])
    return dict(id=entry['id'], scale=scale, mode=mode, step=step, success=bits)


def load_reference(reference, planner='reference'):
    out = {}
    for p in Path(reference, 'runs').rglob('*.json'):
        r = json.loads(p.read_text())
        if r['planner'] == planner and r.get('execution'):
            out[r['task_id']] = r['execution']['episode_success']
    return out


def rate(bits):
    return sum(c == '1' for c in bits) / len(bits)


def validate(suite, reference, output, workers=20, episodes=256):
    suite, output = Path(suite), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((suite / 'manifest.json').read_text())
    tasks = manifest['tasks']
    ref = load_reference(reference)
    straight = load_reference(reference, 'reference_nominal')   # v0.4: straight witness along the goal line
    dev = [e for e in tasks if e['split'] == 'dev']
    dev0 = [e for e in dev if e['index'] == 0]
    with Pool(workers) as pool:
        head = {h['id']: h for h in pool.map(headroom_job, [(suite, e, episodes) for e in tasks])}
        conv = pool.map(eval_job, [(suite, e, episodes, 1., 'combined', .00125) for e in dev0])
        sens = pool.map(eval_job, [(suite, e, episodes, s, 'combined', .0025) for e in dev for s in SCALES if s != 1.]
                        + [(suite, e, episodes, 1., m, .0025) for e in dev for m in ('translation_only', 'rotation_only')])

    # 1. headroom decomposition
    rows = []
    for e in tasks:
        s, g = ref[e['id']], head[e['id']]['goal_fail']
        n = len(s)
        succ = rate(s)
        unavoidable = sum(gc == '1' for gc in g) / n
        transit = sum(sc == '0' and gc == '0' for sc, gc in zip(s, g)) / n
        rows.append(dict(id=e['id'], family=e['family'], level=e['level'], split=e['split'], reference_success=succ,
                         straight_success=rate(straight[e['id']]) if e['id'] in straight else None,
                         goal_failure=unavoidable, bound=1 - unavoidable, path_failure=transit,
                         region_bound=head[e['id']]['region_bound'], region_offset_mm=head[e['id']]['region_offset_mm'],
                         goal_reasons=head[e['id']]['goal_reasons']))
    cells = defaultdict(list)
    for r in rows:
        cells[(r['family'], r['level'])].append(r)
        cells[('all', r['level'])].append(r)
        cells[(r['family'], 'all')].append(r)
        cells[('all', 'all')].append(r)
    headroom = {f'{f}/{l}': dict(tasks=len(v), reference_success=float(np.mean([r['reference_success'] for r in v])),
                                 goal_failure=float(np.mean([r['goal_failure'] for r in v])),
                                 path_failure=float(np.mean([r['path_failure'] for r in v])),
                                 bound=float(np.mean([r['bound'] for r in v])),
                                 region_bound=float(np.mean([r['region_bound'] for r in v])),
                                 straight_success=(float(np.mean([r['straight_success'] for r in v]))
                                                   if all(r['straight_success'] is not None for r in v) else None),
                                 goal_reasons=dict(sum((Counter(r['goal_reasons']) for r in v), Counter())))
                for (f, l), v in cells.items()}
    (output / 'goal_feasibility.json').write_text(json.dumps(
        {i: h['goal_fail'] for i, h in head.items()}, indent=0) + '\n')

    # 2. convergence of the evaluation step
    conv_rows = []
    for c in conv:
        a, b = ref[c['id']], c['success']
        conv_rows.append(dict(id=c['id'], success_step_0025=rate(a), success_step_00125=rate(b),
                              differing_episodes=sum(x != y for x, y in zip(a, b))))

    # 3. sensitivity
    by = defaultdict(dict)
    for e in dev:
        by[e['id']][('combined', 1.)] = rate(ref[e['id']])
    for s in sens:
        by[s['id']][(s['mode'], s['scale'])] = rate(s['success'])
    fam_of = {e['id']: e['family'] for e in tasks}
    lev_of = {e['id']: e['level'] for e in tasks}
    sweep = {}
    for fam in FAMILIES + ['all']:
        for lev in LEVELS + ['all']:
            ids = [i for i in by if (fam == 'all' or fam_of[i] == fam) and (lev == 'all' or lev_of[i] == lev)]
            if not ids:
                continue
            sweep[f'{fam}/{lev}'] = {
                'combined': {str(s): float(np.mean([by[i][('combined', s)] for i in ids])) for s in SCALES},
                'translation_only': float(np.mean([by[i][('translation_only', 1.)] for i in ids])),
                'rotation_only': float(np.mean([by[i][('rotation_only', 1.)] for i in ids]))}
    monotone = all(all(by[i][('combined', a)] >= by[i][('combined', b)] - 2 / episodes for a, b in zip(SCALES, SCALES[1:])) for i in by)

    # 4. descriptor validity
    desc = {}
    succ = np.array([rate(ref[e['id']]) for e in tasks])
    for key in ('design_clearance_m', 'tip_sensitivity', 'lever_arm_m', 'payload_length_m', 'min_nominal_clearance_m',
                'constrained_path_length_m', 'portals'):
        vals = np.array([e['descriptors'][key] for e in tasks], dtype=float)
        rho, pv = spearmanr(vals, succ)
        desc[key] = dict(spearman_rho=float(rho), p_value=float(pv))
    within_family = {}
    for fam in FAMILIES:
        idx = [k for k, e in enumerate(tasks) if e['family'] == fam]
        rho, pv = spearmanr([tasks[k]['descriptors']['tip_sensitivity'] for k in idx], succ[idx])
        within_family[fam] = dict(spearman_rho=float(rho), p_value=float(pv))

    # 5. variance components (reference success)
    var = {}
    for fam in FAMILIES:
        for lev in LEVELS:
            v = [rate(ref[e['id']]) for e in tasks if e['family'] == fam and e['level'] == lev]
            p = float(np.mean(v))
            var[f'{fam}/{lev}'] = dict(between_task_sd=float(np.std(v, ddof=1)),
                                       within_task_binomial_se_256=float(np.sqrt(p * (1 - p) / episodes)))

    result = dict(episodes=episodes, headroom=headroom, headroom_tasks=rows, step_convergence=conv_rows,
                  sensitivity=sweep, sensitivity_monotone=monotone, descriptor_correlation=desc,
                  tip_sensitivity_within_family=within_family, variance=var)
    (output / 'validation.json').write_text(json.dumps(result, indent=1) + '\n')
    return result


def main():
    ap = argparse.ArgumentParser(description='Validity analyses for a gosuite benchmark.')
    ap.add_argument('--suite', default='data/suite_v0.3')
    ap.add_argument('--reference', default='results/suite_v0.3/reference')
    ap.add_argument('--output', default='results/suite_v0.3/validation')
    ap.add_argument('--workers', type=int, default=20)
    a = ap.parse_args()
    r = validate(a.suite, a.reference, a.output, a.workers)
    print(json.dumps({k: r[k] for k in ('headroom', 'sensitivity_monotone', 'descriptor_correlation')}, indent=1)[:4000])


if __name__ == '__main__':
    main()
