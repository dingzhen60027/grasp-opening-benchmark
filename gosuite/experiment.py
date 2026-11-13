"""Run baseline experiments on a generated suite.

    python -m gosuite.experiment --config configs/experiment_baselines_v0.3.json

The config names the suite, split, families, levels, seeds, budget, evaluation episodes,
uncertainty scale/mode and a list of planner entries ``{"label", "planner", "params"}``, where
``planner`` is a registry name (see gosuite.planners) or ``"package.module:function"``.  An entry may
override the seed list with its own ``"seeds"`` (e.g. fewer seeds for ablations) and the planning budget
with its own ``"budget_seconds"`` (anytime studies).

Each (planner entry, task, seed) is one run: plan within the budget, validate the path nominally,
then execute it under the task's common evaluation perturbations.  Results are written to
``<output>/runs/<label>/<task>__s<seed>.json`` (one file per run, so interrupted experiments
resume by skipping existing files) and summarised by gosuite.report.
"""
import argparse
import hashlib
import json
import time
import traceback
from multiprocessing import Pool
from pathlib import Path
import numpy as np
from .evaluate import evaluate_path
from .planners import load_planner
from .planning import PlanningProblem


def select_tasks(suite, split=None, families=None, levels=None, ids=None):
    manifest = json.loads((Path(suite) / 'manifest.json').read_text())
    out = []
    for e in manifest['tasks']:
        if split and e['split'] != split:
            continue
        if families and e['family'] not in families:
            continue
        if levels and e['level'] not in levels:
            continue
        if ids and e['id'] not in ids:
            continue
        out.append(e)
    return manifest, out


def slug(text):
    return ''.join(ch if ch.isalnum() or ch in '-_.' else '_' for ch in text)


def run_one(job):
    entry, cfg, suite, spec, out = job
    task_file = suite / entry['file']
    scene = json.loads(task_file.read_text())
    record = dict(task_id=entry['id'], family=entry['family'], level=entry['level'], split=entry['split'],
                  label=spec['label'], planner=spec['planner'], params=spec.get('params', {}), seed=job_seed(job),
                  budget_seconds=spec.get('budget_seconds', cfg['budget_seconds']), task_sha256=entry['sha256'])
    problem = PlanningProblem(scene, cfg.get('uncertainty_scale', 1.), cfg.get('mode', 'combined'), cfg.get('joint_step', .0025))
    try:
        if spec['planner'] in ('reference', 'reference_nominal'):
            key = 'witness_file' if spec['planner'] == 'reference' else 'witness_nominal_file'
            problem.witness = np.load(suite / entry[key])
        planner = load_planner(spec['planner'])
        begin = time.perf_counter()
        try:
            result = planner(problem, seed=record['seed'], budget=record['budget_seconds'], **spec.get('params', {}))
            error = None
        except Exception:  # a crashing planner is a failed run, not a crashed experiment
            result, error = None, traceback.format_exc(limit=5)
        record['planning_seconds'] = time.perf_counter() - begin
    finally:
        problem.close()
    record['over_budget'] = record['planning_seconds'] > 1.1 * record['budget_seconds'] + 1.
    if result is None:
        record.update(status='planner_error', error=error, path_found=False)
    else:
        record.update(status=result.status, planner_info=_jsonable(result.info), path_found=result.path is not None)
    record['execution'] = None
    record['nominal_valid'] = False
    if record['path_found']:
        path = np.asarray(result.path, dtype=float)
        if spec['planner'] in ('reference', 'reference_nominal'):  # the path IS a stored witness: point at it
            key = 'witness_file' if spec['planner'] == 'reference' else 'witness_nominal_file'
            record['trajectory'] = str(Path(suite) / entry[key])
            record['trajectory_sha256'] = entry[key.replace('_file', '_sha256')]
        else:
            traj = out / 'trajectories' / slug(spec['label']) / f"{entry['id']}__s{record['seed']}.npy"
            traj.parent.mkdir(parents=True, exist_ok=True)
            np.save(traj, path)
            record['trajectory'] = str(traj.relative_to(out))
            record['trajectory_sha256'] = hashlib.sha256(traj.read_bytes()).hexdigest()
        ev = evaluate_path(scene, path, cfg['episodes'], cfg.get('uncertainty_scale', 1.), cfg.get('mode', 'combined'),
                           cfg.get('joint_step', .0025), keep_outcomes=True)
        if ev['status'] == 'evaluated':
            record['nominal_valid'] = True
            outcomes = ev.pop('outcomes')
            ev.pop('error_samples')
            ev['episode_success'] = ''.join('1' if o['success'] else '0' for o in outcomes)
            ev['episode_collision'] = ''.join('1' if o['collision'] else '0' for o in outcomes)
            record['execution'] = ev
        else:
            record['nominal_failure'] = ev.get('nominal')
    record['task_success_rate'] = record['execution']['success_rate'] if record['execution'] else 0.
    dest = run_path(out, spec, entry, record['seed'])
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(record, indent=1, allow_nan=False) + '\n')
    return dest, record


def job_seed(job):
    return job[1]['_seed']


def run_path(out, spec, entry, seed):
    return out / 'runs' / slug(spec['label']) / f"{entry['id']}__s{seed}.json"


def _jsonable(obj):
    return json.loads(json.dumps(obj, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)))


def run(config_path, output=None, workers=None, limit=None):
    cfg = json.loads(Path(config_path).read_text())
    suite = Path(cfg['suite'])
    out = Path(output or cfg['output'])
    out.mkdir(parents=True, exist_ok=True)
    (out / 'config.json').write_text(json.dumps(cfg, indent=1) + '\n')
    manifest, tasks = select_tasks(suite, cfg.get('split'), cfg.get('families'), cfg.get('levels'), cfg.get('task_ids'))
    if limit:
        tasks = tasks[:limit]
    jobs = []
    for spec in cfg['planners']:
        seeds = [0] if spec['planner'] in ('reference', 'reference_nominal') else spec.get('seeds', cfg['seeds'])
        for entry in tasks:
            for seed in seeds:
                if not run_path(out, spec, entry, seed).exists():
                    jobs.append((entry, dict(cfg, _seed=seed), suite, spec, out))
    print(f'{len(jobs)} runs to do ({len(tasks)} tasks, {len(cfg["planners"])} planner entries); output {out}', flush=True)
    begin = time.perf_counter()
    # One process per run: isolates planner state (e.g. OMPL's process-wide RNG seed) and memory.
    with Pool(workers or cfg.get('workers', 8), maxtasksperchild=1) as pool:
        for k, (dest, rec) in enumerate(pool.imap_unordered(run_one, jobs), 1):
            ex = rec['execution']
            tail = f"exec success {ex['success_rate']:.2f}" if ex else rec['status']
            print(f'[{k}/{len(jobs)} {time.perf_counter() - begin:.0f}s] {rec["label"]} {rec["task_id"]} s{rec["seed"]}: '
                  f'{rec["status"]} in {rec["planning_seconds"]:.1f}s, {tail}', flush=True)
    from .report import report
    return report(out)


def main():
    ap = argparse.ArgumentParser(description='Run planner baselines on a gosuite benchmark.')
    ap.add_argument('--config', required=True)
    ap.add_argument('--output', help='override the output directory of the config')
    ap.add_argument('--workers', type=int)
    ap.add_argument('--limit', type=int, help='only the first N selected tasks (smoke tests)')
    a = ap.parse_args()
    run(a.config, a.output, a.workers, a.limit)


if __name__ == '__main__':
    main()
