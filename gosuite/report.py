"""Summarise experiment runs into per-planner tables (CSV + Markdown).

    python -m gosuite.report results/<experiment>

Metric definitions (per run = one planner entry, task and seed):

* planning success   - a path was returned AND it passes nominal validation;
* task success       - fraction of evaluation episodes that are collision-free and satisfy every
                       task rule; a planning failure counts as 0 (unconditional);
* collision | valid  - mean episode collision rate over runs with a valid nominal path;
* times / lengths    - medians over runs with a valid nominal path.

Confidence intervals for task success are 95% percentile bootstraps over TASKS (seed results of
a task are averaged first), so they reflect instance variation rather than episode noise.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
import numpy as np

LEVEL_ORDER = ['easy', 'medium', 'hard']


def load_runs(out):
    return [json.loads(p.read_text()) for p in sorted(Path(out, 'runs').rglob('*.json'))]


def flat(rec):
    ex = rec.get('execution') or {}
    return dict(label=rec['label'], planner=rec['planner'], task_id=rec['task_id'], family=rec['family'], level=rec['level'],
                split=rec['split'], seed=rec['seed'], status=rec['status'], planning_seconds=round(rec['planning_seconds'], 3),
                over_budget=rec['over_budget'], nominal_valid=rec['nominal_valid'], task_success_rate=rec['task_success_rate'],
                collision_rate=ex.get('collision_rate'), episodes=ex.get('episodes'),
                path_joint_length_rad=ex.get('path_joint_length_rad'), ee_path_length_m=ex.get('ee_path_length_m'),
                payload_min_clearance_m=ex.get('payload_min_clearance_m'))


def bootstrap(per_task, n=2000, seed=0):
    values = np.array(list(per_task.values()), dtype=float)
    if len(values) == 0:
        return [float('nan'), float('nan')]
    rng = np.random.default_rng(seed)
    means = values[rng.integers(0, len(values), (n, len(values)))].mean(axis=1)
    return [float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))]


def summarise(rows):
    valid = [r for r in rows if r['nominal_valid']]
    per_task = defaultdict(list)
    for r in rows:
        per_task[r['task_id']].append(r['task_success_rate'])
    per_task = {k: float(np.mean(v)) for k, v in per_task.items()}
    med = lambda key: float(np.median([r[key] for r in valid])) if valid else float('nan')
    return dict(runs=len(rows), tasks=len(per_task),
                planning_success=float(np.mean([r['nominal_valid'] for r in rows])),
                task_success=float(np.mean([r['task_success_rate'] for r in rows])),
                task_success_ci95=bootstrap(per_task),
                collision_given_valid=float(np.mean([r['collision_rate'] for r in valid])) if valid else float('nan'),
                planning_seconds_median=med('planning_seconds'), path_joint_length_median=med('path_joint_length_rad'),
                ee_path_length_median=med('ee_path_length_m'), min_clearance_median_mm=1000 * med('payload_min_clearance_m'),
                over_budget=int(sum(r['over_budget'] for r in rows)))


def report(out, quiet=False):
    out = Path(out)
    rows = [flat(r) for r in load_runs(out)]
    if not rows:
        raise SystemExit(f'no runs under {out}/runs')
    with open(out / 'runs.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    groups = defaultdict(list)
    for r in rows:
        for key in [(r['label'], 'all', 'all'), (r['label'], r['family'], 'all'), (r['label'], r['family'], r['level']),
                    (r['label'], 'all', r['level'])]:
            groups[key].append(r)
    labels = list(dict.fromkeys(r['label'] for r in rows))
    families = sorted({r['family'] for r in rows})
    levels = [l for l in LEVEL_ORDER if any(r['level'] == l for r in rows)]
    summary = []
    for label in labels:
        for fam in ['all'] + families:
            for lev in ['all'] + levels:
                if (label, fam, lev) in groups:
                    summary.append(dict(label=label, family=fam, level=lev, **summarise(groups[(label, fam, lev)])))
    with open(out / 'summary.csv', 'w', newline='') as fh:
        keys = [k for k in summary[0] if k != 'task_success_ci95'] + ['task_success_ci_low', 'task_success_ci_high']
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for s in summary:
            row = {k: v for k, v in s.items() if k != 'task_success_ci95'}
            row['task_success_ci_low'], row['task_success_ci_high'] = s['task_success_ci95']
            w.writerow(row)
    (out / 'summary.json').write_text(json.dumps(summary, indent=1) + '\n')
    lines = ['# Experiment summary', '', f'Runs: {len(rows)}. Task success counts planning failures as 0; '
             'CI = 95% bootstrap over tasks. Collision | valid = mean episode collision rate of valid paths.', '']
    lines += ['| Planner | Family | Level | Runs | Planning success | Task success [95% CI] | Collision \\| valid | '
              'Plan time (s, median) | Min clearance (mm, median) |', '|---|---|---|---:|---:|---:|---:|---:|---:|']
    for s in summary:
        lo, hi = s['task_success_ci95']
        lines.append(f"| {s['label']} | {s['family']} | {s['level']} | {s['runs']} | {100 * s['planning_success']:.0f}% | "
                     f"{100 * s['task_success']:.1f}% [{100 * lo:.1f}, {100 * hi:.1f}] | {100 * s['collision_given_valid']:.1f}% | "
                     f"{s['planning_seconds_median']:.2f} | {s['min_clearance_median_mm']:.1f} |")
    (out / 'summary.md').write_text('\n'.join(lines) + '\n')
    if not quiet:
        print('\n'.join(lines))
    return summary


def main():
    ap = argparse.ArgumentParser(description='Summarise gosuite experiment runs.')
    ap.add_argument('output')
    report(ap.parse_args().output)


if __name__ == '__main__':
    main()
