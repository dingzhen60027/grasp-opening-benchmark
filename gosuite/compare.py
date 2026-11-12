"""Paired comparison of methods on one split (the paper's main results table).

    python -m gosuite.compare --config configs/comparison_v0.4.json

Every method is evaluated on the same grasp-error draws per task (common random numbers), so methods
can be compared task by task. A method's score on a task is its mean over seeds. Tables report the
mean over tasks with 95% percentile-bootstrap intervals over tasks (gosuite.report.bootstrap). A paired
difference A - B is the mean over tasks of the per-task differences, with its own bootstrap interval.
Metric definitions are those of gosuite.report. Writes comparison.md and comparison.json.
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path
import numpy as np
from .report import LEVEL_ORDER, bootstrap, find_feasibility, flat, load_runs

FAMILY_ORDER = ['rack', 'rebar', 'lathe', 'tote']
TIE = 1e-9


def load(cfg):
    """Run rows per method name, restricted to the configured split and labels."""
    names = {m['label']: m['name'] for m in cfg['methods']}
    rows, sources, origin = defaultdict(list), {}, {}
    for d in cfg['results']:
        feas, src = find_feasibility(d)
        sources[d] = str(src) if src else None
        for rec in load_runs(d):
            label = rec['label']
            if origin.setdefault(label, d) != d:
                raise SystemExit(f'label {label!r} appears in both {origin[label]} and {d}')
            if rec['split'] == cfg['split'] and label in names:
                rows[names[label]].append(flat(rec, feas))
    missing = [m['name'] for m in cfg['methods'] if not rows[m['name']]]
    if missing:
        raise SystemExit(f'no {cfg["split"]} runs for: {missing}')
    tasks = {name: {r['task_id'] for r in rs} for name, rs in rows.items()}
    first = next(iter(tasks.values()))
    if any(t != first for t in tasks.values()):
        raise SystemExit('methods were run on different task sets; a paired comparison needs the same tasks')
    return rows, sources


def per_task(rows, key):
    by = defaultdict(list)
    for r in rows:
        by[r['task_id']].append(np.nan if r[key] is None else float(r[key]))
    return {t: float(np.mean(v)) for t, v in by.items()}


def metrics(rows):
    succ = per_task(rows, 'task_success_rate')
    feas = [v for v in per_task(rows, 'feasible_episode_success').values() if not np.isnan(v)]
    valid = [r for r in rows if r['nominal_valid']]
    return dict(tasks=len(succ), runs=len(rows), valid_path=float(np.mean([bool(r['nominal_valid']) for r in rows])),
                task_success=float(np.mean(list(succ.values()))), task_success_ci95=bootstrap(succ),
                feasible_episode_success=float(np.mean(feas)) if feas else float('nan'),
                collision_given_valid=float(np.mean([r['collision_rate'] for r in valid])) if valid else float('nan'))


def paired(rows_a, rows_b):
    a, b = per_task(rows_a, 'task_success_rate'), per_task(rows_b, 'task_success_rate')
    diff = {t: a[t] - b[t] for t in sorted(set(a) & set(b))}
    v = np.array(list(diff.values()))
    return dict(tasks=len(v), mean=float(v.mean()), ci95=bootstrap(diff), a_higher=int((v > TIE).sum()), b_higher=int((v < -TIE).sum()))


def subset(rows, level='all', family='all'):
    return [r for r in rows if level in ('all', r['level']) and family in ('all', r['family'])]


def compare(cfg):
    rows, sources = load(cfg)
    names = [m['name'] for m in cfg['methods']]
    present = {r['level'] for rs in rows.values() for r in rs}
    levels = [l for l in LEVEL_ORDER if l in present] + ['all']
    fams = {r['family'] for rs in rows.values() for r in rs}
    families = [f for f in FAMILY_ORDER if f in fams] + sorted(fams - set(FAMILY_ORDER))
    table = {n: {lev: metrics(subset(rows[n], lev)) for lev in levels} for n in names}
    pairs = [dict(a=a, b=b, levels={lev: paired(subset(rows[a], lev), subset(rows[b], lev)) for lev in levels})
             for a, b in cfg.get('pairs', [])]
    by_family = {f: {lev: {n: metrics(subset(rows[n], lev, f))['task_success'] for n in names} for lev in levels[:-1]}
                 for f in families}
    seeds = {n: len({r['seed'] for r in rows[n]}) for n in names}
    episodes = sorted({int(r['episodes']) for rs in rows.values() for r in rs if r['episodes']})
    return dict(name=cfg.get('name'), description=cfg.get('description'), split=cfg['split'], results=cfg['results'],
                feasibility_sources=sources, seeds=seeds, episodes=episodes, levels=levels, families=families,
                methods=table, pairs=pairs, by_family=by_family)


def pct(x):
    return '-' if x is None or np.isnan(x) else f'{100 * x:.1f}'


def markdown(res):
    names, levels = list(res['methods']), res['levels']
    n_tasks = res['methods'][names[0]]['all']['tasks']
    seeds = ', '.join(f'{n} {k}' for n, k in res['seeds'].items())
    out = [f"# {res['name'] or 'Method comparison'}", '', res['description'] or '', '',
           f"Split: {res['split']} ({n_tasks} tasks). Grasp-error draws per task: {', '.join(map(str, res['episodes']))}, the same for every "
           f"method. Seeds per task: {seeds}. Task success counts planning failures as 0. Intervals are 95% bootstrap over tasks. "
           'Goal-feasible episodes: success on the draws whose goal state is itself feasible, with flags from each results '
           "tree's own validation output. Collision | valid: mean episode collision rate of runs with a valid path.", '',
           'Generated by `python -m gosuite.compare`; do not edit by hand.', '',
           '## Task success by level (%)', '',
           '| Method | Valid path | ' + ' | '.join(l.capitalize() for l in levels[:-1]) + ' | All [95% CI] | Goal-feasible episodes | Collision \\| valid |',
           '|---|---:|' + '---:|' * (len(levels) - 1) + '---:|---:|---:|']
    for n in names:
        m = res['methods'][n]
        a = m['all']
        lo, hi = a['task_success_ci95']
        out.append(f"| {n} | {100 * a['valid_path']:.0f}% | " + ' | '.join(pct(m[l]['task_success']) for l in levels[:-1])
                   + f" | {pct(a['task_success'])} [{pct(lo)}, {pct(hi)}] | {pct(a['feasible_episode_success'])} | {pct(a['collision_given_valid'])} |")
    out += ['', '## Per level', '', '| Level | Method | Valid path | Task success [95% CI] | Goal-feasible episodes | Collision \\| valid |',
            '|---|---|---:|---:|---:|---:|']
    for lev in levels:
        for n in names:
            m = res['methods'][n][lev]
            lo, hi = m['task_success_ci95']
            out.append(f"| {lev} | {n} | {100 * m['valid_path']:.0f}% | {pct(m['task_success'])} [{pct(lo)}, {pct(hi)}] | "
                       f"{pct(m['feasible_episode_success'])} | {pct(m['collision_given_valid'])} |")
    if res['pairs']:
        out += ['', '## Paired differences in task success (A - B, percentage points, same draws per task)', '',
                '| A - B | Level | Mean | 95% CI | Tasks where A is higher | Tasks where B is higher | Tasks |', '|---|---|---:|---:|---:|---:|---:|']
        for p in res['pairs']:
            for lev in levels:
                d = p['levels'][lev]
                lo, hi = d['ci95']
                out.append(f"| {p['a']} - {p['b']} | {lev} | {100 * d['mean']:+.1f} | [{100 * lo:+.1f}, {100 * hi:+.1f}] | "
                           f"{d['a_higher']} | {d['b_higher']} | {d['tasks']} |")
    out += ['', '## Task success by family and level (%)', '', '| Family | Level | ' + ' | '.join(names) + ' |',
            '|---|---|' + '---:|' * len(names)]
    for f in res['families']:
        for lev in levels[:-1]:
            out.append(f'| {f} | {lev} | ' + ' | '.join(pct(res['by_family'][f][lev][n]) for n in names) + ' |')
    return '\n'.join(out) + '\n'


def main():
    ap = argparse.ArgumentParser(description='Paired comparison of gosuite results on one split.')
    ap.add_argument('--config', required=True)
    ap.add_argument('--output', help='output directory (default: the config\'s "output")')
    a = ap.parse_args()
    cfg = json.loads(Path(a.config).read_text())
    out = Path(a.output or cfg['output'])
    out.mkdir(parents=True, exist_ok=True)
    res = compare(cfg)
    (out / 'comparison.json').write_text(json.dumps(res, indent=1) + '\n')
    (out / 'comparison.md').write_text(markdown(res))
    print(markdown(res))


if __name__ == '__main__':
    main()
