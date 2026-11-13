"""Figures for the CRISP results (matplotlib, light surface, same style as gosuite.figures).

    python -m gosuite.crisp_figures --main configs/comparison_crisp_v0.4.json \
        --ablation configs/comparison_crisp_ablation_v0.4.json --output results/suite_v0.4/comparison_crisp_test

Writes, from the committed run records only (no new evaluation):
  methods_by_level.png        task success per level, 95% bootstrap CIs over tasks (dot plot, oracles in grey)
  paired_differences.png      per-task CRISP - B6 and CRISP - risk-aware oracle, sorted
  ablations.png               the ablation rows, same encoding as methods_by_level
  certificate_calibration.png CRISP's certificate (fresh planning draws) vs the observed failure per run

    python -m gosuite.crisp_figures --risk-bounds data/suite_v0.4 --output docs/figures

  risk_bounds.png / .json     Proposition 1 on the dev straight insertions: trajectory-level risk against the
                              state-wise maximum and the Boole sum (1000 planning draws, no evaluation draws)
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from .compare import load, per_task, subset  # noqa: E402
from .figures import GRID, INK, INK2, LEVEL_COLORS, SURFACE  # noqa: E402
from .report import bootstrap  # noqa: E402

ACCENT = '#c2410c'      # the proposed method
METHOD = '#2a78d6'      # other planners
ORACLE = '#8d8b85'      # privileged references
LEVELS = ['easy', 'medium', 'hard', 'all']


def _style(ax):
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right', 'left'):
        ax.spines[side].set_visible(False)
    ax.spines['bottom'].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=8)
    ax.xaxis.grid(True, color=GRID, linewidth=.8, zorder=0)


def _colour(name):
    if 'oracle' in name.lower():
        return ORACLE
    return ACCENT if 'CRISP' in name else METHOD


def dot_plot(cfg, path, title):
    rows, _ = load(cfg)
    names = [m['name'] for m in cfg['methods']]
    fig, axes = plt.subplots(1, len(LEVELS), figsize=(11, .42 * len(names) + 1.4), dpi=170, sharey=True)
    fig.patch.set_facecolor(SURFACE)
    for ax, lev in zip(axes, LEVELS):
        _style(ax)
        for i, n in enumerate(names):
            s = per_task(subset(rows[n], lev), 'task_success_rate')
            mean, (lo, hi) = 100 * np.mean(list(s.values())), [100 * v for v in bootstrap(s)]
            y = len(names) - 1 - i
            c = _colour(n)
            ax.plot([lo, hi], [y, y], color=c, linewidth=2, solid_capstyle='round', zorder=2)
            ax.plot(mean, y, 'o', color=c, markersize=7, markeredgecolor=SURFACE, markeredgewidth=1.5, zorder=3)
            ax.text(mean, y + .28, f'{mean:.1f}', ha='center', va='bottom', fontsize=7, color=INK2)
        ax.set_title(lev.capitalize() if lev != 'all' else 'All levels', fontsize=9.5, color=INK)
        ax.set_xlim(0, 101)
        ax.set_xticks(range(0, 101, 25))
        ax.set_xticklabels([f'{v}%' for v in range(0, 101, 25)])
    axes[0].set_yticks(range(len(names)))
    axes[0].set_yticklabels(names[::-1], fontsize=8.5, color=INK)
    axes[0].set_ylim(-.6, len(names) - .2)
    fig.suptitle(title, x=.01, ha='left', fontsize=10.5, color=INK, fontweight='bold')
    fig.text(.01, .005, 'Task success under grasp error (mean over tasks of the per-task mean over seeds; bars: 95% bootstrap CI over tasks). '
             'Grey: privileged oracle witnesses, not methods.', fontsize=7.5, color=INK2)
    fig.tight_layout(rect=(0, .03, 1, .95))
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def paired_differences(cfg, path, a='CRISP (proposed)', others=('B6 Keyframe-guided', 'CC-VPSTO (adapted)', 'Risk-aware oracle')):
    rows, _ = load(cfg)
    fig, axes = plt.subplots(1, len(others), figsize=(11, 3.4), dpi=170, sharey=True)
    fig.patch.set_facecolor(SURFACE)
    sa = per_task(rows[a], 'task_success_rate')
    level = {r['task_id']: r['level'] for r in rows[a]}
    for ax, b in zip(axes, others):
        _style(ax)
        ax.xaxis.grid(False)
        ax.yaxis.grid(True, color=GRID, linewidth=.8, zorder=0)
        sb = per_task(rows[b], 'task_success_rate')
        d = sorted(((100 * (sa[t] - sb[t]), level[t]) for t in sa), key=lambda x: x[0])
        ax.bar(range(len(d)), [v for v, _ in d], width=.85, color=[LEVEL_COLORS[l] for _, l in d], zorder=2)
        ax.axhline(0, color=INK2, linewidth=.8, zorder=3)
        higher, lower = sum(v > 1e-7 for v, _ in d), sum(v < -1e-7 for v, _ in d)
        ax.set_title(f'{a.split(" (")[0]} − {b}\nhigher on {higher}, lower on {lower} of {len(d)} tasks', fontsize=8.5, color=INK)
        ax.set_xticks([])
        ax.set_xlabel(f"{cfg['split']} tasks, sorted by difference", fontsize=8, color=INK2)
    axes[0].set_ylabel('difference in task success (points)', fontsize=8, color=INK2)
    handles = [plt.Rectangle((0, 0), 1, 1, color=LEVEL_COLORS[l]) for l in ('easy', 'medium', 'hard')]
    axes[-1].legend(handles, ['easy', 'medium', 'hard'], frameon=False, fontsize=7.5, loc='upper left', labelcolor=INK)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def calibration(results, path, label='CRISP (proposed)', split='test'):
    pred, obs, lev = [], [], []
    for f in sorted(Path(results).glob('runs/*/*.json')):
        r = json.loads(f.read_text())
        cert = (r.get('planner_info') or {}).get('certificate')
        if r['label'] != label or r['split'] != split or not cert or not r['execution']:
            continue
        pred.append(100 * cert['risk'])
        obs.append(100 * (1 - r['execution']['success_rate']))
        lev.append(r['level'])
    pred, obs = np.array(pred), np.array(obs)
    fig, ax = plt.subplots(figsize=(4.8, 4.4), dpi=170)
    fig.patch.set_facecolor(SURFACE)
    _style(ax)
    ax.yaxis.grid(True, color=GRID, linewidth=.8, zorder=0)
    top = max(pred.max(), obs.max()) * 1.08
    ax.plot([0, top], [0, top], color=INK2, linewidth=.8, linestyle='--', zorder=1)
    for l in ('easy', 'medium', 'hard'):
        m = np.array(lev) == l
        ax.scatter(pred[m], obs[m], s=16, color=LEVEL_COLORS[l], edgecolor=SURFACE, linewidth=.6, label=l, zorder=2)
    ax.set_xlim(0, top)
    ax.set_ylim(0, top)
    ax.set_xlabel('certificate estimate: risk on 1000 fresh planning draws (%)', fontsize=8, color=INK2)
    ax.set_ylabel('observed failure, 256 evaluation draws (%)', fontsize=8, color=INK2)
    r = np.corrcoef(pred, obs)[0, 1]
    ax.set_title(f'{label}: certificate vs outcome\nmean {pred.mean():.1f}% vs {obs.mean():.1f}%, Pearson r = {r:.2f}, {len(pred)} runs',
                 fontsize=8.5, color=INK)
    ax.legend(frameon=False, fontsize=7.5, labelcolor=INK, loc='upper left')
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)
    return dict(runs=int(len(pred)), predicted=float(pred.mean()), observed=float(obs.mean()), pearson=float(r))


def risk_bounds(suite, out, split='dev', draws=1000):
    """Trajectory-level risk vs its state-wise lower and Boole upper bounds on the straight keyframe insertions."""
    from .planners import crisp, guided
    from .planning import PlanningProblem
    from .risk import RiskModel
    suite = Path(suite)
    manifest = json.loads((suite / 'manifest.json').read_text())
    rows = []
    for e in manifest['tasks']:
        if e['split'] != split:
            continue
        scene = json.loads((suite / e['file']).read_text())
        problem = PlanningProblem(scene)
        try:
            ins = crisp.Insertion(guided.keyframes(scene, problem.goal_pose, .06, 0.), scene['payload']['size'][0])
            res = RiskModel(scene, problem.planning_errors(draws, 300_000)).evaluate(ins.poses(np.zeros(ins.dim)))
        finally:
            problem.close()
        per_state = (res['separations'][:, 1:] <= 0).mean(axis=1)
        rows.append(dict(task=e['id'], level=e['level'], trajectory=res['risk'], statewise=float(per_state.max()),
                         boole=float(per_state.sum()), states=len(per_state)))
    levels = ['easy', 'medium', 'hard']
    agg = {l: {k: float(np.mean([r[k] for r in rows if r['level'] == l])) for k in ('trajectory', 'statewise', 'boole')} for l in levels}
    fig, ax = plt.subplots(figsize=(6.4, 3.6), dpi=170)
    fig.patch.set_facecolor(SURFACE)
    _style(ax)
    ax.xaxis.grid(False)
    ax.yaxis.grid(True, color=GRID, linewidth=.8, zorder=0)
    series = [('statewise', 'state-wise maximum (lower bound)', '#86b6ef'), ('trajectory', 'trajectory-level risk', ACCENT),
              ('boole', 'Boole sum over states (upper bound)', '#104281')]
    for j, (key, name, colour) in enumerate(series):
        xs = np.arange(len(levels)) + (j - 1) * .27
        ys = [100 * agg[l][key] for l in levels]
        ax.bar(xs, ys, width=.25, color=colour, label=name, zorder=2)
        for x, y in zip(xs, ys):
            ax.text(x, y * 1.08, f'{y:.0f}%', ha='center', va='bottom', fontsize=7, color=INK2)
    ax.set_yscale('log')
    ax.set_ylim(1, 5000)
    ax.set_xticks(range(len(levels)))
    ax.set_xticklabels(levels, color=INK)
    ax.set_ylabel('failure probability (%, log scale)', fontsize=8, color=INK2)
    ax.legend(frameon=False, fontsize=7.5, labelcolor=INK, loc='upper left')
    ax.set_title(f'Straight insertions, {split} split ({len(rows)} tasks, {draws} planning draws)', fontsize=8.5, color=INK)
    fig.tight_layout()
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / 'risk_bounds.png', facecolor=SURFACE)
    plt.close(fig)
    (out / 'risk_bounds.json').write_text(json.dumps(dict(levels=agg, tasks=rows), indent=1) + '\n')
    return agg


def estimators(results, suite, out, labels=('CRISP (proposed)',), split='test', seed=0, draws=1000):
    """Risk estimates of the PLANNED paths against their observed failure: trajectory-level (CRISP's certificate
    estimator), state-wise maximum and Boole sum, on fresh planning draws (stream 400000, unused by any planner)."""
    from .evaluate import validate_path
    from .planning import PlanningProblem
    from .risk import RiskModel
    suite = Path(suite)
    manifest = {e['id']: e for e in json.loads((suite / 'manifest.json').read_text())['tasks']}
    rows = []
    for f in sorted(Path(results).glob('runs/*/*.json')):
        r = json.loads(f.read_text())
        if r['label'] not in labels or r['split'] != split or r['seed'] != seed or not r['execution']:
            continue
        scene = json.loads((suite / manifest[r['task_id']]['file']).read_text())
        problem = PlanningProblem(scene)
        try:
            _, _, poses, _ = validate_path(problem.world, scene, np.load(Path(results) / r['trajectory']), problem.joint_step, problem.checker)
            res = RiskModel(scene, problem.planning_errors(draws, 400_000)).evaluate(poses)
        finally:
            problem.close()
        per_state = (res['separations'][:, 1:] <= 0).mean(axis=1)
        rows.append(dict(label=r['label'], task=r['task_id'], level=r['level'], observed=1 - r['execution']['success_rate'],
                         trajectory=res['risk'], statewise=float(per_state.max()), boole=float(per_state.sum())))
    out = Path(out)
    summary = {}
    for lab in labels:
        rs = [x for x in rows if x['label'] == lab]
        obs = np.array([x['observed'] for x in rs])
        summary[lab] = {k: dict(mean=float(np.mean([x[k] for x in rs])),
                                mean_ratio_to_observed=float(np.mean([x[k] for x in rs]) / max(obs.mean(), 1e-9)),
                                runs_underestimating=int(sum(x[k] < x['observed'] for x in rs)))
                        for k in ('trajectory', 'statewise', 'boole')}
        summary[lab]['observed'] = float(obs.mean())
        summary[lab]['runs'] = len(rs)
    (out / 'risk_estimators.json').write_text(json.dumps(dict(summary=summary, runs=rows), indent=1) + '\n')
    lab = labels[0]
    rs = [x for x in rows if x['label'] == lab]
    fig, ax = plt.subplots(figsize=(5.2, 4.4), dpi=170)
    fig.patch.set_facecolor(SURFACE)
    _style(ax)
    ax.yaxis.grid(True, color=GRID, linewidth=.8, zorder=0)
    obs = 100 * np.array([x['observed'] for x in rs])
    for key, name, colour, marker in (('statewise', 'state-wise maximum', '#86b6ef', 'v'), ('trajectory', 'trajectory-level (certificate)', ACCENT, 'o')):
        est = 100 * np.array([x[key] for x in rs])
        ax.scatter(obs, est, s=16, color=colour, marker=marker, edgecolor=SURFACE, linewidth=.5, label=f'{name}: mean {est.mean():.1f}%', zorder=3)
    top = max(obs.max(), 100 * max(x['trajectory'] for x in rs)) * 1.1
    ax.plot([0, top], [0, top], '--', color=INK2, linewidth=.8, zorder=1)
    ax.set_xlim(0, top)
    ax.set_ylim(0, top)
    ax.set_xlabel(f'observed failure, 256 evaluation draws (%), mean {obs.mean():.1f}%', fontsize=8, color=INK2)
    ax.set_ylabel(f'estimated risk, {draws} fresh planning draws (%)', fontsize=8, color=INK2)
    boole = 100 * np.mean([x['boole'] for x in rs])
    ax.set_title(f'{lab}, {split} paths (seed {seed}): risk estimators\nBoole sum over states (not shown): mean {boole:.0f}%',
                 fontsize=8.5, color=INK)
    ax.legend(frameon=False, fontsize=7.5, labelcolor=INK, loc='upper left')
    fig.tight_layout()
    fig.savefig(out / 'risk_estimators.png', facecolor=SURFACE)
    plt.close(fig)
    return summary


def paths_figure(suite, task_ids, sources, out, filename='tip_paths.png'):
    """Payload-tip offset from the goal line vs depth for several planned paths on the same tasks.

    sources: [(name, colour, linestyle, {task_id: (npy path, success)})].  The obstacle outlines and the plotting
    plane use the generator's design block -- for VISUALISATION only; no planner reads it."""
    from matplotlib.patches import Rectangle
    from .figures import OBSTACLE_COLOR, _box_corners, _one_sided_plane, _style_axes
    from .world import SuiteWorld
    suite, out = Path(suite), Path(out)
    manifest = {e['id']: e for e in json.loads((suite / 'manifest.json').read_text())['tasks']}
    fig, axes = plt.subplots(1, len(task_ids), figsize=(3.4 * len(task_ids), 3.9), dpi=170, squeeze=False)
    fig.patch.set_facecolor(SURFACE)
    for ax, tid in zip(axes[0], task_ids):
        scene = json.loads((suite / manifest[tid]['file']).read_text())
        frame = np.asarray(scene['design']['insertion_frame'])
        goal = np.asarray(scene['task']['goal']['object_pose'])
        plane = _one_sided_plane(scene, frame, goal)
        L = scene['payload']['size'][0]
        x0 = min(scene['design']['control_depths'])
        _style_axes(ax)
        for o in scene['obstacles']:
            if o.get('role') != 'intrusion':
                continue
            corners = _box_corners(o)
            d = corners @ frame[:3, 0]
            lat = 1000 * ((corners - goal[:3, 3]) @ frame[:3, plane])
            if lat.min() < 0 < lat.max():
                continue
            lat = np.clip(lat, -60, 60)
            ax.add_patch(Rectangle((1000 * (d.min() - x0), lat.min()), 1000 * (d.max() - d.min()), lat.max() - lat.min(),
                                   facecolor=OBSTACLE_COLOR, edgecolor='none', zorder=1))
        world = SuiteWorld(scene)
        try:
            for name, colour, style, paths in sources:
                if tid not in paths:
                    continue
                path, success = paths[tid]
                tips = []
                for q in np.load(path)[::2]:
                    world.set_q(q)
                    pose = world.place_payload()
                    tips.append(pose[:3, :3] @ np.array([L / 2, 0, 0]) + pose[:3, 3])
                tips = np.array(tips)
                depth = tips @ frame[:3, 0]
                lat = 1000 * ((tips - goal[:3, 3]) @ frame[:3, plane])
                keep = depth > x0 - .15
                lat = np.where(np.abs(lat) > 60, np.nan, lat)
                ax.plot(1000 * (depth[keep] - x0), lat[keep], style, color=colour, linewidth=1.8,
                        label=f'{name} ({100 * success:.0f}%)', zorder=3)
        finally:
            world.close()
        ax.axhline(0, color=GRID, linewidth=1, zorder=0)
        ax.set_ylim(-60, 60)
        ax.set_title(f"{tid}\n(clearance {1000 * scene['design']['clearance_per_side_m']:.0f} mm/side)", fontsize=8.5, color=INK)
        ax.set_xlabel('tip depth past the first obstacle (mm)', fontsize=8, color=INK2)
        ax.legend(loc='lower left', frameon=False, fontsize=6.5, labelcolor=INK)
    axes[0][0].set_ylabel('tip offset from the goal line (mm)', fontsize=8, color=INK2)
    fig.suptitle('Planned insertions on hard test tasks: median-success seed of each method (task success in brackets)',
                 x=.02, ha='left', fontsize=10, color=INK, fontweight='bold')
    fig.subplots_adjust(left=.06, right=.99, top=.8, bottom=.15, wspace=.28)
    fig.savefig(out / filename, facecolor=SURFACE)
    plt.close(fig)


def run_paths(results, label, split='test'):
    """{task_id: (trajectory file, task success)} of one method: per task, the run whose success is the median
    over its seeds (a representative run, neither the best nor the worst)."""
    by = defaultdict(list)
    for f in Path(results).glob('runs/*/*.json'):
        r = json.loads(f.read_text())
        if r['label'] == label and r['split'] == split and r['nominal_valid']:
            by[r['task_id']].append((r['task_success_rate'], r['seed'], Path(results) / r['trajectory']))
    out = {}
    for task, runs in by.items():
        runs.sort()
        success, _, path = runs[(len(runs) - 1) // 2]
        out[task] = (path, success)
    return out


def anytime(out, split='test', seed=0):
    """Task success vs planning budget (CRISP, CC-VPSTO) and vs planning draws N (CRISP), seed 0 throughout.

    The 120 s / N = 128 points are the seed-0 runs of the main experiment (same code, same seed)."""
    def mean_success(results, label):
        vals = {}
        for f in Path(results).glob('runs/*/*.json'):
            r = json.loads(f.read_text())
            if r['label'] == label and r['split'] == split and r['seed'] == seed:
                vals[r['task_id']] = r['task_success_rate']
        levels = defaultdict(list)
        for t, v in vals.items():
            levels[t.split('_')[1]].append(v)
        times = [json.loads(f.read_text())['planning_seconds'] for f in Path(results).glob('runs/*/*.json')
                 if json.loads(f.read_text())['label'] == label and json.loads(f.read_text())['seed'] == seed]
        return dict(tasks=len(vals), all=float(np.mean(list(vals.values()))), **{l: float(np.mean(v)) for l, v in levels.items()},
                    ci95=bootstrap(vals), planning_seconds_median=float(np.median(times)))
    main, bud = 'results/suite_v0.4/crisp', 'results/suite_v0.4/crisp_budget'
    rows = {('CRISP', 30): (bud, 'CRISP, 30 s budget'), ('CRISP', 60): (bud, 'CRISP, 60 s budget'), ('CRISP', 120): (main, 'CRISP (proposed)'),
            ('CC-VPSTO', 30): (bud, 'CC-VPSTO, 30 s budget'), ('CC-VPSTO', 60): (bud, 'CC-VPSTO, 60 s budget'),
            ('CC-VPSTO', 120): (main, 'CC-VPSTO (adapted)')}
    budget = {f'{m}, {b} s': mean_success(*src) for (m, b), src in rows.items()}
    draws = {f'CRISP, N = {n}': mean_success(bud if n != 128 else main, f'CRISP, N = {n} draws' if n != 128 else 'CRISP (proposed)')
             for n in (32, 64, 128, 256)}
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.5), dpi=170)
    fig.patch.set_facecolor(SURFACE)
    for ax in axes:
        _style(ax)
        ax.xaxis.grid(False)
        ax.yaxis.grid(True, color=GRID, linewidth=.8, zorder=0)
    for m, colour in (('CRISP', ACCENT), ('CC-VPSTO', METHOD)):
        xs = [30, 60, 120]
        for lev, style in (('all', '-'), ('hard', '--')):
            ys = [100 * budget[f'{m}, {b} s'][lev] for b in xs]
            axes[0].plot(xs, ys, style, color=colour, marker='o', markersize=4, linewidth=1.8, label=f'{m} ({lev})', zorder=3)
    for lev, style in (('all', '-'), ('hard', '--')):
        axes[0].axhline(100 * {'all': .910, 'hard': .837}[lev], color=ORACLE, linestyle=style, linewidth=1, zorder=1)
    axes[0].text(31, 91.6, 'risk-aware oracle', fontsize=6.5, color=INK2)
    axes[0].set_xticks([30, 60, 120])
    axes[0].set_xlabel('planning budget (s)', fontsize=8, color=INK2)
    axes[0].set_ylabel('task success (%), seed 0', fontsize=8, color=INK2)
    axes[0].set_ylim(60, 100)
    axes[0].legend(frameon=False, fontsize=7, labelcolor=INK, loc='lower right', ncol=2)
    axes[0].set_title('Anytime behaviour (120 s = main protocol)', fontsize=8.5, color=INK)
    ns = [32, 64, 128, 256]
    for lev, style in (('all', '-'), ('hard', '--')):
        axes[1].plot(ns, [100 * draws[f'CRISP, N = {n}'][lev] for n in ns], style, color=ACCENT, marker='o', markersize=4, linewidth=1.8,
                     label=f'CRISP ({lev})', zorder=3)
    axes[1].set_xscale('log', base=2)
    axes[1].set_xticks(ns)
    axes[1].set_xticklabels([str(n) for n in ns])
    axes[1].set_xlabel('planning draws N (120 s budget)', fontsize=8, color=INK2)
    axes[1].set_ylim(60, 100)
    axes[1].legend(frameon=False, fontsize=7, labelcolor=INK, loc='lower right')
    axes[1].set_title('Sample size of the risk estimate (N = 128 = main protocol)', fontsize=8.5, color=INK)
    fig.tight_layout()
    out = Path(out)
    fig.savefig(out / 'anytime.png', facecolor=SURFACE)
    plt.close(fig)
    res = dict(budget=budget, draws=draws)
    (out / 'anytime.json').write_text(json.dumps(res, indent=1) + '\n')
    return res


def render_comparison(suite, out, tasks, methods, workers=4, cell=(900, 600)):
    """Simulator close-ups of the insertion (payload trail from the opening to the goal) for each method and task.

    methods: [(name, {task: (path file, success)})].  Uses gosuite.render (realistic materials)."""
    from multiprocessing import Pool
    from PIL import Image
    from .render import labelled_grid, render_task
    suite, out = Path(suite), Path(out)
    manifest = {e['id']: e for e in json.loads((suite / 'manifest.json').read_text())['tasks']}
    jobs, labels = [], []
    for task in tasks:
        for name, paths in methods:
            path, success = paths[task]
            jobs.append((str(suite / manifest[task]['file']), str(path), str(out / 'renders' / name.split()[0]), cell[0], cell[1]))
            labels.append(f"{name.split()[0]}, {task.split('_')[0]}: {100 * success:.0f}% success")
    todo = [j for j in jobs if not (Path(j[2]) / f'{Path(j[0]).stem}_goal.png').exists()]
    if todo:
        with Pool(workers) as pool:
            pool.map(render_task, todo)
    images = [Image.open(Path(j[2]) / f'{Path(j[0]).stem}_goal.png') for j in jobs]
    board = labelled_grid(images, labels, len(methods), cell)
    board.save(out / 'render_comparison.png')
    return out / 'render_comparison.png'


def main():
    ap = argparse.ArgumentParser(description='Figures for the CRISP results.')
    ap.add_argument('--risk-bounds', metavar='SUITE', help='only the Proposition 1 figure, from this suite')
    ap.add_argument('--estimators', metavar='SUITE', help='also the risk-estimator comparison on the planned paths')
    ap.add_argument('--paths', metavar='SUITE', help='only the tip-path figure (hard test task 00 of each family)')
    ap.add_argument('--anytime', action='store_true', help='only the budget / sample-size figure (seed 0)')
    ap.add_argument('--render', metavar='SUITE', help='only the simulator close-ups, B6 vs CRISP (median seeds, hard test task 00)')
    ap.add_argument('--main')
    ap.add_argument('--ablation')
    ap.add_argument('--output', required=True)
    a = ap.parse_args()
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=True)
    if a.risk_bounds:
        print(json.dumps(risk_bounds(a.risk_bounds, out)))
        return
    if a.render:
        methods = [('B6 keyframe-guided', run_paths('results/suite_v0.4/baselines', 'B6 Keyframe-guided')),
                   ('CRISP', run_paths('results/suite_v0.4/crisp', 'CRISP (proposed)'))]
        print(render_comparison(a.render, out, [f'{f}_hard_test_00' for f in ('rack', 'rebar', 'lathe', 'tote')], methods))
        return
    if a.anytime:
        print(json.dumps(anytime(out), indent=1))
        return
    if a.paths:
        ref = {}
        for f in Path('results/suite_v0.4/reference').glob('runs/*/*.json'):
            r = json.loads(f.read_text())
            ref.setdefault(r['label'], {})[r['task_id']] = (Path(r['trajectory']), r['task_success_rate'])
        sources = [('B6 keyframe-guided', '#eb6834', '--', run_paths('results/suite_v0.4/baselines', 'B6 Keyframe-guided')),
                   ('risk-aware oracle', ORACLE, '-', ref['Reference (risk-aware oracle)']),
                   ('CC-VPSTO (adapted)', METHOD, '-', run_paths('results/suite_v0.4/crisp', 'CC-VPSTO (adapted)')),
                   ('CRISP', ACCENT, '-', run_paths('results/suite_v0.4/crisp', 'CRISP (proposed)'))]
        paths_figure(a.paths, [f'{f}_hard_test_00' for f in ('rack', 'rebar', 'lathe', 'tote')], sources, out)
        return
    cfg = json.loads(Path(a.main).read_text())
    dot_plot(cfg, out / 'methods_by_level.png', f"Task success on the {cfg['split']} split")
    paired_differences(cfg, out / 'paired_differences.png')
    crisp_dir = next(d for d in cfg['results'] if 'crisp' in Path(d).name)
    stats = calibration(crisp_dir, out / 'certificate_calibration.png', split=cfg['split'])
    if a.ablation:
        dot_plot(json.loads(Path(a.ablation).read_text()), out / 'ablations.png', 'CRISP ablations (one component changed each)')
    if a.estimators:
        stats['estimators'] = estimators(crisp_dir, a.estimators, out, split=cfg['split'],
                                         labels=('CRISP (proposed)', 'Ablation: state-wise risk', 'Ablation: linearised Boole risk'))
    print(json.dumps(stats))


if __name__ == '__main__':
    main()
