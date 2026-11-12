"""Static figures for experiment summaries (matplotlib, light surface).

    python -m gosuite.figures results/suite_v0.3/reference

Writes ``task_success_by_family_level.png`` next to the summary: grouped bars (family x level),
ordinal one-hue ramp for the clearance levels (validated light->dark ramp), 95% bootstrap CIs.
"""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

SURFACE, INK, INK2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
LEVEL_COLORS = {'easy': '#86b6ef', 'medium': '#2a78d6', 'hard': '#104281'}  # ordinal blue ramp, steps 250/450/650
LEVEL_NAMES = {'easy': 'easy (20 mm/side)', 'medium': 'medium (10 mm/side)', 'hard': 'hard (5 mm/side)'}
FAMILY_NAMES = {'rack': 'F1 Rack bay', 'rebar': 'F2 Rebar cage', 'lathe': 'F3 Lathe chuck', 'tote': 'F4 Kitting tote'}


def success_by_family_level(out, label=None, metric='task_success', filename='task_success_by_family_level.png'):
    out = Path(out)
    summary = json.loads((out / 'summary.json').read_text())
    label = label or summary[0]['label']
    rows = {(s['family'], s['level']): s for s in summary if s['label'] == label}
    families = [f for f in FAMILY_NAMES if (f, 'all') in rows]
    levels = [l for l in LEVEL_COLORS if ('all', l) in rows]
    fig, ax = plt.subplots(figsize=(8.6, 4.2), dpi=170)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    width = .26
    for j, lev in enumerate(levels):
        xs, ys, lo, hi = [], [], [], []
        for i, fam in enumerate(families):
            s = rows[(fam, lev)]
            xs.append(i + (j - (len(levels) - 1) / 2) * width)
            ys.append(100 * s[metric])
            ci = s.get(metric + '_ci95') or [s[metric]] * 2
            lo.append(100 * (s[metric] - ci[0]))
            hi.append(100 * (ci[1] - s[metric]))
        ax.bar(xs, ys, width=width * .92, color=LEVEL_COLORS[lev], edgecolor=SURFACE, linewidth=1.2, label=LEVEL_NAMES[lev], zorder=2)
        ax.errorbar(xs, ys, yerr=[lo, hi], fmt='none', ecolor=INK2, elinewidth=.9, capsize=2.5, capthick=.9, zorder=3)
        for x, y, h in zip(xs, ys, hi):
            ax.text(x, y + h + 1.5, f'{y:.0f}', ha='center', va='bottom', fontsize=7.5, color=INK2)
    ax.set_xticks(range(len(families)))
    ax.set_xticklabels([FAMILY_NAMES[f] for f in families], fontsize=9.5, color=INK)
    ax.set_ylim(0, 110)
    ax.set_yticks(range(0, 101, 20))
    ax.set_yticklabels([f'{v}%' for v in range(0, 101, 20)], fontsize=8.5, color=INK2)
    ax.set_ylabel('Task success under grasp error', fontsize=9, color=INK2)
    ax.yaxis.grid(True, color=GRID, linewidth=.8, zorder=0)
    ax.set_axisbelow(True)
    for side in ('top', 'right', 'left'):
        ax.spines[side].set_visible(False)
    ax.spines['bottom'].set_color(GRID)
    ax.tick_params(length=0)
    ax.legend(loc='upper left', bbox_to_anchor=(0, 1.13), ncol=len(levels), frameon=False, fontsize=8.5, labelcolor=INK,
              handlelength=1.2, columnspacing=1.4)
    n_tasks = rows[(families[0], levels[0])]['tasks']
    fig.suptitle(f'{label}: success by family and clearance level', x=.075, ha='left', y=.985, fontsize=11.5,
                 color=INK, fontweight='bold')
    fig.text(.075, .905, f'{n_tasks} tasks per cell; error bars are 95% bootstrap intervals over tasks; '
             'grasp error 2 mm / 1 deg per axis (uncalibrated)', fontsize=8, color=INK2)
    fig.subplots_adjust(left=.075, right=.985, top=.80, bottom=.1)
    fig.savefig(out / filename, facecolor=SURFACE)
    plt.close(fig)
    return out / filename


STRAIGHT_COLOR, ROBUST_COLOR, OBSTACLE_COLOR = '#eb6834', '#2a78d6', '#c9c7c0'


def witness_comparison(suite, task_ids, out, filename='witness_comparison.png'):
    """Payload-tip offset from the goal line vs insertion depth, straight vs risk-aware witness (v0.4)."""
    import numpy as np
    from matplotlib.patches import Rectangle
    from .world import SuiteWorld
    suite, out = Path(suite), Path(out)
    manifest = {e['id']: e for e in json.loads((suite / 'manifest.json').read_text())['tasks']}
    fig, axes = plt.subplots(1, len(task_ids), figsize=(3.3 * len(task_ids), 3.5), dpi=170, squeeze=False)
    fig.patch.set_facecolor(SURFACE)
    for ax, tid in zip(axes[0], task_ids):
        entry = manifest[tid]
        scene = json.loads((suite / entry['file']).read_text())
        frame = np.asarray(scene['design']['insertion_frame'])
        goal = np.asarray(scene['task']['goal']['object_pose'])
        plane = _one_sided_plane(scene, frame, goal)
        L = scene['payload']['size'][0]
        world = SuiteWorld(scene)
        try:
            curves = {}
            for kind, key in (('straight', 'witness_nominal_file'), ('risk-aware', 'witness_file')):
                tips = []
                for q in np.load(suite / entry[key])[::3]:
                    world.set_q(q)
                    pose = world.place_payload()
                    tips.append(pose[:3, :3] @ np.array([L / 2, 0, 0]) + pose[:3, 3])
                tips = np.array(tips)
                depth = tips @ frame[:3, 0]
                lateral = (tips - goal[:3, 3]) @ frame[:3, plane]
                keep = depth > min(scene['design']['control_depths']) - .15
                lat_mm = 1000 * lateral[keep]
                lat_mm[np.abs(lat_mm) > 60] = np.nan   # the descent / approach before the insertion
                curves[kind] = (depth[keep], lat_mm)
        finally:
            world.close()
        x0 = min(scene['design']['control_depths'])
        _style_axes(ax)
        for o in scene['obstacles']:
            if o.get('role') != 'intrusion':
                continue
            corners = _box_corners(o)
            d = corners @ frame[:3, 0]
            lat = 1000 * ((corners - goal[:3, 3]) @ frame[:3, plane])
            if lat.min() < 0 < lat.max():   # constrains the other plane: not drawn here
                continue
            lat = np.clip(lat, -60, 60)
            ax.add_patch(Rectangle((1000 * (d.min() - x0), lat.min()), 1000 * (d.max() - d.min()), lat.max() - lat.min(),
                                   facecolor=OBSTACLE_COLOR, edgecolor='none', zorder=1))
        for kind, color in (('straight', STRAIGHT_COLOR), ('risk-aware', ROBUST_COLOR)):
            d, lat = curves[kind]
            ax.plot(1000 * (d - x0), lat, color=color, linewidth=2, label=f'{kind} witness', zorder=3)
        ax.axhline(0, color=GRID, linewidth=1, zorder=0)
        ax.set_ylim(-60, 60)
        ax.set_title(f"{tid}\n(clearance {1000 * scene['design']['clearance_per_side_m']:.0f} mm/side)", fontsize=8.5, color=INK)
        ax.set_xlabel('tip depth past the first obstacle (mm)', fontsize=8, color=INK2)
    axes[0][0].set_ylabel('tip offset from the goal line (mm)', fontsize=8, color=INK2)
    axes[0][0].legend(loc='lower left', frameon=False, fontsize=7.5, labelcolor=INK)
    fig.suptitle('Straight vs risk-aware insertion: the tip steers away from obstacles it passes with a long lever arm',
                 x=.02, ha='left', fontsize=10.5, color=INK, fontweight='bold')
    fig.subplots_adjust(left=.07, right=.99, top=.78, bottom=.17, wspace=.28)
    fig.savefig(out / filename, facecolor=SURFACE)
    plt.close(fig)
    return out / filename


def _box_corners(item):
    import numpy as np
    pose = np.asarray(item['pose'])
    pts = []
    for prt in item.get('parts') or [dict(size=item['size'], center=[0, 0, 0])]:
        half = np.asarray(prt['size']) / 2
        local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half + prt.get('center', [0, 0, 0])
        pts.append(local @ pose[:3, :3].T + pose[:3, 3])
    return np.vstack(pts)


def _one_sided_plane(scene, frame, goal):
    """Deviation plane ('y' -> 1, 'z' -> 2) in which most intrusions lie entirely on one side of the goal line."""
    import numpy as np
    best, score = 1, -1
    for name in scene['design']['deviation_planes']:
        axis = {'y': 1, 'z': 2}[name]
        n = 0
        for o in scene['obstacles']:
            if o.get('role') == 'intrusion':
                lat = (_box_corners(o) - goal[:3, 3]) @ frame[:3, axis]
                n += not (lat.min() < 0 < lat.max())
        if n > score:
            best, score = axis, n
    return best


def _style_axes(ax):
    ax.set_facecolor(SURFACE)
    ax.yaxis.grid(True, color=GRID, linewidth=.6, zorder=0)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(length=0, labelsize=7.5, colors=INK2)


def main():
    ap = argparse.ArgumentParser(description='Plot an experiment summary.')
    ap.add_argument('output')
    ap.add_argument('--label')
    a = ap.parse_args()
    print(success_by_family_level(a.output, a.label))


if __name__ == '__main__':
    main()
