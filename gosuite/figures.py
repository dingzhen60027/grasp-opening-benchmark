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


def main():
    ap = argparse.ArgumentParser(description='Plot an experiment summary.')
    ap.add_argument('output')
    ap.add_argument('--label')
    a = ap.parse_args()
    print(success_by_family_level(a.output, a.label))


if __name__ == '__main__':
    main()
