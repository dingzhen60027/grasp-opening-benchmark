"""LaTeX (booktabs) tables for the paper, generated from the committed comparison and study outputs.

    python -m gosuite.paper_tables --output paper/tables

Reads results/suite_v0.4/comparison_crisp_test/comparison.json (main + paired), comparison_crisp_ablation_test,
robustness_crisp_test/robustness.json, and the risk_estimators / anytime JSON written by gosuite.crisp_figures.
Numbers are never typed by hand.
"""
import argparse
import json
from pathlib import Path

ROOT = Path('results/suite_v0.4')


def pct(x, digits=1):
    return f'{100 * x:.{digits}f}'


def ci(c):
    return f'[{100 * c[0]:.1f}, {100 * c[1]:.1f}]'


def pvalue(p):
    if p < 1e-4:
        mant, exp = f'{p:.1e}'.split('e')
        return f'${mant}\\times10^{{{int(exp)}}}$'
    return f'{p:.3f}' if p >= .001 else f'{p:.4f}'


def main_table(res):
    rows = [r'\begin{tabular}{lrrrrrr}', r'\toprule',
            r'Method & Valid path & Easy & Medium & Hard & All [95\% CI] & Collision $|$ valid \\', r'\midrule']
    for name, m in res['methods'].items():
        a = m['all']
        best = name.startswith('CRISP')
        cell = (lambda s: f'\\textbf{{{s}}}') if best else (lambda s: s)
        rows.append(f"{cell(name)} & {100 * a['valid_path']:.0f}\\% & " + ' & '.join(cell(pct(m[l]['task_success'])) for l in ('easy', 'medium', 'hard'))
                    + f" & {cell(pct(a['task_success']))} {ci(a['task_success_ci95'])} & {pct(a['collision_given_valid'])} \\\\")
    rows += [r'\bottomrule', r'\end{tabular}']
    return '\n'.join(rows) + '\n'


def paired_table(res):
    rows = [r'\begin{tabular}{lrrrrrr}', r'\toprule',
            r'Comparison (A $-$ B) & Easy & Medium & Hard & All [95\% CI] & A higher / lower & $p$ (Holm) \\', r'\midrule']
    for p in res['pairs']:
        lv = p['levels']
        a = lv['all']
        rows.append(f"{p['a']} $-$ {p['b']} & " + ' & '.join(f"{100 * lv[l]['mean']:+.1f}" for l in ('easy', 'medium', 'hard'))
                    + f" & {100 * a['mean']:+.1f} [{100 * a['ci95'][0]:+.1f}, {100 * a['ci95'][1]:+.1f}] & {a['a_higher']} / {a['b_higher']} & {pvalue(a['p_holm'])} \\\\")
    rows += [r'\bottomrule', r'\end{tabular}']
    return '\n'.join(rows) + '\n'


def robustness_table(rob):
    conds = list(next(iter(rob['table'].values())))
    rows = [r'\begin{tabular}{l' + 'r' * len(conds) + '}', r'\toprule',
            'Method & ' + ' & '.join(c.replace('combined, ', '').replace('s = ', '$s=') + ('$' if 's = ' in c else '') for c in conds) + r' \\',
            r'\midrule']
    for name, row in rob['table'].items():
        rows.append(f'{name} & ' + ' & '.join(pct(row[c]['mean']) for c in conds) + r' \\')
    rows += [r'\bottomrule', r'\end{tabular}']
    return '\n'.join(rows) + '\n'


def estimators_table(est):
    rows = [r'\begin{tabular}{lrrrr}', r'\toprule',
            r'Paths (seed 0) & Observed & Trajectory-level & State-wise max. & Boole sum \\', r'\midrule']
    for lab, v in est['summary'].items():
        cell = lambda k: f"{100 * v[k]['mean']:.1f} ({v[k]['mean_ratio_to_observed']:.2f}$\\times$)"
        rows.append(f"{lab.replace('Ablation: ', '')} & {100 * v['observed']:.1f} & {cell('trajectory')} & {cell('statewise')} & "
                    f"{100 * v['boole']['mean']:.0f} ({v['boole']['mean_ratio_to_observed']:.0f}$\\times$) \\\\")
    rows += [r'\bottomrule', r'\end{tabular}']
    return '\n'.join(rows) + '\n'


def anytime_table(res):
    rows = [r'\begin{tabular}{lrrrrr}', r'\toprule',
            r'Variant (seed 0) & Easy & Medium & Hard & All [95\% CI] & Time (s) \\', r'\midrule']
    for part in ('budget', 'draws'):
        for k, v in res[part].items():
            if k == 'CRISP, N = 128':
                continue
            rows.append(f"{k} & {pct(v['easy'])} & {pct(v['medium'])} & {pct(v['hard'])} & {pct(v['all'])} {ci(v['ci95'])} & "
                        f"{v['planning_seconds_median']:.0f} \\\\")
        rows.append(r'\midrule')
    rows[-1] = r'\bottomrule'
    rows.append(r'\end{tabular}')
    return '\n'.join(rows) + '\n'


def main():
    ap = argparse.ArgumentParser(description='Write the paper tables (LaTeX).')
    ap.add_argument('--output', default='paper/tables')
    a = ap.parse_args()
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=True)
    written = []
    main_res = json.loads((ROOT / 'comparison_crisp_test' / 'comparison.json').read_text())
    (out / 'main_results.tex').write_text(main_table(main_res))
    (out / 'paired_differences.tex').write_text(paired_table(main_res))
    written += ['main_results.tex', 'paired_differences.tex']
    abl = ROOT / 'comparison_crisp_ablation_test' / 'comparison.json'
    if abl.exists():
        res = json.loads(abl.read_text())
        (out / 'ablations.tex').write_text(main_table(res))
        (out / 'ablation_paired.tex').write_text(paired_table(res))
        written += ['ablations.tex', 'ablation_paired.tex']
    rob = ROOT / 'robustness_crisp_test' / 'robustness.json'
    if rob.exists():
        (out / 'robustness.tex').write_text(robustness_table(json.loads(rob.read_text())))
        written.append('robustness.tex')
    figs = ROOT / 'comparison_crisp_test'
    if (figs / 'risk_estimators.json').exists():
        (out / 'risk_estimators.tex').write_text(estimators_table(json.loads((figs / 'risk_estimators.json').read_text())))
        written.append('risk_estimators.tex')
    if (figs / 'anytime.json').exists():
        (out / 'anytime.tex').write_text(anytime_table(json.loads((figs / 'anytime.json').read_text())))
        written.append('anytime.tex')
    print('wrote', ', '.join(written), 'to', out)


if __name__ == '__main__':
    main()
