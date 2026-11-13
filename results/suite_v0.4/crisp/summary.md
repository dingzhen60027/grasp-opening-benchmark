# Experiment summary

Runs: 2016. Task success counts planning failures as 0; CI = 95% bootstrap over tasks. Collision | valid = mean episode collision rate of valid paths. Feasible-episode success: goal-state flags from `../validation/goal_feasibility.json`.

| Planner | Family | Level | Runs | Planning success | Task success [95% CI] | Feasible-episode success | Collision \| valid | Plan time (s, median) | Min clearance (mm, median) |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| Ablation: CVaR objective | all | all | 216 | 100% | 93.2% [91.9, 94.3] | 96.7% | 4.3% | 92.79 | 8.4 |
| Ablation: CVaR objective | all | easy | 72 | 100% | 94.8% [92.8, 96.5] | 97.0% | 2.1% | 104.02 | 11.7 |
| Ablation: CVaR objective | all | medium | 72 | 100% | 94.5% [92.9, 95.9] | 97.6% | 3.3% | 90.79 | 9.1 |
| Ablation: CVaR objective | all | hard | 72 | 100% | 90.1% [87.7, 92.1] | 95.6% | 7.6% | 92.51 | 6.2 |
| Ablation: CVaR objective | lathe | all | 54 | 100% | 91.1% [87.6, 94.0] | 94.7% | 7.0% | 68.76 | 9.3 |
| Ablation: CVaR objective | lathe | easy | 18 | 100% | 92.6% [87.8, 95.3] | 95.0% | 4.1% | 47.33 | 12.4 |
| Ablation: CVaR objective | lathe | medium | 18 | 100% | 95.0% [93.1, 96.5] | 97.5% | 3.5% | 69.00 | 9.4 |
| Ablation: CVaR objective | lathe | hard | 18 | 100% | 85.7% [79.4, 91.5] | 91.6% | 13.2% | 88.32 | 6.0 |
| Ablation: CVaR objective | rack | all | 54 | 100% | 89.8% [87.9, 91.6] | 95.2% | 6.0% | 98.38 | 8.2 |
| Ablation: CVaR objective | rack | easy | 18 | 100% | 90.6% [87.9, 93.5] | 94.6% | 3.5% | 111.77 | 11.9 |
| Ablation: CVaR objective | rack | medium | 18 | 100% | 89.6% [85.8, 92.2] | 95.6% | 6.4% | 99.92 | 8.3 |
| Ablation: CVaR objective | rack | hard | 18 | 100% | 89.3% [85.5, 92.4] | 95.5% | 8.1% | 91.76 | 6.1 |
| Ablation: CVaR objective | rebar | all | 54 | 100% | 94.5% [93.3, 95.7] | 98.1% | 2.0% | 78.35 | 9.1 |
| Ablation: CVaR objective | rebar | easy | 18 | 100% | 96.4% [95.1, 97.7] | 98.6% | 0.6% | 56.91 | 14.8 |
| Ablation: CVaR objective | rebar | medium | 18 | 100% | 95.2% [94.5, 96.0] | 98.5% | 1.9% | 67.91 | 9.4 |
| Ablation: CVaR objective | rebar | hard | 18 | 100% | 92.0% [90.2, 94.0] | 97.2% | 3.5% | 87.69 | 7.5 |
| Ablation: CVaR objective | tote | all | 54 | 100% | 97.1% [95.8, 98.3] | 98.8% | 2.4% | 111.02 | 7.5 |
| Ablation: CVaR objective | tote | easy | 18 | 100% | 99.5% [99.3, 99.8] | 99.7% | 0.3% | 110.63 | 8.4 |
| Ablation: CVaR objective | tote | medium | 18 | 100% | 98.3% [98.0, 98.5] | 99.0% | 1.5% | 111.02 | 8.1 |
| Ablation: CVaR objective | tote | hard | 18 | 100% | 93.6% [92.4, 95.1] | 97.9% | 5.6% | 111.44 | 6.0 |
| Ablation: fixed goal pose | all | all | 216 | 100% | 90.4% [88.8, 91.9] | 97.3% | 8.0% | 93.45 | 8.8 |
| Ablation: fixed goal pose | all | easy | 72 | 100% | 94.8% [93.2, 96.3] | 98.2% | 3.8% | 81.65 | 14.0 |
| Ablation: fixed goal pose | all | medium | 72 | 100% | 92.9% [90.6, 94.8] | 97.4% | 4.9% | 91.54 | 10.0 |
| Ablation: fixed goal pose | all | hard | 72 | 100% | 83.5% [81.5, 85.3] | 96.4% | 15.4% | 102.24 | 6.0 |
| Ablation: fixed goal pose | lathe | all | 54 | 100% | 90.8% [88.1, 93.2] | 97.8% | 8.3% | 71.89 | 10.0 |
| Ablation: fixed goal pose | lathe | easy | 18 | 100% | 93.2% [91.3, 94.8] | 97.7% | 5.8% | 38.00 | 14.0 |
| Ablation: fixed goal pose | lathe | medium | 18 | 100% | 95.5% [94.6, 96.5] | 99.7% | 3.2% | 77.51 | 10.0 |
| Ablation: fixed goal pose | lathe | hard | 18 | 100% | 83.7% [80.4, 86.2] | 96.0% | 16.0% | 87.65 | 6.0 |
| Ablation: fixed goal pose | rack | all | 54 | 100% | 84.8% [81.5, 87.7] | 94.6% | 12.0% | 99.72 | 10.0 |
| Ablation: fixed goal pose | rack | easy | 18 | 100% | 90.1% [87.9, 92.3] | 96.1% | 8.1% | 112.08 | 14.0 |
| Ablation: fixed goal pose | rack | medium | 18 | 100% | 85.8% [81.0, 89.3] | 94.0% | 9.1% | 94.13 | 10.0 |
| Ablation: fixed goal pose | rack | hard | 18 | 100% | 78.5% [73.9, 82.0] | 93.7% | 18.7% | 95.71 | 6.0 |
| Ablation: fixed goal pose | rebar | all | 54 | 100% | 91.9% [89.3, 94.3] | 98.3% | 5.9% | 58.71 | 10.0 |
| Ablation: fixed goal pose | rebar | easy | 18 | 100% | 96.5% [95.7, 97.8] | 99.3% | 0.8% | 39.84 | 16.0 |
| Ablation: fixed goal pose | rebar | medium | 18 | 100% | 94.3% [91.3, 96.2] | 98.3% | 3.3% | 55.46 | 10.1 |
| Ablation: fixed goal pose | rebar | hard | 18 | 100% | 85.0% [82.6, 87.4] | 97.5% | 13.6% | 81.60 | 6.1 |
| Ablation: fixed goal pose | tote | all | 54 | 100% | 94.1% [91.4, 96.5] | 98.6% | 5.8% | 110.95 | 8.0 |
| Ablation: fixed goal pose | tote | easy | 18 | 100% | 99.3% [98.9, 99.7] | 99.5% | 0.5% | 110.18 | 8.3 |
| Ablation: fixed goal pose | tote | medium | 18 | 100% | 96.0% [94.4, 97.6] | 97.7% | 3.9% | 111.51 | 8.2 |
| Ablation: fixed goal pose | tote | hard | 18 | 100% | 86.8% [85.2, 88.4] | 98.5% | 13.1% | 110.83 | 6.0 |
| Ablation: linearised Boole risk | all | all | 216 | 100% | 92.1% [90.6, 93.4] | 95.6% | 4.3% | 108.05 | 8.9 |
| Ablation: linearised Boole risk | all | easy | 72 | 100% | 95.3% [93.8, 96.8] | 97.4% | 1.5% | 109.69 | 13.0 |
| Ablation: linearised Boole risk | all | medium | 72 | 100% | 92.9% [90.7, 95.0] | 95.6% | 3.6% | 94.03 | 9.1 |
| Ablation: linearised Boole risk | all | hard | 72 | 100% | 88.1% [85.1, 90.5] | 93.8% | 7.8% | 103.64 | 6.4 |
| Ablation: linearised Boole risk | lathe | all | 54 | 100% | 91.0% [88.4, 93.3] | 94.1% | 5.0% | 87.72 | 9.0 |
| Ablation: linearised Boole risk | lathe | easy | 18 | 100% | 94.1% [90.8, 96.5] | 96.0% | 2.0% | 76.05 | 15.2 |
| Ablation: linearised Boole risk | lathe | medium | 18 | 100% | 93.0% [89.3, 96.1] | 95.1% | 3.7% | 87.58 | 9.1 |
| Ablation: linearised Boole risk | lathe | hard | 18 | 100% | 85.8% [82.5, 89.2] | 91.2% | 9.3% | 90.04 | 5.8 |
| Ablation: linearised Boole risk | rack | all | 54 | 100% | 87.8% [84.4, 90.5] | 93.7% | 6.9% | 98.42 | 8.2 |
| Ablation: linearised Boole risk | rack | easy | 18 | 100% | 92.8% [90.9, 94.8] | 96.7% | 2.2% | 112.87 | 12.1 |
| Ablation: linearised Boole risk | rack | medium | 18 | 100% | 87.4% [84.0, 90.0] | 92.9% | 6.8% | 97.70 | 8.2 |
| Ablation: linearised Boole risk | rack | hard | 18 | 100% | 83.4% [76.0, 89.3] | 91.7% | 11.7% | 92.94 | 6.1 |
| Ablation: linearised Boole risk | rebar | all | 54 | 100% | 92.3% [89.7, 94.4] | 95.6% | 3.6% | 85.92 | 10.2 |
| Ablation: linearised Boole risk | rebar | easy | 18 | 100% | 94.8% [92.0, 97.2] | 97.1% | 1.9% | 78.99 | 15.7 |
| Ablation: linearised Boole risk | rebar | medium | 18 | 100% | 92.7% [88.2, 95.8] | 95.7% | 3.5% | 85.80 | 10.3 |
| Ablation: linearised Boole risk | rebar | hard | 18 | 100% | 89.3% [84.0, 92.6] | 94.0% | 5.4% | 88.36 | 8.0 |
| Ablation: linearised Boole risk | tote | all | 54 | 100% | 97.4% [96.1, 98.6] | 99.0% | 1.8% | 110.76 | 8.0 |
| Ablation: linearised Boole risk | tote | easy | 18 | 100% | 99.7% [99.5, 99.9] | 99.8% | 0.0% | 110.22 | 9.1 |
| Ablation: linearised Boole risk | tote | medium | 18 | 100% | 98.6% [98.1, 99.1] | 98.9% | 0.6% | 111.25 | 8.4 |
| Ablation: linearised Boole risk | tote | hard | 18 | 100% | 93.8% [92.7, 95.3] | 98.1% | 4.8% | 111.29 | 6.5 |
| Ablation: max-clearance objective | all | all | 216 | 100% | 89.4% [87.7, 90.9] | 94.2% | 8.8% | 97.47 | 10.0 |
| Ablation: max-clearance objective | all | easy | 72 | 100% | 93.0% [90.9, 95.0] | 96.3% | 6.0% | 89.63 | 14.0 |
| Ablation: max-clearance objective | all | medium | 72 | 100% | 91.1% [89.7, 92.4] | 94.9% | 7.2% | 105.59 | 11.0 |
| Ablation: max-clearance objective | all | hard | 72 | 100% | 84.1% [81.0, 87.0] | 91.4% | 13.3% | 103.14 | 8.0 |
| Ablation: max-clearance objective | lathe | all | 54 | 100% | 89.7% [86.8, 92.3] | 94.6% | 7.7% | 68.01 | 10.9 |
| Ablation: max-clearance objective | lathe | easy | 18 | 100% | 93.0% [91.2, 94.6] | 97.6% | 6.1% | 45.31 | 14.0 |
| Ablation: max-clearance objective | lathe | medium | 18 | 100% | 93.7% [91.9, 95.3] | 96.3% | 4.2% | 76.34 | 11.1 |
| Ablation: max-clearance objective | lathe | hard | 18 | 100% | 82.5% [79.4, 85.6] | 90.0% | 12.7% | 86.69 | 7.9 |
| Ablation: max-clearance objective | rack | all | 54 | 100% | 84.8% [81.4, 87.8] | 92.5% | 12.5% | 97.18 | 11.0 |
| Ablation: max-clearance objective | rack | easy | 18 | 100% | 89.6% [87.6, 91.9] | 95.6% | 8.7% | 111.88 | 14.0 |
| Ablation: max-clearance objective | rack | medium | 18 | 100% | 87.1% [85.7, 88.8] | 94.8% | 9.7% | 96.21 | 11.0 |
| Ablation: max-clearance objective | rack | hard | 18 | 100% | 77.8% [71.4, 83.9] | 87.3% | 19.2% | 94.19 | 8.3 |
| Ablation: max-clearance objective | rebar | all | 54 | 100% | 89.5% [86.5, 92.0] | 93.4% | 8.8% | 84.90 | 11.0 |
| Ablation: max-clearance objective | rebar | easy | 18 | 100% | 90.1% [84.8, 93.8] | 92.7% | 8.5% | 53.69 | 16.0 |
| Ablation: max-clearance objective | rebar | medium | 18 | 100% | 93.3% [92.2, 94.5] | 96.6% | 5.4% | 84.46 | 11.2 |
| Ablation: max-clearance objective | rebar | hard | 18 | 100% | 85.2% [79.3, 89.4] | 90.9% | 12.6% | 110.94 | 9.4 |
| Ablation: max-clearance objective | tote | all | 54 | 100% | 93.6% [91.6, 95.7] | 96.3% | 6.3% | 110.72 | 10.0 |
| Ablation: max-clearance objective | tote | easy | 18 | 100% | 99.2% [98.9, 99.5] | 99.4% | 0.6% | 110.10 | 10.0 |
| Ablation: max-clearance objective | tote | medium | 18 | 100% | 90.4% [88.0, 92.6] | 91.9% | 9.6% | 110.89 | 10.0 |
| Ablation: max-clearance objective | tote | hard | 18 | 100% | 91.1% [89.3, 92.8] | 97.5% | 8.6% | 111.55 | 7.6 |
| Ablation: state-wise risk | all | all | 216 | 100% | 92.3% [90.8, 93.6] | 96.3% | 4.9% | 108.90 | 8.6 |
| Ablation: state-wise risk | all | easy | 72 | 100% | 95.4% [94.0, 96.8] | 97.9% | 1.9% | 109.78 | 12.9 |
| Ablation: state-wise risk | all | medium | 72 | 100% | 92.9% [90.0, 95.2] | 96.1% | 4.0% | 97.54 | 9.0 |
| Ablation: state-wise risk | all | hard | 72 | 100% | 88.6% [86.3, 90.6] | 94.9% | 8.8% | 109.07 | 6.5 |
| Ablation: state-wise risk | lathe | all | 54 | 100% | 93.1% [91.2, 94.6] | 97.3% | 5.0% | 80.57 | 9.2 |
| Ablation: state-wise risk | lathe | easy | 18 | 100% | 95.0% [94.3, 95.7] | 97.9% | 1.8% | 51.60 | 13.8 |
| Ablation: state-wise risk | lathe | medium | 18 | 100% | 95.4% [94.6, 96.3] | 98.3% | 3.1% | 79.91 | 9.2 |
| Ablation: state-wise risk | lathe | hard | 18 | 100% | 88.8% [85.6, 91.3] | 95.7% | 10.1% | 85.89 | 6.3 |
| Ablation: state-wise risk | rack | all | 54 | 100% | 86.3% [83.5, 89.0] | 92.2% | 8.0% | 99.90 | 8.9 |
| Ablation: state-wise risk | rack | easy | 18 | 100% | 91.4% [89.3, 93.7] | 95.9% | 4.4% | 112.73 | 12.3 |
| Ablation: state-wise risk | rack | medium | 18 | 100% | 83.5% [78.7, 87.1] | 89.3% | 8.9% | 95.49 | 8.9 |
| Ablation: state-wise risk | rack | hard | 18 | 100% | 84.0% [79.8, 88.8] | 91.3% | 10.9% | 94.54 | 6.3 |
| Ablation: state-wise risk | rebar | all | 54 | 100% | 92.8% [90.6, 94.7] | 96.7% | 4.1% | 85.52 | 9.1 |
| Ablation: state-wise risk | rebar | easy | 18 | 100% | 95.9% [93.9, 97.8] | 98.2% | 1.3% | 60.87 | 14.3 |
| Ablation: state-wise risk | rebar | medium | 18 | 100% | 94.3% [93.1, 95.7] | 97.6% | 3.0% | 85.31 | 9.2 |
| Ablation: state-wise risk | rebar | hard | 18 | 100% | 88.1% [84.2, 91.2] | 94.1% | 8.0% | 87.09 | 7.9 |
| Ablation: state-wise risk | tote | all | 54 | 100% | 97.0% [95.7, 98.2] | 99.0% | 2.6% | 111.21 | 7.3 |
| Ablation: state-wise risk | tote | easy | 18 | 100% | 99.3% [99.0, 99.6] | 99.5% | 0.3% | 110.98 | 8.2 |
| Ablation: state-wise risk | tote | medium | 18 | 100% | 98.4% [98.2, 98.5] | 99.2% | 1.2% | 110.68 | 8.2 |
| Ablation: state-wise risk | tote | hard | 18 | 100% | 93.3% [92.1, 94.4] | 98.4% | 6.2% | 111.53 | 6.1 |
| Ablation: translation-only error model | all | all | 216 | 100% | 80.8% [77.9, 83.4] | 86.6% | 18.2% | 39.17 | 8.9 |
| Ablation: translation-only error model | all | easy | 72 | 100% | 93.0% [90.9, 95.0] | 96.3% | 6.0% | 39.97 | 14.0 |
| Ablation: translation-only error model | all | medium | 72 | 100% | 79.4% [76.2, 82.6] | 83.1% | 19.8% | 39.12 | 10.0 |
| Ablation: translation-only error model | all | hard | 72 | 100% | 70.1% [67.0, 73.5] | 80.3% | 28.9% | 38.63 | 6.7 |
| Ablation: translation-only error model | lathe | all | 54 | 100% | 81.6% [77.0, 86.0] | 87.6% | 18.0% | 24.22 | 10.0 |
| Ablation: translation-only error model | lathe | easy | 18 | 100% | 93.0% [91.2, 94.6] | 97.6% | 6.1% | 20.67 | 14.0 |
| Ablation: translation-only error model | lathe | medium | 18 | 100% | 80.8% [78.2, 83.5] | 84.3% | 19.1% | 25.04 | 10.0 |
| Ablation: translation-only error model | lathe | hard | 18 | 100% | 70.9% [66.7, 75.2] | 81.0% | 28.7% | 26.56 | 6.7 |
| Ablation: translation-only error model | rack | all | 54 | 100% | 76.4% [72.2, 81.0] | 84.9% | 22.0% | 45.52 | 10.0 |
| Ablation: translation-only error model | rack | easy | 18 | 100% | 89.6% [87.6, 91.9] | 95.6% | 8.7% | 111.56 | 14.0 |
| Ablation: translation-only error model | rack | medium | 18 | 100% | 71.9% [70.2, 73.8] | 78.8% | 26.0% | 49.17 | 10.0 |
| Ablation: translation-only error model | rack | hard | 18 | 100% | 67.7% [66.4, 69.2] | 80.2% | 31.3% | 42.46 | 6.8 |
| Ablation: translation-only error model | rebar | all | 54 | 100% | 75.2% [69.6, 81.3] | 80.1% | 23.1% | 24.51 | 9.9 |
| Ablation: translation-only error model | rebar | easy | 18 | 100% | 90.1% [84.8, 93.8] | 92.7% | 8.5% | 22.27 | 16.0 |
| Ablation: translation-only error model | rebar | medium | 18 | 100% | 74.5% [69.9, 77.7] | 77.6% | 24.4% | 24.34 | 10.0 |
| Ablation: translation-only error model | rebar | hard | 18 | 100% | 61.1% [59.6, 62.5] | 70.1% | 36.3% | 28.10 | 6.9 |
| Ablation: translation-only error model | tote | all | 54 | 100% | 90.1% [86.3, 93.6] | 93.7% | 9.9% | 110.59 | 8.8 |
| Ablation: translation-only error model | tote | easy | 18 | 100% | 99.2% [98.9, 99.5] | 99.3% | 0.7% | 110.16 | 8.8 |
| Ablation: translation-only error model | tote | medium | 18 | 100% | 90.3% [88.0, 92.6] | 91.9% | 9.7% | 110.89 | 8.8 |
| Ablation: translation-only error model | tote | hard | 18 | 100% | 80.8% [77.8, 84.2] | 89.8% | 19.2% | 110.80 | 6.5 |
| CC-VPSTO (adapted) | all | all | 360 | 100% | 83.9% [80.6, 86.8] | 90.0% | 15.0% | 87.13 | 8.0 |
| CC-VPSTO (adapted) | all | easy | 120 | 100% | 94.3% [92.5, 95.8] | 97.7% | 4.3% | 87.20 | 10.1 |
| CC-VPSTO (adapted) | all | medium | 120 | 100% | 87.4% [84.5, 90.0] | 91.5% | 11.5% | 87.08 | 8.0 |
| CC-VPSTO (adapted) | all | hard | 120 | 100% | 70.0% [64.8, 74.9] | 80.7% | 29.1% | 86.99 | 5.0 |
| CC-VPSTO (adapted) | lathe | all | 90 | 100% | 78.4% [70.0, 85.7] | 83.9% | 21.1% | 84.22 | 7.7 |
| CC-VPSTO (adapted) | lathe | easy | 30 | 100% | 93.6% [91.9, 94.9] | 98.2% | 5.1% | 84.07 | 11.1 |
| CC-VPSTO (adapted) | lathe | medium | 30 | 100% | 84.9% [82.7, 87.0] | 88.6% | 15.0% | 84.42 | 8.2 |
| CC-VPSTO (adapted) | lathe | hard | 30 | 100% | 56.6% [48.6, 62.1] | 65.0% | 43.2% | 84.28 | 4.3 |
| CC-VPSTO (adapted) | rack | all | 90 | 100% | 78.2% [72.5, 83.7] | 86.9% | 19.6% | 90.28 | 7.1 |
| CC-VPSTO (adapted) | rack | easy | 30 | 100% | 91.6% [89.8, 93.6] | 97.7% | 6.1% | 114.52 | 10.8 |
| CC-VPSTO (adapted) | rack | medium | 30 | 100% | 79.8% [77.7, 82.0] | 87.4% | 17.6% | 88.80 | 7.1 |
| CC-VPSTO (adapted) | rack | hard | 30 | 100% | 63.3% [57.9, 68.0] | 75.4% | 35.1% | 86.88 | 4.5 |
| CC-VPSTO (adapted) | rebar | all | 90 | 100% | 90.2% [86.6, 93.3] | 96.4% | 8.0% | 84.24 | 9.6 |
| CC-VPSTO (adapted) | rebar | easy | 30 | 100% | 93.7% [88.5, 97.0] | 96.4% | 4.5% | 83.99 | 11.1 |
| CC-VPSTO (adapted) | rebar | medium | 30 | 100% | 91.5% [84.3, 95.8] | 95.3% | 6.6% | 84.21 | 9.9 |
| CC-VPSTO (adapted) | rebar | hard | 30 | 100% | 85.2% [81.9, 88.4] | 97.7% | 12.9% | 85.92 | 6.0 |
| CC-VPSTO (adapted) | tote | all | 90 | 100% | 88.7% [83.9, 93.2] | 92.7% | 11.2% | 113.32 | 8.0 |
| CC-VPSTO (adapted) | tote | easy | 30 | 100% | 98.3% [97.8, 98.8] | 98.5% | 1.5% | 113.02 | 8.0 |
| CC-VPSTO (adapted) | tote | medium | 30 | 100% | 93.2% [91.4, 94.8] | 94.8% | 6.8% | 113.28 | 8.0 |
| CC-VPSTO (adapted) | tote | hard | 30 | 100% | 74.7% [74.2, 75.2] | 84.7% | 25.3% | 113.46 | 5.4 |
| CRISP (proposed) | all | all | 360 | 100% | 92.2% [91.0, 93.3] | 96.4% | 5.3% | 108.68 | 8.6 |
| CRISP (proposed) | all | easy | 120 | 100% | 95.3% [93.8, 96.6] | 97.5% | 1.9% | 109.21 | 13.7 |
| CRISP (proposed) | all | medium | 120 | 100% | 92.9% [90.9, 94.6] | 96.0% | 4.7% | 92.40 | 9.1 |
| CRISP (proposed) | all | hard | 120 | 100% | 88.6% [86.6, 90.3] | 95.5% | 9.5% | 109.39 | 6.5 |
| CRISP (proposed) | lathe | all | 90 | 100% | 92.5% [90.6, 94.1] | 95.7% | 5.5% | 69.03 | 8.7 |
| CRISP (proposed) | lathe | easy | 30 | 100% | 93.7% [90.4, 95.7] | 95.8% | 2.9% | 40.94 | 15.3 |
| CRISP (proposed) | lathe | medium | 30 | 100% | 94.9% [93.8, 96.0] | 97.1% | 3.6% | 66.99 | 9.2 |
| CRISP (proposed) | lathe | hard | 30 | 100% | 88.8% [86.5, 91.0] | 94.3% | 10.1% | 87.33 | 5.8 |
| CRISP (proposed) | rack | all | 90 | 100% | 87.4% [85.0, 89.7] | 94.4% | 8.9% | 98.46 | 8.4 |
| CRISP (proposed) | rack | easy | 30 | 100% | 92.6% [91.1, 94.2] | 96.9% | 3.2% | 112.64 | 13.7 |
| CRISP (proposed) | rack | medium | 30 | 100% | 85.9% [83.3, 88.3] | 92.4% | 9.9% | 96.90 | 8.5 |
| CRISP (proposed) | rack | hard | 30 | 100% | 83.8% [79.6, 87.7] | 94.0% | 13.7% | 93.50 | 6.6 |
| CRISP (proposed) | rebar | all | 90 | 100% | 93.8% [91.9, 95.5] | 97.6% | 2.8% | 67.59 | 9.5 |
| CRISP (proposed) | rebar | easy | 30 | 100% | 95.8% [93.1, 98.0] | 98.2% | 1.1% | 45.50 | 15.5 |
| CRISP (proposed) | rebar | medium | 30 | 100% | 95.5% [94.5, 96.6] | 98.5% | 1.3% | 59.85 | 9.8 |
| CRISP (proposed) | rebar | hard | 30 | 100% | 90.2% [87.2, 92.7] | 96.1% | 6.1% | 110.68 | 7.4 |
| CRISP (proposed) | tote | all | 90 | 100% | 95.3% [93.6, 96.9] | 97.7% | 4.1% | 110.36 | 7.9 |
| CRISP (proposed) | tote | easy | 30 | 100% | 99.1% [98.6, 99.5] | 99.3% | 0.3% | 109.86 | 9.1 |
| CRISP (proposed) | tote | medium | 30 | 100% | 95.2% [93.2, 96.9] | 96.2% | 3.9% | 110.37 | 8.6 |
| CRISP (proposed) | tote | hard | 30 | 100% | 91.5% [90.2, 92.6] | 97.6% | 8.1% | 110.73 | 6.4 |
