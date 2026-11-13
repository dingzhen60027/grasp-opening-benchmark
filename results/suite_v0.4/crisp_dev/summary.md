# Experiment summary

Runs: 216. Task success counts planning failures as 0; CI = 95% bootstrap over tasks. Collision | valid = mean episode collision rate of valid paths. Feasible-episode success: goal-state flags from `../validation/goal_feasibility.json`.

| Planner | Family | Level | Runs | Planning success | Task success [95% CI] | Feasible-episode success | Collision \| valid | Plan time (s, median) | Min clearance (mm, median) |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| B6 Keyframe-guided | all | all | 72 | 100% | 76.7% [69.5, 83.1] | 82.0% | 22.6% | 114.04 | 8.8 |
| B6 Keyframe-guided | all | easy | 24 | 100% | 93.4% [91.1, 95.9] | 96.8% | 5.6% | 13.01 | 14.0 |
| B6 Keyframe-guided | all | medium | 24 | 100% | 80.1% [75.5, 84.9] | 84.0% | 19.3% | 114.04 | 10.0 |
| B6 Keyframe-guided | all | hard | 24 | 100% | 56.5% [50.5, 62.4] | 65.1% | 43.0% | 114.04 | 6.0 |
| B6 Keyframe-guided | lathe | all | 18 | 100% | 76.9% [64.3, 88.9] | 82.6% | 22.5% | 7.94 | 10.0 |
| B6 Keyframe-guided | lathe | easy | 6 | 100% | 93.8% [92.2, 95.3] | 99.2% | 5.1% | 7.84 | 14.0 |
| B6 Keyframe-guided | lathe | medium | 6 | 100% | 80.8% [76.4, 85.2] | 84.4% | 18.6% | 63.71 | 10.0 |
| B6 Keyframe-guided | lathe | hard | 6 | 100% | 56.2% [55.9, 56.6] | 64.1% | 43.8% | 7.40 | 6.0 |
| B6 Keyframe-guided | rack | all | 18 | 100% | 70.2% [54.1, 85.0] | 78.3% | 28.4% | 65.75 | 10.0 |
| B6 Keyframe-guided | rack | easy | 6 | 100% | 90.4% [89.1, 91.8] | 96.3% | 8.0% | 13.01 | 14.0 |
| B6 Keyframe-guided | rack | medium | 6 | 100% | 75.6% [71.1, 80.1] | 82.8% | 23.0% | 114.04 | 10.0 |
| B6 Keyframe-guided | rack | hard | 6 | 100% | 44.5% [42.2, 46.9] | 56.0% | 54.1% | 65.75 | 6.0 |
| B6 Keyframe-guided | rebar | all | 18 | 100% | 74.1% [62.8, 85.3] | 78.5% | 25.2% | 114.04 | 10.0 |
| B6 Keyframe-guided | rebar | easy | 6 | 100% | 90.8% [89.8, 91.8] | 93.0% | 8.0% | 63.27 | 16.0 |
| B6 Keyframe-guided | rebar | medium | 6 | 100% | 74.2% [74.2, 74.2] | 77.0% | 25.2% | 61.11 | 10.0 |
| B6 Keyframe-guided | rebar | hard | 6 | 100% | 57.2% [54.6, 59.8] | 65.4% | 42.3% | 114.04 | 6.0 |
| B6 Keyframe-guided | tote | all | 18 | 100% | 85.5% [74.7, 95.2] | 88.4% | 14.5% | 114.05 | 8.0 |
| B6 Keyframe-guided | tote | easy | 6 | 100% | 98.6% [98.0, 99.2] | 98.8% | 1.2% | 114.05 | 8.0 |
| B6 Keyframe-guided | tote | medium | 6 | 100% | 89.6% [87.9, 91.4] | 91.8% | 10.4% | 114.05 | 8.4 |
| B6 Keyframe-guided | tote | hard | 6 | 100% | 68.2% [64.8, 71.5] | 74.7% | 31.8% | 114.05 | 6.0 |
| CC-VPSTO (adapted) | all | all | 72 | 100% | 84.6% [78.7, 89.8] | 90.7% | 14.5% | 120.26 | 8.0 |
| CC-VPSTO (adapted) | all | easy | 24 | 100% | 94.8% [93.1, 96.5] | 98.3% | 4.0% | 90.58 | 9.6 |
| CC-VPSTO (adapted) | all | medium | 24 | 100% | 90.0% [86.3, 93.5] | 94.4% | 9.1% | 121.86 | 8.2 |
| CC-VPSTO (adapted) | all | hard | 24 | 100% | 68.9% [60.3, 78.3] | 79.3% | 30.5% | 105.69 | 5.3 |
| CC-VPSTO (adapted) | lathe | all | 18 | 100% | 79.3% [67.8, 90.1] | 85.1% | 20.1% | 84.31 | 8.1 |
| CC-VPSTO (adapted) | lathe | easy | 6 | 100% | 93.5% [91.5, 95.4] | 98.9% | 5.1% | 83.56 | 10.6 |
| CC-VPSTO (adapted) | lathe | medium | 6 | 100% | 84.1% [80.9, 87.4] | 87.9% | 15.3% | 105.19 | 8.2 |
| CC-VPSTO (adapted) | lathe | hard | 6 | 100% | 60.2% [59.0, 61.5] | 68.7% | 39.8% | 84.08 | 4.1 |
| CC-VPSTO (adapted) | rack | all | 18 | 100% | 77.7% [64.0, 90.6] | 87.0% | 20.4% | 107.22 | 7.1 |
| CC-VPSTO (adapted) | rack | easy | 6 | 100% | 92.3% [91.8, 92.8] | 98.3% | 5.7% | 90.58 | 9.8 |
| CC-VPSTO (adapted) | rack | medium | 6 | 100% | 87.2% [83.1, 91.4] | 95.6% | 11.1% | 122.99 | 7.9 |
| CC-VPSTO (adapted) | rack | hard | 6 | 100% | 53.5% [52.2, 54.8] | 67.2% | 44.5% | 107.37 | 4.3 |
| CC-VPSTO (adapted) | rebar | all | 18 | 100% | 92.5% [88.5, 96.1] | 98.6% | 6.4% | 104.59 | 9.1 |
| CC-VPSTO (adapted) | rebar | easy | 6 | 100% | 95.4% [93.8, 97.1] | 97.7% | 3.2% | 104.35 | 11.9 |
| CC-VPSTO (adapted) | rebar | medium | 6 | 100% | 95.6% [93.8, 97.5] | 99.1% | 2.9% | 104.98 | 9.3 |
| CC-VPSTO (adapted) | rebar | hard | 6 | 100% | 86.4% [85.2, 87.6] | 99.0% | 13.0% | 104.55 | 6.0 |
| CC-VPSTO (adapted) | tote | all | 18 | 100% | 88.8% [79.6, 96.2] | 92.0% | 11.2% | 121.08 | 8.0 |
| CC-VPSTO (adapted) | tote | easy | 6 | 100% | 98.0% [97.9, 98.0] | 98.2% | 1.9% | 121.30 | 8.0 |
| CC-VPSTO (adapted) | tote | medium | 6 | 100% | 93.0% [91.5, 94.4] | 95.2% | 7.0% | 121.32 | 8.0 |
| CC-VPSTO (adapted) | tote | hard | 6 | 100% | 75.3% [68.6, 82.0] | 82.6% | 24.7% | 120.97 | 5.3 |
| CRISP (proposed) | all | all | 72 | 100% | 93.5% [91.6, 95.2] | 97.4% | 3.8% | 108.84 | 8.4 |
| CRISP (proposed) | all | easy | 24 | 100% | 95.9% [94.2, 97.6] | 98.6% | 1.0% | 91.24 | 13.8 |
| CRISP (proposed) | all | medium | 24 | 100% | 94.8% [92.6, 96.8] | 98.0% | 3.1% | 111.41 | 8.7 |
| CRISP (proposed) | all | hard | 24 | 100% | 89.8% [86.3, 93.2] | 95.7% | 7.2% | 108.04 | 6.3 |
| CRISP (proposed) | lathe | all | 18 | 100% | 92.9% [90.2, 95.6] | 96.4% | 5.0% | 77.71 | 8.8 |
| CRISP (proposed) | lathe | easy | 6 | 100% | 95.1% [92.8, 97.4] | 98.8% | 1.2% | 44.02 | 16.0 |
| CRISP (proposed) | lathe | medium | 6 | 100% | 94.5% [92.6, 96.5] | 97.1% | 3.7% | 92.46 | 8.8 |
| CRISP (proposed) | lathe | hard | 6 | 100% | 89.1% [86.5, 91.8] | 93.3% | 10.1% | 84.75 | 6.3 |
| CRISP (proposed) | rack | all | 18 | 100% | 88.9% [84.9, 92.4] | 96.8% | 6.2% | 103.56 | 9.1 |
| CRISP (proposed) | rack | easy | 6 | 100% | 93.4% [92.4, 94.3] | 98.4% | 2.5% | 86.30 | 11.6 |
| CRISP (proposed) | rack | medium | 6 | 100% | 90.6% [89.5, 91.8] | 97.2% | 6.4% | 113.34 | 9.1 |
| CRISP (proposed) | rack | hard | 6 | 100% | 82.8% [80.1, 85.5] | 94.7% | 9.8% | 101.82 | 5.9 |
| CRISP (proposed) | rebar | all | 18 | 100% | 95.3% [93.6, 96.7] | 97.9% | 1.3% | 95.75 | 10.3 |
| CRISP (proposed) | rebar | easy | 6 | 100% | 96.0% [95.4, 96.6] | 97.9% | 0.1% | 79.20 | 15.8 |
| CRISP (proposed) | rebar | medium | 6 | 100% | 96.1% [94.7, 97.5] | 98.7% | 0.8% | 74.48 | 10.3 |
| CRISP (proposed) | rebar | hard | 6 | 100% | 93.9% [91.4, 96.4] | 97.2% | 3.1% | 110.53 | 7.3 |
| CRISP (proposed) | tote | all | 18 | 100% | 96.8% [94.1, 98.7] | 98.6% | 2.6% | 110.40 | 7.5 |
| CRISP (proposed) | tote | easy | 6 | 100% | 99.1% [98.7, 99.5] | 99.3% | 0.2% | 110.54 | 9.2 |
| CRISP (proposed) | tote | medium | 6 | 100% | 98.0% [97.9, 98.2] | 99.0% | 1.6% | 110.46 | 7.5 |
| CRISP (proposed) | tote | hard | 6 | 100% | 93.4% [90.6, 96.1] | 97.6% | 6.0% | 109.89 | 5.9 |
