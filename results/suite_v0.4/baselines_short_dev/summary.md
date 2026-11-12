# Experiment summary

Runs: 96. Task success counts planning failures as 0; CI = 95% bootstrap over tasks. Collision | valid = mean episode collision rate of valid paths.

| Planner | Family | Level | Runs | Planning success | Task success [95% CI] | Feasible-episode success | Collision \| valid | Plan time (s, median) | Min clearance (mm, median) |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| B1 RRTConnect | all | all | 24 | 46% | 16.6% [7.6, 26.6] | 18.7% | 62.1% | 85.55 | 2.1 |
| B1 RRTConnect | all | easy | 8 | 62% | 21.7% [5.5, 40.8] | 22.8% | 61.6% | 85.55 | 0.8 |
| B1 RRTConnect | all | medium | 8 | 50% | 24.8% [6.4, 42.2] | 28.0% | 50.4% | 85.56 | 2.5 |
| B1 RRTConnect | all | hard | 8 | 25% | 3.3% [0.0, 8.0] | 5.2% | 86.7% | 85.56 | 0.7 |
| B1 RRTConnect | lathe | all | 6 | 67% | 20.6% [5.2, 36.2] | 21.7% | 69.1% | 85.55 | 1.5 |
| B1 RRTConnect | lathe | easy | 2 | 100% | 15.6% [9.4, 21.9] | 15.7% | 84.4% | 85.55 | 0.6 |
| B1 RRTConnect | lathe | medium | 2 | 100% | 46.1% [40.6, 51.6] | 49.4% | 53.9% | 85.55 | 2.7 |
| B1 RRTConnect | lathe | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rack | all | 6 | 17% | 9.1% [0.0, 27.3] | 10.4% | 28.1% | 85.55 | 6.6 |
| B1 RRTConnect | rack | easy | 2 | 50% | 27.3% [0.0, 54.7] | 31.1% | 28.1% | 85.55 | 6.6 |
| B1 RRTConnect | rack | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rack | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | tote | all | 6 | 100% | 36.7% [17.2, 57.6] | 42.6% | 63.0% | 85.56 | 1.5 |
| B1 RRTConnect | tote | easy | 2 | 100% | 43.8% [12.5, 75.0] | 44.3% | 55.5% | 85.55 | 2.6 |
| B1 RRTConnect | tote | medium | 2 | 100% | 53.1% [48.4, 57.8] | 62.6% | 46.9% | 85.57 | 2.5 |
| B1 RRTConnect | tote | hard | 2 | 100% | 13.3% [7.8, 18.8] | 21.0% | 86.7% | 85.56 | 0.7 |
| B3 RRTConnect + 3 mm margin | all | all | 24 | 25% | 17.2% [5.7, 30.1] | 17.6% | 30.7% | 85.55 | 4.4 |
| B3 RRTConnect + 3 mm margin | all | easy | 8 | 50% | 34.4% [9.4, 58.8] | 34.6% | 30.9% | 85.55 | 5.5 |
| B3 RRTConnect + 3 mm margin | all | medium | 8 | 25% | 17.2% [0.0, 40.6] | 18.2% | 30.5% | 85.54 | 3.7 |
| B3 RRTConnect + 3 mm margin | all | hard | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | lathe | all | 6 | 33% | 19.5% [0.0, 39.8] | 19.7% | 41.4% | 85.55 | 4.9 |
| B3 RRTConnect + 3 mm margin | lathe | easy | 2 | 100% | 58.6% [56.2, 60.9] | 59.1% | 41.4% | 85.55 | 4.9 |
| B3 RRTConnect + 3 mm margin | lathe | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | lathe | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rack | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rack | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rack | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rack | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | tote | all | 6 | 67% | 49.2% [21.9, 76.6] | 50.8% | 25.4% | 85.55 | 4.4 |
| B3 RRTConnect + 3 mm margin | tote | easy | 2 | 100% | 78.9% [75.0, 82.8] | 79.5% | 20.3% | 85.55 | 5.7 |
| B3 RRTConnect + 3 mm margin | tote | medium | 2 | 100% | 68.8% [56.2, 81.2] | 72.8% | 30.5% | 85.54 | 3.7 |
| B3 RRTConnect + 3 mm margin | tote | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | all | 24 | 17% | 10.2% [2.0, 20.1] | 10.6% | 34.4% | 85.55 | 5.8 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | easy | 8 | 50% | 30.7% [8.6, 52.9] | 31.9% | 34.4% | 85.55 | 5.8 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | medium | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | hard | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | all | 6 | 17% | 12.5% [0.0, 37.5] | 12.7% | 25.0% | 85.55 | 6.3 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | easy | 2 | 50% | 37.5% [0.0, 75.0] | 38.1% | 25.0% | 85.55 | 6.3 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | all | 6 | 17% | 9.1% [0.0, 27.3] | 10.4% | 28.1% | 85.55 | 6.6 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | easy | 2 | 50% | 27.3% [0.0, 54.7] | 31.1% | 28.1% | 85.55 | 6.6 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | all | 6 | 33% | 19.3% [0.0, 42.2] | 19.4% | 42.2% | 85.55 | 4.3 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | easy | 2 | 100% | 57.8% [46.9, 68.8] | 58.2% | 42.2% | 85.55 | 4.3 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B6 Keyframe-guided | all | all | 24 | 100% | 77.1% [70.2, 83.4] | 94.0% | 22.3% | 85.53 | 8.4 |
| B6 Keyframe-guided | all | easy | 8 | 100% | 92.2% [89.5, 95.3] | 95.0% | 7.2% | 7.68 | 14.0 |
| B6 Keyframe-guided | all | medium | 8 | 100% | 81.4% [76.8, 86.1] | 92.3% | 18.0% | 85.54 | 10.0 |
| B6 Keyframe-guided | all | hard | 8 | 100% | 57.6% [51.2, 63.9] | 94.6% | 41.6% | 85.54 | 6.0 |
| B6 Keyframe-guided | lathe | all | 6 | 100% | 77.6% [66.7, 87.8] | 90.2% | 22.4% | 7.09 | 10.0 |
| B6 Keyframe-guided | lathe | easy | 2 | 100% | 92.2% [90.6, 93.8] | 92.9% | 7.8% | 6.95 | 14.0 |
| B6 Keyframe-guided | lathe | medium | 2 | 100% | 79.7% [75.0, 84.4] | 85.5% | 20.3% | 46.21 | 10.0 |
| B6 Keyframe-guided | lathe | hard | 2 | 100% | 60.9% [57.8, 64.1] | 92.3% | 39.1% | 7.09 | 6.0 |
| B6 Keyframe-guided | rack | all | 6 | 100% | 70.1% [54.4, 84.4] | 92.7% | 27.9% | 51.69 | 10.0 |
| B6 Keyframe-guided | rack | easy | 2 | 100% | 89.8% [87.5, 92.2] | 97.0% | 8.6% | 7.68 | 14.0 |
| B6 Keyframe-guided | rack | medium | 2 | 100% | 75.8% [70.3, 81.2] | 89.1% | 21.9% | 85.54 | 10.0 |
| B6 Keyframe-guided | rack | hard | 2 | 100% | 44.5% [43.8, 45.3] | 91.9% | 53.1% | 51.69 | 6.0 |
| B6 Keyframe-guided | rebar | all | 6 | 100% | 75.0% [63.0, 85.4] | 96.1% | 24.5% | 85.54 | 8.8 |
| B6 Keyframe-guided | rebar | easy | 2 | 100% | 89.1% [85.9, 92.2] | 92.6% | 10.2% | 45.64 | 14.8 |
| B6 Keyframe-guided | rebar | medium | 2 | 100% | 78.9% [78.1, 79.7] | 95.7% | 21.1% | 46.06 | 8.8 |
| B6 Keyframe-guided | rebar | hard | 2 | 100% | 57.0% [50.0, 64.1] | 100.0% | 42.2% | 85.55 | 6.0 |
| B6 Keyframe-guided | tote | all | 6 | 100% | 85.7% [75.0, 95.6] | 97.0% | 14.3% | 85.54 | 8.0 |
| B6 Keyframe-guided | tote | easy | 2 | 100% | 97.7% [95.3, 100.0] | 97.6% | 2.3% | 85.55 | 8.0 |
| B6 Keyframe-guided | tote | medium | 2 | 100% | 91.4% [90.6, 92.2] | 99.1% | 8.6% | 85.54 | 8.4 |
| B6 Keyframe-guided | tote | hard | 2 | 100% | 68.0% [65.6, 70.3] | 94.3% | 32.0% | 85.54 | 6.0 |
