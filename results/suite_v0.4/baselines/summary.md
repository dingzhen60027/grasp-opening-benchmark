# Experiment summary

Runs: 1440. Task success counts planning failures as 0; CI = 95% bootstrap over tasks. Collision | valid = mean episode collision rate of valid paths. Feasible-episode success: goal-state flags from `../validation/goal_feasibility.json`.

| Planner | Family | Level | Runs | Planning success | Task success [95% CI] | Feasible-episode success | Collision \| valid | Plan time (s, median) | Min clearance (mm, median) |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| B1 RRTConnect | all | all | 360 | 31% | 14.5% [9.8, 19.0] | 15.0% | 53.3% | 114.05 | 1.9 |
| B1 RRTConnect | all | easy | 120 | 40% | 19.6% [11.2, 28.2] | 20.0% | 50.5% | 114.05 | 2.0 |
| B1 RRTConnect | all | medium | 120 | 33% | 16.4% [8.1, 24.7] | 16.7% | 50.9% | 114.05 | 1.9 |
| B1 RRTConnect | all | hard | 120 | 20% | 7.4% [2.1, 13.8] | 8.4% | 62.9% | 114.05 | 1.3 |
| B1 RRTConnect | lathe | all | 90 | 28% | 11.4% [6.7, 16.1] | 11.9% | 58.8% | 114.05 | 1.9 |
| B1 RRTConnect | lathe | easy | 30 | 47% | 18.9% [14.5, 25.5] | 19.8% | 59.3% | 114.05 | 1.8 |
| B1 RRTConnect | lathe | medium | 30 | 37% | 15.3% [9.2, 22.8] | 15.9% | 58.2% | 114.05 | 1.9 |
| B1 RRTConnect | lathe | hard | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rack | all | 90 | 7% | 3.0% [0.0, 8.4] | 3.2% | 52.5% | 114.06 | 3.9 |
| B1 RRTConnect | rack | easy | 30 | 20% | 9.0% [0.0, 23.5] | 9.5% | 52.5% | 114.06 | 3.9 |
| B1 RRTConnect | rack | medium | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rack | hard | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | all | 90 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | easy | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | medium | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | hard | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | tote | all | 90 | 90% | 43.4% [36.5, 49.5] | 45.0% | 51.6% | 114.05 | 1.6 |
| B1 RRTConnect | tote | easy | 30 | 93% | 50.5% [47.0, 54.4] | 50.6% | 45.6% | 114.05 | 2.0 |
| B1 RRTConnect | tote | medium | 30 | 97% | 50.2% [44.4, 55.9] | 51.0% | 48.1% | 114.05 | 1.9 |
| B1 RRTConnect | tote | hard | 30 | 80% | 29.6% [18.1, 42.9] | 33.5% | 62.9% | 114.05 | 1.3 |
| B3 RRTConnect + 3 mm margin | all | all | 360 | 27% | 17.7% [12.1, 23.5] | 18.5% | 34.3% | 114.05 | 4.1 |
| B3 RRTConnect + 3 mm margin | all | easy | 120 | 38% | 25.5% [13.7, 37.3] | 26.0% | 30.6% | 114.05 | 4.4 |
| B3 RRTConnect + 3 mm margin | all | medium | 120 | 27% | 17.2% [7.7, 27.7] | 17.6% | 35.3% | 114.05 | 4.0 |
| B3 RRTConnect + 3 mm margin | all | hard | 120 | 18% | 10.4% [3.5, 18.3] | 11.8% | 40.6% | 114.05 | 3.5 |
| B3 RRTConnect + 3 mm margin | lathe | all | 90 | 17% | 10.0% [5.6, 14.7] | 10.5% | 39.5% | 114.05 | 4.1 |
| B3 RRTConnect + 3 mm margin | lathe | easy | 30 | 33% | 20.6% [15.3, 26.2] | 21.7% | 37.8% | 114.05 | 4.1 |
| B3 RRTConnect + 3 mm margin | lathe | medium | 30 | 17% | 9.5% [5.3, 12.8] | 9.9% | 43.0% | 114.05 | 3.9 |
| B3 RRTConnect + 3 mm margin | lathe | hard | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rack | all | 90 | 8% | 4.2% [0.0, 11.0] | 4.5% | 38.2% | 114.05 | 4.9 |
| B3 RRTConnect + 3 mm margin | rack | easy | 30 | 20% | 11.0% [0.0, 29.5] | 11.5% | 36.3% | 114.05 | 5.3 |
| B3 RRTConnect + 3 mm margin | rack | medium | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rack | hard | 30 | 3% | 1.7% [0.0, 5.0] | 2.0% | 49.2% | 114.05 | 3.7 |
| B3 RRTConnect + 3 mm margin | rebar | all | 90 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rebar | easy | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rebar | medium | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rebar | hard | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | tote | all | 90 | 84% | 56.6% [48.5, 64.3] | 58.8% | 32.9% | 114.05 | 4.1 |
| B3 RRTConnect + 3 mm margin | tote | easy | 30 | 97% | 70.5% [64.9, 76.3] | 70.6% | 26.9% | 114.05 | 4.7 |
| B3 RRTConnect + 3 mm margin | tote | medium | 30 | 90% | 59.5% [52.7, 66.0] | 60.5% | 33.9% | 114.05 | 4.3 |
| B3 RRTConnect + 3 mm margin | tote | hard | 30 | 67% | 39.9% [26.5, 53.2] | 45.2% | 40.1% | 114.05 | 3.4 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | all | 360 | 21% | 13.2% [8.4, 18.3] | 13.6% | 36.5% | 114.05 | 3.7 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | easy | 120 | 31% | 21.0% [10.1, 32.5] | 21.2% | 31.5% | 114.05 | 4.3 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | medium | 120 | 21% | 12.9% [5.9, 20.7] | 13.2% | 38.0% | 114.05 | 3.9 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | hard | 120 | 11% | 5.6% [1.3, 11.2] | 6.4% | 47.7% | 114.05 | 2.9 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | all | 90 | 13% | 7.0% [3.4, 11.1] | 7.3% | 47.3% | 114.05 | 3.7 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | easy | 30 | 27% | 12.6% [8.6, 17.0] | 13.3% | 52.3% | 114.05 | 3.3 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | medium | 30 | 13% | 8.4% [2.2, 16.0] | 8.7% | 37.1% | 114.05 | 4.4 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | hard | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | all | 90 | 2% | 1.5% [0.0, 4.4] | 1.5% | 29.5% | 114.05 | 6.0 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | easy | 30 | 7% | 4.4% [0.0, 13.1] | 4.6% | 29.5% | 114.05 | 6.0 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | medium | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | hard | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | all | 90 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | easy | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | medium | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | hard | 30 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | all | 90 | 68% | 44.2% [34.5, 54.2] | 45.5% | 34.6% | 114.05 | 3.7 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | easy | 30 | 90% | 66.9% [58.6, 76.0] | 67.0% | 25.5% | 114.05 | 4.4 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | medium | 30 | 70% | 43.3% [36.9, 50.1] | 44.0% | 38.1% | 114.05 | 3.7 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | hard | 30 | 43% | 22.6% [10.1, 34.8] | 25.5% | 47.7% | 114.05 | 2.9 |
| B6 Keyframe-guided | all | all | 360 | 99% | 75.1% [70.7, 79.2] | 80.3% | 23.0% | 114.03 | 8.8 |
| B6 Keyframe-guided | all | easy | 120 | 96% | 89.6% [81.2, 94.7] | 92.8% | 5.5% | 114.04 | 14.0 |
| B6 Keyframe-guided | all | medium | 120 | 100% | 79.0% [76.3, 81.7] | 82.8% | 20.2% | 64.48 | 10.0 |
| B6 Keyframe-guided | all | hard | 120 | 100% | 56.6% [53.5, 59.6] | 65.3% | 42.7% | 114.04 | 6.0 |
| B6 Keyframe-guided | lathe | all | 90 | 100% | 76.6% [69.2, 83.4] | 82.1% | 22.9% | 7.88 | 10.0 |
| B6 Keyframe-guided | lathe | easy | 30 | 100% | 93.8% [92.8, 94.7] | 98.4% | 5.3% | 8.03 | 14.0 |
| B6 Keyframe-guided | lathe | medium | 30 | 100% | 79.7% [77.6, 81.8] | 83.1% | 20.2% | 7.56 | 10.0 |
| B6 Keyframe-guided | lathe | hard | 30 | 100% | 56.5% [54.7, 58.4] | 64.8% | 43.3% | 8.54 | 6.0 |
| B6 Keyframe-guided | rack | all | 90 | 94% | 66.5% [55.0, 75.1] | 74.0% | 28.0% | 13.21 | 10.0 |
| B6 Keyframe-guided | rack | easy | 30 | 83% | 74.2% [44.0, 90.6] | 79.4% | 9.5% | 114.05 | 14.0 |
| B6 Keyframe-guided | rack | medium | 30 | 100% | 71.9% [70.2, 73.8] | 78.8% | 26.0% | 13.06 | 10.0 |
| B6 Keyframe-guided | rack | hard | 30 | 100% | 53.3% [51.4, 55.3] | 63.6% | 45.6% | 8.85 | 6.0 |
| B6 Keyframe-guided | rebar | all | 90 | 100% | 72.6% [63.9, 80.7] | 76.9% | 26.1% | 114.03 | 9.4 |
| B6 Keyframe-guided | rebar | easy | 30 | 100% | 92.2% [90.3, 93.9] | 94.9% | 6.4% | 62.47 | 16.0 |
| B6 Keyframe-guided | rebar | medium | 30 | 100% | 75.4% [73.9, 76.9] | 78.5% | 23.5% | 63.44 | 10.0 |
| B6 Keyframe-guided | rebar | hard | 30 | 100% | 50.2% [44.7, 55.0] | 57.4% | 48.5% | 114.04 | 6.0 |
| B6 Keyframe-guided | tote | all | 90 | 100% | 84.7% [78.2, 90.7] | 88.2% | 15.2% | 114.04 | 8.0 |
| B6 Keyframe-guided | tote | easy | 30 | 100% | 98.4% [97.4, 99.3] | 98.6% | 1.4% | 114.05 | 8.8 |
| B6 Keyframe-guided | tote | medium | 30 | 100% | 89.1% [87.1, 91.0] | 90.6% | 10.9% | 114.05 | 8.0 |
| B6 Keyframe-guided | tote | hard | 30 | 100% | 66.5% [63.9, 68.9] | 75.5% | 33.4% | 114.04 | 6.0 |
