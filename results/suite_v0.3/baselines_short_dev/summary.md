# Experiment summary

Runs: 168. Task success counts planning failures as 0; CI = 95% bootstrap over tasks. Collision | valid = mean episode collision rate of valid paths. Feasible-episode success: goal-state flags from `../validation/goal_feasibility.json`.

| Planner | Family | Level | Runs | Planning success | Task success [95% CI] | Feasible-episode success | Collision \| valid | Plan time (s, median) | Min clearance (mm, median) |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| B0 Reference (oracle) | all | all | 24 | 100% | 69.1% [58.5, 79.1] | 94.5% | 28.6% | 0.00 | 8.0 |
| B0 Reference (oracle) | all | easy | 8 | 100% | 92.6% [85.5, 98.2] | 99.8% | 3.9% | 0.00 | 14.0 |
| B0 Reference (oracle) | all | medium | 8 | 100% | 75.6% [66.0, 85.2] | 95.0% | 21.9% | 0.00 | 9.0 |
| B0 Reference (oracle) | all | hard | 8 | 100% | 39.1% [30.5, 47.3] | 88.8% | 60.0% | 0.00 | 5.0 |
| B0 Reference (oracle) | lathe | all | 6 | 100% | 80.5% [63.0, 96.6] | 100.0% | 19.3% | 0.00 | 10.0 |
| B0 Reference (oracle) | lathe | easy | 2 | 100% | 99.2% [98.4, 100.0] | 100.0% | 0.0% | 0.00 | 20.0 |
| B0 Reference (oracle) | lathe | medium | 2 | 100% | 92.2% [89.1, 95.3] | 100.0% | 7.8% | 0.00 | 10.0 |
| B0 Reference (oracle) | lathe | hard | 2 | 100% | 50.0% [46.9, 53.1] | 100.0% | 50.0% | 0.00 | 5.0 |
| B0 Reference (oracle) | rack | all | 6 | 100% | 57.8% [41.7, 72.9] | 90.7% | 33.6% | 0.00 | 8.0 |
| B0 Reference (oracle) | rack | easy | 2 | 100% | 77.3% [71.9, 82.8] | 100.0% | 10.9% | 0.00 | 8.0 |
| B0 Reference (oracle) | rack | medium | 2 | 100% | 64.1% [60.9, 67.2] | 90.2% | 25.8% | 0.00 | 8.0 |
| B0 Reference (oracle) | rack | hard | 2 | 100% | 32.0% [28.1, 35.9] | 82.0% | 64.1% | 0.00 | 5.0 |
| B0 Reference (oracle) | rebar | all | 6 | 100% | 59.6% [35.9, 83.1] | 87.4% | 40.1% | 0.00 | 10.0 |
| B0 Reference (oracle) | rebar | easy | 2 | 100% | 94.5% [93.8, 95.3] | 99.2% | 4.7% | 0.00 | 20.0 |
| B0 Reference (oracle) | rebar | medium | 2 | 100% | 60.9% [59.4, 62.5] | 89.8% | 39.1% | 0.00 | 10.0 |
| B0 Reference (oracle) | rebar | hard | 2 | 100% | 23.4% [20.3, 26.6] | 73.1% | 76.6% | 0.00 | 5.0 |
| B0 Reference (oracle) | tote | all | 6 | 100% | 78.4% [61.5, 93.8] | 100.0% | 21.4% | 0.00 | 8.0 |
| B0 Reference (oracle) | tote | easy | 2 | 100% | 99.2% [98.4, 100.0] | 100.0% | 0.0% | 0.00 | 8.0 |
| B0 Reference (oracle) | tote | medium | 2 | 100% | 85.2% [82.8, 87.5] | 100.0% | 14.8% | 0.00 | 8.0 |
| B0 Reference (oracle) | tote | hard | 2 | 100% | 50.8% [46.9, 54.7] | 100.0% | 49.2% | 0.00 | 5.0 |
| B1 RRTConnect | all | all | 24 | 21% | 12.0% [2.5, 22.6] | 14.4% | 37.2% | 85.56 | 3.2 |
| B1 RRTConnect | all | easy | 8 | 50% | 27.7% [6.2, 50.6] | 31.6% | 40.2% | 85.56 | 2.8 |
| B1 RRTConnect | all | medium | 8 | 12% | 8.2% [0.0, 24.6] | 11.7% | 25.0% | 85.56 | 3.5 |
| B1 RRTConnect | all | hard | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | lathe | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | lathe | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | lathe | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | lathe | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rack | all | 6 | 50% | 29.9% [6.0, 56.0] | 39.6% | 31.2% | 85.56 | 3.5 |
| B1 RRTConnect | rack | easy | 2 | 100% | 57.0% [35.9, 78.1] | 72.2% | 34.4% | 85.56 | 4.1 |
| B1 RRTConnect | rack | medium | 2 | 50% | 32.8% [0.0, 65.6] | 46.7% | 25.0% | 85.56 | 3.5 |
| B1 RRTConnect | rack | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | tote | all | 6 | 33% | 18.0% [0.0, 45.6] | 18.0% | 46.1% | 85.55 | 1.7 |
| B1 RRTConnect | tote | easy | 2 | 100% | 53.9% [25.0, 82.8] | 54.1% | 46.1% | 85.55 | 1.7 |
| B1 RRTConnect | tote | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B1 RRTConnect | tote | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | all | all | 24 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | all | easy | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | all | medium | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | all | hard | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | lathe | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | lathe | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | lathe | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | lathe | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | rack | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | rack | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | rack | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | rack | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | tote | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | tote | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | tote | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B2 BIT* | tote | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | all | all | 24 | 21% | 13.8% [3.8, 25.8] | 16.2% | 29.4% | 85.55 | 3.5 |
| B3 RRTConnect + 3 mm margin | all | easy | 8 | 50% | 33.2% [11.3, 57.4] | 36.8% | 30.5% | 85.55 | 3.8 |
| B3 RRTConnect + 3 mm margin | all | medium | 8 | 12% | 8.2% [0.0, 24.6] | 11.7% | 25.0% | 85.55 | 3.5 |
| B3 RRTConnect + 3 mm margin | all | hard | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | lathe | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | lathe | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | lathe | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | lathe | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rack | all | 6 | 50% | 26.3% [7.6, 48.2] | 35.5% | 40.1% | 85.55 | 3.5 |
| B3 RRTConnect + 3 mm margin | rack | easy | 2 | 100% | 46.1% [45.3, 46.9] | 59.8% | 47.7% | 85.55 | 3.5 |
| B3 RRTConnect + 3 mm margin | rack | medium | 2 | 50% | 32.8% [0.0, 65.6] | 46.7% | 25.0% | 85.55 | 3.5 |
| B3 RRTConnect + 3 mm margin | rack | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | tote | all | 6 | 33% | 28.9% [0.0, 60.2] | 29.2% | 13.3% | 85.55 | 5.8 |
| B3 RRTConnect + 3 mm margin | tote | easy | 2 | 100% | 86.7% [79.7, 93.8] | 87.5% | 13.3% | 85.55 | 5.8 |
| B3 RRTConnect + 3 mm margin | tote | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B3 RRTConnect + 3 mm margin | tote | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | all | 24 | 17% | 9.9% [1.9, 19.3] | 12.2% | 35.2% | 85.55 | 3.5 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | easy | 8 | 38% | 21.5% [5.7, 41.6] | 24.9% | 38.5% | 85.55 | 3.5 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | medium | 8 | 12% | 8.2% [0.0, 24.6] | 11.7% | 25.0% | 85.55 | 3.5 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | all | hard | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | lathe | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | all | 6 | 50% | 26.3% [7.6, 48.2] | 35.5% | 40.1% | 85.55 | 3.5 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | easy | 2 | 100% | 46.1% [45.3, 46.9] | 59.8% | 47.7% | 85.56 | 3.6 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | medium | 2 | 50% | 32.8% [0.0, 65.6] | 46.7% | 25.0% | 85.55 | 3.5 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rack | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | all | 6 | 17% | 13.3% [0.0, 39.8] | 13.3% | 20.3% | 85.55 | 3.5 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | easy | 2 | 50% | 39.8% [0.0, 79.7] | 39.8% | 20.3% | 85.55 | 3.5 |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B4 RRTConnect + scenarios (N=8, risk 0.25) | tote | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | all | all | 24 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | all | easy | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | all | medium | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | all | hard | 8 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | lathe | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | lathe | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | lathe | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | lathe | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | rack | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | rack | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | rack | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | rack | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | tote | all | 6 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | tote | easy | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | tote | medium | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B5 BIT* + clearance cost | tote | hard | 2 | 0% | 0.0% [0.0, 0.0] | 0.0% | nan% | nan | nan |
| B6 Keyframe-guided | all | all | 24 | 100% | 67.9% [57.4, 77.9] | 93.1% | 30.4% | 85.54 | 8.0 |
| B6 Keyframe-guided | all | easy | 8 | 100% | 91.8% [85.0, 97.7] | 99.0% | 5.9% | 85.54 | 9.9 |
| B6 Keyframe-guided | all | medium | 8 | 100% | 73.0% [64.5, 82.8] | 92.0% | 25.0% | 85.54 | 9.0 |
| B6 Keyframe-guided | all | hard | 8 | 100% | 38.9% [30.1, 47.3] | 88.2% | 60.4% | 85.54 | 5.0 |
| B6 Keyframe-guided | lathe | all | 6 | 100% | 80.5% [63.0, 96.6] | 100.0% | 19.3% | 85.54 | 10.0 |
| B6 Keyframe-guided | lathe | easy | 2 | 100% | 99.2% [98.4, 100.0] | 100.0% | 0.0% | 85.55 | 20.0 |
| B6 Keyframe-guided | lathe | medium | 2 | 100% | 92.2% [89.1, 95.3] | 100.0% | 7.8% | 85.55 | 10.0 |
| B6 Keyframe-guided | lathe | hard | 2 | 100% | 50.0% [46.9, 53.1] | 100.0% | 50.0% | 85.54 | 4.8 |
| B6 Keyframe-guided | rack | all | 6 | 100% | 57.3% [40.6, 72.4] | 89.6% | 36.7% | 14.56 | 8.0 |
| B6 Keyframe-guided | rack | easy | 2 | 100% | 77.3% [71.9, 82.8] | 100.0% | 15.6% | 7.28 | 8.0 |
| B6 Keyframe-guided | rack | medium | 2 | 100% | 63.3% [59.4, 67.2] | 89.1% | 28.9% | 48.48 | 8.0 |
| B6 Keyframe-guided | rack | hard | 2 | 100% | 31.2% [26.6, 35.9] | 79.7% | 65.6% | 51.63 | 5.0 |
| B6 Keyframe-guided | rebar | all | 6 | 100% | 58.6% [35.7, 80.5] | 86.3% | 41.1% | 85.54 | 9.9 |
| B6 Keyframe-guided | rebar | easy | 2 | 100% | 91.4% [89.1, 93.8] | 95.9% | 7.8% | 46.43 | 15.4 |
| B6 Keyframe-guided | rebar | medium | 2 | 100% | 60.9% [59.4, 62.5] | 89.8% | 39.1% | 85.54 | 9.9 |
| B6 Keyframe-guided | rebar | hard | 2 | 100% | 23.4% [20.3, 26.6] | 73.1% | 76.6% | 85.54 | 4.4 |
| B6 Keyframe-guided | tote | all | 6 | 100% | 75.3% [59.1, 91.4] | 96.4% | 24.5% | 85.54 | 6.5 |
| B6 Keyframe-guided | tote | easy | 2 | 100% | 99.2% [98.4, 100.0] | 100.0% | 0.0% | 85.54 | 8.9 |
| B6 Keyframe-guided | tote | medium | 2 | 100% | 75.8% [68.8, 82.8] | 89.3% | 24.2% | 85.54 | 5.3 |
| B6 Keyframe-guided | tote | hard | 2 | 100% | 50.8% [46.9, 54.7] | 100.0% | 49.2% | 85.55 | 5.0 |
