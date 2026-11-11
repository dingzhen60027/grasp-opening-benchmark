# Experiment summary

Runs: 72. Task success counts planning failures as 0; CI = 95% bootstrap over tasks. Collision | valid = mean episode collision rate of valid paths.

| Planner | Family | Level | Runs | Planning success | Task success [95% CI] | Collision \| valid | Plan time (s, median) | Min clearance (mm, median) |
|---|---|---|---:|---:|---:|---:|---:|---:|
| Margin RRT-Connect (5 mm) | all | all | 24 | 12% | 9.2% [0.0, 20.4] | 12.0% | 28.48 | 8.0 |
| Margin RRT-Connect (5 mm) | all | easy | 8 | 38% | 27.7% [5.7, 54.9] | 12.0% | 28.48 | 8.0 |
| Margin RRT-Connect (5 mm) | all | medium | 8 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | all | hard | 8 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | lathe | all | 6 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | lathe | easy | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | lathe | medium | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | lathe | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | rack | all | 6 | 17% | 7.6% [0.0, 22.7] | 12.5% | 23.38 | 8.0 |
| Margin RRT-Connect (5 mm) | rack | easy | 2 | 50% | 22.7% [0.0, 45.3] | 12.5% | 23.38 | 8.0 |
| Margin RRT-Connect (5 mm) | rack | medium | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | rack | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | tote | all | 6 | 33% | 29.4% [0.0, 59.6] | 11.7% | 30.50 | 6.8 |
| Margin RRT-Connect (5 mm) | tote | easy | 2 | 100% | 88.3% [85.9, 90.6] | 11.7% | 30.50 | 6.8 |
| Margin RRT-Connect (5 mm) | tote | medium | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Margin RRT-Connect (5 mm) | tote | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | all | all | 24 | 17% | 9.0% [1.8, 17.5] | 41.8% | 25.31 | 3.5 |
| RRT-Connect | all | easy | 8 | 38% | 20.1% [5.5, 38.1] | 42.7% | 28.65 | 2.4 |
| RRT-Connect | all | medium | 8 | 12% | 6.8% [0.0, 20.5] | 39.1% | 21.96 | 4.5 |
| RRT-Connect | all | hard | 8 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | lathe | all | 6 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | lathe | easy | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | lathe | medium | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | lathe | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | rack | all | 6 | 33% | 20.3% [0.0, 42.7] | 30.5% | 19.31 | 6.7 |
| RRT-Connect | rack | easy | 2 | 50% | 33.6% [0.0, 67.2] | 21.9% | 16.66 | 9.0 |
| RRT-Connect | rack | medium | 2 | 50% | 27.3% [0.0, 54.7] | 39.1% | 21.96 | 4.5 |
| RRT-Connect | rack | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | tote | all | 6 | 33% | 15.6% [0.0, 32.3] | 53.1% | 31.31 | 2.0 |
| RRT-Connect | tote | easy | 2 | 100% | 46.9% [43.8, 50.0] | 53.1% | 31.31 | 2.0 |
| RRT-Connect | tote | medium | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect | tote | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | all | all | 24 | 17% | 11.3% [2.3, 21.7] | 27.0% | 26.42 | 5.2 |
| Scenario RRT-Connect (N=8, risk 0.25) | all | easy | 8 | 38% | 27.0% [8.0, 50.0] | 22.9% | 24.67 | 5.6 |
| Scenario RRT-Connect (N=8, risk 0.25) | all | medium | 8 | 12% | 6.8% [0.0, 20.5] | 39.1% | 29.39 | 4.7 |
| Scenario RRT-Connect (N=8, risk 0.25) | all | hard | 8 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | lathe | all | 6 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | lathe | easy | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | lathe | medium | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | lathe | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | rack | all | 6 | 50% | 31.2% [9.1, 54.9] | 30.2% | 28.16 | 4.7 |
| Scenario RRT-Connect (N=8, risk 0.25) | rack | easy | 2 | 100% | 66.4% [64.1, 68.8] | 25.8% | 24.07 | 4.6 |
| Scenario RRT-Connect (N=8, risk 0.25) | rack | medium | 2 | 50% | 27.3% [0.0, 54.7] | 39.1% | 29.39 | 4.7 |
| Scenario RRT-Connect (N=8, risk 0.25) | rack | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | tote | all | 6 | 17% | 13.8% [0.0, 41.4] | 17.2% | 24.67 | 5.9 |
| Scenario RRT-Connect (N=8, risk 0.25) | tote | easy | 2 | 50% | 41.4% [0.0, 82.8] | 17.2% | 24.67 | 5.9 |
| Scenario RRT-Connect (N=8, risk 0.25) | tote | medium | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| Scenario RRT-Connect (N=8, risk 0.25) | tote | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
