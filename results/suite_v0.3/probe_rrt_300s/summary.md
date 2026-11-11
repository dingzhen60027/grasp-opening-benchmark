# Experiment summary

Runs: 12. Task success counts planning failures as 0; CI = 95% bootstrap over tasks. Collision | valid = mean episode collision rate of valid paths.

| Planner | Family | Level | Runs | Planning success | Task success [95% CI] | Collision \| valid | Plan time (s, median) | Min clearance (mm, median) |
|---|---|---|---:|---:|---:|---:|---:|---:|
| RRT-Connect (300 s) | all | all | 12 | 17% | 6.8% [0.0, 16.9] | 59.4% | 217.11 | 1.4 |
| RRT-Connect (300 s) | all | easy | 4 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect (300 s) | all | medium | 4 | 25% | 15.2% [0.0, 45.7] | 39.1% | 158.96 | 2.3 |
| RRT-Connect (300 s) | all | hard | 4 | 25% | 5.1% [0.0, 15.2] | 79.7% | 275.26 | 0.5 |
| RRT-Connect (300 s) | lathe | all | 6 | 33% | 13.5% [0.0, 33.9] | 59.4% | 217.11 | 1.4 |
| RRT-Connect (300 s) | lathe | easy | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect (300 s) | lathe | medium | 2 | 50% | 30.5% [0.0, 60.9] | 39.1% | 158.96 | 2.3 |
| RRT-Connect (300 s) | lathe | hard | 2 | 50% | 10.2% [0.0, 20.3] | 79.7% | 275.26 | 0.5 |
| RRT-Connect (300 s) | rebar | all | 6 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect (300 s) | rebar | easy | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect (300 s) | rebar | medium | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
| RRT-Connect (300 s) | rebar | hard | 2 | 0% | 0.0% [0.0, 0.0] | nan% | nan | nan |
