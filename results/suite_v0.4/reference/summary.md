# Experiment summary

Runs: 192. Task success counts planning failures as 0; CI = 95% bootstrap over tasks. Collision | valid = mean episode collision rate of valid paths. Feasible-episode success: goal-state flags from `../validation/goal_feasibility.json`.

| Planner | Family | Level | Runs | Planning success | Task success [95% CI] | Feasible-episode success | Collision \| valid | Plan time (s, median) | Min clearance (mm, median) |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| Reference (risk-aware oracle) | all | all | 96 | 100% | 91.0% [89.7, 92.2] | 98.0% | 7.7% | 0.00 | 8.0 |
| Reference (risk-aware oracle) | all | easy | 32 | 100% | 96.1% [95.2, 96.9] | 99.5% | 2.2% | 0.00 | 14.0 |
| Reference (risk-aware oracle) | all | medium | 32 | 100% | 93.4% [92.1, 94.7] | 98.0% | 5.4% | 0.00 | 9.6 |
| Reference (risk-aware oracle) | all | hard | 32 | 100% | 83.6% [82.3, 84.9] | 96.6% | 15.3% | 0.00 | 5.9 |
| Reference (risk-aware oracle) | lathe | all | 24 | 100% | 91.2% [88.8, 93.4] | 98.3% | 7.5% | 0.00 | 9.8 |
| Reference (risk-aware oracle) | lathe | easy | 8 | 100% | 95.1% [94.3, 96.0] | 100.0% | 2.6% | 0.00 | 14.0 |
| Reference (risk-aware oracle) | lathe | medium | 8 | 100% | 94.9% [93.6, 95.9] | 99.0% | 3.9% | 0.00 | 9.8 |
| Reference (risk-aware oracle) | lathe | hard | 8 | 100% | 83.7% [82.3, 84.8] | 95.9% | 16.0% | 0.00 | 5.7 |
| Reference (risk-aware oracle) | rack | all | 24 | 100% | 87.9% [85.4, 90.2] | 98.4% | 9.6% | 0.00 | 10.0 |
| Reference (risk-aware oracle) | rack | easy | 8 | 100% | 93.2% [92.1, 94.2] | 99.3% | 4.5% | 0.00 | 14.0 |
| Reference (risk-aware oracle) | rack | medium | 8 | 100% | 90.0% [88.7, 91.3] | 98.7% | 7.5% | 0.00 | 10.0 |
| Reference (risk-aware oracle) | rack | hard | 8 | 100% | 80.5% [77.9, 82.9] | 97.2% | 16.7% | 0.00 | 5.9 |
| Reference (risk-aware oracle) | rebar | all | 24 | 100% | 90.5% [87.9, 93.1] | 96.7% | 8.1% | 0.00 | 10.0 |
| Reference (risk-aware oracle) | rebar | easy | 8 | 100% | 96.6% [95.5, 97.9] | 99.3% | 1.4% | 0.00 | 15.6 |
| Reference (risk-aware oracle) | rebar | medium | 8 | 100% | 91.6% [88.5, 94.2] | 95.3% | 7.3% | 0.00 | 10.0 |
| Reference (risk-aware oracle) | rebar | hard | 8 | 100% | 83.3% [81.0, 85.7] | 95.5% | 15.6% | 0.00 | 6.0 |
| Reference (risk-aware oracle) | tote | all | 24 | 100% | 94.4% [92.0, 96.7] | 98.7% | 5.5% | 0.00 | 8.0 |
| Reference (risk-aware oracle) | tote | easy | 8 | 100% | 99.3% [98.9, 99.7] | 99.5% | 0.5% | 0.00 | 8.0 |
| Reference (risk-aware oracle) | tote | medium | 8 | 100% | 97.2% [96.7, 97.6] | 99.0% | 2.8% | 0.00 | 8.0 |
| Reference (risk-aware oracle) | tote | hard | 8 | 100% | 86.9% [85.5, 88.4] | 97.7% | 13.1% | 0.00 | 5.9 |
| Reference (straight oracle) | all | all | 96 | 100% | 77.2% [73.8, 79.9] | 82.5% | 22.0% | 0.00 | 9.0 |
| Reference (straight oracle) | all | easy | 32 | 100% | 93.7% [92.4, 95.0] | 97.1% | 5.2% | 0.00 | 14.0 |
| Reference (straight oracle) | all | medium | 32 | 100% | 79.9% [77.3, 82.6] | 83.7% | 19.2% | 0.00 | 10.0 |
| Reference (straight oracle) | all | hard | 32 | 100% | 57.8% [55.4, 60.4] | 66.7% | 41.5% | 0.00 | 6.0 |
| Reference (straight oracle) | lathe | all | 24 | 100% | 77.2% [70.6, 83.5] | 82.7% | 22.3% | 0.00 | 10.0 |
| Reference (straight oracle) | lathe | easy | 8 | 100% | 93.8% [92.9, 94.7] | 98.6% | 5.2% | 0.00 | 14.0 |
| Reference (straight oracle) | lathe | medium | 8 | 100% | 81.2% [78.7, 83.7] | 84.7% | 18.5% | 0.00 | 10.0 |
| Reference (straight oracle) | lathe | hard | 8 | 100% | 56.6% [55.4, 57.9] | 64.9% | 43.2% | 0.00 | 6.0 |
| Reference (straight oracle) | rack | all | 24 | 100% | 71.3% [64.6, 77.8] | 79.2% | 26.9% | 0.00 | 10.0 |
| Reference (straight oracle) | rack | easy | 8 | 100% | 90.0% [88.2, 91.9] | 95.9% | 8.1% | 0.00 | 14.0 |
| Reference (straight oracle) | rack | medium | 8 | 100% | 72.9% [70.8, 75.2] | 79.8% | 25.0% | 0.00 | 10.0 |
| Reference (straight oracle) | rack | hard | 8 | 100% | 51.1% [47.8, 53.7] | 61.7% | 47.5% | 0.00 | 6.0 |
| Reference (straight oracle) | rebar | all | 24 | 100% | 74.4% [68.1, 80.6] | 79.0% | 24.4% | 0.00 | 10.0 |
| Reference (straight oracle) | rebar | easy | 8 | 100% | 92.1% [90.7, 93.5] | 94.6% | 6.5% | 0.00 | 16.0 |
| Reference (straight oracle) | rebar | medium | 8 | 100% | 75.5% [74.1, 77.1] | 78.6% | 23.5% | 0.00 | 10.0 |
| Reference (straight oracle) | rebar | hard | 8 | 100% | 55.6% [53.6, 57.6] | 63.7% | 43.1% | 0.00 | 6.0 |
| Reference (straight oracle) | tote | all | 24 | 100% | 85.7% [80.1, 91.1] | 89.1% | 14.2% | 0.00 | 8.0 |
| Reference (straight oracle) | tote | easy | 8 | 100% | 99.0% [98.7, 99.4] | 99.2% | 0.8% | 0.00 | 8.0 |
| Reference (straight oracle) | tote | medium | 8 | 100% | 90.1% [88.1, 92.1] | 91.8% | 9.9% | 0.00 | 8.0 |
| Reference (straight oracle) | tote | hard | 8 | 100% | 67.9% [65.4, 70.1] | 76.3% | 32.1% | 0.00 | 6.0 |
