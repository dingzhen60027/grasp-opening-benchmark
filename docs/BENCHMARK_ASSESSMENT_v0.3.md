# Is gosuite v0.3 suitable for a robust-planning paper?

Assessment, 2026-11-11. The evidence below comes from `python -m gosuite.validate` ([results/suite_v0.3/validation/validation.md](../results/suite_v0.3/validation/validation.md)) and the baseline short test ([results/suite_v0.3/baselines_short_dev/summary.md](../results/suite_v0.3/baselines_short_dev/summary.md)).

## Verdict

**v0.3 is a solid benchmark of *feasibility and sensitivity*, but not yet of *robust planning*.**

The construction, evaluation and protocol hold up: they are reproducible, numerically stable, and their difficulty measure is valid. However, almost all of the achievable risk is fixed by the task definition itself, so no planner can do much better than a planner that passes the declared openings centred and aligned:

- **Headroom:** over all 96 tasks, the best achievable success is **72.8%**, and the scripted reference already reaches **69.1%**. This bound assumes the best final pose anywhere inside the goal tolerance.
- **Guided planning already saturates it:** a nominal keyframe-guided baseline reaches 93.1% success on goal-feasible episodes, against 94.5% for the oracle.
- **Consequence:** a new robust planner could improve on the guided baseline by only a few points. A robust-planning paper needs room to show a gain.

**Follow-up (done): v0.4 implements the recommendation below; see [SUITE_v0.4.md](../SUITE_v0.4.md).** At the hard level, its risk-aware oracle beats the straight one by 25.8 points (13.5 at medium).

**Recommendation: do not start the full experiment on v0.3 for the paper's main claim.** Build v0.4 tasks with planner-controllable risk (Section 4), validate them with the same tools, and then run the baselines. v0.3 remains useful as a calibration and sensitivity track.

## 1. What is solid (keep)

| Check | Result |
|---|---|
| Reproducibility | Regenerating from the master seed gives byte-identical tasks and witnesses (12/12 checked) |
| Geometric fidelity | Measured clearance at the tight features equals the design value (10.0 mm at the medium level) |
| Evaluation step | Halving the joint step (0.0025 → 0.00125 rad) changed **0 of 3,072** episode outcomes (12 tasks × 256) |
| Sensitivity | Success decreases monotonically with the error scale for every dev task (100 → 93 → 69 → 50 → 33 → 12% at ×0 … ×3) |
| Error components | Rotation error dominates: rotation-only gives 72.8% success, translation-only 99.3% |
| Difficulty measure | Tip sensitivity `c / (lever · σ_r)` predicts reference success: ρ = 0.96 overall and 0.96–0.98 within each family |
| Statistical resolution | Between-task SD is 0.4–6.7 points per cell; the episode-level SE at 256 episodes is ≤ 3.1 points |
| Protocol | Common random numbers, a disjoint planning stream, a frozen test split, an independent audit (96/96) and 36 tests |

## 2. What is not solid for a robust-planning claim

**(a) Terminal risk dominates.** For each task, I measured the share of the common evaluation draws for which the **goal state itself** fails: the payload parked at the goal configuration is in contact, outside a keep-in region, outside the tolerance, or badly placed. Every path that ends there loses those episodes.

| Level | Reference success | Fails at the goal state | Fails only along the path | Best achievable (goal-region bound) |
|---|---:|---:|---:|---:|
| easy | 93.4% | 6.3% | 0.3% | 95.4% |
| medium | 76.1% | 20.4% | 3.5% | 81.1% |
| hard | 37.8% | 59.5% | 2.8% | 41.8% |

By family, the gap between the bound and the reference is +9.9 points for the rack (by parking 2–4 mm higher over the saddles), +4.4 for rebar, +0.3 for lathe and 0.0 for tote.

**(b) Centred, aligned passage is optimal.** The openings are closed and symmetric (windows, a bore, cells, guide ears), and the in-hand error is fixed and unobservable during execution. The deviation of each payload point is therefore set by the error and its lever arm, and a path cannot reduce it. The best a planner can do is pass each opening centred and aligned. Any planner given the opening frames does exactly that: the keyframe-guided baseline reaches 93.1% success on goal-feasible episodes, against 94.5% for the oracle.

**(c) Unguided planners compare on feasibility, not robustness.** In 90 s, RRTConnect solved 5/24 dev tasks and BIT\* 0/24. On this suite, a table of standard planners mostly reports who finds a path at all. TAPOM's results show the same pattern.

**(d) The uncertainty model is isotropic and uncalibrated.** Real parallel-jaw grasps mostly pivot about the jaw-closing axis and slip along the part. An isotropic model hides the orientation-dependent risk that a planner could exploit.

The robustness effect itself is real where the task leaves room for it. On paths that RRTConnect did find, a 3 mm margin lowered the episode collision rate (37.2% → 29.4%). The benchmark simply gives a careful nominal planner too little room to fail.

## 3. Baseline selection and short test

The selection follows TAPOM's comparison set plus the uncertainty-aware categories from the survey (docs/BENCHMARK_DESIGN_v0.3.md):

| ID | Baseline | Category | Status |
|---|---|---|---|
| B0 | Scripted reference witness | Oracle / calibration (not a method) | Keep |
| B1 | OMPL RRTConnect | Standard sampling-based planner (TAPOM's strongest classical baseline) | Keep |
| B2 | OMPL BIT\* | Asymptotically optimal planner (TAPOM set) | Optional: 0/24 solved in 90 s |
| B3 | RRTConnect + 3 mm payload margin | Obstacle inflation (TAPOM's own uncertainty treatment) | Keep |
| B4 | RRTConnect + 8 grasp scenarios, risk 0.25 | Chance-constraint / scenario approximation | Keep |
| B5 | BIT\* + clearance-integral cost | Clearance optimisation | Optional: 0/24 solved in 90 s |
| B6 | Keyframe-guided (opening keyframes + RRTConnect transfer) | Structure-guided planning (TAPOM-style guidance) | Keep: strongest nominal baseline |
| — | TAPOM | Frontier method | Not yet adapted (ETH-hosted dependencies) |

Short test on the dev split (`configs/experiment_baselines_short_v0.3.json`; 24 tasks, 1 seed, 90 s, 64 episodes; results in `results/suite_v0.3/baselines_short_dev`):

| Baseline | Planning success | Task success [95% CI over tasks] | Success on goal-feasible episodes | Collision rate given a valid path |
|---|---:|---:|---:|---:|
| B0 Reference (oracle) | 24/24 | 69.1% [58.5, 79.1] | 94.5% | 28.6% |
| B6 Keyframe-guided | 24/24 | 67.9% [57.4, 77.9] | 93.1% | 30.4% |
| B3 RRTConnect + 3 mm margin | 5/24 | 13.8% [3.8, 25.8] | 16.2% | 29.4% |
| B1 RRTConnect | 5/24 | 12.0% [2.5, 22.6] | 14.4% | 37.2% |
| B4 RRTConnect + scenarios | 4/24 | 9.9% [1.9, 19.3] | 12.2% | 35.2% |
| B2 BIT\* | 0/24 | 0% | 0% | — |
| B5 BIT\* + clearance cost | 0/24 | 0% | 0% | — |

No run exceeded its budget. Unguided failures were timeouts or approximate solutions. Planners now spend any time left after finding a path on budget-bounded shortcutting, so the median planning time equals the budget. The time to the first solution is recorded in each run's `planner_info`.

## 4. Recommended v0.4: tasks where planning decisions change the risk

The design levers below are listed in order of expected effect. Each gives a robust planner a decision that a nominal planner gets wrong.

1. **Goals with slack instead of terminal tight fits.** Use placement regions: anywhere along a deck segment between stops; a V-cradle or collet with a capture width; an open pocket. The planner then chooses where and how to finish, and goal-state failure stays small. The tight constraints move into transit.
2. **Asymmetric and one-sided transit constraints.** Examples: protruding neighbouring stock on one side, door edges and column corners, clearance that differs along the payload. The shortest nominal path hugs an obstacle; a lever-arm-aware path keeps clearance where the far end swings.
3. **Confined reorientation.** The payload turns right after passing an opening, as in TAPOM's out-of-plane manoeuvres, e.g. a door narrower than the bar followed by a 90° turn to the chuck axis. The choice of pivot and timing decides the risk.
4. **Alternatives.** Two admissible entry windows or goal slots with different clearance and path length, so risk-aware choice matters.
5. **Anisotropic, literature-grounded grasp error.** A pivot about the jaw-closing axis dominates, plus slip along the part. Square sections get a 90°-roll goal symmetry, so the planner can choose which direction the dominant error moves the tip.

Acceptance criteria for v0.4, measured with the existing `gosuite.validate` tooling before any baseline table is produced:

- goal-state failure ≤ 15% at the hard level (the risk lives in the path);
- goal-region bound − keyframe-guided nominal success ≥ 15 points at medium and hard (room for robust planning);
- goal-region bound − best unguided nominal planner ≥ 25 points;
- the same stability checks as v0.3: step convergence, monotone sensitivity, descriptor validity, reproducibility.

## 5. If you still run v0.3

`configs/experiment_baselines_v0.3.json` runs B1, B3, B4 and B6 on the test split: 5 seeds, 120 s, 256 episodes, 1,440 runs, about 3 hours on 20 workers. B0 is already recorded. Report the feasible-episode success column, which `gosuite.report` now produces. It counts only episodes whose goal state is feasible, so it isolates what each path adds or loses. Present v0.3 as a feasibility and sensitivity benchmark, not as evidence that robust planning pays off.
