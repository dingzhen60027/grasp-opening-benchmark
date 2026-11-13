# Changelog

## v0.4.1-dev — 2026-11-12 (proposed method)

- **CRISP** (`gosuite/planners/crisp.py`, registry name `crisp`) is the proposed method: chance-optimal insertion under in-hand grasp error. It keeps the keyframe structure of the openings and optimises the insertion for the trajectory-level success probability. Its components:
  - a task-space front/rear-end deviation parameterisation;
  - CMA-ES with a goal-first stage and a free goal pose inside the goal region;
  - adaptive pre-entry and shrinkage candidates for arm realisation;
  - a Clopper-Pearson risk certificate on fresh planning draws.

  It reads only the planner interface, which is tested.
- `gosuite/risk.py`: an exact, vectorised grasp-error risk model.
  - Separating-axis margins between oriented boxes; keep-in rules become forbidden boxes; final-state rule margins.
  - Lossless pruning by the lever-arm displacement bound.
  - Validated against the simulator: zero missed contacts in 1.26 M checks; conservative disagreements under 0.5 mm.
- `gosuite/planners/ccvpsto.py`: CC-VPSTO (Brudermueller et al., IJRR 2026) re-implemented as a comparator. Its exact binomial padding reproduces the published thresholds.
- Experiments:
  - development on the dev split (`configs/experiment_crisp_dev_v0.4.json`);
  - one run on the frozen test split with code hashes (`configs/experiment_crisp_v0.4.json`, `results/suite_v0.4/crisp`, 2016 runs);
  - an anytime and sample-size study (`configs/experiment_crisp_budget_v0.4.json`);
  - a misspecification study (`gosuite.robustness`, `configs/robustness_crisp_v0.4.json`).
- Results on test: CRISP reaches 92.2% task success.
  - Against B6 (75.1%): +17.1 points, p = 8e-12.
  - Against CC-VPSTO (83.9%): +8.4 points, p = 5e-11.
  - Against the privileged risk-aware oracle (91.0%): +1.3 points, p = 0.029.

  Ablations: removing rotation from the error model costs 11.6 points, a max-clearance objective 3.0, a fixed goal 2.0. State-wise and linearised-Boole risk plan equally well, and CVaR is 0.7 points better (not selected on dev).
- Harness: per-planner `seeds` and `budget_seconds` overrides; `gosuite.compare` reports two-sided Wilcoxon signed-rank p-values with Holm adjustment and accepts a seed filter; figures (`gosuite.crisp_figures`) and LaTeX tables (`gosuite.paper_tables`).
- Documentation: `docs/CRISP.md` (method, theory, related work, development log, results, limitations); `paper/` (manuscript draft, references, generated tables).

## v0.4.0-dev — 2026-11-12

- Validation of v0.3 (`gosuite.validate`): reproducible and numerically stable, with a valid difficulty measure, but 93% of reference failures happen at the goal state. The best achievable success was 72.8% against 69.1% for the oracle, leaving too little room to show robust planning (docs/BENCHMARK_ASSESSMENT_v0.3.md).
- New v0.4 suite (`SUITE_v0.4.md`, `data/suite_v0.4`): the same four applications, rebuilt so that the risk is planner-controllable. Level-controlled obstacles near the opening are passed by the payload tip (largest lever arm); goals keep 25–45 mm slack.
- Two oracle witnesses per task: a risk-aware one (cross-entropy search on planning-stream draws, `gosuite/robust.py`) and a straight one. Their gap on the evaluation draws measures the headroom.
- Baselines: OMPL adapter (RRTConnect, BIT\*, ...; margin, scenario and clearance-cost options), a keyframe-guided planner and deadline-bounded shortcutting. The report gains a feasible-episode (path-attributable) success column.
- Dependencies: `ompl==1.7.0` (prebuilt wheels, no build step) is now pinned in `requirements.txt` and `requirements-lock.txt`; the OMPL and keyframe-guided baselines need it. The quick start creates the environment with `python3.10`, because the pinned scipy and pybullet have no Python 3.12 wheels.
- Full baseline experiment on the v0.4 test split (`configs/experiment_baselines_v0.4.json`: 72 tasks, four baselines x 5 seeds, 120 s, 256 common draws; `results/suite_v0.4/baselines`) and `gosuite.compare`, which builds the paired main table (`configs/comparison_v0.4.json` -> `results/suite_v0.4/comparison_test`). The keyframe-guided baseline reaches 75.1% against 77.3% for the straight oracle; the risk-aware oracle is 14.2 / 27.0 points above it at medium / hard (SUITE_v0.4.md, Section 5).
- Bug fix: `gosuite.report` fell back to `results/suite_v0.3/validation/goal_feasibility.json` when a results tree had no validation output yet. Task ids repeat across suite versions, so the v0.4 reference and short-test summaries computed feasible-episode success with v0.3 flags (guided baseline 94.0% instead of 83.2%). The fallback is removed, each summary names the flag file it used, and every report was regenerated: only that column changed, and older v0.3 reports gain it. `tools/check_evidence.py` now recomputes the column from the recorded episodes.
- The validation report generator (headroom, step convergence, sensitivity, descriptors, variance) runs on both suites.
- Results (validation): at the hard level the risk-aware oracle beats the straight one by 25.8 points (13.5 at medium); goal-state failure at hard is 13.4%. The keyframe-guided baseline matches the straight oracle (about 77%).
- Reproducibility: v0.4 regenerates byte-identically. The v0.3 data was produced by the generator at commit 3c8dba0; the current generator adds metadata fields, so regenerate v0.3 from that commit for byte-identical files.

## v0.3.0-dev — 2026-11-11

- New multi-environment suite `gosuite` (`SUITE_v0.3.md`): four application families (rack bay delivery, rebar cage threading, lathe chuck loading, kitting tote insertion) x three clearance levels (20 / 10 / 5 mm per side) x 2 dev + 6 test geometries = 96 tasks.
- Generic task rules declared per task file: per-state keep-in regions (exact segment clipping), ordered portals, payload goal region, placement. No geometry-specific constants in the evaluator.
- **Bug fix (v0.2):** the rack reference paths entered the outer bay and slid sideways over the dividers into the target bay; the old success check looked at the bay only at the final pose. The v0.3 keep-in rule rejects such paths, and the rebuilt rack is placed within the UR5e reach window so that straight in-bay insertion is feasible for every bay. The v0.2 data and results are kept only as a historical snapshot.
- Quality control: exact layout-conflict check, rejection of witnesses that pass within 15 mm of non-designed features, independent audit (96/96), per-task difficulty descriptors.
- Evaluation: common random numbers across methods, a separate planning-sample stream, unconditional task success, bootstrap CIs over tasks.
- Experiment harness: planner plug-in interface (`PlanningProblem` / `PlanResult`), parallel resumable runner, report and figure generators; built-in reference oracle and RRT-Connect (nominal, margin, scenario variants); custom-planner template.
- Reference-witness evaluation of all 96 tasks (difficulty calibration) and simulator screenshots (`previews/suite_v0.3`).

## v0.2.0-dev — 2026-11-10

- First public development snapshot.
- Warehouse / line-side short-stock rack application: three geometries and paired storage/retrieval tasks.
- Manufacturer UR5e / Robotiq models, hollow payload geometry, physical rack structures and neighboring inventory.
- Fixed grasp-error evaluator, object goal region, support checks, open-hand exit audit and an RRT-Connect adapter.
- Nominal checks, 128-episode development evaluations, raw outcomes, simulator previews and regression tests.
- Explicit research status: no new robust planner, held-out rack test set or real-robot validation yet.

## v0.1 — retained diagnostic work

- Five simplified geometric families and 60 nominally feasible tasks.
- Controlled grasp-error experiments and nominal planning / fixed-detour examples.
- Kept as diagnostics rather than application evidence.
