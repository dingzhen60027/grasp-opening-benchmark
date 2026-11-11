# Changelog

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
