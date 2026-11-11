# Benchmark design v0.3: multi-environment suite

Status: **implemented 2026-11-11** as `gosuite` (see [SUITE_v0.3.md](../SUITE_v0.3.md)); baselines not yet run. This file keeps the rationale and its mapping to related benchmarks.

Implementation notes (where the build differs from or refines the plan below):

- **F5 not built.** F5 (`infeed`) was not implemented; the suite has F1-F4 (96 tasks).
- **Lathe door.** The lathe uses a realistic tall door (0.42-0.50 m). The bar enters broadside with the wrist reaching inside, and the chuck bore is the level-controlled feature.
- **Difficulty isolation.** Witnesses must keep at least 15 mm from everything except the family's designed tight features, so incidental near-misses do not blur the levels.
- **Fixed rack placement tolerance.** The rack's 2-20 mm placement tolerance is fixed across levels, so the easy rack level still contains a placement challenge.
- **Keep-in during planning.** Keep-in rules are per-state constraints and are exposed to planners through `PlanningProblem.check`.
- **Reference calibration.** On all 96 tasks, reference-witness success is 93.4% / 76.1% / 37.8% for easy / medium / hard.

## 1. Why v0.2 is not enough

- **One environment family.** The rack suite has 3 geometries and 6 directional tasks. Retrieval tasks reverse the storage tasks, so they are not independent samples.
- **A task-validity bug.** All three v0.2 reference storage paths push the tube 0.12–0.22 m into the outer bay (y = −0.47) and then slide it sideways over the dividers into the target bay. `rackbench/evaluate.py` checks the bay only at the final pose, so these paths pass. Some of the recorded uncertainty collisions come from this wrong-bay sweep rather than from the intended insertion: 3/39, 1/37 and 12/40 hit `bin_divider_-0.4`.
- **The central bay cannot be entered straight at the v0.2 rack placement.** With the tool horizontal directly in front of the base, the UR5e + Robotiq TCP can only reach x ≈ 0.42–1.06 m. A straight insertion of a 0.50 m tube needs about 0.54 m of travel starting at least 0.505 m before the rack front. That only fits if the rack front is at x ≥ ≈0.93 m; v0.2 uses 0.75 m.
- **The facade check uses hard-coded limits** (`.527/.169/.471`). These only fit the v0.2 rack.

The v0.2 data stays as a historical snapshot. It is not the benchmark the paper will report.

## 2. What related work does, and what we take from it

| Source | How they build the benchmark | What we adopt | Gap we fill |
|---|---|---|---|
| **TAPOM** (Li et al., 2025, [arXiv:2511.05052](https://arxiv.org/abs/2511.05052)). The closest work: elongated objects through narrow passages. | 5 hand-built application scenes: Part Delivery through industrial shelving (PD), Ceiling Installation (CI), Rebar Assembly (RA), Rescue, Beam Transportation (BT). UR5e or ABB IRB 4600. 10 trials per scene, 600 s budget. OMPL baselines (RRTConnect, EST, STRIDE, BFMT*, PRM, LazyRRT, BIT*, EIT*) plus cuRobo. Metrics: success rate, mean time, mean successful time. | Application-motivated scene families with an explicit analogue to PD and RA. The same baseline categories (RRTConnect as strongest classical baseline; optimizing planners; "inflate obstacles" uncertainty handling). | One instance per scene; no clearance numbers or difficulty control; no held-out instances. Uncertainty is handled only by enlarging collision volumes. The real-robot run reports a 0.187 rad rotation disturbance of the grasped object, but execution error is never evaluated. |
| **MotionBenchMaker** (Chamzas et al., RA-L 2022, [arXiv:2112.06402](https://arxiv.org/abs/2112.06402)) | Scene templates with procedural pose noise (Gaussian or uniform). Start/goal from collision-aware IK. Each problem is filtered for solvability by a tuned planner (60 s). 8 scenes × 5 robots, 100 problems each. They show planner rankings reverse when only 5–10 problems are used. | Template + sampled parameters, a solvability witness for every instance, and enough instances per family that rankings are stable. We will report rank stability against instance count. | No uncertainty; no difficulty measure. |
| **Gaebert et al.** (ICRA 2023, [arXiv:2302.14422](https://arxiv.org/abs/2302.14422)) | 20 manipulator environments, 214 narrow-passage problems. Parameter tuning on 17 environments, evaluation on 3 held-out environments. | Split by geometry, not by query: tuning only on dev geometries, then a frozen test set. | Difficulty described only in words. |
| **BARN** (Perille et al., SSRR 2020, [arXiv:2008.13315](https://arxiv.org/abs/2008.13315)) | 300 procedurally generated environments, each checked solvable (flood fill + A*). Difficulty metrics computed along the path (e.g. distance to the closest obstacle), validated with 3000 planner trials. A hidden test set for the challenge. | Per-instance difficulty descriptors computed on the witness path, validated against measured baseline outcomes. | Mobile robots, not manipulation. |
| **MπNets** (Fishman et al., CoRL 2022, [arXiv:2210.12209](https://arxiv.org/abs/2210.12209)) | Scene categories (tabletop, dresser, cubby), procedurally generated; test problems grouped by which expert planner solves them. | Report results per family and per "solved-by" subset. | — |
| **Berenson et al.** (IROS 2009, TSR under pose uncertainty); **p-Chekov** ([arXiv:1811.03073](https://arxiv.org/abs/1811.03073)); scenario chance constraints ([arXiv:2209.06936](https://arxiv.org/abs/2209.06936)) | Plans must be valid for all pose hypotheses, or collision probability is estimated by sampling. | An uncertainty-aware baseline that is valid for N sampled grasp errors (scenario / particle style), evaluated with Monte Carlo. | None of these targets in-hand grasp error of a long payload in a narrow passage. |

## 3. Problem definition (unchanged core, stated precisely)

A UR5e + Robotiq 2F-85 holds a rigid elongated payload with nominal grasp `T_EO`. During execution the actual grasp is

`T_WO(q, ξ) = T_WE(q) · [Exp_SO3(φ), δt] · T_EO`, with `ξ = (δt, φ) ~ N(0, diag(σ_t², σ_r²))`

and ξ is drawn **once per episode** and held for the whole trajectory. A planner receives the task JSON (scene, start configuration, goal region, nominal grasp, σ). It never receives the reference witness. It returns a joint path. The path is executed kinematically under M sampled grasp errors that are shared across all methods (common random numbers) and disjoint from any samples the planner used.

## 4. Environment families

Every family is a parameterised template. Each instance is sampled from the template and accepted only if a scripted Cartesian witness passes the full nominal validation (Section 6). Clearance level controls the tightest feature: **easy 20 mm, medium 10 mm, hard 5 mm per side**. With a 0.45 m lever arm, 1° of grasp rotation moves the payload tip about 7.9 mm, so "hard" is below one standard deviation of tip displacement.

| ID | Family | Application analogue | Tightest feature | What makes it different |
|---|---|---|---|---|
| F1 | `rack` | Part delivery into industrial shelving (TAPOM PD) | Guide ears at the destination bay | Several identical bays: the payload must enter **only** the target bay. Placement over two saddles. |
| F2 | `rebar` | Bar insertion into a rebar cage (TAPOM RA, UR5e scale) | 2–4 coaxial stirrup windows | Threading through several aligned openings; angular error accumulates with depth. |
| F3 | `lathe` | CNC machine tending: load a bar into a chuck | Chuck bore (2-D clearance) | Enter through the machine door, rotate the bar 90° inside the enclosure, then insert axially into the bore. |
| F4 | `tote` | Kitting: insert a rod into a compartment of a divided tote | Compartment walls | Vertical, top-down insertion among neighbouring parts; error about the horizontal axes moves the tip. |
| F5 (stretch) | `infeed` | Feed a bar through a guard tunnel into a downstream machine | Yawed tunnel | Non-axis-aligned approach and a long constrained segment ending at a stop. |

Sampled parameters per family (each instance records all of them):

- **F1 rack:** target bay (left/centre/right); tube length 0.38–0.50 m; rear overhang 0.02–0.05 m; rack front distance, sampled inside the reach window; guide clearance by level; neighbouring stock present in the other bays.
- **F2 rebar:** number of stirrups 2–4; spacing 0.08–0.15 m; cage yaw ±20°; cage height; target channel position; bar length 0.40–0.55 m; window clearance by level; insertion depth.
- **F3 lathe:** door width/height margin; enclosure depth; chuck position and facing direction (±y of the door frame); bar length 0.20–0.30 m; bore clearance by level; insertion depth 0.03–0.05 m.
- **F4 tote:** grid size (2×3 to 3×4); target cell; tote yaw; occupied neighbouring cells; rod length 0.22–0.32 m; compartment clearance by level; insertion depth.

All families share the same cell (floor, robot pedestal, base plate) and a source fixture from which the payload starts already grasped. Hollow payloads stay as unions of walls, not convex hulls.

## 5. Task semantics (generic, declared in each task file)

Success is decided from the payload pose sequence only, using rules the task file declares. Nothing about a specific geometry is hard-coded in the evaluator:

1. **Collision-free:** robot–environment, payload–environment, robot self-collision and payload–robot (only the two fingertip links may touch the payload).
2. **Keep-in rules** `(restricted box R, allowed box A)`: any payload sample point inside R must also lie inside A. This fixes the v0.2 bug. Example: R = rack interior, A = target-bay corridor. Payload sample points are the corners plus points every 10 mm along the long edges.
3. **Portals:** an ordered list of opening rectangles that the payload must cross through in order. This rules out going around a wall.
4. **Goal region:** payload position and orientation tolerance around the nominal goal pose. `q_goal` is only a reference IK solution.
5. **Placement (optional):** support coverage and pre-release gap (2–20 mm) above declared supports.

## 6. Instance generation and validation

- Seeded sampling per (family, level, index), with rejection sampling. The rejection log is saved with its reasons, as v0.1 already does.
- Witness: scripted EE knots, IK continuation from several seed configurations, densified to 0.005 rad, then audited at 0.0025 rad with the same validator the evaluator uses (collision + keep-in + portals + goal + placement).
- Every accepted task stores its parameters, scene SHA-256, witness SHA-256 and difficulty descriptors.
- No instance is filtered on uncertain outcomes or on planner success, to avoid selection bias.

## 7. Splits and size

- Per family and level: **2 dev + 6 test geometries**, giving 24 tasks per family and 96 tasks for F1–F4 (120 with F5).
- Dev is used for tuning baseline parameters (margins, scenario count, step size). Test is frozen by hash before any baseline is tuned, as in Gaebert et al. and BARN.
- Reverse (retrieve) tasks are optional extras and never counted as independent geometries.
- The v0.1 five-family diagnostic set is kept as a separate **diagnostic track** for controlled ablations.

## 8. Difficulty descriptors (per instance, BARN-style)

Computed on the witness path and saved in the manifest:

- minimum nominal payload clearance;
- lever arm (maximum TCP-to-payload-extremity distance);
- constrained length (path length with payload clearance < 20 mm);
- number of portals;
- reorientation inside the constraint;
- tip sensitivity = clearance / (lever × σ_r).

These are validated by correlating them with measured baseline collision rates and success rates.

## 9. Evaluation protocol

- **Uncertainty:** default σ_t = 2 mm and σ_r = 1° per axis. These are **not calibrated** and are stated as such. Scale sweep {0, 0.5, 1, 1.5, 2}; translation-only and rotation-only ablations.
- **Episodes:** 256 per (task, method, seed) for the main tables (95% Clopper–Pearson half-width ≤ about 6%); 1024 for the headline subset.
- **Planning:** 5 seeds per task; budget 120 s for the main table (raised from the planned 60 s after the smoke run), plus a 600 s run on the hard subset for comparability with TAPOM.
- **Metrics:**
  - planning success, time to first solution, joint and EE path length;
  - execution collision rate, and task success (planning failure counts as failure);
  - minimum nominal clearance.
- **Aggregation:** per family and level; mean over instances with bootstrap CIs over instances; rank stability against instance count (MotionBenchMaker).
- **Output:** per-run JSON plus a flat CSV that can be loaded into Planner-Arena-style plots.

## 10. Baselines (run after the benchmark is frozen)

| ID | Method | Role |
|---|---|---|
| B0 | Reference witness (scripted Cartesian) | Feasibility reference, not a method |
| B1 | RRT-Connect + shortcut smoothing | TAPOM's strongest classical baseline |
| B2 | Optimizing / other sampling planners (BIT*, EIT*, PRM via OMPL if the Python bindings install cleanly) | Matches TAPOM's comparison set |
| B3 | Inflated-margin RRT-Connect (payload margin δ tuned on dev) | TAPOM's own uncertainty treatment |
| B4 | Scenario RRT-Connect: every state valid for N planning-time grasp samples | Chance-constraint / all-hypotheses style (Berenson; scenario approach) |
| B5 | Clearance-maximizing trajectory optimization (STOMP/TrajOpt-style) on B1 paths | Optimization-based robustness |
| F | TAPOM adapted to these tasks | Frontier comparison, if its dependencies can be satisfied |

## 11. Build order

1. Generic core: task-file schema v0.3, world, the generic success checker (keep-in, portals, goal, placement) and evaluator. Tests, including one where a wrong-bay path must fail.
2. F1 rack, rebuilt with reach-aware placement and in-bay-only insertion.
3. F2 rebar, F3 lathe, F4 tote, one at a time, each with tests and preview images.
4. Freeze: generate all splits, audit every witness, compute descriptors, record hashes, render previews.
5. Witness uncertainty evaluation (B0) to confirm the difficulty levels separate.
6. Baselines B1–B5, then the frontier comparison.
