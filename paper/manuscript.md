# Chance-Optimal Insertion Planning for Elongated Payloads under In-Hand Grasp Uncertainty

*Draft manuscript (MDPI article structure). Every number is generated from the committed run records; each table and figure names its source. Author list, affiliations, funding and data-availability statements are to be completed by the authors. Citation keys refer to `paper/references.bib`.*

## Abstract

Industrial manipulators often have to pass long, grasped workpieces, such as tubes, bars and rods, through narrow openings. The grasp leaves an unknown error in the part's pose in the hand. That error is fixed for the whole motion, and its rotational component is amplified along the part. We formulate insertion planning as maximising the probability that the whole execution succeeds under a static SE(3) grasp error, and propose CRISP, a planner that optimises this probability directly. CRISP combines:
- keyframes derived from the declared openings;
- a task-space parameterisation of the offsets of the payload's front and rear ends;
- an exact, vectorised separating-axis risk model with lossless pruning (5–40 ms per candidate);
- CMA-ES with a free goal pose inside the goal region;
- a Clopper–Pearson risk certificate.

We also release gosuite v0.4, a benchmark of 96 tasks in four industrial application families with three clearance levels and a frozen test split. Every method is evaluated on the same 256 grasp errors per task.

On the 72 test tasks, CRISP reaches 92.2% task success [95% CI 91.0, 93.3]. That is 17.1 points above a keyframe-guided baseline and 8.4 points above a re-implementation of the recent chance-constrained optimiser CC-VPSTO (Holm-adjusted Wilcoxon p < 10⁻¹⁰), and 1.3 points above a risk-aware oracle that uses privileged scene knowledge (p = 0.029). At the hardest level CRISP reaches 88.6%, against 70.0% for CC-VPSTO and 56.6% for the baseline.

Ablations attribute the gain to three components: modelling the rotational error (11.6 points), a probabilistic rather than clearance objective (3.0 points), and the free goal pose (2.0 points). State-wise and Boole-bounded risk give equally good plans, but they misstate the risk of the planned path by 0.8× and 100×; CRISP's certificate is calibrated (mean predicted 8.0% against 7.8% observed). With a quarter of the planning budget, CRISP still outperforms CC-VPSTO with the full budget. Re-executed under errors from half to twice the planned magnitude, CRISP keeps its lead over both competitors (p ≤ 10⁻⁶ at every scale). Errors of rotation alone reproduce most of the baseline's failures (77.6% success), while CRISP keeps 94.8%.

**Keywords:** motion planning under uncertainty; chance constraints; grasp pose uncertainty; robotic insertion; elongated objects; benchmark; sample average approximation

## 1. Introduction

Handling long parts is routine in manufacturing and construction: bars and tubes are stored in racks, rebar is threaded into cages, bar stock is loaded into machine tools, and rods are placed in kitting totes. Each of these tasks ends with an insertion through an opening only a few millimetres wider than the part. A grasp is never exact. Fixturing, vision and finger contact all leave an in-hand error of a few millimetres and around a degree [hsiao2011robust]. For an elongated part this error has a specific structure: it is drawn once and then held for the whole motion. A point at distance ℓ from the grasp moves by roughly |δt| + |φ|ℓ, so a 1° rotation becomes about 8 mm at the tip of a 0.45 m part.

Classical planners treat the problem deterministically. They find a nominally collision-free path, and perhaps keep a uniform safety margin. Structure-aware planners for elongated objects, such as topology-guided keyframes [li2025tapom], solve the geometric part of the problem efficiently but do not reason about the in-hand error. Chance-constrained planners do reason about uncertainty [blackmore2011chance, luders2010ccrrt, dai2019pchekov, dawson2023scora, brudermuller2026ccvpsto]. They were, however, mostly designed for motion or obstacle noise that varies over time, and they typically bound the risk of each state separately or linearise it. For a static error the failure event of a path is a union over its states. These events are strongly correlated, and the risk depends on the lever arm.

This paper studies the problem with a reproducible benchmark and a planner designed for it. Our contributions are:

1. **Formulation and analysis.** We formulate grasp-error-robust insertion as maximising the trajectory-level success probability under a static SE(3) error. We characterise the gap between this probability and its state-wise lower and Boole upper bounds (Proposition 1). We then show on the benchmark that the per-state quantities misstate the risk of planned paths, while the trajectory-level estimate is calibrated.
2. **CRISP, a chance-optimal insertion planner.** It combines:
   - keyframes derived from the declared openings;
   - a task-space parameterisation of the payload's front and rear ends that makes the lever arm explicit;
   - an exact, vectorised risk model on box geometry with lossless pruning;
   - CMA-ES optimisation with a free goal pose inside the goal region;
   - an a-posteriori Clopper–Pearson risk certificate.
3. **Benchmark, protocol and evidence.** gosuite v0.4 has 96 tasks in four application families. Each task carries two oracle witnesses, and evaluation uses common random numbers. The protocol was fixed in advance: development on a dev split, a single run on a frozen test split, and paired statistics with multiple-comparison correction. The evidence covers a re-implemented recent comparator (CC-VPSTO [brudermuller2026ccvpsto]), six ablations, an anytime and sample-size study, and a study of error-model misspecification.

## 2. Related Work

**Chance-constrained motion planning.** Blackmore et al. [blackmore2011chance] and Ono and Williams [ono2008iterative] bound a joint chance constraint by allocating risk over time steps with Boole's inequality. CC-RRT [luders2010ccrrt] and CC-RRT* [luders2013ccrrtstar] embed per-step chance constraints in sampling-based planners. p-Chekov [dai2019pchekov] and SCORA [dawson2023scora] bring risk allocation to high-DOF manipulators, through quadrature and through linearised risk estimates respectively. These methods target motion or obstacle uncertainty. Under a static in-hand error, Boole allocation over T states can be conservative by up to a factor T (Section 3).

**Trajectory-level sampled risk.** The scenario-based MPC of de Groot et al. [degroot2025scenario] explicitly bounds the joint collision probability over the planning horizon. CC-VPSTO [brudermuller2026ccvpsto] estimates a trajectory-wise chance constraint by Monte Carlo, pads its threshold with an exact binomial test, and optimises joint-space via-points [jankowski2023vpsto] with CMA-ES. Lew et al. [lew2023riskaverse] optimise a conditional value-at-risk by sample average approximation [shapiro2014lectures]. CRISP belongs to this family. It adds:
- exact SE(3) propagation of a static in-hand error;
- an exact geometric risk model fast enough to optimise with;
- a lever-arm-aware task-space parameterisation and a free goal pose.

We re-implement CC-VPSTO as a comparator and evaluate CVaR as a variant.

**Probabilistic collision checking under pose uncertainty.** Most bounds assume Gaussian positional errors. Wang et al. [wang2025pcd] handle SE(3) pose errors with superquadric models. Schmerling and Pavone [schmerling2017evaluating] estimate trajectory collision probabilities by adaptive importance sampling. Sun et al. [sun2013safe] minimise the collision probability of imprecise manipulators. Quintero-Peña et al. [quinteropena2024stochastic] learn stochastic signed distance functions. Risk contours [jasour2021risk] give continuous-time guarantees for polynomial models. Our setting needs the trajectory-level risk of thousands of candidates per second under exact box geometry, which motivates the model of Section 4.2.

**Parametric uncertainty and sensitivity.** A static grasp error is a parametric uncertainty. Sensitivity-aware planning [brault2021robust, srour2024sensitivity] minimises the sensitivity of states to parameters, including payload parameters. CRISP instead maximises the success probability directly. In our tasks the sensitivity of a payload point is fixed by its lever arm, so what can be shaped is the relation between clearance and lever arm along the path.

**Elongated objects and fine motion.** TAPOM [li2025tapom] derives keyframes from task-space topology to pass elongated objects through narrow passages; our keyframe structure follows the same idea. Task-space regions have been adapted to pose uncertainty [berenson2009pose], and fine-motion planning under uncertainty goes back to preimages [lozanoperez1984fine]. Reducing the in-hand error by sensing or compliance is complementary to open-loop risk-aware planning.

## 3. Problem Formulation

A UR5e arm with a Robotiq 2F-85 gripper holds a box-shaped payload of length L ≈ 0.3–0.45 m, with nominal tool-to-payload transform T_EO. The execution grasp is Δ(ξ) T_EO, where Δ(ξ) = [Exp(φ), δt] and ξ = (δt, φ) ~ N(0, Σ), Σ = diag(σ_t² I₃, σ_r² I₃), with σ_t = 2 mm and σ_r = 1°. The error is drawn once per execution. A joint path π induces the payload poses P_t(π, ξ) = T_WE(q_t) Δ(ξ) T_EO, t = 1, …, T. The execution succeeds if four conditions hold:

1. no payload pose touches an obstacle or an arm link;
2. the payload stays inside the task's keep-in regions;
3. it crosses the declared openings in order;
4. the final pose meets the goal region (position and orientation tolerance) and the placement rule.

We seek

  π* = argmax_π p(π),  p(π) = Pr_ξ[ success(π, ξ) ].   (1)

**Proposition 1.** Let F_t(π) = {ξ : state t fails}. Then F(π) = ∪_t F_t(π), and

  max_t Pr[F_t] ≤ Pr[F] ≤ min(1, Σ_t Pr[F_t]).   (2)

A state-wise chance constraint controls the left-hand side, so it can accept paths whose true risk is up to T times larger. Risk allocation controls the right-hand side, so it can be conservative by up to a factor T. For a static error, the sets F_t of consecutive states overlap heavily. On the straight insertions of the 24 dev tasks (about 105 states, 1000 planning samples), the trajectory-level risk is 1.2–1.55 times the state-wise maximum, and the Boole sum is 22–43 times the trajectory-level risk (Figure 2).

**Lemma 2 (lever arm).** For a payload point p in the tool frame, |Exp(φ)p + δt − p| ≤ |δt| + 2 sin(|φ|/2)|p| ≤ |δt| + |φ||p|. To first order, the displacement of p along a unit normal n has variance σ_t² + σ_r²|n × p|². A point 0.45 m from the grasp therefore has a lateral standard deviation of about 8 mm, against 2 mm at the grasp.

## 4. Method: CRISP

### 4.1 Structure

Payload keyframes are built backwards from the goal pose. For each declared opening, from last to first, the payload is retracted along the opening's axis until its leading end is 6 cm before the opening plane. The keyframes keep the goal orientation, which gives a straight reference insertion P₀(s), s ∈ [0, 1]. A joint-space transfer from the start to the pre-entry configuration is planned with RRT-Connect [kuffner2000rrtconnect, sucan2012ompl]. This structure is shared with the keyframe-guided baseline and the comparator.

### 4.2 Exact trajectory-level risk model

Every obstacle is an oriented box or a compound of boxes, and the payload is a box; a hollow tube is represented by its outer box, which is exact because no obstacle enters the bore. A keep-in rule (restricted box R, allowed box A) forbids R \ A, a union of at most six boxes, which are added as virtual obstacles. For samples ξ_i and a pose sequence P_t, the margin of sample i is

  m_i = min_t min_o g(P_t D(ξ_i), B_o),  D(ξ) = T_EO⁻¹ Δ(ξ) T_EO,

where g is the separating-axis margin of two oriented boxes [gottschalk1996obbtree]. It is positive and a lower bound on the distance when the boxes are apart, and minus the smallest overlap otherwise. The final-state rules (goal tolerances, placement) enter as further signed margins on the last pose.

By Lemma 2, a (state, box, sample) triple whose nominal margin exceeds |δt_i| + |φ_i| ℓ_max cannot touch, and is skipped. This pruning never changes a failure indicator.

The model was validated against the simulator's collision queries on 1.26 million (state, sample) pairs along the dev witnesses. It missed no contact, and disagreed on 0.017% of pairs, always conservatively and by less than 0.5 mm. Its final-state rules agree with the simulator on 6140 of 6144 dev samples. A candidate costs 5–40 ms on one CPU core.

### 4.3 Parameterisation

The insertion is P(s; θ) = P₀(s) D(s; θ). At K + 1 = 7 knots, θ holds the lateral and vertical offsets (y_f, z_f, y_r, z_r) of the payload's front and rear ends, which set its position, yaw and pitch (bounds ±60 mm). Offsets between knots follow a monotone cubic interpolation [fritsch1980pchip]. The goal knot is bounded at ±10 mm and also has an axial offset, so the planner chooses the final pose inside the goal region (12 mm, 5°); the success model penalises leaving it. In front/rear coordinates the payload's offset at any depth is an affine function of the two end offsets. No task-specific information enters the parameterisation.

### 4.4 Objective and optimisation

With N = 128 planning samples, common to all candidates, the cost is the sample-average approximation [shapiro2014lectures]

  J(θ) = (1/N) Σ_i σ(−m_i(θ)/τ) + λ_pen [m₀(θ) < m_min] + λ_reg ‖θ̄‖²,   (3)

where σ is the logistic function with τ = 0.05 mm, m₀ is the nominal margin (m_min = 1 mm), and θ̄ is θ normalised by its bounds. The smoothed indicator only breaks ties [penaordieres2020smooth]. Its width matters: with τ = 0.5 mm, near-miss samples dominate the cost, and on a dev task the surrogate ranked a path with a true risk of 8.2% below the straight insertion (18.6%).

Optimisation has two stages:

1. CMA-ES [hansen2016cma] on the goal knot alone, minimising the final-state term, which is a floor for every path ending at that goal.
2. CMA-ES on all knots, started from the stage-1 goal, for at most 60% of the budget or 200 iterations.

### 4.5 Realisation and certificate

The best distinct candidates are tracked in order: eight, then four shrinkage candidates θ·{0.8, 0.6, 0.4, 0.2}. Tracking is by IK continuation backwards from the goal, with arm-collision and keep-in checks at every state and a dense check at the evaluation resolution. If tracking fails while the payload is still outside the first opening, the insertion starts at the last tracked state (adaptive pre-entry). The straight insertion is the last resort.

After the transfer, the path is densified and validated, then re-scored on M = 1000 fresh planning samples. The planner reports the empirical risk and its one-sided 95% Clopper–Pearson upper bound [clopper1934]. Because these samples are independent of the path, the bound is valid for the modelled failure modes. The planner reads only the planning interface; a test confirms that removing the generator's metadata (design parameters, descriptors, obstacle roles) does not change the plan.

## 5. Benchmark and Experimental Protocol

**gosuite v0.4.** The benchmark has four application families (Figure 1):

| Family | Task | Intruding obstacles |
|---|---|---|
| F1 rack | delivery into a shelf bay | brackets |
| F2 rebar | threading into a cage | tie hook and side clip |
| F3 lathe | axial loading through a machine door | coolant nozzle and chip-guard lip |
| F4 tote | lowering into a divided tote cell | lid flap and cell ledge |

Each family has three clearance levels (16, 10 and 6 mm per side on the goal line). The obstacles sit near the opening, one shallow on one side and one deep on the other, and the goal holds 25–45 mm of slack. The risk is therefore controllable by the planner rather than decided by the goal. Each family × level has 2 dev and 6 test geometries, giving 96 tasks; the 72 test tasks are frozen by hash. Each task stores two oracle witnesses:
- the *straight* witness inserts along the goal line;
- the *risk-aware* witness was optimised by the generator with privileged knowledge (the obstacle depths and the useful deviation planes).

Both are references, not methods.

**Evaluation.** A path is densified to 2.5 mrad per joint, validated, and executed under 256 grasp errors per task, drawn from a fixed evaluation stream shared by all methods (common random numbers). Planning samples come from independent streams. Task success counts a planning failure as 0. Statistics are task-level means over seeds with 95% bootstrap intervals over tasks. Paired differences use two-sided Wilcoxon signed-rank tests over tasks [wilcoxon1945], Holm-adjusted across the comparisons of a table [holm1979].

**Methods.** All methods get a 120 s budget, run as 20 parallel jobs.
- **CRISP:** five seeds.
- **CC-VPSTO (adapted)** [brudermuller2026ccvpsto], five seeds:
  - joint-space via-points (5) through a clamped cubic spline, warm-started from the tracked straight insertion;
  - a trajectory-wise Monte Carlo chance constraint with N = 100 and exact binomial padding (β = 0.05, η = 0.05); the thresholds reproduce the published values;
  - a barrier-penalty ranking and CMA-ES;
  - CRISP's exact risk model and arm check, so only the formulation and the parameterisation differ;
  - step size tuned on dev.
- **Baselines** (five seeds): B6 keyframe-guided (structure-aware, risk-unaware); B1 RRT-Connect; B3 RRT-Connect with a 3 mm payload inflation; B4 RRT-Connect with a state-wise scenario constraint (N = 8, risk 0.25).
- **Ablations** (three seeds, each changing one component of CRISP): state-wise risk; linearised Boole risk (first-order Gaussian per state and box, summed); CVaR₀.₁ of the negative margin [lew2023riskaverse, rockafellar2000cvar]; max-clearance objective (no error model); translation-only error model; fixed goal pose.

**Protocol.** Every design choice was made on the dev split (Section 6.7). The code was frozen and hashed before the test run (`results/suite_v0.4/crisp/code_sha256.json`), and the test split was run once. The later studies (anytime, misspecification) use the frozen planner. The only additions are a harness option for per-method budgets, and reserve times scaled to the budget, which are identical at 120 s. No setting was changed on the basis of test results.

## 6. Results

All results in this section are on the 72 frozen test tasks unless stated otherwise. Tables 1–3 are generated by `python -m gosuite.paper_tables` from `results/suite_v0.4/comparison_crisp_test` and `comparison_crisp_ablation_test`.

### 6.1 Comparison with baselines, a recent method and the oracles

**Table 1.** Task success under grasp error (%), with planning failures counted as 0. Values are means over tasks of the per-task mean over seeds; the interval is a 95% bootstrap over tasks. "Collision | valid" is the mean episode collision rate of valid paths.

| Method | Valid path | Easy | Medium | Hard | All [95% CI] | Collision \| valid |
|---|---:|---:|---:|---:|---:|---:|
| **CRISP (ours)** | 100% | 95.3 | 92.9 | **88.6** | **92.2** [91.0, 93.3] | 5.3 |
| Risk-aware oracle (privileged) | 100% | 96.1 | 93.2 | 83.7 | 91.0 [89.5, 92.4] | 7.7 |
| Straight oracle | 100% | 93.8 | 79.8 | 58.2 | 77.3 [73.3, 80.8] | 21.8 |
| CC-VPSTO (adapted) [brudermuller2026ccvpsto] | 100% | 94.3 | 87.4 | 70.0 | 83.9 [80.6, 86.8] | 15.0 |
| B6 keyframe-guided | 99% | 89.6 | 79.0 | 56.6 | 75.1 [70.7, 79.2] | 23.0 |
| B3 RRT-Connect + 3 mm margin | 27% | 25.5 | 17.2 | 10.4 | 17.7 [12.1, 23.5] | 34.3 |
| B1 RRT-Connect | 31% | 19.6 | 16.4 | 7.4 | 14.5 [9.8, 19.0] | 53.3 |
| B4 RRT-Connect + scenarios | 21% | 21.0 | 12.9 | 5.6 | 13.2 [8.4, 18.3] | 36.5 |

**Table 2.** Paired differences in task success (percentage points). Two-sided Wilcoxon signed-rank tests over the 72 tasks, Holm-adjusted over the five comparisons.

| A − B | Easy | Medium | Hard | All [95% CI] | Tasks A higher / lower | p (Holm) |
|---|---:|---:|---:|---:|---:|---:|
| CRISP − B6 keyframe-guided | +5.7 | +13.9 | +31.9 | +17.1 [+13.5, +21.0] | 67 / 4 | 8.4×10⁻¹² |
| CRISP − CC-VPSTO (adapted) | +1.0 | +5.5 | +18.6 | +8.4 [+6.1, +11.0] | 63 / 7 | 4.8×10⁻¹¹ |
| CRISP − risk-aware oracle | −0.8 | −0.4 | +4.9 | +1.3 [+0.3, +2.3] | 43 / 27 | 0.029 |
| CRISP − straight oracle | +1.5 | +13.1 | +30.3 | +15.0 [+12.1, +18.1] | 67 / 4 | 1.0×10⁻¹¹ |
| CC-VPSTO − B6 | +4.7 | +8.3 | +13.3 | +8.8 [+5.7, +12.3] | 62 / 9 | 4.3×10⁻¹⁰ |

Among the methods (the oracles are references, not methods), CRISP is the best at every difficulty level, and its advantage grows with difficulty (Figure 3, Table 1).

- **Against the keyframe-guided baseline B6** it gains +17.1 points, rising to +31.9 at the hard level, and is higher on 67 of 72 tasks.
- **Against CC-VPSTO** it gains +8.4 points (+18.6 at hard). Both use the same exact risk model, so this difference isolates the formulation: a lever-arm-aware task-space parameterisation with a free goal against joint-space via-points to a fixed goal.
- **Against the privileged oracle** CRISP is +1.3 points overall, slightly lower at easy and medium, and +4.9 at hard. At hard it leads in every family: rack 83.8 vs 81.4, rebar 90.2 vs 82.9, lathe 88.8 vs 84.3, tote 91.5 vs 86.1.
- **The unguided planners** are limited by feasibility: they return a valid path in only 21–31% of runs.

Figure 5 shows representative insertions: the median-success seed of each method, as tip offsets (a) and as simulator close-ups with the payload trail (b). Without any information about where the intruding obstacles are, CRISP finds the strategy that the generator's oracle encodes by design. The payload tip swings away from each obstacle while passing it with the long lever arm, and returns to the goal line when only short-lever points are next to the obstacle.

### 6.2 Where the gain over the oracle comes from

For 7.2% of the evaluation draws, the payload fails even when parked at the task's reference goal pose. Every fixed-goal method loses these draws. On the remaining draws, the oracle succeeds in 98.0%, CRISP in 96.4% and CRISP with a fixed goal in 97.4%. On the reference-infeasible draws, CRISP succeeds in 38.0% of cases by choosing a different goal pose inside the goal region. The net advantage over the oracle therefore comes from goal selection; the oracle's path, which uses privileged knowledge, remains marginally better on the remaining draws. The price is a small increase in goal-tolerance failures: 1.6% of episodes, against 0.7% for the oracle.

### 6.3 Ablations

**Table 3.** Ablations (three seeds; each row changes one component of CRISP). The Δ column is CRISP minus the variant.

| Variant | Easy | Medium | Hard | All [95% CI] | Δ | Tasks CRISP higher / lower | p (Holm) |
|---|---:|---:|---:|---:|---:|---:|---:|
| CRISP (seeds 0–2) | 95.4 | 93.5 | 88.4 | 92.4 [91.1, 93.6] | | | |
| Translation-only error model | 93.0 | 79.4 | 70.1 | 80.8 [77.9, 83.4] | +11.6 | 68 / 3 | 2.8×10⁻¹² |
| Max-clearance objective | 93.0 | 91.1 | 84.1 | 89.4 [87.7, 90.9] | +3.0 | 60 / 10 | 5.6×10⁻⁹ |
| Fixed goal pose | 94.8 | 92.9 | 83.5 | 90.4 [88.8, 91.9] | +2.0 | 44 / 24 | 8.1×10⁻⁴ |
| State-wise risk | 95.4 | 92.9 | 88.6 | 92.3 [90.8, 93.6] | +0.1 | 36 / 34 | 0.84 |
| Linearised Boole risk | 95.3 | 92.9 | 88.1 | 92.1 [90.6, 93.4] | +0.3 | 27 / 43 | 0.61 |
| CVaR₀.₁ objective | 94.8 | 94.5 | 90.1 | 93.2 [91.9, 94.3] | −0.7 | 20 / 50 | 0.012 |

Three components carry the gain:

- **Modelling the rotational error** is the most important (−11.6 points without it, −18.3 at hard). A planner that sees only the translational error does not know that the risk sits at the payload tip.
- **A probabilistic objective** beats the classical maximum-clearance heuristic by 3.0 points (4.3 at hard). Equal clearances are not equally valuable at the tip and near the hand.
- **The free goal pose** is worth 2.0 points (4.9 at hard).

The choice of risk functional is second-order *for planning*: state-wise, linearised-Boole and trajectory-level objectives give statistically indistinguishable plans. Empirically, the failure sets of consecutive states are nested enough that these objectives rank candidate paths alike. They differ in what they claim about the planned path (Section 6.4).

The tail objective CVaR₀.₁ of the negative margin [lew2023riskaverse] was 0.7 points better than CRISP's chance objective (Holm p = 0.012). This variant was not selected on the dev split, so we report it as a finding rather than adopt it; confirming it requires fresh test tasks.

### 6.4 Calibration of risk estimates on the planned paths

For the seed-0 test paths of CRISP and of two ablations, the trajectory-level risk, the state-wise maximum and the Boole sum were estimated on 1000 fresh planning draws (a stream no planner used) and compared with the failure observed on the 256 evaluation draws.

**Table 4.** Risk estimators against the observed failure (%). Each estimator column gives its mean, its ratio to the observed mean, and the number of runs it underestimates. Source: `results/suite_v0.4/comparison_crisp_test/risk_estimators.json`.

| Paths | Observed failure | Trajectory-level | State-wise maximum | Boole sum |
|---|---:|---:|---:|---:|
| CRISP (proposed) paths | 7.5 | 7.7 (1.03×; 35/72) | 6.2 (0.83×; 49/72) | 747 (100×) |
| state-wise risk paths | 7.7 | 7.8 (1.01×; 35/72) | 5.9 (0.77×; 54/72) | 745 (97×) |
| linearised Boole risk paths | 7.5 | 7.7 (1.03×; 33/72) | 6.5 (0.87×; 45/72) | 442 (59×) |

The trajectory-level estimate is calibrated. Its ratio to the observed failure is 1.01–1.03, and it underestimates in about half of the runs, as a calibrated estimate should. The state-wise maximum is optimistic: it reports 77–87% of the true risk and underestimates in 45–54 of 72 runs, with the largest gaps on the riskiest paths (Figure 7). The Boole sum exceeds 100% on average (59–100× the true risk) and carries no information. Across all 360 CRISP test runs, the certificate predicted a mean failure of 8.0% against 7.8% observed (Pearson r = 0.92; Figure 8). So although the three functionals rank candidate paths alike (Section 6.3), only the trajectory-level one is a correct statement about the path that is executed. This is the practical content of Proposition 1.

### 6.5 Anytime behaviour and sample size

Seed 0 throughout. The 120 s and N = 128 rows are the seed-0 runs of the main experiment, and "CRISP, 120 s" is the same run as N = 128.

**Table 5.** Task success (%) by planning budget and by number of planning draws N. Source: `results/suite_v0.4/comparison_crisp_test/anytime.json`.

| Variant | Easy | Medium | Hard | All [95% CI] | Planning time (s, median) |
|---|---:|---:|---:|---:|---:|
| CRISP, 30 s | 95.0 | 89.7 | 84.1 | 89.6 [87.0, 92.1] | 38 |
| CRISP, 60 s | 96.1 | 93.8 | 88.9 | 92.9 [91.7, 94.1] | 59 |
| CRISP, 120 s | 96.2 | 93.8 | 87.6 | 92.5 [90.8, 94.0] | 109 |
| CC-VPSTO, 30 s | 93.7 | 86.8 | 68.5 | 83.0 [79.3, 86.5] | 36 |
| CC-VPSTO, 60 s | 94.5 | 87.2 | 69.5 | 83.7 [80.2, 86.8] | 50 |
| CC-VPSTO, 120 s | 94.0 | 87.5 | 70.5 | 84.0 [80.7, 86.9] | 85 |
| CRISP, N = 32 | 93.3 | 92.1 | 85.6 | 90.4 [88.3, 92.1] | 90 |
| CRISP, N = 64 | 95.7 | 93.1 | 89.5 | 92.8 [91.7, 93.9] | 109 |
| CRISP, N = 256 | 96.3 | 93.9 | 89.2 | 93.2 [91.9, 94.3] | 109 |

CRISP is a useful anytime planner. With a quarter of the budget (30 s) it reaches 89.6%, more than CC-VPSTO with 120 s (84.0%), and its hard-level success (84.1%) already matches the privileged oracle (83.7%). Beyond 60 s nothing changes, because CMA-ES converges in about 43 s. CC-VPSTO gains only 1.0 point from 30 to 120 s.

The risk estimate needs some, but not many, draws. N = 32 costs 2.1 points relative to N = 128, spread over all levels (−2.9, −1.7 and −2.0 points at easy, medium and hard). N = 64 and N = 256 are within the seed-to-seed spread of N = 128 (92.8% and 93.2% against 92.5%).

### 6.6 Robustness to a misspecified error model

Every method planned with the protocol error model (s = 1, rotation and translation combined). Its stored seed-0 test paths (the oracles' witnesses) were then executed again under misspecified models: every standard deviation scaled by s, with common random numbers (the same normal draws, scaled), and translation-only and rotation-only errors at s = 1. No method replanned. Tasks without a path count as 0. Source: `results/suite_v0.4/robustness_crisp_test/` (`python -m gosuite.robustness --config configs/robustness_crisp_v0.4.json`).

**Table 6.** Task success (%) under each execution error model (Figure 10).

| Method | s = 0.5 | s = 0.75 | s = 1 | s = 1.5 | s = 2 | translation-only, s = 1 | rotation-only, s = 1 |
|---|---:|---:|---:|---:|---:|---:|---:|
| CRISP (proposed) | 99.8 | 98.0 | 92.5 | 70.8 | 47.3 | 99.9 | 94.8 |
| Risk-aware oracle | 99.8 | 97.3 | 91.0 | 70.0 | 47.5 | 99.9 | 93.4 |
| CC-VPSTO (adapted) | 97.7 | 92.5 | 84.0 | 62.0 | 41.2 | 99.1 | 86.8 |
| B6 Keyframe-guided | 94.4 | 85.9 | 75.5 | 54.5 | 35.9 | 97.8 | 77.6 |
| Straight oracle | 96.1 | 87.6 | 77.3 | 56.1 | 37.2 | 99.9 | 79.0 |

Paired differences in percentage points (two-sided Wilcoxon over the 72 tasks, unadjusted):

| Paired difference (seed 0) | s = 0.5 | s = 0.75 | s = 1 | s = 1.5 | s = 2 | translation-only, s = 1 | rotation-only, s = 1 |
|---|---:|---:|---:|---:|---:|---:|---:|
| CRISP − B6 Keyframe-guided | +5.3 (p=1e-13) | +12.1 (p=9e-12) | +17.1 (p=5e-13) | +16.3 (p=1e-12) | +11.3 (p=6e-12) | +2.1 (p=0.005) | +17.2 (p=6e-13) |
| CRISP − CC-VPSTO (adapted) | +2.1 (p=1e-06) | +5.5 (p=2e-09) | +8.5 (p=1e-11) | +8.8 (p=1e-11) | +6.1 (p=6e-10) | +0.9 (p=8e-06) | +8.0 (p=1e-11) |
| CRISP − Risk-aware oracle | +0.0 (p=0.06) | +0.7 (p=0.002) | +1.5 (p=0.0009) | +0.9 (p=0.2) | -0.2 (p=0.7) | +0.1 (p=0.04) | +1.4 (p=0.0002) |

What this shows:

- **The ranking holds when the error model is wrong.** From half to twice the planned error, CRISP stays ahead of CC-VPSTO (+2.1 to +8.8 points) and B6 (+5.3 to +17.1), with p ≤ 10⁻⁶ at every scale. The gain is largest near the planning model and shrinks when the error becomes so small that every path succeeds, or so large that every path fails. At 1.5× and 2× the planned error, CRISP is level with the privileged oracle (+0.9 and −0.2, not significant). Nothing suggests that CRISP over-fits the error scale it planned for.
- **The risk is rotational.** Under translation-only errors every method succeeds 97.8–99.9% of the time. Under rotation-only errors B6 falls to 77.6% and CC-VPSTO to 86.8%, while CRISP keeps 94.8%, above the oracle's 93.4%. This is the lever-arm thesis measured directly, and it agrees with the translation-only ablation (−11.6 points).

### 6.7 Development decisions and computation

All design choices were made on the dev split; the evidence for each is in `docs/CRISP.md`, Section 6. Two of them were driven by diagnosis rather than tuning:

- **Smoothing width.** A wide sigmoid made the surrogate prefer straight paths; with τ = 0.05 mm, CRISP's dev insertion risk on fresh planning draws fell from 6.9% to 6.4% (4 seeds).
- **Adaptive pre-entry.** It removed a systematic arm self-collision at the start of the approach.

Two candidate components were tested and dropped as non-significant: held-out re-ranking, and a 1024-draw final-state estimate (p = 0.86).

On the test split, the CRISP optimiser converged in a median of 43 s and 5525 candidate evaluations (128 per second on one core). The first candidate was arm-feasible in 347 of 360 runs; a shrinkage candidate was used in 9, a lower-ranked one in 1, and the straight insertion in 3. The planning time recorded by the harness has a median of 109 s, because the shortcutting of the transfer, shared with B6, runs to its deadline. CC-VPSTO evaluated 37 candidates per second, because each candidate needs forward kinematics of a joint-space spline.

## 7. Discussion

**Why CRISP works.** The ablations point to two structural decisions:

1. **Error propagation must include the lever arm.** The rotational part of a static grasp error dominates the payload tip, so a planner that models only position errors optimises the wrong quantity.
2. **The decision variables should express lever-arm trade-offs directly.** In front/rear-end coordinates, "move the tip away while the body passes an obstacle on the other side" is a simple, near-linear move. In joint space (CC-VPSTO), the same move is a coupled, ill-conditioned perturbation. This is consistent with CC-VPSTO improving on its warm start in all 16 medium and hard dev tasks while staying 18.6 points behind CRISP at hard on the test split.

The free goal pose matters because the task's own tolerance region is a resource. The final state is a floor for the risk of every path that ends there.

**What the risk functional does and does not change.** The literature distinguishes per-step and trajectory-level chance constraints mainly because of their guarantees [degroot2025scenario, brudermuller2026ccvpsto]. Our results support that distinction precisely. As objectives they rank candidate paths alike, but as *claims about the planned path* they differ (Section 6.4). A planner that reports a per-state risk understates the probability that its path fails; one that reports a Boole bound is uninformative. CRISP's certificate is calibrated against independent evaluation draws.

**Limitations.**
- **Simulation only.** The study is kinematic simulation with an uncalibrated Gaussian error model; there are no contact dynamics, compliance, tracking error or hardware experiments.
- **Box geometry.** The exact risk model relies on boxes. Meshes would need a convex decomposition or signed-distance queries, and would be slower.
- **The arm is not in the optimiser.** Its feasibility is checked after optimisation, and in 3 of 360 test runs (rack tasks) every candidate made the arm touch the rack.
- **Designed benchmark.** It was built so that lever-arm-aware planning matters. This is stated openly, and the families are drawn from real applications, but absolute success rates should not be read as industrial predictions.
- **CVaR found after the fact.** The CVaR advantage was observed after the test run and was not adopted.

## 8. Conclusions

We studied the insertion of elongated payloads through narrow openings under a static in-hand grasp error, and showed that the trajectory-level success probability can be optimised directly and efficiently. The proposed planner, CRISP, combines:
- an exact, pruned risk model;
- a lever-arm-aware task-space parameterisation;
- CMA-ES with a free goal pose;
- a calibrated risk certificate.

On a 72-task frozen test split it reached 92.2% task success: 17.1 points above a structure-aware baseline, 8.4 points above a re-implementation of a recent chance-constrained optimiser, and 1.3 points above a privileged oracle, all with corrected significance. Future work:

- calibrate the error model on hardware;
- make the optimiser arm-aware;
- validate the CVaR variant on fresh tasks;
- couple CRISP with in-hand sensing that shrinks the error before insertion.

**Data and code availability.** Benchmark, planners, run records (every episode outcome), configurations, code hashes and the scripts that generate every table and figure are in the repository. Table and figure sources are listed in `docs/CRISP.md`.

## Figures

All figures are generated from the committed run records; regenerate them with the commands in `docs/CRISP.md`.

| Figure | Content | File |
|---|---|---|
| 1 | The four application families, rendered in the simulator | `previews/suite_v0.4/overview.png`, `previews/suite_v0.4/scenes.png` |
| 2 | Proposition 1 on straight insertions: state-wise maximum, trajectory-level risk and Boole sum | `docs/figures/risk_bounds.png` |
| 3 | Task success by level and method (Table 1) | `results/suite_v0.4/comparison_crisp_test/methods_by_level.png` |
| 4 | Per-task paired differences: CRISP − B6, CRISP − CC-VPSTO, CRISP − oracle | `results/suite_v0.4/comparison_crisp_test/paired_differences.png` |
| 5 | Representative tip paths on the hard test tasks (a) and simulator close-ups with the payload trail, B6 against CRISP (b) | `results/suite_v0.4/comparison_crisp_test/tip_paths.png`, `results/suite_v0.4/comparison_crisp_test/render_comparison.png` |
| 6 | Ablations (Table 3) | `results/suite_v0.4/comparison_crisp_test/ablations.png` |
| 7 | Risk estimators against the observed failure (Table 4) | `results/suite_v0.4/comparison_crisp_test/risk_estimators.png` |
| 8 | Certificate calibration over 360 CRISP runs | `results/suite_v0.4/comparison_crisp_test/certificate_calibration.png` |
| 9 | Anytime behaviour and sample size (Table 5) | `results/suite_v0.4/comparison_crisp_test/anytime.png` |
| 10 | Execution under a misspecified error model (Table 6) | `results/suite_v0.4/robustness_crisp_test/robustness.png` |

LaTeX versions of Tables 1–6 are in `paper/tables/` (generated by `python -m gosuite.paper_tables`).

## References

See `paper/references.bib` (keys cited in brackets).
