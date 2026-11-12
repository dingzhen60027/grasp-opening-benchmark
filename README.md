# Grasp Opening Benchmark

[中文说明](README.zh-CN.md) · [Suite v0.4 (current)](SUITE_v0.4.md) · [Why v0.4](docs/BENCHMARK_ASSESSMENT_v0.3.md) · [Suite v0.3](SUITE_v0.3.md) · [Project status](PROJECT_STATUS.zh-CN.md)

A reproducible benchmark for transporting **grasped elongated workpieces through constrained openings under grasp-pose uncertainty**, with an experiment harness that runs baselines directly.

**Status: development benchmark, not a completed robust-planning method or a published journal result.** Simulation is kinematic (PyBullet) with a UR5e and a Robotiq 2F-85 gripper; the grasp-error model is uncalibrated; there is no real-robot evidence yet.

![Four application families](previews/suite_v0.4/overview.png)

## v0.4 suite (current)

v0.4 puts the level-controlled obstacles where the payload tip, which has the largest lever arm, must pass them during insertion. Goals keep slack, so the risk is decided by the path rather than by the goal state. Every task stores two oracle paths: a straight insertion and a risk-optimised one. Their measured success gap is the room a robust planner has to show its value. On the frozen test split, a keyframe-guided planner that follows the openings but ignores uncertainty reaches 75.1% task success, close to the straight oracle (77.3%); the risk-aware oracle reaches 91.0% and is 27.0 points above the guided planner at the hard level. See [SUITE_v0.4.md](SUITE_v0.4.md), Section 5.

## v0.3 suite (calibration and sensitivity track)

| Component | Available |
|---|---|
| Families | Rack bay delivery, rebar cage threading, lathe chuck loading, kitting tote insertion |
| Tasks | 96 = 4 families x 3 clearance levels (20 / 10 / 5 mm per side) x (2 dev + 6 test) geometries |
| Task rules | Collision-free; per-state keep-in regions (e.g. enter only the target bay); ordered portals; payload goal region; placement |
| Evidence | Scene JSON, scripted feasibility witness, SHA-256 hashes, difficulty descriptors, rejection logs, an independent audit (96/96) |
| Evaluation | Fixed per-execution grasp perturbation (2 mm / 1 deg per axis), 256 common-random-number episodes, bootstrap CIs over tasks |
| Harness | Planner plug-in interface, parallel resumable runner, summary tables and figures |
| Built-in baselines | Reference oracle; RRT-Connect (nominal, margin-inflated, scenario) |
| Tests | `bash tools/python.sh -m pytest -q tests` |

Difficulty is calibrated by executing the scripted reference witnesses under the protocol. This is not a method comparison: see [SUITE_v0.3.md, Section 7](SUITE_v0.3.md#7-difficulty-calibration-reference-witnesses-oracle).

![Difficulty levels](previews/suite_v0.3/levels.png)

## Quick start

Tested with Python 3.10 on Linux; no ROS, Isaac Sim, private submodules or network access is needed after installing dependencies.

```bash
git clone https://github.com/dingzhen60027/grasp-opening-benchmark.git
cd grasp-opening-benchmark
python3.10 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
bash tools/python.sh -m pytest -q tests
bash tools/python.sh tools/check_evidence.py
bash tools/python.sh -m gosuite.audit --suite data/suite_v0.4

# Run the baseline table (plan -> validate -> execute under common grasp errors -> tables)
bash tools/python.sh -m gosuite.experiment --config configs/experiment_baselines_v0.4.json
```

To add your own planner, implement `plan(problem, seed, budget, **params)` and reference it as `"module:function"` in a config. [examples/custom_planner.py](examples/custom_planner.py) is a template; the interface is described in [SUITE_v0.3.md, Section 8](SUITE_v0.3.md#8-running-experiments).

## Earlier versions

- **v0.2 rack (`rackbench/`, `data/rack_v0.2/`)**: historical snapshot only. Its three reference storage paths enter the outer bay and slide sideways over the dividers into the target bay, and its success check examined the bay only at the final pose. The v0.3 rack family replaces it, with a per-state keep-in rule and reach-aware placement. Records: [RACK_BENCHMARK.zh-CN.md](RACK_BENCHMARK.zh-CN.md), [RACK_RESULTS.zh-CN.md](RACK_RESULTS.zh-CN.md).
- **v0.1 diagnostics (`gobench/`, `data/v0.1/`)**: five simplified opening families, 60 tasks, kept as a controlled diagnostic track ([DIAGNOSTIC_V0.1.zh-CN.md](DIAGNOSTIC_V0.1.zh-CN.md)).

## Repository map

- `gosuite/`: suites v0.3 and v0.4. Generator (`generate`), scene families (`families/`), world and collision checks (`world`), task rules (`task`), evaluator (`evaluate`), audit, planning interface (`planning`), baselines (`planners/`), harness (`experiment`, `report`, `compare`, `figures`), validation (`validate`) and screenshots (`render`).
- `data/suite_v0.4/` (current) and `data/suite_v0.3/`: 96 tasks each, witnesses, manifest with hashes, rejection log.
- `configs/`: protocol, experiment and comparison configs. `results/suite_v0.4/`: audit, validation, oracle and baseline runs, comparison tables; `results/suite_v0.3/`: audit, validation, reference evaluation, short baseline test and smoke runs.
- `previews/`: simulator screenshots; `assets/`: bundled models with licenses and SHA-256 provenance.
- `docs/`: design rationale, TAPOM reproduction status, CI template.

## Research status

Remaining work:

- adapt TAPOM ([arXiv:2511.05052](https://arxiv.org/abs/2511.05052)) and possibly other frontier methods;
- develop the proposed uncertainty-aware planner;
- calibrate the grasp-error distribution;
- run real-robot experiments.

No acceptance, publication or superiority claim is made. Third-party assets retain their bundled licenses. A first-party project license has not yet been selected; see [the notices](THIRD_PARTY_NOTICES.md).
