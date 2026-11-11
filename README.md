# Grasp Opening Benchmark

[中文说明](README.zh-CN.md) · [Current project status](PROJECT_STATUS.zh-CN.md) · [Recorded results](RACK_RESULTS.zh-CN.md)

A reproducible development benchmark for transporting elongated workpieces through constrained openings under **grasp-pose uncertainty**.

**Status: development benchmark, not a completed robust-planning method or a published journal result.** The current application is warehouse / line-side rack storage and retrieval with a UR5e and a Robotiq 2F-85 gripper, simulated in PyBullet.

![Actual simulator geometry and task stages](previews/rack_v0.2/central_bay/workflow.png)

[Watch the kinematic replay](previews/rack_v0.2/central_bay/replay.mp4)

## What is available

| Component | Available evidence |
|---|---|
| Rack application v0.2 | 3 geometric configurations, 6 directional tasks; storage/retrieval pairs are not independent geometries |
| Robot and payload | Public manufacturer models; 460–560 mm hollow square tubes; an end grasp with explicit lever arm |
| Environment | Source table, rack beams/decks/dividers, neighboring stock, destination supports and guides |
| Grasp uncertainty | One fixed 6D perturbation per trajectory; translation-only, rotation-only and combined modes |
| Task validation | Full-path collision checks, specified bay, complete insertion/withdrawal, object goal region and support coverage |
| Nominal checks | 6/6 reference transfers and 3/3 nominal open-gripper exit paths passed |
| Baseline | RRT-Connect, independent of feasibility witnesses; one rack task/seed has been evaluated |
| Development tests | 16 passing tests; raw episode errors, outcomes, manifests and asset hashes included |
| Legacy diagnostics v0.1 | 5 simplified geometric families, 60 tasks; retained separately from the application benchmark |

The reference trajectories establish nominal task feasibility. **They are not outputs of a proposed robust planner.** The scenes are engineered synthetic layouts, not measured warehouse digital twins. Grasp closure, contact placement, slip and controller tracking are outside the current kinematic model. A standalone TAPOM reproduction is summarized in [this status note](docs/TAPOM_REPRODUCTION_STATUS.md); TAPOM has not yet been adapted to these rack tasks.

## Quick start

Tested with Python 3.10 on Linux. Runtime does not require ROS, Isaac Sim, private submodules or network access after installing dependencies.

```bash
git clone https://github.com/dingzhen60027/grasp-opening-benchmark.git
cd grasp-opening-benchmark
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
bash tools/python.sh -m pytest -q tests
bash tools/python.sh tools/check_evidence.py

# Recheck all six nominal transfers and the three open-gripper exits.
bash tools/python.sh -m rackbench.audit \
  --data data/rack_v0.2 --output reports/nominal_audit

# Run a planner without reading the feasibility witnesses.
bash tools/python.sh -m rackbench.plan \
  --scene data/rack_v0.2/tasks/central_bay_store.json \
  --output reports/rrt_seed0 --seed 0 --budget 30

# Evaluate only if the planner saved a trajectory.
bash tools/python.sh -m rackbench.evaluate \
  --scene data/rack_v0.2/tasks/central_bay_store.json \
  --path reports/rrt_seed0/trajectory.npy \
  --output reports/rrt_seed0/evaluation.json --episodes 128
```

Commands require fresh output paths so that recorded results are not silently overwritten. Use `reports/` for new runs. More generation, evaluation and rendering commands are in the [rack specification](RACK_BENCHMARK.zh-CN.md).

## Recorded development results

At 2 mm / 1 degree per-axis standard deviation, each reference storage path was evaluated with 128 fixed grasp-error draws. Observed collisions were 39/128, 37/128 and 40/128 across the three configurations. The single evaluated RRT path had 102/128 collisions despite nominal validity. These are diagnostic measurements, not a method ranking or a robustness guarantee. See [the results and limitations](RACK_RESULTS.zh-CN.md).

## Repository map

- `rackbench/`, `data/rack_v0.2/`: current rack tasks and tools.
- `gobench/`, `data/v0.1/`: shared utilities and [controlled geometric diagnostics](DIAGNOSTIC_V0.1.zh-CN.md).
- `assets/`: bundled models, upstream revisions, licenses and SHA256 provenance.
- `results/`: recorded evidence; `previews/`: simulator images and videos.
- `PROJECT_STATUS.zh-CN.md`: completed work, remaining research work and scope boundaries.
- `THIRD_PARTY_NOTICES.md`: asset provenance and licensing status.
- [CI setup](docs/CI_SETUP.md): a supplied workflow template; GitHub Actions is not enabled because the publishing credential lacks workflow permission.

## Research status

A new uncertainty-aware planner, strong baseline comparisons, independent held-out application geometries, measured grasp-error distributions and real-robot experiments remain to be completed. No acceptance, publication or superiority claim is made.

Task motivation includes [TAPOM (arXiv:2511.05052)](https://arxiv.org/abs/2511.05052). This repository does not distribute its unavailable dependencies and does not reproduce its full experimental tables.

Third-party assets retain their bundled licenses. A first-party project license has not yet been selected; see [the notices](THIRD_PARTY_NOTICES.md).
