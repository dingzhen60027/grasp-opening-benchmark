# v0.2.0-dev: warehouse rack development snapshot

First public snapshot of the grasp-pose uncertainty benchmark.

- UR5e + Robotiq 2F-85 models, 3 rack geometries and 6 directional storage/retrieval tasks.
- Nominal reference trajectories, whole-transfer uncertainty evaluation, support checks and an RRT-Connect baseline.
- Raw development outcomes, simulator previews, asset provenance and current research status.
- The source repository also retains 60 v0.1 controlled diagnostic tasks.

Local verification passed: 16 tests, 77 asset hashes, 66 task records, six nominal rack transfers and three open-gripper exit checks.

The attached `warehouse_rack_v0.2.zip` is a standalone **rack-only** bundle with seven rack tests. The full repository contains both diagnostic and rack suites (16 tests). The attached SHA256 file identifies the bundle. Install dependencies using `requirements-lock.txt` and follow the included Chinese README.

This is a development prerelease: no proposed robust planner, independent held-out rack test set, measured grasp-error calibration or real-robot validation is claimed. Reference trajectories are feasibility evidence, not a new method. Contact placement and grasp dynamics are outside the current model.

GitHub Actions is not enabled because the current publishing credential lacks workflow permission; the source repository includes a workflow template and local verification commands. Third-party asset licenses are retained; the first-party project license is not yet selected.

[Project](https://github.com/dingzhen60027/grasp-opening-benchmark) · [Current status](https://github.com/dingzhen60027/grasp-opening-benchmark/blob/main/PROJECT_STATUS.zh-CN.md) · [Recorded results](https://github.com/dingzhen60027/grasp-opening-benchmark/blob/main/RACK_RESULTS.zh-CN.md)
