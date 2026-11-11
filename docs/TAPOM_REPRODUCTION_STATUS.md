# TAPOM partial reproduction status

This is a status record for a separate local reproduction, not a TAPOM implementation or a rack-benchmark comparison.

- Upstream: https://github.com/Jeong-zju/TAPOM-Planner
- Paper: https://arxiv.org/abs/2511.05052
- Source commit: `fb6e4bb4a983374bb89b36cd9a4377d688136e2e` (`release/tapom`).
- Executed case: `rebar_1/task_1`, ABB IRB 4600, seed `78543287`, 600 s budget.
- Observed planning wall time including topology initialization: 207.0934 s.
- Saved trajectory: 20,000 × 6 joint states; endpoint error 0 rad.
- Independent discrete validation: 20,000 states, 0 detected collisions.

The [sanitized result record](../results/external_reference/tapom_rebar_smoke.json) preserves measured outputs and the original record hash; only the local absolute output path was removed, and explanatory provenance fields were added. The upstream solver, compatibility patches, model assets, environment and full trajectory for that separate experiment are not distributed here, so this record alone is not a self-contained reproduction package.

The remaining referenced Husky cases depended on an ETH GitLab model submodule that was inaccessible during reproduction. Their full statistics were not reproduced. The available release code snapshot also cannot be assumed identical to the exact code used for every result in the paper.

TAPOM has not yet been adapted to this repository's UR5e rack scenes. A single successful external reproduction does not establish a success rate, reproduce the paper's multi-scene ranking, or provide a comparison to a new robust planner.
