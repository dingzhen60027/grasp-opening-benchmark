# Planner code changes after the test run was launched

`code_sha256.json` records the planner code at launch, before the one-shot test run (2026-11-12 07:28). Only one later change touched planner code. It was needed for the anytime study (`configs/experiment_crisp_budget_v0.4.json`), which runs the planners with 30 s and 60 s budgets. Analysis-only modules were added afterwards (`crisp_figures`, `robustness`, `paper_tables`); they do not plan.

| File | Change | Effect at the 120 s protocol budget |
|---|---|---|
| `gosuite/planners/crisp.py` | `post_reserve = min(post_reserve, .1 * budget)`; candidate-loop guard `min(10, .1 * budget)` instead of 10 | none: min(12, 12) = 12 and min(10, 12) = 10 |
| `gosuite/planners/ccvpsto.py` | transfer reserve `min(8, .1 * budget)` instead of 8; candidate-loop guard `min(10, .1 * budget)` instead of 10 | none: min(8, 12) = 8 and min(10, 12) = 10 |
| `gosuite/experiment.py` | an optional per-planner `budget_seconds` override | none: the main configs do not set it |

Every number in the main test tables comes from runs made with the code hashed in `code_sha256.json`.
