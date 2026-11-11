"""Template for plugging your own baseline into the gosuite experiment harness.

Run it with a config entry such as

    {"label": "Direct joint interpolation", "planner": "examples.custom_planner:plan", "params": {"margin": 0.0}}

(see configs/experiment_custom_example.json).  The harness calls ``plan(problem, seed, budget, **params)``
in a worker process and then densifies, validates and executes whatever path you return.

What ``problem`` (gosuite.planning.PlanningProblem) gives you:
    problem.q_start, problem.q_goal_reference     start configuration, one IK solution of the goal region
    problem.lower, problem.upper                  joint limits (rad)
    problem.check(q, margin=0., scenarios=None)   None if valid, else a dict (collision / keep-in / limits)
    problem.planning_errors(n, seed)              grasp perturbations you MAY use (never the evaluation draws)
    problem.payload_pose(q), problem.in_goal_region(q)
    problem.validate(path)                        the evaluation's own nominal validation
    problem.scene                                 the full task JSON (obstacles, payload, rules); no witness

This example is the simplest possible baseline: straight-line interpolation in joint space.  It is
a useful lower bound because it solves only tasks whose start and goal are trivially connected.
"""
import time
import numpy as np
from gosuite.planning import PlanResult


def plan(problem, seed=0, budget=60., margin=0., resolution=.005, **_):
    begin = time.perf_counter()
    q0, q1 = problem.q_start, problem.q_goal_reference
    n = max(2, int(np.ceil(np.max(np.abs(q1 - q0)) / resolution)) + 1)
    path = np.linspace(q0, q1, n)
    for k, q in enumerate(path):
        if time.perf_counter() - begin > budget:
            return PlanResult(None, 'timeout', dict(checked_states=k))
        hit = problem.check(q, margin=margin)
        if hit:
            return PlanResult(None, 'failed', dict(blocked_at_state=k, reason=hit['reason']))
    report, _ = problem.validate(path[[0, -1]])
    status = 'solved' if report['valid'] else 'post_validation_failed'
    return PlanResult(path[[0, -1]] if report['valid'] else None, status, dict(validation=report.get('reason')))
