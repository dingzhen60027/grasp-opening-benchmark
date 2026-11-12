"""OMPL baselines (the planner family TAPOM compares against), on the gosuite validity checker.

Parameters
----------
planner      OMPL geometric planner class name: RRTConnect, BITstar, EITstar, PRM, KPIECE1, STRIDE, ...
range        maximum extension (rad, joint-space L2) for tree planners; None keeps the OMPL default.
margin       payload-environment safety margin (m) = obstacle inflation (TAPOM's uncertainty treatment).
scenarios,   planning-time grasp samples checked at every state; at most floor(risk * scenarios) may
risk         collide (scenario / particle approximation of a chance constraint).
objective    'length' (default) or 'clearance': path cost = integral of 1 / (payload clearance + 2 cm),
             a clearance-aware cost for asymptotically optimal planners such as BITstar.
simplify     random shortcutting of the solution under the same validity rule, bounded by the budget.

Robust conditions (margin, scenarios) apply to every state except the given start and goal
configurations, which only need nominal validity (the goal lies inside the tight feature by design).
Only exact solutions count; the harness validates the returned path (validate=True re-checks inside the budget).  OMPL is imported inside the functions, i.e. only in harness worker
processes: the 1.7 Python bindings can abort during full interpreter shutdown, which forked workers skip.
"""
import time
from builtins import range as _range  # the planners' 'range' parameter shadows the builtin
import numpy as np
from ..planning import PlanResult
from .common import shortcut

CLEARANCE_CUTOFF = .05
CLEARANCE_EPS = .02


def validity(problem, endpoints, margin=0., scenarios=None, risk=0.):
    """State validity with the endpoint exemption: given endpoints need nominal validity only."""
    def valid_q(q):
        if any(np.allclose(q, e, atol=1e-9) for e in endpoints):
            return problem.check(q) is None
        return problem.check(q, margin=margin, scenarios=scenarios, risk=risk) is None
    return valid_q


def connect(problem, qa, qb, valid_q, time_limit, seed=0, planner='RRTConnect', range=None, objective='length',
            resolution=.005, simplify=True, simplify_fraction=.15):
    """Joint-space query qa -> qb. Returns (Nx6 waypoints or None, info)."""
    from ompl import base as ob, geometric as og, util as ou
    ou.setLogLevel(ou.LOG_ERROR)
    ou.RNG.setSeed(int(seed) + 1)
    begin = time.perf_counter()
    world = problem.world

    def to_np(state):
        return np.array([state[i] for i in _range(6)])

    class Checker(ob.StateValidityChecker):
        def __init__(self, si):
            super().__init__(si)

        def isValid(self, state):
            return valid_q(to_np(state))

        def clearance(self, state):
            world.set_q(to_np(state))
            world.place_payload()
            return world.payload_clearance(CLEARANCE_CUTOFF)[0]

    class ClearanceCost(ob.StateCostIntegralObjective):
        def __init__(self, si, checker):
            super().__init__(si, False)
            self.checker = checker

        def stateCost(self, state):
            return ob.Cost(1. / (self.checker.clearance(state) + CLEARANCE_EPS))

    space = ob.RealVectorStateSpace(6)
    bounds = ob.RealVectorBounds(6)
    for i in _range(6):
        bounds.setLow(i, float(problem.lower[i]))
        bounds.setHigh(i, float(problem.upper[i]))
    space.setBounds(bounds)
    si = ob.SpaceInformation(space)
    checker = Checker(si)
    si.setStateValidityChecker(checker)
    si.setStateValidityCheckingResolution(resolution / space.getMaximumExtent())
    si.setup()
    pdef = ob.ProblemDefinition(si)
    start, goal = ob.State(space), ob.State(space)
    for i in _range(6):
        start[i], goal[i] = float(qa[i]), float(qb[i])
    pdef.setStartAndGoalStates(start, goal)
    if objective == 'clearance':
        pdef.setOptimizationObjective(ClearanceCost(si, checker))
    p = getattr(og, planner)(si)
    if range is not None and hasattr(p, 'setRange'):
        p.setRange(float(range))
    p.setProblemDefinition(pdef)
    p.setup()
    solve_time = max(.5, time_limit * ((1 - simplify_fraction) if simplify else 1.))
    status = p.solve(solve_time)
    info = dict(ompl_status=status.asString(), solve_seconds=time.perf_counter() - begin)
    if not (bool(status) and pdef.hasExactSolution()):
        return None, info
    path = pdef.getSolutionPath()
    info['raw_waypoints'] = int(path.getStateCount())
    waypoints = np.array([to_np(path.getState(i)) for i in _range(path.getStateCount())])
    waypoints[0], waypoints[-1] = qa, qb
    if simplify:  # deadline-bounded shortcutting (OMPL's PathSimplifier ignores its time limit for one full pass)
        waypoints, info['shortcut_attempts'] = shortcut(waypoints, valid_q, resolution, begin + time_limit,
                                                        np.random.default_rng(seed))
    info['waypoints'] = len(waypoints)
    return waypoints, info


def plan(problem, seed=0, budget=60., planner='RRTConnect', range=None, margin=0., scenarios=0, risk=0.,
         objective='length', resolution=.005, simplify=True, validation_reserve=.05, validate=False, **_):
    begin = time.perf_counter()
    samples = problem.planning_errors(int(scenarios), seed) if scenarios else None
    q0, q1 = problem.q_start, problem.q_goal_reference
    valid_q = validity(problem, (q0, q1), margin, samples, risk)
    stats = dict(ompl_planner=planner, range=range, margin_m=margin, scenarios=int(scenarios), risk=risk,
                 objective=objective, resolution_rad=resolution)
    if not valid_q(q0) or not valid_q(q1):
        return PlanResult(None, 'invalid_endpoint', dict(stats, wall_seconds=time.perf_counter() - begin))
    waypoints, info = connect(problem, q0, q1, valid_q, (1 - validation_reserve) * budget, seed, planner, range,
                              objective, resolution, simplify)
    stats.update(info)
    if waypoints is None:
        stats.update(wall_seconds=time.perf_counter() - begin, collision_checks=problem.collision_checks)
        return PlanResult(None, 'timeout' if 'Timeout' in info['ompl_status'] or 'No solution' in info['ompl_status'] else 'approximate', stats)
    stats.update(wall_seconds=time.perf_counter() - begin, collision_checks=problem.collision_checks)
    if validate:  # optional: the harness validates every returned path anyway
        report, _ = problem.validate(waypoints)
        if not report['valid']:
            stats['validation'] = report.get('reason')
            return PlanResult(None, 'post_validation_failed', stats)
    return PlanResult(waypoints, 'solved', stats)
