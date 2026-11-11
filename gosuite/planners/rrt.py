"""Seeded joint-space RRT-Connect baseline with random shortcutting.

Variants via parameters (same code path, so differences are attributable to the option):

* ``margin``     payload-environment safety margin in metres (obstacle inflation, as TAPOM uses to
                 absorb uncertainty);
* ``scenarios``  number of planning-time grasp perturbations checked at every state, of which at
  ``risk``       most floor(risk * scenarios) may collide (scenario / particle approximation of a
                 per-state chance constraint).

The robust conditions apply to every state except the given start and goal configurations, which
only need nominal validity: the goal pose lies inside the tight feature by design, so its inherent
risk under grasp error is not the planner's choice.  If the conditions make the approach to the
goal infeasible, the run ends as a planning failure (counted as task failure).

The planner connects ``q_start`` to ``q_goal_reference`` (one IK solution of the goal region);
it does not sample the goal region.  Edges are checked at ``resolution`` rad per joint; 10% of the
budget is reserved for re-validating the final path at the evaluation step.
"""
import time
import numpy as np
from ..planning import PlanResult

TRAPPED, ADVANCED, REACHED = 0, 1, 2


class _Tree:
    def __init__(self, root):
        self.nodes = np.empty((1024, 6))
        self.nodes[0] = root
        self.parent = [-1]
        self.n = 1

    def add(self, q, parent):
        if self.n == len(self.nodes):
            self.nodes = np.vstack([self.nodes, np.empty_like(self.nodes)])
        self.nodes[self.n] = q
        self.parent.append(parent)
        self.n += 1
        return self.n - 1

    def nearest(self, q):
        return int(np.argmin(np.sum((self.nodes[:self.n] - q) ** 2, axis=1)))

    def trace(self, i):
        out = []
        while i != -1:
            out.append(self.nodes[i].copy())
            i = self.parent[i]
        return out[::-1]


def plan(problem, seed=0, budget=60., margin=0., scenarios=0, risk=0., step=.25, resolution=.005, goal_bias=.05,
         smoothing_fraction=.25, validation_reserve=.1, **_):
    rng = np.random.default_rng(seed)
    begin = time.perf_counter()
    deadline = begin + (1 - validation_reserve) * budget
    samples = problem.planning_errors(int(scenarios), seed) if scenarios else None
    stats = dict(margin_m=margin, scenarios=int(scenarios), risk=risk, step_rad=step, resolution_rad=resolution, iterations=0)
    q0, q1 = problem.q_start, problem.q_goal_reference

    def valid(q):
        if np.array_equal(q, q0) or np.array_equal(q, q1):  # given endpoints: nominal validity only
            return problem.check(q) is None
        return problem.check(q, margin=margin, scenarios=samples, risk=risk) is None

    def edge(a, b):
        n = max(1, int(np.ceil(np.max(np.abs(b - a)) / resolution)))
        for k in range(1, n + 1):
            if time.perf_counter() > deadline:
                return False
            if not valid(b if k == n else a + (b - a) * (k / n)):  # exact endpoint, so the goal exemption applies
                return False
        return True

    def extend(tree, target):
        j = tree.nearest(target)
        near = tree.nodes[j]
        d = target - near
        dist = np.linalg.norm(d)
        new = target.copy() if dist <= step else near + d * (step / dist)
        if not edge(near, new):
            return TRAPPED, None
        return (REACHED if dist <= step else ADVANCED), tree.add(new, j)

    def finish(status, path=None, **extra):
        stats.update(extra, wall_seconds=time.perf_counter() - begin, collision_checks=problem.collision_checks)
        return PlanResult(path, status, stats)

    if not valid(q0) or not valid(q1):
        return finish('invalid_endpoint')
    trees = [_Tree(q0), _Tree(q1)]
    start_tree = trees[0]
    path = None
    while time.perf_counter() < deadline:
        stats['iterations'] += 1
        a, b = trees
        target = b.nodes[0] if rng.random() < goal_bias else rng.uniform(problem.lower, problem.upper)
        status, ia = extend(a, target)
        if status != TRAPPED:
            qa = a.nodes[ia]
            while True:
                status_b, ib = extend(b, qa)
                if status_b != ADVANCED:
                    break
            if status_b == REACHED:
                path = a.trace(ia) + b.trace(ib)[::-1][1:]
                if a is not start_tree:  # a grew from the goal: reverse so the path starts at q_start
                    path = path[::-1]
                break
        trees.reverse()
    nodes = sum(t.n for t in trees)
    if path is None:
        return finish('timeout', tree_nodes=nodes)
    first_solution = time.perf_counter() - begin
    raw_waypoints = len(path)
    smooth_until = min(deadline, time.perf_counter() + smoothing_fraction * budget)
    attempts = 0
    while len(path) > 2 and time.perf_counter() < smooth_until:
        attempts += 1
        i, j = sorted(rng.choice(len(path), 2, replace=False))
        if j - i > 1 and edge(path[i], path[j]):
            path = path[:i + 1] + path[j:]
    path = np.array(path)
    report, _ = problem.validate(path)
    if not report['valid']:
        return finish('post_validation_failed', None, tree_nodes=nodes, time_to_first_solution=first_solution,
                      validation=report.get('reason'))
    return finish('solved', path, tree_nodes=nodes, time_to_first_solution=first_solution, raw_waypoints=raw_waypoints,
                  waypoints=len(path), shortcut_attempts=attempts)
