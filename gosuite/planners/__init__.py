"""Planner registry.  Built-in baselines plus any user planner given as 'package.module:function'.

Built-ins
---------
reference    Feasibility witness (oracle; NOT a method).  In v0.4 this is the risk-optimised witness;
reference_nominal  the straight v0.4 witness (oracle of a risk-unaware insertion along the goal line).
rrt_connect  Pure-Python seeded RRT-Connect with shortcut smoothing (no extra dependency).
ompl         OMPL planners (RRTConnect, BITstar, EITstar, PRM, KPIECE1, STRIDE, ...), with optional
             margin (obstacle inflation), scenarios/risk (chance-constraint approximation) and a
             clearance-integral objective.  Needs `pip install ompl==1.7.0`.
guided       Keyframe-guided planner: keyframes from the declared openings + OMPL transfer.
"""
import importlib
from . import guided, ompl_planner, reference, rrt

PLANNERS = {'reference': reference.plan, 'reference_nominal': reference.plan, 'rrt_connect': rrt.plan,
            'ompl': ompl_planner.plan, 'guided': guided.plan}


def load_planner(spec):
    if spec in PLANNERS:
        return PLANNERS[spec]
    if ':' not in spec:
        raise KeyError(f'unknown planner {spec!r}; use one of {sorted(PLANNERS)} or "module:function"')
    module, function = spec.split(':', 1)
    return getattr(importlib.import_module(module), function)
