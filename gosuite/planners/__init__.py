"""Planner registry.  Built-in baselines plus any user planner given as 'package.module:function'.

Built-ins
---------
reference    Scripted feasibility witness (oracle; NOT a method, reads the witness).
rrt_connect  Seeded joint-space RRT-Connect with shortcut smoothing.  Options:
             margin=<m>     payload-environment safety margin (TAPOM-style inflation),
             scenarios=<n>  every state must also be valid for n planning-time grasp samples
                            (scenario / particle style uncertainty-aware baseline).
"""
import importlib
from . import reference, rrt

PLANNERS = {'reference': reference.plan, 'rrt_connect': rrt.plan}


def load_planner(spec):
    if spec in PLANNERS:
        return PLANNERS[spec]
    if ':' not in spec:
        raise KeyError(f'unknown planner {spec!r}; use one of {sorted(PLANNERS)} or "module:function"')
    module, function = spec.split(':', 1)
    return getattr(importlib.import_module(module), function)
