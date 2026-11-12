"""Scripted feasibility witnesses: Cartesian TCP knots followed by continuous IK.

A witness proves that a sampled instance is nominally solvable.  It is generator-side evidence
only; planners never receive it (the ``reference`` planner is the explicit exception and is
labelled as an oracle feasibility reference, not a method).
"""
import numpy as np
from .geometry import interpolate_pose, pose_distance


def follow_knots(world, knots, seeds, max_jump=.25, linear_step=.004, angular_step=.025, min_steps=8, start_q=None):
    """Track TCP knots with IK continuation. Returns (Nx6 path, None) or (None, failure dict).

    Each start seed is tried in turn for the first knot; the path is rejected on IK failure, on a
    joint jump larger than `max_jump` between interpolation points (branch switch) or on any
    nominal collision at an interpolation point.  With `start_q` the first configuration is fixed
    (used to give a second witness exactly the same start state).
    """
    failure = None
    for attempt, seed in enumerate([start_q] if start_q is not None else seeds):
        q = np.asarray(start_q, dtype=float) if start_q is not None else world.ik(knots[0], np.asarray(seed, dtype=float))
        if q is None:
            failure = dict(reason='ik_start', seed_index=attempt)
            continue
        hit = world.check(q)
        if hit:
            failure = dict(reason='start_' + hit['reason'], seed_index=attempt, detail=hit)
            continue
        path = [q]
        failure = None
        for segment, (a, b) in enumerate(zip(knots[:-1], knots[1:])):
            distance, angle = pose_distance(a, b)
            n = max(min_steps, int(np.ceil(distance / linear_step)), int(np.ceil(angle / angular_step)))
            for u in np.linspace(0, 1, n + 1)[1:]:
                q = world.ik(interpolate_pose(a, b, u), path[-1])
                if q is None:
                    failure = dict(reason='ik', segment=segment, u=float(u), seed_index=attempt)
                    break
                if np.max(np.abs(q - path[-1])) > max_jump:
                    failure = dict(reason='ik_jump', segment=segment, u=float(u), seed_index=attempt)
                    break
                hit = world.check(q)
                if hit:
                    failure = dict(reason=hit['reason'], segment=segment, u=float(u), seed_index=attempt,
                                   detail={k: v for k, v in hit.items() if k != 'reason'})
                    break
                path.append(q)
            if failure:
                break
        if not failure:
            return np.array(path), None
    return None, failure
