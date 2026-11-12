"""Keyframe-guided baseline: task-space keyframes from the declared openings + joint-space transfer.

In the spirit of TAPOM's topology-derived keyframes, but using only information every planner gets in
the task file (portal frames, goal pose, payload geometry, grasp); it never reads the witness.

1. Backward-chain payload keyframes from the goal: for each portal, last to first, retract the current
   pose along that portal's axis until the portal's reference point lies `approach` m before its plane.
   The keyframes keep the goal orientation, so every opening is entered aligned and centred.
2. Track the keyframes from q_goal_reference backwards with IK continuation; every interpolated state
   is validity-checked (collisions + keep-in).  This gives the insertion segment pre-entry -> goal.
3. Connect q_start to the pre-entry configuration with OMPL (RRTConnect by default) under the same
   validity rule (optionally with margin / scenarios).  The harness validates the whole path.

Several approach distances are tried in turn.  The insertion is a straight Cartesian motion ending at
the goal pose; the planner does not reason about uncertainty, so it is a NOMINAL guided baseline.
"""
import time
import numpy as np
from ..geometry import interpolate_pose, inverse, pose_distance
from ..planning import PlanResult
from ..task import reference_points
from .ompl_planner import connect, validity


def keyframes(scene, goal, approach, lift=0.):
    """Payload keyframes [pre-entry, ..., goal], built backwards from the goal along the portal axes.

    With lift > 0 the opening is traversed `lift` m above the goal height and the payload is lowered
    onto the goal pose at the end ("insert above, then settle"); lift = 0 is a straight insertion.
    """
    size = scene['payload']['size']
    frames = [goal.copy()]
    if lift > 0:
        raised = goal.copy()
        raised[2, 3] += lift
        frames.insert(0, raised)
    for portal in reversed(scene['task'].get('portals', [])):
        pose = np.asarray(portal['pose'], dtype=float)
        axis = pose[:3, 0]
        ref = reference_points(frames[0][None], size, portal.get('point', 'front'))[0]
        signed = float(np.dot(ref - pose[:3, 3], axis))
        back = signed + approach
        if back > 1e-6:
            k = frames[0].copy()
            k[:3, 3] -= back * axis
            frames.insert(0, k)
    return frames


def track(problem, tcp_poses, q_seed, linear_step=.004, angular_step=.025, max_jump=.25, deadline=None):
    """IK continuation along consecutive TCP poses starting from configuration q_seed (at tcp_poses[0])."""
    world = problem.world
    path = [np.asarray(q_seed, dtype=float)]
    for seg, (a, b) in enumerate(zip(tcp_poses[:-1], tcp_poses[1:])):
        distance, angle = pose_distance(a, b)
        n = max(4, int(np.ceil(distance / linear_step)), int(np.ceil(angle / angular_step)))
        for u in np.linspace(0, 1, n + 1)[1:]:
            if deadline and time.perf_counter() > deadline:
                return None, dict(reason='timeout', segment=seg)
            q = world.ik(interpolate_pose(a, b, u), path[-1])
            if q is None:
                return None, dict(reason='ik', segment=seg, u=float(u))
            if np.max(np.abs(q - path[-1])) > max_jump:
                return None, dict(reason='ik_jump', segment=seg, u=float(u))
            hit = problem.check(q)
            if hit:
                return None, dict(reason=hit['reason'], segment=seg, u=float(u))
            path.append(q)
    return np.array(path), None


def plan(problem, seed=0, budget=60., approaches=(.04, .07, .10), lifts=(0., .02, .03, .04), transfer_planner='RRTConnect',
         range=None, margin=0., scenarios=0, risk=0., resolution=.005, simplify=True, validation_reserve=.05, validate=False, **_):
    begin = time.perf_counter()
    deadline = begin + (1 - validation_reserve) * budget
    scene = problem.scene
    goal = problem.goal_pose
    to_tcp = inverse(problem.world.T_EO)
    attempts = []
    insertion = None
    for lift in lifts:
        for approach in approaches:
            frames = keyframes(scene, goal, approach, lift)
            tcp = [f @ to_tcp for f in frames][::-1]  # goal -> pre-entry
            path, failure = track(problem, tcp, problem.q_goal_reference, deadline=deadline)
            attempts.append(dict(approach_m=approach, lift_m=lift, keyframes=len(frames), failure=failure))
            if path is not None:
                insertion = path[::-1]
                break
        if insertion is not None:
            break
    stats = dict(transfer_planner=transfer_planner, insertion_attempts=attempts, margin_m=margin, scenarios=int(scenarios), risk=risk)
    if insertion is None:
        stats.update(wall_seconds=time.perf_counter() - begin)
        return PlanResult(None, 'keyframe_tracking_failed', stats)
    q_pre = insertion[0]
    samples = problem.planning_errors(int(scenarios), seed) if scenarios else None
    valid_q = validity(problem, (problem.q_start, q_pre), margin, samples, risk)
    remaining = deadline - time.perf_counter()
    if remaining < .5:
        stats.update(wall_seconds=time.perf_counter() - begin)
        return PlanResult(None, 'timeout', stats)
    transfer, info = connect(problem, problem.q_start, q_pre, valid_q, remaining, seed, transfer_planner, range, 'length',
                             resolution, simplify)
    stats.update(transfer=info, insertion_states=len(insertion))
    if transfer is None:
        stats.update(wall_seconds=time.perf_counter() - begin, collision_checks=problem.collision_checks)
        return PlanResult(None, 'timeout', stats)
    path = np.vstack([transfer, insertion[1:]])
    stats.update(wall_seconds=time.perf_counter() - begin, collision_checks=problem.collision_checks)
    if validate:  # optional: the harness validates every returned path anyway
        report, _ = problem.validate(path)
        if not report['valid']:
            stats['validation'] = report.get('reason')
            return PlanResult(None, 'post_validation_failed', stats)
    return PlanResult(path, 'solved', stats)
