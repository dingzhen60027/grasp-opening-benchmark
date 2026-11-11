"""Nominal validation and Monte Carlo execution evaluation under fixed grasp perturbations.

Evaluation protocol (shared by every method):

* The returned joint path is densified to ``joint_step`` (default 0.0025 rad per joint).
* Nominal validity: starts at ``q_start``; every state passes ``SuiteWorld.check``; the payload
  pose sequence satisfies the task rules (keep-in, portals, goal region, placement).
* Execution: ``episodes`` grasp perturbations xi ~ N(0, diag(sigma^2)) are drawn from the task's
  fixed evaluation stream (common random numbers: identical for every method and run).  Each xi is
  held for the whole trajectory.  An episode fails on the first payload-environment or
  payload-arm contact, otherwise it is judged by the same task rules.
* Planning-time samples come from a different stream (``planning_errors``) and never coincide
  with evaluation samples.
"""
import argparse
import hashlib
import json
import time
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.stats import beta
from .geometry import densify, grasp_delta
from .task import TaskChecker
from .world import SuiteWorld

DEFAULT_JOINT_STEP = 0.0025


def interval(k, n, confidence=.95):
    """Two-sided Clopper-Pearson interval for a Bernoulli rate."""
    if n == 0:
        return [0., 1.]
    a = (1 - confidence) / 2
    return [float(beta.ppf(a, k, n - k + 1)) if k else 0., float(beta.ppf(1 - a, k + 1, n - k)) if k < n else 1.]


def _std(scene, scale, mode):
    unc = scene['uncertainty']
    std = np.r_[unc['translation_std_m'], unc['rotation_std_rad']] * float(scale)
    if mode == 'translation_only':
        std[3:] = 0
    elif mode == 'rotation_only':
        std[:3] = 0
    elif mode != 'combined':
        raise ValueError(f'unknown uncertainty mode {mode!r}')
    return std


def evaluation_errors(scene, n, scale=1., mode='combined'):
    """Common-random-number evaluation draws; prefixes are identical for any n."""
    rng = np.random.default_rng([int(scene['evaluation_seed']), 0])
    return rng.normal(size=(int(n), 6)) * _std(scene, scale, mode)


def planning_errors(scene, n, seed=0, scale=1., mode='combined'):
    """Grasp perturbations a planner may use; an independent stream from the evaluation draws."""
    rng = np.random.default_rng([int(scene['evaluation_seed']), 1, int(seed)])
    return rng.normal(size=(int(n), 6)) * _std(scene, scale, mode)


def validate_path(world, scene, path, joint_step=DEFAULT_JOINT_STEP, checker=None):
    """Densify and nominally validate. Returns (report, dense_path, payload_poses, ee_poses)."""
    try:
        dense = densify(path, joint_step)
    except ValueError as exc:
        return dict(valid=False, reason='malformed_path', detail=str(exc)), None, None, None
    if np.max(np.abs(dense[0] - np.asarray(scene['q_start']))) > 1e-5:
        return dict(valid=False, reason='wrong_start', state=0), dense, None, None
    poses = np.empty((len(dense), 4, 4))
    ee = np.empty((len(dense), 4, 4))
    for i, q in enumerate(dense):
        hit = world.check(q)
        if hit:
            return dict(valid=False, reason='collision:' + hit['reason'], state=i, detail=hit), dense, None, None
        ee[i] = world.ee_pose()
        poses[i] = world.payload_T
    ok, reason, state = (checker or TaskChecker(scene))(poses)
    return dict(valid=bool(ok), reason=reason, state=state), dense, poses, ee


def path_metrics(world, dense, ee, clearance_stride=2, cutoff=.05):
    """Joint/EE path length and nominal payload clearance along the path."""
    clear = []
    for i in range(0, len(dense), clearance_stride):
        world.set_q(dense[i])
        world.place_payload()
        clear.append(world.payload_clearance(cutoff)[0])
    clear = np.array(clear)
    return dict(path_joint_length_rad=float(np.linalg.norm(np.diff(dense, axis=0), axis=1).sum()),
                ee_path_length_m=float(np.linalg.norm(np.diff(ee[:, :3, 3], axis=0), axis=1).sum()),
                payload_min_clearance_m=float(clear.min()),
                payload_clearance_p05_m=float(np.percentile(clear, 5)),
                trajectory_states=int(len(dense)))


def evaluate_path(scene, path, episodes=256, scale=1., mode='combined', joint_step=DEFAULT_JOINT_STEP,
                  keep_outcomes=True, metrics=True):
    begin = time.perf_counter()
    world = SuiteWorld(scene)
    checker = TaskChecker(scene)
    try:
        report, dense, nominal, ee = validate_path(world, scene, path, joint_step, checker)
        if not report['valid']:
            return dict(status='invalid_nominal_path', task_id=scene['id'], nominal=report,
                        wall_seconds=time.perf_counter() - begin)
        result = dict(status='evaluated', task_id=scene['id'], family=scene['family'], level=scene['level'],
                      split=scene['split'], episodes=int(episodes), uncertainty_scale=float(scale), mode=mode,
                      joint_step_max_rad=float(joint_step), evaluation_seed=scene['evaluation_seed'])
        if metrics:
            result.update(path_metrics(world, dense, ee))
        errors = evaluation_errors(scene, episodes, scale, mode)
        deltas = np.array([grasp_delta(xi) @ world.T_EO for xi in errors])
        alive = np.ones(len(errors), bool)
        hits = [None] * len(errors)
        for t, q in enumerate(dense):
            if not alive.any():
                break
            world.set_q(q)
            for e in np.flatnonzero(alive):
                world.place_payload_pose(ee[t] @ deltas[e])
                hit = world.payload_environment_hit() or world.payload_robot_hit()
                if hit:
                    alive[e] = False
                    hits[e] = dict(hit, state=t)
        outcomes = []
        for e in range(len(errors)):
            if hits[e]:
                outcomes.append(dict(collision=True, success=False, reason=hits[e]['reason'],
                                     obstacle=hits[e].get('obstacle', hits[e].get('link')), state=hits[e]['state']))
            else:
                ok, reason, state = checker(np.einsum('tij,jk->tik', ee, deltas[e]))
                outcomes.append(dict(collision=False, success=bool(ok), reason=reason, obstacle=None, state=state))
        nc = sum(o['collision'] for o in outcomes)
        ns = sum(o['success'] for o in outcomes)
        result.update(collision_count=nc, collision_rate=nc / episodes, collision_ci95=interval(nc, episodes),
                      success_count=ns, success_rate=ns / episodes, success_ci95=interval(ns, episodes),
                      failures_by_reason=dict(Counter(o['reason'].split(':')[0] for o in outcomes if not o['success'])),
                      colliding_obstacles=dict(Counter(o['obstacle'] for o in outcomes if o['collision'])),
                      wall_seconds=time.perf_counter() - begin,
                      qualification='Discrete kinematic evaluation of a fixed grasp perturbation per episode; '
                                    'no contact dynamics, tracking error or robustness certificate')
        if keep_outcomes:
            result['error_samples'] = errors.tolist()
            result['outcomes'] = outcomes
        return result
    finally:
        world.close()


def main():
    ap = argparse.ArgumentParser(description='Evaluate a joint path on one gosuite task.')
    ap.add_argument('--task', type=Path, required=True)
    ap.add_argument('--path', type=Path, required=True, help='Nx6 .npy joint path starting at q_start')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--episodes', type=int, default=256)
    ap.add_argument('--scale', type=float, default=1.)
    ap.add_argument('--mode', choices=['combined', 'translation_only', 'rotation_only'], default='combined')
    ap.add_argument('--joint-step', type=float, default=DEFAULT_JOINT_STEP)
    a = ap.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    scene = json.loads(a.task.read_text())
    result = evaluate_path(scene, np.load(a.path), a.episodes, a.scale, a.mode, a.joint_step)
    result['task_sha256'] = hashlib.sha256(a.task.read_bytes()).hexdigest()
    result['path_sha256'] = hashlib.sha256(a.path.read_bytes()).hexdigest()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, indent=1, allow_nan=False) + '\n')
    print({k: v for k, v in result.items() if k not in ('error_samples', 'outcomes')})


if __name__ == '__main__':
    main()
