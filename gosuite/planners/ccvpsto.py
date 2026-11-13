"""CC-VPSTO adapted to grasp error: a recent sampling-based chance-constrained trajectory optimiser.

Brudermueller, Berger, Jankowski, Bhattacharyya, Calinon, Jungers, Hawes, "CC-VPSTO: Chance-Constrained
Via-Point-Based Stochastic Trajectory Optimisation for Online Robot Motion Planning under Uncertainty",
IJRR 2026 (arXiv 2402.01370).  Re-implemented here as a comparison method; the elements kept from the paper:

* decision variables: joint-space via-points; the trajectory is the cubic spline through them with fixed
  boundary configurations and zero boundary velocities (VP-STO's minimum-acceleration interpolant);
* trajectory-wise (joint) chance constraint estimated by Monte Carlo: s_N(x) = number of the N samples,
  drawn ONCE before optimisation, for which the trajectory violates the constraint anywhere;
* padding: k_thresh = max{k : BinomCDF(k; N, eta) <= beta} (their Eq. 10, beta = 0.05);
* penalty ranking: cost = J(x) if s_N <= k_thresh, else J_pen_min + a (s_N - k_thresh - 1) + J(x), so every
  infeasible candidate ranks below every feasible one and infeasible ones are ordered by excess violations;
* CMA-ES over the via-points; the result is the best-ranked candidate.

Adaptation to this benchmark (stated in the paper's terms): the uncertainty delta is the in-hand grasp error
xi; g(x, xi) > 0 iff the perturbed payload touches the environment, leaves a keep-in region or misses the
final-state rules (the same exact estimator as CRISP, gosuite.risk); J is the joint path length (VP-STO
minimises duration, which for a fixed velocity limit is monotone in it); nominal arm validity is a hard
constraint, checked like CRISP's: on the shortlisted candidates after optimisation, best first (a nominal
payload contact counts N violations inside the loop).  The structure is the same as B6 and CRISP: the
optimised segment is the insertion pre-entry -> goal, warm-started from the tracked straight insertion, and
the transfer from the start uses RRTConnect.  The goal configuration is the task's q_goal_reference, as the
paper fixes the boundary conditions.
"""
import time
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.stats import binom
from ..evaluate import validate_path
from ..geometry import densify
from ..planning import PlanResult
from ..risk import RiskModel
from . import guided
from .crisp import cma_es, distinct, outside_first_portal, realise, Insertion, insertion_valid
from .ompl_planner import connect, validity


def k_binom(beta, n, eta):
    """Largest violation count k with BinomCDF(k; n, eta) <= beta (-1 if even k = 0 fails the test)."""
    ks = np.arange(n + 1)
    ok = ks[binom.cdf(ks, n, eta) <= beta]
    return int(ok.max()) if len(ok) else -1


class ViaPointTrajectory:
    def __init__(self, q0, q1, via, step=.01):
        self.q0, self.q1, self.n = np.asarray(q0, float), np.asarray(q1, float), via
        self.step = step

    def path(self, via):
        knots = np.vstack([self.q0, np.asarray(via).reshape(self.n, 6), self.q1])
        s = np.linspace(0, 1, len(knots))
        spline = CubicSpline(s, knots, bc_type='clamped')
        coarse = spline(np.linspace(0, 1, 50))
        length = np.abs(np.diff(coarse, axis=0)).max(axis=1).sum()
        return spline(np.linspace(0, 1, max(20, int(np.ceil(length / self.step)) + 1)))


def plan(problem, seed=0, budget=120., samples=100, eta=.05, beta=.05, via_points=5, sigma0=.005, popsize=24,
         penalty_min=1e3, penalty_slope=10., approach=.06, optimise_fraction=.6, candidates=12, transfer_planner='RRTConnect',
         resolution=.005, validation_reserve=.05, **_):
    begin = time.perf_counter()
    deadline = begin + (1 - validation_reserve) * budget
    scene, world = problem.scene, problem.world
    rng = np.random.default_rng(seed)
    k_thresh = k_binom(beta, samples, eta)
    stats = dict(method='CC-VPSTO (adapted)', samples=samples, eta=eta, beta=beta, k_thresh=k_thresh, via_points=via_points)

    # warm start: the tracked straight insertion (as B6), with the adaptive pre-entry of CRISP
    ins = Insertion(guided.keyframes(scene, problem.goal_pose, approach, 0.), scene['payload']['size'][0], free_goal=False)
    poses = ins.poses(np.zeros(ins.dim))
    straight, failure = realise(problem, poses, problem.q_goal_reference, deadline, outside=outside_first_portal(scene, poses))
    if straight is None:
        remaining = deadline - time.perf_counter()
        result = guided.plan(problem, seed, max(remaining, 1.)) if remaining > 5 else PlanResult(None, 'timeout', {})
        result.info = dict(stats, fallback='guided', warm_start_failure=failure, guided=result.info)
        return result
    q_pre, q_goal = straight[0], straight[-1]
    traj = ViaPointTrajectory(q_pre, q_goal, via_points)
    idx = np.linspace(0, len(straight) - 1, via_points + 2).round().astype(int)[1:-1]
    x0 = straight[idx].ravel()
    model = RiskModel(scene, problem.planning_errors(samples, seed))

    def payload_poses(path):
        out = np.empty((len(path), 4, 4))
        for i, q in enumerate(path):
            world.set_q(q)
            out[i] = world.place_payload()
        return out

    evaluations = [0]

    def violations(via):
        """Violated draws of the via-point trajectory (payload model only; the arm is checked after optimisation,
        on the shortlisted candidates, exactly as for CRISP)."""
        path = traj.path(via)
        sep = model.separations(payload_poses(path))
        sep[-1] = np.minimum(sep[-1], model.final_margins(world.payload_T))
        if sep[:, 0].min() <= 0:
            return samples, path
        return int((sep[:, 1:].min(axis=0) <= 0).sum()), path

    def cost(z):
        evaluations[0] += 1
        via = x0 + z * sigma0 / .3   # CMA-ES works on a normalised box; .3 normalised = sigma0 rad
        k, path = violations(via)
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        return length + (0. if k <= k_thresh else penalty_min + penalty_slope * (k - k_thresh - 1))

    t0 = time.perf_counter()
    ranked, iterations = cma_es(cost, np.zeros(len(x0)), rng, begin + optimise_fraction * budget, sigma0=.3, popsize=popsize)
    k_warm, _ = violations(x0)
    stats.update(iterations=iterations, evaluations=evaluations[0], optimise_seconds=time.perf_counter() - t0, violations_warm_start=k_warm)
    insertion, tried = None, 0
    for c, z in distinct(ranked, candidates):   # best-ranked first; the first arm-valid one is used
        if time.perf_counter() > deadline - min(10., .1 * budget):   # identical at 120 s; scales for short budgets
            break
        tried += 1
        k, path = violations(x0 + z * sigma0 / .3)
        if k < samples and insertion_valid(problem, path, problem.joint_step):
            insertion = path
            stats.update(violations_best=k, chance_feasible=k <= k_thresh, chosen_rank=tried - 1)
            break
    stats['candidates_tried'] = tried
    if insertion is None:
        insertion = straight
        stats['used_warm_start'] = True

    remaining = deadline - min(8., .1 * budget) - time.perf_counter()   # keep time for the validation (shortcutting runs to its deadline)
    if remaining < .5:
        return PlanResult(None, 'timeout', dict(stats, wall_seconds=time.perf_counter() - begin))
    transfer, info = connect(problem, problem.q_start, insertion[0], validity(problem, (problem.q_start, insertion[0])),
                             remaining, seed, transfer_planner, None, 'length', resolution, True)
    stats['transfer'] = info
    if transfer is None:
        return PlanResult(None, 'timeout', dict(stats, wall_seconds=time.perf_counter() - begin))
    path = np.vstack([transfer, insertion[1:]])
    report, _, _, _ = validate_path(world, scene, path, problem.joint_step, problem.checker)
    stats.update(validation=report.get('reason'), wall_seconds=time.perf_counter() - begin)
    if not report['valid']:
        return PlanResult(None, 'post_validation_failed', stats)
    return PlanResult(path, 'solved', stats)
