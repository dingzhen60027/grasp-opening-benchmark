"""CRISP: chance-optimal insertion planning under in-hand grasp uncertainty (the proposed method).

Problem
-------
A grasp error xi = (dt, phi) ~ N(0, Sigma) is drawn once and held for the whole execution.  A path
succeeds for xi iff the perturbed payload never touches the environment and the final state meets the
task rules (goal region, placement).  The quantity to maximise is therefore the TRAJECTORY-LEVEL success
probability Pr_xi[success(path, xi)], whose failure event is a union over states.  A state-wise chance
constraint (B4) bounds each state separately and can accept paths whose union risk is far larger.

Under xi, a payload point at lever arm l from the grasp moves by about |dt| + |phi| l, so the same
clearance is much riskier at the payload tip than near the hand.  CRISP does not encode this as a rule:
it propagates every sampled error exactly through SE(3), so lever-arm effects follow from the geometry.

Method
------
1. Structure.  Payload keyframes from the declared openings (as B6): pre-entry -> goal.
2. Parameterisation.  The insertion is the straight keyframe motion P0(s), s in [0, 1], composed with a
   transverse deviation D(s; theta) in the payload frame.  At K+1 knots theta holds the lateral (y) and
   vertical (z) offsets of the payload's front and rear ends, so position, yaw and pitch are all free;
   the goal knot also has an axial offset.  The goal pose is thus chosen inside the goal region instead
   of being fixed to its nominal value.  Offsets between knots follow a monotone cubic (PCHIP).
3. Objective.  Sample-average approximation of the failure probability on N planning draws, common to
   all candidates: each draw's smallest margin over the whole insertion (separating-axis margin to every
   obstacle box, keep-in rules) and the final-state rules, passed through a sigmoid of width tau.  A
   nominal margin of at least m_min is required (penalty).  See gosuite.risk for the exact model.
4. Optimiser.  Cross-entropy method warm-started at the straight insertion.
5. Realisation.  IK continuation from the goal backwards, trying candidates in order of cost until one is
   arm-feasible, then a joint-space transfer from the start with RRTConnect (as B6).
6. Certificate.  The executed (densified) path is re-scored on M fresh planning draws; the planner reports
   the empirical risk and its one-sided Clopper-Pearson upper bound.

Information used: obstacle boxes, payload geometry and grasp, portals, goal region, keep-in and placement
rules, q_start, q_goal_reference (IK seed) and the grasp-error distribution, i.e. the planner interface.
The generator's design block, descriptors and witnesses are never read (tests/test_crisp.py).

Ablation variants
-----------------
objective='statewise'   maximise the worst per-state success probability (state-wise chance constraint)
objective='clearance'   maximise the smallest nominal margin (deterministic robustness, no error model)
error_model='translation'  plan with the rotational part of the grasp error removed (no lever arm)
free_goal=False         keep the goal pose at the task's nominal goal
"""
import time
import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.spatial.transform import Rotation
from scipy.stats import beta
from ..evaluate import _std, validate_path
from ..geometry import interpolate_pose, inverse, pose_distance
from ..planning import PlanResult
from ..risk import RiskModel
from . import guided
from .ompl_planner import connect, validity

CERTIFICATE_STREAM = 100_000   # planning-stream seed offsets: certificate draws and candidate-selection draws are
SELECTION_STREAM = 200_000     # disjoint from the optimisation draws (offset 0) and from each other


# --------------------------------------------------------------------------- parameterisation
def polyline(frames, step):
    """Payload poses along consecutive keyframes about every `step` m; returns (poses, progress in [0, 1])."""
    lengths = [max(pose_distance(a, b)[0], 1e-9) for a, b in zip(frames[:-1], frames[1:])]
    total, done = sum(lengths), 0.
    poses, progress = [], []
    for (a, b), length in zip(zip(frames[:-1], frames[1:]), lengths):
        n = max(1, int(np.ceil(length / step)))
        for u in np.arange(n) / n:
            poses.append(interpolate_pose(a, b, u))
            progress.append((done + u * length) / total)
        done += length
    poses.append(np.array(frames[-1], dtype=float))
    progress.append(1.)
    return np.array(poses), np.array(progress)


class Insertion:
    """theta -> payload pose sequence of a deviated insertion (pre-entry first, goal last)."""

    def __init__(self, frames, length, knots=6, max_offset=.03, goal_offset=.006, step=.004, free_goal=True):
        self.P0, self.s = polyline(frames, step)
        self.length = float(length)
        self.u = np.linspace(0., 1., knots + 1)          # knot progress; the last knot is the goal
        goal = goal_offset if free_goal else 0.
        self.bounds = np.r_[np.full(4 * knots, max_offset), np.full(5, goal)]
        self.free = self.bounds > 0
        self.dim = int(self.free.sum())

    def full(self, theta):
        out = np.zeros(len(self.bounds))
        out[self.free] = theta
        return np.clip(out, -self.bounds, self.bounds)

    def deviations(self, theta):
        """(T,4,4) payload-frame deviation transforms along the insertion."""
        x = self.full(theta)
        ends = PchipInterpolator(self.u, x[:-1].reshape(len(self.u), 4), axis=0)(self.s)   # (T,4): yf, zf, yr, zr
        yf, zf, yr, zr = ends.T
        D = np.tile(np.eye(4), (len(self.s), 1, 1))
        D[:, :3, :3] = Rotation.from_rotvec(np.c_[np.zeros_like(yf), -(zf - zr) / self.length, (yf - yr) / self.length]).as_matrix()
        D[:, :3, 3] = np.c_[x[-1] * self.s, (yf + yr) / 2, (zf + zr) / 2]
        return D

    def poses(self, theta):
        return np.einsum('tij,tjk->tik', self.P0, self.deviations(theta))

    def goal_slice(self):
        """Indices of the goal-knot parameters (front/rear y, z and the axial offset) within theta."""
        full = np.zeros(len(self.bounds), bool)
        full[-5:] = True
        return np.flatnonzero(full[self.free])

    def final_pose(self, theta):
        """Final payload pose only (the deviation at s = 1 equals the goal knot exactly)."""
        x = self.full(theta)
        yf, zf, yr, zr = x[-5:-1]
        D = np.eye(4)
        D[:3, :3] = Rotation.from_rotvec([0., -(zf - zr) / self.length, (yf - yr) / self.length]).as_matrix()
        D[:3, 3] = [x[-1], (yf + yr) / 2, (zf + zr) / 2]
        return self.P0[-1] @ D


# --------------------------------------------------------------------------- objective and optimiser
def sigmoid(x):
    return .5 * (1 + np.tanh(x / 2))


class Objective:
    """Cost of a deviation vector: risk term + nominal-margin penalty + regulariser.

    kind (risk term, all on the same planning draws and the same exact geometry):
      chance     smoothed trajectory-level failure probability (CRISP)
      statewise  worst per-state failure probability (state-wise chance constraint, as B4 / CC-RRT per step)
      cvar       CVaR_alpha of the negative per-draw margin (risk-averse SAA, Lew et al. RA-L 2023), scaled by reach
      linear     first-order Gaussian risk per (state, obstacle) summed by Boole's inequality (risk allocation as in
                 CC-RRT / p-Chekov / SCORA), mapped to B / (1 + B); final-state task rules by the draws
      clearance  smallest nominal margin, no error model (deterministic robustness heuristic)
    """

    KINDS = ('chance', 'statewise', 'cvar', 'linear', 'clearance')

    def __init__(self, model, insertion, kind='chance', tau=5e-5, min_margin=1e-3, regularisation=.02, std=None, alpha=.1,
                 final_model=None):
        if kind not in self.KINDS:
            raise ValueError(f'unknown objective {kind!r}')
        if kind == 'linear' and std is None:
            raise ValueError('the linear objective needs the per-axis standard deviation of the grasp error')
        if final_model is not None and not np.allclose(final_model.errors[:model.samples], model.errors):
            raise ValueError('the path draws must be a prefix of the final-state draws')
        self.model, self.insertion, self.kind = model, insertion, kind
        self.tau, self.min_margin, self.reg, self.std, self.alpha = tau, min_margin, regularisation, std, alpha
        self.final_model = final_model if kind in ('chance', 'statewise') else None
        self.evaluations = 0

    def final_state(self, final):
        """(N_f+1,) margins of the final state (contact, keep-in and final-state rules) on the final-state draws."""
        key = final.tobytes()
        if getattr(self, '_final_key', None) != key:   # cached: with a fixed goal every candidate shares it
            fm = self.final_model
            self._final_key, self._final = key, np.minimum(fm.separations(final[None])[0], fm.final_margins(final))
        return self._final

    def separations(self, poses):
        """(T, N+1) margins of a pose sequence; the last row includes the final-state rules."""
        sep = self.model.separations(poses)
        sep[-1] = np.minimum(sep[-1], self.model.final_margins(poses[-1]))
        return sep

    def margins(self, theta):
        return self.separations(self.insertion.poses(theta))

    def value(self, poses):
        """(risk term, nominal margin) of a payload pose sequence.

        With a final-state model (chance, statewise) the final state is scored on N_f >> N draws, because it
        costs a single pose and its failures (goal tolerance, placement) are tail events that N draws resolve
        poorly.  The trajectory-level union is then estimated without bias by
            Pr[A u B] = E[a] + E[b] - E[ab],
        with a = path failure (N draws), b = final-state failure (N_f draws for E[b]; the N path draws are a
        prefix of the N_f draws, which gives E[ab])."""
        if self.final_model is not None:
            sep = self.model.separations(poses)
            fin = self.final_state(poses[-1])
            nominal = float(min(sep[:, 0].min(), fin[0]))
            b = sigmoid(-fin[1:] / self.tau)
            if len(poses) == 1:   # goal stage: the final state alone
                return float(b.mean()), nominal
            if self.kind == 'chance':
                a = sigmoid(-sep[:, 1:].min(axis=0) / self.tau)
                return float(a.mean() + b.mean() - (a * b[:len(a)]).mean()), nominal
            per_state = sigmoid(-sep[:-1, 1:] / self.tau).mean(axis=1)
            return float(max(per_state.max(initial=0.), b.mean())), nominal
        sep = self.separations(poses)
        nominal = float(sep[:, 0].min())
        if self.kind == 'chance':
            return float(sigmoid(-sep[:, 1:].min(axis=0) / self.tau).mean()), nominal
        if self.kind == 'statewise':
            return float(sigmoid(-sep[:, 1:] / self.tau).mean(axis=1).max()), nominal
        if self.kind == 'cvar':
            loss = -np.minimum(sep[:, 1:].min(axis=0), self.model.reach)
            k = max(1, int(np.ceil(self.alpha * len(loss))))
            return float(np.sort(loss)[-k:].mean() / self.model.reach), nominal
        if self.kind == 'linear':
            bound = float(self.model.linearised(poses, self.std).sum())
            bound += float(sigmoid(-self.model.final_margins(poses[-1])[1:] / self.tau).mean())
            return bound / (1 + bound), nominal
        return -min(nominal, self.model.reach) / self.model.reach, nominal

    def _cost(self, poses, x):
        value, nominal = self.value(poses)
        penalty = 0. if nominal >= self.min_margin else 1. + (self.min_margin - nominal) / self.min_margin
        return value + penalty + self.reg * float(np.mean(x ** 2))

    def _normalised(self, theta):
        ins = self.insertion
        return ins.full(theta)[ins.free] / ins.bounds[ins.free]

    def __call__(self, theta):
        self.evaluations += 1
        return self._cost(self.insertion.poses(theta), self._normalised(theta))

    def goal_cost(self, theta):
        """Cost of the final state alone: a floor for every path that ends there (stage 1 of the optimiser)."""
        self.evaluations += 1
        return self._cost(self.insertion.final_pose(theta)[None], self._normalised(theta)[self.insertion.goal_slice()])

    def risk(self, theta):
        """(trajectory-level empirical risk, nominal margin) on the objective's draws."""
        sep = self.margins(theta)
        return float((sep[:, 1:].min(axis=0) <= 0).mean()), float(sep[:, 0].min())


def cross_entropy(objective, dim, bounds, rng, deadline, population=40, elite=6, iterations=20, init_std=1 / 3, smoothing=.7):
    """CEM from theta = 0.  Returns (sorted [(cost, theta)], iterations completed)."""
    history = []

    def score(theta):
        history.append((objective(theta), np.array(theta)))
        return history[-1][0]

    mean, std = np.zeros(dim), bounds * init_std
    score(mean)
    done = 0
    for _ in range(iterations):
        thetas = np.clip(mean + rng.normal(size=(population, dim)) * std, -bounds, bounds)
        costs = []
        for t in thetas:
            if time.perf_counter() > deadline:
                break
            costs.append(score(t))
        if len(costs) < population:
            break
        best = thetas[np.argsort(costs)[:elite]]
        mean = smoothing * best.mean(axis=0) + (1 - smoothing) * mean
        std = smoothing * best.std(axis=0) + (1 - smoothing) * std + 1e-3 * bounds
        score(mean)
        done += 1
    return sorted(history, key=lambda h: h[0]), done


def cma_es(f, x0, rng, deadline, sigma0=.3, popsize=None, max_iterations=200, tol=1e-4):
    """(mu/mu_w, lambda)-CMA-ES (Hansen 2016) on the box [-1, 1]^n, repaired by clipping.

    f takes a normalised vector.  Returns (sorted [(cost, x)], iterations completed)."""
    n = len(x0)
    lam = popsize or 4 + int(3 * np.log(n))
    mu = lam // 2
    w = np.log(mu + .5) - np.log(np.arange(1, mu + 1))
    w /= w.sum()
    mueff = 1 / np.sum(w ** 2)
    cc = (4 + mueff / n) / (n + 4 + 2 * mueff / n)
    cs = (mueff + 2) / (n + mueff + 5)
    c1 = 2 / ((n + 1.3) ** 2 + mueff)
    cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((n + 2) ** 2 + mueff))
    damps = 1 + 2 * max(0., np.sqrt((mueff - 1) / (n + 1)) - 1) + cs
    chi = np.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n ** 2))
    m, sigma = np.clip(np.asarray(x0, dtype=float), -1, 1), sigma0
    pc, ps, C = np.zeros(n), np.zeros(n), np.eye(n)
    B, D = np.eye(n), np.ones(n)
    history = [(f(m), m.copy())]
    done = 0
    for it in range(max_iterations):
        y = (rng.normal(size=(lam, n)) * D) @ B.T
        x = np.clip(m + sigma * y, -1, 1)
        y = (x - m) / sigma                                   # Lamarckian repair of the clipped samples
        costs = []
        for xi in x:
            if time.perf_counter() > deadline:
                break
            costs.append(f(xi))
            history.append((costs[-1], xi.copy()))
        if len(costs) < lam:
            break
        order = np.argsort(costs)[:mu]
        step = w @ y[order]
        m = np.clip(m + sigma * step, -1, 1)
        inv_sqrt = B @ np.diag(1 / D) @ B.T
        ps = (1 - cs) * ps + np.sqrt(cs * (2 - cs) * mueff) * inv_sqrt @ step
        hsig = np.linalg.norm(ps) / np.sqrt(1 - (1 - cs) ** (2 * (it + 1))) / chi < 1.4 + 2 / (n + 1)
        pc = (1 - cc) * pc + hsig * np.sqrt(cc * (2 - cc) * mueff) * step
        C = ((1 - c1 - cmu) * C + c1 * (np.outer(pc, pc) + (1 - hsig) * cc * (2 - cc) * C)
             + cmu * (y[order].T * w) @ y[order])
        sigma *= np.exp(cs / damps * (np.linalg.norm(ps) / chi - 1))
        C = (C + C.T) / 2
        evals, B = np.linalg.eigh(C)
        D = np.sqrt(np.maximum(evals, 1e-20))
        done += 1
        if sigma * D.max() < tol:
            break
    history.append((f(m), m.copy()))
    return sorted(history, key=lambda h: h[0]), done


def optimise(cost, ins, rng, deadline, optimiser='cmaes', goal_first=True, goal_fraction=.15, popsize=24, sigma0=.3,
             restarts=1, max_iterations=200, goal_iterations=60, freeze_goal=False, **cem):
    """Stage 1 (optional): goal pose in the goal region by its final-state cost.  Stage 2: the whole insertion,
    CMA-ES from the stage-1 goal with `restarts` IPOP restarts (population doubled, started from the best point).

    Returns (sorted [(cost, theta)], stats)."""
    b = ins.bounds[ins.free]
    begin = time.perf_counter()
    x0, stats = np.zeros(ins.dim), {}
    if goal_first and ins.free[-5:].all():
        g = ins.goal_slice()

        def goal_cost(z):
            x = np.zeros(ins.dim)
            x[g] = z
            return cost.goal_cost(x * b)
        ranked, it = cma_es(goal_cost, np.zeros(len(g)), rng, begin + goal_fraction * (deadline - begin), popsize=12,
                            max_iterations=goal_iterations)
        x0[g] = ranked[0][1]
        stats.update(goal_iterations=it, goal_cost_straight=goal_cost(np.zeros(len(g))), goal_cost=ranked[0][0])
    if optimiser == 'cmaes':
        # with freeze_goal, stage 2 moves only the path knots and keeps the stage-1 goal
        free = np.ones(ins.dim, bool)
        if freeze_goal and goal_first and ins.free[-5:].all():
            free[ins.goal_slice()] = False

        def lift(z):
            x = x0.copy()
            x[free] = z
            return x
        history, runs, start = [], [], x0[free]
        t = time.perf_counter()
        for k in range(restarts + 1):
            share = t + (k + 1) * (deadline - t) / (restarts + 1)
            ranked, it = cma_es(lambda z: cost(lift(z) * b), start, rng, share, sigma0=sigma0, popsize=popsize * 2 ** k,
                                max_iterations=max_iterations)
            runs.append(dict(iterations=it, best=ranked[0][0]))
            history += ranked
            start = min(history, key=lambda h: h[0])[1]
        ranked = sorted(((c, lift(z) * b) for c, z in history), key=lambda h: h[0])
        stats.update(runs=runs, goal_frozen=not free.all())
    else:
        ranked, it = cross_entropy(cost, ins.dim, b, rng, deadline, **cem)
        stats.update(iterations=it)
    stats.update(optimiser=optimiser)
    return ranked, stats


def distinct(ranked, count, tol=1e-3):
    """The best `count` candidates that differ from each other by more than `tol` in some parameter."""
    out = []
    for c, th in ranked:
        if all(np.max(np.abs(th - t)) > tol for _, t in out):
            out.append((c, th))
            if len(out) == count:
                break
    return out


def rerank(candidates, ins, model, tau):
    """Re-score candidates on held-out planning draws (SAA winner's-curse correction).

    Returns [(held-out smoothed failure, held-out risk, theta)] sorted best first."""
    out = []
    for _, th in candidates:
        poses = ins.poses(th)
        sep = model.separations(poses)
        sep[-1] = np.minimum(sep[-1], model.final_margins(poses[-1]))
        if sep[:, 0].min() <= 0:
            continue
        m = sep[:, 1:].min(axis=0)
        out.append((float(sigmoid(-m / tau).mean()), float((m <= 0).mean()), th))
    return sorted(out, key=lambda r: r[0])


# --------------------------------------------------------------------------- realisation
def outside_first_portal(scene, poses, clearance=.01):
    """(T,) bool: the payload's leading end is at least `clearance` before the first opening's plane."""
    portals = scene['task'].get('portals', [])
    if not portals:
        return np.zeros(len(poses), bool)
    portal = np.asarray(portals[0]['pose'], dtype=float)
    front = poses[:, :3, 3] + poses[:, :3, 0] * scene['payload']['size'][0] / 2
    return (front - portal[:3, 3]) @ portal[:3, 0] < -clearance


def realise(problem, poses, q_goal_seed, deadline, max_jump=.25, outside=None):
    """IK continuation from the goal backwards along payload poses. Returns (Nx6 path pre-entry->goal, failure).

    Adaptive pre-entry: if tracking fails at a state while the payload is still outside the first opening
    (`outside`), the insertion starts at the last tracked state instead (the transfer reaches it)."""
    world = problem.world
    tcp = np.einsum('tij,jk->tik', poses, inverse(world.T_EO))
    q = world.ik(tcp[-1], np.asarray(q_goal_seed, dtype=float))
    if q is None:
        return None, dict(reason='ik_goal')
    hit = problem.check(q)
    if hit:
        return None, dict(reason='goal_' + hit['reason'])
    path = [q]
    for k in range(len(tcp) - 2, -1, -1):
        if time.perf_counter() > deadline:
            return None, dict(reason='timeout', state=k)
        q = world.ik(tcp[k], path[-1])
        failure = None
        if q is None:
            failure = dict(reason='ik', state=k)
        elif np.max(np.abs(q - path[-1])) > max_jump:
            failure = dict(reason='ik_jump', state=k)
        else:
            hit = problem.check(q)
            if hit:
                failure = dict(reason=hit['reason'], state=k)
        if failure:
            if outside is not None and len(path) > 1 and outside[k + 1]:
                return np.array(path[::-1]), dict(failure, truncated_at=k + 1)
            return None, failure
        path.append(q)
    return np.array(path[::-1]), None


def insertion_valid(problem, insertion, step):
    """Nominal check of the densified insertion (the harness's resolution)."""
    from ..geometry import densify
    return all(problem.check(q) is None for q in densify(insertion, step)[1:])


def clopper_pearson_upper(k, n, confidence=.95):
    return 1. if k >= n else float(beta.ppf(confidence, k + 1, n - k))


# --------------------------------------------------------------------------- planner
def design(problem, seed=0, budget=120., objective='chance', error_model='combined', free_goal=True, samples=128, knots=6,
         approach=.06, max_offset=.06, goal_offset=.01, step=.004, tau=5e-5, min_margin=1e-3, regularisation=.02,
         optimiser='cmaes', goal_first=True, popsize=24, sigma0=.3, restarts=0, max_iterations=200, goal_iterations=60,
         population=40, elite=6, iterations=20, optimise_fraction=.6, candidates=8, selection=0, final_samples=0,
         shrink=(.8, .6, .4, .2), freeze_goal=False, **_):
    """Steps 1-4: parameterisation, risk model, optimisation and held-out selection (no arm involved).

    Returns (insertion, shortlist [(score, theta)] best first, cost, stats, begin, deadline)."""

    begin = time.perf_counter()
    deadline = begin + .95 * budget
    scene = problem.scene
    rng = np.random.default_rng(seed)
    stats = dict(method='CRISP', objective=objective, error_model=error_model, free_goal=free_goal, samples=samples, knots=knots)

    # 1-2. structure and parameterisation
    frames = guided.keyframes(scene, problem.goal_pose, approach, 0.)
    ins = Insertion(frames, scene['payload']['size'][0], knots, max_offset, goal_offset, step, free_goal)

    # 3-4. risk model on planning draws and optimisation
    def draws(n, stream):
        errors = problem.planning_errors(n, seed + stream)
        if error_model == 'translation':
            errors[:, 3:] = 0.
        elif error_model != 'combined':
            raise ValueError(f'unknown error model {error_model!r}')
        return errors
    model = RiskModel(scene, draws(samples, 0))
    final_model = RiskModel(scene, draws(final_samples, 0)) if final_samples else None
    std = _std(scene, problem.uncertainty_scale, problem.mode)
    if error_model == 'translation':
        std[3:] = 0.
    cost = Objective(model, ins, objective, tau, min_margin, regularisation, std, final_model=final_model)
    straight = cost.risk(np.zeros(ins.dim))
    t0 = time.perf_counter()
    ranked, info = optimise(cost, ins, rng, begin + optimise_fraction * budget, optimiser, goal_first and free_goal, popsize=popsize,
                            sigma0=sigma0, restarts=restarts, max_iterations=max_iterations, goal_iterations=goal_iterations,
                            freeze_goal=freeze_goal, population=population, elite=elite, iterations=iterations)
    stats.update(info, insertion_states=len(ins.s), straight_risk=straight[0], straight_nominal_margin=straight[1],
                 evaluations=cost.evaluations, optimise_seconds=time.perf_counter() - t0)

    # 4b. selection: re-score the best distinct candidates on held-out planning draws (chance objective only;
    #     the other objectives are not probability estimates and keep their own order)
    shortlist = distinct(ranked, candidates)
    # shrinkage toward the (usually arm-feasible) straight insertion: near-duplicate candidates tend to fail the
    # arm check for the same reason, while a shrunk deviation keeps part of the risk reduction
    best = shortlist[0][1]
    shrunk = [(cost(a * best), a * best) for a in shrink]
    shortlist = shortlist[:4] + shrunk + shortlist[4:]
    if selection and objective == 'chance':   # the other objectives keep their own order (ablations)
        held = rerank(shortlist, ins, RiskModel(scene, draws(selection, SELECTION_STREAM)), tau)
        shortlist = [(r, th) for _, r, th in held]
        stats['selection'] = dict(draws=selection, held_out_risk=[r for _, r, _ in held[:5]])

    return ins, shortlist, cost, stats, begin, deadline


def plan(problem, seed=0, budget=120., objective='chance', error_model='combined', free_goal=True, samples=128, knots=6,
         approach=.06, max_offset=.06, goal_offset=.01, step=.004, tau=5e-5, min_margin=1e-3, regularisation=.02,
         optimiser='cmaes', goal_first=True, popsize=24, sigma0=.3, restarts=0, max_iterations=200, goal_iterations=60,
         population=40, elite=6, iterations=20, optimise_fraction=.6, candidates=8, selection=0, final_samples=0,
         shrink=(.8, .6, .4, .2), freeze_goal=False, certify=1000, confidence=.95, transfer_planner='RRTConnect',
         resolution=.005, validation_reserve=.05, post_reserve=12., **_):
    """CRISP planner (see the module docstring); returns a PlanResult."""
    post_reserve = min(post_reserve, .1 * budget)   # identical at the 120 s protocol budget; scales down for short budgets
    ins, shortlist, cost, stats, begin, _ = design(
        problem, seed, budget, objective, error_model, free_goal, samples, knots, approach, max_offset, goal_offset, step, tau,
        min_margin, regularisation, optimiser, goal_first, popsize, sigma0, restarts, max_iterations, goal_iterations, population,
        elite, iterations, optimise_fraction, candidates, selection, final_samples, shrink, freeze_goal)
    deadline = begin + (1 - validation_reserve) * budget
    scene = problem.scene

    # 5a. arm realisation in order of preference; the straight insertion as the last resort
    tried, failures, insertion, chosen = 0, [], None, None
    for c, th in shortlist + [(None, np.zeros(ins.dim))]:
        if time.perf_counter() > deadline - post_reserve - min(10., .1 * budget):   # keep time for transfer, validation, certificate
            break
        tried += 1
        poses = ins.poses(th)
        path, failure = realise(problem, poses, problem.q_goal_reference, deadline, outside=outside_first_portal(scene, poses))
        if path is not None and not insertion_valid(problem, path, problem.joint_step):
            path, failure = None, dict(reason='dense_check')
        if path is None:
            failures.append(failure)
            continue
        insertion, chosen = path, (c, th)
        if failure:
            stats['pre_entry_truncated_at'] = failure['truncated_at']
        break
    stats.update(candidates_tried=tried, realisation_failures=failures)
    if insertion is None:   # nothing trackable: fall back to the keyframe-guided baseline with the remaining time
        remaining = deadline - time.perf_counter()
        result = guided.plan(problem, seed, max(remaining, 1.)) if remaining > 5 else PlanResult(None, 'timeout', {})
        result.info = dict(stats, fallback='guided', guided=result.info, wall_seconds=time.perf_counter() - begin)
        return result
    theta = ins.full(chosen[1])
    risk, nominal = cost.risk(chosen[1])
    stats.update(chosen_rank=tried - 1, straight_used=chosen[0] is None, predicted_risk=risk, nominal_margin=nominal,
                 goal_deviation_m=theta[-5:].tolist(), max_offset_used_m=float(np.abs(theta).max()))

    # 5b. joint-space transfer start -> pre-entry (as B6)
    q_pre = insertion[0]
    remaining = deadline - post_reserve - time.perf_counter()   # the shortcutting runs to its deadline
    if remaining < .5:
        return PlanResult(None, 'timeout', dict(stats, wall_seconds=time.perf_counter() - begin))
    valid_q = validity(problem, (problem.q_start, q_pre))
    transfer, info = connect(problem, problem.q_start, q_pre, valid_q, remaining, seed, transfer_planner, None, 'length',
                             resolution, True)
    stats['transfer'] = info
    if transfer is None:
        return PlanResult(None, 'timeout', dict(stats, wall_seconds=time.perf_counter() - begin, collision_checks=problem.collision_checks))
    path = np.vstack([transfer, insertion[1:]])

    # 6. nominal validation at the evaluation resolution and certificate on fresh planning draws
    report, dense, payload, _ = validate_path(problem.world, scene, path, problem.joint_step, problem.checker)
    stats['validation'] = report.get('reason')
    if not report['valid']:
        return PlanResult(None, 'post_validation_failed', dict(stats, wall_seconds=time.perf_counter() - begin))
    if certify:
        fresh = problem.planning_errors(certify, seed + CERTIFICATE_STREAM)   # always the full error model
        cert = RiskModel(scene, fresh).evaluate(payload)
        k = int(cert['failures'].sum())
        stats['certificate'] = dict(draws=int(certify), failures=k, risk=k / certify, confidence=confidence,
                                    upper_bound=clopper_pearson_upper(k, certify, confidence),
                                    scope='payload-environment contact, keep-in and final-state rules of the executed path')
    stats.update(wall_seconds=time.perf_counter() - begin, collision_checks=problem.collision_checks)
    return PlanResult(path, 'solved', stats)
