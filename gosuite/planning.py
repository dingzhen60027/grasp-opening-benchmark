"""What a baseline planner receives, and what it must return.

A planner is any callable ``plan(problem, seed, budget, **params) -> PlanResult``.

* ``problem`` (PlanningProblem) exposes the task JSON (never the witness), start configuration,
  the reference goal configuration (one IK solution of the goal region), joint limits, a
  collision/validity checker with optional payload margin and grasp-perturbation scenarios,
  planning-time grasp samples (independent of the evaluation draws), a goal-region test and the
  full nominal validator used by the evaluation.
* ``seed`` seeds every random choice of the planner; ``budget`` is a wall-clock limit in seconds
  that the planner must honour (the harness records the measured time and flags overruns).
* The returned path is an Nx6 array of joint waypoints starting at ``q_start``.  The harness
  densifies it to the evaluation step, validates it nominally and then executes it under the
  common evaluation grasp perturbations.
"""
from dataclasses import dataclass, field
import numpy as np
from scipy.spatial.transform import Rotation
from .evaluate import DEFAULT_JOINT_STEP, planning_errors, validate_path
from .geometry import grasp_delta
from .task import TaskChecker
from .world import SuiteWorld


@dataclass
class PlanResult:
    path: object = None                      # Nx6 numpy array or None
    status: str = 'failed'                   # solved | timeout | failed | invalid_endpoint | ...
    info: dict = field(default_factory=dict)  # planner-specific statistics (JSON-serialisable)


class PlanningProblem:
    def __init__(self, scene, uncertainty_scale=1., mode='combined', joint_step=DEFAULT_JOINT_STEP):
        self.scene = scene
        self.task_id = scene['id']
        self.q_start = np.asarray(scene['q_start'], dtype=float)
        self.q_goal_reference = np.asarray(scene['q_goal_reference'], dtype=float)
        self.uncertainty_scale = uncertainty_scale
        self.mode = mode
        self.joint_step = joint_step
        self.world = SuiteWorld(scene)
        self.lower, self.upper = self.world.lower.copy(), self.world.upper.copy()
        self.checker = TaskChecker(scene)
        self.witness = None  # set by the harness ONLY for the labelled 'reference' oracle
        goal = scene['task']['goal']
        self.goal_pose = np.asarray(goal['object_pose'], dtype=float)
        self.goal_tolerance = (goal['position_tolerance_m'], goal['orientation_tolerance_rad'])
        self.collision_checks = 0

    def close(self):
        self.world.close()

    def check(self, q, margin=0., scenarios=None, risk=0.):
        """None if q is valid, else a dict with the violated constraint.

        Nominal validity = SuiteWorld.check (joint limits, collisions; optional payload-environment
        `margin`) AND the task's keep-in rules, which are per-state constraints (e.g. the payload may
        not enter a rack bay other than the target).  Portal order and the goal region are
        path-level and are verified by `validate`.

        With `scenarios` (Nx6 grasp perturbations) the state must additionally keep the payload
        collision-free and inside the keep-in regions for all but floor(risk * N) of them.
        """
        self.collision_checks += 1
        hit = self.world.check(q, margin=margin)
        if hit:
            return hit
        ee = self.world.ee_pose()
        rule = self.checker.state_violation(ee @ self.world.T_EO)
        if rule:
            return dict(reason='keep_in', rule=rule)
        if scenarios is None or len(scenarios) == 0:
            return None
        allowed, colliding = int(np.floor(risk * len(scenarios) + 1e-9)), 0
        for xi in scenarios:
            pose = self.world.place_payload_pose(ee @ grasp_delta(xi) @ self.world.T_EO)
            if (self.world.payload_environment_hit() or self.world.payload_robot_hit()
                    or self.checker.state_violation(pose)):
                colliding += 1
                if colliding > allowed:
                    break
        self.world.place_payload_pose(ee @ self.world.T_EO)
        if colliding > allowed:
            return dict(reason='scenario_risk', colliding=colliding, samples=len(scenarios), allowed=allowed)
        return None

    def is_valid(self, q, margin=0., scenarios=None, risk=0.):
        return self.check(q, margin, scenarios, risk) is None

    def planning_errors(self, n, seed=0):
        """Grasp perturbations from the planning stream (never equal to evaluation draws)."""
        return planning_errors(self.scene, n, seed, self.uncertainty_scale, self.mode)

    def payload_pose(self, q):
        self.world.set_q(q)
        return self.world.place_payload()

    def in_goal_region(self, q):
        t = self.payload_pose(q)
        dp = np.linalg.norm(t[:3, 3] - self.goal_pose[:3, 3])
        da = Rotation.from_matrix(self.goal_pose[:3, :3].T @ t[:3, :3]).magnitude()
        return dp <= self.goal_tolerance[0] and da <= self.goal_tolerance[1]

    def validate(self, path):
        """The evaluation's nominal validation: (report dict, densified path)."""
        report, dense, _, _ = validate_path(self.world, self.scene, path, self.joint_step, self.checker)
        return report, dense
