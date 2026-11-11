"""B0 reference: returns the scripted feasibility witness.  An oracle row for calibration only."""
from ..planning import PlanResult


def plan(problem, seed=0, budget=0., **params):
    if problem.witness is None:
        return PlanResult(None, 'failed', dict(error='witness not provided by the harness'))
    return PlanResult(problem.witness.copy(), 'solved', dict(witness_used=True, oracle=True))
