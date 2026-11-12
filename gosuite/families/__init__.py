"""Scene families. Each module exposes NAME, DESCRIPTION, TIGHT_ROLES, sample(rng, clearance) and build(params).

v0.3 families return (scene, knots, seeds); v0.4 families additionally return a `robust` spec that marks
the insertion segment the risk-aware witness optimiser may reshape (see gosuite.robust).
"""
from . import lathe, lathe4, rack, rack4, rebar, rebar4, tote, tote4

FAMILIES = {m.NAME: m for m in (rack, rebar, lathe, tote)}           # v0.3
FAMILIES_V04 = {m.NAME: m for m in (rack4, rebar4, lathe4, tote4)}    # v0.4
