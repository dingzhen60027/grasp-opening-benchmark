"""Scene families of the v0.3 suite. Each module exposes NAME, DESCRIPTION, sample(rng, clearance) and build(params)."""
from . import lathe, rack, rebar, tote

FAMILIES = {m.NAME: m for m in (rack, rebar, lathe, tote)}
