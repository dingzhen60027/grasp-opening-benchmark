"""Helpers shared by the built-in planners."""
import time
import numpy as np


def edge_valid(a, b, valid_q, resolution, deadline=None):
    """Discrete edge check at `resolution` rad per joint; the exact endpoint b is checked last."""
    n = max(1, int(np.ceil(np.max(np.abs(b - a)) / resolution)))
    for k in range(1, n + 1):
        if deadline is not None and time.perf_counter() > deadline:
            return False
        if not valid_q(b if k == n else a + (b - a) * (k / n)):
            return False
    return True


def shortcut(path, valid_q, resolution, deadline, rng):
    """Random shortcutting that never runs past `deadline` (returns the best path found so far)."""
    path = [np.asarray(q, dtype=float) for q in path]
    attempts = 0
    while len(path) > 2 and time.perf_counter() < deadline:
        attempts += 1
        i, j = sorted(rng.choice(len(path), 2, replace=False))
        if j - i > 1 and edge_valid(path[i], path[j], valid_q, resolution, deadline):
            path = path[:i + 1] + path[j:]
    return np.array(path), attempts
