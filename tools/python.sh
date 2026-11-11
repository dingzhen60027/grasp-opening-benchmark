#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
unset PYTHONPATH
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MPLBACKEND=Agg
if [[ -n "${GOBENCH_PYTHON:-}" ]]; then
  exec "$GOBENCH_PYTHON" -s "$@"
elif [[ -x .venv/bin/python ]]; then
  exec .venv/bin/python -s "$@"
else
  exec python3 -s "$@"
fi
