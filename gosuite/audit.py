"""Independent audit of a generated suite: hashes, schema fields, and full nominal re-validation.

    python -m gosuite.audit --suite data/suite_v0.3 --output reports/suite_audit.json
"""
import argparse
import hashlib
import json
import time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
from .evaluate import DEFAULT_JOINT_STEP, validate_path
from .world import SuiteWorld

REQUIRED = ['schema_version', 'id', 'family', 'level', 'split', 'obstacles', 'payload', 'uncertainty', 'task', 'q_start',
            'q_goal_reference', 'evaluation_seed', 'descriptors']


def audit_one(job):
    suite, entry = job
    task_file, witness_file = suite / entry['file'], suite / entry['witness_file']
    out = dict(id=entry['id'], sha256_ok=hashlib.sha256(task_file.read_bytes()).hexdigest() == entry['sha256'],
               witness_sha256_ok=hashlib.sha256(witness_file.read_bytes()).hexdigest() == entry['witness_sha256'])
    scene = json.loads(task_file.read_text())
    out['missing_fields'] = [k for k in REQUIRED if k not in scene]
    world = SuiteWorld(scene)
    try:
        begin = time.perf_counter()
        report, dense, _, _ = validate_path(world, scene, np.load(witness_file), DEFAULT_JOINT_STEP)
        out.update(nominal_valid=report['valid'], reason=report['reason'], states=len(dense),
                   seconds=round(time.perf_counter() - begin, 2))
    finally:
        world.close()
    out['ok'] = out['sha256_ok'] and out['witness_sha256_ok'] and not out['missing_fields'] and out['nominal_valid']
    return out


def audit(suite, output=None, workers=8):
    suite = Path(suite)
    manifest = json.loads((suite / 'manifest.json').read_text())
    with Pool(workers) as pool:
        results = pool.map(audit_one, [(suite, e) for e in manifest['tasks']])
    summary = dict(suite=str(suite), version=manifest['version'], tasks=len(results), passed=sum(r['ok'] for r in results),
                   failed=[r['id'] for r in results if not r['ok']], joint_step_rad=DEFAULT_JOINT_STEP, results=results)
    if output:
        Path(output).parent.mkdir(parents=True, exist_ok=True)
        Path(output).write_text(json.dumps(summary, indent=1) + '\n')
    print(f"audit: {summary['passed']}/{summary['tasks']} tasks passed; failed: {summary['failed']}")
    return summary


def main():
    ap = argparse.ArgumentParser(description='Audit a gosuite benchmark directory.')
    ap.add_argument('--suite', required=True)
    ap.add_argument('--output')
    ap.add_argument('--workers', type=int, default=8)
    a = ap.parse_args()
    s = audit(a.suite, a.output, a.workers)
    raise SystemExit(0 if not s['failed'] else 1)


if __name__ == '__main__':
    main()
