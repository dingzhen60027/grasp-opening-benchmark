"""Generate the v0.3 suite: seeded sampling, scripted witnesses, nominal validation, splits and manifest.

Each (family, level, split, index) gets its own seed.  Parameters are re-sampled until a scripted
witness passes the full nominal validation (collision + keep-in + portals + goal + placement) at
the evaluation step.  Every rejected attempt is logged with its reason; no instance is filtered
on uncertain outcomes or on planner success.
"""
import argparse
import hashlib
import json
import platform
import subprocess
import time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import pybullet
from . import __version__
from .evaluate import DEFAULT_JOINT_STEP, validate_path
from .families import FAMILIES
from .families.common import InfeasibleDesign
from .geometry import densify
from .world import SuiteWorld, ROOT
from .witness import follow_knots

LEVELS = {'easy': .020, 'medium': .010, 'hard': .005}
SPLITS = {'dev': 2, 'test': 6}
MASTER_SEED = 20261111
WITNESS_STEP = .005
INCIDENTAL_MIN = .015  # witnesses keep >= 15 mm from everything except the family's designed tight features


def digest(text, n=12):
    return int(hashlib.sha256(text.encode()).hexdigest()[:n], 16)


def task_id(family, level, split, index):
    return f'{family}_{level}_{split}_{index:02d}'


def write_json(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=1, allow_nan=False) + '\n')


def clearance_profile(world, dense, stride=2, cutoff=.05):
    """Per-state minimum payload clearance and per-obstacle minimum distance along a path."""
    per_state, per_obstacle = [], {}
    for i in range(0, len(dense), stride):
        world.set_q(dense[i])
        world.place_payload()
        d = world.payload_distances(cutoff)
        per_state.append((i, min(d.values()) if d else cutoff))
        for name, v in d.items():
            per_obstacle[name] = min(per_obstacle.get(name, cutoff), v)
    return per_state, per_obstacle


def incidental_conflict(scene, per_obstacle, tight_roles, minimum=INCIDENTAL_MIN):
    """Name of a non-designed obstacle that the witness passes closer than `minimum`, if any."""
    roles = {o['name']: o.get('role') for o in scene['obstacles']}
    near = [(d, n) for n, d in per_obstacle.items() if roles.get(n) not in tight_roles and d < minimum]
    return min(near)[1] if near else None


def descriptors(world, scene, dense, poses, ee, profile=None, near=.02):
    """BARN-style difficulty descriptors computed on the accepted witness."""
    per_state, per_obstacle = profile or clearance_profile(world, dense)
    clear, moved, last = [], 0., None
    for i, d in per_state:
        clear.append(d)
        if last is not None and d < near:
            moved += float(np.linalg.norm(ee[i, :3, 3] - ee[last, :3, 3]))
        last = i
    pay = scene['payload']
    lever = pay['grasp']['lever_arm_m']
    sigma_r = scene['uncertainty']['rotation_std_rad'][0]
    c = scene['design']['clearance_per_side_m']
    return dict(design_clearance_m=c, payload_length_m=pay['size'][0], lever_arm_m=lever,
                min_nominal_clearance_m=round(float(min(clear)), 5),
                constrained_path_length_m=round(moved, 4),
                ee_path_length_m=round(float(np.linalg.norm(np.diff(ee[:, :3, 3], axis=0), axis=1).sum()), 4),
                portals=len(scene['task'].get('portals', [])),
                keep_in_rules=len(scene['task'].get('keep_in', [])),
                obstacles=len(scene['obstacles']),
                tip_sensitivity=round(c / (lever * sigma_r), 4),
                closest_features={n: round(d, 4) for d, n in sorted((d, n) for n, d in per_obstacle.items())[:4]},
                witness_states=int(len(dense)))


def generate_one(job):
    family, level, split, index, master, max_attempts, out = job
    module = FAMILIES[family]
    tid = task_id(family, level, split, index)
    seed = digest(f'{master}:{tid}')
    rng = np.random.default_rng(seed)
    log, begin = [], time.perf_counter()
    for attempt in range(max_attempts):
        params = module.sample(rng, LEVELS[level])
        try:
            scene, knots, seeds = module.build(params)
        except InfeasibleDesign as exc:
            log.append(dict(attempt=attempt, reason='design:' + str(exc)))
            continue
        world = SuiteWorld(scene)
        try:
            reason = world.layout_conflict()
            if reason:
                log.append(dict(attempt=attempt, reason=reason))
                continue
            variants = knots if isinstance(knots[0], list) else [knots]
            for variant_index, variant in enumerate(variants):
                path, failure = follow_knots(world, variant, seeds)
                if not failure:
                    break
            if failure:
                log.append(dict(attempt=attempt, **{k: v for k, v in failure.items() if k in ('reason', 'segment', 'u', 'detail')}))
                continue
            path = densify(path, WITNESS_STEP)
            scene['q_start'] = path[0].tolist()
            scene['q_goal_reference'] = path[-1].tolist()
            world.set_q(path[-1])
            scene['task']['goal']['object_pose'] = world.place_payload().tolist()
            report, dense, poses, ee = validate_path(world, scene, path, DEFAULT_JOINT_STEP)
            if not report['valid']:
                log.append(dict(attempt=attempt, reason='validation:' + str(report['reason']), state=report.get('state')))
                continue
            profile = clearance_profile(world, dense)
            near = incidental_conflict(scene, profile[1], module.TIGHT_ROLES)
            if near:
                log.append(dict(attempt=attempt, reason='incidental_clearance:' + near))
                continue
            desc = descriptors(world, scene, dense, poses, ee, profile)
        finally:
            world.close()
        scene.update(id=tid, level=level, split=split, index=index,
                     generator=dict(seed=seed, attempt=attempt, master_seed=master, version=__version__,
                                    witness_variant=variant_index),
                     evaluation_seed=digest(f'evaluation:{master}:{tid}', 8),
                     descriptors=desc,
                     validation=dict(witness_joint_step_rad=WITNESS_STEP, audit_joint_step_rad=DEFAULT_JOINT_STEP,
                                     nominal_valid=True, continuous_certificate=False))
        task_file = out / 'tasks' / family / split / f'{tid}.json'
        witness_file = out / 'witnesses' / family / split / f'{tid}.npy'
        write_json(task_file, scene)
        witness_file.parent.mkdir(parents=True, exist_ok=True)
        np.save(witness_file, path)
        return dict(id=tid, family=family, level=level, split=split, index=index, seed=seed,
                    file=str(task_file.relative_to(out)), sha256=hashlib.sha256(task_file.read_bytes()).hexdigest(),
                    witness_file=str(witness_file.relative_to(out)), witness_sha256=hashlib.sha256(witness_file.read_bytes()).hexdigest(),
                    attempts=attempt + 1, rejections=log, wall_seconds=round(time.perf_counter() - begin, 2), descriptors=desc)
    return dict(id=tid, family=family, level=level, split=split, index=index, seed=seed, failed=True,
                attempts=max_attempts, rejections=log, wall_seconds=round(time.perf_counter() - begin, 2))


def git_commit():
    try:
        return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def generate(out, families=None, levels=None, splits=None, master=MASTER_SEED, workers=8, max_attempts=80):
    out = Path(out)
    if (out / 'manifest.json').exists():
        raise FileExistsError(f'{out} already holds a suite; choose a fresh directory')
    families = families or list(FAMILIES)
    levels = levels or list(LEVELS)
    splits = splits or dict(SPLITS)
    jobs = [(f, l, s, i, master, max_attempts, out) for f in families for l in levels for s, n in splits.items()
            for i in range(n)]
    begin = time.perf_counter()
    with Pool(workers) as pool:
        entries = []
        for e in pool.imap_unordered(generate_one, jobs):
            entries.append(e)
            status = 'FAILED' if e.get('failed') else f"ok after {e['attempts']} attempt(s)"
            print(f"{e['id']}: {status} ({e['wall_seconds']} s)", flush=True)
    entries.sort(key=lambda e: (families.index(e['family']), levels.index(e['level']), list(splits).index(e['split']), e['index']))
    failed = [e['id'] for e in entries if e.get('failed')]
    rejections = {e['id']: e.pop('rejections') for e in entries}
    write_json(out / 'rejections.json', rejections)
    provenance = json.loads((ROOT / 'assets/ur5e_robotiq/provenance.json').read_text())
    manifest = dict(
        suite='gosuite', version=__version__, status='development benchmark; test split frozen by the hashes below',
        created_unix=int(time.time()), master_seed=master, code_commit=git_commit(),
        environment=dict(python=platform.python_version(), numpy=np.__version__, pybullet=pybullet.__version__ if hasattr(pybullet, '__version__') else 'unknown'),
        families={f: FAMILIES[f].DESCRIPTION for f in families},
        levels={l: dict(clearance_per_side_m=LEVELS[l]) for l in levels}, splits=splits,
        split_policy='each task is its own sampled geometry; dev for tuning, test frozen before any baseline tuning',
        selection_policy='nominal witness feasibility only; no filtering on uncertain outcomes or planner success',
        witness_policy='witnesses are generator evidence; planners must not read them (except the labelled reference oracle)',
        counts=dict(tasks=len(entries) - len(failed), failed=len(failed),
                    rejected_attempts=sum(len(v) for v in rejections.values())),
        failed=failed, generation_wall_seconds=round(time.perf_counter() - begin, 1),
        robot_provenance=provenance, tasks=[e for e in entries if not e.get('failed')])
    write_json(out / 'manifest.json', manifest)
    return manifest


def main():
    ap = argparse.ArgumentParser(description='Generate the gosuite v0.3 benchmark into a fresh directory.')
    ap.add_argument('--output', required=True)
    ap.add_argument('--families', nargs='*', choices=list(FAMILIES))
    ap.add_argument('--levels', nargs='*', choices=list(LEVELS))
    ap.add_argument('--dev', type=int, default=SPLITS['dev'])
    ap.add_argument('--test', type=int, default=SPLITS['test'])
    ap.add_argument('--seed', type=int, default=MASTER_SEED)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--max-attempts', type=int, default=80)
    a = ap.parse_args()
    m = generate(a.output, a.families, a.levels, {'dev': a.dev, 'test': a.test}, a.seed, a.workers, a.max_attempts)
    print(json.dumps(m['counts']), 'failed:', m['failed'])


if __name__ == '__main__':
    main()
