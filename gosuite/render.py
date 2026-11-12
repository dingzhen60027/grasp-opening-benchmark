"""Render benchmark screenshots from the actual simulator geometry (PyBullet TinyRenderer).

    python -m gosuite.render --suite data/suite_v0.3 --output previews/suite_v0.3

Produces, for representative tasks:
  <family>_workflow.png  four phases (start, aligned at the opening, inside the constraint, goal)
  overview.png           one close-up per family
  levels.png             family x difficulty grid of goal close-ups
Close-ups may hide occluding parts (machine roof, upper rack shelf); captions say so.
"""
import argparse
import json
import tempfile
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import pybullet as p
from PIL import Image, ImageDraw, ImageFont
from .geometry import apply, inverse
from .task import reference_points
from .world import SuiteWorld

HIDE_FOR_CLOSEUP = {'lathe': ('roof', 'front_wall_top'), 'rack': ('deck_3', 'beam_3_0', 'beam_3_1', 'rear_guard')}
HIDE_FOR_CLOSEUP_V04 = dict(HIDE_FOR_CLOSEUP, lathe=('roof', 'front_wall_top', 'side_wall_pos', 'side_wall_neg', 'front_wall_left', 'front_wall_right'))
HIDDEN_NOTE = {'lathe': 'machine roof and door lintel hidden', 'rack': 'upper shelf hidden'}
HIDDEN_NOTE_V04 = dict(HIDDEN_NOTE, lathe='machine roof and walls hidden')
FAMILY_TITLE = {'rack': 'F1 Rack bay delivery', 'rebar': 'F2 Rebar cage threading', 'lathe': 'F3 Lathe chuck loading',
                'tote': 'F4 Kitting tote insertion'}
FAMILY_TITLE_V04 = dict(FAMILY_TITLE, lathe='F3 Machine tending (axial loading)')


def is_v04(scene):
    return scene.get('generator', {}).get('suite_version') == 'v0.4'


def font(size, bold=False):
    name = 'DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf'
    try:
        return ImageFont.truetype(f'/usr/share/fonts/truetype/dejavu/{name}', size)
    except OSError:
        return ImageFont.load_default()


def snapshot(world, q, eye, target, width, height, fov=42):
    world.set_q(q)
    world.place_payload()
    view = p.computeViewMatrix(list(eye), list(target), [0, 0, 1])
    proj = p.computeProjectionMatrixFOV(fov, width / height, .02, 8)
    rgba = p.getCameraImage(width, height, view, proj, renderer=p.ER_TINY_RENDERER, lightDirection=[-3, -4, 7], shadow=1,
                            lightAmbientCoeff=.55, lightDiffuseCoeff=.6, lightSpecularCoeff=.2, physicsClientId=world.cid)[2]
    return np.reshape(np.asarray(rgba, dtype=np.uint8), (height, width, 4))[:, :, :3]


def phases(scene, path, world):
    """State indices: start, aligned in front of the first opening, inside the constraint, goal."""
    poses = []
    for q in path:
        world.set_q(q)
        poses.append(world.place_payload())
    poses = np.array(poses)
    portal = scene['task']['portals'][0]
    x = apply(inverse(np.asarray(portal['pose'])), reference_points(poses, scene['payload']['size'], portal.get('point', 'front')))[:, 0]
    crossing = int(np.flatnonzero((x[:-1] <= 0) & (x[1:] > 0))[0])
    aligned = int(np.flatnonzero(x[:crossing + 1] <= -.02)[-1]) if np.any(x[:crossing + 1] <= -.02) else max(0, crossing - 5)
    inside = (crossing + len(path) - 1) // 2
    return [0, aligned, inside, len(path) - 1]


def cameras(scene):
    g = np.asarray(scene['task']['goal']['object_pose'])[:3, 3]
    radial = g[:2] / max(np.linalg.norm(g[:2]), 1e-6)
    side = np.array([-radial[1], radial[0]])
    az = np.arctan2(radial[1], radial[0])
    rot = np.array([[np.cos(az), -np.sin(az)], [np.sin(az), np.cos(az)]])
    target = np.r_[.55 * g[:2], max(g[2] - .1, -.1)]
    wide = (np.r_[target[:2] + rot @ np.array([-1.55, -1.45]), target[2] + 1.25], target)
    fam = scene['family']
    up = np.array([0., 0., 1.])
    axis = np.asarray(scene['task']['goal']['object_pose'])[:3, 0]
    a_h = np.r_[axis[:2], 0.] / max(np.linalg.norm(axis[:2]), 1e-6)
    lateral = np.array([-a_h[1], a_h[0], 0.])
    if fam == 'tote':
        close = (np.r_[g[:2] - .30 * radial + .22 * side, g[2] + .55], np.r_[g[:2], g[2] - .05])
    elif fam == 'lathe' and is_v04(scene):
        door = np.asarray(scene['task']['portals'][0]['pose'])
        mid = door[:3, 3] + .14 * door[:3, 0]
        mid[2] = g[2]
        close = (mid + .62 * lateral + .18 * up - .05 * door[:3, 0], mid + .02 * door[:3, 0])
    elif fam == 'lathe':
        door = np.asarray(scene['task']['portals'][0]['pose'])[:3, 0]
        close = (g + .30 * door + .72 * up - .10 * axis, g + .04 * axis)
    elif fam == 'rack':
        close = (g - .78 * a_h + .10 * lateral + .62 * up, g + .10 * a_h - .03 * up)
    else:
        centre = np.mean([np.asarray(pt['pose'])[:3, 3] for pt in scene['task']['portals']], axis=0)
        close = (centre + .55 * lateral - .30 * a_h + .42 * up, centre + .02 * a_h)
    return wide, close


def annotate(img, title, footer):
    img = Image.fromarray(img)
    w, h = img.size
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, w, 60], fill='#102431')
    d.text((22, 15), title, font=font(26, True), fill='white')
    d.rectangle([0, h - 38, w, h], fill='#eef2f3')
    d.text((18, h - 31), footer, font=font(17), fill='#354853')
    return img


def footer_text(scene, hidden=False):
    ds = scene['design']
    c = 1000 * ds['clearance_per_side_m']
    notes = HIDDEN_NOTE_V04 if is_v04(scene) else HIDDEN_NOTE
    note = f"  |  {notes[scene['family']]}" if hidden and scene['family'] in notes else ''
    return f"{scene['id']}  |  clearance {c:.0f} mm/side at {ds['tightest_feature']}  |  payload {1000 * scene['payload']['size'][0]:.0f} mm{note}"


def render_task(job):
    task_file, witness_file, out, width, height = job
    scene = json.loads(Path(task_file).read_text())
    path = np.load(witness_file)
    fam = scene['family']
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    world = SuiteWorld(scene)
    try:
        idx = phases(scene, path, world)
        (wide_eye, wide_t), (close_eye, close_t) = cameras(scene)
        start = snapshot(world, path[idx[0]], wide_eye, wide_t, width, height)
        aligned = snapshot(world, path[idx[1]], wide_eye, wide_t, width, height)
        for name in (HIDE_FOR_CLOSEUP_V04 if is_v04(scene) else HIDE_FOR_CLOSEUP).get(fam, ()):
            for body, n in list(world.names.items()):
                if n == name:
                    p.removeBody(body, physicsClientId=world.cid)
        inside = snapshot(world, path[idx[2]], close_eye, close_t, width, height)
        goal = snapshot(world, path[idx[3]], close_eye, close_t, width, height)
    finally:
        world.close()
    hidden = fam in (HIDE_FOR_CLOSEUP_V04 if is_v04(scene) else HIDE_FOR_CLOSEUP)
    titles = FAMILY_TITLE_V04 if is_v04(scene) else FAMILY_TITLE
    panels = [annotate(start, f'1  Payload grasped at the source  -  {titles[fam]}', footer_text(scene)),
              annotate(aligned, '2  Aligned in front of the opening', footer_text(scene)),
              annotate(inside, '3  Inside the constrained passage', footer_text(scene, hidden)),
              annotate(goal, '4  Goal pose reached', footer_text(scene, hidden))]
    board = Image.new('RGB', (2 * width, 2 * height), 'white')
    for i, img in enumerate(panels):
        board.paste(img, ((i % 2) * width, (i // 2) * height))
    stem = scene['id']
    board.save(out / f'{stem}_workflow.png')
    panels[2].save(out / f'{stem}_inside.png')
    panels[3].save(out / f'{stem}_goal.png')
    panels[1].save(out / f'{stem}_aligned.png')
    return scene['id'], fam, scene['level']


def grid(images, cols, cell, title=None):
    rows = (len(images) + cols - 1) // cols
    top = 70 if title else 0
    board = Image.new('RGB', (cols * cell[0], rows * cell[1] + top), 'white')
    if title:
        d = ImageDraw.Draw(board)
        d.rectangle([0, 0, board.size[0], top], fill='#0b1a24')
        d.text((24, 18), title, font=font(30, True), fill='white')
    for i, img in enumerate(images):
        board.paste(img.resize(cell, Image.LANCZOS), ((i % cols) * cell[0], (i // cols) * cell[1] + top))
    return board


def main():
    ap = argparse.ArgumentParser(description='Render gosuite screenshots.')
    ap.add_argument('--suite', default='data/suite_v0.3')
    ap.add_argument('--output', default='previews/suite_v0.3')
    ap.add_argument('--split', default='test')
    ap.add_argument('--index', type=int, default=0)
    ap.add_argument('--width', type=int, default=1200)
    ap.add_argument('--height', type=int, default=900)
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--panels', help='also keep the per-task panels in this directory')
    a = ap.parse_args()
    suite, out = Path(a.suite), Path(a.output)
    out.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((suite / 'manifest.json').read_text())
    chosen = [e for e in manifest['tasks'] if e['split'] == a.split and e['index'] == a.index]
    with tempfile.TemporaryDirectory() as tmp:
        tasks = Path(a.panels) if a.panels else Path(tmp)
        jobs = [(suite / e['file'], suite / e['witness_file'], tasks, a.width, a.height) for e in chosen]
        with Pool(a.workers) as pool:
            done = pool.map(render_task, jobs)
        families = list(dict.fromkeys(f for _, f, _ in done))
        levels = ['easy', 'medium', 'hard']
        by = {(f, l): i for i, f, l in done}
        for fam in families:
            Image.open(tasks / f"{by[(fam, 'medium')]}_workflow.png").save(out / f'{fam}_workflow.png', optimize=True)
        version = manifest.get('suite_version', 'v0.3')
        grid([Image.open(tasks / f"{by[(f, 'medium')]}_inside.png") for f in families], 2, (a.width, a.height),
             f'gosuite {version} - four application families (medium level, test split)').save(out / 'overview.png', optimize=True)
        grid([Image.open(tasks / f"{by[(f, l)]}_goal.png") for f in families for l in levels], 3, (a.width * 2 // 3, a.height * 2 // 3),
             'Difficulty levels: clearance ' + ' / '.join(f"{1000 * manifest['levels'][l]['clearance_per_side_m']:.0f}" for l in levels)
             + ' mm per side (columns), one row per family').save(out / 'levels.png', optimize=True)
    print('rendered', len(done), 'tasks into', out)


if __name__ == '__main__':
    main()
