"""Render benchmark screenshots from the actual simulator geometry (PyBullet TinyRenderer).

    python -m gosuite.render --suite data/suite_v0.3 --output previews/suite_v0.3

Produces, for representative tasks:
  <family>_workflow.png  four phases (start, aligned at the opening, inside the constraint, goal)
  scenes.png             paper-style figure: whole station per family, payload motion as a translucent swept volume
  overview.png           one close-up per family
  levels.png             family x difficulty grid of goal close-ups
Close-ups may hide occluding parts (machine roof, upper rack shelf); captions say so.
"""
import argparse
import json
import tempfile
import zlib
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


# Render-only materials. Task files keep their own diagnostic colours (and their hashes); screenshots use
# procedural textures of industrial materials instead. Names are matched by prefix first, then roles.
MATERIAL = dict(
    worktop=(.66, .52, .37), powder_coat=(.34, .36, .38), black_oxide=(.17, .18, .19), dark_steel=(.22, .23, .24),
    machined_al=(.66, .67, .68), bright_steel=(.66, .68, .70), aluminium=(.76, .77, .78), mill_steel=(.50, .52, .54),
    galvanised=(.60, .62, .63), rack_blue=(.09, .25, .43), rack_orange=(.76, .37, .14), safety_yellow=(.84, .66, .16),
    rust=(.45, .27, .16), stand_paint=(.30, .33, .32), wire=(.16, .16, .17), machine_grey=(.82, .83, .81),
    cast_iron=(.30, .31, .32), hose_blue=(.16, .37, .68), stainless=(.64, .65, .66), tote_blue=(.14, .34, .60),
    tote_lid=(.11, .28, .50))
TEXTURE_KIND = dict(worktop='wood', rust='rust', bright_steel='brushed', aluminium='brushed', machined_al='brushed',
                    stainless='brushed', mill_steel='brushed', galvanised='galvanised', black_oxide='dark', dark_steel='dark',
                    cast_iron='dark', wire='dark', tote_blue='plastic', tote_lid='plastic', hose_blue='plastic')   # others: paint
NAME_MATERIAL = [('robot_pedestal', 'dark_steel'), ('robot_base_plate', 'machined_al'), ('source_table', 'worktop'),
                 ('tote_table_leg', 'powder_coat'), ('tote_table', 'worktop'), ('source_leg', 'powder_coat'),
                 ('source_cradle', 'black_oxide'), ('source_holder', 'black_oxide'), ('source_waiting', 'mill_steel'),
                 ('upright', 'rack_blue'), ('foot', 'rack_blue'), ('beam', 'rack_orange'), ('deck', 'galvanised'),
                 ('rear_guard', 'powder_coat'), ('bracket', 'safety_yellow'), ('target_saddle', 'black_oxide'),
                 ('target_guide', 'black_oxide'), ('stock', 'aluminium'), ('longitudinal', 'rust'), ('stirrup', 'rust'),
                 ('stand_', 'stand_paint'), ('tie_hook', 'wire'), ('side_clip', 'wire'), ('machine_bed', 'cast_iron'),
                 ('coolant_nozzle', 'hose_blue'), ('chip_guard', 'stainless'), ('fixture', 'black_oxide'), ('chuck', 'black_oxide'),
                 ('lid_flap', 'tote_lid'), ('cell_ledge', 'tote_blue'), ('tote_floor', 'tote_blue'), ('tote_wall', 'tote_blue'),
                 ('kitted_part', 'machined_al')]
ROLE_MATERIAL = dict(machine_enclosure='machine_grey', machine_structure='cast_iron', rack_structure='rack_blue',
                     bin_divider='galvanised', neighbor_stock='aluminium', cage_structure='rust', cage_support='stand_paint',
                     station_table='worktop', tote='tote_blue', robot_mount='dark_steel', source_fixture='powder_coat',
                     source_stock='mill_steel', target_guide='black_oxide', target_support='black_oxide')
ROLE_MATERIAL_BY_FAMILY = {('rack', 'intrusion'): 'safety_yellow', ('rebar', 'intrusion'): 'wire',
                           ('tote', 'intrusion'): 'tote_lid', ('tote', 'bin_divider'): 'tote_blue'}
TRAIL_COLOR = np.array([240., 172., 38.])   # translucent amber swept volume of the payload
_TEXTURES, _TEX_DIR = {}, []


def material_key(scene, name, role):
    if (scene['family'], role) in ROLE_MATERIAL_BY_FAMILY:
        return ROLE_MATERIAL_BY_FAMILY[(scene['family'], role)]
    if scene['family'] == 'tote' and name.startswith('divider'):
        return 'tote_blue'
    return next((m for prefix, m in NAME_MATERIAL if name.startswith(prefix)), None) or ROLE_MATERIAL.get(role)


def texture_image(key, size=1024):
    """Procedural texture (wood grain, rust, brushed or galvanised metal, oxide, plastic, paint), deterministic."""
    from scipy.ndimage import gaussian_filter
    rng = np.random.default_rng(zlib.crc32(key.encode()))
    noise = lambda sigma: (lambda a: a / (a.std() + 1e-9))(gaussian_filter(rng.normal(size=(size, size)), sigma, mode='wrap'))
    base = np.array(MATERIAL[key]) * 255
    kind = TEXTURE_KIND.get(key, 'paint')
    if kind == 'wood':
        y = np.mgrid[0:size, 0:size][0]
        rings = (np.sin(y / 9.6 + 2.5 * noise((80, 280)) + .6 * noise((4, 100))) + 1) / 2
        img = base * (1 - .24 * rings ** 2 + .03 * noise(2))[..., None]
    elif kind == 'rust':
        blot, speck = noise(36), noise(3)
        orange, dark = np.array([158., 86., 40.]), np.array([70., 40., 26.])
        mix = np.clip(.5 + .3 * blot, 0, 1)[..., None]
        img = (dark * (1 - mix) + orange * mix) * (1 + .10 * speck)[..., None] * .95 + base * .05
    elif kind == 'brushed':
        img = base * (1 + .05 * noise((1.5, 180)) + .015 * noise(1.5))[..., None]
    elif kind == 'galvanised':
        img = base * (1 + .012 * noise(16) + .01 * noise(2))[..., None]
    elif kind == 'dark':
        img = base * (1 + .05 * noise(24) + .03 * noise(2))[..., None]
    elif kind == 'plastic':
        img = base * (1 + .02 * noise(2))[..., None]
    else:   # paint: faint orange peel and wear
        img = base * (1 + .006 * noise(56) + .007 * noise(2))[..., None]
    return np.clip(img, 0, 255).astype(np.uint8)


def texture(world, key):
    if (world.cid, key) not in _TEXTURES:
        if not _TEX_DIR:
            _TEX_DIR.append(tempfile.mkdtemp(prefix='gosuite_tex_'))
        path = Path(_TEX_DIR[0]) / f'{key}.png'
        if not path.exists():
            Image.fromarray(texture_image(key)).save(path)
        _TEXTURES[(world.cid, key)] = p.loadTexture(str(path), physicsClientId=world.cid)
    return _TEXTURES[(world.cid, key)]


def _paint(world, body, key=None, rgba=None):
    for i, shape in enumerate(p.getVisualShapeData(body, physicsClientId=world.cid)):
        if key is not None:
            p.changeVisualShape(body, -1, shapeIndex=i, textureUniqueId=texture(world, key), rgbaColor=[1, 1, 1, 1],
                                physicsClientId=world.cid)
        else:   # unknown object: keep its hue, muted towards grey
            grey = float(np.dot(shape[7][:3], [.3, .59, .11]))
            p.changeVisualShape(body, -1, shapeIndex=i, rgbaColor=[.45 * c + .55 * grey for c in shape[7][:3]] + [1.],
                                physicsClientId=world.cid)


def apply_materials(world, scene, floor=False):
    """Textured industrial materials; the floor is removed (objects on a clean white background)."""
    for body, name in list(world.names.items()):
        role = world.roles.get(body)
        if role == 'floor' or name == 'floor':
            if not floor:
                p.removeBody(body, physicsClientId=world.cid)
                del world.names[body]
            continue
        _paint(world, body, material_key(scene, name, role))
    _paint(world, world.payload, 'aluminium' if 'alumin' in scene['payload'].get('material', '') else 'bright_steel')


def font(size, bold=False):
    name = 'DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf'
    try:
        return ImageFont.truetype(f'/usr/share/fonts/truetype/dejavu/{name}', size)
    except OSError:
        return ImageFont.load_default()


def render(world, eye, target, width, height, fov=42):
    view = p.computeViewMatrix(list(eye), list(target), [0, 0, 1])
    proj = p.computeProjectionMatrixFOV(fov, width / height, .02, 8)
    img = p.getCameraImage(width, height, view, proj, renderer=p.ER_TINY_RENDERER, lightDirection=[-3, -4, 7], shadow=1,
                           lightColor=[1., .98, .95], lightAmbientCoeff=.5, lightDiffuseCoeff=.55, lightSpecularCoeff=.12,
                           physicsClientId=world.cid)
    rgb = np.reshape(np.asarray(img[2], dtype=np.uint8), (height, width, 4))[:, :, :3].copy()
    seg = np.reshape(np.asarray(img[4]), (height, width))
    rgb[seg < 0] = 255
    return rgb, seg


def snapshot(world, q, eye, target, width, height, fov=42, trail=None, alpha=.14):
    """Image at configuration q. ``trail``: payload poses drawn as a translucent amber swept volume."""
    world.set_q(q)
    world.place_payload()
    rgb, seg = render(world, eye, target, width, height, fov)
    if trail is not None and len(trail):
        held = world.payload_T
        count, light = np.zeros(seg.shape), np.zeros(seg.shape)
        for pose in trail:
            world.place_payload_pose(pose)
            img, s = render(world, eye, target, width, height, fov)
            m = s == world.payload
            count += m
            light += m * img.mean(axis=2) / 255
        world.place_payload_pose(held)
        cover = (1 - (1 - alpha) ** count)[..., None]
        shade = (.62 + .5 * light / np.maximum(count, 1))[..., None]
        ghost = np.clip(TRAIL_COLOR * shade, 0, 255)
        keep = (seg == world.payload)[..., None]   # the held payload itself stays opaque
        rgb = np.where(keep, rgb, rgb * (1 - cover) + ghost * cover).astype(np.uint8)
    return rgb


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


def frame_scene(world, eye, target, aspect, payload_poses=(), fov=42, fill=.9):
    """Camera along the eye->target direction that fits every body (and the payload's path) into the image."""
    boxes = [p.getAABB(b, physicsClientId=world.cid) for b in world.names]
    boxes += [p.getAABB(world.robot, l, physicsClientId=world.cid) for l in range(-1, p.getNumJoints(world.robot, physicsClientId=world.cid))]
    pts = np.array([[c[i][0], c[j][1], c[k][2]] for c in boxes for i in (0, 1) for j in (0, 1) for k in (0, 1)])
    if len(payload_poses):
        pts = np.vstack([pts, np.array([t[:3, 3] for t in payload_poses])])
    centre = (pts.min(0) + pts.max(0)) / 2
    back = np.asarray(eye, float) - np.asarray(target, float)
    back /= np.linalg.norm(back)
    right = np.cross([0, 0, 1.], -back)
    right /= np.linalg.norm(right)
    up = np.cross(-back, right)
    t_y = np.tan(np.radians(fov) / 2)
    dist = 2.
    for _ in range(30):
        rel = pts - (centre + back * dist)
        z = np.maximum(rel @ -back, 1e-3)
        sx = np.abs(rel @ right / z) / (t_y * aspect)
        sy = np.abs(rel @ up / z) / t_y
        worst = max(sx.max(), sy.max()) / fill
        dist *= worst ** .5 if worst > 1 else max(worst, .7) ** .5
        if abs(worst - 1) < .01:
            break
    return centre + back * dist, centre


def fit_to_content(img, width, height, margin=.04):
    """Crop to the non-white content, then centre it on a white width x height canvas."""
    ys, xs = np.nonzero((img < 250).any(axis=2))
    crop = Image.fromarray(img[ys.min():ys.max() + 1, xs.min():xs.max() + 1])
    scale = min(width * (1 - 2 * margin) / crop.width, height * (1 - 2 * margin) / crop.height)
    crop = crop.resize((round(crop.width * scale), round(crop.height * scale)), Image.LANCZOS)
    canvas = Image.new('RGB', (width, height), 'white')
    canvas.paste(crop, ((width - crop.width) // 2, (height - crop.height) // 2))
    return np.asarray(canvas)


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
        apply_materials(world, scene)
        idx = phases(scene, path, world)
        poses = []
        for q in path:
            world.set_q(q)
            poses.append(world.place_payload())
        pick = lambda a, b, n: [poses[i] for i in np.unique(np.linspace(a, b, n).round().astype(int))]
        (wide_eye, wide_t), (close_eye, close_t) = cameras(scene)
        start = snapshot(world, path[idx[0]], wide_eye, wide_t, width, height)
        aligned = snapshot(world, path[idx[1]], wide_eye, wide_t, width, height)
        scene_eye, scene_t = frame_scene(world, wide_eye, wide_t, width / height, poses)
        whole = snapshot(world, path[idx[2]], scene_eye, scene_t, 2 * width, 2 * height, trail=pick(0, len(path) - 1, 48), alpha=.10)
        whole = fit_to_content(whole, width, height)
        for name in (HIDE_FOR_CLOSEUP_V04 if is_v04(scene) else HIDE_FOR_CLOSEUP).get(fam, ()):
            for body, n in list(world.names.items()):
                if n == name:
                    p.removeBody(body, physicsClientId=world.cid)
        inside = snapshot(world, path[idx[2]], close_eye, close_t, width, height)
        goal = snapshot(world, path[idx[3]], close_eye, close_t, width, height, trail=pick(idx[1], len(path) - 1, 18))
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
    Image.fromarray(whole).save(out / f'{stem}_scene.png')
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


def labelled_grid(images, labels, cols, cell, pad=14):
    """Paper-style figure: panels on white with (a), (b), ... captions underneath."""
    rows = (len(images) + cols - 1) // cols
    cap = 54
    board = Image.new('RGB', (cols * cell[0], rows * (cell[1] + cap)), 'white')
    d = ImageDraw.Draw(board)
    for i, (img, label) in enumerate(zip(images, labels)):
        x, y = (i % cols) * cell[0], (i // cols) * (cell[1] + cap)
        board.paste(img.resize(cell, Image.LANCZOS), (x, y))
        d.text((x + pad, y + cell[1] + 10), f'({chr(97 + i)}) {label}', font=font(28, True), fill='#111111')
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
        titles = FAMILY_TITLE_V04 if version == 'v0.4' else FAMILY_TITLE
        labelled_grid([Image.open(tasks / f"{by[(f, 'medium')]}_scene.png") for f in families],
                      [titles[f].split(' ', 1)[1] for f in families], 2, (a.width, a.height)).save(out / 'scenes.png', optimize=True)
        grid([Image.open(tasks / f"{by[(f, 'medium')]}_inside.png") for f in families], 2, (a.width, a.height),
             f'gosuite {version} - four application families (medium level, test split)').save(out / 'overview.png', optimize=True)
        grid([Image.open(tasks / f"{by[(f, l)]}_goal.png") for f in families for l in levels], 3, (a.width * 2 // 3, a.height * 2 // 3),
             'Difficulty levels: clearance ' + ' / '.join(f"{1000 * manifest['levels'][l]['clearance_per_side_m']:.0f}" for l in levels)
             + ' mm per side (columns), one row per family').save(out / 'levels.png', optimize=True)
    print('rendered', len(done), 'tasks into', out)


if __name__ == '__main__':
    main()
