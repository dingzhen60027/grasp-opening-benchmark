"""F2 rebar: thread a bar through the coaxial stirrup windows of a prefabricated cage.

Application: reinforcement-cage assembly at UR5e scale (analogue of TAPOM's 'Rebar Assembly').
Each stirrup frame offers two side-by-side windows; the target window is the threading channel,
the other one is often already occupied by a bar.  The bar must cross every target window in
order (portals) and never enter the neighbouring window (keep-in).  Window clearance per side is
the difficulty level; angular error accumulates over 2-4 windows spread along the bar.
"""
import numpy as np
from ..geometry import NEIGHBOR, STEEL, box_at, compound, frame, from_rt, part, tool_horizontal
from . import common
from .common import FLOOR_Z, InfeasibleDesign, r4

NAME = 'rebar'
DESCRIPTION = 'Thread a bar through 2-4 coaxial stirrup windows of a cage without entering the neighbouring window'
W = H = .036
OVERHANG = .02
BAR = .012          # stirrup / longitudinal rebar section (square proxy)
TIP_CLEARANCE = .03
GRIPPER_STANDOFF = .065  # TCP stays this far in front of the first stirrup
TIGHT_ROLES = ('cage_structure', 'neighbor_stock')


def sample(rng, clearance):
    return dict(clearance_m=clearance, length=r4(rng.uniform(.42, .54)), overhang=OVERHANG, rings=int(rng.integers(2, 5)),
                spacing=r4(rng.uniform(.08, .14)), rho=r4(rng.uniform(.44, .54)), azimuth=r4(rng.uniform(-.45, .35)),
                skew=r4(rng.uniform(-.25, .25)), height=r4(rng.uniform(.12, .30)), tip_beyond=r4(rng.uniform(.03, .06)),
                neighbor_side=int(rng.choice([-1, 1])), neighbor_occupied=bool(rng.random() < .6))


def build(params):
    c, L, oh = params['clearance_m'], params['length'], params['overhang']
    lever = L - oh
    k_rings, s = params['rings'], params['spacing']
    while k_rings > 2 and lever < (k_rings - 1) * s + BAR + params['tip_beyond'] + TIP_CLEARANCE + GRIPPER_STANDOFF:
        k_rings -= 1
    span = (k_rings - 1) * s + BAR
    if lever < span + params['tip_beyond'] + TIP_CLEARANCE + GRIPPER_STANDOFF:
        raise InfeasibleDesign('bar too short for the cage span')
    psi = params['azimuth'] + params['skew']
    d = np.array([np.cos(psi), np.sin(psi), 0.])
    z = params['height']
    ee_pre = np.array([params['rho'] * np.cos(params['azimuth']), params['rho'] * np.sin(params['azimuth']), z])
    tip_pre = ee_pre + lever * d
    first = tip_pre + (TIP_CLEARANCE + BAR / 2) * d
    station = frame(first, psi)  # x along the bar axis, origin at the centre of the first target window
    win = W + 2 * c
    sigma = params['neighbor_side']
    y_n = sigma * (win + BAR)
    y_lo, y_hi = min(0., y_n) - win / 2 - BAR, max(0., y_n) + win / 2 + BAR
    z_out = win / 2 + BAR

    st = []
    ring_parts = [part([BAR, y_hi - y_lo, BAR], [0, (y_lo + y_hi) / 2, s_z * (win / 2 + BAR / 2)], STEEL) for s_z in (-1, 1)]
    for y in (y_lo + BAR / 2, sigma * (win / 2 + BAR / 2), y_hi - BAR / 2):
        ring_parts.append(part([BAR, BAR, win], [0, y, 0], STEEL))
    # Target stirrups 0..K-1 plus one further stirrup that the bar does not reach (the cage continues);
    # it stays >= 45 mm beyond the final bar tip so that it never becomes an unintended tight feature.
    x_extra = (k_rings - 1) * s + max(s, BAR + params['tip_beyond'] + .045)
    ring_x = [k * s for k in range(k_rings)] + [x_extra]
    for k, x in enumerate(ring_x):
        st.append(compound(f'stirrup_{k}', [dict(p) for p in ring_parts], station @ from_rt(np.eye(3), [x, 0, 0]), 'cage_structure'))
    x0, x1 = -.05, x_extra + .08
    for yi, y in enumerate((y_lo + BAR / 2, y_hi - BAR / 2)):
        for zi, zz in enumerate((-(z_out + BAR / 2), z_out + BAR / 2)):
            st.append(box_at(f'longitudinal_{yi}{zi}', [(x0 + x1) / 2, y, zz], [x1 - x0, BAR, BAR], STEEL, 'cage_structure', station))
    if params['neighbor_occupied']:
        st.append(box_at('threaded_bar', [x_extra / 2 + .02, y_n, 0], [x_extra + .24, W, H], NEIGHBOR, 'neighbor_stock', station))
    bottom = z - z_out - BAR
    if bottom - FLOOR_Z < .1:
        raise InfeasibleDesign('cage too low')
    # Steel stands (cross beam + two legs) under the first and the extra stirrup.
    for k, x in enumerate((0., x_extra)):
        st.append(box_at(f'stand_beam_{k}', [x, (y_lo + y_hi) / 2, bottom - .015 - z], [.04, y_hi - y_lo + .16, .03], STEEL, 'cage_support', station))
        for side, y in (('a', y_lo - .06), ('b', y_hi + .06)):
            st.append(box_at(f'stand_leg_{k}{side}', [x, y, (FLOOR_Z + bottom - .03) / 2 - z], [.035, .035, bottom - .03 - FLOOR_Z],
                             STEEL, 'cage_support', station))
    obs = common.tag(common.cell(), 'cell') + common.tag(common.horizontal_source(lever)[0], 'source') + common.tag(st, 'station')
    start = common.horizontal_source(lever)[1]

    tip_final = first + ((k_rings - 1) * s + BAR / 2 + params['tip_beyond']) * d
    ee_final = tip_final - lever * d
    z_tr = max(.34, z + z_out + BAR + H / 2 + .06)
    src = common.SOURCE_TCP
    knots = [start, tool_horizontal([src[0], src[1], z_tr], common.SOURCE_YAW), tool_horizontal([src[0], src[1], z_tr], psi),
             tool_horizontal([ee_pre[0], ee_pre[1], z_tr], psi), tool_horizontal(ee_pre, psi), tool_horizontal(ee_final, psi)]
    portals = [common.portal(f'stirrup_{k}', station @ from_rt(np.eye(3), [k * s, 0, 0]), win, win, 'front') for k in range(k_rings)]
    task = dict(operation='thread', description=DESCRIPTION,
                start_phase='payload grasped 80 mm above the source nest', goal_phase='bar threaded through every target window',
                goal=common.goal(.012, 5.),
                keep_in=[dict(name='cage_target_window',
                              restricted=common.station_box(station, [-BAR / 2, y_lo, -z_out], [(k_rings - 1) * s + BAR / 2, y_hi, z_out]),
                              allowed=common.station_box(station, [-BAR / 2, -win / 2, -win / 2], [(k_rings - 1) * s + BAR / 2, win / 2, win / 2]),
                              tolerance_m=.001)],
                portals=portals, placement=None)
    params = dict(params, rings=k_rings)
    record = common.scene_record(NAME, params, obs, common.payload('solid_square_bar', L, W, H, oh, None, 'steel (nominal)', 7850.),
                                 task, DESCRIPTION)
    record['design'].update(window_m=r4(win), rings=k_rings, cage_axis_yaw=r4(psi), tightest_feature='stirrup windows')
    return record, knots, [common.SEED_SOURCE_HORIZONTAL, common.SEED_HOME]
