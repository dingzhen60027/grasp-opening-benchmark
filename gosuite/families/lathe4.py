"""F3 lathe (v0.4): load a bar axially through a machine door onto fixture pads, past machine fittings.

Planner-controllable risk in the vertical plane: a coolant nozzle hangs from the roof just inside the
door and a chip-guard lip rises from below deeper inside, on the opposite side.  The goal line passes
both with clearance c (the level).  A straight axial insertion passes both with the bar TIP at
clearance c; a risk-aware insertion lifts / lowers and pitches the bar so that only short-lever
points pass them at small clearance.  The bar ends 14 mm above two pads with 25 mm side guides, so
the goal state itself carries little risk.
"""
import numpy as np
from ..geometry import DARK, ORANGE, TEAL, box_at, frame, from_rt, tool_horizontal
from . import common
from .common import FLOOR_Z, InfeasibleDesign, r4

NAME = 'lathe'
DESCRIPTION = 'Load a bar axially through a machine door onto fixture pads past a coolant nozzle (above) and a chip-guard lip (below)'
W = H = .036
OVERHANG = .02
WALL = .02
STANDOFF = .03          # TCP stays this far outside the door plane at the goal
PRE_RELEASE_GAP = .014
GUIDE_SLACK = .025
TIP_CLEARANCE = .03
ENCLOSURE = [.80, .82, .80, 1.]
TIGHT_ROLES = ('intrusion', 'target_support', 'target_guide')


def sample(rng, clearance):
    return dict(clearance_m=clearance, length=r4(rng.uniform(.40, .48)), overhang=OVERHANG,
                rho_door=r4(rng.uniform(.86, .92)), azimuth=r4(rng.uniform(-.25, .30)), skew=r4(rng.uniform(-.10, .10)),
                z_goal=r4(rng.uniform(.18, .28)), door_width=r4(rng.uniform(.30, .36)), door_height=r4(rng.uniform(.30, .36)),
                shallow_depth=r4(rng.uniform(.03, .05)), deep_depth=r4(rng.uniform(.11, .14)))


def build(params):
    c, L, oh = params['clearance_m'], params['length'], params['overhang']
    lever = L - oh
    psi = params['azimuth'] + params['skew']
    origin = np.array([params['rho_door'] * np.cos(params['azimuth']), params['rho_door'] * np.sin(params['azimuth']), 0.])
    station = frame(origin, psi)                     # x into the machine, door plane at x = 0
    z_g = params['z_goal']
    wd, hd = params['door_width'], params['door_height']
    zd0, zd1 = z_g - hd / 2, z_g + hd / 2
    depth = lever + .30
    y_in = wd / 2 + .18
    roof = zd1 + .08
    x_tcp = -STANDOFF
    x_s, x_d = WALL + params['shallow_depth'], WALL + params['shallow_depth'] + params['deep_depth']
    if x_d - x_tcp > .24:
        raise InfeasibleDesign('deep fitting too far from the gripper at the goal')
    st = []
    y_out = y_in + WALL
    st += [box_at('front_wall_left', [WALL / 2, (-y_out - wd / 2) / 2, (FLOOR_Z + roof + WALL) / 2], [WALL, y_out - wd / 2, roof + WALL - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('front_wall_right', [WALL / 2, (wd / 2 + y_out) / 2, (FLOOR_Z + roof + WALL) / 2], [WALL, y_out - wd / 2, roof + WALL - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('front_wall_top', [WALL / 2, 0, (zd1 + roof + WALL) / 2], [WALL, wd, roof + WALL - zd1], ENCLOSURE, 'machine_enclosure', station),
           box_at('front_wall_bottom', [WALL / 2, 0, (FLOOR_Z + zd0) / 2], [WALL, wd, zd0 - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('back_wall', [WALL + depth + WALL / 2, 0, (FLOOR_Z + roof + WALL) / 2], [WALL, 2 * y_out, roof + WALL - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('side_wall_neg', [WALL + depth / 2, -y_in - WALL / 2, (FLOOR_Z + roof + WALL) / 2], [depth, WALL, roof + WALL - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('side_wall_pos', [WALL + depth / 2, y_in + WALL / 2, (FLOOR_Z + roof + WALL) / 2], [depth, WALL, roof + WALL - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('roof', [WALL + depth / 2, 0, roof + WALL / 2], [depth, 2 * y_in, WALL], ENCLOSURE, 'machine_enclosure', station)]
    bed_top = z_g - H / 2 - PRE_RELEASE_GAP - .03
    st.append(box_at('machine_bed', [WALL + depth / 2, 0, (FLOOR_Z + bed_top) / 2], [depth, 2 * y_in, bed_top - FLOOR_Z], DARK, 'machine_structure', station))
    # One fixture pad (slack placement) with side guides near the bar end, beyond the fittings.
    pad_top = z_g - H / 2 - PRE_RELEASE_GAP
    x_pad = x_tcp - oh + .85 * L
    if x_pad < x_d + .05:
        raise InfeasibleDesign('fixture pad overlaps the chip-guard lip')
    supports = ['fixture_pad']
    st.append(box_at('fixture_pad', [x_pad, 0, (bed_top + pad_top) / 2], [.05, .10, pad_top - bed_top], TEAL, 'target_support', station))
    for side in (-1, 1):
        st.append(box_at(f'fixture_guide_{"lr"[side > 0]}', [x_pad, side * (W / 2 + GUIDE_SLACK + .007), pad_top + .03],
                         [.035, .014, .06], TEAL, 'target_guide', station))
    # Intruding machine fittings: nozzle from the roof (shallow), chip-guard lip from the bed (deep).
    st.append(box_at('coolant_nozzle', [x_s, 0, (z_g + H / 2 + c + roof) / 2], [.03, .16, roof - (z_g + H / 2 + c)], ORANGE, 'intrusion', station))
    st.append(box_at('chip_guard_lip', [x_d, 0, (bed_top + z_g - H / 2 - c) / 2], [.03, .24, (z_g - H / 2 - c) - bed_top], ORANGE, 'intrusion', station))
    obs = common.tag(common.cell(), 'cell')
    src, start = common.horizontal_source(lever)
    obs += common.tag(src, 'source') + common.tag(st, 'station')

    to_world = lambda p: (station @ np.r_[p, 1.])[:3]
    pre = to_world([-(TIP_CLEARANCE + lever), 0, z_g])
    goal_line = to_world([x_tcp, 0, z_g])
    z_tr = max(.34, z_g + .08)
    src = common.SOURCE_TCP
    if np.hypot(pre[0], pre[1]) < .40:
        raise InfeasibleDesign('pre-insertion pose inside the inner reach limit')
    knots = [start, tool_horizontal([src[0], src[1], z_tr], common.SOURCE_YAW),
             tool_horizontal([pre[0], -.47, z_tr], common.SOURCE_YAW), tool_horizontal([pre[0], -.47, z_tr], psi),
             tool_horizontal([pre[0], pre[1], z_tr], psi), tool_horizontal(pre, psi), tool_horizontal(goal_line, psi)]
    task = dict(operation='load', description=DESCRIPTION,
                start_phase='payload grasped 80 mm above the source nest', goal_phase='bar 14 mm above the fixture pad',
                goal=common.goal(.012, 5.),
                portals=[common.portal('machine_door', station @ from_rt(np.eye(3), [WALL / 2, 0, (zd0 + zd1) / 2]), wd, hd, 'front')],
                placement=dict(supports=supports, gap_range_m=[.002, .030], edge_margin_m=.025, lateral_tolerance_m=.03))
    record = common.scene_record(NAME, params, obs, common.payload('solid_square_bar', L, W, H, oh, None, 'steel (nominal)', 7850.),
                                 task, DESCRIPTION)
    record['design'].update(door_width_m=r4(wd), door_height_m=r4(hd), tightest_feature='coolant nozzle (above) and chip-guard lip (below)',
                            goal_slack_m=GUIDE_SLACK, fitting_depths_from_tcp_goal_m=[r4(x_s - x_tcp), r4(x_d - x_tcp)])
    axis_frame = station.copy()
    axis_frame[:3, 3] = 0.
    depth_of = lambda x: float(np.dot(to_world([x, 0, z_g]), station[:3, 0]))
    robust = dict(segment=(5, 6), frame=axis_frame, max_offset=.04, lever=lever, planes=('z',),
                  event_depths=[depth_of(x_s), depth_of(x_d)], control_depths=[depth_of(x_s), depth_of(x_d)])
    return record, knots, [common.SEED_SOURCE_HORIZONTAL, common.SEED_HOME], robust
