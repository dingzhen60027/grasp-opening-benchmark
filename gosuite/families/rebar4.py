"""F2 rebar (v0.4): thread a bar into a cage past a tie hook (above) and a side clip.

The stirrup windows are generous (30 mm around the goal line), so threading them is not the tight part.
Inside the cage a tie-wire hook hangs from the top bar near the entrance (vertical constraint) and a
spacer clip protrudes from one side deeper in (lateral constraint).  The goal line passes both with
clearance c (the level).  The two constraints act in orthogonal planes, so a risk-aware path can give
the tip room in both while it passes them, and converge once only short-lever points are next to them.
"""
import numpy as np
from ..geometry import NEIGHBOR, ORANGE, STEEL, box_at, compound, frame, from_rt, part, tool_horizontal
from . import common
from .common import FLOOR_Z, InfeasibleDesign, r4

NAME = 'rebar'
DESCRIPTION = 'Thread a bar into a cage through generous stirrups past a tie hook (above) and a side clip'
W = H = .036
OVERHANG = .02
BAR = .012
WINDOW_SLACK = .045
TIP_CLEARANCE = .03
GRIPPER_STANDOFF = .05
TIGHT_ROLES = ('intrusion', 'cage_structure', 'neighbor_stock')


def sample(rng, clearance):
    return dict(clearance_m=clearance, length=r4(rng.uniform(.44, .54)), overhang=OVERHANG, rings=int(rng.integers(3, 5)),
                standoff=r4(rng.uniform(.08, .10)), rho=r4(rng.uniform(.44, .52)), azimuth=r4(rng.uniform(-.10, .50)),
                skew=r4(rng.uniform(-.2, .2)), height=r4(rng.uniform(.22, .32)), tip_beyond=r4(rng.uniform(.03, .06)),
                hook_depth=r4(rng.uniform(.02, .04)), lap_depth=r4(rng.uniform(.06, .09)), lap_side=int(rng.choice([-1, 1])))


def build(params):
    c, L, oh = params['clearance_m'], params['length'], params['overhang']
    lever = L - oh
    k_rings = params['rings']
    x_tcp = -(BAR / 2 + params['standoff'])                         # station x of the TCP at the goal
    tip_final = x_tcp + lever
    s = (tip_final - params['tip_beyond'] - BAR / 2) / (k_rings - 1)  # stirrups spread over the inserted length
    if s < .08:
        raise InfeasibleDesign('stirrups too close for this bar length')
    psi = params['azimuth'] + params['skew']
    d = np.array([np.cos(psi), np.sin(psi), 0.])
    z = params['height']
    ee_pre = np.array([params['rho'] * np.cos(params['azimuth']), params['rho'] * np.sin(params['azimuth']), z])
    first = ee_pre + (lever + TIP_CLEARANCE + BAR / 2) * d        # centre of the first stirrup
    station = frame(first, psi)
    span = (k_rings - 1) * s
    win = W + 2 * WINDOW_SLACK
    z_out = win / 2 + BAR
    st = []
    ring = [part([BAR, win + 2 * BAR, BAR], [0, 0, zz * (win / 2 + BAR / 2)], STEEL) for zz in (-1, 1)] + \
           [part([BAR, BAR, win], [0, yy * (win / 2 + BAR / 2), 0], STEEL) for yy in (-1, 1)]
    for k in range(k_rings):
        st.append(compound(f'stirrup_{k}', [dict(p) for p in ring], station @ from_rt(np.eye(3), [k * s, 0, 0]), 'cage_structure'))
    for yi, yy in enumerate((-1, 1)):
        for zi, zz in enumerate((-1, 1)):
            st.append(box_at(f'longitudinal_{yi}{zi}', [(span + .05) / 2, yy * (win / 2 + BAR / 2), zz * (z_out + BAR / 2)],
                             [span + .05, BAR, BAR], STEEL, 'cage_structure', station))
    x_hook = BAR / 2 + params['hook_depth']
    x_lap = x_hook + params['lap_depth']
    if x_lap - x_tcp > .23 or x_lap > span - .03:
        raise InfeasibleDesign('lap bar too deep for the bar length / cage')
    st.append(box_at('tie_hook', [x_hook, 0, (H / 2 + c + win / 2) / 2], [.012, .05, win / 2 - H / 2 - c], ORANGE, 'intrusion', station))
    side = params['lap_side']
    st.append(box_at('side_clip', [x_lap, side * (W / 2 + c + win / 2) / 2, 0], [.025, win / 2 - W / 2 - c, .03], ORANGE, 'intrusion', station))
    bottom = z - z_out - BAR
    if bottom - FLOOR_Z < .1:
        raise InfeasibleDesign('cage too low')
    for k, x in enumerate((0., span)):
        st.append(box_at(f'stand_beam_{k}', [x, 0, bottom - .015 - z], [.04, win + .2, .03], STEEL, 'cage_support', station))
        for sd, yy in (('a', -(win / 2 + .08)), ('b', win / 2 + .08)):
            st.append(box_at(f'stand_leg_{k}{sd}', [x, yy, (FLOOR_Z + bottom - .03) / 2 - z], [.035, .035, bottom - .03 - FLOOR_Z],
                             STEEL, 'cage_support', station))
    obs = common.tag(common.cell(), 'cell') + common.tag(common.horizontal_source(lever)[0], 'source') + common.tag(st, 'station')
    start = common.horizontal_source(lever)[1]
    ee_final = first + x_tcp * d
    z_tr = max(.34, z + z_out + BAR + H / 2 + .06)
    src = common.SOURCE_TCP
    knots = [start, tool_horizontal([src[0], src[1], z_tr], common.SOURCE_YAW), tool_horizontal([src[0], src[1], z_tr], psi),
             tool_horizontal([ee_pre[0], ee_pre[1], z_tr], psi), tool_horizontal(ee_pre, psi), tool_horizontal(ee_final, psi)]
    portals = [common.portal(f'stirrup_{k}', station @ from_rt(np.eye(3), [k * s, 0, 0]), win, win, 'front') for k in range(k_rings)]
    task = dict(operation='thread', description=DESCRIPTION,
                start_phase='payload grasped 80 mm above the source nest', goal_phase='bar threaded through every stirrup',
                goal=common.goal(.012, 5.), portals=portals, placement=None)
    record = common.scene_record(NAME, dict(params, rings=k_rings), obs,
                                 common.payload('solid_square_bar', L, W, H, oh, None, 'steel (nominal)', 7850.), task, DESCRIPTION)
    record['design'].update(window_m=r4(win), rings=k_rings, cage_axis_yaw=r4(psi), goal_slack_m=WINDOW_SLACK,
                            tightest_feature='tie hook (above) and side clip',
                            fitting_depths_from_tcp_goal_m=[r4(x_hook - x_tcp), r4(x_lap - x_tcp)])
    axis_frame = station.copy()
    axis_frame[:3, 3] = 0.
    depth_of = lambda x: float(np.dot((station @ np.r_[x, 0, 0, 1.])[:3], d))
    robust = dict(segment=(4, 5), frame=axis_frame, max_offset=WINDOW_SLACK + .005, lever=lever, planes=('y', 'z'),
                  event_depths=[depth_of(x_hook), depth_of(x_lap)], control_depths=[depth_of(x_hook), depth_of(x_lap)])
    return record, knots, [common.SEED_SOURCE_HORIZONTAL, common.SEED_HOME], robust
