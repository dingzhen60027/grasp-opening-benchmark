"""F1 rack (v0.4): deliver a tube into a bay past intruding brackets, onto saddles with slack.

Planner-controllable risk: two full-height brackets intrude into the target bay, a shallow one near
the bay mouth on one side and a deep one on the other side.  The goal line passes both with
clearance c (the level).  A straight insertion along the goal line passes both with the tube TIP
(largest lever arm) at clearance c; a risk-aware insertion steers the tip away and converges to the
goal line only when short-lever points are next to the brackets.  The saddles keep 25 mm guides and
a 2-30 mm pre-release window, so the goal state itself carries little risk.
"""
import numpy as np
from ..geometry import BLUE, NEIGHBOR, ORANGE, TEAL, box_at, compound, from_rt, hollow_tube_parts, tool_horizontal
from . import common
from .common import FLOOR_Z, InfeasibleDesign, r4

NAME = 'rack'
DESCRIPTION = 'Deliver a tube into the requested bay past intruding brackets (shallow and deep, opposite sides) onto two saddles'
BAY_PITCH = .26
HALF_WIDTH = .55
UPRIGHT = .045
X_PRE_MIN = .44
TIP_CLEARANCE = .025
W = H = .04
WALL = .003
OVERHANG = .02
GUIDE_SLACK = .025
PRE_RELEASE_GAP = .014
INSERT_GAP = .030
BRACKET_DEPTH = .03
TIGHT_ROLES = ('intrusion', 'target_guide', 'target_support')
SRC = common.SOURCE_TCP


def sample(rng, clearance):
    return dict(clearance_m=clearance, bay=int(rng.integers(-1, 2)), length=r4(rng.uniform(.40, .50)), overhang=OVERHANG,
                deck_z=r4(rng.uniform(.10, .18)), tier_h=r4(rng.uniform(.27, .32)), front_jitter=r4(rng.uniform(0., .02)),
                rear_depth=r4(rng.uniform(.03, .04)), shallow_side=int(rng.choice([-1, 1])),
                shallow_depth=r4(rng.uniform(.05, .07)), deep_depth=r4(rng.uniform(.22, .28)),
                divider_h=r4(rng.uniform(.16, .20)), stock_present=[bool(v) for v in rng.random(6) < .8],
                stock_length=[r4(v) for v in rng.uniform(.42, .56, 6)])


def build(params):
    c, L, oh = params['clearance_m'], params['length'], params['overhang']
    lever = L - oh
    y_t = params['bay'] * BAY_PITCH
    x_f = X_PRE_MIN + lever + TIP_CLEARANCE + UPRIGHT / 2 + params['front_jitter']
    face = x_f - UPRIGHT / 2
    depth = max(.58, L + .14)
    x_b = x_f + depth
    deck_z, th = params['deck_z'], params['tier_h']
    decks = [deck_z + k * th for k in (-2, -1, 0, 1)]
    top = decks[-1] + .20
    deck_top = deck_z + .009
    upper_bottom = decks[3] - .009
    upper_beam_bottom = decks[3] - .055
    saddle_top = deck_top + .012
    z_ins = saddle_top + INSERT_GAP + H / 2
    z_rel = saddle_top + PRE_RELEASE_GAP + H / 2
    if upper_beam_bottom - (z_ins + H / 2) < .05:
        raise InfeasibleDesign('tier too low for the insertion height')

    rear = x_f + params['rear_depth']          # tube rear end at the goal
    x_goal = rear + oh                          # TCP at the goal
    x_shallow = rear + params['shallow_depth']
    x_deep = rear + params['deep_depth']
    s = params['shallow_side']
    bay_half = BAY_PITCH / 2 - .006
    if bay_half - W / 2 - c < .03:
        raise InfeasibleDesign('bay too narrow for the brackets')

    obs = common.tag(common.cell(), 'cell')
    src, start = common.horizontal_source(lever)
    obs += common.tag(src, 'source')
    st = []
    for xi, x in enumerate((x_f, x_b)):
        for yi, y in enumerate((-HALF_WIDTH, HALF_WIDTH)):
            st.append(box_at(f'upright_{xi}{yi}', [x, y, (FLOOR_Z + top) / 2], [UPRIGHT, UPRIGHT, top - FLOOR_Z], BLUE, 'rack_structure'))
            st.append(box_at(f'foot_{xi}{yi}', [x, y, FLOOR_Z + .012], [.12, .11, .024], BLUE, 'rack_structure'))
    for k, z in enumerate(decks):
        st.append(box_at(f'deck_{k}', [(x_f + x_b) / 2, 0, z], [x_b - x_f, 2 * HALF_WIDTH - .04, .018], [.64, .68, .70, 1], 'rack_structure'))
        for xi, x in enumerate((x_f, x_b)):
            st.append(box_at(f'beam_{k}_{xi}', [x, 0, z - .025], [UPRIGHT, 2 * HALF_WIDTH, .06], ORANGE, 'rack_structure'))
    st.append(box_at('rear_guard', [x_b + .03, 0, (FLOOR_Z + top) / 2 + .05], [.012, 2 * HALF_WIDTH - .03, top - FLOOR_Z - .1],
                     [.27, .34, .37, 1], 'rack_structure'))
    div_x0, div_x1 = x_f + .06, x_b - .02
    for k, y in enumerate((-1.5 * BAY_PITCH, -.5 * BAY_PITCH, .5 * BAY_PITCH, 1.5 * BAY_PITCH)):
        st.append(box_at(f'divider_{k}', [(div_x0 + div_x1) / 2, y, deck_top + params['divider_h'] / 2],
                         [div_x1 - div_x0, .012, params['divider_h']], BLUE, 'bin_divider'))
    slot = 0
    for tier, z_deck in ((0, deck_z), (-1, decks[1])):
        for bay in (-1, 0, 1):
            if tier == 0 and bay == params['bay']:
                continue
            present, length = params['stock_present'][slot], params['stock_length'][slot]
            slot += 1
            if present:
                for lane, dy in enumerate((-.034, .034)):
                    st.append(compound(f'stock_{tier}_{bay}_{lane}', hollow_tube_parts(length, .04, .04, .003, NEIGHBOR),
                                       from_rt(np.eye(3), [x_f + .06 + length / 2, bay * BAY_PITCH + dy, z_deck + .029]), 'neighbor_stock'))
    # Intruding full-height brackets: faces at clearance c from the goal line.
    h0, h1 = deck_top, upper_bottom
    for name, x, side in (('bracket_shallow', x_shallow, s), ('bracket_deep', x_deep, -s)):
        y0, y1 = y_t + side * (W / 2 + c), y_t + side * bay_half
        st.append(box_at(name, [x, (y0 + y1) / 2, (h0 + h1) / 2], [BRACKET_DEPTH, abs(y1 - y0), h1 - h0], ORANGE, 'intrusion'))
    # Saddles with slack guides: one between the brackets, one near the tube end.
    supports = []
    for k, xs in enumerate(((x_shallow + x_deep) / 2, rear + .9 * L)):
        supports.append(f'target_saddle_{k}')
        st.append(box_at(f'target_saddle_{k}', [xs, y_t, deck_top + .006], [.05, .10, .012], TEAL, 'target_support'))
        for side in (-1, 1):
            st.append(box_at(f'target_guide_{k}_{"lr"[side > 0]}', [xs, y_t + side * (W / 2 + GUIDE_SLACK + .007), saddle_top + .03],
                             [.035, .014, .06], TEAL, 'target_guide'))
    if abs(((x_shallow + x_deep) / 2) - x_shallow) < .06 or rear + .9 * L > x_b - .04:
        raise InfeasibleDesign('saddle placement conflicts with the brackets or the rear guard')
    obs += common.tag(st, 'station')

    x_pre = face - TIP_CLEARANCE - lever
    z_tr = max(.34, z_ins + .06)
    knots = [start, tool_horizontal([SRC[0], SRC[1], z_tr], common.SOURCE_YAW),
             tool_horizontal([x_pre, -.47, z_tr], common.SOURCE_YAW), tool_horizontal([x_pre, -.47, z_tr], 0.),
             tool_horizontal([x_pre, y_t, z_tr], 0.), tool_horizontal([x_pre, y_t, z_ins], 0.),
             tool_horizontal([x_goal, y_t, z_ins], 0.), tool_horizontal([x_goal, y_t, z_rel], 0.)]
    world = np.eye(4)
    task = dict(operation='store', description=DESCRIPTION,
                start_phase='payload grasped 80 mm above the source nest', goal_phase='pre-release pose above both saddles',
                goal=common.goal(.012, 5.),
                keep_in=[dict(name='rack_target_bay',
                              restricted=common.station_box(world, [face, -HALF_WIDTH - UPRIGHT / 2, FLOOR_Z], [x_b + UPRIGHT / 2, HALF_WIDTH + UPRIGHT / 2, top]),
                              allowed=common.station_box(world, [face, y_t - bay_half, deck_top], [x_b + UPRIGHT / 2, y_t + bay_half, upper_bottom]),
                              tolerance_m=.001)],
                portals=[common.portal('bay_entry', from_rt(np.eye(3), [face, y_t, (deck_top + upper_beam_bottom) / 2]),
                                       2 * bay_half, upper_beam_bottom - deck_top, 'front')],
                placement=dict(supports=supports, gap_range_m=[.002, .030], edge_margin_m=.025, lateral_tolerance_m=.03))
    record = common.scene_record(NAME, params, obs, common.payload('hollow_square_tube', L, W, H, oh, WALL), task, DESCRIPTION)
    record['design'].update(rack_front_x=r4(x_f), target_bay_y=r4(y_t), tightest_feature='intruding brackets (shallow and deep)',
                            goal_slack_m=GUIDE_SLACK, bracket_depths_from_tcp_goal_m=[r4(x_shallow - x_goal), r4(x_deep - x_goal)])
    robust = dict(segment=(5, 6), frame=np.eye(4), active=('dy', 'yaw'), max_offset=min(.045, bay_half - W / 2 - .01),
                  lever=lever, event_depths=[x_shallow, x_deep], control_depths=[x_shallow, x_deep], planes=('y',))
    return record, knots, [common.SEED_SOURCE_HORIZONTAL, common.SEED_HOME], robust
