"""F1 rack: deliver a hollow tube into one bay of a multi-bay industrial rack.

Application: line-side / warehouse rack storage (analogue of TAPOM's 'Part Delivery through
industrial shelving').  Constraints: the tube must enter ONLY the requested bay (keep-in rule;
fixes the v0.2 wrong-bay sweep), pass between the guide ears of two destination saddles
(clearance per side = level) and finish at a pre-release pose 2-20 mm above both saddles.

Reach-aware layout: with the tool horizontal, the UR5e + Robotiq TCP reaches roughly
x = 0.42-1.06 m straight ahead.  The rack front is therefore placed so that the pre-insertion
pose (tube fully outside the facade, aligned with the bay) sits at x >= X_PRE_MIN.
"""
import numpy as np
from ..geometry import BLUE, NEIGHBOR, ORANGE, STEEL, TEAL, box_at, compound, frame, from_rt, hollow_tube_parts, tool_horizontal
from . import common
from .common import FLOOR_Z, InfeasibleDesign, r4

NAME = 'rack'
DESCRIPTION = 'Deliver a hollow tube into the requested bay of a three-bay rack; enter only that bay; guide ears and two saddles'
BAY_PITCH = .26
HALF_WIDTH = .55
UPRIGHT = .045
X_PRE_MIN = .44
TIP_CLEARANCE = .025  # tube tip to facade face before insertion
W = H = .04
WALL = .003
OVERHANG = .02
SRC = common.SOURCE_TCP
TIGHT_ROLES = ('target_guide', 'target_support')


def sample(rng, clearance):
    return dict(clearance_m=clearance, bay=int(rng.integers(-1, 2)), length=r4(rng.uniform(.38, .50)),
                overhang=OVERHANG, deck_z=r4(rng.uniform(.10, .18)), tier_h=r4(rng.uniform(.26, .32)),
                front_jitter=r4(rng.uniform(0., .02)), rear_depth=r4(rng.uniform(.03, .045)),
                divider_h=r4(rng.uniform(.14, .20)), stock_present=[bool(v) for v in rng.random(6) < .8],
                stock_length=[r4(v) for v in rng.uniform(.42, .56, 6)])


def build(params):
    c, L, oh = params['clearance_m'], params['length'], params['overhang']
    lever = L - oh
    y_t = params['bay'] * BAY_PITCH
    x_f = X_PRE_MIN + lever + TIP_CLEARANCE + UPRIGHT / 2 + params['front_jitter']
    face = x_f - UPRIGHT / 2
    depth = max(.55, L + .12)
    x_b = x_f + depth
    deck_z, th = params['deck_z'], params['tier_h']
    decks = [deck_z + k * th for k in (-2, -1, 0, 1)]
    top = decks[-1] + .20
    deck_top = deck_z + .009
    upper_bottom = decks[3] - .009
    upper_beam_bottom = decks[3] - .055
    if upper_beam_bottom - (deck_top + .012 + .034 + H) < .04:
        raise InfeasibleDesign('tier too low for insertion height')

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
    div_x0, div_x1 = x_f + .08, x_b - .02
    for k, y in enumerate((-1.5 * BAY_PITCH, -.5 * BAY_PITCH, .5 * BAY_PITCH, 1.5 * BAY_PITCH)):
        st.append(box_at(f'divider_{k}', [(div_x0 + div_x1) / 2, y, deck_top + params['divider_h'] / 2],
                         [div_x1 - div_x0, .012, params['divider_h']], BLUE, 'bin_divider'))
    # Neighbouring stock: target tier (other bays) and the tier below (all bays).
    slot = 0
    for tier, z_deck in ((0, deck_z), (-1, decks[1])):
        for bay in (-1, 0, 1):
            if tier == 0 and bay == params['bay']:
                continue
            present, length = params['stock_present'][slot], params['stock_length'][slot]
            slot += 1
            if not present:
                continue
            for lane, dy in enumerate((-.034, .034)):
                st.append(compound(f'stock_{tier}_{bay}_{lane}', hollow_tube_parts(length, .04, .04, .003, NEIGHBOR),
                                   from_rt(np.eye(3), [x_f + .06 + length / 2, bay * BAY_PITCH + dy, z_deck + .009 + .02]), 'neighbor_stock'))
    saddle_top = deck_top + .012
    supports = []
    for k, f in enumerate((.22, .78)):
        xs = x_f + params['rear_depth'] + f * L
        supports.append(f'target_saddle_{k}')
        st.append(box_at(f'target_saddle_{k}', [xs, y_t, deck_top + .006], [.05, .10, .012], TEAL, 'target_support'))
        for s in (-1, 1):
            st.append(box_at(f'target_guide_{k}_{"lr"[s > 0]}', [xs, y_t + s * (W / 2 + c + .007), saddle_top + .03],
                             [.035, .014, .06], ORANGE, 'target_guide'))
    obs += common.tag(st, 'station')

    z_ins = saddle_top + .034 + H / 2
    z_rel = saddle_top + .008 + H / 2
    x_pre = face - TIP_CLEARANCE - lever
    x_goal = x_f + params['rear_depth'] + oh
    z_tr = max(.34, z_ins + .06)
    knots = [start, tool_horizontal([SRC[0], SRC[1], z_tr], common.SOURCE_YAW),
             tool_horizontal([x_pre, -.47, z_tr], common.SOURCE_YAW), tool_horizontal([x_pre, -.47, z_tr], 0.),
             tool_horizontal([x_pre, y_t, z_tr], 0.), tool_horizontal([x_pre, y_t, z_ins], 0.),
             tool_horizontal([x_goal, y_t, z_ins], 0.), tool_horizontal([x_goal, y_t, z_rel], 0.)]
    world = np.eye(4)
    bay_half = BAY_PITCH / 2 - .006
    task = dict(operation='store', description=DESCRIPTION,
                start_phase='payload grasped 80 mm above the source nest', goal_phase='pre-release pose above both target saddles',
                goal=common.goal(.012, 5.),
                keep_in=[dict(name='rack_target_bay',
                              restricted=common.station_box(world, [face, -HALF_WIDTH - UPRIGHT / 2, FLOOR_Z], [x_b + UPRIGHT / 2, HALF_WIDTH + UPRIGHT / 2, top]),
                              allowed=common.station_box(world, [face, y_t - bay_half, deck_top], [x_b + UPRIGHT / 2, y_t + bay_half, upper_bottom]),
                              tolerance_m=.001)],
                portals=[common.portal('bay_entry', from_rt(np.eye(3), [face, y_t, (deck_top + upper_beam_bottom) / 2]),
                                       2 * bay_half, upper_beam_bottom - deck_top, 'front')],
                placement=dict(supports=supports, gap_range_m=[.002, .020], edge_margin_m=.025, lateral_tolerance_m=.025))
    record = common.scene_record(NAME, params, obs, common.payload('hollow_square_tube', L, W, H, oh, WALL), task, DESCRIPTION)
    record['design'].update(rack_front_x=r4(x_f), target_bay_y=r4(y_t), tightest_feature='target guide ears')
    seeds = [common.SEED_SOURCE_HORIZONTAL, common.SEED_HOME]
    return record, knots, seeds
