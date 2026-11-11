"""F3 lathe: load a bar through a machine door into the bore of a side-facing chuck.

Application: CNC lathe tending.  The spindle axis runs parallel to the front wall, so the bar is
carried broadside through the door opening (with the wrist reaching into the enclosure), aligned
with the spindle axis, then inserted axially into the chuck jaws.  The jaws form a square bore
whose clearance per side is the difficulty level; both lateral and vertical errors matter.
"""
import numpy as np
from ..geometry import BLUE, DARK, NEIGHBOR, ORANGE, STEEL, TEAL, box_at, frame, from_rt, rz, tool_horizontal
from . import common
from .common import FLOOR_Z, InfeasibleDesign, r4

NAME = 'lathe'
DESCRIPTION = 'Carry a bar through a machine door and insert it axially into a side-facing chuck bore'
W = H = .036
OVERHANG = .02
WALL = .02
JAW_LEN = .05
TIP_CLEARANCE = .04   # bar tip to chuck face before the axial insertion
DOOR_EDGE = .025      # bar tip to the chuck-side door edge while entering
ENCLOSURE = [.80, .82, .80, 1.]
TIGHT_ROLES = ('target_fixture',)


def sample(rng, clearance):
    return dict(clearance_m=clearance, length=r4(rng.uniform(.20, .30)), overhang=OVERHANG,
                rho_door=r4(rng.uniform(.42, .52)), azimuth=r4(rng.uniform(.70, 1.30)), skew=r4(rng.uniform(-.15, .15)),
                z_axis=r4(rng.uniform(.16, .28)), axis_depth=r4(rng.uniform(.10, .15)), chuck_side=int(rng.choice([-1, 1])),
                insert_depth=r4(rng.uniform(.025, .03)), door_extra=r4(rng.uniform(0., .06)), door_height=r4(rng.uniform(.42, .50)))


def build(params):
    c, L, oh = params['clearance_m'], params['length'], params['overhang']
    lever = L - oh
    sigma = params['chuck_side']
    psi = params['azimuth'] + params['skew']
    origin = np.array([params['rho_door'] * np.cos(params['azimuth']), params['rho_door'] * np.sin(params['azimuth']), 0.])
    station = frame(origin, psi)  # x into the machine, y along the spindle axis, z up; door plane at x = 0
    z_ax, d_ax = params['z_axis'], params['axis_depth']
    x_ax = WALL + d_ax
    wd, hd = lever + .40 + params['door_extra'], params['door_height']
    # Door spans y in [-wd/2, wd/2]; while entering, the bar tip stays DOOR_EDGE inside its chuck-side edge.
    y_e = -sigma * (lever + DOOR_EDGE - wd / 2)                # TCP lateral position while entering
    y_face = y_e + sigma * (lever + TIP_CLEARANCE)              # chuck face plane (just beyond the door edge)
    y_in = wd / 2 + .23                                         # half interior width (room for chuck body + headstock)
    depth = d_ax + .24
    bed_top = z_ax - .16
    zd0 = max(bed_top, z_ax + .08 - hd / 2)
    zd1 = zd0 + hd
    hd = zd1 - zd0
    z_door = (zd0 + zd1) / 2
    roof = zd1 + .06
    bore = W / 2 + c
    jaw_out = bore + .035

    st = []
    door_lo, door_hi = -wd / 2, wd / 2
    y_out = y_in + WALL
    st += [box_at('front_wall_left', [WALL / 2, (-y_out + door_lo) / 2, (FLOOR_Z + roof + WALL) / 2], [WALL, door_lo + y_out, roof + WALL - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('front_wall_right', [WALL / 2, (door_hi + y_out) / 2, (FLOOR_Z + roof + WALL) / 2], [WALL, y_out - door_hi, roof + WALL - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('front_wall_top', [WALL / 2, 0, (zd1 + roof + WALL) / 2], [WALL, wd, roof + WALL - zd1], ENCLOSURE, 'machine_enclosure', station),
           box_at('front_wall_bottom', [WALL / 2, 0, (FLOOR_Z + zd0) / 2], [WALL, wd, zd0 - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('back_wall', [WALL + depth + WALL / 2, 0, (FLOOR_Z + roof + WALL) / 2], [WALL, 2 * y_out, roof + WALL - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('side_wall_neg', [WALL + depth / 2, -y_in - WALL / 2, (FLOOR_Z + roof + WALL) / 2], [depth, WALL, roof + WALL - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('side_wall_pos', [WALL + depth / 2, y_in + WALL / 2, (FLOOR_Z + roof + WALL) / 2], [depth, WALL, roof + WALL - FLOOR_Z], ENCLOSURE, 'machine_enclosure', station),
           box_at('roof', [WALL + depth / 2, 0, roof + WALL / 2], [depth, 2 * y_in, WALL], ENCLOSURE, 'machine_enclosure', station),
           box_at('machine_bed', [WALL + depth / 2, 0, (FLOOR_Z + bed_top) / 2], [depth, 2 * y_in, bed_top - FLOOR_Z], DARK, 'machine_structure', station)]
    # Chuck jaws (square bore along y), chuck body, headstock.
    y0, y1 = y_face, y_face + sigma * JAW_LEN
    yc, ly = (y0 + y1) / 2, JAW_LEN
    st += [box_at('chuck_jaw_xp', [x_ax + (bore + jaw_out) / 2, yc, z_ax], [jaw_out - bore, ly, 2 * jaw_out], TEAL, 'target_fixture', station),
           box_at('chuck_jaw_xn', [x_ax - (bore + jaw_out) / 2, yc, z_ax], [jaw_out - bore, ly, 2 * jaw_out], TEAL, 'target_fixture', station),
           box_at('chuck_jaw_zp', [x_ax, yc, z_ax + (bore + jaw_out) / 2], [2 * bore, ly, jaw_out - bore], TEAL, 'target_fixture', station),
           box_at('chuck_jaw_zn', [x_ax, yc, z_ax - (bore + jaw_out) / 2], [2 * bore, ly, jaw_out - bore], TEAL, 'target_fixture', station)]
    body0, body1 = y1, y1 + sigma * .06
    st.append(box_at('chuck_body', [x_ax, (body0 + body1) / 2, z_ax], [.18, .06, .18], STEEL, 'target_fixture', station))
    head_end = sigma * y_in
    st.append(box_at('headstock', [x_ax, (body1 + head_end) / 2, (bed_top + z_ax + .12) / 2], [.22, abs(head_end - body1), z_ax + .12 - bed_top],
                     BLUE, 'machine_structure', station))
    tail0, tail1 = -sigma * y_in, -sigma * (y_in - .12)
    st.append(box_at('tailstock', [x_ax, (tail0 + tail1) / 2, (bed_top + z_ax + .06) / 2], [.14, .12, z_ax + .06 - bed_top], BLUE, 'machine_structure', station))
    st.append(box_at('tool_turret', [x_ax + .155, -sigma * .05, z_ax + .02], [.07, .16, .16], NEIGHBOR, 'machine_structure', station))
    obs = common.tag(common.cell(), 'cell')
    src, start = common.horizontal_source(lever)
    obs += common.tag(src, 'source') + common.tag(st, 'station')

    yaw = psi + sigma * np.pi / 2                     # tool axis along sigma * y_station
    to_world = lambda p: (station @ np.r_[p, 1.])[:3]
    z_tr = max(.34, z_ax + .10)
    pre_out = to_world([-(.10 + W / 2), y_e, z_tr])
    pre = to_world([-(.10 + W / 2), y_e, z_ax])
    inside = to_world([x_ax, y_e, z_ax])
    final = to_world([x_ax, y_e + sigma * (TIP_CLEARANCE + params['insert_depth']), z_ax])
    src_tcp = np.array([common.SOURCE_TCP[0], common.SOURCE_TCP[1], z_tr])
    raised = tool_horizontal(src_tcp, common.SOURCE_YAW)
    tail = [tool_horizontal(pre_out, yaw), tool_horizontal(pre, yaw), tool_horizontal(inside, yaw), tool_horizontal(final, yaw)]
    knots = []
    for radius in (.50, .58):
        via, outward = common.via_point(src_tcp, pre_out, z_tr, radius)
        knots += [[start, raised, tool_horizontal(via, outward)] + tail,      # payload points outward mid-transfer
                  [start, raised, tool_horizontal(via, yaw)] + tail]          # payload already parallel to the door
    bore_frame = station @ from_rt(rz(sigma * np.pi / 2), [x_ax, y_face, z_ax])
    task = dict(operation='load', description=DESCRIPTION,
                start_phase='payload grasped 80 mm above the source nest', goal_phase='bar inserted into the chuck bore',
                goal=common.goal(.010, 5.),
                keep_in=[dict(name='chuck_bore',
                              restricted=common.station_box(station, [x_ax - jaw_out, min(y0, body1), z_ax - jaw_out], [x_ax + jaw_out, max(y0, body1), z_ax + jaw_out]),
                              allowed=common.station_box(station, [x_ax - bore, min(y0, body1), z_ax - bore], [x_ax + bore, max(y0, body1), z_ax + bore]),
                              tolerance_m=.001)],
                portals=[common.portal('machine_door', station @ from_rt(np.eye(3), [WALL / 2, 0, z_door]), wd, hd, 'center'),
                         common.portal('chuck_bore', bore_frame, 2 * bore, 2 * bore, 'front')],
                placement=None)
    record = common.scene_record(NAME, params, obs, common.payload('solid_square_bar', L, W, H, oh, None, 'steel (nominal)', 7850.),
                                 task, DESCRIPTION)
    record['design'].update(door_width_m=r4(wd), door_height_m=r4(hd), bore_m=r4(2 * bore), chuck_side=sigma,
                            tightest_feature='chuck bore (2-D clearance)')
    return record, knots, [common.SEED_SOURCE_HORIZONTAL, common.SEED_HOME]
