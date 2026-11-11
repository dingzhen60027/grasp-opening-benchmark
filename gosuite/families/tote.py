"""F4 tote: insert a vertically held rod into one cell of a divided kitting tote.

Application: kitting / part presentation.  The rod is grasped at its top end and lowered into the
target cell among neighbouring parts, some of which protrude above the dividers.  Cell clearance
per side is the difficulty level; tilt errors about both horizontal axes move the rod tip.
"""
import numpy as np
from ..geometry import BLUE, NEIGHBOR, ORANGE, STEEL, TEAL, box_at, frame, from_rt, rz, tool_down
from . import common
from .common import FLOOR_Z, InfeasibleDesign, r4

NAME = 'tote'
DESCRIPTION = 'Lower a vertically held rod into the target cell of a divided kitting tote among neighbouring parts'
W = H = .036
OVERHANG = .02
DIVIDER = .006
TIP_CLEARANCE = .03
GRIPPER_LOW = .014      # fingertips extend this far below the TCP
GRIPPER_MARGIN = .02
TIGHT_ROLES = ('tote', 'neighbor_stock')


def sample(rng, clearance):
    nx, ny = int(rng.integers(2, 4)), int(rng.integers(3, 5))
    return dict(clearance_m=clearance, length=r4(rng.uniform(.24, .32)), overhang=OVERHANG,
                rho=r4(rng.uniform(.46, .60)), azimuth=r4(rng.uniform(0., .65)), yaw=r4(rng.uniform(-.6, .6)),
                table_z=r4(rng.uniform(-.12, .02)), nx=nx, ny=ny, cell_height=r4(rng.uniform(.08, .13)),
                target=[int(rng.integers(0, nx)), int(rng.integers(0, ny))],
                occupied=[bool(v) for v in rng.random(nx * ny) < .55],
                protrusion=[r4(v) for v in rng.uniform(-.02, .06, nx * ny)], jaw_turn=int(rng.integers(0, 2)))


def build(params):
    c, L, oh = params['clearance_m'], params['length'], params['overhang']
    lever = L - oh
    nx, ny, hc = params['nx'], params['ny'], params['cell_height']
    a = W + 2 * c
    pitch = a + DIVIDER
    gx, gy = nx * pitch + DIVIDER, ny * pitch + DIVIDER
    station = frame([params['rho'] * np.cos(params['azimuth']), params['rho'] * np.sin(params['azimuth']), 0.], params['yaw'])
    z_tab = params['table_z']
    floor_top = z_tab + .01
    top = floor_top + hc
    cell_x = lambda i: -gx / 2 + DIVIDER + a / 2 + i * pitch
    cell_y = lambda j: -gy / 2 + DIVIDER + a / 2 + j * pitch
    ti, tj = params['target']

    st = [box_at('tote_table', [0, 0, z_tab - .015], [gx + .20, gy + .20, .03], [.35, .39, .41, 1], 'station_table', station)]
    for sx in (-1, 1):
        for sy in (-1, 1):
            st.append(box_at(f'tote_table_leg_{sx}{sy}', [sx * (gx / 2 + .07), sy * (gy / 2 + .07), (FLOOR_Z + z_tab - .03) / 2],
                             [.035, .035, z_tab - .03 - FLOOR_Z], STEEL, 'station_table', station))
    st.append(box_at('tote_floor', [0, 0, z_tab + .005], [gx, gy, .01], BLUE, 'tote', station))
    for i in range(nx + 1):
        st.append(box_at(f'divider_x{i}', [-gx / 2 + DIVIDER / 2 + i * pitch, 0, floor_top + hc / 2], [DIVIDER, gy, hc], BLUE, 'tote', station))
    for j in range(ny + 1):
        st.append(box_at(f'divider_y{j}', [0, -gy / 2 + DIVIDER / 2 + j * pitch, floor_top + hc / 2], [gx, DIVIDER, hc], BLUE, 'tote', station))
    highest = top
    for k in range(nx * ny):
        i, j = divmod(k, ny)
        if (i, j) == (ti, tj) or not params['occupied'][k]:
            continue
        h = hc + params['protrusion'][k]
        st.append(box_at(f'kitted_part_{i}{j}', [cell_x(i), cell_y(j), floor_top + h / 2], [.03, .03, h], NEIGHBOR, 'neighbor_stock', station))
        if max(abs(i - ti), abs(j - tj)) <= 1:
            highest = max(highest, floor_top + h)
    if floor_top + .008 + lever - GRIPPER_LOW < highest + GRIPPER_MARGIN:
        raise InfeasibleDesign('rod too short: gripper would reach the dividers or neighbouring parts')
    obs = common.tag(common.cell(), 'cell')
    src, start = common.vertical_source(lever)
    obs += common.tag(src, 'source') + common.tag(st, 'station')

    centre = (station @ np.r_[cell_x(ti), cell_y(tj), 0., 1.])[:3]
    jaw = params['yaw'] + params['jaw_turn'] * np.pi / 2
    tcp_pre = max(top, highest) + TIP_CLEARANCE + lever
    tcp_final = floor_top + .008 + lever
    z_tr = max(start[2, 3], tcp_pre) + .06
    knots = [start, tool_down([start[0, 3], start[1, 3], z_tr], 0.), tool_down([centre[0], centre[1], z_tr], jaw),
             tool_down([centre[0], centre[1], tcp_pre], jaw), tool_down([centre[0], centre[1], tcp_final], jaw)]
    down = np.array([[0., 1., 0.], [0., 0., -1.], [-1., 0., 0.]])  # portal +x points down
    task = dict(operation='kit', description=DESCRIPTION,
                start_phase='rod grasped 80 mm above the source holder', goal_phase='rod lowered into the target cell, 8 mm above its floor',
                goal=common.goal(.010, 5.),
                keep_in=[dict(name='tote_target_cell',
                              restricted=common.station_box(station, [-gx / 2, -gy / 2, floor_top], [gx / 2, gy / 2, top]),
                              allowed=common.station_box(station, [cell_x(ti) - a / 2, cell_y(tj) - a / 2, floor_top],
                                                         [cell_x(ti) + a / 2, cell_y(tj) + a / 2, top]),
                              tolerance_m=.001)],
                portals=[common.portal('cell_opening', station @ from_rt(down, [cell_x(ti), cell_y(tj), top]), a, a, 'front')],
                placement=None)
    record = common.scene_record(NAME, params, obs, common.payload('solid_square_rod', L, W, H, oh, None, 'steel (nominal)', 7850.),
                                 task, DESCRIPTION)
    record['design'].update(cell_m=r4(a), grid=[nx, ny], tightest_feature='tote cell walls')
    return record, knots, [common.SEED_HOME, common.SEED_DOWN, common.SEED_SOURCE_HORIZONTAL]
