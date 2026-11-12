"""F4 tote (v0.4): lower a rod into a roomy tote cell past a half-open lid flap and a cell ledge.

The target cell is roomy (30 mm around the rod), so the goal itself carries little risk.  A half-open
lid flap overhangs the cell just above the rim on one side and a moulded ledge protrudes from the
opposite cell wall lower down.  The vertical goal line passes both with clearance c (the level).  A
straight descent passes both with the rod TIP at clearance c; a risk-aware descent offsets / tilts the
rod so that only short-lever points pass them at small clearance.
"""
import numpy as np
from ..geometry import BLUE, NEIGHBOR, ORANGE, STEEL, box_at, frame, from_rt, tool_down
from . import common
from .common import FLOOR_Z, InfeasibleDesign, r4

NAME = 'tote'
DESCRIPTION = 'Lower a rod into a roomy tote cell past a half-open lid flap (rim) and a cell ledge (lower, opposite side)'
W = H = .036
OVERHANG = .02
DIVIDER = .006
CELL_SLACK = .030
TIP_CLEARANCE = .03
TIGHT_ROLES = ('intrusion', 'tote')


def sample(rng, clearance):
    nx, ny = int(rng.integers(2, 4)), int(rng.integers(2, 4))
    return dict(clearance_m=clearance, length=r4(rng.uniform(.31, .36)), overhang=OVERHANG,
                rho=r4(rng.uniform(.52, .62)), azimuth=r4(rng.uniform(.15, .75)), yaw=r4(rng.uniform(-.6, .6)),
                table_z=r4(rng.uniform(-.22, -.12)), nx=nx, ny=ny, cell_height=r4(rng.uniform(.15, .18)),
                target=[int(rng.integers(0, nx)), int(rng.integers(0, ny))], occupied=[bool(v) for v in rng.random(nx * ny) < .5],
                flap_side=int(rng.choice([-1, 1])), flap_height=r4(rng.uniform(.015, .03)), ledge_depth=r4(rng.uniform(.04, .07)))


def build(params):
    c, L, oh = params['clearance_m'], params['length'], params['overhang']
    lever = L - oh
    nx, ny, hc = params['nx'], params['ny'], params['cell_height']
    a = W + 2 * CELL_SLACK
    pitch = a + DIVIDER
    gx, gy = nx * pitch + DIVIDER, ny * pitch + DIVIDER
    station = frame([params['rho'] * np.cos(params['azimuth']), params['rho'] * np.sin(params['azimuth']), 0.], params['yaw'])
    z_tab = params['table_z']
    floor_top = z_tab + .01
    top = floor_top + hc
    cell_x = lambda i: -gx / 2 + DIVIDER + a / 2 + i * pitch
    cell_y = lambda j: -gy / 2 + DIVIDER + a / 2 + j * pitch
    ti, tj = params['target']
    cx, cy = cell_x(ti), cell_y(tj)
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
    for k in range(nx * ny):
        i, j = divmod(k, ny)
        if (i, j) != (ti, tj) and params['occupied'][k]:
            h = hc - .03
            st.append(box_at(f'kitted_part_{i}{j}', [cell_x(i), cell_y(j), floor_top + h / 2], [.03, .03, h], NEIGHBOR, 'neighbor_stock', station))
    s = params['flap_side']
    # Half-open lid flap above the rim on side s (station y), face at c from the rod's goal line.
    flap_z = top + params['flap_height']
    y_face, y_rim = cy + s * (W / 2 + c), cy + s * (a / 2 + DIVIDER)
    st.append(box_at('lid_flap', [cx, (y_face + y_rim) / 2 + s * .03, flap_z], [a + .04, abs(y_rim - y_face) + .06, .006], ORANGE, 'intrusion', station))
    # Moulded ledge on the opposite wall, lower in the cell.
    ledge_z = top - params['ledge_depth']
    y_face2, y_wall = cy - s * (W / 2 + c), cy - s * (a / 2)
    st.append(box_at('cell_ledge', [cx, (y_face2 + y_wall) / 2, ledge_z], [a, abs(y_wall - y_face2), .012], ORANGE, 'intrusion', station))
    tcp_final = floor_top + .008 + lever
    if tcp_final - .014 < top + params['flap_height'] + .02:
        raise InfeasibleDesign('rod too short: the gripper would reach the lid flap')
    if tcp_final - ledge_z > .24:
        raise InfeasibleDesign('ledge too far below the gripper at the goal')
    obs = common.tag(common.cell(), 'cell')
    src, start = common.vertical_source(lever, holder_height=.02)
    obs += common.tag(src, 'source') + common.tag(st, 'station')

    centre = (station @ np.r_[cx, cy, 0., 1.])[:3]
    jaw = params['yaw']
    tcp_pre = flap_z + .003 + TIP_CLEARANCE + lever
    z_tr = max(start[2, 3], tcp_pre) + .06
    knots = [start, tool_down([start[0, 3], start[1, 3], z_tr], 0.), tool_down([centre[0], centre[1], z_tr], jaw),
             tool_down([centre[0], centre[1], tcp_pre], jaw), tool_down([centre[0], centre[1], tcp_final], jaw)]
    down = np.array([[0., 1., 0.], [0., 0., -1.], [-1., 0., 0.]])
    task = dict(operation='kit', description=DESCRIPTION,
                start_phase='rod grasped 80 mm above the source holder', goal_phase='rod in the target cell, 8 mm above its floor',
                goal=common.goal(.012, 5.),
                keep_in=[dict(name='tote_target_cell',
                              restricted=common.station_box(station, [-gx / 2, -gy / 2, floor_top], [gx / 2, gy / 2, top]),
                              allowed=common.station_box(station, [cx - a / 2, cy - a / 2, floor_top], [cx + a / 2, cy + a / 2, top]),
                              tolerance_m=.001)],
                portals=[common.portal('cell_opening', station @ from_rt(down, [cx, cy, top]), a, a, 'front')],
                placement=None)
    record = common.scene_record(NAME, params, obs, common.payload('solid_square_rod', L, W, H, oh, None, 'steel (nominal)', 7850.),
                                 task, DESCRIPTION)
    record['design'].update(cell_m=r4(a), grid=[nx, ny], goal_slack_m=CELL_SLACK, tightest_feature='lid flap (rim) and cell ledge (opposite, lower)',
                            fitting_depths_from_tcp_goal_m=[r4(tcp_final - flap_z), r4(tcp_final - ledge_z)])
    y_axis = station[:3, 1]
    axis_frame = from_rt(np.column_stack([[0, 0, -1.], y_axis, np.cross([0, 0, -1.], y_axis)]), [0, 0, 0])
    robust = dict(segment=(3, 4), frame=axis_frame, max_offset=min(.04, CELL_SLACK + .005), lever=lever, planes=('y',),
                  event_depths=[-flap_z, -ledge_z], control_depths=[-flap_z, -ledge_z])
    return record, knots, [common.SEED_HOME, common.SEED_DOWN, common.SEED_SOURCE_HORIZONTAL], robust
