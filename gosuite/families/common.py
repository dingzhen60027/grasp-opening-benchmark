"""Shared work cell, source fixtures and record builders for all scene families."""
import numpy as np
from ..geometry import (DARK, FLOOR, NEIGHBOR, ORANGE, PAYLOAD, STEEL, box_at, compound, from_rt, grasp_transform,
                        hollow_tube_parts, rz, solid_bar_parts, tool_down, tool_horizontal)

FLOOR_Z = -.75
FINGERTIPS = ['robotiq_85_left_finger_tip_link', 'robotiq_85_right_finger_tip_link']
# Start configuration of the v0.2 rack tasks (payload grasped above the source nest); used as an IK seed.
SEED_SOURCE_HORIZONTAL = [-1.8677, -0.8403, 1.231, -0.3907, -1.0969, 0.0]
SEED_HOME = [0., -1.5, 1.5, -1.5, -1.57, 0.]
SEED_DOWN = [-1.3, -1.2, 1.6, -1.97, -1.57, 0.]
SOURCE_TCP = np.array([.12, -.47, .23])
SOURCE_YAW = .8
SOURCE_TABLE_TOP = .0675
DEFAULT_UNCERTAINTY = dict(
    convention='T_WO = T_WE @ [Exp_SO3(phi), delta_t] @ T_EO; TCP frame; one fixed draw per execution',
    distribution='independent zero-mean Gaussian per axis',
    translation_std_m=[.002] * 3, rotation_std_rad=[float(np.deg2rad(1.))] * 3, calibrated=False)


class InfeasibleDesign(ValueError):
    """Raised by a family builder when sampled parameters cannot form a valid layout."""


def r4(v):
    return round(float(v), 4)


def cell():
    return [box_at('floor', [.30, 0, FLOOR_Z - .025], [2.8, 3.0, .05], FLOOR, 'floor'),
            box_at('robot_pedestal', [0, 0, -.39], [.23, .25, .70], DARK, 'robot_mount'),
            box_at('robot_base_plate', [0, 0, -.025], [.24, .24, .04], STEEL, 'robot_mount')]


def source_table():
    obs = [box_at('source_table', [.27, -.39, .05], [.60, .52, .035], [.35, .39, .41, 1], 'source_fixture')]
    for x in (.02, .52):
        for y in (-.61, -.17):
            obs.append(box_at(f'source_leg_{x}_{y}', [x, y, (FLOOR_Z + .0325) / 2], [.035, .035, .0325 - FLOOR_Z], STEEL, 'source_fixture'))
    return obs


def horizontal_source(lever):
    """Source table with two cradles under a horizontally grasped payload (80 mm above the nest)."""
    obs = source_table()
    direction = np.array([np.cos(SOURCE_YAW), np.sin(SOURCE_YAW), 0])
    for k, f in enumerate((.25, .75)):
        c = SOURCE_TCP + f * lever * direction
        obs.append(box_at(f'source_cradle_{k}', [c[0], c[1], .114], [.075, .10, .035], ORANGE, 'source_fixture', yaw=SOURCE_YAW))
    for k, x in enumerate((.065, .37)):
        obs.append(compound(f'source_waiting_stock_{k}', hollow_tube_parts(.36, .04, .04, .003, NEIGHBOR),
                            from_rt(rz(SOURCE_YAW), [x, -.43, .09]), 'source_stock'))
    start = tool_horizontal(SOURCE_TCP, SOURCE_YAW)
    return obs, start


def vertical_source(lever, yaw=0., holder_height=.10):
    """Source table with a vertical holder; the rod hangs from the gripper, 80 mm above the holder."""
    obs = source_table()
    holder_top = SOURCE_TABLE_TOP + holder_height
    obs.append(box_at('source_holder', [.28, -.42, SOURCE_TABLE_TOP + holder_height / 2], [.08, .08, holder_height], ORANGE, 'source_fixture'))
    for k, (x, y) in enumerate([(.16, -.44), (.40, -.34)]):
        obs.append(box_at(f'source_waiting_rod_{k}', [x, y, SOURCE_TABLE_TOP + .13], [.036, .036, .26], NEIGHBOR, 'source_stock'))
    start = tool_down([.28, -.42, holder_top + .08 + lever], yaw)
    return obs, start


def via_point(a, b, z, radius=.5):
    """Transfer waypoint on a circle around the robot base, half way (in azimuth) between a and b.

    Returns (position, outward_yaw).  Keeps transfers away from the base, where a horizontally held
    long payload cannot be reached without self-collision.
    """
    ta, tb = np.arctan2(a[1], a[0]), np.arctan2(b[1], b[0])
    mid = ta + np.angle(np.exp(1j * (tb - ta))) / 2
    return np.array([radius * np.cos(mid), radius * np.sin(mid), z]), float(mid)


def payload(kind, length, width, height, overhang, hollow_wall=None, material='aluminium (nominal)', density=2700.):
    if hollow_wall:
        parts = hollow_tube_parts(length, width, height, hollow_wall, PAYLOAD)
        area = width * height - (width - 2 * hollow_wall) * (height - 2 * hollow_wall)
    else:
        parts = solid_bar_parts(length, width, height, PAYLOAD)
        area = width * height
    return dict(kind=kind, material=material, parts=parts, size=[r4(length), r4(width), r4(height)],
                wall_thickness_m=hollow_wall, estimated_mass_kg=r4(area * length * density),
                T_EO=grasp_transform(length, overhang).tolist(),
                grasp=dict(model='frozen Robotiq 2F-85 parallel grasp at the rear end', overhang_m=r4(overhang),
                           lever_arm_m=r4(length - overhang), contact_links=FINGERTIPS,
                           payload_frame='+x long axis towards the free end, +y jaw-closing axis'))


def region(pose, size):
    return dict(pose=np.asarray(pose).tolist(), size=[r4(v) for v in size])


def station_box(station, lo, hi):
    """Region record for an axis-aligned box [lo, hi] given in a station frame."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return region(station @ from_rt(np.eye(3), (lo + hi) / 2), hi - lo)


def portal(name, pose, width, height, point='front'):
    return dict(name=name, pose=np.asarray(pose).tolist(), size=[r4(width), r4(height)], point=point)


def goal(position_tolerance=.012, orientation_tolerance_deg=5.):
    # object_pose is filled in from the accepted witness by the generator.
    return dict(object_pose=None, position_tolerance_m=position_tolerance,
                orientation_tolerance_rad=float(np.deg2rad(orientation_tolerance_deg)),
                representation='payload pose region; q_goal_reference is one IK solution only')


def scene_record(family, params, obstacles, payload_record, task, description):
    return dict(schema_version='0.3.0', suite='gosuite', family=family, description=description,
                robot=dict(model='UR5e + Robotiq 2F-85', urdf='assets/ur5e_robotiq/ur5e.urdf', tcp_link='benchmark_tcp',
                           mount='base frame at the world origin, z=0 on a pedestal; floor at z=-0.75'),
                units=dict(length='m', angle='rad'), obstacles=obstacles, payload=payload_record,
                uncertainty=dict(DEFAULT_UNCERTAINTY), task=task,
                design=dict(params=params, clearance_per_side_m=params['clearance_m']),
                qualification='Engineered synthetic layout with dimensioned fixtures; not a measured digital twin')


def tag(obstacles, group):
    """Label obstacles as 'cell', 'source' or 'station' (used by the layout-conflict check)."""
    for o in obstacles:
        o.setdefault('group', group)
    return obstacles
