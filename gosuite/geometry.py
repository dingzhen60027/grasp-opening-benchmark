"""Small rigid-body helpers shared by scene families, the simulator world and the task checker.

Conventions
-----------
* Units are metres and radians; poses are 4x4 homogeneous matrices (row-major lists in JSON).
* Payload frame: local +x is the long axis and points from the grasped end to the free
  (leading) end; local +y is the jaw-closing axis; local +z completes the frame.
* Tool (TCP) frame of the UR5e + Robotiq model: +z is the approach axis, +x the jaw-closing
  axis.  The payload long axis is aligned with TCP +z, so ``T_EO`` is the same for every family.
"""
import numpy as np
from scipy.spatial.transform import Rotation

# Maps TCP axes onto world axes for a horizontal tool at yaw 0: TCP z -> world x, TCP x -> world y.
R_TOOL_HORIZONTAL = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
# Payload axes expressed in the TCP frame (x_O = z_E, y_O = x_E, z_O = y_E).
R_EO = R_TOOL_HORIZONTAL.T.copy()

STEEL = [.55, .61, .64, 1.]
DARK = [.27, .29, .31, 1.]
BLUE = [.075, .21, .36, 1.]
ORANGE = [.94, .38, .07, 1.]
TEAL = [.05, .55, .55, 1.]
PAYLOAD = [.98, .74, .10, 1.]
NEIGHBOR = [.48, .52, .56, 1.]
FLOOR = [.84, .85, .83, 1.]


def transform(position=(0., 0., 0.), quaternion=(0., 0., 0., 1.)):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    t[:3, 3] = position
    return t


def from_rt(rotation, position):
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = position
    return t


def rz(yaw):
    return Rotation.from_euler('z', yaw).as_matrix()


def frame(origin, yaw=0.):
    """Planar station frame: origin and rotation about world z."""
    return from_rt(rz(yaw), origin)


def apply(t, points):
    points = np.asarray(points, dtype=float)
    return points @ t[:3, :3].T + t[:3, 3]


def inverse(t):
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return out


def grasp_delta(xi):
    """[Exp_SO3(phi), delta_t] about the TCP origin; NOT the SE(3) exponential of a twist."""
    xi = np.asarray(xi, dtype=float)
    return from_rt(Rotation.from_rotvec(xi[3:]).as_matrix(), xi[:3])


def grasp_transform(length, overhang):
    """Nominal T_EO: payload long axis along TCP +z, grasped `overhang` from its rear end."""
    return from_rt(R_EO, [0., 0., length / 2 - overhang])


def tool_horizontal(position, yaw):
    """TCP pose with the approach axis horizontal, pointing along world yaw; jaws close horizontally."""
    return from_rt(rz(yaw) @ R_TOOL_HORIZONTAL, position)


def tool_down(position, yaw):
    """TCP pose with the approach axis pointing down (-z); jaw axis along world yaw."""
    c, s = np.cos(yaw), np.sin(yaw)
    r = np.array([[c, s, 0.], [s, -c, 0.], [0., 0., -1.]])
    return from_rt(r, position)


def box(name, size, pose, color=STEEL, role='structure'):
    return dict(name=name, role=role, size=[float(v) for v in size], color=list(color), pose=np.asarray(pose).tolist())


def box_at(name, center, size, color=STEEL, role='structure', station=None, yaw=0.):
    """Axis-aligned box in a station frame (identity = world)."""
    local = from_rt(rz(yaw), center)
    world = local if station is None else station @ local
    return box(name, size, world, color, role)


def compound(name, parts, pose, role='structure'):
    return dict(name=name, role=role, parts=parts, pose=np.asarray(pose).tolist())


def part(size, center=(0., 0., 0.), color=STEEL):
    return dict(size=[float(v) for v in size], center=[float(v) for v in center], color=list(color))


def hollow_tube_parts(length, width, height, wall, color=PAYLOAD):
    """Rectangular hollow section as four walls; the bore stays open (no convex hull)."""
    return [part([length, width, wall], [0, 0, z], color) for z in (-(height - wall) / 2, (height - wall) / 2)] + [
        part([length, wall, height - 2 * wall], [0, y, 0], color) for y in (-(width - wall) / 2, (width - wall) / 2)]


def solid_bar_parts(length, width, height, color=PAYLOAD):
    return [part([length, width, height], [0, 0, 0], color)]


def interpolate_pose(a, b, u):
    """Linear position and spherical-linear orientation interpolation between two 4x4 poses."""
    ra = Rotation.from_matrix(a[:3, :3])
    rel = (ra.inv() * Rotation.from_matrix(b[:3, :3])).as_rotvec()
    rot = (ra * Rotation.from_rotvec(rel * u)).as_matrix()
    return from_rt(rot, (1 - u) * a[:3, 3] + u * b[:3, 3])


def pose_distance(a, b):
    """(translation distance, rotation angle) between two poses."""
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), float(Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude())


def densify(path, step):
    """Insert states so that no joint moves more than `step` rad between consecutive states."""
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 6 or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError('Expected a finite Nx6 joint path with at least two states')
    if step <= 0:
        raise ValueError('step must be positive')
    out = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, int(np.ceil(np.max(np.abs(b - a)) / step)))
        out.extend(a + (b - a) * i / n for i in range(1, n + 1))
    return np.array(out)


def obstacle_aabb(item):
    """Conservative world AABB of a scene obstacle record (box or compound of boxes)."""
    pose = np.asarray(item['pose'])
    parts = item.get('parts') or [dict(size=item['size'], center=[0, 0, 0])]
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for prt in parts:
        half = np.asarray(prt['size']) / 2
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half + prt.get('center', [0, 0, 0])
        world = apply(pose, corners)
        lo = np.minimum(lo, world.min(0))
        hi = np.maximum(hi, world.max(0))
    return lo, hi
