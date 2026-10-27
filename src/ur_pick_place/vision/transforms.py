"""Homogeneous transforms and UR rotation-vector poses (numpy only, no OpenCV)."""
import math

import numpy as np

from ..config import vector


def _skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def rotvec_to_matrix(rotvec):
    r = np.asarray(rotvec, dtype=float).reshape(3)
    angle = float(np.linalg.norm(r))
    if angle < 1e-12:
        return np.eye(3) + _skew(r)
    k = _skew(r / angle)
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def matrix_to_quaternion(rotation):
    """Return (w, x, y, z) with w >= 0; numerically stable for all angles."""
    m = np.asarray(rotation, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.asarray(q)
    q /= np.linalg.norm(q)
    return -q if q[0] < 0 else q


def matrix_to_rotvec(rotation):
    w, x, y, z = matrix_to_quaternion(rotation)
    sine = math.sqrt(x * x + y * y + z * z)
    if sine < 1e-12:
        return np.zeros(3)
    return np.array([x, y, z]) * (2.0 * math.atan2(sine, w) / sine)


def pose_to_matrix(pose):
    """UR pose [x, y, z, rx, ry, rz] (m, rad) -> 4x4 matrix."""
    pose = vector(list(pose), 6, 'Pose')
    result = np.eye(4)
    result[:3, :3] = rotvec_to_matrix(pose[3:])
    result[:3, 3] = pose[:3]
    return result


def matrix_to_pose(matrix):
    """4x4 matrix -> UR pose list [x, y, z, rx, ry, rz] (m, rad)."""
    m = np.asarray(matrix, dtype=float)
    return [float(v) for v in m[:3, 3]] + [float(v) for v in matrix_to_rotvec(m[:3, :3])]


def invert(matrix):
    m = np.asarray(matrix, dtype=float)
    result = np.eye(4)
    result[:3, :3] = m[:3, :3].T
    result[:3, 3] = -m[:3, :3].T @ m[:3, 3]
    return result


def translation(xyz):
    result = np.eye(4)
    result[:3, 3] = np.asarray(xyz, dtype=float).reshape(3)
    return result


def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotation_angle(a, b=None):
    """Angle in radians of rotation ``a`` (3x3 or 4x4), or between ``a`` and ``b``."""
    ra = np.asarray(a, dtype=float)[:3, :3]
    relative = ra if b is None else ra.T @ np.asarray(b, dtype=float)[:3, :3]
    cosine = (np.trace(relative) - 1.0) / 2.0
    # arccos loses precision near zero; use the antisymmetric part as well.
    sine = np.linalg.norm([relative[2, 1] - relative[1, 2], relative[0, 2] - relative[2, 0],
                           relative[1, 0] - relative[0, 1]]) / 2.0
    return float(math.atan2(sine, max(-1.0, min(1.0, cosine))))


def project_to_rotation(matrix):
    """Nearest proper rotation (Frobenius norm) via SVD."""
    u, _, vt = np.linalg.svd(np.asarray(matrix, dtype=float)[:3, :3])
    d = np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))])
    return u @ d @ vt


def mean_rotation(rotations):
    return project_to_rotation(np.mean([np.asarray(r, float)[:3, :3] for r in rotations], axis=0))


def check_transform(matrix, name, *, tolerance=1e-6):
    """Validate a finite rigid 4x4 transform and return it as a float array."""
    try:
        m = np.asarray(matrix, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a 4x4 numeric matrix') from exc
    if m.shape != (4, 4) or not np.isfinite(m).all():
        raise ValueError(f'{name} must be a finite 4x4 matrix')
    if not np.allclose(m[3], [0, 0, 0, 1], atol=tolerance):
        raise ValueError(f'{name} bottom row must be [0, 0, 0, 1]')
    r = m[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=tolerance) or abs(np.linalg.det(r) - 1) > tolerance:
        raise ValueError(f'{name} rotation must be orthonormal with determinant +1')
    return m


def yaw_tilt_split(rotation):
    """Split a rotation into a twist about base Z and the remaining tilt (swing).

    Returns ``(yaw_rad, tilt_rad)`` with ``rotation == Rz(yaw) @ swing`` and
    ``tilt = angle(swing)``.
    """
    w, _, _, z = matrix_to_quaternion(rotation)
    norm = math.hypot(w, z)
    yaw = 0.0 if norm < 1e-12 else 2.0 * math.atan2(z, w)
    swing = rotation_z(-yaw) @ np.asarray(rotation, float)[:3, :3]
    return yaw, rotation_angle(swing)


def describe(matrix):
    """JSON-friendly summary of a 4x4 transform."""
    m = np.asarray(matrix, dtype=float)
    return {'matrix': m.tolist(), 'pose_ur': matrix_to_pose(m),
            'xyz_mm': [float(v) * 1000 for v in m[:3, 3]]}
