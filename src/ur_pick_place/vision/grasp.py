"""Taught marker-to-TCP relationships and the safety checks on computed targets.

Teaching stores ``T_marker_tcp = inv(T_base_marker_taught) @ T_base_tcp_taught``. At
run time the target TCP pose is ``T_base_marker_now @ Trans(offset) @ T_marker_tcp``.
Hand-eye error largely cancels when the marker is viewed the same way as during
teaching (same observe joints, or the marker-relative refine view), so teach with
the same observation procedure that the job uses.
"""
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from ..config import vector
from ..poses import atomic_json
from .errors import TargetValidationError, VisionError
from .robot_state import same_tcp
from .targets import same_target
from .transforms import (check_transform, describe, invert, matrix_to_pose, pose_to_matrix, rotation_angle,
                         rotation_z, translation, yaw_tilt_split)

GRASP_SCHEMA = 1
#: A taught TCP more than this far from its marker is almost certainly a teaching mistake.
MAX_MARKER_TCP_DISTANCE_M = 0.5
BASE_DOWN = np.array([0.0, 0.0, -1.0])


@dataclass(frozen=True)
class GraspModel:
    role: str
    marker_tcp: np.ndarray        # T_marker_tcp (without the configured offset)
    reference_marker: np.ndarray  # T_base_marker when taught (or implied by a configured grasp)
    reference_tcp: np.ndarray     # T_base_tcp when taught (or the configured reference)
    target: dict | None
    tcp_offset: list | None
    calibration_sha256: str | None
    source: str
    sha256: str = ''
    taught_refined: bool | None = None       # None for configured grasps
    taught_view: np.ndarray | None = None    # T_camera_marker of the taught observation


def tool_tilt_deg(matrix):
    """Angle between the TCP Z axis and base -Z (0 = tool pointing straight down)."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.asarray(matrix)[:3, 2] @ BASE_DOWN)))))


def teach(observation, tcp_pose, *, role, calibration, limits, tcp_offset=None):
    """Return the taught-grasp record; raise if the relationship is implausible."""
    tcp = pose_to_matrix(vector(tcp_pose, 6, 'Taught TCP pose'))
    if observation.calibration_sha256 != calibration.sha256:
        raise VisionError('Observation was made with a different hand-eye calibration; observe again')
    if observation.role != role:
        raise VisionError(f'Observation is for the {observation.role} target, not {role}')
    marker_tcp = invert(observation.base_target) @ tcp
    distance = float(np.linalg.norm(marker_tcp[:3, 3]))
    if distance > MAX_MARKER_TCP_DISTANCE_M:
        raise VisionError(f'Taught TCP is {distance * 1000:.0f} mm from the marker; check the pose and observation')
    tilt = tool_tilt_deg(tcp)
    if tilt > limits.max_tool_tilt_deg:
        raise VisionError(f'Taught TCP Z axis is {tilt:.1f} deg from base -Z (limit {limits.max_tool_tilt_deg:g}); '
                          'force-guided descent runs along base -Z')
    tcp_offset = tcp_offset if tcp_offset is not None else observation.robot.get('tcp_offset')
    return {'schema_version': GRASP_SCHEMA, 'type': 'taught_marker_grasp', 'role': role,
            'target': observation.target, 'marker_to_tcp': describe(marker_tcp),
            'taught_marker_in_base': describe(observation.base_target), 'taught_tcp_in_base': matrix_to_pose(tcp),
            'tcp_offset': None if tcp_offset is None else vector(tcp_offset, 6, 'TCP offset'),
            'calibration_sha256': calibration.sha256,
            'observation': {'refined': observation.refined, 'joints_rad': observation.robot.get('joints_rad'),
                            'target_in_camera': describe(observation.camera_target),
                            'timestamp': observation.timestamp,
                            'camera_distance_mm': float(np.linalg.norm(observation.camera_target[:3, 3])) * 1000,
                            'quality': observation.quality},
            'taught_at': datetime.now(timezone.utc).isoformat(),
            'conventions': 'metres, UR rotation vectors (rad), robot base frame; marker X right, Y down, Z into marker'}


def write_taught(path, record, *, force=False):
    path = Path(path)
    if path.exists() and not force:
        raise VisionError(f'{path} exists; pass --force to replace the taught grasp')
    atomic_json(path, record)


def load_taught(path):
    path = Path(path)
    try:
        data = path.read_bytes()
        raw = json.loads(data)
    except (OSError, ValueError) as exc:
        raise VisionError(f'Cannot read taught grasp {path}: {exc}') from exc
    if not isinstance(raw, dict) or raw.get('type') != 'taught_marker_grasp':
        raise VisionError(f'{path} is not a taught marker grasp')
    if raw.get('schema_version') != GRASP_SCHEMA:
        raise VisionError(f'Unsupported taught grasp schema_version {raw.get("schema_version")!r}')
    try:
        marker_tcp = check_transform(raw['marker_to_tcp']['matrix'], 'marker_to_tcp', tolerance=1e-6)
        reference_marker = check_transform(raw['taught_marker_in_base']['matrix'], 'taught_marker_in_base',
                                           tolerance=1e-6)
        reference_tcp = pose_to_matrix(raw['taught_tcp_in_base'])
        tcp_offset = None if raw.get('tcp_offset') is None else vector(raw['tcp_offset'], 6, 'tcp_offset')
        taught = raw.get('observation') or {}
        view = taught.get('target_in_camera')
        view = None if view is None else check_transform(view['matrix'], 'observation.target_in_camera')
    except (KeyError, TypeError, ValueError) as exc:
        raise VisionError(f'Malformed taught grasp {path}: {exc}') from exc
    implied = reference_marker @ marker_tcp
    if np.linalg.norm(implied[:3, 3] - reference_tcp[:3, 3]) > 1e-5 or rotation_angle(implied, reference_tcp) > 1e-5:
        raise VisionError(f'Taught grasp {path} is internally inconsistent')
    return GraspModel(role=raw.get('role'), marker_tcp=marker_tcp, reference_marker=reference_marker,
                      reference_tcp=reference_tcp, target=raw.get('target'), tcp_offset=tcp_offset,
                      calibration_sha256=raw.get('calibration_sha256'), source=str(path),
                      sha256=hashlib.sha256(data).hexdigest(), taught_refined=bool(taught.get('refined')),
                      taught_view=view)


def configured_grasp(raw, role):
    """``{"marker_to_tcp": pose, "reference_tcp_pose": pose}`` written by hand instead of taught."""
    unknown = set(raw) - {'marker_to_tcp', 'reference_tcp_pose'}
    if unknown:
        raise ValueError(f'Unknown grasp settings: {sorted(unknown)}')
    if 'marker_to_tcp' not in raw or 'reference_tcp_pose' not in raw:
        raise ValueError('A configured grasp needs marker_to_tcp and reference_tcp_pose (used for the shift limit)')
    marker_tcp = pose_to_matrix(vector(raw['marker_to_tcp'], 6, 'marker_to_tcp'))
    if np.linalg.norm(marker_tcp[:3, 3]) > MAX_MARKER_TCP_DISTANCE_M:
        raise ValueError('marker_to_tcp translation exceeds 0.5 m; units must be metres')
    reference_tcp = pose_to_matrix(vector(raw['reference_tcp_pose'], 6, 'reference_tcp_pose'))
    return GraspModel(role=role, marker_tcp=marker_tcp, reference_marker=reference_tcp @ invert(marker_tcp),
                      reference_tcp=reference_tcp, target=None, tcp_offset=None, calibration_sha256=None,
                      source='vision configuration')


def grasp_for(config, role_name):
    role = config.role(role_name)
    if role.grasp is None:
        raise VisionError(f'No {role_name} grasp configured; teach one with teach-grasp --role {role_name}')
    if isinstance(role.grasp, dict):
        return configured_grasp(role.grasp, role_name)
    return load_taught(config.path(role.grasp))


def target_pose(model, observation, role, *, calibration_sha256, tcp_offset, target, strict=True):
    """Compute and validate the TCP target. Returns ``(T_base_tcp, report)``.

    ``strict`` also requires the observation procedure (refined or not) to match
    teaching; read-only previews relax it and report a warning instead.
    """
    reasons = []
    if not same_target(observation.target, target):
        reasons.append('observation target differs from the configured target; observe again')
    procedure_matches = model.taught_refined is None or model.taught_refined == observation.refined
    if strict and not procedure_matches:
        reasons.append(f'grasp was taught {"with" if model.taught_refined else "without"} the refine view but this '
                       f'observation was {"refined" if observation.refined else "not refined"}; teach and run with '
                       'the same observe procedure')
    if model.role not in (None, observation.role):
        reasons.append(f'grasp was taught for the {model.role} target, observation is {observation.role}')
    if model.target is not None and not same_target(model.target, target):
        reasons.append(f'grasp was taught on {model.target}, configured target is {target.describe()}')
    if model.calibration_sha256 is not None and model.calibration_sha256 != calibration_sha256:
        reasons.append('grasp was taught with a different hand-eye calibration; re-teach it')
    if observation.calibration_sha256 != calibration_sha256:
        reasons.append('observation used a different hand-eye calibration')
    if model.tcp_offset is not None:
        if tcp_offset is None:
            reasons.append('active TCP offset unknown; cannot confirm it matches the taught TCP')
        elif not same_tcp(model.tcp_offset, tcp_offset):
            reasons.append(f'active TCP {list(tcp_offset)} differs from the taught TCP {model.tcp_offset}')
    limits = role.limits
    now = observation.base_target
    delta = now @ invert(model.reference_marker)
    yaw, tilt = yaw_tilt_split(delta[:3, :3])
    height_mm = (now[2, 3] - model.reference_marker[2, 3]) * 1000
    used = now
    if role.motion_model == 'planar':
        # The object stays on its support: keep the taught height and tilt, follow XY and yaw.
        used = np.eye(4)
        used[:3, :3] = rotation_z(yaw) @ model.reference_marker[:3, :3]
        used[:3, 3] = [now[0, 3], now[1, 3], model.reference_marker[2, 3]]
        if abs(height_mm) > limits.max_height_change_mm:
            reasons.append(f'marker height changed {height_mm:+.1f} mm '
                           f'(planar limit {limits.max_height_change_mm:g} mm)')
    if math.degrees(tilt) > limits.max_tilt_change_deg:
        reasons.append(f'marker tilt changed {math.degrees(tilt):.1f} deg from the taught pose '
                       f'(limit {limits.max_tilt_change_deg:g} deg)')
    offset_m = np.asarray(role.offset_mm, float) / 1000.0
    pose = used @ translation(offset_m) @ model.marker_tcp
    shift_mm = float(np.linalg.norm(pose[:3, 3] - model.reference_tcp[:3, 3])) * 1000
    turn_deg = math.degrees(rotation_angle(pose, model.reference_tcp))
    tool_tilt = tool_tilt_deg(pose)
    if shift_mm > limits.max_shift_mm:
        reasons.append(f'target is {shift_mm:.1f} mm from the taught pose (limit {limits.max_shift_mm:g} mm)')
    if turn_deg > limits.max_rotation_deg:
        reasons.append(f'target orientation differs {turn_deg:.1f} deg from the taught pose '
                       f'(limit {limits.max_rotation_deg:g} deg)')
    if tool_tilt > limits.max_tool_tilt_deg:
        reasons.append(f'tool Z axis {tool_tilt:.1f} deg from base -Z (limit {limits.max_tool_tilt_deg:g} deg)')
    report = {'role': observation.role, 'grasp_source': model.source, 'motion_model': role.motion_model,
              'offset_mm': list(role.offset_mm), 'tcp_pose_m_rad': matrix_to_pose(pose),
              'taught_tcp_pose_m_rad': matrix_to_pose(model.reference_tcp),
              'shift_from_taught_mm': shift_mm, 'rotation_from_taught_deg': turn_deg,
              'marker_yaw_change_deg': math.degrees(yaw), 'marker_tilt_change_deg': math.degrees(tilt),
              'marker_height_change_mm': height_mm, 'tool_tilt_deg': tool_tilt,
              'observation_refined': observation.refined, 'same_procedure_as_teaching': procedure_matches}
    if reasons:
        raise TargetValidationError(f'{observation.role} target refused', reasons)
    return pose, report
