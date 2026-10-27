"""Load and check the eye-in-hand calibration written by ur10e-handeye (schema 1).

``camera_in_flange.matrix`` maps camera-optical coordinates into the flange (tool0)
frame, so ``p_base = T_base_flange @ camera_in_flange @ p_cam``. Units are metres
and UR rotation-vector radians. Unknown schema versions and ``poor`` verdicts are
refused; inconsistent redundant fields are treated as a corrupt file.
"""
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from ..config import number, vector
from .errors import CalibrationError
from .transforms import check_transform, invert, matrix_to_pose, pose_to_matrix, rotation_angle

SCHEMA_VERSION = 1
ACCEPTED_VERDICTS = ('good', 'acceptable')
KNOWN_VERDICTS = ACCEPTED_VERDICTS + ('poor',)
#: A wrist camera sits within a few decimetres of the flange; larger values usually mean mm units.
MAX_CAMERA_OFFSET_M = 0.5
#: Redundant fields (pose_ur vs matrix, camera_in_tcp vs camera_in_flange) must agree to this.
CONSISTENCY_M = 1e-4
CONSISTENCY_RAD = 1e-4


@dataclass(frozen=True)
class HandEyeCalibration:
    camera_in_flange: np.ndarray
    camera_in_tcp: np.ndarray
    tcp_offset: list
    intrinsics: dict
    verdict: str
    metrics: dict = field(default_factory=dict)
    board_in_base: np.ndarray | None = None
    source: str = ''
    sha256: str = ''

    def summary(self):
        loo = self.metrics.get('leave_one_out') if isinstance(self.metrics.get('leave_one_out'), dict) else {}
        uncertainty = self.metrics.get('camera_in_flange_uncertainty')
        return {'source': self.source, 'sha256': self.sha256, 'verdict': self.verdict,
                'camera_in_flange_pose_ur': matrix_to_pose(self.camera_in_flange),
                'tcp_offset': list(self.tcp_offset),
                'reprojection_rmse_px': self.metrics.get('reprojection_rmse_px'),
                'leave_one_out_position_rms_mm': loo.get('position_rms_mm'),
                'camera_translation_sigma_mm': (uncertainty.get('combined_translation_mm')
                                                 if isinstance(uncertainty, dict) else None),
                'intrinsics': dict(self.intrinsics)}


def _section(raw, key):
    value = raw.get(key)
    if not isinstance(value, dict):
        raise CalibrationError(f'Hand-eye calibration is missing the {key} object')
    return value


def _transform(section, name):
    try:
        matrix = check_transform(section.get('matrix'), f'{name}.matrix', tolerance=1e-5)
    except ValueError as exc:
        raise CalibrationError(str(exc)) from exc
    if 'pose_ur' in section:
        try:
            pose = vector(section['pose_ur'], 6, f'{name}.pose_ur')
        except ValueError as exc:
            raise CalibrationError(str(exc)) from exc
        _consistent(pose_to_matrix(pose), matrix, f'{name}.pose_ur and {name}.matrix')
    return matrix


def _consistent(a, b, what):
    distance = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    angle = rotation_angle(a, b)
    if distance > CONSISTENCY_M or angle > CONSISTENCY_RAD:
        raise CalibrationError(f'Hand-eye calibration is inconsistent: {what} differ by '
                               f'{distance * 1000:.3f} mm / {math.degrees(angle):.4f} deg')


def check_intrinsics(raw, name='intrinsics'):
    if not isinstance(raw, dict):
        raise CalibrationError(f'{name} must be an object')
    try:
        width, height = raw.get('width'), raw.get('height')
        if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
            raise ValueError(f'{name} width/height must be positive integers')
        for key in ('fx', 'fy'):
            number(raw.get(key), f'{name}.{key}', positive=True)
        for key in ('ppx', 'ppy'):
            number(raw.get(key), f'{name}.{key}')
        model = raw.get('model', 'none')
        coeffs = [number(v, f'{name}.coeffs') for v in raw.get('coeffs', [])]
    except ValueError as exc:
        raise CalibrationError(str(exc)) from exc
    if model in ('brown_conrady', 'opencv'):
        if len(coeffs) != 5:
            raise CalibrationError(f'{name} model {model} requires five coefficients (k1, k2, p1, p2, k3)')
    elif model in ('none', 'inverse_brown_conrady', 'modified_brown_conrady'):
        if any(coeffs):
            raise CalibrationError(f'{name} model {model!r} with nonzero coefficients is not supported; '
                                   'use an OpenCV/Brown-Conrady calibration')
    else:
        raise CalibrationError(f'{name} has unsupported distortion model {model!r}')
    return {'width': width, 'height': height, 'fx': float(raw['fx']), 'fy': float(raw['fy']),
            'ppx': float(raw['ppx']), 'ppy': float(raw['ppy']),
            'model': 'brown_conrady' if any(coeffs) else 'none',
            'coeffs': coeffs if any(coeffs) else [0.0] * 5}


def parse_handeye(raw, *, source='', sha256=''):
    if not isinstance(raw, dict):
        raise CalibrationError('Hand-eye calibration must be a JSON object')
    version = raw.get('schema_version')
    if type(version) is not int or version != SCHEMA_VERSION:
        raise CalibrationError(f'Unsupported hand-eye schema_version {version!r}; this build reads {SCHEMA_VERSION}')
    if raw.get('type') != 'eye_in_hand':
        raise CalibrationError(f'Calibration type {raw.get("type")!r} is not eye_in_hand')
    verdict = raw.get('verdict')
    if verdict not in KNOWN_VERDICTS:
        raise CalibrationError(f'Unknown hand-eye verdict {verdict!r}')
    if verdict not in ACCEPTED_VERDICTS:
        raise CalibrationError(f'Hand-eye calibration verdict is {verdict!r}; recalibrate before vision-guided motion')
    flange = _transform(_section(raw, 'camera_in_flange'), 'camera_in_flange')
    if np.linalg.norm(flange[:3, 3]) > MAX_CAMERA_OFFSET_M:
        raise CalibrationError('camera_in_flange translation exceeds 0.5 m; units must be metres')
    tcp_section = _section(raw, 'camera_in_tcp')
    in_tcp = _transform(tcp_section, 'camera_in_tcp')
    try:
        tcp_offset = vector(tcp_section.get('tcp_offset'), 6, 'camera_in_tcp.tcp_offset')
    except ValueError as exc:
        raise CalibrationError(str(exc)) from exc
    _consistent(invert(pose_to_matrix(tcp_offset)) @ flange, in_tcp,
                'camera_in_tcp.matrix and inv(tcp_offset) @ camera_in_flange.matrix')
    board = _transform(raw['board_in_base'], 'board_in_base') if isinstance(raw.get('board_in_base'), dict) else None
    metrics = raw.get('metrics') if isinstance(raw.get('metrics'), dict) else {}
    return HandEyeCalibration(camera_in_flange=flange, camera_in_tcp=in_tcp, tcp_offset=tcp_offset,
                              intrinsics=check_intrinsics(raw.get('intrinsics')), verdict=verdict,
                              metrics=metrics, board_in_base=board, source=source, sha256=sha256)


def load_handeye(path):
    path = Path(path)
    try:
        data = path.read_bytes()
        raw = json.loads(data)
    except (OSError, ValueError) as exc:
        raise CalibrationError(f'Cannot read hand-eye calibration {path}: {exc}') from exc
    return parse_handeye(raw, source=str(path), sha256=hashlib.sha256(data).hexdigest())


def check_metrics(calibration, max_loo_position_rms_mm=None, max_reprojection_rmse_px=None,
                  max_translation_sigma_mm=None):
    """Optional stricter acceptance than the calibration's own verdict."""
    loo = calibration.metrics.get('leave_one_out')
    checks = ((max_loo_position_rms_mm, loo.get('position_rms_mm') if isinstance(loo, dict) else None,
               'leave-one-out position RMS', 'mm'),
              (max_reprojection_rmse_px, calibration.metrics.get('reprojection_rmse_px'),
               'reprojection RMSE', 'px'))
    for limit, value, label, unit in checks:
        if limit is None:
            continue
        if type(value) not in (int, float) or not math.isfinite(value):
            raise CalibrationError(f'Calibration does not report {label}; cannot apply the configured limit')
        if value > limit:
            raise CalibrationError(f'Calibration {label} {value:.3f} {unit} exceeds the configured {limit:g} {unit}')
    if max_translation_sigma_mm is not None:
        uncertainty = calibration.metrics.get('camera_in_flange_uncertainty')
        sigma = uncertainty.get('combined_translation_mm') if isinstance(uncertainty, dict) else None
        if (not isinstance(sigma, list) or len(sigma) != 3 or
                any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in sigma)):
            raise CalibrationError('Calibration does not report combined camera translation 1-sigma; '
                                   'cannot apply the configured limit')
        if max(sigma) > max_translation_sigma_mm:
            raise CalibrationError(f'Calibration camera translation 1-sigma {max(sigma):.3f} mm exceeds '
                                   f'the configured {max_translation_sigma_mm:g} mm per-axis limit')
