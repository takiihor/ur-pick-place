"""Where is the flange? Calibrated forward kinematics, or read-only TCP feedback.

With an RTDE control session the flange pose is ``getForwardKinematics(q, [0]*6)``:
calibrated kinematics, independent of the active TCP. Read-only (RTDE receive)
use derives it from ``getActualTCPPose() @ inv(tcp_offset)``, which is valid only
when the active TCP equals the TCP recorded with the hand-eye calibration. RTDE
receive cannot report the active TCP, so the operator must declare it and the
declaration is cross-checked against nominal UR10e kinematics.
"""
from dataclasses import dataclass
import math
import time

import numpy as np

from ..config import vector
from .errors import CalibrationError, VisionError
from .kinematics import nominal_flange
from .transforms import invert, matrix_to_pose, pose_to_matrix, rotation_angle

#: Two TCP offsets are "the same" within this tolerance.
TCP_MATCH_M = 1e-4
TCP_MATCH_RAD = 1e-3


def same_tcp(a, b):
    ta, tb = pose_to_matrix(a), pose_to_matrix(b)
    return (float(np.linalg.norm(ta[:3, 3] - tb[:3, 3])) <= TCP_MATCH_M
            and rotation_angle(ta, tb) <= TCP_MATCH_RAD)


@dataclass
class RobotSnapshot:
    q: list
    base_flange: np.ndarray
    tcp_pose: list
    tcp_offset: list | None
    source: str
    timestamp: float

    def summary(self):
        return {'joints_rad': list(self.q), 'tcp_pose_m_rad': list(self.tcp_pose),
                'flange_pose_m_rad': matrix_to_pose(self.base_flange),
                'tcp_offset': None if self.tcp_offset is None else list(self.tcp_offset),
                'flange_source': self.source}


def _check_safety(receive):
    if not receive.isConnected():
        raise VisionError('Robot RTDE receive interface disconnected')
    if receive.getSafetyMode() not in (1, 2):
        raise VisionError('Robot is outside normal/reduced safety mode')


class ControlPoseSource:
    """Flange from the controller's calibrated kinematics (requires RTDE control)."""
    read_only = False

    def __init__(self, control, receive):
        self.control, self.receive = control, receive

    def snapshot(self):
        _check_safety(self.receive)
        q = vector(self.receive.getActualQ(), 6, 'Actual joints')
        flange = pose_to_matrix(self.control.getForwardKinematics(q, [0.0] * 6))
        return RobotSnapshot(q=q, base_flange=flange,
                             tcp_pose=vector(self.receive.getActualTCPPose(), 6, 'Actual TCP pose'),
                             tcp_offset=vector(self.control.getTCPOffset(), 6, 'Active TCP offset'),
                             source='getForwardKinematics(q, [0]*6)', timestamp=time.time())


class ReceivePoseSource:
    """Flange from actual TCP feedback; refuses unless the declared TCP is the calibrated one."""
    read_only = True

    def __init__(self, receive, calibration, declared_tcp, checks):
        if declared_tcp is None:
            raise CalibrationError(
                'Read-only observation needs the active TCP offset: set robot.active_tcp_offset or pass '
                '--active-tcp (RTDE receive cannot report it)')
        declared_tcp = vector(declared_tcp, 6, 'Declared active TCP offset')
        if not same_tcp(declared_tcp, calibration.tcp_offset):
            raise CalibrationError(
                f'Active TCP {declared_tcp} differs from the TCP recorded with the hand-eye calibration '
                f'{calibration.tcp_offset}; read-only flange recovery refused. Activate the calibrated TCP '
                'or observe with an RTDE control session (--execute)')
        self.receive, self.calibration, self.checks = receive, calibration, checks
        self.tcp_offset = list(calibration.tcp_offset)

    def snapshot(self):
        _check_safety(self.receive)
        q = vector(self.receive.getActualQ(), 6, 'Actual joints')
        tcp_pose = vector(self.receive.getActualTCPPose(), 6, 'Actual TCP pose')
        tcp = pose_to_matrix(tcp_pose)
        offset = pose_to_matrix(self.tcp_offset)
        # Independent check that the declared TCP is really active.
        nominal = nominal_flange(q) @ offset
        distance = float(np.linalg.norm(nominal[:3, 3] - tcp[:3, 3])) * 1000
        angle = math.degrees(rotation_angle(nominal, tcp))
        if distance > self.checks.tcp_check_mm or angle > self.checks.tcp_check_deg:
            raise CalibrationError(
                f'Actual TCP pose differs from nominal UR10e kinematics with the declared TCP by '
                f'{distance:.1f} mm / {angle:.2f} deg (limits {self.checks.tcp_check_mm:g} mm / '
                f'{self.checks.tcp_check_deg:g} deg); the active TCP is probably not the calibrated one')
        return RobotSnapshot(q=q, base_flange=tcp @ invert(offset), tcp_pose=tcp_pose,
                             tcp_offset=list(self.tcp_offset), source='getActualTCPPose() @ inv(tcp_offset)',
                             timestamp=time.time())


def require_stationary(before, after, tolerance_rad):
    moved = max(abs(a - b) for a, b in zip(before.q, after.q))
    if moved > tolerance_rad:
        raise VisionError(f'Robot moved {moved:.5f} rad during image capture (limit {tolerance_rad:g} rad); '
                          'observation discarded')
