"""Look at a target: optional observe/refine moves, multi-frame capture, base-frame pose.

``p_base = T_base_flange @ T_flange_camera @ p_camera``. The robot must be stationary
for the whole capture; joint feedback before and after is compared.
"""
from dataclasses import dataclass, field
import math
import time

import numpy as np

from ..config import vector
from .detection import capture, measurer_for
from .errors import VisionError
from .handeye import check_intrinsics
from .robot_state import require_stationary
from .targets import target_from_config
from .transforms import check_transform, describe, invert, matrix_to_pose, pose_to_matrix, rotation_angle

OBSERVATION_SCHEMA = 1


@dataclass
class Observation:
    role: str
    target: dict
    camera_target: np.ndarray
    base_flange: np.ndarray
    base_camera: np.ndarray
    base_target: np.ndarray
    robot: dict
    quality: dict
    refined: bool
    calibration_sha256: str
    timestamp: float = field(default_factory=time.time)
    notes: list = field(default_factory=list)
    first_pass: dict | None = None

    def to_json(self):
        return {'schema_version': OBSERVATION_SCHEMA, 'type': 'marker_observation', 'role': self.role,
                'target': self.target, 'target_in_base': describe(self.base_target),
                'target_in_camera': describe(self.camera_target),
                'camera_in_base': describe(self.base_camera), 'flange_in_base': describe(self.base_flange),
                'robot': self.robot, 'quality': _jsonable(self.quality), 'refined': self.refined,
                'calibration_sha256': self.calibration_sha256, 'timestamp': self.timestamp,
                'notes': list(self.notes), 'first_pass': self.first_pass,
                'frames': 'camera optical X right/Y down/Z forward; target X right/Y down/Z into surface; metres'}

    @classmethod
    def from_json(cls, raw):
        if not isinstance(raw, dict) or raw.get('type') != 'marker_observation':
            raise VisionError('Not a marker observation file')
        if raw.get('schema_version') != OBSERVATION_SCHEMA:
            raise VisionError(f'Unsupported observation schema_version {raw.get("schema_version")!r}')
        try:
            matrices = {key: check_transform(raw[key]['matrix'], key)
                        for key in ('target_in_base', 'target_in_camera', 'camera_in_base', 'flange_in_base')}
            if raw['role'] not in ('pick', 'place') or type(raw['refined']) is not bool:
                raise ValueError('invalid role or refined flag')
            if not math.isfinite(float(raw['timestamp'])):
                raise ValueError('timestamp must be finite')
            if not isinstance(raw['target'], dict) or not isinstance(raw['quality'], dict):
                raise ValueError('target and quality must be objects')
            vector(raw['robot']['joints_rad'], 6, 'observation joints')
            vector(raw['robot']['tcp_offset'], 6, 'observation TCP offset')
            implied = matrices['camera_in_base'] @ matrices['target_in_camera']
            if not np.allclose(implied, matrices['target_in_base'], atol=1e-6, rtol=0):
                raise ValueError('target_in_base disagrees with camera_in_base @ target_in_camera')
            return cls(role=raw['role'], target=raw['target'], camera_target=matrices['target_in_camera'],
                       base_flange=matrices['flange_in_base'], base_camera=matrices['camera_in_base'],
                       base_target=matrices['target_in_base'], robot=raw['robot'], quality=raw['quality'],
                       refined=raw['refined'], calibration_sha256=raw['calibration_sha256'],
                       timestamp=float(raw['timestamp']), notes=list(raw.get('notes', [])),
                       first_pass=raw.get('first_pass'))
        except (KeyError, TypeError, ValueError) as exc:
            raise VisionError(f'Malformed observation file: {exc}') from exc

    def summary(self):
        return {'role': self.role, 'target': self.target, 'target_in_base_pose_ur': matrix_to_pose(self.base_target),
                'target_in_base_xyz_mm': [v * 1000 for v in self.base_target[:3, 3]],
                'camera_distance_mm': float(np.linalg.norm(self.camera_target[:3, 3])) * 1000,
                'refined': self.refined, 'quality': _jsonable(self.quality), 'notes': list(self.notes),
                'robot': self.robot}


def _jsonable(value):
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def intrinsics_checker(calibration, camera_config):
    """The live stream must match the calibrated profile; return the intrinsics to use."""
    calibrated = calibration.intrinsics

    def check(frame):
        live = check_intrinsics(frame.intrinsics, 'camera intrinsics')
        height, width = frame.bgr.shape[:2]
        if (live['width'], live['height']) != (calibrated['width'], calibrated['height']) or \
                (width, height) != (calibrated['width'], calibrated['height']):
            raise VisionError(f'Camera stream {width}x{height} does not match the calibrated '
                              f'{calibrated["width"]}x{calibrated["height"]} profile')
        mismatch = max(abs(live[k] - calibrated[k]) for k in ('fx', 'fy', 'ppx', 'ppy'))
        if mismatch > camera_config.max_intrinsics_mismatch_px:
            raise VisionError(f'Camera intrinsics differ from the hand-eye calibration by {mismatch:.2f} px '
                              f'(limit {camera_config.max_intrinsics_mismatch_px:g}); wrong camera or profile?')
        return calibrated if camera_config.intrinsics == 'calibration' else live
    return check


def open_camera(camera_config):
    """Open the RealSense colour stream (lazy SDK import) and let exposure settle."""
    try:
        from charuco_realsense.camera import RealsenseCamera, StreamConfig
    except ImportError as exc:
        raise VisionError('Live capture needs charuco-realsense and pyrealsense2 (vision extra)') from exc
    camera = RealsenseCamera(camera_config.serial, StreamConfig(camera_config.width, camera_config.height,
                                                                camera_config.fps))
    try:
        deadline = time.monotonic() + camera_config.warmup_s
        while time.monotonic() < deadline:
            camera.read()
        if camera_config.lock_exposure:
            camera.lock_exposure()
    except BaseException:
        camera.close()
        raise
    return camera


def check_camera_identity(camera, calibration):
    calibration.require_hardware()
    if calibration.camera_serial and getattr(camera, 'serial', None) != calibration.camera_serial:
        raise VisionError('Camera serial differs from the hand-eye calibration; use the calibrated camera')


class RobotMover:
    """Observe/refine moves through an existing RTDERobot, with controller checks first."""

    def __init__(self, robot):
        self.robot = robot

    def move_joints(self, joints):
        control = self.robot.control
        if control.isJointsWithinSafetyLimits(joints) is not True:
            raise VisionError(f'Observe joints {joints} are outside the robot safety limits')
        self.robot.validator.validate_pose(control.getForwardKinematics(joints))
        self.robot.move_joints(joints)

    def move_linear(self, pose):
        control = self.robot.control
        q = self.robot.receive.getActualQ()
        if control.getInverseKinematicsHasSolution(pose, q) is not True:
            raise VisionError(f'Refine pose {pose} has no inverse-kinematics solution')
        self.robot.move_linear(pose)  # workspace + isPoseWithinSafetyLimits inside


def refine_tcp_pose(observation, target, calibration, tcp_offset, refine, current_tcp_pose, view=None):
    """TCP pose for the close-up view of the target.

    With ``view`` (``T_camera_target`` recorded when the grasp was taught) the camera
    returns to exactly that marker-relative viewpoint, so the hand-eye error affects
    teaching and execution identically and cancels in the taught relationship.
    Without it (while teaching), the camera is placed ``refine.distance_m`` in front
    of the target centre along its normal, keeping the current camera X (projected)
    to minimise rotation.
    """
    base_target, base_camera = observation.base_target, observation.base_camera
    if view is not None:
        camera = base_target @ invert(view)
    else:
        centre = (base_target @ np.append(target.centre, 1.0))[:3]
        z = base_target[:3, 2]
        x = base_camera[:3, 0] - (base_camera[:3, 0] @ z) * z
        if np.linalg.norm(x) < 1e-6:
            x = base_target[:3, 0]
        x /= np.linalg.norm(x)
        camera = np.eye(4)
        camera[:3, :3] = np.column_stack([x, np.cross(z, x), z])
        camera[:3, 3] = centre - refine.distance_m * z
    tcp = camera @ invert(calibration.camera_in_flange) @ pose_to_matrix(tcp_offset)
    current = pose_to_matrix(current_tcp_pose)
    move_mm = float(np.linalg.norm(tcp[:3, 3] - current[:3, 3])) * 1000
    turn_deg = math.degrees(rotation_angle(tcp, current))
    if move_mm > refine.max_move_mm or turn_deg > refine.max_rotation_deg:
        raise VisionError(f'Refine move of {move_mm:.0f} mm / {turn_deg:.1f} deg exceeds the configured '
                          f'{refine.max_move_mm:g} mm / {refine.max_rotation_deg:g} deg')
    return matrix_to_pose(tcp)


class Observer:
    def __init__(self, config, calibration, camera, pose_source, *, mover=None, on_step=None, views=None):
        self.config, self.calibration, self.camera = config, calibration, camera
        self.pose_source, self.mover = pose_source, mover
        self.views = views or {}  # role -> taught T_camera_target for the refine view
        self.on_step = on_step or (lambda message: None)
        self._check = intrinsics_checker(calibration, config.camera)
        self._targets = {}

    def target(self, role_name):
        if role_name not in self._targets:
            role = self.config.role(role_name)
            self._targets[role_name] = target_from_config(role.target, self.config.base_dir)
        return self._targets[role_name]

    def look(self, role_name, *, allow_motion=False):
        role = self.config.role(role_name)
        target = self.target(role_name)
        if allow_motion:
            if self.mover is None:
                raise VisionError('Observe motion requires an RTDE control session')
            self.on_step(f'Moving to {role_name} observe joints')
            self.mover.move_joints(vector(role.observe_joints, 6, 'observe_joints'))
        observation = self._capture(role_name, target)
        if not role.refine.enabled:
            return observation
        if not allow_motion:
            observation.notes.append('refine skipped: read-only observation from the current pose')
            return observation
        snapshot = self.pose_source.snapshot()
        view = self.views.get(role_name)
        pose = refine_tcp_pose(observation, target, self.calibration, snapshot.tcp_offset, role.refine,
                               snapshot.tcp_pose, view)
        self.on_step(f'Refining {role_name}: moving camera to the '
                     f'{"taught marker-relative view" if view is not None else "frontal close-up view"}')
        self.mover.move_linear(pose)
        refined = self._capture(role_name, target)
        refined.refined = True
        refined.first_pass = observation.summary()
        return refined

    def _capture(self, role_name, target):
        if self.config.robot.settle_s:
            time.sleep(self.config.robot.settle_s)
        before = self.pose_source.snapshot()
        def checked_frame(frame):
            require_stationary(before, self.pose_source.snapshot(), self.config.robot.stationary_joint_rad)
            return self._check(frame)
        measurement = capture(measurer_for(target, self.config.quality), self.camera, self.config.quality, checked_frame)
        after = self.pose_source.snapshot()
        require_stationary(before, after, self.config.robot.stationary_joint_rad)
        base_camera = before.base_flange @ self.calibration.camera_in_flange
        return Observation(role=role_name, target=target.describe(), camera_target=measurement.camera_target,
                           base_flange=before.base_flange, base_camera=base_camera,
                           base_target=base_camera @ measurement.camera_target, robot=before.summary(),
                           quality=measurement.details, refined=False,
                           calibration_sha256=self.calibration.sha256)
