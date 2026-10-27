"""Strict JSON vision configuration, kept separate from the pick/place recipe.

Paths (calibration, manifests, taught grasps) are relative to the vision file.
Lengths are mm unless the key says _m; joints are radians; angles are degrees.
"""
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

from ..config import construct, number, vector


def _flag(value, name):
    if type(value) is not bool:
        raise ValueError(f'{name} must be a JSON boolean')


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in {low}..{high}')


def _nested(owner, key, cls, optional=False):
    value = getattr(owner, key)
    if value is None and optional:
        return
    if isinstance(value, dict):
        setattr(owner, key, construct(cls, value))
    elif not isinstance(value, cls):
        raise ValueError(f'{key} has invalid settings')


@dataclass
class Camera:
    serial: str | None = None
    width: int = 1280
    height: int = 720
    fps: int = 30
    warmup_s: float = 2.0
    lock_exposure: bool = False
    intrinsics: str = 'calibration'
    max_intrinsics_mismatch_px: float = 5.0

    def __post_init__(self):
        if self.serial is not None and (not isinstance(self.serial, str) or not self.serial.strip()):
            raise ValueError('camera.serial must be a nonempty string or null')
        _integer(self.width, 'camera.width', 64, 8192)
        _integer(self.height, 'camera.height', 64, 8192)
        _integer(self.fps, 'camera.fps', 1, 120)
        number(self.warmup_s, 'camera.warmup_s', low=0, high=30)
        _flag(self.lock_exposure, 'camera.lock_exposure')
        if self.intrinsics not in ('calibration', 'camera'):
            raise ValueError('camera.intrinsics must be calibration or camera')
        number(self.max_intrinsics_mismatch_px, 'camera.max_intrinsics_mismatch_px', positive=True, high=100)


@dataclass
class Quality:
    frames: int = 10
    max_attempt_frames: int = 30
    max_reprojection_rmse_px: float = 1.0
    max_corner_jitter_px: float = 0.5
    min_pixels_per_module: float = 5.0
    max_view_angle_deg: float = 60.0
    max_position_spread_mm: float = 1.0
    max_rotation_spread_deg: float = 1.0
    min_charuco_corners: int = 6
    min_board_coverage: float = 0.25
    min_edge_margin_px: float = 4.0

    def __post_init__(self):
        _integer(self.frames, 'quality.frames', 1, 300)
        _integer(self.max_attempt_frames, 'quality.max_attempt_frames', self.frames, 1000)
        number(self.max_reprojection_rmse_px, 'max_reprojection_rmse_px', positive=True, high=10)
        number(self.max_corner_jitter_px, 'max_corner_jitter_px', positive=True, high=10)
        number(self.min_pixels_per_module, 'min_pixels_per_module', low=1, high=100)
        number(self.max_view_angle_deg, 'max_view_angle_deg', positive=True, high=85)
        number(self.max_position_spread_mm, 'max_position_spread_mm', positive=True, high=50)
        number(self.max_rotation_spread_deg, 'max_rotation_spread_deg', positive=True, high=30)
        _integer(self.min_charuco_corners, 'min_charuco_corners', 4, 1000)
        number(self.min_board_coverage, 'min_board_coverage', low=0, high=1)
        number(self.min_edge_margin_px, 'min_edge_margin_px', low=0, high=200)


@dataclass
class Refine:
    enabled: bool = False
    distance_m: float = 0.3
    max_move_mm: float = 300.0
    max_rotation_deg: float = 45.0

    def __post_init__(self):
        _flag(self.enabled, 'refine.enabled')
        number(self.distance_m, 'refine.distance_m', low=0.05, high=2.0)
        number(self.max_move_mm, 'refine.max_move_mm', positive=True, high=1000)
        number(self.max_rotation_deg, 'refine.max_rotation_deg', positive=True, high=90)


@dataclass
class Limits:
    max_shift_mm: float = 30.0
    max_rotation_deg: float = 15.0
    max_tilt_change_deg: float = 10.0
    max_height_change_mm: float = 10.0
    max_tool_tilt_deg: float = 20.0

    def __post_init__(self):
        number(self.max_shift_mm, 'max_shift_mm', positive=True, high=500)
        number(self.max_rotation_deg, 'max_rotation_deg', positive=True, high=180)
        number(self.max_tilt_change_deg, 'max_tilt_change_deg', positive=True, high=90)
        number(self.max_height_change_mm, 'max_height_change_mm', positive=True, high=500)
        number(self.max_tool_tilt_deg, 'max_tool_tilt_deg', positive=True, high=90)


@dataclass
class Role:
    """One marker-defined target (pick or place) and how to look at it."""
    target: dict
    observe_joints: list
    grasp: str | dict | None = None
    offset_mm: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    motion_model: str = 'full'
    refine: Refine = field(default_factory=Refine)
    limits: Limits = field(default_factory=Limits)

    def __post_init__(self):
        if not isinstance(self.target, dict):
            raise ValueError('target must be a JSON object')
        self.observe_joints = vector(self.observe_joints, 6, 'observe_joints')
        if any(abs(q) > 2 * math.pi for q in self.observe_joints):
            raise ValueError('observe_joints must be radians within +/-2*pi')
        if self.grasp is not None and not isinstance(self.grasp, (str, dict)):
            raise ValueError('grasp must be a taught-grasp file path or a marker_to_tcp object')
        self.offset_mm = vector(self.offset_mm, 3, 'offset_mm')
        if any(abs(v) > 500 for v in self.offset_mm):
            raise ValueError('offset_mm components must be within +/-500 mm')
        if self.motion_model not in ('full', 'planar'):
            raise ValueError('motion_model must be full or planar')
        _nested(self, 'refine', Refine)
        _nested(self, 'limits', Limits)


@dataclass
class RobotChecks:
    settle_s: float = 0.5
    stationary_joint_rad: float = 2e-4
    active_tcp_offset: list | None = None
    tcp_check_mm: float = 10.0
    tcp_check_deg: float = 2.0

    def __post_init__(self):
        number(self.settle_s, 'robot.settle_s', low=0, high=10)
        number(self.stationary_joint_rad, 'robot.stationary_joint_rad', positive=True, high=0.01)
        if self.active_tcp_offset is not None:
            self.active_tcp_offset = vector(self.active_tcp_offset, 6, 'robot.active_tcp_offset')
        number(self.tcp_check_mm, 'robot.tcp_check_mm', positive=True, high=50)
        number(self.tcp_check_deg, 'robot.tcp_check_deg', positive=True, high=10)


@dataclass
class Preview:
    max_age_s: float = 900.0
    max_deviation_mm: float = 3.0
    max_deviation_deg: float = 1.0

    def __post_init__(self):
        number(self.max_age_s, 'preview.max_age_s', positive=True, high=86400)
        number(self.max_deviation_mm, 'preview.max_deviation_mm', positive=True, high=50)
        number(self.max_deviation_deg, 'preview.max_deviation_deg', positive=True, high=30)


@dataclass
class VisionConfig:
    handeye_calibration: str
    pick: Role
    place: Role | None = None
    camera: Camera = field(default_factory=Camera)
    quality: Quality = field(default_factory=Quality)
    robot: RobotChecks = field(default_factory=RobotChecks)
    preview: Preview = field(default_factory=Preview)
    max_handeye_loo_mm: float | None = None
    max_handeye_reprojection_px: float | None = None
    max_handeye_translation_sigma_mm: float | None = None
    base_dir: str = '.'

    def __post_init__(self):
        if not isinstance(self.handeye_calibration, str) or not self.handeye_calibration:
            raise ValueError('handeye_calibration must be a file path')
        for key, cls, optional in (('pick', Role, False), ('place', Role, True), ('camera', Camera, False),
                                   ('quality', Quality, False), ('robot', RobotChecks, False),
                                   ('preview', Preview, False)):
            _nested(self, key, cls, optional)
        for key in ('max_handeye_loo_mm', 'max_handeye_reprojection_px', 'max_handeye_translation_sigma_mm'):
            if getattr(self, key) is not None:
                number(getattr(self, key), key, positive=True, high=100)

    def path(self, relative):
        return Path(self.base_dir) / relative

    def role(self, name):
        if name not in ('pick', 'place'):
            raise ValueError('role must be pick or place')
        role = getattr(self, name)
        if role is None:
            raise ValueError(f'No {name} target is configured')
        return role

    @classmethod
    def load(cls, path):
        path = Path(path)
        raw = json.loads(path.read_text())
        if not isinstance(raw, dict):
            raise ValueError('Vision configuration must be a JSON object')
        if 'base_dir' in raw:
            raise ValueError('base_dir is derived from the file location')
        return construct(cls, dict(raw, base_dir=str(path.resolve().parent)))
