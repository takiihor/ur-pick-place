"""Strict JSON configuration. Poses: [x,y,z,rx,ry,rz], metres/radians, base frame."""
from dataclasses import dataclass, field, fields
import json
import math
from pathlib import Path


def number(value, name, low=None, high=None, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    if positive and value <= 0 or low is not None and value < low or high is not None and value > high:
        raise ValueError(f'{name} is outside its permitted range')
    return float(value)


def vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{name} must contain {size} numbers')
    return [number(v, name) for v in value]


def construct(cls, value):
    if not isinstance(value, dict):
        raise ValueError(f'{cls.__name__} must be a JSON object')
    unknown = set(value) - {f.name for f in fields(cls)}
    if unknown:
        raise ValueError(f'Unknown {cls.__name__} settings: {sorted(unknown)}')
    try:
        return cls(**value)
    except TypeError as exc:
        raise ValueError(f'Invalid {cls.__name__}: {exc}') from exc


@dataclass
class Probe:
    contact_force_n: float = 10.0
    command_force_n: float = 12.0
    speed_mm_s: float = 5.0
    max_descent_mm: float = 150.0
    timeout_s: float = 45.0
    lateral_tolerance_mm: float = 5.0
    rotation_tolerance_rad: float = 0.1

    def __post_init__(self):
        number(self.contact_force_n, 'contact_force_n', low=1, high=100)
        number(self.command_force_n, 'command_force_n', low=self.contact_force_n, high=100)
        number(self.speed_mm_s, 'speed_mm_s', positive=True, high=50)
        number(self.max_descent_mm, 'max_descent_mm', positive=True, high=500)
        number(self.timeout_s, 'timeout_s', positive=True, high=120)
        number(self.lateral_tolerance_mm, 'lateral_tolerance_mm', positive=True, high=20)
        number(self.rotation_tolerance_rad, 'rotation_tolerance_rad', positive=True, high=0.5)


@dataclass
class Motion:
    joint_speed_rad_s: float = 0.1
    joint_accel_rad_s2: float = 0.2
    transit_speed_mm_s: float = 20.0
    grasp_speed_mm_s: float = 10.0
    tcp_accel_m_s2: float = 0.1
    settle_s: float = 0.5

    def __post_init__(self):
        for key in ('joint_speed_rad_s', 'joint_accel_rad_s2'):
            number(getattr(self, key), key, positive=True, high=0.4)
        for key in ('transit_speed_mm_s', 'grasp_speed_mm_s'):
            number(getattr(self, key), key, positive=True, high=100)
        number(self.tcp_accel_m_s2, 'tcp_accel_m_s2', positive=True, high=1)
        number(self.settle_s, 'settle_s', low=0, high=10)


@dataclass
class Gripper:
    kind: str = 'rg2'
    host: str | None = None
    port: int = 41414
    tool_index: int = 0
    timeout_s: float = 8.0
    open_width_mm: float = 80.0
    grip_width_mm: float = 20.0
    grip_force_n: float = 10.0
    release_force_n: float = 10.0
    serial_port: str | None = None
    startup_wait_s: float = 2.0
    settle_s: float = 1.0
    grip_command: str = 'tight'
    grip_min_raw: list = field(default_factory=lambda: [None, None])
    release_max_raw: list = field(default_factory=lambda: [None, None])
    pressure_samples: int = 3

    def __post_init__(self):
        if self.kind not in ('rg2', 'arduino'):
            raise ValueError('gripper.kind must be rg2 or arduino')
        if self.host is not None and (not isinstance(self.host, str) or not self.host.strip()):
            raise ValueError('gripper.host must be a nonempty hostname')
        if type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError('gripper.port must be 1..65535')
        if type(self.tool_index) is not int or self.tool_index not in (0, 1, 2):
            raise ValueError('tool_index must be 0, 1, or 2')
        number(self.timeout_s, 'gripper.timeout_s', positive=True, high=30)
        number(self.open_width_mm, 'open_width_mm', low=0, high=110)
        number(self.grip_width_mm, 'grip_width_mm', low=0, high=110)
        if self.open_width_mm <= self.grip_width_mm:
            raise ValueError('open_width_mm must exceed grip_width_mm')
        for key in ('grip_force_n', 'release_force_n'):
            number(getattr(self, key), key, low=3, high=40)
        number(self.startup_wait_s, 'startup_wait_s', low=0, high=10)
        number(self.settle_s, 'gripper.settle_s', low=0, high=10)
        if self.grip_command not in ('tight', 'loose'):
            raise ValueError('grip_command must be tight or loose')
        if type(self.pressure_samples) is not int or not 1 <= self.pressure_samples <= 20:
            raise ValueError('pressure_samples must be an integer in 1..20')
        for key in ('grip_min_raw', 'release_max_raw'):
            values = getattr(self, key)
            if not isinstance(values, list) or len(values) != 2:
                raise ValueError(f'{key} must contain two raw thresholds')
            for v in values:
                if v is not None:
                    number(v, key, low=0, high=3069)
        if self.kind == 'arduino':
            if not isinstance(self.serial_port, str) or not self.serial_port.strip():
                raise ValueError('Arduino requires gripper.serial_port')
            for grip, release in zip(self.grip_min_raw, self.release_max_raw):
                if grip is not None and release is not None and release >= grip:
                    raise ValueError('Each release threshold must be below its grip threshold')

    def require_live_ready(self):
        if self.kind == 'arduino' and any(v is None for v in self.grip_min_raw + self.release_max_raw):
            raise ValueError('Calibrate both grip_min_raw and release_max_raw before Arduino execution')


@dataclass
class Recipe:
    robot_ip: str
    home_joints: list
    pick_pose: list
    place_pose: list
    workspace_min_m: list
    workspace_max_m: list
    pick_approach_mm: float = 100.0
    place_approach_mm: float = 100.0
    pick_backoff_mm: float = 50.0
    grasp_offset_mm: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    lift_mm: float = 100.0
    retract_mm: float = 100.0
    return_home: bool = False
    pick_probe: Probe = field(default_factory=Probe)
    place_probe: Probe = field(default_factory=Probe)
    motion: Motion = field(default_factory=Motion)
    gripper: Gripper = field(default_factory=Gripper)

    def __post_init__(self):
        if not isinstance(self.robot_ip, str) or not self.robot_ip.strip():
            raise ValueError('robot_ip must be a nonempty hostname')
        for key in ('home_joints', 'pick_pose', 'place_pose'):
            setattr(self, key, vector(getattr(self, key), 6, key))
        if any(abs(q) > 2 * math.pi for q in self.home_joints):
            raise ValueError('home_joints must be radians within +/-2*pi')
        for key in ('workspace_min_m', 'workspace_max_m', 'grasp_offset_mm'):
            setattr(self, key, vector(getattr(self, key), 3, key))
        if any(a >= b for a, b in zip(self.workspace_min_m, self.workspace_max_m)):
            raise ValueError('workspace minimum must be below maximum on every axis')
        for key in ('pick_approach_mm', 'place_approach_mm', 'lift_mm', 'retract_mm'):
            number(getattr(self, key), key, positive=True, high=500)
        number(self.pick_backoff_mm, 'pick_backoff_mm', low=0, high=500)
        if type(self.return_home) is not bool:
            raise ValueError('return_home must be a JSON boolean')
        for key, cls in (('pick_probe', Probe), ('place_probe', Probe), ('motion', Motion), ('gripper', Gripper)):
            value = getattr(self, key)
            if isinstance(value, dict):
                setattr(self, key, construct(cls, value))
            elif not isinstance(value, cls):
                raise ValueError(f'{key} has invalid settings')

    @classmethod
    def load(cls, path):
        return construct(cls, json.loads(Path(path).read_text()))
