"""Offline simulation for vision tests: fake RTDE robot, rendered wrist camera, ground truth.

Never imports ur_rtde or pyrealsense2. Importing this module raises unittest.SkipTest
when the optional vision dependencies are missing.
"""
from contextlib import contextmanager
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

try:
    import cv2  # noqa: F401
    import numpy as np
except ImportError as exc:  # pragma: no cover - depends on installed extras
    raise unittest.SkipTest(f'vision extra not installed: {exc}') from None

from ur_pick_place.config import Recipe
from ur_pick_place.controller import PickPlaceController, SequenceError
from ur_pick_place.robot import RTDERobot
from ur_pick_place.vision import synthetic
from ur_pick_place.vision.kinematics import nominal_flange
from ur_pick_place.vision.targets import ArucoTarget
from ur_pick_place.vision.transforms import describe, invert, matrix_to_pose, pose_to_matrix, rotvec_to_matrix

ROOT = Path(__file__).resolve().parents[1]
INTRINSICS = {'width': 1280, 'height': 720, 'fx': 912.4, 'fy': 910.8, 'ppx': 641.7, 'ppy': 362.3,
              'model': 'brown_conrady', 'coeffs': [0.012, -0.035, 0.0004, -0.0006, 0.009]}
TCP_OFFSET = [0.0, 0.0, 0.2, 0.0, 0.0, 0.0]
OBSERVE_PICK = [0.3, -1.75, 2.1, -1.92, -1.57, 0.3]
OBSERVE_PLACE = [-0.05, -1.75, 2.1, -1.92, -1.57, -0.05]
HOME = [0.1, -1.6, 1.8, -1.77, -1.57, 0.1]
TABLE_Z = 0.08
TANK = ArucoTarget(marker_id=7, dictionary='DICT_5X5_100', marker_length_m=0.030, border_bits=1)
PLACE_MARKER = ArucoTarget(marker_id=12, dictionary='DICT_5X5_100', marker_length_m=0.030, border_bits=1)


def make_transform(rotvec=(0, 0, 0), xyz=(0, 0, 0)):
    result = np.eye(4)
    result[:3, :3] = rotvec_to_matrix(rotvec)
    result[:3, 3] = xyz
    return result


def face_up(x, y, z=TABLE_Z, yaw_deg=0.0):
    """Marker lying on the table: marker Z (into the marker) points down."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    result = np.eye(4)
    result[:3, :3] = np.column_stack([[c, s, 0], [s, -c, 0], [0, 0, -1]])
    result[:3, 3] = [x, y, z]
    return result


# Camera optical frame in the flange frame (ground truth): looks along flange Z, offset sideways.
CAMERA_IN_FLANGE = make_transform((0.012, -0.02, -1.5708), (0.004, 0.072, 0.038))
# The controller's calibrated kinematics differ slightly from nominal DH values.
KINEMATIC_DELTA = make_transform((0.0004, -0.0003, 0.0002), (0.0011, -0.0007, 0.0009))
# TCP pose expressed in the marker frame (taught relationship, ground truth).
MARKER_TCP = make_transform((0, 0, math.pi / 2), (0.05, 0.012, 0.02))


def true_flange(q):
    return nominal_flange(q) @ KINEMATIC_DELTA


def camera_view_point(q, height=TABLE_Z):
    """Where the optical axis of the true camera hits the table at joints q."""
    camera = true_flange(q) @ CAMERA_IN_FLANGE
    origin, axis = camera[:3, 3], camera[:3, 2]
    scale = (height - origin[2]) / axis[2]
    return origin + scale * axis


def calibration_dict(camera_in_flange=CAMERA_IN_FLANGE, *, verdict='good', tcp_offset=TCP_OFFSET,
                     schema_version=1, intrinsics=INTRINSICS):
    in_tcp = invert(pose_to_matrix(tcp_offset)) @ camera_in_flange
    return {'schema_version': schema_version, 'type': 'eye_in_hand',
            'camera_in_flange': {**describe(camera_in_flange), 'rpy_deg': [0, 0, 0],
                                 'quaternion_xyzw': [0, 0, 0, 1]},
            'camera_in_tcp': {**describe(in_tcp), 'tcp_offset': list(tcp_offset)},
            'board_in_base': describe(np.eye(4)), 'intrinsics': dict(intrinsics), 'verdict': verdict,
            'metrics': {'reprojection_rmse_px': 0.18, 'leave_one_out': {'position_rms_mm': 0.6}}}


def handeye_error(mm=(1.5, -1.0, 1.2), deg=(0.3, -0.25, 0.4)):
    return make_transform(np.radians(deg), np.asarray(mm) / 1000)


class FakeRTDE:
    """RTDE control+receive with synchronous moves, simple force-mode descent and ground truth."""

    def __init__(self, q=OBSERVE_PICK, tcp_offset=TCP_OFFSET, surfaces=None):
        self.q = list(q)
        self.flange = true_flange(self.q)
        self.tcp = list(tcp_offset)
        self.registry = {}
        self.safety_mode = 1
        self.connected = True
        self.events = []
        self.reachable = lambda pose: True
        self.surfaces = surfaces or (lambda x, y: -1.0)
        self.force_active = False
        self.contact = False
        self._moves = 0

    # Receive interface
    def isConnected(self):
        return self.connected

    def getSafetyMode(self):
        return self.safety_mode

    def getTimestamp(self):
        import time
        return time.monotonic()

    def getActualQ(self):
        return list(self.q)

    def getActualQd(self):
        return [0.0] * 6

    def tcp_matrix(self):
        return self.flange @ pose_to_matrix(self.tcp)

    def getActualTCPPose(self):
        return matrix_to_pose(self.tcp_matrix())

    def getActualTCPForce(self):
        return [0.0, 0.0, 25.0 if self.contact else 0.0, 0.0, 0.0, 0.0]

    # Control interface
    def _flange_for(self, q):
        return self.registry.get(tuple(round(v, 9) for v in q), None) if q else self.flange

    def getForwardKinematics(self, q=None, tcp_offset=None):
        flange = self._flange_for(q) if q else self.flange
        if flange is None:
            flange = true_flange(q)
        return matrix_to_pose(flange @ pose_to_matrix(self.tcp if tcp_offset is None else tcp_offset))

    def getTCPOffset(self):
        return list(self.tcp)

    def isJointsWithinSafetyLimits(self, q):
        return True

    def isPoseWithinSafetyLimits(self, pose):
        return bool(self.reachable(pose))

    def getInverseKinematicsHasSolution(self, pose, qnear=None, *args):
        return bool(self.reachable(pose))

    def moveJ(self, q, speed, acceleration, asynchronous=False):
        self.events.append(('moveJ', list(q)))
        self.q = list(q)
        self.flange = self._flange_for(q) if self._flange_for(q) is not None else true_flange(q)
        self.contact = False
        return True

    def moveL(self, pose, speed, acceleration, asynchronous=False):
        self.events.append(('moveL', list(pose)))
        self.flange = pose_to_matrix(pose) @ invert(pose_to_matrix(self.tcp))
        self._moves += 1
        # Joint values for a Cartesian move are synthetic but unique and stable.
        self.q = [v + 1e-3 * self._moves for v in OBSERVE_PICK]
        self.registry[tuple(round(v, 9) for v in self.q)] = self.flange.copy()
        self.contact = False
        return True

    def zeroFtSensor(self):
        return True

    def initPeriod(self):
        return 0.0

    def waitPeriod(self, began):
        return None

    def forceMode(self, frame, selection, wrench, kind, limits):
        self.force_active = True
        tcp = self.tcp_matrix()
        surface = self.surfaces(tcp[0, 3], tcp[1, 3])
        step = limits[2] * 0.008 * 40  # accelerated descent
        new_z = max(tcp[2, 3] - step, surface)
        self.contact = new_z <= surface + 1e-9
        self.flange[2, 3] += new_z - tcp[2, 3]
        return True

    def forceModeStop(self):
        self.force_active = False
        self.events.append(('forceModeStop',))
        return True

    def stopScript(self):
        return True

    def disconnect(self):
        self.connected = False


class FakeCamera:
    """Renders the visible targets from the fake robot's true camera pose."""

    def __init__(self, rtde, scene, *, camera_in_flange=CAMERA_IN_FLANGE, intrinsics=INTRINSICS, noise=1.5,
                 seed=3, live_intrinsics=None, shake_px=0.0):
        self.rtde, self.scene = rtde, scene  # scene: list of (target, T_base_target)
        self.camera_in_flange, self.intrinsics = camera_in_flange, intrinsics
        self.live_intrinsics = live_intrinsics or intrinsics
        self.noise, self.rng, self.shake_px = noise, np.random.default_rng(seed), shake_px
        self.textures = {}
        self._cache = (None, None)
        self.reads = 0
        self.closed = False

    def render_clean(self):
        key = self.rtde.flange.tobytes() + b''.join(pose.tobytes() for _, pose in self.scene)
        if self._cache[0] != key:
            base_camera = self.rtde.flange @ self.camera_in_flange
            items = []
            for target, base_target in self.scene:
                if id(target) not in self.textures:
                    self.textures[id(target)] = synthetic.texture_for(target)
                items.append((self.textures[id(target)], invert(base_camera) @ base_target))
            self._cache = (key, synthetic.render(items, self.intrinsics))
        return self._cache[1]

    def read(self):
        self.reads += 1
        clean = self.render_clean()
        if self.shake_px:
            dx, dy = self.rng.normal(0, self.shake_px, 2)
            clean = cv2.warpAffine(clean, np.float32([[1, 0, dx], [0, 1, dy]]), clean.shape[::-1],
                                   borderMode=cv2.BORDER_REPLICATE)
        return SimpleNamespace(bgr=synthetic.noisy_frame(clean, self.rng, self.noise),
                               intrinsics=dict(self.live_intrinsics))

    def close(self):
        self.closed = True


class FakeGripper:
    def __init__(self, rtde):
        self.rtde, self.events, self.holding = rtde, [], False
        self.grip_tcp = None

    def require_ready(self):
        self.events.append('ready')

    def open(self):
        self.events.append('open')
        self.holding = False

    def grip(self):
        self.events.append('grip')
        self.grip_tcp = self.rtde.tcp_matrix().copy()
        self.holding = True

    def require_holding(self):
        if not self.holding:
            raise SequenceError('not holding')

    @contextmanager
    def probe_guard(self):
        yield lambda: None

    def close(self):
        self.events.append('close')


def rtde_robot(recipe, rtde):
    """A real RTDERobot (all its checks) wired to the fake interfaces."""
    robot = RTDERobot.__new__(RTDERobot)
    robot.config, robot.control, robot.receive = recipe, rtde, rtde
    robot.force_active = False
    robot._last_timestamp, robot._last_fresh_at = None, 0.0
    robot.validator = PickPlaceController(recipe)
    import time
    robot._last_fresh_at = time.monotonic()
    return robot


def recipe(**changes):
    raw = json.loads((ROOT / 'examples/rg2.json').read_text())
    raw.update(home_joints=HOME, pick_pose=[-0.45, -0.3, 0.3, 0, math.pi, 0],
               place_pose=[-0.55, -0.1, 0.3, 0, math.pi, 0])
    raw['motion']['settle_s'] = 0
    raw.update(changes)
    return Recipe(**raw)


def vision_config(directory, *, place=True, refine=False, motion_model='full', grasp_files=True, **overrides):
    directory = Path(directory)
    (directory / 'handeye.json').write_text(json.dumps(calibration_dict(overrides.pop('camera_in_flange',
                                                                                      CAMERA_IN_FLANGE))))
    role = {'observe_joints': OBSERVE_PICK, 'motion_model': motion_model,
            'target': {'kind': 'aruco', 'marker_id': 7, 'dictionary': 'DICT_5X5_100', 'marker_length_mm': 30},
            'refine': {'enabled': refine, 'distance_m': 0.25}}
    if grasp_files:
        role['grasp'] = 'grasp_pick.json'
    raw = {'handeye_calibration': 'handeye.json', 'pick': role,
           'quality': {'frames': 6, 'max_attempt_frames': 12},
           'robot': {'settle_s': 0, 'active_tcp_offset': TCP_OFFSET}}
    if place:
        raw['place'] = {'observe_joints': OBSERVE_PLACE, 'refine': {'enabled': refine, 'distance_m': 0.25},
                        'target': {'kind': 'aruco', 'marker_id': 12, 'dictionary': 'DICT_5X5_100',
                                   'marker_length_mm': 30}}
        if grasp_files:
            raw['place']['grasp'] = 'grasp_place.json'
    for key, value in overrides.items():
        raw[key] = value
    path = directory / 'vision.json'
    path.write_text(json.dumps(raw, indent=2))
    return path
