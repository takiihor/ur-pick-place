"""Reuse the teaching session for one interruptible pick/place cycle."""
from contextlib import contextmanager
from dataclasses import asdict
import time

from .config import Recipe, construct, vector
from .controller import PickPlaceController, SequenceError
from .robot import RTDERobot

STEPS = [
    ('home', 'Home & open'), ('approach_pick', 'Pick approach'),
    ('force_pick', 'Force search'), ('grasp', 'Offset & grip'),
    ('lift_home', 'Verify, lift & home'), ('place', 'Force place & retract'),
]


class SequenceStopped(SequenceError):
    pass


def initial_recipe(settings):
    return Recipe(robot_ip=settings.robot_ip, home_joints=[0]*6,
                  pick_pose=[.4, -.1, .25, 0, 3.141592653589793, 0],
                  place_pose=[.45, -.17, .2, 0, 3.141592653589793, 0],
                  workspace_min_m=settings.workspace_min_m,
                  workspace_max_m=settings.workspace_max_m)


def resolve_recipe(base, settings, store, payload, demo):
    if not isinstance(payload, dict):
        raise ValueError('Expected sequence settings')
    if set(payload) - {'home_id', 'pick_id', 'place_id', 'camera_pose', 'options'}:
        raise ValueError('Unknown sequence setting')
    raw = asdict(base)
    options = payload.get('options', {})
    allowed = {'pick_approach_mm', 'place_approach_mm', 'pick_backoff_mm',
               'grasp_offset_mm', 'lift_mm', 'retract_mm', 'pick_probe',
               'place_probe', 'gripper'}
    if not isinstance(options, dict) or set(options) - allowed:
        raise ValueError('Unsupported sequence options')
    raw.update(options)
    raw.update(robot_ip=settings.robot_ip, workspace_min_m=settings.workspace_min_m,
               workspace_max_m=settings.workspace_max_m)
    raw['motion'].update(joint_speed_rad_s=settings.joint_speed_rad_s,
                         joint_accel_rad_s2=settings.joint_accel_rad_s2,
                         transit_speed_mm_s=settings.tcp_speed_mm_s,
                         tcp_accel_m_s2=settings.tcp_accel_m_s2)
    names = {}
    for role, field, coordinates in (('home', 'home_joints', 'joints_rad'),
                                      ('pick', 'pick_pose', 'tcp_pose_m_rad'),
                                      ('place', 'place_pose', 'tcp_pose_m_rad')):
        if role == 'pick' and payload.get('camera_pose') is not None:
            raw[field] = vector(payload['camera_pose'], 6, 'Camera TCP pose (base m/rad)')
            names[role] = 'Camera TCP target'
            continue
        ident = payload.get(role+'_id')
        if not ident:
            raise ValueError(f'Select a saved {role} pose')
        pose = store.get(ident)
        if bool(pose.get('demo')) != demo:
            raise ValueError('Demo and live poses cannot be mixed')
        raw[field] = pose[coordinates]
        names[role] = pose['name']
    recipe = construct(Recipe, raw)
    recipe.gripper.require_live_ready()
    return recipe, names


class SessionRobot(RTDERobot):
    """Borrow interfaces owned by ManualService; never reconnect or close them."""
    def __init__(self, recipe, control, receive, tick, *, demo=False):
        self.config, self.control, self.receive = recipe, control, receive
        self.tick, self.demo = tick, demo
        self.force_active = False
        self.stop_failed = False
        self._last_timestamp = None
        self._last_fresh_at = time.monotonic()
        self.validator = PickPlaceController(recipe)

    def _read(self):
        self.tick()
        return super()._read()

    def pause(self, seconds):
        end = time.monotonic()+seconds
        while True:
            self.tick()
            remaining = end-time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(.01, remaining))

    def _baseline(self):
        self.tick()
        self._require_success(self.control.zeroFtSensor(), 'Force sensor zero')
        self.pause(.3)
        values = []
        for _ in range(20):
            _, force = self._read()
            values.append(force[2])
            self.pause(.01)
        return sum(values)/len(values)

    def _move(self, mode, target, speed, acceleration):
        if self.force_active:
            raise SequenceError('Position movement blocked while force mode is active')
        self._read()
        if mode == 'joint':
            self._require_success(self.control.isJointsWithinSafetyLimits(target), 'Joint safety check')
            self.validator.validate_pose(self.control.getForwardKinematics(target))
        else:
            self.validator.validate_pose(target)
            self._require_success(self.control.isPoseWithinSafetyLimits(target), 'TCP safety check')
        before = self.control.getAsyncOperationProgress()
        started = False
        deadline = time.monotonic()+120
        try:
            method = self.control.moveJ if mode == 'joint' else self.control.moveL
            self._require_success(method(target, speed, acceleration, True), 'Sequence movement')
            while True:
                self._read()
                progress = self.control.getAsyncOperationProgress()
                if progress >= 0:
                    started = True
                elif started or progress != before:
                    break
                if time.monotonic() > deadline:
                    raise SequenceError('Sequence movement timed out')
                self.pause(.01)
        except BaseException:
            try:
                if mode == 'joint':
                    self.control.stopJ(1.0)
                else:
                    self.control.stopL(1.0)
            except Exception as exc:
                self.stop_failed = True
                raise SequenceError(f'Cannot stop sequence movement: {exc}') from exc
            raise
        self.pause(self.config.motion.settle_s)

    def move_joints(self, joints):
        c = self.config.motion
        self._move('joint', joints, c.joint_speed_rad_s, c.joint_accel_rad_s2)

    def move_linear(self, pose, grasp_speed=False):
        c = self.config.motion
        self._move('tcp', pose, (c.grasp_speed_mm_s if grasp_speed else c.transit_speed_mm_s)/1000,
                   c.tcp_accel_m_s2)

    def probe(self, settings, *, endpoint_z=None, guard=lambda: None):
        if self.demo:
            self.control.prepare_probe(endpoint_z if endpoint_z is not None else self.config.place_pose[2]+.02)
        return super().probe(settings, endpoint_z=endpoint_z, guard=guard)

    def close(self):
        # Connection belongs to the manual worker.
        self.stop_force()


class DemoGripper:
    """Simulates verification only; never imports a hardware driver."""
    def __init__(self, config, tick, pause, *, state=None):
        from types import SimpleNamespace
        self.config, self.tick, self.pause = config, tick, pause
        self.state = state if state is not None else SimpleNamespace()
        if not hasattr(self.state, 'gripper_holding'):
            self.state.gripper_holding = False

    @property
    def holding(self):
        return self.state.gripper_holding

    @holding.setter
    def holding(self, value):
        self.state.gripper_holding = value

    def require_ready(self):
        self.tick()
        if self.holding:
            raise SequenceError('Demo gripper already holding an object')

    def open(self):
        self.tick()
        self.holding = False
        self.pause(self.config.settle_s)

    def grip(self):
        self.tick()
        self.holding = True
        self.pause(self.config.settle_s)

    def require_holding(self):
        self.tick()
        if not self.holding:
            raise SequenceError('Demo grip check failed')

    @contextmanager
    def probe_guard(self):
        yield self.tick

    def close(self):
        pass
