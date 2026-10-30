"""Single-use approach tests using Robot Bench's monitored motion worker."""
import math
import time

from ..manual import ManualService
from ..robot import rotation_distance
from .robot_state import same_tcp
from .teaching import digest

MAX_CAPTURE_AGE_S = 60


def check_capture(project, record):
    project.check_inputs()
    if record is None or record['status'] != 'captured':
        raise ValueError('Capture a fresh validation marker before each motion attempt')
    if not 0 <= time.time() - record['timestamp'] <= MAX_CAPTURE_AGE_S:
        raise ValueError('Validation capture expired (60 seconds). Capture again.')
    if digest(project.directory / 'grasp_pick.json') != record['grasp_sha256']:
        raise ValueError('Taught relationship changed. Capture again.')


def pose_error(actual, target):
    return {'position_mm': math.dist(actual[:3], target[:3]) * 1000,
            'rotation_deg': math.degrees(rotation_distance(actual[3:], target[3:]))}


class ValidationMotion(ManualService):
    def __init__(self, settings, store, *, project, record, guard, factory=None):
        self.project, self.record, self.guard = project, record, guard
        super().__init__(settings, store, factory=factory)

    def _read(self):
        feedback = super()._read()
        if not same_tcp(self.control.getTCPOffset(), self.project.settings['tcp']):
            raise ValueError('Active robot TCP differs from the teaching TCP; motion refused')
        return feedback

    def _execute(self, command, payload, generation):
        if command != 'move_tcp':
            return super()._execute(command, payload, generation)
        before = self._read()
        time.sleep(.25)
        after = self._read()
        if max(abs(a-b) for a, b in zip(before['joints_rad'], after['joints_rad'])) > 2e-4:
            raise ValueError('Robot must be stationary before validation motion')
        captured = self.record['capture_robot']
        error = pose_error(after['tcp_pose_m_rad'], captured['tcp_pose_m_rad'])
        if (max(abs(a-b) for a, b in zip(after['joints_rad'], captured['joints_rad'])) > .0035
                or error['position_mm'] > 3 or error['rotation_deg'] > 1):
            raise ValueError('Robot moved since validation capture. Return to observation and capture again.')
        target = self._workspace(self.record['approach_tcp_pose_m_rad'])
        self._workspace(after['tcp_pose_m_rad'])
        if after['safety_mode'] not in (1, 2):
            raise ValueError('Robot must be in normal or reduced safety mode to move')
        if self.control.getInverseKinematicsHasSolution(target, after['joints_rad']) is not True:
            raise ValueError('Validation approach has no inverse kinematics solution')
        if self.control.isPoseWithinSafetyLimits(target) is not True:
            raise ValueError('TCP target is unreachable or outside robot safety limits')
        progress = self.control.getAsyncOperationProgress()
        # SDK preflight can take time. Check the operator's stop and all capture
        # conditions AFTER it, immediately before reserving/sending the movement.
        latest = self._read()
        if latest['safety_mode'] not in (1, 2):
            raise ValueError('Robot safety mode changed during validation preflight')
        if max(abs(a-b) for a, b in zip(after['joints_rad'], latest['joints_rad'])) > 2e-4:
            raise ValueError('Robot moved during validation preflight. Capture again.')
        self.guard()
        with self._lock:
            if generation != self._generation or self._stop_requested.is_set() or self._shutdown.is_set():
                raise ValueError('Validation motion stopped before dispatch')
            self._motion = dict(mode='tcp', before=progress, started=False, deadline=time.monotonic()+120)
            self._message = 'Moving TCP'
        if self.control.moveL(target, self.settings.tcp_speed_mm_s/1000,
                              self.settings.tcp_accel_m_s2, True) is not True:
            raise RuntimeError('Robot rejected the movement')

    def _stop_motion(self):
        # Validation only submits linear moves. Treat a rejected stop as a failure.
        with self._lock:
            motion = self._motion
        if motion and self.control:
            if self.control.stopL(1.0) is not True:
                raise RuntimeError('Robot rejected the validation stop')
            with self._lock:
                self._motion = None
                self._message = 'Movement stopped'
