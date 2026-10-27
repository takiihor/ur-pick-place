"""Direct ur_rtde backend; forceMode is refreshed every RTDE control period."""
import math
import time

from .config import vector
from .controller import PickPlaceController, ProbeResult, SequenceError


def rotation_distance(a, b):
    def quaternion(r):
        angle = math.sqrt(sum(v * v for v in r))
        if angle < 1e-12:
            return [1.0, 0.0, 0.0, 0.0]
        return [math.cos(angle / 2), *(v * math.sin(angle / 2) / angle for v in r)]
    dot = abs(sum(x * y for x, y in zip(quaternion(a), quaternion(b))))
    return 2 * math.acos(min(1.0, dot))


class RTDERobot:
    def __init__(self, recipe):
        from rtde_control import RTDEControlInterface
        from rtde_receive import RTDEReceiveInterface
        self.config = recipe
        self.control = self.receive = None
        self.force_active = False
        self.stop_failed = False
        self._last_timestamp = None
        self._last_fresh_at = time.monotonic()
        self.validator = PickPlaceController(recipe)
        try:
            self.control = RTDEControlInterface(recipe.robot_ip, 125.0)
            self.receive = RTDEReceiveInterface(recipe.robot_ip, 125.0)
            self._read()
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _require_success(result, operation):
        if result is not True:
            raise SequenceError(f'{operation} failed')

    def _read(self):
        if not self.control.isConnected() or not self.receive.isConnected():
            raise SequenceError('Robot disconnected')
        if self.receive.getSafetyMode() not in (1, 2):
            raise SequenceError('Robot is outside normal/reduced safety mode')
        timestamp = self.receive.getTimestamp()
        if not math.isfinite(timestamp):
            raise SequenceError('Invalid RTDE timestamp')
        now = time.monotonic()
        if timestamp != self._last_timestamp:
            self._last_timestamp, self._last_fresh_at = timestamp, now
        elif now - self._last_fresh_at > 0.25:
            raise SequenceError('RTDE feedback stopped updating')
        pose = self.validator.validate_pose(self.receive.getActualTCPPose())
        force = vector(self.receive.getActualTCPForce(), 6, 'TCP force')
        return pose, force

    def preflight(self, home_joints, poses):
        self._read()
        self._require_success(self.control.isJointsWithinSafetyLimits(home_joints), 'Home joint safety check')
        home_pose = self.validator.validate_pose(self.control.getForwardKinematics(home_joints))
        for pose in [home_pose, *poses]:
            self._require_success(self.control.isPoseWithinSafetyLimits(pose), f'Pose safety check {pose}')

    def pause(self, seconds):
        time.sleep(seconds)

    def _move(self, mode, target, speed, acceleration):
        if self.force_active:
            raise SequenceError('Position movement blocked while force mode is active')
        self._read()
        target = vector(target, 6, 'Movement target')
        if mode == 'joint':
            self._require_success(self.control.isJointsWithinSafetyLimits(target), 'Joint safety check')
            tcp = self.validator.validate_pose(self.control.getForwardKinematics(target))
            self._require_success(self.control.isPoseWithinSafetyLimits(tcp), 'Joint target TCP safety check')
        else:
            self.validator.validate_pose(target)
            self._require_success(self.control.isPoseWithinSafetyLimits(target), 'TCP safety check')
            self._require_success(self.control.getInverseKinematicsHasSolution(
                target, self.receive.getActualQ()), 'TCP inverse kinematics check')
        before = self.control.getAsyncOperationProgress()
        started = False
        deadline = time.monotonic() + 120
        try:
            method = self.control.moveJ if mode == 'joint' else self.control.moveL
            self._require_success(method(target, speed, acceleration, True), 'Movement')
            while True:
                self._read()
                progress = self.control.getAsyncOperationProgress()
                if progress >= 0:
                    started = True
                elif started or progress != before:
                    break
                if time.monotonic() > deadline:
                    raise SequenceError('Movement timed out')
                self.pause(0.01)
            self.pause(self.config.motion.settle_s)
            actual, _ = self._read()
            if mode == 'joint':
                joints = vector(self.receive.getActualQ(), 6, 'Actual joints')
                if max(abs(a - b) for a, b in zip(joints, target)) > 0.0035:
                    raise SequenceError('Joint move completed away from the commanded target')
            elif (math.dist(actual[:3], target[:3]) > 0.001
                  or rotation_distance(actual[3:], target[3:]) > 0.005):
                raise SequenceError('Linear move completed away from the commanded target')
        except BaseException:
            try:
                stop = self.control.stopJ if mode == 'joint' else self.control.stopL
                self._require_success(stop(1.0), 'Movement stop')
            except Exception as exc:
                self.stop_failed = True
                raise SequenceError(f'Cannot stop movement: {exc}') from exc
            raise

    def move_joints(self, joints):
        c = self.config.motion
        self._move('joint', joints, c.joint_speed_rad_s, c.joint_accel_rad_s2)

    def move_linear(self, pose, grasp_speed=False):
        c = self.config.motion
        self._move('tcp', pose, (c.grasp_speed_mm_s if grasp_speed else c.transit_speed_mm_s) / 1000,
                   c.tcp_accel_m_s2)

    def _baseline(self):
        self._require_success(self.control.zeroFtSensor(), 'Force sensor zero')
        time.sleep(0.3)
        values = []
        for _ in range(20):
            _, force = self._read()
            values.append(force[2])
            time.sleep(0.01)
        return sum(values) / len(values)

    def stop_force(self):
        if self.force_active:
            self._require_success(self.control.forceModeStop(), 'Force mode stop; further motion blocked')
            self.force_active = False

    def probe(self, settings, *, endpoint_z=None, guard=lambda: None):
        if self.force_active:
            raise SequenceError('Force probe already active')
        guard()
        baseline = self._baseline()
        guard()
        start_pose, _ = self._read()
        floor_z = start_pose[2] - settings.max_descent_mm / 1000
        if endpoint_z is not None and endpoint_z >= start_pose[2]:
            raise SequenceError('Pick endpoint must be below the probe start')
        if endpoint_z is not None:
            floor_z = max(floor_z, endpoint_z)
        floor_pose = list(start_pose)
        floor_pose[2] = floor_z
        self.validator.validate_pose(floor_pose)
        limits = [settings.lateral_tolerance_mm / 1000] * 2 + [settings.speed_mm_s / 1000] + [settings.rotation_tolerance_rad] * 3
        wrench = [0.0, 0.0, -settings.command_force_n, 0.0, 0.0, 0.0]
        deadline = time.monotonic() + settings.timeout_s
        try:
            while True:
                period = self.control.initPeriod()
                guard()
                pose, force = self._read()
                if math.hypot(pose[0] - start_pose[0], pose[1] - start_pose[1]) > settings.lateral_tolerance_mm / 1000:
                    raise SequenceError('Lateral drift exceeded probe limit')
                if rotation_distance(pose[3:], start_pose[3:]) > settings.rotation_tolerance_rad:
                    raise SequenceError('Orientation drift exceeded probe limit')
                if pose[2] > start_pose[2] + 0.005:
                    raise SequenceError('Probe moved upward unexpectedly')
                if pose[2] < start_pose[2] - settings.max_descent_mm / 1000 - 1e-6:
                    raise SequenceError('Probe travel budget exceeded; contact cannot authorize release or grip')
                if endpoint_z is not None and pose[2] <= endpoint_z:
                    result = ProbeResult(pose, 'endpoint')
                    break
                if abs(force[2] - baseline) >= settings.contact_force_n:
                    result = ProbeResult(pose, 'contact')
                    break
                if pose[2] <= floor_z:
                    raise SequenceError('Probe travel budget exhausted without contact')
                if time.monotonic() >= deadline:
                    raise SequenceError('Force probe timed out')
                # Mark active before sending: a lost reply must still attempt stop.
                self.force_active = True
                self._require_success(self.control.forceMode([0.0] * 6, [0, 0, 1, 0, 0, 0],
                                                             wrench, 2, limits), 'Force mode update')
                self.control.waitPeriod(period)
        finally:
            self.stop_force()
        guard()
        return result

    def close(self):
        errors = []
        operations = []
        if self.control is not None:
            operations.extend([self.stop_force, self.control.stopScript, self.control.disconnect])
        if self.receive is not None:
            operations.append(self.receive.disconnect)
        for operation in operations:
            try:
                operation()
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise SequenceError('Robot cleanup failed: ' + '; '.join(errors))
