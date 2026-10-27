"""In-process RTDE-shaped simulation for the teaching UI; never imports RTDE."""
import math
import time

from .geometry import rotate_base


class DemoRobot:
    def __init__(self):
        self.connected = True
        self.pose = [.4, -.1, .35, 0.0, math.pi, 0.0]
        self.joints = [0, -1.57, 1.57, -1.57, -1.57, 0]
        self._move = None
        self._velocity = None
        self._velocity_at = time.monotonic()
        self._token = -1
        self._duration = .6
        self._force_active = False
        self._force_target = .25
        self._force_speed = .005
        self._force_at = time.monotonic()

    def _update(self):
        now = time.monotonic()
        if self._velocity:
            elapsed = now-self._velocity_at
            for index in range(3):
                self.pose[index] += self._velocity[index]*elapsed
            for index in range(3):
                self.pose = rotate_base(self.pose, index, math.degrees(self._velocity[index+3]*elapsed))
        self._velocity_at = now
        if self._force_active:
            self.pose[2] = max(self._force_target, self.pose[2] - self._force_speed*30*(now-self._force_at))
        self._force_at = now
        if self._move:
            attr, start, target, began, duration = self._move
            fraction = min(1, (time.monotonic()-began)/duration)
            setattr(self, attr, [a+(b-a)*fraction for a, b in zip(start, target)])
            if fraction >= 1:
                self._move = None
                self._token = -2 if self._token == -1 else -1

    def isConnected(self):
        return self.connected

    def getSafetyMode(self):
        return 1

    def getTimestamp(self):
        return time.monotonic()

    def getActualTCPPose(self):
        self._update()
        return list(self.pose)

    def getActualQ(self):
        self._update()
        return list(self.joints)

    def getActualTCPForce(self):
        self._update()
        if self._force_active and self.pose[2] <= self._force_target:
            return [0.0, 0.0, 20.0, 0.0, 0.0, 0.0]
        return [0.0]*6

    def getAsyncOperationProgress(self):
        self._update()
        return 0 if self._move else self._token

    def _start(self, attr, target):
        self._update()
        self._move = (attr, list(getattr(self, attr)), list(target), time.monotonic(), self._duration)
        return True

    def moveL(self, pose, speed, acceleration, asynchronous):
        return self._start('pose', pose)

    def moveJ(self, joints, speed, acceleration, asynchronous):
        return self._start('joints', joints)

    def isPoseWithinSafetyLimits(self, pose):
        return True

    def isJointsWithinSafetyLimits(self, joints):
        return True

    def getInverseKinematicsHasSolution(self, pose, qnear=None):
        return True

    def getForwardKinematics(self, joints):
        return list(self.pose)

    def stopL(self, acceleration=1.0):
        self._update()
        if self._move:
            self._move = None
            self._token = -2 if self._token == -1 else -1
        return True

    def stopJ(self, acceleration=1.0):
        return self.stopL()

    def speedL(self, velocity, acceleration, duration):
        self._update()
        self._velocity = list(velocity)
        return True

    def speedStop(self, acceleration=1.0):
        self._update()
        self._velocity = None
        return True

    def stopScript(self):
        self.forceModeStop()
        self.speedStop()
        self.stopL()
        return True

    def prepare_probe(self, target_z):
        self._force_target = target_z

    def zeroFtSensor(self):
        return True

    def initPeriod(self):
        return time.monotonic()

    def waitPeriod(self, began):
        time.sleep(max(0, .008-(time.monotonic()-began)))

    def forceMode(self, frame, axes, wrench, kind, limits):
        self._update()
        self._force_active = True
        self._force_speed = limits[2]
        return True

    def forceModeStop(self):
        self._update()
        self._force_active = False
        return True

    def disconnect(self):
        self.connected = False
