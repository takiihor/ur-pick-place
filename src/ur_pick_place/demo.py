"""In-process RTDE-shaped simulation for the teaching UI; never imports RTDE."""
import math
import time


class DemoRobot:
    def __init__(self):
        self.connected = True
        self.pose = [.4, -.1, .35, 0.0, math.pi, 0.0]
        self.joints = [0, -1.57, 1.57, -1.57, -1.57, 0]
        self._move = None
        self._token = -1

    def _update(self):
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
        return [0.0]*6

    def getAsyncOperationProgress(self):
        self._update()
        return 0 if self._move else self._token

    def _start(self, attr, target):
        self._update()
        self._move = (attr, list(getattr(self, attr)), list(target), time.monotonic(), .6)
        return True

    def moveL(self, pose, speed, acceleration, asynchronous):
        return self._start('pose', pose)

    def moveJ(self, joints, speed, acceleration, asynchronous):
        return self._start('joints', joints)

    def isPoseWithinSafetyLimits(self, pose):
        return True

    def isJointsWithinSafetyLimits(self, joints):
        return True

    def getForwardKinematics(self, joints):
        return list(self.pose)

    def stopL(self, acceleration=1.0):
        self._update()
        if self._move:
            self._move = None
            self._token = -2 if self._token == -1 else -1

    def stopJ(self, acceleration=1.0):
        self.stopL()

    def stopScript(self):
        self.stopL()
        return True

    def disconnect(self):
        self.connected = False
