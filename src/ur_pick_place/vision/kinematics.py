"""Nominal UR10e forward kinematics, used only as an independent plausibility check.

The controller's calibrated kinematics differ from these nominal DH parameters by
up to a few millimetres, so this never replaces ``getForwardKinematics``. It is
precise enough to detect a wrong declared TCP offset in read-only mode.
"""
import math

import numpy as np

# Universal Robots published DH parameters for the UR10e (metres, radians).
UR10E_DH = {
    'a': [0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0],
    'd': [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655],
    'alpha': [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0],
}


def nominal_flange(q, dh=UR10E_DH):
    """Flange (tool0) pose in the robot base frame as a 4x4 matrix."""
    result = np.eye(4)
    for theta, a, d, alpha in zip(q, dh['a'], dh['d'], dh['alpha']):
        ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
        result = result @ np.array([[ct, -st * ca, st * sa, a * ct],
                                    [st, ct * ca, -ct * sa, a * st],
                                    [0.0, sa, ca, d],
                                    [0.0, 0.0, 0.0, 1.0]])
    return result
