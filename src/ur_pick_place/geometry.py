"""Small base-frame orientation increments without Euler-angle ambiguity."""
import math

from .config import number, vector


def rotate_base(pose, axis, angle_deg):
    pose = vector(pose, 6, 'TCP pose')
    if type(axis) is not int or axis not in (0, 1, 2):
        raise ValueError('Rotation axis must be X, Y or Z')
    number(angle_deg, 'rotation increment', low=-10, high=10)
    angle = math.sqrt(sum(v * v for v in pose[3:]))
    current = [1.0, 0.0, 0.0, 0.0] if angle < 1e-12 else [
        math.cos(angle / 2), *(v * math.sin(angle / 2) / angle for v in pose[3:])]
    half = math.radians(angle_deg) / 2
    delta = [math.cos(half), 0.0, 0.0, 0.0]
    delta[axis + 1] = math.sin(half)
    a, b, c, d = delta
    e, f, g, h = current
    q = [a*e-b*f-c*g-d*h, a*f+b*e+c*h-d*g,
         a*g-b*h+c*e+d*f, a*h+b*g-c*f+d*e]
    norm = math.sqrt(sum(v*v for v in q))
    q = [v/norm for v in q]
    if q[0] < 0:
        q = [-v for v in q]
    sine = math.sqrt(sum(v*v for v in q[1:]))
    rotation = [0.0]*3 if sine < 1e-12 else [v * 2 * math.atan2(sine, q[0]) / sine for v in q[1:]]
    return pose[:3] + rotation
