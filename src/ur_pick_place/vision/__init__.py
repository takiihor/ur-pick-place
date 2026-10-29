"""Wrist-camera perception that turns marker observations into base-frame TCP targets.

Optional: requires the ``vision`` extra (numpy, OpenCV 4.11, pyrealsense2 for live
capture) and, for ChArUco boards, the sibling charuco-generator/charuco-realsense
packages. Nothing here is imported by the base package, its CLI or Robot Bench.

Frames: poses are 4x4 homogeneous matrices in metres, or UR ``[x, y, z, rx, ry, rz]``
(metres, rotation vector in radians). ``T_a_b`` maps coordinates in frame ``b`` into
frame ``a``. The camera optical frame is X right, Y down, Z forward. Marker and board
frames are X right, Y down, Z into the printed surface.
"""
