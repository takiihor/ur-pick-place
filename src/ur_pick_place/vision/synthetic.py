"""Exact synthetic camera images of planar targets, for offline tests and simulation.

Every output pixel is supersampled: each sub-pixel ray is undistorted through the
intrinsics, intersected with the target plane, and looked up in a high-resolution
texture of the printed target. Pixel centres are at integer coordinates (OpenCV).
"""
import cv2
import numpy as np

from .detection import UNDISTORT_CRITERIA, camera_matrix
from .transforms import invert

TEXEL_M = 0.00005  # 0.05 mm texture resolution


class PlanarTexture:
    """A grey-level printed target; ``origin_px`` is where the target frame origin lies."""

    def __init__(self, image, texel_m, origin_px, corners_m):
        self.image = image.astype(np.float32)
        self.texel_m = texel_m
        self.origin_px = np.asarray(origin_px, float)
        self.corners_m = np.asarray(corners_m, float)  # outline including white margin, target frame


def aruco_texture(target, margin_m=0.006):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, target.dictionary))
    modules = dictionary.markerSize + 2 * target.border_bits
    per_module = max(1, round(target.marker_length_m / modules / TEXEL_M))
    side = modules * per_module
    texel = target.marker_length_m / side
    margin = round(margin_m / texel)
    image = dictionary.generateImageMarker(target.marker_id, side, borderBits=target.border_bits)
    image = cv2.copyMakeBorder(image, margin, margin, margin, margin, cv2.BORDER_CONSTANT, value=255)
    # Texel (i, j) covers [i, i+1) * texel; the marker centre sits at margin + side / 2.
    centre = margin + side / 2
    extent = (margin + side / 2) * texel
    corners = [[-extent, -extent, 0], [extent, -extent, 0], [extent, extent, 0], [-extent, extent, 0]]
    return PlanarTexture(image, texel, (centre, centre), corners)


def charuco_texture(target, margin_m=0.008):
    record = target.record
    squares_x, squares_y = record['chessboard_squares']
    per_square = max(1, round(record['square_length_m'] / TEXEL_M))
    texel = record['square_length_m'] / per_square
    image = target.board.generateImage((squares_x * per_square, squares_y * per_square), marginSize=0,
                                       borderBits=target.border_bits)
    margin = round(margin_m / texel)
    image = cv2.copyMakeBorder(image, margin, margin, margin, margin, cv2.BORDER_CONSTANT, value=255)
    width, height = squares_x * per_square * texel, squares_y * per_square * texel
    m = margin * texel
    corners = [[-m, -m, 0], [width + m, -m, 0], [width + m, height + m, 0], [-m, height + m, 0]]
    return PlanarTexture(image, texel, (margin, margin), corners)


def texture_for(target):
    return aruco_texture(target) if target.kind == 'aruco' else charuco_texture(target)


def render(textures, intrinsics, *, supersample=4, background=255.0, black=25.0, white=225.0, blur_px=0.5):
    """Render ``[(PlanarTexture, T_cam_target), ...]`` into a float grey image (0..255)."""
    width, height = intrinsics['width'], intrinsics['height']
    k, coeffs = camera_matrix(intrinsics)
    image = np.full((height, width), background, np.float32)
    for texture, camera_target in textures:
        projected, _ = cv2.projectPoints(texture.corners_m, cv2.Rodrigues(camera_target[:3, :3])[0],
                                         camera_target[:3, 3], k, coeffs)
        pts = projected.reshape(-1, 2)
        depth = (camera_target[:3, :3] @ texture.corners_m.T)[2] + camera_target[2, 3]
        if not np.isfinite(pts).all() or np.any(depth <= 0):
            continue
        x0, y0 = np.maximum(np.floor(pts.min(axis=0)).astype(int) - 3, 0)
        x1, y1 = np.minimum(np.ceil(pts.max(axis=0)).astype(int) + 3, [width - 1, height - 1])
        if x0 > x1 or y0 > y1:
            continue
        s = supersample
        fine_x = (np.arange((x1 - x0 + 1) * s) + 0.5) / s - 0.5 + x0
        fine_y = (np.arange((y1 - y0 + 1) * s) + 0.5) / s - 0.5 + y0
        grid_x, grid_y = np.meshgrid(fine_x, fine_y)
        pixels = np.stack([grid_x.ravel(), grid_y.ravel()], axis=1).reshape(-1, 1, 2)
        normalized = cv2.undistortPointsIter(pixels, k, coeffs, None, None, UNDISTORT_CRITERIA).reshape(-1, 2)
        rays = np.column_stack([normalized, np.ones(len(normalized))])
        # Intersect camera rays with the target plane z=0 in target coordinates.
        target_camera = invert(camera_target)
        origin = target_camera[:3, 3]
        directions = rays @ target_camera[:3, :3].T
        with np.errstate(divide='ignore', invalid='ignore'):
            scale = -origin[2] / directions[:, 2]
        points = origin + directions * scale[:, None]
        valid = scale > 0
        u = points[:, 0] / texture.texel_m + texture.origin_px[0] - 0.5
        v = points[:, 1] / texture.texel_m + texture.origin_px[1] - 0.5
        u[~valid], v[~valid] = -10.0, -10.0
        shape = grid_x.shape
        sampled = cv2.remap(texture.image, u.reshape(shape).astype(np.float32), v.reshape(shape).astype(np.float32),
                            cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=-1.0)
        inside = sampled >= 0
        values = np.where(inside, black + (white - black) * sampled / 255.0, background)
        coarse = cv2.resize(values.astype(np.float32), (x1 - x0 + 1, y1 - y0 + 1), interpolation=cv2.INTER_AREA)
        region = image[y0:y1 + 1, x0:x1 + 1]
        coverage = cv2.resize(inside.astype(np.float32), (x1 - x0 + 1, y1 - y0 + 1), interpolation=cv2.INTER_AREA)
        image[y0:y1 + 1, x0:x1 + 1] = np.where(coverage > 0, np.minimum(region, coarse), region)
    if blur_px > 0:
        image = cv2.GaussianBlur(image, (0, 0), blur_px)
    return image


def noisy_frame(clean, rng, sigma=1.5):
    noisy = clean + rng.normal(0.0, sigma, clean.shape) if sigma > 0 else clean
    gray = np.clip(np.rint(noisy), 0, 255).astype(np.uint8)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
