"""Per-frame target measurement, multi-frame averaging and quality gates.

ArUco markers: OpenCV ``CORNER_REFINE_SUBPIX`` corners are unbiased on average but
pulled ~0.15-0.4 px towards the marker centre (the corners of a blurred square are
rounded), which inflates depth by ~0.5-1 %. Each outer edge is therefore located
to subpixel precision along its normal (area method, independent of blur and pixel
phase), a line is fitted per side in undistorted coordinates, and the corners are
the line intersections. The pose comes from ``solvePnPGeneric`` with
``SOLVEPNP_IPPE_SQUARE`` plus Levenberg-Marquardt refinement.

ChArUco boards: charuco-realsense's ``CharucoDetector``/``estimate_pose``. OpenCV
4.11's ``CharucoDetector.detectBoard`` returns corners biased by about +0.5 px in x
and y, so one extra ``cv2.cornerSubPix`` pass is applied to the ChArUco corners
before pose estimation (half-window 0.25 x corner spacing, clamped to 2..8 px).

Averaging assumes a stationary camera: corners are averaged per ID over the frames
that pass the per-frame gates and the final pose is solved once from the means.
"""
from dataclasses import dataclass, field
import math

import cv2
import numpy as np

from .errors import QualityGateError, VisionError
from .transforms import mean_rotation, rotation_angle

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-4)
UNDISTORT_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 1e-12)
#: Edge points scattering more than this from their fitted side indicate occlusion or clutter.
EDGE_FIT_MAX_RMS_PX = 0.5


def camera_matrix(intrinsics):
    k = np.array([[intrinsics['fx'], 0.0, intrinsics['ppx']],
                  [0.0, intrinsics['fy'], intrinsics['ppy']],
                  [0.0, 0.0, 1.0]])
    coeffs = np.asarray(intrinsics.get('coeffs') or [0.0] * 5, dtype=float)
    return k, coeffs


def charuco_subpix_window(spacing_px):
    """Half-window for the extra ChArUco cornerSubPix pass."""
    return int(min(8, max(2, round(0.25 * spacing_px))))


def view_angle_deg(transform, point):
    """Angle between the line of sight to ``point`` (target frame) and the target normal."""
    ray = transform[:3, :3] @ np.asarray(point, float) + transform[:3, 3]
    normal = transform[:3, 2]
    cosine = abs(float(normal @ ray)) / max(float(np.linalg.norm(ray)), 1e-12)
    return math.degrees(math.acos(min(1.0, cosine)))


def refine_marker_edges(gray, corners, module_px, intrinsics, samples=40):
    """Corners from straight-line fits to the four outer marker edges.

    Returns ``(corners, edge_rms_px)`` or ``(None, reason)``. Needs a white quiet
    zone of at least ~0.6 module plus 2 px around the marker.
    """
    image = gray.astype(np.float32)
    k, coeffs = camera_matrix(intrinsics)
    corners = np.asarray(corners, float).reshape(4, 2)
    centre = corners.mean(axis=0)
    inner, outer = 0.45 * module_px, 0.6 * module_px + 2.0
    steps = np.arange(-inner, outer + 1e-9, 0.05)
    black_zone, white_zone = steps <= -0.75 * inner, steps >= 0.7 * outer
    lines, residuals = [], []
    for side in range(4):
        a, b = corners[side], corners[(side + 1) % 4]
        direction = (b - a) / np.linalg.norm(b - a)
        normal = np.array([-direction[1], direction[0]])
        if normal @ (centre - (a + b) / 2) > 0:
            normal = -normal  # outward, from the black border into the white quiet zone
        bases = a + np.outer(np.linspace(0.15, 0.85, samples), b - a)
        xy = bases[:, None, :] + steps[None, :, None] * normal
        profiles = cv2.remap(image, xy[..., 0].astype(np.float32), xy[..., 1].astype(np.float32), cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_REPLICATE)
        dark, light = profiles[:, black_zone].mean(axis=1), profiles[:, white_zone].mean(axis=1)
        usable = light - dark > 20
        if usable.sum() < samples // 2:
            return None, 'low contrast or occluded marker border'
        fraction = np.clip((profiles[usable] - dark[usable, None]) / (light - dark)[usable, None], -0.5, 1.5)
        # Area method: for a step at e, integral of the normalized profile equals steps[-1] - e.
        offsets = steps[-1] - np.trapezoid(fraction, steps, axis=1)
        points = bases[usable] + offsets[:, None] * normal
        undistorted = cv2.undistortPointsIter(points.reshape(-1, 1, 2), k, coeffs, None, None,
                                              UNDISTORT_CRITERIA).reshape(-1, 2)
        mean = undistorted.mean(axis=0)
        line_dir = np.linalg.svd(undistorted - mean)[2][0]
        line_normal = np.array([-line_dir[1], line_dir[0]])
        lines.append((line_normal, float(line_normal @ mean)))
        distance = (undistorted - mean) @ line_normal
        residuals.append(float(np.sqrt(np.mean(distance ** 2))) * (k[0, 0] + k[1, 1]) / 2)
    intersections = []
    for side in range(4):
        (n1, c1), (n2, c2) = lines[side - 1], lines[side]
        matrix = np.array([n1, n2])
        if abs(np.linalg.det(matrix)) < 1e-9:
            return None, 'degenerate marker edges'
        intersections.append(np.linalg.solve(matrix, [c1, c2]))
    normalized = np.column_stack([np.array(intersections), np.ones(4)])
    pixels, _ = cv2.projectPoints(normalized, np.zeros(3), np.zeros(3), k, coeffs)
    rms = float(np.sqrt(np.mean(np.square(residuals))))
    if rms > EDGE_FIT_MAX_RMS_PX:
        return None, f'marker edge fit RMS {rms:.2f} px > {EDGE_FIT_MAX_RMS_PX} px (occlusion or clutter)'
    return pixels.reshape(4, 2), rms


def _pose_matrix(rotation, tvec):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(tvec, float).reshape(3)
    return result


@dataclass
class Measurement:
    """One frame (or the multi-frame mean) of a target in the camera optical frame."""
    reasons: list = field(default_factory=list)
    corners: dict = field(default_factory=dict)  # id -> (x, y) px
    camera_target: np.ndarray | None = None      # T_cam_target
    reprojection_rmse_px: float | None = None
    pixels_per_module: float | None = None
    view_angle_deg: float | None = None
    details: dict = field(default_factory=dict)

    @property
    def ok(self):
        return not self.reasons and self.camera_target is not None


class ArucoMeasurer:
    def __init__(self, target, quality):
        self.target, self.quality = target, quality
        try:
            dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, target.dictionary))
        except AttributeError as exc:
            raise VisionError(f'Unknown ArUco dictionary {target.dictionary}') from exc
        params = cv2.aruco.DetectorParameters()
        params.markerBorderBits = target.border_bits
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementMaxIterations = 100
        params.cornerRefinementMinAccuracy = 0.001
        self.modules = dictionary.markerSize + 2 * target.border_bits
        self._detector = cv2.aruco.ArucoDetector(dictionary, params)
        half = target.marker_length_m / 2
        # OpenCV's IPPE_SQUARE layout (Y up, Z towards the camera); converted below.
        self._object = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], float)

    def measure(self, gray, intrinsics):
        corners, ids, _ = self._detector.detectMarkers(gray)
        ids = [] if ids is None else [int(i) for i in np.asarray(ids).ravel()]
        matches = [c for c, i in zip(corners, ids) if i == self.target.marker_id]
        if not matches:
            return Measurement(reasons=[f'marker {self.target.marker_id} not detected '
                                        f'(visible IDs {sorted(set(ids))})'])
        if len(matches) > 1:
            return Measurement(reasons=[f'marker {self.target.marker_id} detected {len(matches)} times; '
                                        'identity ambiguous'])
        points = np.asarray(matches[0], float).reshape(4, 2)
        sides = [np.linalg.norm(points[i] - points[(i + 1) % 4]) for i in range(4)]
        refined, edge = refine_marker_edges(gray, points, min(sides) / self.modules, intrinsics)
        if refined is None:
            return Measurement(reasons=[edge])
        result = self.solve({i: refined[i] for i in range(4)}, intrinsics, gray.shape[::-1])
        result.details['edge_fit_rms_px'] = edge
        return result

    def solve(self, corners, intrinsics, image_size):
        points = np.array([corners[i] for i in range(4)], float)
        k, coeffs = camera_matrix(intrinsics)
        count, rvecs, tvecs, _ = cv2.solvePnPGeneric(self._object, points, k, coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        candidates = []
        for rvec, tvec in zip(rvecs[:count], tvecs[:count]):
            rvec, tvec = cv2.solvePnPRefineLM(self._object, points, k, coeffs, rvec.copy(), tvec.copy())
            projected, _ = cv2.projectPoints(self._object, rvec, tvec, k, coeffs)
            rmse = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - points) ** 2, axis=1))))
            rotation = cv2.Rodrigues(rvec)[0]
            depth_ok = bool(np.all((self._object @ rotation.T + tvec.ravel())[:, 2] > 0))
            candidates.append((not depth_ok, rmse, rotation, tvec.ravel()))
        if not candidates:
            return Measurement(reasons=['marker pose could not be solved'], corners=corners)
        candidates.sort(key=lambda c: (c[0], c[1]))
        bad_depth, rmse, rotation, tvec = candidates[0]
        # IPPE_SQUARE frame (Y up, Z out) -> marker frame (Y down, Z into marker): flip Y and Z.
        transform = _pose_matrix(rotation @ np.diag([1.0, -1.0, -1.0]), tvec)
        sides = [np.linalg.norm(points[i] - points[(i + 1) % 4]) for i in range(4)]
        result = Measurement(corners=corners, camera_target=transform, reprojection_rmse_px=rmse,
                             pixels_per_module=float(min(sides)) / self.modules,
                             view_angle_deg=view_angle_deg(transform, self.target.centre))
        if len(candidates) > 1:
            alternative = _pose_matrix(candidates[1][2] @ np.diag([1.0, -1.0, -1.0]), candidates[1][3])
            result.details['ippe_alternative_rmse_px'] = candidates[1][1]
            result.details['ippe_alternative_rotation_deg'] = math.degrees(rotation_angle(transform, alternative))
        if bad_depth:
            result.reasons.append('marker pose has non-positive depth')
        self._gate(result, points, image_size)
        return result

    def _gate(self, result, points, image_size):
        q = self.quality
        if result.reprojection_rmse_px > q.max_reprojection_rmse_px:
            result.reasons.append(f'reprojection RMSE {result.reprojection_rmse_px:.3f} px > '
                                  f'{q.max_reprojection_rmse_px:g} px')
        if result.pixels_per_module < q.min_pixels_per_module:
            result.reasons.append(_resolution_reason(result.pixels_per_module, q.min_pixels_per_module))
        if result.view_angle_deg > q.max_view_angle_deg:
            result.reasons.append(f'viewing angle {result.view_angle_deg:.1f} deg > {q.max_view_angle_deg:g} deg')
        _edge_gate(result, points, image_size, q.min_edge_margin_px)


def _resolution_reason(value, limit):
    return f'{value:.2f} px per module < {limit:g}; move closer'


def _edge_gate(result, points, image_size, margin):
    width, height = image_size
    pts = np.asarray(points, float).reshape(-1, 2)
    closest = float(min(pts[:, 0].min(), pts[:, 1].min(), width - 1 - pts[:, 0].max(), height - 1 - pts[:, 1].max()))
    if closest < margin:
        result.reasons.append(f'target within {closest:.1f} px of the image border (minimum {margin:g} px)')


class CharucoMeasurer:
    def __init__(self, target, quality):
        try:
            from charuco_realsense.detect import CharucoDetector
            from charuco_realsense.pose import PoseOptions, estimate_pose
        except ImportError as exc:
            raise VisionError('ChArUco targets need charuco-realsense: '
                              'pip install -e ../charuco-realsense --no-deps') from exc
        self.target, self.quality = target, quality
        self._estimate = estimate_pose
        self._options = PoseOptions(min_corners=quality.min_charuco_corners, min_coverage=quality.min_board_coverage,
                                    max_rmse_px=quality.max_reprojection_rmse_px,
                                    max_error_px=max(quality.max_reprojection_rmse_px * 3, 1.0))
        self._detector = CharucoDetector(target.board, target.border_bits, refine=False)
        record = target.record
        self.columns = record['chessboard_squares'][0] - 1
        self.modules = target.board.getDictionary().markerSize + 2 * target.border_bits
        self.board_ids = set(record['marker_ids'])
        self.spacing_ratio = record['square_length_m'] / record['marker_length_m']

    def _spacing_px(self, detection):
        ids = list(detection.charuco_ids)
        points = dict(zip(ids, detection.charuco_corners.reshape(-1, 2)))
        distances = []
        for corner_id, point in points.items():
            for neighbour in (corner_id + 1, corner_id + self.columns):
                if neighbour in points and (neighbour != corner_id + 1 or (corner_id + 1) % self.columns):
                    distances.append(float(np.linalg.norm(points[neighbour] - point)))
        if distances:
            return float(np.median(distances))
        sides = [np.linalg.norm(c[i] - c[(i + 1) % 4]) for c in detection.marker_corners for i in range(4)]
        return float(np.median(sides)) * self.spacing_ratio if sides else 0.0

    def measure(self, gray, intrinsics):
        detection = self._detector.detect(gray, intrinsics)
        own = [(i, c) for i, c in zip(detection.marker_ids, detection.marker_corners) if i in self.board_ids]
        if not own:
            return Measurement(reasons=[f'no markers of board {self.target.board_name!r} detected'])
        own_ids = [i for i, _ in own]
        if len(own_ids) != len(set(own_ids)):
            return Measurement(reasons=['duplicate board marker IDs detected; board identity ambiguous'])
        if len(detection.charuco_ids) < 4:
            return Measurement(reasons=[f'only {len(detection.charuco_ids)} ChArUco corners detected'])
        spacing = self._spacing_px(detection)
        refined = cv2.cornerSubPix(gray, np.ascontiguousarray(detection.charuco_corners, np.float32).reshape(-1, 1, 2),
                                   (charuco_subpix_window(spacing),) * 2, (-1, -1), SUBPIX_CRITERIA)
        corners = {int(i): p.astype(float) for i, p in zip(detection.charuco_ids, refined.reshape(-1, 2))}
        sides = [np.linalg.norm(c[i] - c[(i + 1) % 4]) for _, c in own for i in range(4)]
        result = self.solve(corners, intrinsics, gray.shape[::-1], marker_sides=sides)
        result.details['corner_spacing_px'] = spacing
        result.details['subpix_half_window_px'] = charuco_subpix_window(spacing)
        return result

    def solve(self, corners, intrinsics, image_size, marker_sides=None):
        from charuco_realsense.detect import Detection
        ids = sorted(corners)
        pixels = np.array([corners[i] for i in ids], np.float32).reshape(-1, 1, 2)
        detection = Detection(tuple(int(v) for v in image_size), (), np.empty((0, 4, 2), np.float32),
                              tuple(ids), pixels, 0)
        pose = self._estimate(self.target.board, detection, intrinsics, self._options)
        if pose is None:
            return Measurement(reasons=['board pose could not be solved (too few or collinear corners)'],
                               corners=corners)
        transform = _pose_matrix(np.asarray(pose['rotation_matrix']), pose['tvec_m'])
        result = Measurement(corners=corners, camera_target=transform,
                             reprojection_rmse_px=float(pose['reprojection_rmse_px']),
                             view_angle_deg=view_angle_deg(transform, self.target.centre),
                             details={'corner_count': pose['corner_count'],
                                      'board_coverage_fraction': pose['board_coverage_fraction'],
                                      'pose_quality_reasons': list(pose['quality_reasons'])})
        if marker_sides:
            result.pixels_per_module = float(min(marker_sides)) / self.modules
        result.reasons.extend(f'board pose quality: {r}' for r in pose['quality_reasons'])
        q = self.quality
        if result.pixels_per_module is not None and result.pixels_per_module < q.min_pixels_per_module:
            result.reasons.append(_resolution_reason(result.pixels_per_module, q.min_pixels_per_module))
        if result.view_angle_deg > q.max_view_angle_deg:
            result.reasons.append(f'viewing angle {result.view_angle_deg:.1f} deg > {q.max_view_angle_deg:g} deg')
        _edge_gate(result, pixels, image_size, q.min_edge_margin_px)
        return result


def measurer_for(target, quality):
    return ArucoMeasurer(target, quality) if target.kind == 'aruco' else CharucoMeasurer(target, quality)


def _summarize(rejected, limit=5):
    ordered = sorted(rejected.values(), key=lambda item: -item['count'])[:limit]
    return [f"{item['count']}x {item['example']}" for item in ordered]


def combine(measurer, frames, intrinsics, image_size, quality, *, attempts=None, rejected=None):
    """Average accepted frames and apply multi-frame gates; raise QualityGateError on failure."""
    rejected = rejected or {}
    report = {'frames_accepted': len(frames), 'frames_attempted': attempts or len(frames),
              'frame_rejections': _summarize(rejected, limit=20)}
    if len(frames) < quality.frames:
        reasons = [f'only {len(frames)} of {report["frames_attempted"]} frames passed '
                   f'(need {quality.frames})'] + _summarize(rejected)
        raise QualityGateError(f'{measurer.target.label} observation rejected', reasons, report)
    common = set(frames[0].corners)
    for frame in frames[1:]:
        common &= set(frame.corners)
    reasons = []
    stacks = {i: np.array([f.corners[i] for f in frames]) for i in sorted(common)}
    if len(frames) > 1:
        jitter = {i: float(np.sqrt(np.var(s[:, 0], ddof=1) + np.var(s[:, 1], ddof=1))) for i, s in stacks.items()}
        worst = max(jitter.values()) if jitter else 0.0
        report['corner_jitter_max_px'] = worst
        report['corner_jitter_rms_px'] = float(np.sqrt(np.mean(np.square(list(jitter.values()))))) if jitter else 0.0
        if worst > quality.max_corner_jitter_px:
            reasons.append(f'corner jitter {worst:.3f} px > {quality.max_corner_jitter_px:g} px '
                           '(camera or target moving?)')
        transforms = [f.camera_target for f in frames]
        mean_t = np.mean([t[:3, 3] for t in transforms], axis=0)
        mean_r = mean_rotation(transforms)
        spread_mm = max(float(np.linalg.norm(t[:3, 3] - mean_t)) for t in transforms) * 1000
        spread_deg = max(math.degrees(rotation_angle(t, mean_r)) for t in transforms)
        report.update(position_spread_mm=spread_mm, rotation_spread_deg=spread_deg)
        if spread_mm > quality.max_position_spread_mm:
            reasons.append(f'per-frame position spread {spread_mm:.2f} mm > {quality.max_position_spread_mm:g} mm')
        if spread_deg > quality.max_rotation_spread_deg:
            reasons.append(f'per-frame rotation spread {spread_deg:.2f} deg > {quality.max_rotation_spread_deg:g} deg '
                           '(possible planar pose ambiguity)')
    mean_corners = {i: s.mean(axis=0) for i, s in stacks.items()}
    if measurer.target.kind == 'charuco':
        sides = [f.pixels_per_module * measurer.modules for f in frames if f.pixels_per_module]
        final = measurer.solve(mean_corners, intrinsics, image_size, marker_sides=sides)
    else:
        final = measurer.solve(mean_corners, intrinsics, image_size)
    reasons.extend(final.reasons)
    report.update(final.details)
    report.update(reprojection_rmse_px=final.reprojection_rmse_px, pixels_per_module=final.pixels_per_module,
                  view_angle_deg=final.view_angle_deg, corner_count=len(mean_corners))
    if reasons or final.camera_target is None:
        raise QualityGateError(f'{measurer.target.label} observation rejected', reasons, report)
    final.details = report
    return final


def capture(measurer, camera, quality, intrinsics_check):
    """Read frames until ``quality.frames`` pass or ``max_attempt_frames`` are used.

    ``intrinsics_check(frame)`` returns the intrinsics to use, or raises.
    """
    accepted, rejected, attempts, size, intrinsics = [], {}, 0, None, None
    while len(accepted) < quality.frames and attempts < quality.max_attempt_frames:
        frame = camera.read()
        attempts += 1
        intrinsics = intrinsics_check(frame)
        gray = frame.bgr if frame.bgr.ndim == 2 else cv2.cvtColor(frame.bgr, cv2.COLOR_BGR2GRAY)
        if size is not None and gray.shape[::-1] != size:
            raise VisionError('Camera image size changed during an observation')
        size = gray.shape[::-1]
        measurement = measurer.measure(gray, intrinsics)
        if measurement.ok:
            accepted.append(measurement)
            continue
        for reason in measurement.reasons:
            entry = rejected.setdefault(_bucket(reason), {'count': 0, 'example': reason})
            entry['count'] += 1
    if not accepted:
        raise QualityGateError(f'{measurer.target.label} not observed in {attempts} frames', _summarize(rejected),
                               {'frames_attempted': attempts, 'frame_rejections': _summarize(rejected, limit=20)})
    return combine(measurer, accepted, intrinsics, size, quality, attempts=attempts, rejected=rejected)


def _bucket(reason):
    """Group reasons that differ only in their numbers, e.g. 'RMSE 1.2 px > 1 px'."""
    head = reason.split(' (')[0]
    return ' '.join(w for w in head.split() if not any(ch.isdigit() for ch in w))
