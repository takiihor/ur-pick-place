"""Detection accuracy on exact renders, the ChArUco bias correction, and quality gates."""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from vision_sim import INTRINSICS, TANK, make_transform

import cv2
import numpy as np

from ur_pick_place.vision import synthetic
from ur_pick_place.vision.config import Camera, Quality
from ur_pick_place.vision.detection import (ArucoMeasurer, camera_matrix, capture, charuco_subpix_window,
                                            measurer_for)
from ur_pick_place.vision.errors import QualityGateError, VisionError
from ur_pick_place.vision.handeye import parse_handeye
from ur_pick_place.vision.observe import intrinsics_checker
from ur_pick_place.vision.targets import ArucoTarget
from ur_pick_place.vision.transforms import rotation_angle


class StaticCamera:
    def __init__(self, clean, intrinsics=INTRINSICS, noise=1.5, seed=5, shake_px=0.0):
        self.clean, self.intrinsics, self.noise = clean, intrinsics, noise
        self.rng, self.shake_px = np.random.default_rng(seed), shake_px

    def read(self):
        image = self.clean
        if self.shake_px:
            dx, dy = self.rng.normal(0, self.shake_px, 2)
            image = cv2.warpAffine(image, np.float32([[1, 0, dx], [0, 1, dy]]), image.shape[::-1],
                                   borderMode=cv2.BORDER_REPLICATE)
        return SimpleNamespace(bgr=synthetic.noisy_frame(image, self.rng, self.noise), intrinsics=dict(self.intrinsics))


def render_aruco(pose, target=TANK, **kwargs):
    return synthetic.render([(synthetic.aruco_texture(target), pose)], INTRINSICS, **kwargs)


def observe(target, clean, quality=None, **camera):
    quality = quality or Quality(frames=8, max_attempt_frames=12)
    return capture(measurer_for(target, quality), StaticCamera(clean, **camera), quality, lambda frame: INTRINSICS)


class ArucoTests(unittest.TestCase):
    def test_pose_matches_ground_truth(self):
        for rotvec, xyz in (((0.35, -0.2, 0.5), (0.03, -0.02, 0.35)), ((0.05, 0.6, -1.0), (-0.07, 0.05, 0.42)),
                            ((0.1, 0.05, math.pi - 0.1), (0.0, 0.01, 0.3))):
            truth = make_transform(rotvec, xyz)
            with self.subTest(rotvec=rotvec):
                result = observe(TANK, render_aruco(truth))
                error_mm = np.linalg.norm(result.camera_target[:3, 3] - truth[:3, 3]) * 1000
                self.assertLess(error_mm, 0.1)
                self.assertLess(math.degrees(rotation_angle(result.camera_target, truth)), 0.15)
                self.assertGreater(result.details['pixels_per_module'], 5)

    def test_edge_fit_removes_corner_shrinkage_of_subpixel_refinement(self):
        truth = make_transform((0.35, -0.2, 0.5), (0.03, -0.02, 0.35))
        gray = np.clip(np.rint(render_aruco(truth)), 0, 255).astype(np.uint8)
        k, coeffs = camera_matrix(INTRINSICS)
        half = TANK.marker_length_m / 2
        corners = np.array([[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]])
        expected = cv2.projectPoints(corners, cv2.Rodrigues(truth[:3, :3])[0], truth[:3, 3], k, coeffs)[0].reshape(4, 2)
        measurer = ArucoMeasurer(TANK, Quality())
        raw, _, _ = measurer._detector.detectMarkers(gray)
        subpix_error = np.abs(np.asarray(raw[0]).reshape(4, 2) - expected).max()
        result = measurer.measure(gray, INTRINSICS)
        fitted = np.array([result.corners[i] for i in range(4)])
        self.assertGreater(subpix_error, 0.1)  # OpenCV's corners are pulled towards the centre
        self.assertLess(np.abs(fitted - expected).max(), 0.05)

    def test_marker_frame_convention(self):
        """Marker X right, Y down, Z into the marker, origin at its centre."""
        truth = np.eye(4)
        truth[:3, 3] = [0, 0, 0.3]  # frontal: marker axes equal camera axes
        result = observe(TANK, render_aruco(truth))
        np.testing.assert_allclose(result.camera_target[:3, :3], np.eye(3), atol=5e-3)

    def test_wrong_marker_reports_visible_ids(self):
        clean = synthetic.render([(synthetic.aruco_texture(TANK), make_transform(xyz=(0, 0, 0.3)))], INTRINSICS)
        with self.assertRaises(QualityGateError) as caught:
            observe(ArucoTarget(9, 'DICT_5X5_100', 0.03), clean)
        self.assertIn('marker 9 not detected', str(caught.exception))
        self.assertIn('[7]', str(caught.exception))


class QualityGateTests(unittest.TestCase):
    def assertRejected(self, fragment, *args, **kwargs):
        with self.assertRaises(QualityGateError) as caught:
            observe(*args, **kwargs)
        self.assertIn(fragment, str(caught.exception))
        return caught.exception

    def test_too_few_pixels_per_module(self):
        clean = render_aruco(make_transform((0.1, 0, 0), (0, 0, 1.3)))
        error = self.assertRejected('px per module', TANK, clean)
        self.assertIn('move closer', str(error))

    def test_oblique_view(self):
        clean = render_aruco(make_transform((math.radians(40), 0, 0), (0, 0, 0.35)))
        self.assertRejected('viewing angle', TANK, clean, Quality(frames=4, max_view_angle_deg=30))

    def test_corner_jitter_from_motion(self):
        clean = render_aruco(make_transform((0.2, 0.1, 0), (0, 0, 0.35)))
        self.assertRejected('corner jitter', TANK, clean, Quality(frames=8, max_corner_jitter_px=0.2), shake_px=0.6)

    def test_reprojection_limit(self):
        clean = render_aruco(make_transform((0.2, 0.1, 0), (0, 0, 0.35)))
        self.assertRejected('reprojection RMSE', TANK, clean, Quality(frames=4, max_reprojection_rmse_px=1e-4))

    def test_image_border(self):
        clean = render_aruco(make_transform((0, 0, 0), (0.175, 0.0, 0.3)))
        self.assertRejected('image border', TANK, clean, Quality(frames=4, min_edge_margin_px=80))

    def test_not_visible(self):
        self.assertRejected('not detected', TANK, np.full((720, 1280), 200.0, np.float32))

    def test_intrinsics_must_match_calibration(self):
        from vision_sim import calibration_dict
        calibration = parse_handeye(calibration_dict())
        check = intrinsics_checker(calibration, Camera())
        frame = StaticCamera(np.full((720, 1280), 200.0, np.float32)).read()
        self.assertEqual(check(frame)['fx'], INTRINSICS['fx'])
        frame.intrinsics['fx'] += 12
        with self.assertRaisesRegex(VisionError, 'wrong camera or profile'):
            check(frame)
        small = SimpleNamespace(bgr=np.zeros((480, 640, 3), np.uint8),
                                intrinsics=dict(INTRINSICS, width=640, height=480))
        with self.assertRaisesRegex(VisionError, 'does not match the calibrated'):
            check(small)


def charuco_target(directory):
    """A front board manifest built like charuco-realsense's tests (no PDF stack needed)."""
    from charuco_generator import __version__ as generator_version
    from charuco_generator.board import canonical_hash, create_board, layout_record
    from charuco_generator.config import load_config
    from ur_pick_place.vision.targets import target_from_config
    generator = Path(__file__).resolve().parents[2] / 'charuco-generator'
    config = load_config(generator / 'config' / 'boards.json')
    manifest = {'schema_version': 1, 'generator_version': generator_version,
                'combined_pdf': None, 'pdf_layouts': {}, 'sha256': {},
                'environment': {'opencv': cv2.__version__},
                'configuration': config.to_dict(), 'configuration_sha256': canonical_hash(config.to_dict()),
                'boards': [layout_record(config.geometry, spec, create_board(config.geometry, spec))
                           for spec in config.boards]}
    (Path(directory) / 'board_manifest.json').write_text(json.dumps(manifest))
    return target_from_config({'kind': 'charuco', 'board': 'front', 'board_manifest': 'board_manifest.json'},
                              directory)


try:
    import charuco_generator.board  # noqa: F401
    import charuco_realsense  # noqa: F401
    HAVE_CHARUCO = (Path(__file__).resolve().parents[2] / 'charuco-generator' / 'config' / 'boards.json').exists()
except ImportError:
    HAVE_CHARUCO = False


@unittest.skipUnless(HAVE_CHARUCO, 'charuco-generator/charuco-realsense not installed')
class CharucoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.target = charuco_target(cls.directory.name)
        cls.truth = make_transform((0.25, 0.15, -0.3), (-0.05, -0.04, 0.27))
        cls.clean = synthetic.render([(synthetic.charuco_texture(cls.target), cls.truth)], INTRINSICS)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def expected_corners(self):
        k, coeffs = camera_matrix(INTRINSICS)
        points = np.asarray(self.target.board.getChessboardCorners(), float)
        rvec = cv2.Rodrigues(self.truth[:3, :3])[0]
        return cv2.projectPoints(points, rvec, self.truth[:3, 3], k, coeffs)[0].reshape(-1, 2)

    def test_detect_board_half_pixel_bias_is_corrected(self):
        from charuco_realsense.detect import CharucoDetector
        gray = np.clip(np.rint(self.clean), 0, 255).astype(np.uint8)
        expected = self.expected_corners()
        raw = CharucoDetector(self.target.board, self.target.border_bits, refine=False).detect(gray, INTRINSICS)
        raw_bias = (raw.charuco_corners.reshape(-1, 2) - expected[list(raw.charuco_ids)]).mean(axis=0)
        np.testing.assert_allclose(raw_bias, [0.5, 0.5], atol=0.1)  # OpenCV 4.11 detectBoard offset
        measured = measurer_for(self.target, Quality()).measure(gray, INTRINSICS)
        ids = sorted(measured.corners)
        refined = np.array([measured.corners[i] for i in ids]) - expected[ids]
        self.assertLess(np.abs(refined.mean(axis=0)).max(), 0.03)
        self.assertLess(np.abs(refined).max(), 0.1)
        self.assertEqual(measured.details['subpix_half_window_px'],
                         charuco_subpix_window(measured.details['corner_spacing_px']))

    def test_board_pose_matches_ground_truth(self):
        result = observe(self.target, self.clean)
        self.assertLess(np.linalg.norm(result.camera_target[:3, 3] - self.truth[:3, 3]) * 1000, 0.1)
        self.assertLess(math.degrees(rotation_angle(result.camera_target, self.truth)), 0.1)
        self.assertGreaterEqual(result.details['corner_count'], 12)

    def test_subpix_window_is_clamped(self):
        self.assertEqual(charuco_subpix_window(4), 2)
        self.assertEqual(charuco_subpix_window(20), 5)
        self.assertEqual(charuco_subpix_window(100), 8)

    def test_board_identity_includes_physical_geometry(self):
        from ur_pick_place.vision.targets import same_target
        description = self.target.describe()
        self.assertTrue(same_target(description, self.target))
        for field, value in (('square_length_mm', 30), ('marker_length_mm', 22.5),
                             ('chessboard_squares', [7, 5]), ('legacy_pattern', True), ('border_bits', 2)):
            with self.subTest(field=field):
                self.assertFalse(same_target(dict(description, **{field: value}), self.target))


if __name__ == '__main__':
    unittest.main()
