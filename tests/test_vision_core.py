"""Transforms, hand-eye contract, flange sources, taught grasps and preview checks."""
import copy
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

from vision_sim import (CAMERA_IN_FLANGE, MARKER_TCP, OBSERVE_PICK, TCP_OFFSET, FakeRTDE, calibration_dict,
                        face_up, make_transform, true_flange)

import numpy as np

from ur_pick_place.vision import transforms as tf
from ur_pick_place.vision.config import Limits, Role, VisionConfig
from ur_pick_place.vision.errors import CalibrationError, TargetValidationError, VisionError
from ur_pick_place.vision.grasp import configured_grasp, load_taught, target_pose, teach, write_taught
from ur_pick_place.vision.handeye import check_metrics, load_handeye, parse_handeye
from ur_pick_place.vision.observe import Observation
from ur_pick_place.vision.robot_state import ControlPoseSource, ReceivePoseSource, require_stationary
from ur_pick_place.vision.targets import ArucoTarget, aruco_target, canonical_hash


class TransformTests(unittest.TestCase):
    def test_rotation_vector_round_trip_including_half_turn(self):
        for rotvec in ([0, 0, 0], [0.1, -0.2, 0.3], [0, math.pi, 0], [2.0, -1.0, 1.5], [0, 0, math.pi - 1e-9]):
            matrix = tf.rotvec_to_matrix(rotvec)
            back = tf.rotvec_to_matrix(tf.matrix_to_rotvec(matrix))
            np.testing.assert_allclose(back, matrix, atol=1e-9)

    def test_pose_matrix_round_trip_and_inverse(self):
        pose = [0.4, -0.1, 0.25, 0.1, 3.0, -0.2]
        matrix = tf.pose_to_matrix(pose)
        np.testing.assert_allclose(tf.pose_to_matrix(tf.matrix_to_pose(matrix)), matrix, atol=1e-12)
        np.testing.assert_allclose(matrix @ tf.invert(matrix), np.eye(4), atol=1e-12)

    def test_yaw_tilt_split(self):
        rotation = tf.rotation_z(0.4) @ tf.rotvec_to_matrix([0.05, 0.0, 0.0])
        yaw, tilt = tf.yaw_tilt_split(rotation)
        self.assertAlmostEqual(yaw, 0.4, places=9)
        self.assertAlmostEqual(tilt, 0.05, places=9)

    def test_check_transform_rejects_non_rigid(self):
        bad = np.eye(4)
        bad[0, 0] = 1.1
        with self.assertRaises(ValueError):
            tf.check_transform(bad, 'x')


class HandEyeTests(unittest.TestCase):
    def test_valid_calibration_loads(self):
        calibration = parse_handeye(calibration_dict())
        np.testing.assert_allclose(calibration.camera_in_flange, CAMERA_IN_FLANGE, atol=1e-12)
        self.assertEqual(calibration.tcp_offset, TCP_OFFSET)

    def test_poor_and_unknown_verdicts_refused(self):
        with self.assertRaisesRegex(CalibrationError, "verdict is 'poor'"):
            parse_handeye(calibration_dict(verdict='poor'))
        with self.assertRaisesRegex(CalibrationError, 'Unknown hand-eye verdict'):
            parse_handeye(calibration_dict(verdict='great'))

    def test_unknown_schema_refused(self):
        for version in (2, '1', None, True):
            with self.subTest(version=version), self.assertRaisesRegex(CalibrationError, 'schema_version'):
                parse_handeye(calibration_dict(schema_version=version))

    def test_inconsistent_redundant_fields_refused(self):
        raw = calibration_dict()
        raw['camera_in_tcp']['tcp_offset'] = [0, 0, 0.15, 0, 0, 0]
        with self.assertRaisesRegex(CalibrationError, 'inconsistent'):
            parse_handeye(raw)
        raw = calibration_dict()
        raw['camera_in_flange']['pose_ur'][0] += 0.01
        with self.assertRaisesRegex(CalibrationError, 'inconsistent'):
            parse_handeye(raw)

    def test_millimetre_units_refused(self):
        mm = CAMERA_IN_FLANGE.copy()
        mm[:3, 3] *= 1000
        with self.assertRaisesRegex(CalibrationError, 'metres'):
            parse_handeye(calibration_dict(mm))

    def test_missing_or_bad_intrinsics_refused(self):
        raw = calibration_dict()
        raw['intrinsics']['coeffs'] = [0.1, 0.2]
        with self.assertRaisesRegex(CalibrationError, 'five coefficients'):
            parse_handeye(raw)
        raw = calibration_dict()
        del raw['camera_in_flange']
        with self.assertRaisesRegex(CalibrationError, 'camera_in_flange'):
            parse_handeye(raw)

    def test_file_hash_and_optional_metric_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'handeye.json'
            path.write_text(json.dumps(calibration_dict()))
            calibration = load_handeye(path)
            self.assertEqual(len(calibration.sha256), 64)
            check_metrics(calibration, max_loo_position_rms_mm=1.0)
            with self.assertRaisesRegex(CalibrationError, 'leave-one-out'):
                check_metrics(calibration, max_loo_position_rms_mm=0.5)

    def test_camera_uncertainty_limit_uses_sigma_instead_of_loo(self):
        raw = calibration_dict()
        raw['metrics']['camera_in_flange_uncertainty'] = {'combined_translation_mm': [0.1, 0.7, 0.2]}
        calibration = parse_handeye(raw)
        check_metrics(calibration, max_translation_sigma_mm=1)
        with self.assertRaisesRegex(CalibrationError, '1-sigma'):
            check_metrics(calibration, max_translation_sigma_mm=0.5)
        for sigma in (None, [0.1, float('nan'), 0.1], [-1, 0, 0], [0.1, 0.1]):
            raw['metrics']['camera_in_flange_uncertainty']['combined_translation_mm'] = sigma
            with self.subTest(sigma=sigma), self.assertRaisesRegex(CalibrationError, 'does not report'):
                check_metrics(parse_handeye(raw), max_translation_sigma_mm=1)


class FlangeSourceTests(unittest.TestCase):
    def setUp(self):
        self.calibration = parse_handeye(calibration_dict())
        self.checks = SimpleNamespace(tcp_check_mm=10.0, tcp_check_deg=2.0)

    def test_control_source_uses_calibrated_fk_without_tcp(self):
        rtde = FakeRTDE()
        rtde.tcp = [0.01, 0.0, 0.3, 0, 0, 0]  # any active TCP
        snapshot = ControlPoseSource(rtde, rtde).snapshot()
        np.testing.assert_allclose(snapshot.base_flange, true_flange(OBSERVE_PICK), atol=1e-12)
        self.assertEqual(snapshot.tcp_offset, rtde.tcp)

    def test_receive_source_recovers_flange_with_calibrated_tcp(self):
        rtde = FakeRTDE()
        snapshot = ReceivePoseSource(rtde, self.calibration, TCP_OFFSET, self.checks).snapshot()
        np.testing.assert_allclose(snapshot.base_flange, true_flange(OBSERVE_PICK), atol=1e-12)

    def test_receive_source_refuses_undeclared_or_different_tcp(self):
        rtde = FakeRTDE()
        with self.assertRaisesRegex(CalibrationError, 'needs the active TCP'):
            ReceivePoseSource(rtde, self.calibration, None, self.checks)
        with self.assertRaisesRegex(CalibrationError, 'differs from the TCP recorded'):
            ReceivePoseSource(rtde, self.calibration, [0, 0, 0.18, 0, 0, 0], self.checks)

    def test_receive_source_detects_wrongly_declared_tcp(self):
        rtde = FakeRTDE(tcp_offset=[0, 0, 0.17, 0, 0, 0])  # robot actually runs another TCP
        source = ReceivePoseSource(rtde, self.calibration, TCP_OFFSET, self.checks)
        with self.assertRaisesRegex(CalibrationError, 'probably not the calibrated one'):
            source.snapshot()

    def test_safety_mode_and_motion_refused(self):
        rtde = FakeRTDE()
        rtde.safety_mode = 3
        with self.assertRaisesRegex(VisionError, 'safety mode'):
            ControlPoseSource(rtde, rtde).snapshot()
        before = SimpleNamespace(q=[0] * 6)
        after = SimpleNamespace(q=[0, 0, 0.001, 0, 0, 0])
        with self.assertRaisesRegex(VisionError, 'moved'):
            require_stationary(before, after, 2e-4)


def observation(base_target, *, role='pick', sha='abc', tcp_offset=TCP_OFFSET):
    return Observation(role=role, target=ArucoTarget(7, 'DICT_5X5_100', 0.03).describe(),
                       camera_target=make_transform(xyz=(0, 0, 0.3)), base_flange=np.eye(4), base_camera=np.eye(4),
                       base_target=base_target, robot={'tcp_offset': tcp_offset, 'joints_rad': OBSERVE_PICK},
                       quality={}, refined=False, calibration_sha256=sha)


class GraspTests(unittest.TestCase):
    def setUp(self):
        self.calibration = SimpleNamespace(sha256='abc')
        self.marker = face_up(-0.4, -0.33, yaw_deg=20)
        self.truth_tcp = self.marker @ MARKER_TCP
        self.record = teach(observation(self.marker), tf.matrix_to_pose(self.truth_tcp), role='pick',
                            calibration=self.calibration, limits=Limits())
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / 'grasp.json'
        write_taught(self.path, self.record)
        self.model = load_taught(self.path)
        self.target = ArucoTarget(7, 'DICT_5X5_100', 0.03)

    def tearDown(self):
        self.directory.cleanup()

    def role(self, **changes):
        return Role(target={'kind': 'aruco'}, observe_joints=OBSERVE_PICK, **changes)

    def compute(self, marker, role=None, **kwargs):
        options = dict(calibration_sha256='abc', tcp_offset=TCP_OFFSET, target=self.target)
        options.update(kwargs)
        return target_pose(self.model, observation(marker), role or self.role(), **options)

    def test_taught_relationship_follows_marker(self):
        moved = face_up(-0.39, -0.34, yaw_deg=28)
        pose, report = self.compute(moved)
        np.testing.assert_allclose(pose, moved @ MARKER_TCP, atol=1e-12)
        self.assertAlmostEqual(report['marker_yaw_change_deg'], 8.0, places=6)

    def test_offset_is_applied_in_marker_frame(self):
        moved = face_up(-0.39, -0.34, yaw_deg=90)
        pose, _ = self.compute(moved, self.role(offset_mm=[10, 0, 0],
                                                  limits={'max_shift_mm': 200, 'max_rotation_deg': 90}))
        expected = moved @ tf.translation([0.01, 0, 0]) @ MARKER_TCP
        np.testing.assert_allclose(pose, expected, atol=1e-12)
        # Marker X points along base +Y at yaw 90 deg.
        np.testing.assert_allclose(pose[:3, 3] - (moved @ MARKER_TCP)[:3, 3], [0, 0.01, 0], atol=1e-12)

    def test_shift_beyond_limit_refused(self):
        far = face_up(-0.36, -0.33, yaw_deg=20)
        with self.assertRaises(TargetValidationError) as caught:
            self.compute(far)
        self.assertIn('from the taught pose', str(caught.exception))
        pose, _ = self.compute(far, self.role(limits={'max_shift_mm': 60}))
        self.assertTrue(np.isfinite(pose).all())

    def test_tilted_marker_and_tool_refused(self):
        tilted = self.marker @ make_transform((math.radians(25), 0, 0))
        with self.assertRaises(TargetValidationError) as caught:
            self.compute(tilted, self.role(limits={'max_rotation_deg': 90, 'max_shift_mm': 200}))
        message = str(caught.exception)
        self.assertIn('marker tilt changed', message)
        self.assertIn('tool Z axis', message)

    def test_planar_model_keeps_taught_height_and_tilt(self):
        noisy = face_up(-0.39, -0.335, z=0.083, yaw_deg=25) @ make_transform((0.02, -0.015, 0))
        pose, report = self.compute(noisy, self.role(motion_model='planar'))
        expected = face_up(-0.39, -0.335, z=0.08, yaw_deg=25) @ MARKER_TCP
        np.testing.assert_allclose(pose[:3, 3], expected[:3, 3], atol=2e-4)
        self.assertLess(tf.rotation_angle(pose, expected), 1e-3)
        self.assertAlmostEqual(report['marker_height_change_mm'], 3.0, places=6)
        with self.assertRaisesRegex(TargetValidationError, 'height changed'):
            self.compute(face_up(-0.39, -0.335, z=0.1, yaw_deg=25), self.role(motion_model='planar'))

    def test_tcp_calibration_and_target_mismatches_refused(self):
        moved = face_up(-0.395, -0.335, yaw_deg=22)
        with self.assertRaisesRegex(TargetValidationError, 'active TCP'):
            self.compute(moved, tcp_offset=[0, 0, 0.25, 0, 0, 0])
        with self.assertRaisesRegex(TargetValidationError, 'different hand-eye calibration'):
            self.compute(moved, calibration_sha256='other')
        with self.assertRaisesRegex(TargetValidationError, 'configured target'):
            self.compute(moved, target=ArucoTarget(8, 'DICT_5X5_100', 0.03))

    def test_wrong_saved_observation_target_is_refused(self):
        observed = observation(self.marker)
        observed.target = ArucoTarget(8, 'DICT_5X5_100', 0.03).describe()
        with self.assertRaisesRegex(TargetValidationError, 'observation target'):
            target_pose(self.model, observed, self.role(), calibration_sha256='abc',
                        tcp_offset=TCP_OFFSET, target=self.target)

    def test_observation_procedure_must_match_teaching(self):
        import dataclasses
        model = dataclasses.replace(self.model, taught_refined=True)
        moved = face_up(-0.395, -0.335, yaw_deg=22)
        options = dict(calibration_sha256='abc', tcp_offset=TCP_OFFSET, target=self.target)
        with self.assertRaisesRegex(TargetValidationError, 'same observe procedure'):
            target_pose(model, observation(moved), self.role(), **options)
        _, report = target_pose(model, observation(moved), self.role(), strict=False, **options)
        self.assertFalse(report['same_procedure_as_teaching'])

    def test_teach_refuses_implausible_relationships(self):
        far_tcp = self.marker @ tf.translation([0.7, 0, 0]) @ MARKER_TCP
        with self.assertRaisesRegex(VisionError, 'from the marker'):
            teach(observation(self.marker), tf.matrix_to_pose(far_tcp), role='pick',
                  calibration=self.calibration, limits=Limits())
        with self.assertRaisesRegex(VisionError, 'different hand-eye'):
            teach(observation(self.marker, sha='old'), tf.matrix_to_pose(self.truth_tcp), role='pick',
                  calibration=self.calibration, limits=Limits())
        with self.assertRaisesRegex(VisionError, 'exists'):
            write_taught(self.path, self.record)

    def test_corrupted_taught_file_refused(self):
        raw = json.loads(self.path.read_text())
        raw['taught_tcp_in_base'][0] += 0.01
        self.path.write_text(json.dumps(raw))
        with self.assertRaisesRegex(VisionError, 'inconsistent'):
            load_taught(self.path)

    def test_configured_grasp_uses_reference_for_limits(self):
        model = configured_grasp({'marker_to_tcp': tf.matrix_to_pose(MARKER_TCP),
                                  'reference_tcp_pose': tf.matrix_to_pose(self.truth_tcp)}, 'pick')
        np.testing.assert_allclose(model.reference_marker, self.marker, atol=1e-9)
        with self.assertRaises(ValueError):
            configured_grasp({'marker_to_tcp': [0.6, 0, 0, 0, 0, 0], 'reference_tcp_pose': [0] * 6}, 'pick')


class TargetConfigTests(unittest.TestCase):
    def test_aruco_manifest_supplies_size_and_is_verified(self):
        configuration = {'dictionary': 'DICT_5X5_100', 'marker_length_mm': 30.0, 'border_bits': 1,
                         'printing': {}, 'marker_ids': list(range(1, 21))}
        manifest = {'schema_version': 1, 'kind': 'aruco', 'configuration': configuration,
                    'configuration_sha256': canonical_hash(configuration), 'marker_ids': list(range(1, 21))}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'aruco_manifest.json'
            path.write_text(json.dumps(manifest))
            target = aruco_target({'kind': 'aruco', 'marker_id': 7, 'aruco_manifest': 'aruco_manifest.json'}, directory)
            self.assertEqual((target.dictionary, target.marker_length_m), ('DICT_5X5_100', 0.03))
            with self.assertRaisesRegex(ValueError, 'not in ArUco manifest'):
                aruco_target({'kind': 'aruco', 'marker_id': 30, 'aruco_manifest': 'aruco_manifest.json'}, directory)
            with self.assertRaisesRegex(ValueError, 'contradicts'):
                aruco_target({'kind': 'aruco', 'marker_id': 7, 'aruco_manifest': 'aruco_manifest.json',
                              'marker_length_mm': 25}, directory)
            tampered = copy.deepcopy(manifest)
            tampered['configuration']['marker_length_mm'] = 25.0
            path.write_text(json.dumps(tampered))
            with self.assertRaisesRegex(VisionError, 'checksum'):
                aruco_target({'kind': 'aruco', 'marker_id': 7, 'aruco_manifest': 'aruco_manifest.json'}, directory)

    def test_strict_vision_config(self):
        from vision_sim import vision_config
        with tempfile.TemporaryDirectory() as directory:
            path = vision_config(directory)
            config = VisionConfig.load(path)
            self.assertEqual(config.base_dir, str(Path(directory).resolve()))
            raw = json.loads(path.read_text())
            for change in ({'unknown': 1}, {'pick': dict(raw['pick'], motion_model='free')},
                           {'quality': {'frames': 0}}, {'pick': dict(raw['pick'], observe_joints=[0] * 5)}):
                with self.subTest(change=change):
                    path.write_text(json.dumps(dict(raw, **change)))
                    with self.assertRaises(ValueError):
                        VisionConfig.load(path)


class PreviewTests(unittest.TestCase):
    def setUp(self):
        from vision_sim import recipe, vision_config
        from ur_pick_place.vision.planner import VisionJob
        self.directory = tempfile.TemporaryDirectory()
        path = vision_config(self.directory.name, place=False)
        self.config = VisionConfig.load(path)
        self.calibration = load_handeye(self.config.path('handeye.json'))
        marker = face_up(-0.4, -0.33, yaw_deg=20)
        record = teach(observation(marker, sha=self.calibration.sha256), tf.matrix_to_pose(marker @ MARKER_TCP),
                       role='pick', calibration=self.calibration, limits=Limits())
        write_taught(self.config.path('grasp_pick.json'), record)
        self.job = VisionJob(recipe(), self.config, self.calibration, inputs={'recipe': 'r1'})
        self.observations = {'pick': observation(face_up(-0.395, -0.335, yaw_deg=24), sha=self.calibration.sha256)}
        self.plan = self.job.compose(self.observations, TCP_OFFSET)

    def tearDown(self):
        self.directory.cleanup()

    def test_plan_feeds_existing_envelope(self):
        envelope = self.plan['envelope']
        self.assertEqual(envelope['pick'], self.plan['pick_pose'])
        self.assertAlmostEqual(envelope['pick_approach'][2], self.plan['pick_pose'][2] + 0.1)
        self.assertEqual(self.plan['place_pose'], self.job.recipe.place_pose)

    def test_preview_checks(self):
        preview = self.job.preview_record(self.plan)
        self.job.check_preview(preview)
        self.job.check_against_preview(preview, self.plan)
        with self.assertRaisesRegex(VisionError, 'old'):
            self.job.check_preview(preview, now=time.time() + 10000)
        used = dict(preview, consumed_at=time.time())
        with self.assertRaisesRegex(VisionError, 'already used'):
            self.job.check_preview(used)
        changed = dict(preview, inputs=dict(preview['inputs'], recipe='r2'))
        with self.assertRaisesRegex(VisionError, 'recipe'):
            self.job.check_preview(changed)
        moved = self.job.compose({'pick': observation(face_up(-0.39, -0.335, yaw_deg=24),
                                                      sha=self.calibration.sha256)}, TCP_OFFSET)
        with self.assertRaisesRegex(TargetValidationError, 'since the preview'):
            self.job.check_against_preview(preview, moved)

    def test_envelope_violation_refused(self):
        low = face_up(-0.395, -0.335, z=-0.0, yaw_deg=24)
        self.config.pick.limits.max_shift_mm = 200
        with self.assertRaisesRegex(TargetValidationError, 'envelope'):
            self.job.compose({'pick': observation(low, sha=self.calibration.sha256)}, TCP_OFFSET)


class ReviewRegressionTests(unittest.TestCase):
    def test_simulated_calibration_cannot_authorize_hardware(self):
        raw = calibration_dict()
        raw['camera'] = {'simulated': True}
        with self.assertRaisesRegex(CalibrationError, 'Simulated'):
            parse_handeye(raw).require_hardware()

    def test_saved_observation_requires_rigid_consistent_transforms(self):
        observed = observation(np.eye(4))
        observed.base_target = observed.base_camera @ observed.camera_target
        raw = observed.to_json()
        self.assertEqual(Observation.from_json(raw).role, 'pick')
        for key in ('target_in_base', 'target_in_camera', 'camera_in_base', 'flange_in_base'):
            bad = copy.deepcopy(raw)
            bad[key]['matrix'][3][0] = 0.1
            with self.subTest(key=key), self.assertRaisesRegex(VisionError, 'bottom row'):
                Observation.from_json(bad)
        bad = copy.deepcopy(raw)
        bad['target_in_base']['matrix'][0][3] += 0.01
        with self.assertRaisesRegex(VisionError, 'disagrees'):
            Observation.from_json(bad)
        raw['refined'] = 'false'
        with self.assertRaisesRegex(VisionError, 'flag'):
            Observation.from_json(raw)

    def test_demo_observe_pose_cannot_authorize_preview_motion(self):
        from vision_sim import ROOT, vision_config
        from ur_pick_place.poses import PoseStore
        from ur_pick_place.vision.cli import _load, _parser
        with tempfile.TemporaryDirectory() as directory:
            path = vision_config(directory)
            poses = Path(directory) / 'poses.json'
            PoseStore(poses).save('demo view', [0]*6, [0]*6, robot_ip='demo', demo=True)
            args = _parser().parse_args(['vision-pick', '--config', str(ROOT / 'examples/rg2.json'),
                        '--vision', str(path), '--pose-store', str(poses), '--observe-pose', 'demo view',
                        '--observe-motion'])
            with self.assertRaisesRegex(ValueError, 'Demo poses'):
                _load(args, None)

    def test_preview_lock_serializes_concurrent_use(self):
        from ur_pick_place.vision.cli import _preview_lock
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'preview.json'
            with _preview_lock(path):
                with self.assertRaisesRegex(ValueError, 'already in use'):
                    with _preview_lock(path):
                        self.fail('second process acquired preview')
            self.assertFalse(Path(str(path) + '.lock').exists())

    def test_camera_identity_is_checked(self):
        from ur_pick_place.vision.observe import check_camera_identity
        raw = calibration_dict()
        raw['camera'] = {'device': {'serial_number': 'expected'}}
        calibration = parse_handeye(raw)
        check_camera_identity(SimpleNamespace(serial='expected'), calibration)
        with self.assertRaisesRegex(VisionError, 'serial differs'):
            check_camera_identity(SimpleNamespace(serial='other'), calibration)

    def test_tcp_change_during_capture_is_refused(self):
        before = SimpleNamespace(q=[0]*6, tcp_offset=[0]*6)
        after = SimpleNamespace(q=[0]*6, tcp_offset=[0, 0, 0.1, 0, 0, 0])
        with self.assertRaisesRegex(VisionError, 'TCP changed'):
            require_stationary(before, after, 0.001)


if __name__ == '__main__':
    unittest.main()
