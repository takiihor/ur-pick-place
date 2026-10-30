"""Read-only teaching with rendered markers, TCP feedback and the real vision pipeline."""
import json
from pathlib import Path
import tempfile
import subprocess
import threading
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from vision_sim import (CAMERA_IN_FLANGE, MARKER_TCP, OBSERVE_PICK, TCP_OFFSET, calibration_dict, matrix_to_pose)
import numpy as np
from test_vision_pipeline import FakeRuntime, World
from test_vision_detection import HAVE_CHARUCO
from ur_pick_place.vision.grasp import load_taught
from ur_pick_place.vision.errors import VisionError
from ur_pick_place.vision.teaching import TeachingProject, defaults, read_json
from ur_pick_place.vision.teach_ui import TeachingService, TeachingServer
from ur_pick_place.vision.transforms import pose_to_matrix


class MarkerTeachingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.world = World()
        self.world.rtde.q = list(OBSERVE_PICK)
        from vision_sim import true_flange
        self.world.rtde.flange = true_flange(OBSERVE_PICK)
        self.runtime = FakeRuntime(self.world.rtde, self.world.camera)
        calibration = self.root / 'calibration.json'
        calibration.write_text(json.dumps(calibration_dict()))
        self.settings = {'project': str(self.root / 'project'), 'calibration': str(calibration),
                         'robot_ip': 'fake-robot', 'tcp': list(TCP_OFFSET),
                         'target': {'kind': 'aruco', 'marker_id': 7, 'dictionary': 'DICT_5X5_100', 'marker_length_mm': 30},
                         'motion_model': 'planar', 'approach_mm': 100}
        self.project = TeachingProject(self.settings, runtime=self.runtime)
        self.project.config.robot.settle_s = 0
        self.project.config.quality.frames = 3
        self.project.config.quality.max_attempt_frames = 8
        self.taught_pose = None

    def capture_and_teach(self):
        self.project.capture()
        # Pendant teaching: change the feedback flange so its TCP is at the grasp.
        self.taught_pose = self.world.pick @ MARKER_TCP
        from ur_pick_place.vision.transforms import invert
        self.world.rtde.flange = self.taught_pose @ invert(pose_to_matrix(TCP_OFFSET))
        # Model a physically consistent set of feedback through a patched nominal FK.
        with patch('ur_pick_place.vision.robot_state.nominal_flange', return_value=self.world.rtde.flange):
            self.project.save_pick(marker_unchanged=True)
        from vision_sim import true_flange
        self.world.rtde.flange = true_flange(OBSERVE_PICK)

    def test_capture_teach_move_marker_export_and_reopen(self):
        self.capture_and_teach()
        self.world.move_pick(10, -8, 4)
        exported = self.project.locate()
        expected = self.world.pick @ MARKER_TCP
        measured = pose_to_matrix(exported['pick_tcp_pose_m_rad'])
        self.assertLess(np.linalg.norm(measured[:3, 3] - expected[:3, 3]), 0.001)
        pick = np.array(exported['pick_tcp_pose_m_rad'])
        approach = np.array(exported['approach_tcp_pose_m_rad'])
        np.testing.assert_allclose(approach - pick, [0, 0, .1, 0, 0, 0], atol=1e-12)
        self.assertEqual(read_json(self.project.directory / 'target.json'), pick.tolist())
        self.assertTrue((self.project.directory / 'capture.jpg').exists())
        script = (self.project.directory / 'target.script').read_text()
        self.assertIn('global vision_pick = p[', script)
        self.assertNotIn('movel(', script)
        self.assertTrue(self.project.flow['exported'])
        # Files use the existing runner's schemas, not a second grasp/config format.
        from ur_pick_place.vision.config import VisionConfig
        config = VisionConfig.load(self.project.directory / 'vision.json')
        self.assertEqual(config.pick.grasp, 'grasp_pick.json')
        self.assertEqual(config.pick.observe_joints, list(OBSERVE_PICK))
        load_taught(self.project.directory / 'grasp_pick.json')
        reopened = TeachingProject(self.settings, runtime=self.runtime)
        self.assertTrue(reopened.flow['taught'])
        self.assertFalse(reopened.flow['exported'])
        self.assertEqual(set(self.runtime.calls), {'receive', 'camera'})
        self.assertFalse(self.world.rtde.events)  # no control or force mode was called

    def test_failed_new_read_removes_old_target_files(self):
        self.capture_and_teach()
        self.project.locate()
        self.world.move_pick(100, 0, 0)
        with self.assertRaisesRegex(VisionError, 'target refused'):
            self.project.locate()
        self.assertFalse(self.project.flow['exported'])
        for name in ('target.json', 'pick_target.json', 'target.script'):
            self.assertFalse((self.project.directory / name).exists())
        self.assertFalse(self.world.rtde.connected)

    def test_validation_rotation_preserves_teaching_and_exports(self):
        self.capture_and_teach()
        self.project.locate()
        reference = (self.project.directory / 'observation.json').read_bytes()
        grasp = (self.project.directory / 'grasp_pick.json').read_bytes()
        exported = (self.project.directory / 'target.json').read_bytes()
        self.world.move_pick(10, -8, 5)
        record = self.project.capture_validation()
        expected = self.world.pick @ MARKER_TCP
        np.testing.assert_allclose(pose_to_matrix(record['pick_tcp_pose_m_rad']), expected, atol=.002)
        self.assertAlmostEqual(record['validation']['marker_yaw_change_deg'], 5, delta=.3)
        self.assertEqual((self.project.directory / 'observation.json').read_bytes(), reference)
        self.assertEqual((self.project.directory / 'grasp_pick.json').read_bytes(), grasp)
        self.assertEqual((self.project.directory / 'target.json').read_bytes(), exported)
        self.assertTrue(self.project.flow['exported'])
        self.assertTrue((self.project.directory / 'validation.jpg').exists())
        self.project.record_validation(dict(record, status='completed', visual_result='aligned'))
        reopened = TeachingProject(self.settings, runtime=self.runtime)
        self.assertIsNone(reopened.validation)
        self.assertEqual(reopened.validation_history[0]['visual_result'], 'aligned')
        self.world.move_pick(100, 0, 0)
        with self.assertRaises(VisionError):
            self.project.capture_validation()
        self.assertIsNone(self.project.validation)
        self.assertFalse((self.project.directory / 'validation_capture.json').exists())
        self.assertEqual((self.project.directory / 'target.json').read_bytes(), exported)

    def test_capture_again_invalidates_taught_offset(self):
        self.capture_and_teach()
        self.project.capture()
        self.assertFalse(self.project.flow['taught'])
        self.assertFalse((self.project.directory / 'grasp_pick.json').exists())
        self.assertEqual(len(list((self.project.directory / 'history').glob('*-grasp_pick.json'))), 1)
        with self.assertRaisesRegex(ValueError, 'Save a picking position'):
            self.project.locate()

    def test_confirmations_and_changed_inputs(self):
        with self.assertRaisesRegex(ValueError, 'Capture the marker'):
            self.project.save_pick(marker_unchanged=True)
        self.project.capture()
        with self.assertRaisesRegex(ValueError, 'stayed fixed'):
            self.project.save_pick()
        with self.assertRaisesRegex(ValueError, 'different settings'):
            TeachingProject(dict(self.settings, approach_mm=120))
        path = Path(self.settings['calibration'])
        path.write_text(path.read_text()+'\n')
        with self.assertRaisesRegex(ValueError, 'Calibration changed'):
            self.project.capture()

    def test_different_observation_joints_refused(self):
        self.capture_and_teach()
        # Same image/pose feedback but different joint history: strict viewpoint gate.
        self.world.rtde.q[0] += .02
        with patch('ur_pick_place.vision.robot_state.nominal_flange', return_value=self.world.rtde.flange):
            with self.assertRaisesRegex(VisionError, 'different joints'):
                self.project.locate()

    def test_simulation_and_wrong_tcp_refused_before_hardware(self):
        raw = calibration_dict()
        raw['simulation_truth'] = {'demo': True}
        path = self.root / 'sim.json'
        path.write_text(json.dumps(raw))
        with self.assertRaisesRegex(VisionError, 'Simulated calibration'):
            TeachingProject(dict(self.settings, calibration=str(path), project=str(self.root / 'sim-project')))
        with self.assertRaisesRegex(ValueError, 'active gripper TCP') as caught:
            TeachingProject(dict(self.settings, tcp=[0]*6, project=str(self.root / 'other-project')))
        self.assertIn('Entered TCP (XYZ mm / rotation-vector rad): 0, 0, 0, 0, 0, 0', str(caught.exception))
        self.assertIn('Calibration TCP (XYZ mm / rotation-vector rad): 0, 0, 200, 0, 0, 0', str(caught.exception))
        self.assertIn(self.settings['calibration'], str(caught.exception))
        self.assertFalse(self.runtime.calls)

    def test_validation_worker_preserves_exports_on_success_and_failure(self):
        self.capture_and_teach()
        exported = self.project.locate()
        record = self.project.capture_validation()
        service = TeachingService(self.root)
        service.project = self.project
        self.addCleanup(service.close)
        success = Mock(returncode=0)
        success.communicate.return_value = ('', '')
        with patch('ur_pick_place.vision.teach_ui.subprocess.Popen', return_value=success):
            service._run('capture_validation', self.settings)
        self.assertEqual(service.project.validation['id'], record['id'])
        self.assertEqual(service.project.exported, exported)
        self.assertTrue(service.project.flow['exported'])
        failed = Mock(returncode=1)
        failed.communicate.return_value = ('', 'Marker detection failed')
        with patch('ur_pick_place.vision.teach_ui.subprocess.Popen', return_value=failed):
            service._run('capture_validation', self.settings)
        self.assertIsNone(service.project.validation)
        self.assertEqual(service.project.exported, exported)
        self.assertTrue(service.project.flow['taught'])

    def test_defaults_prefer_local_tcp_copy_and_skip_invalid_calibrations(self):
        repo = self.root / 'pick-place'
        original = self.root / 'ur10e-handeye' / 'sessions' / 'original' / 'calibration.json'
        local = repo / 'data' / 'calibrations' / 'rg2' / 'calibration.json'
        invalid = repo / 'data' / 'calibrations' / 'zzz-invalid' / 'calibration.json'
        simulated = repo / 'data' / 'calibrations' / 'zzz-simulation' / 'calibration.json'
        for path in (original, local, invalid, simulated):
            path.parent.mkdir(parents=True, exist_ok=True)
        original.write_text(json.dumps(calibration_dict(tcp_offset=[0]*6)))
        tcp = [0, 0, .2286, 0, 0, 0]
        local.write_text(json.dumps(calibration_dict(tcp_offset=tcp)))
        invalid.write_text('{}')
        demo = calibration_dict()
        demo['simulation_truth'] = {'demo': True}
        simulated.write_text(json.dumps(demo))
        discovered = defaults(repo)
        self.assertEqual([c['path'] for c in discovered['calibrations']], [str(local), str(original)])
        self.assertEqual(discovered['settings']['calibration'], str(local))
        self.assertEqual(discovered['settings']['tcp'], tcp)
        self.assertFalse(self.runtime.calls)

    @unittest.skipUnless(HAVE_CHARUCO, 'charuco-generator/charuco-realsense not installed')
    def test_charuco_capture_uses_same_teaching_pipeline(self):
        from vision_sim import FakeCamera, camera_view_point, face_up
        from ur_pick_place.vision.targets import target_from_config
        from test_vision_detection import charuco_target
        target = charuco_target(self.root)
        spec = {'kind': 'charuco', 'board_manifest': str(self.root / 'board_manifest.json'), 'board': 'front'}
        point = camera_view_point(OBSERVE_PICK)
        base_target = face_up(point[0], point[1])
        base_target[:3, 3] -= base_target[:3, :3] @ target.centre
        self.world.camera = FakeCamera(self.world.rtde, [(target, base_target)])
        runtime = FakeRuntime(self.world.rtde, self.world.camera)
        project = TeachingProject(dict(self.settings, target=spec, project=str(self.root/'charuco')), runtime=runtime)
        project.config.robot.settle_s = 0
        project.config.quality.frames = 3
        summary = project.capture()
        self.assertEqual(summary['target']['kind'], 'charuco')
        self.assertGreaterEqual(summary['quality']['corner_count'], 6)
        from ur_pick_place.vision.transforms import invert
        tcp = base_target @ MARKER_TCP
        self.world.rtde.flange = tcp @ invert(pose_to_matrix(TCP_OFFSET))
        with patch('ur_pick_place.vision.robot_state.nominal_flange', return_value=self.world.rtde.flange):
            project.save_pick(marker_unchanged=True)
        from vision_sim import true_flange
        self.world.rtde.flange = true_flange(OBSERVE_PICK)
        self.assertEqual(project.locate()['target']['kind'], 'charuco')

    def test_80_mm_marker_capture_and_export(self):
        from ur_pick_place.vision.targets import ArucoTarget
        target = ArucoTarget(7, 'DICT_5X5_100', .080)
        self.world.camera.scene = [(target, self.world.pick)]
        # Controlled exposure for this size/format test; noisy frames are covered by detection tests.
        self.world.camera.noise = 0.2
        settings = dict(self.settings, project=str(self.root / 'marker80'),
                        target=dict(self.settings['target'], marker_length_mm=80))
        self.project = TeachingProject(settings, runtime=self.runtime)
        self.project.config.robot.settle_s = 0
        self.project.config.quality.frames = 3
        self.capture_and_teach()
        result = self.project.locate()
        self.assertEqual(result['target']['marker_length_mm'], 80)
        self.assertGreater(result['quality']['pixels_per_module'], 5)

    def test_worker_launch_failure_and_timeout(self):
        service = TeachingService(self.root)
        service.configure(dict(self.settings, tcp_confirmed=True))
        fake = Mock(returncode=1)
        fake.communicate.return_value = ('', 'Expected fake camera failure')
        with patch('ur_pick_place.vision.teach_ui.subprocess.Popen', return_value=fake) as factory:
            service._run('capture', self.settings)
            self.assertEqual(service.state()['error'], 'Expected fake camera failure')
            self.assertEqual(factory.call_args.args[0][2], 'ur_pick_place.vision.teach_ui')
        fake = Mock()
        fake.communicate.side_effect = [subprocess.TimeoutExpired('worker', 90), ('', '')]
        with patch('ur_pick_place.vision.teach_ui.subprocess.Popen', return_value=fake):
            service._run('capture', self.settings)
            self.assertIn('timed out', service.state()['error'])
            fake.kill.assert_called_once()
            self.assertIsNone(service.state()['busy'])

    def test_http_setup_assets_and_authorization_without_connections(self):
        service = TeachingService(self.root)
        server = TeachingServer(('127.0.0.1', 0), service)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        self.addCleanup(service.close)
        url = f'http://127.0.0.1:{server.server_address[1]}'
        with urlopen(url) as r:
            page = r.read().decode()
            self.assertIn('Teach the picking position', page)
            self.assertIn('aria-label="Teaching steps"', page)
            self.assertIn(server.token, page)
        req = Request(url+'/api/configure', data=json.dumps(dict(self.settings, tcp_confirmed=True)).encode(),
                      headers={'Content-Type':'application/json'}, method='POST')
        with self.assertRaises(HTTPError) as caught:
            urlopen(req)
        self.assertEqual(caught.exception.code, 403)
        req.add_header('X-Teach-Token', server.token)
        with urlopen(req) as r:
            self.assertTrue(json.load(r)['project'])
        for endpoint in ('/api/move', '/api/force', '/api/connect'):
            req = Request(url+endpoint, data=b'{}', headers={'Content-Type':'application/json', 'X-Teach-Token':server.token})
            with self.assertRaises(HTTPError) as caught:
                urlopen(req)
            self.assertEqual(caught.exception.code, 404)
        # No generated target downloads are available before a successful fresh read.
        with self.assertRaises(HTTPError) as caught:
            urlopen(url+'/api/download/target.json')
        self.assertEqual(caught.exception.code, 409)
        self.assertFalse(self.runtime.calls)


if __name__ == '__main__':
    unittest.main()
