"""Validation uses simulated RTDE only; no hardware connections are opened."""
import copy
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest

from vision_sim import TCP_OFFSET, calibration_dict
from ur_pick_place.demo import DemoRobot
from ur_pick_place.vision.teach_ui import TeachingService
from ur_pick_place.vision.teaching import TeachingProject, digest, read_json


class ValidationRobot(DemoRobot):
    def __init__(self):
        super().__init__()
        self.moves = []
        self.stops = 0
        self.tcp = list(TCP_OFFSET)
        self._duration = .15

    def getTCPOffset(self):
        return self.tcp

    def moveL(self, *args):
        self.moves.append(args)
        return super().moveL(*args)

    def stopL(self, *args):
        self.stops += 1
        return super().stopL(*args)


class ValidationTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        calibration = self.root / 'calibration.json'
        calibration.write_text(json.dumps(calibration_dict()))
        self.settings = dict(project=str(self.root/'project'), calibration=str(calibration), robot_ip='fake',
            tcp=list(TCP_OFFSET), motion_model='planar', approach_mm=100,
            target=dict(kind='aruco', marker_id=7, dictionary='DICT_5X5_100', marker_length_mm=30))
        self.project = TeachingProject(self.settings)
        (self.project.directory / 'grasp_pick.json').write_text('{}')
        self.project.flow['taught'] = True
        self.robot = ValidationRobot()
        self.record = dict(id='test-capture', status='captured', timestamp=time.time(),
            capture_robot=dict(tcp_pose_m_rad=list(self.robot.pose), joints_rad=list(self.robot.joints)),
            grasp_sha256=digest(self.project.directory/'grasp_pick.json'),
            approach_tcp_pose_m_rad=[.42, -.12, .4, .1, math.pi, 0])
        self.project.validation = copy.deepcopy(self.record)
        self.service = TeachingService(self.root, motion_factory=lambda _: (self.robot, self.robot))
        self.service.project = self.project
        self.service.heartbeat()
        self.addCleanup(self.service.close)
        self.body = dict(capture_id='test-capture', path_confirmed=True, speed_mm_s=10,
                         workspace_min_m=[-.9, -.9, .02], workspace_max_m=[.9, .9, 1.2])

    def wait_done(self, heartbeat=True):
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            if heartbeat:
                self.service.heartbeat()
            if not self.service.state()['busy']:
                return
            time.sleep(.01)
        self.fail('Validation worker did not finish')

    def test_motion_is_single_use_and_records_measured_error_and_visual_review(self):
        self.service.move_validation(self.body)
        self.wait_done()
        self.assertEqual(self.service.error, '')
        result = self.project.validation
        self.assertEqual(result['status'], 'completed')
        self.assertLess(result['error']['position_mm'], 1)
        self.assertLess(result['error']['rotation_deg'], .287)
        for actual, expected in zip(self.robot.moves[0][0], self.record['approach_tcp_pose_m_rad']):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(self.robot.moves[0][1:3], (.01, .05))
        self.assertFalse(self.robot.connected)
        with self.assertRaisesRegex(ValueError, 'fresh validation'):
            self.service.move_validation(self.body)
        self.service.review_validation(dict(capture_id=self.record['id'], result='aligned'))
        saved = read_json(self.project.directory/'validation_history.json')
        self.assertEqual(len(saved), 1)
        self.assertEqual(saved[0]['visual_result'], 'aligned')

    def test_expired_changed_or_unreviewed_capture_never_connects(self):
        cases = [dict(path_confirmed=False), dict(capture_id='old'), dict(speed_mm_s=21),
                 dict(workspace_min_m=[1, 2]), dict(workspace_min_m=[1, 2, 3])]
        for values in cases:
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.service.move_validation(dict(self.body, **values))
        self.project.validation['timestamp'] -= 61
        with self.assertRaisesRegex(ValueError, 'expired'):
            self.service.move_validation(self.body)
        self.project.validation = copy.deepcopy(self.record)
        (self.project.directory/'grasp_pick.json').write_text('{"changed":true}')
        with self.assertRaisesRegex(ValueError, 'relationship changed'):
            self.service.move_validation(self.body)
        self.assertIsNone(self.service.motion_thread)
        self.assertFalse(self.robot.moves)

    def test_wrong_tcp_robot_shift_and_workspace_refuse_motion(self):
        for problem in ('tcp', 'joints', 'workspace', 'ik', 'safety', 'calibration'):
            with self.subTest(problem=problem):
                self.robot = ValidationRobot()
                self.project.validation = copy.deepcopy(self.record)
                body = dict(self.body)
                if problem == 'tcp':
                    self.robot.tcp = [0]*6
                elif problem == 'joints':
                    self.robot.joints[0] += .1
                elif problem == 'workspace':
                    body['workspace_max_m'] = [.41, .9, 1.2]
                elif problem == 'ik':
                    self.robot.getInverseKinematicsHasSolution = lambda *args: False
                elif problem == 'safety':
                    self.robot.isPoseWithinSafetyLimits = lambda *args: False
                elif problem == 'calibration':
                    Path(self.settings['calibration']).write_text('{}')
                    with self.assertRaisesRegex(ValueError, 'Calibration changed'):
                        self.service.move_validation(body)
                    continue
                self.service.heartbeat()
                self.service.move_validation(body)
                self.wait_done()
                self.assertTrue(self.service.error)
                self.assertFalse(self.robot.moves)
                self.assertEqual(self.project.validation['status'], 'failed')

    def test_stop_cancels_connection_before_move_and_requires_recapture(self):
        connecting, release = threading.Event(), threading.Event()
        def factory(_):
            connecting.set()
            release.wait(2)
            return self.robot, self.robot
        self.service.motion_factory = factory
        self.service.move_validation(self.body)
        self.assertTrue(connecting.wait(1))
        self.service.stop()
        release.set()
        self.wait_done()
        self.assertFalse(self.robot.moves)
        self.assertEqual(self.project.validation['status'], 'stopped')

    def test_stop_during_robot_preflight_never_starts_a_move(self):
        checking, release = threading.Event(), threading.Event()

        def safety_check(_):
            checking.set()
            release.wait(2)
            return True

        self.robot.isPoseWithinSafetyLimits = safety_check
        self.service.move_validation(self.body)
        self.assertTrue(checking.wait(1))
        self.service.stop()
        release.set()
        self.wait_done()
        self.assertFalse(self.robot.moves)
        self.assertEqual(self.project.validation['status'], 'stopped')

    def test_robot_change_during_preflight_never_starts_a_move(self):
        for problem in ('safety', 'joints', 'tcp'):
            with self.subTest(problem=problem):
                checking, release = threading.Event(), threading.Event()
                self.robot = ValidationRobot()
                self.project.validation = copy.deepcopy(self.record)

                def safety_check(_):
                    checking.set()
                    release.wait(2)
                    return True

                self.robot.isPoseWithinSafetyLimits = safety_check
                self.service.heartbeat()
                self.service.move_validation(self.body)
                self.assertTrue(checking.wait(1))
                if problem == 'safety':
                    self.robot.getSafetyMode = lambda: 7
                elif problem == 'joints':
                    self.robot.joints[0] += .01
                else:
                    self.robot.tcp = [0]*6
                release.set()
                self.wait_done()
                self.assertFalse(self.robot.moves)
                self.assertEqual(self.project.validation['status'], 'failed')

    def test_heartbeat_loss_and_explicit_stop_interrupt_motion(self):
        for reason in ('stop', 'heartbeat'):
            with self.subTest(reason=reason):
                self.robot = ValidationRobot()
                self.robot._duration = 10
                self.project.validation = copy.deepcopy(self.record)
                self.service.heartbeat()
                self.service.move_validation(self.body)
                deadline = time.monotonic()+2
                while not self.robot.moves and time.monotonic() < deadline:
                    self.service.heartbeat()
                    time.sleep(.01)
                self.assertTrue(self.robot.moves)
                if reason == 'stop':
                    self.service.stop()
                else:
                    self.service.heartbeat_at = time.monotonic()-3
                self.wait_done(heartbeat=False)
                self.assertGreater(self.robot.stops, 0)
                self.assertNotEqual(self.project.validation['status'], 'completed')
                self.assertFalse(self.robot.connected)

    def test_inaccurate_completion_fails_and_cannot_be_marked_aligned(self):
        def wrong_move(pose, *args):
            self.robot.moves.append((pose, *args))
            target = list(pose)
            target[0] += .01
            return DemoRobot.moveL(self.robot, target, *args)
        self.robot.moveL = wrong_move
        self.service.move_validation(self.body)
        self.wait_done()
        self.assertEqual(self.project.validation['status'], 'failed')
        self.assertGreater(self.project.validation['error']['position_mm'], 9)
        with self.assertRaises(ValueError):
            self.service.review_validation(dict(capture_id=self.record['id'], result='aligned'))

    def test_tcp_change_or_rejected_stop_during_motion_is_reported(self):
        for problem in ('tcp', 'stop'):
            with self.subTest(problem=problem):
                self.robot = ValidationRobot()
                self.robot._duration = 10
                self.project.validation = copy.deepcopy(self.record)
                self.service.heartbeat()
                self.service.move_validation(self.body)
                deadline = time.monotonic()+2
                while not self.robot.moves and time.monotonic() < deadline:
                    self.service.heartbeat()
                    time.sleep(.01)
                self.assertTrue(self.robot.moves)
                if problem == 'tcp':
                    self.robot.tcp = [0]*6
                else:
                    self.robot.stopL = lambda *args: False
                    self.service.stop()
                self.wait_done()
                self.assertEqual(self.project.validation['status'], 'failed')
                self.assertIn('TCP' if problem == 'tcp' else 'rejected the validation stop', self.service.error)
                self.assertFalse(self.robot.connected)


if __name__ == '__main__':
    unittest.main()
