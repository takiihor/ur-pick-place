from dataclasses import asdict
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from ur_pick_place.config import Gripper, Motion
from ur_pick_place.demo import DemoRobot
from ur_pick_place.manual import ManualService, ManualSettings
from ur_pick_place.poses import PoseStore
from ur_pick_place.sequence_ui import DemoGripper, initial_recipe
from test_manual import wait_for


class SequenceUITests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.store = PoseStore(Path(self.directory.name)/'poses.json')
        settings = ManualSettings()
        config = initial_recipe(settings)
        config.motion = Motion(settle_s=0)
        config.gripper = Gripper(settle_s=0)
        self.robot = DemoRobot()
        self.robot._duration = .03
        self.service = ManualService(settings, self.store, demo=True, recipe=config,
                                     factory=lambda host: (self.robot, self.robot),
                                     sequence_path=Path(self.directory.name)/'sequence.json')
        self.addCleanup(self.service.close)
        self.payload = self.service.seed_demo_poses()
        self.service.submit('connect')
        wait_for(lambda: self.service.state()['connected'] and not self.service.state()['pending'])
        self.heartbeat_stop = threading.Event()
        def heartbeat():
            while not self.heartbeat_stop.wait(.1):
                self.service.heartbeat()
        self.heartbeat = threading.Thread(target=heartbeat, daemon=True)
        self.heartbeat.start()
        self.addCleanup(self.stop_heartbeat)

    def stop_heartbeat(self):
        self.heartbeat_stop.set()
        self.heartbeat.join(timeout=1)

    def preview(self, options=None):
        return self.service.preview_sequence(dict(self.payload, options=options or {}))

    def run_preview(self, preview):
        self.service.submit('run_sequence', {'preview_id':preview['id']})

    def test_preview_makes_no_motion_and_cannot_run_without_preview(self):
        preview = self.preview()
        self.assertEqual(preview['gripper'], 'rg2')
        self.assertFalse(self.robot._force_active)
        self.assertIsNone(self.robot._move)
        with self.assertRaises(ValueError):
            self.service.submit('run_sequence', {'preview_id':'wrong'})
        self.assertTrue((Path(self.directory.name)/'sequence.json').exists())

    def test_one_cycle_reuses_connection_and_reports_all_steps(self):
        self.run_preview(self.preview())
        wait_for(lambda: not self.service.state()['pending'], timeout=8)
        seq = self.service.state()['sequence']
        self.assertEqual(seq['status'], 'completed', self.service.state())
        self.assertEqual(list(dict.fromkeys(item['step'] for item in seq['history'])),
                         ['home', 'approach_pick', 'force_pick', 'grasp', 'lift_home', 'place'])
        self.assertTrue(self.service.state()['connected'])
        self.assertFalse(self.robot._force_active)
        self.assertFalse(self.robot.gripper_holding)
        self.assertIsNone(seq['preview'])
        with self.assertRaises(ValueError):
            self.run_preview({'id':'consumed'})

    def test_stop_during_force_search_blocks_grip_and_lift(self):
        self.run_preview(self.preview())
        wait_for(lambda: self.robot._force_active, timeout=5)
        self.service.stop()
        wait_for(lambda: not self.service.state()['pending'])
        seq = self.service.state()['sequence']
        self.assertEqual(seq['status'], 'stopped')
        self.assertFalse(self.robot._force_active)
        self.assertFalse(self.robot.gripper_holding)
        self.assertFalse(any(item['step'] == 'lift_home' for item in seq['history']))
        self.assertTrue(self.service.state()['connected'])

    def test_stop_during_joint_move_stops_before_open_or_probe(self):
        self.robot._duration = 1
        self.run_preview(self.preview())
        wait_for(lambda: self.robot._move is not None)
        with self.assertRaises(ValueError):
            self.service.submit('jog', {'axis':'x', 'step':1})
        self.service.stop()
        wait_for(lambda: not self.service.state()['pending'])
        self.assertIsNone(self.robot._move)
        self.assertFalse(self.robot._force_active)
        self.assertEqual(self.service.state()['sequence']['status'], 'stopped')

    def test_stop_after_grip_does_not_automatically_release(self):
        self.run_preview(self.preview({'gripper':{'kind':'rg2', 'settle_s':.3}}))
        wait_for(lambda: getattr(self.robot, 'gripper_holding', False), timeout=5)
        self.service.stop()
        wait_for(lambda: not self.service.state()['pending'])
        self.assertTrue(self.robot.gripper_holding)
        self.assertEqual(self.service.state()['sequence']['status'], 'stopped')
        self.assertFalse(any(item['step'] == 'place' for item in self.service.state()['sequence']['history']))

    def test_force_stop_failure_blocks_grasp_and_disconnects(self):
        with patch.object(self.robot, 'forceModeStop', return_value=False):
            self.run_preview(self.preview())
            wait_for(lambda: not self.service.state()['pending'], timeout=5)
        state = self.service.state()
        self.assertEqual(state['sequence']['status'], 'failed')
        self.assertFalse(state['connected'])
        self.assertFalse(any(item['step'] == 'grasp' for item in state['sequence']['history']))

    def test_arm_stop_failure_disconnects_and_blocks_subsequent_moves(self):
        self.robot._duration = 1
        with patch.object(self.robot, 'stopJ', side_effect=RuntimeError('stop rejected')):
            self.run_preview(self.preview())
            wait_for(lambda: self.robot._move is not None)
            self.service.stop()
            wait_for(lambda: not self.service.state()['pending'])
        self.assertFalse(self.service.state()['connected'])
        self.assertEqual(self.service.state()['sequence']['status'], 'failed')
        with self.assertRaises(ValueError):
            self.service.submit('jog', {'axis':'x', 'step':1})

    def test_heartbeat_loss_stops_force_probe(self):
        self.run_preview(self.preview())
        wait_for(lambda: self.robot._force_active, timeout=5)
        self.stop_heartbeat()
        self.service._heartbeat_at = time.monotonic()-3
        wait_for(lambda: not self.service.state()['pending'])
        self.assertFalse(self.robot._force_active)
        self.assertIn('heartbeat', self.service.state()['sequence']['message'])

    def test_rg2_and_arduino_are_both_selectable(self):
        self.assertEqual(self.preview()['gripper'], 'rg2')
        arduino = {'kind':'arduino', 'serial_port':'fake', 'grip_min_raw':[200, 200],
                   'release_max_raw':[20, 20], 'settle_s':0}
        preview = self.preview({'gripper':arduino})
        self.assertEqual(preview['gripper'], 'arduino')
        self.run_preview(preview)
        wait_for(lambda: not self.service.state()['pending'], timeout=8)
        self.assertEqual(self.service.state()['sequence']['status'], 'completed')
        self.assertEqual(self.preview()['gripper'], 'rg2')

    def test_arduino_requires_calibration_and_camera_target_is_validated(self):
        with self.assertRaises(ValueError):
            self.preview({'gripper':{'kind':'arduino', 'serial_port':'fake'}})
        outside = dict(self.payload, camera_pose=[2, 0, .3, 0, 3.14, 0])
        with self.assertRaises(Exception):
            self.service.preview_sequence(outside)
        target = [.42, -.12, .27, 0, 3.141592653589793, 0]
        preview = self.service.preview_sequence(dict(self.payload, pick_id='', camera_pose=target))
        self.assertEqual(preview['plan']['pick'], target)
        self.assertEqual(preview['names']['pick'], 'Camera TCP target')

    def test_manual_move_invalidates_preview(self):
        preview = self.preview()
        self.service.submit('jog', {'axis':'x', 'step':1})
        wait_for(lambda: not self.service.state()['pending'] and not self.service.state()['moving'])
        with self.assertRaises(ValueError):
            self.run_preview(preview)

    def test_no_second_connection_and_live_driver_selected_by_kind(self):
        # Keep fake RTDE interfaces but exercise live driver selection. All
        # gripper traffic is replaced by DemoGripper before any command.
        for kind in ('rg2', 'arduino'):
            options = {'kind':kind, 'settle_s':0}
            if kind == 'arduino':
                options.update(serial_port='fake', grip_min_raw=[200, 200], release_max_raw=[20, 20])
            with patch('ur_pick_place.grippers.adapters.connect_gripper') as factory:
                factory.side_effect = lambda config, host: DemoGripper(config, lambda: None, lambda s: None)
                self.service.demo = False
                for pose in self.store._poses:
                    pose['demo'] = False
                preview = self.preview({'gripper':options})
                self.run_preview(preview)
                wait_for(lambda: not self.service.state()['pending'], timeout=8)
                self.assertEqual(self.service.state()['sequence']['status'], 'completed', self.service.state())
                self.assertEqual(factory.call_args.args[0].kind, kind)
                factory.assert_called_once()
                self.assertIs(self.service.control, self.robot)
