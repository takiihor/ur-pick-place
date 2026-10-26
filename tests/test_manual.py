from contextlib import redirect_stdout
from dataclasses import asdict
import io
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from ur_pick_place.cli import main as cli
from ur_pick_place.demo import DemoRobot
from ur_pick_place.geometry import rotate_base
from ur_pick_place.manual import ManualService, ManualSettings
from ur_pick_place.poses import PoseStore
from ur_pick_place.robot import rotation_distance

ROOT = Path(__file__).resolve().parents[1]


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('Timed out waiting for backend')


class PoseTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'poses.json'
        self.store = PoseStore(self.path)

    def save(self, name='home'):
        return self.store.save(name, [.4, -.1, .25, 0, math.pi, 0], [0]*6, robot_ip='demo', demo=True)

    def test_record_rename_reload_preserves_both_coordinate_sets(self):
        pose = self.save()
        self.store.rename(pose['id'], 'home_a')
        reloaded = PoseStore(self.path).by_name('home_a')
        self.assertEqual(reloaded['tcp_pose_m_rad'], pose['tcp_pose_m_rad'])
        self.assertEqual(reloaded['joints_rad'], [0]*6)
        self.assertEqual(reloaded['id'], pose['id'])
        with self.assertRaises(ValueError):
            self.store.by_name('home')

    def test_duplicate_and_invalid_names_do_not_change_storage(self):
        pose = self.save()
        before = self.path.read_bytes()
        for name in (' HOME ', '', ' ' * 5, 'x'*81, None):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.save(name)
        self.assertEqual(self.path.read_bytes(), before)
        second = self.save('pick')
        with self.assertRaises(ValueError):
            self.store.rename(second['id'], 'Home')
        self.assertEqual(self.store.get(pose['id'])['name'], 'home')

    def test_failed_disk_write_does_not_mutate_memory_or_file(self):
        pose = self.save()
        before = self.path.read_bytes()
        with patch('ur_pick_place.poses.os.replace', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                self.store.rename(pose['id'], 'new')
        self.assertEqual(self.store.get(pose['id'])['name'], 'home')
        self.assertEqual(self.path.read_bytes(), before)

    def test_ui_poses_can_override_recipe_and_demo_cannot_execute(self):
        self.save()
        args = ['--config', str(ROOT/'examples/rg2.json'), '--pose-store', str(self.path), '--home-pose', 'home']
        with redirect_stdout(io.StringIO()):
            self.assertEqual(cli(args), 0)
            self.assertEqual(cli(args+['--execute']), 1)


class GeometryTests(unittest.TestCase):
    def test_rotation_is_composed_about_base_axis(self):
        pose = [.4, -.1, .35, 0, math.pi, 0]
        changed = rotate_base(pose, 0, 5)
        self.assertEqual(changed[:3], pose[:3])
        self.assertAlmostEqual(rotation_distance(pose[3:], changed[3:]), math.radians(5))
        # Adding 5 degrees to Rx would give a different rotation at Ry=pi.
        self.assertGreater(rotation_distance(changed[3:], [math.radians(5), math.pi, 0]), .01)
        restored = rotate_base(changed, 0, -5)
        self.assertAlmostEqual(rotation_distance(pose[3:], restored[3:]), 0, places=6)


class ManualTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.store = PoseStore(Path(self.directory.name)/'poses.json')
        self.robot = DemoRobot()
        self.service = ManualService(ManualSettings(), self.store, demo=True,
                                     factory=lambda host: (self.robot, self.robot))
        self.addCleanup(self.service.close)
        self.service.submit('connect')
        wait_for(lambda: self.service.state()['connected'] and not self.service.state()['pending'])
        self.service.heartbeat()

    def move(self, command, payload):
        self.service.submit(command, payload)
        wait_for(lambda: not self.service.state()['pending'])

    def test_translation_is_mm_and_recording_uses_fresh_feedback(self):
        before = self.service.state()['feedback']['tcp_pose_m_rad'][0]
        self.move('jog', {'axis':'x', 'step':2})
        self.assertTrue(self.service.state()['moving'])
        with self.assertRaises(ValueError):
            self.service.record('during move')
        wait_for(lambda: not self.service.state()['moving'])
        pose = self.service.record('pick')
        self.assertAlmostEqual(pose['tcp_pose_m_rad'][0], before+.002)
        self.assertTrue(pose['demo'])

    def test_jog_and_workspace_limits_block_commands(self):
        for payload in ({'axis':'x', 'step':11}, {'axis':'rx', 'step':float('nan')}, {'axis':'bad', 'step':1}):
            self.move('jog', payload)
            self.assertFalse(self.service.state()['moving'])
            self.assertIsNotNone(self.service.state()['error'])
        self.move('move_tcp', {'pose_mm_deg':[5000, 0, 300, 0, 180, 0]})
        self.assertIn('workspace', self.service.state()['error'])
        self.assertIsNone(self.robot._move)

    def test_overlapping_moves_rejected_and_stop_is_processed(self):
        self.move('jog', {'axis':'z', 'step':5})
        with self.assertRaises(ValueError):
            self.service.submit('jog', {'axis':'x', 'step':1})
        self.service.stop()
        wait_for(lambda: not self.service.state()['moving'])
        self.assertIsNone(self.robot._move)

    def test_disconnect_stops_and_cleans_up(self):
        self.move('jog', {'axis':'z', 'step':5})
        self.service.submit('disconnect')
        wait_for(lambda: not self.service.state()['connected'])
        self.assertFalse(self.robot.connected)
        self.assertIsNone(self.robot._move)

    def test_stale_feedback_stops_and_disconnects(self):
        self.move('jog', {'axis':'z', 'step':5})
        stamp = self.robot.getTimestamp()
        with patch.object(self.robot, 'getTimestamp', return_value=stamp):
            self.service._last_timestamp = stamp
            self.service._fresh_at = time.monotonic()-1
            wait_for(lambda: not self.service.state()['connected'])
        self.assertIn('stale', self.service.state()['error'])
        self.assertIsNone(self.robot._move)

    def test_heartbeat_loss_stops_and_disconnects(self):
        self.move('jog', {'axis':'z', 'step':5})
        self.service._heartbeat_at = time.monotonic()-3
        wait_for(lambda: not self.service.state()['connected'])
        self.assertIn('heartbeat', self.service.state()['error'])
        self.assertIsNone(self.robot._move)

    def test_settings_require_disconnected_robot(self):
        with self.assertRaises(ValueError):
            self.service.update_settings(asdict(ManualSettings()))

    def test_saved_pose_recall_with_async_joint_motion(self):
        saved = self.service.record('home')
        self.move('move_saved', {'id': saved['id'], 'mode':'joint'})
        self.assertTrue(self.service.state()['moving'])
        wait_for(lambda: not self.service.state()['moving'])
        self.assertIsNone(self.service.state()['error'])
