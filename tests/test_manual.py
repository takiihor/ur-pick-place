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

    def test_coordinate_update_preserves_id_and_is_persisted(self):
        original = self.save()
        tcp, joints = [.45, -.2, .3, 0, math.pi, 0], [.1]*6
        saved = self.store.save('home', tcp, joints, robot_ip='new-host', demo=True, ident=original['id'])
        self.assertEqual(saved['id'], original['id'])
        self.assertEqual(len(self.store.list()), 1)
        reloaded = PoseStore(self.path).get(original['id'])
        self.assertEqual(reloaded['tcp_pose_m_rad'], tcp)
        self.assertEqual(reloaded['joints_rad'], joints)
        self.assertEqual(reloaded['robot_ip'], 'new-host')
        self.assertNotEqual(reloaded['recorded_at'], original['recorded_at'])

    def test_failed_coordinate_update_preserves_existing_pose(self):
        original = self.save()
        before = self.path.read_bytes()
        with patch('ur_pick_place.poses.os.replace', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                self.store.save('home', [.5, 0, .3, 0, 0, 0], [1]*6,
                                robot_ip='demo', demo=True, ident=original['id'])
        self.assertEqual(self.store.get(original['id']), original)
        self.assertEqual(self.path.read_bytes(), before)

    def test_coordinate_update_rejects_unknown_id_and_mixed_modes(self):
        original = self.save()
        for ident, demo in (('missing', True), (original['id'], False)):
            with self.assertRaises(ValueError):
                self.store.save('home', [.5, 0, .3, 0, 0, 0], [1]*6,
                                robot_ip='demo', demo=demo, ident=ident)
        self.assertEqual(self.store.get(original['id']), original)

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

    def test_record_existing_waypoint_updates_both_coordinate_sets(self):
        original = self.service.record('pick')
        self.robot.pose[0] += .02
        self.robot.joints[0] += .1
        wait_for(lambda: self.service.state()['feedback']['joints_rad'][0] == self.robot.joints[0])
        updated = self.service.record(' PICK ')
        self.assertEqual(updated['id'], original['id'])
        self.assertEqual(updated['name'], 'pick')
        self.assertEqual(updated['tcp_pose_m_rad'], self.robot.pose)
        self.assertEqual(updated['joints_rad'], self.robot.joints)
        self.assertEqual(len(self.store.list()), 1)
        # Updating by ID keeps a renamed waypoint and invalidates old previews.
        self.store.rename(original['id'], 'pick_new')
        self.service._preview = {'old':'preview'}
        self.assertEqual(self.service.record(ident=original['id'])['name'], 'pick_new')
        self.assertIsNone(self.service._preview)

    def test_waypoint_update_requires_stationary_fresh_feedback(self):
        original = self.service.record('pick')
        self.move('jog_start', {'axis':'x', 'speed':1, 'jog_id':'held'})
        with self.assertRaises(ValueError):
            self.service.record(ident=original['id'])
        self.service.stop()
        wait_for(lambda: not self.service.state()['moving'])
        with self.service._lock:
            self.service._feedback['received_at'] = time.monotonic()-1
            with self.assertRaises(ValueError):
                self.service.record('pick')
        self.assertEqual(self.store.get(original['id']), original)

    def test_jog_and_workspace_limits_block_commands(self):
        for payload in ({'axis':'x', 'step':11}, {'axis':'rx', 'step':float('nan')}, {'axis':'bad', 'step':1}):
            self.move('jog', payload)
            self.assertFalse(self.service.state()['moving'])
            self.assertIsNotNone(self.service.state()['error'])
        self.move('move_tcp', {'pose_mm_deg':[5000, 0, 300, 0, 180, 0]})
        self.assertIn('workspace', self.service.state()['error'])
        self.assertIsNone(self.robot._move)

    def test_held_jog_moves_continuously_until_release(self):
        before = self.robot.getActualTCPPose()[0]
        self.move('jog_start', {'axis':'x', 'speed':10, 'jog_id':'held'})
        for _ in range(8):
            self.service.heartbeat('held')
            time.sleep(.1)
        self.assertTrue(self.service.state()['moving'])
        self.assertGreater(self.robot.getActualTCPPose()[0], before+.006)
        self.assertIsNone(self.robot._move)  # Velocity control, no stepped moves.
        self.service.stop()
        wait_for(lambda: not self.service.state()['moving'])
        stopped = self.robot.getActualTCPPose()
        time.sleep(.1)
        self.assertEqual(self.robot.getActualTCPPose(), stopped)

    def test_regular_heartbeat_does_not_keep_released_jog_alive(self):
        self.move('jog_start', {'axis':'x', 'speed':1, 'jog_id':'held'})
        for _ in range(8):
            self.service.heartbeat()
            self.service.heartbeat('different-button')
            time.sleep(.1)
        self.assertFalse(self.service.state()['moving'])
        self.assertTrue(self.service.state()['connected'])
        self.assertIsNone(self.robot._velocity)
        self.service.heartbeat('held')
        time.sleep(.1)
        self.assertFalse(self.service.state()['moving'])

    def test_held_jog_rotation_is_about_base_axis(self):
        before = self.robot.getActualTCPPose()
        self.move('jog_start', {'axis':'rx', 'speed':-5, 'jog_id':'held'})
        time.sleep(.15)
        self.service.stop()
        wait_for(lambda: not self.service.state()['moving'])
        after = self.robot.getActualTCPPose()
        self.assertEqual(before[:3], after[:3])
        self.assertGreater(rotation_distance(before[3:], after[3:]), .005)
        expected = rotate_base(before, 0, -math.degrees(rotation_distance(before[3:], after[3:])))
        self.assertAlmostEqual(rotation_distance(expected[3:], after[3:]), 0, places=6)

    def test_held_jog_all_directions_and_translation_speed_cap(self):
        self.service.settings.tcp_speed_mm_s = 2
        for index, axis in enumerate(('x', 'y', 'z', 'rx', 'ry', 'rz')):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    self.move('jog_start', {'axis':axis, 'speed':10*sign, 'jog_id':'held'})
                    velocity = self.robot._velocity
                    expected = 2/1000 if index < 3 else math.radians(10)
                    self.assertAlmostEqual(velocity[index], expected*sign)
                    self.assertTrue(all(v == 0 for i, v in enumerate(velocity) if i != index))
                    self.service.stop()
                    wait_for(lambda: not self.service.state()['moving'])

    def test_disconnect_stops_held_jog(self):
        self.move('jog_start', {'axis':'z', 'speed':1, 'jog_id':'held'})
        self.service.submit('disconnect')
        wait_for(lambda: not self.service.state()['connected'])
        self.assertIsNone(self.robot._velocity)

    def test_held_jog_supports_faster_translation_with_configured_cap(self):
        self.service.settings.tcp_speed_mm_s = 50
        self.move('jog_start', {'axis':'x', 'speed':40, 'jog_id':'held'})
        self.assertAlmostEqual(self.robot._velocity[0], .04)
        self.service.stop()
        wait_for(lambda: not self.service.state()['moving'])
        self.move('jog_start', {'axis':'y', 'speed':-100, 'jog_id':'held'})
        self.assertAlmostEqual(self.robot._velocity[1], -.05)

    def test_stale_feedback_stops_held_jog(self):
        self.move('jog_start', {'axis':'z', 'speed':1, 'jog_id':'held'})
        stamp = self.robot.getTimestamp()
        with patch.object(self.robot, 'getTimestamp', return_value=stamp):
            self.service._last_timestamp = stamp
            self.service._fresh_at = time.monotonic()-1
            wait_for(lambda: not self.service.state()['connected'])
        self.assertIsNone(self.robot._velocity)
        self.assertIn('stale', self.service.state()['error'])

    def test_held_jog_validates_speed_and_safety_lookahead(self):
        for payload in ({'axis':'x', 'speed':101, 'jog_id':'held'},
                        {'axis':'rx', 'speed':11, 'jog_id':'held'},
                        {'axis':'rx', 'speed':float('nan'), 'jog_id':'held'},
                        {'axis':'bad', 'speed':1, 'jog_id':'held'},
                        {'axis':'x', 'speed':0, 'jog_id':'held'}):
            self.move('jog_start', payload)
            self.assertFalse(self.service.state()['moving'])
            self.assertIsNotNone(self.service.state()['error'])
        self.robot.pose[0] = self.service.settings.workspace_max_m[0]-.001
        self.move('jog_start', {'axis':'x', 'speed':10, 'jog_id':'held'})
        self.assertIn('workspace', self.service.state()['error'])
        self.assertIsNone(self.robot._velocity)

    def test_held_jog_stops_when_next_target_is_unsafe(self):
        self.move('jog_start', {'axis':'z', 'speed':1, 'jog_id':'held'})
        with patch.object(self.robot, 'isPoseWithinSafetyLimits', return_value=False):
            wait_for(lambda: not self.service.state()['moving'])
        self.assertIsNone(self.robot._velocity)
        self.assertIn('safety limits', self.service.state()['error'])

    def test_stop_cancels_queued_held_jog(self):
        with self.service._lock:
            self.service.submit('jog_start', {'axis':'x', 'speed':1, 'jog_id':'held'})
            self.service.stop()
        wait_for(lambda: not self.service.state()['pending'])
        self.assertFalse(self.service.state()['moving'])
        self.assertIsNone(self.robot._velocity)

    def test_failed_jog_stop_disconnects(self):
        self.move('jog_start', {'axis':'x', 'speed':1, 'jog_id':'held'})
        with patch.object(self.robot, 'speedStop', return_value=False):
            self.service.stop()
            wait_for(lambda: not self.service.state()['connected'])
        self.assertIn('stop failed', self.service.state()['error'])

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
