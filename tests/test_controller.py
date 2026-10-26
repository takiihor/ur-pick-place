from contextlib import contextmanager, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from ur_pick_place import Recipe, PickPlaceController
from ur_pick_place.cli import main
from ur_pick_place.controller import ProbeResult, SequenceError

ROOT = Path(__file__).resolve().parents[1]


def recipe():
    return Recipe.load(ROOT / 'examples/rg2.json')


class FakeRobot:
    def __init__(self, events, pick_reason='endpoint', pick_z=.25, failure=None):
        self.events, self.pick_reason, self.pick_z, self.failure = events, pick_reason, pick_z, failure
        self.probes = 0

    def preflight(self, joints, poses):
        self.events.append(('preflight',))

    def move_joints(self, joints):
        self.events.append(('joint',))

    def move_linear(self, pose, grasp_speed=False):
        self.events.append(('linear', pose))

    def probe(self, settings, endpoint_z=None, guard=lambda: None):
        self.events.append(('force_start',))
        self.probes += 1
        guard()
        self.events.append(('force_stop',))
        if self.failure:
            raise SequenceError(self.failure)
        if endpoint_z is not None:
            return ProbeResult([.4, -.1, self.pick_z, 0, 3.141592653589793, 0], self.pick_reason)
        return ProbeResult([.45, -.17, .24, 0, 3.141592653589793, 0], 'contact')


class FakeGripper:
    def __init__(self, events, fail=None):
        self.events, self.fail, self.opens = events, fail, 0

    def require_ready(self):
        self.events.append(('ready',))

    def open(self):
        self.opens += 1
        self.events.append(('open',))
        if self.fail == 'release' and self.opens > 1:
            raise SequenceError('Release failed')

    def grip(self):
        self.events.append(('grip',))

    def require_holding(self):
        self.events.append(('holding',))
        if self.fail == 'holding':
            raise SequenceError('No grip')

    @contextmanager
    def probe_guard(self):
        yield lambda: None


class ControllerTests(unittest.TestCase):
    def run_fake(self, config=None, **kwargs):
        events = []
        r = FakeRobot(events, **kwargs)
        g = FakeGripper(events)
        count = PickPlaceController(config or recipe(), r, g).run()
        return count, events

    def test_complete_sequence_order(self):
        count, events = self.run_fake()
        self.assertEqual(count, 1)
        self.assertEqual([e[0] for e in events], [
            'preflight', 'ready', 'joint', 'open', 'linear',
            'force_start', 'force_stop', 'linear', 'grip', 'holding',
            'linear', 'holding', 'joint', 'holding', 'linear',
            'force_start', 'force_stop', 'open', 'linear'])
        self.assertAlmostEqual(events[-1][1][2], .34)  # retract from measured placement

    def test_early_contact_backoff_then_xyz_offset_and_lift(self):
        c = recipe()
        c.grasp_offset_mm = [10, -20, 5]
        _, events = self.run_fake(c, pick_reason='contact', pick_z=.29)
        grasp, lift = events[7][1], events[10][1]
        for actual, expected in zip(grasp[:3], [.41, -.12, .345]):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(lift[2], .445)
        self.assertEqual(grasp[3:], c.pick_pose[3:])

    def test_endpoint_offset_without_backoff(self):
        c = recipe()
        c.grasp_offset_mm = [0, 0, 10]
        _, events = self.run_fake(c)
        self.assertAlmostEqual(events[7][1][2], .26)
        self.assertAlmostEqual(events[10][1][2], .36)

    def test_camera_target_overrides_recipe_and_is_validated_before_motion(self):
        c = recipe()
        controller = PickPlaceController(c)
        target = [.5, -.2, .3, 0, 3.14, 0]
        self.assertEqual(controller.plan(target)['pick'], target)
        events = []
        controller = PickPlaceController(c, FakeRobot(events), FakeGripper(events))
        with self.assertRaises(SequenceError):
            controller.run(pick_pose=[2, 0, .2, 0, 0, 0])
        self.assertEqual(events, [])

    def test_out_of_workspace_offset_blocks_all_motion(self):
        c = recipe()
        c.grasp_offset_mm = [10000, 0, 0]
        events = []
        with self.assertRaises(SequenceError):
            PickPlaceController(c, FakeRobot(events), FakeGripper(events)).run()
        self.assertEqual(events, [])

    def test_force_stop_failure_blocks_grip(self):
        events = []
        with self.assertRaises(SequenceError):
            PickPlaceController(recipe(), FakeRobot(events, failure='Stop failed'), FakeGripper(events)).run()
        self.assertNotIn('grip', [e[0] for e in events])
        self.assertEqual(events[-1][0], 'force_stop')

    def test_missing_grip_blocks_lift_and_joint_return(self):
        events = []
        with self.assertRaises(SequenceError):
            PickPlaceController(recipe(), FakeRobot(events), FakeGripper(events, 'holding')).run()
        self.assertEqual(events[-1][0], 'holding')
        self.assertEqual([e[0] for e in events].count('joint'), 1)

    def test_failed_release_blocks_retract_and_next_cycle(self):
        events = []
        with self.assertRaises(SequenceError):
            PickPlaceController(recipe(), FakeRobot(events), FakeGripper(events, 'release')).run(cycles=2)
        self.assertEqual(events[-1][0], 'open')
        self.assertEqual([e[0] for e in events].count('ready'), 1)

    def test_successful_cycles_and_final_home(self):
        events = []
        c = recipe()
        c.return_home = True
        self.assertEqual(PickPlaceController(c, FakeRobot(events), FakeGripper(events)).run(
            cycles=2, cycle_wait_s=0), 2)
        self.assertEqual([e[0] for e in events].count('joint'), 6)

    def test_default_cli_preview_imports_no_hardware(self):
        with patch('ur_pick_place.session.connect', side_effect=AssertionError('hardware')), redirect_stdout(io.StringIO()) as out:
            self.assertEqual(main(['--config', str(ROOT / 'examples/rg2.json')]), 0)
        self.assertIn('Preview only', out.getvalue())

    def test_arduino_example_previews_but_cannot_execute_uncalibrated(self):
        c = Recipe.load(ROOT / 'examples/arduino.json')
        PickPlaceController(c).plan()
        with self.assertRaises(ValueError):
            c.gripper.require_live_ready()

    def test_strict_recipe_validation(self):
        raw = json.loads((ROOT / 'examples/rg2.json').read_text())
        for field, value in [('grasp_offset_mm', [True, 0, 0]), ('pick_pose', [0]*5),
                             ('return_home', 1), ('unknown', 1), ('lift_mm', float('nan'))]:
            bad = dict(raw, **{field: value})
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'bad.json'
                path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):
                    Recipe.load(path)
