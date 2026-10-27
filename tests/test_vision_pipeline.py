"""End-to-end simulation: observe -> teach-grasp -> vision-pick preview -> execute, and refusals.

The fake world renders the real markers through the true camera pose; the calibration
file carries a deliberate hand-eye error. No robot, camera or network is touched.
"""
from contextlib import contextmanager, redirect_stdout
from dataclasses import asdict
import io
import json
import math
from pathlib import Path
import shutil
import tempfile
import unittest

from vision_sim import (CAMERA_IN_FLANGE, HOME, MARKER_TCP, OBSERVE_PICK, OBSERVE_PLACE, PLACE_MARKER, TANK,
                        TCP_OFFSET, FakeCamera, FakeGripper, FakeRTDE, calibration_dict, camera_view_point, face_up,
                        handeye_error, make_transform, recipe, rtde_robot, vision_config)

import numpy as np

from ur_pick_place.controller import PickPlaceController
from ur_pick_place.vision.cli import main
from ur_pick_place.vision.transforms import invert, matrix_to_pose, pose_to_matrix, rotation_angle

PLACE_TCP = make_transform((0, 0, 0), (0.04, 0.0, -0.03))  # release 30 mm above the place marker plane
PICK_A = camera_view_point(OBSERVE_PICK) + [0.02, -0.015, 0]
PLACE_POINT = camera_view_point(OBSERVE_PLACE) + [-0.01, 0.01, 0]


class FakeRuntime:
    def __init__(self, rtde, camera):
        self.rtde, self.camera_device, self.calls = rtde, camera, []
        self.gripper = FakeGripper(rtde)

    def receive(self, host):
        self.calls.append('receive')
        self.rtde.connected = True  # every CLI run opens a fresh connection
        return self.rtde

    def camera(self, config):
        self.calls.append('camera')
        return self.camera_device

    def robot(self, recipe):
        self.calls.append('robot')
        self.rtde.connected = True
        return rtde_robot(recipe, self.rtde)

    @contextmanager
    def connect(self, recipe):
        self.calls.append('connect')
        self.rtde.connected = True
        yield PickPlaceController(recipe, rtde_robot(recipe, self.rtde), self.gripper)


class World:
    """Pick marker on an object, place marker on a fixture, contact surfaces for force probing."""

    def __init__(self):
        self.pick = face_up(PICK_A[0], PICK_A[1], yaw_deg=20)
        self.place = face_up(PLACE_POINT[0], PLACE_POINT[1], yaw_deg=-10)
        self.rtde = FakeRTDE(surfaces=self.surface)
        self.camera = FakeCamera(self.rtde, [(TANK, self.pick), (PLACE_MARKER, self.place)])

    def move_pick(self, dx_mm, dy_mm, dyaw_deg):
        """Shift in base XY and turn about the marker's own vertical axis."""
        moved = self.pick.copy()
        moved[:3, :3] = make_transform((0, 0, math.radians(dyaw_deg)))[:3, :3] @ self.pick[:3, :3]
        moved[:2, 3] += np.array([dx_mm, dy_mm]) / 1000
        self.pick = moved
        self.camera.scene[0] = (TANK, self.pick)

    def surface(self, x, y):
        pick_top = (self.pick @ MARKER_TCP)[2, 3] - 0.004   # below the endpoint: the search reaches it
        place_top = (self.place @ PLACE_TCP)[2, 3] - 0.008  # contact before the place reference
        if math.hypot(x - self.pick[0, 3], y - self.pick[1, 3]) < 0.12:
            return pick_top
        if math.hypot(x - self.place[0, 3], y - self.place[1, 3]) < 0.12:
            return place_top
        return -1.0


def run(runtime, *argv):
    out = io.StringIO()
    with redirect_stdout(out):
        code = main(list(map(str, argv)), runtime=runtime)
    return code, out.getvalue()


def first_json(text):
    return json.JSONDecoder().raw_decode(text[text.index('{'):])[0]


class VisionPipelineTests(unittest.TestCase):
    refine = False

    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.root = Path(cls.directory.name)
        estimated = CAMERA_IN_FLANGE @ handeye_error()  # deliberately imperfect calibration
        cls.vision = vision_config(cls.root, refine=cls.refine, camera_in_flange=estimated)
        cls.recipe_path = cls.root / 'pick_place.json'
        cls.recipe_path.write_text(json.dumps(asdict(recipe())))
        world = World()
        runtime = FakeRuntime(world.rtde, world.camera)
        for role, marker, tcp in (('pick', world.pick, MARKER_TCP), ('place', world.place, PLACE_TCP)):
            code, out = run(runtime, 'observe', '--config', cls.recipe_path, '--vision', cls.vision, '--role', role,
                            '--execute', '--save', cls.root / f'obs_{role}.json')
            assert code == 0, out
            code, out = run(runtime, 'teach-grasp', '--vision', cls.vision, '--role', role,
                            '--observation', cls.root / f'obs_{role}.json',
                            '--grasp-tcp', *matrix_to_pose(marker @ tcp))
            assert code == 0, out

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def setUp(self):
        self.world = World()
        self.runtime = FakeRuntime(self.world.rtde, self.world.camera)
        self.preview = self.root / f'preview_{self._testMethodName}.json'

    def common(self):
        return ['--config', self.recipe_path, '--vision', self.vision]

    def preview_cycle(self, *extra):
        if self.refine:  # an exact preview needs the same refine view as execution
            return run(self.runtime, 'vision-pick', *self.common(), '--observe-motion', '--preview-out', self.preview,
                       *extra)
        self.world.rtde.moveJ(OBSERVE_PICK, 0.1, 0.1)  # operator parks the robot at the observe pose
        return run(self.runtime, 'vision-pick', *self.common(), '--observation',
                   f'place={self.root / "obs_place.json"}', '--preview-out', self.preview, *extra)

    def execute(self):
        return run(self.runtime, 'vision-pick', *self.common(), '--execute', '--preview', self.preview)

    def assert_no_pick_motion(self):
        self.assertNotIn('grip', self.runtime.gripper.events)
        self.assertNotIn(('moveJ', HOME), self.world.rtde.events)

    def test_observe_read_only_reports_base_pose(self):
        self.world.rtde.moveJ(OBSERVE_PICK, 0.1, 0.1)
        self.world.rtde.events.clear()
        code, out = run(self.runtime, 'observe', *self.common())
        self.assertEqual(code, 0, out)
        report = first_json(out)
        self.assertEqual(report['mode'], 'read-only')
        observed = pose_to_matrix(report['observation']['target_in_base_pose_ur'])
        # Absolute accuracy is limited by the injected hand-eye error (~2 mm), not by detection.
        error_mm = np.linalg.norm(observed[:3, 3] - self.world.pick[:3, 3]) * 1000
        self.assertLess(error_mm, 6)
        self.assertEqual(self.world.rtde.events, [])  # read-only: nothing commanded
        self.assertNotIn('robot', self.runtime.calls)

    def test_taught_pick_and_marker_place_end_to_end(self):
        self.world.move_pick(14, -9, 8)
        code, out = self.preview_cycle('--write-pick-target', self.root / 'target.json')
        self.assertEqual(code, 0, out)
        preview = json.loads(self.preview.read_text())
        self.assertIsNone(preview['consumed_at'])
        self.assertEqual(json.loads((self.root / 'target.json').read_text()), preview['pick_pose'])
        self.assertEqual(self.runtime.gripper.events, [])
        self.assertEqual(first_json(out)['warnings'], [])
        code, out = self.execute()
        self.assertEqual(code, 0, out)
        truth = self.world.pick @ MARKER_TCP
        grip = self.runtime.gripper.grip_tcp
        self.assertIsNotNone(grip)
        error_mm = np.linalg.norm(grip[:3, 3] - truth[:3, 3]) * 1000
        error_deg = math.degrees(rotation_angle(grip, truth))
        # Hand-eye error (~3 mm absolute) mostly cancels; it cancels almost fully with the refine view.
        self.assertLess(error_mm, 0.25 if self.refine else 1.0, f'grip error {error_mm:.3f} mm')
        self.assertLess(error_deg, 0.4)
        self.assertEqual(self.runtime.gripper.events, ['ready', 'open', 'grip', 'open'])
        # Placement followed the place marker: release happened at contact above it.
        place_truth = self.world.place @ PLACE_TCP
        release = [e for e in self.world.rtde.events if e[0] == 'forceModeStop']
        self.assertEqual(len(release), 2)
        self.assertLess(np.linalg.norm(self.world.rtde.tcp_matrix()[:2, 3] - place_truth[:2, 3]) * 1000, 0.6)
        self.assertIsNotNone(json.loads(self.preview.read_text())['consumed_at'])
        code, out = self.execute()
        self.assertEqual(code, 1)  # one preview, one execution

    def test_hand_eye_error_cancels_through_taught_relationship(self):
        self.world.move_pick(10, 6, -5)
        code, out = self.preview_cycle()
        self.assertEqual(code, 0, out)
        plan = first_json(out)
        observed = pose_to_matrix(plan['observations']['pick']['target_in_base_pose_ur'])
        absolute_error = np.linalg.norm(observed[:3, 3] - self.world.pick[:3, 3]) * 1000
        pick = pose_to_matrix(plan['pick_pose'])
        relative_error = np.linalg.norm(pick[:3, 3] - (self.world.pick @ MARKER_TCP)[:3, 3]) * 1000
        self.assertGreater(absolute_error, 1.0)  # the calibration really is off
        self.assertLess(relative_error, absolute_error / 3)
        if self.refine:  # marker-relative view: the hand-eye error cancels almost exactly
            self.assertLess(relative_error, 0.2)

    def test_shift_beyond_limit_refused_at_preview(self):
        self.world.move_pick(40, 0, 0)
        code, out = self.preview_cycle()
        self.assertEqual(code, 1)
        self.assertFalse(self.preview.exists())

    def test_target_moved_after_preview_refused_before_pick(self):
        self.assertEqual(self.preview_cycle()[0], 0)
        self.world.move_pick(6, 0, 0)
        with self.assertLogs(level='ERROR') as logs:
            code, _ = self.execute()
        self.assertEqual(code, 1)
        self.assertIn('since the preview', '\n'.join(logs.output))
        self.assert_no_pick_motion()

    def test_unreachable_target_refused_before_pick(self):
        self.assertEqual(self.preview_cycle()[0], 0)
        self.world.rtde.reachable = lambda pose: pose[2] > 0.1
        with self.assertLogs(level='ERROR') as logs:
            code, _ = self.execute()
        self.assertEqual(code, 1)
        self.assertIn('not reachable', '\n'.join(logs.output))
        self.assert_no_pick_motion()

    def test_tcp_changed_since_teaching_refused(self):
        self.assertEqual(self.preview_cycle()[0], 0)
        self.world.rtde.tcp = [0.0, 0.0, 0.215, 0.0, 0.0, 0.0]
        with self.assertLogs(level='ERROR') as logs:
            code, _ = self.execute()
        self.assertEqual(code, 1)
        self.assertIn('differs from the taught TCP', '\n'.join(logs.output))
        self.assert_no_pick_motion()

    def test_execute_requires_fresh_preview(self):
        code, _ = self.execute()
        self.assertEqual(code, 1)
        self.assertEqual(self.runtime.calls, [])
        code, _ = run(self.runtime, 'vision-pick', *self.common(), '--execute')
        self.assertEqual(code, 1)
        self.assertEqual(self.runtime.calls, [])

    def test_read_only_tcp_mismatch_refused(self):
        with self.assertLogs(level='ERROR') as logs:
            code, _ = run(self.runtime, 'observe', *self.common(), '--active-tcp', 0, 0, 0.15, 0, 0, 0)
        self.assertEqual(code, 1)
        self.assertIn('differs from the TCP recorded', '\n'.join(logs.output))
        self.assertNotIn('camera', self.runtime.calls)

    def test_quality_gate_failure_reports_reason(self):
        self.world.rtde.moveJ(OBSERVE_PICK, 0.1, 0.1)
        self.world.camera.shake_px = 1.0
        with self.assertLogs(level='ERROR') as logs:
            code, _ = run(self.runtime, 'observe', *self.common())
        self.assertEqual(code, 1)
        self.assertIn('corner jitter', '\n'.join(logs.output))

    def test_poor_calibration_refused_before_hardware(self):
        directory = self.root / 'poor'
        directory.mkdir(exist_ok=True)
        shutil.copy(self.vision, directory / 'vision.json')
        (directory / 'handeye.json').write_text(json.dumps(calibration_dict(verdict='poor')))
        with self.assertLogs(level='ERROR') as logs:
            code, _ = run(self.runtime, 'observe', '--config', self.recipe_path, '--vision', directory / 'vision.json')
        self.assertEqual(code, 1)
        self.assertIn("verdict is 'poor'", '\n'.join(logs.output))
        self.assertEqual(self.runtime.calls, [])

    def test_recalibration_invalidates_taught_grasp(self):
        directory = self.root / 'recalibrated'
        directory.mkdir(exist_ok=True)
        for name in ('vision.json', 'grasp_pick.json', 'grasp_place.json'):
            shutil.copy(self.root / name, directory / name)
        (directory / 'handeye.json').write_text(json.dumps(calibration_dict()))
        self.world.rtde.moveJ(OBSERVE_PICK, 0.1, 0.1)
        with self.assertLogs(level='ERROR') as logs:
            code, _ = run(self.runtime, 'vision-pick', '--config', self.recipe_path,
                          '--vision', directory / 'vision.json',
                          '--observation', f'place={self.root / "obs_place.json"}', '--preview-out', self.preview)
        self.assertEqual(code, 1)
        self.assertIn('re-teach', '\n'.join(logs.output))

    def test_read_only_preview_warns_when_refine_is_skipped(self):
        if not self.refine:
            self.skipTest('refine disabled')
        self.world.rtde.moveJ(OBSERVE_PICK, 0.1, 0.1)
        code, out = run(self.runtime, 'vision-pick', *self.common(), '--observation',
                        f'place={self.root / "obs_place.json"}', '--preview-out', self.preview)
        self.assertEqual(code, 0, out)
        self.assertIn('--observe-motion', first_json(out)['warnings'][0])
        self.assertNotIn('robot', self.runtime.calls)


class RefinedVisionPipelineTests(VisionPipelineTests):
    """Same scenarios with the marker-relative refine view during teaching and execution."""
    refine = True

    def test_refine_view_is_marker_relative(self):
        self.world.move_pick(12, -8, 10)
        self.assertEqual(self.preview_cycle()[0], 0)
        self.assertEqual(self.execute()[0], 0)
        refine_moves = [e for e in self.world.rtde.events if e[0] == 'moveL'][:2]
        camera = pose_to_matrix(refine_moves[0][1]) @ invert(pose_to_matrix(TCP_OFFSET)) @ CAMERA_IN_FLANGE
        relative = invert(camera) @ self.world.pick
        # Camera sits about 0.25 m in front of the marker, looking along its normal.
        self.assertAlmostEqual(relative[2, 3], 0.25, delta=0.01)
        self.assertLess(math.degrees(math.acos(min(1.0, relative[2, 2]))), 2.0)

