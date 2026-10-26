import time
import unittest
from unittest.mock import Mock, patch

from ur_pick_place.controller import PickPlaceController, SequenceError
from ur_pick_place.robot import RTDERobot, rotation_distance
from test_controller import recipe


class ProbeTests(unittest.TestCase):
    def make_robot(self, samples, stop=True):
        robot = RTDERobot.__new__(RTDERobot)
        robot.config = recipe()
        robot.control = Mock()
        robot.receive = Mock()
        robot.force_active = False
        robot.validator = PickPlaceController(robot.config)
        robot.control.forceMode.return_value = True
        robot.control.forceModeStop.return_value = stop
        robot._baseline = Mock(return_value=0)
        robot._read = Mock(side_effect=samples)
        return robot

    @staticmethod
    def sample(z, fz=0):
        return ([.4, -.1, z, 0, 3.141592653589793, 0], [0, 0, fz, 0, 0, 0])

    def test_force_refreshes_until_endpoint_then_stops(self):
        r = self.make_robot([self.sample(.35), self.sample(.35), self.sample(.3), self.sample(.25)])
        result = r.probe(r.config.pick_probe, endpoint_z=.25)
        self.assertEqual(result.reason, 'endpoint')
        self.assertEqual(r.control.forceMode.call_count, 2)
        self.assertEqual(r.control.forceMode.call_args.args[2][2], -12)
        r.control.forceModeStop.assert_called_once()
        self.assertFalse(r.force_active)

    def test_contact_returns_measured_pose(self):
        r = self.make_robot([self.sample(.35), self.sample(.35), self.sample(.29, 11)])
        result = r.probe(r.config.pick_probe, endpoint_z=.25)
        self.assertEqual(result.reason, 'contact')
        self.assertAlmostEqual(result.pose[2], .29)
        r.control.forceModeStop.assert_called_once()

    def test_placement_travel_budget_requires_contact(self):
        r = self.make_robot([self.sample(.35), self.sample(.35), self.sample(.19)])
        with self.assertRaisesRegex(SequenceError, 'travel budget'):
            r.probe(r.config.place_probe)
        r.control.forceModeStop.assert_called_once()

    def test_stop_failure_prevents_success(self):
        r = self.make_robot([self.sample(.35), self.sample(.35), self.sample(.25)], stop=False)
        with self.assertRaisesRegex(SequenceError, 'Force mode stop'):
            r.probe(r.config.pick_probe, endpoint_z=.25)
        self.assertTrue(r.force_active)

    def test_force_update_failure_still_stops(self):
        r = self.make_robot([self.sample(.35), self.sample(.35)])
        r.control.forceMode.return_value = False
        with self.assertRaisesRegex(SequenceError, 'Force mode update'):
            r.probe(r.config.pick_probe, endpoint_z=.25)
        r.control.forceModeStop.assert_called_once()

    def test_guard_failure_stops_force(self):
        r = self.make_robot([self.sample(.35), self.sample(.35)])
        guard = Mock(side_effect=[None, None, None, SequenceError('guard failed')])
        with self.assertRaisesRegex(SequenceError, 'guard failed'):
            r.probe(r.config.pick_probe, endpoint_z=.25, guard=guard)
        r.control.forceModeStop.assert_called_once()

    def test_timeout_stops_force(self):
        r = self.make_robot([self.sample(.35)] * 10)
        with patch('ur_pick_place.robot.time.monotonic', side_effect=[0, 0, 1000]):
            with self.assertRaisesRegex(SequenceError, 'timed out'):
                r.probe(r.config.pick_probe, endpoint_z=.25)
        r.control.forceModeStop.assert_called_once()

    def test_lateral_drift_stops_force(self):
        off = self.sample(.30)
        off[0][0] += .02
        r = self.make_robot([self.sample(.35), self.sample(.35), off])
        with self.assertRaisesRegex(SequenceError, 'Lateral drift'):
            r.probe(r.config.pick_probe, endpoint_z=.25)
        r.control.forceModeStop.assert_called_once()

    def test_read_rejects_stale_feedback(self):
        r = RTDERobot.__new__(RTDERobot)
        r.config = recipe()
        r.validator = PickPlaceController(r.config)
        r.control = Mock()
        r.receive = Mock()
        r.receive.getSafetyMode.return_value = 1
        r.receive.getTimestamp.return_value = 123.0
        r._last_timestamp = 123.0
        r._last_fresh_at = time.monotonic() - 1
        with self.assertRaisesRegex(SequenceError, 'stopped updating'):
            r._read()

    def test_equivalent_rotation_vectors(self):
        self.assertAlmostEqual(rotation_distance([0, 3.141592653589793, 0],
                                                 [0, -3.141592653589793, 0]), 0)
