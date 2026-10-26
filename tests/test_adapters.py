import unittest
from unittest.mock import Mock, patch

from ur_pick_place.config import Gripper
from ur_pick_place.controller import SequenceError
from ur_pick_place.grippers.adapters import Arduino, RG2
from ur_pick_place.grippers.arduino_driver import PadPressure


class AdapterTests(unittest.TestCase):
    def arduino(self):
        g = Arduino.__new__(Arduino)
        g.config = Gripper(kind='arduino', serial_port='fake', grip_min_raw=[200, 300],
                           release_max_raw=[20, 30], pressure_samples=2, settle_s=0)
        g.driver = Mock()
        return g

    def test_both_pressure_pads_must_confirm_grip(self):
        g = self.arduino()
        g.driver.pressure.return_value = PadPressure(250, 299)
        with patch('ur_pick_place.grippers.adapters.time.sleep'), self.assertRaises(SequenceError):
            g.grip()
        g.driver.tight.assert_called_once()

    def test_all_pressure_samples_must_confirm_release(self):
        g = self.arduino()
        g.driver.pressure.side_effect = [PadPressure(0, 0), PadPressure(0, 50)]
        with patch('ur_pick_place.grippers.adapters.time.sleep'), self.assertRaises(SequenceError):
            g.open()

    def test_calibrated_grip_and_release(self):
        g = self.arduino()
        with patch('ur_pick_place.grippers.adapters.time.sleep'):
            g.driver.pressure.return_value = PadPressure(250, 350)
            g.grip()
            g.require_holding()
            g.driver.pressure.return_value = PadPressure(0, 0)
            g.open()

    def test_rg2_probe_guard_blocks_grip_after_fault(self):
        g = RG2.__new__(RG2)
        g.driver = Mock()
        g.driver.check_safety.side_effect = [None, RuntimeError('fault')]
        with self.assertRaises(RuntimeError):
            with g.probe_guard() as check:
                check()
        g.driver.move.assert_not_called()
