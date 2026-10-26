"""Protocol tests with fake serial hardware; no robot or gripper commands."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


from ur_pick_place.grippers.arduino_driver import ArduinoGripper, GripperError, PadPressure


class ArduinoGripperTests(unittest.TestCase):
    def setUp(self):
        self.connection = Mock()
        self.connection.write.side_effect = len
        self.factory = Mock(return_value=self.connection)
        self.modules = patch.dict(sys.modules, {
            'serial': SimpleNamespace(Serial=self.factory, SerialException=OSError),
        })
        self.modules.start()
        self.addCleanup(self.modules.stop)
        self.gripper = ArduinoGripper('fake-port', startup_wait_s=0)
        self.addCleanup(self.gripper.close)

    def test_opening_connection_does_not_move_jaws(self):
        self.connection.write.assert_not_called()
        self.assertEqual(self.factory.call_args.kwargs['baudrate'], 9600)

    def test_jaw_commands_match_firmware(self):
        for action, command, reply in (
            ('tight', b'1\n', b'Tight\n'),
            ('loose', b'2\n', b'Loose\r\n'),
            ('open', b'3\n', b'Open\n'),
        ):
            with self.subTest(action=action):
                self.connection.readline.return_value = reply
                self.assertEqual(getattr(self.gripper, action)(), reply.decode().strip())
                self.connection.write.assert_called_with(command)

    def test_pressure_is_raw_sum(self):
        self.connection.readline.return_value = b'FSR1:3069 / FSR2:123\n'
        self.assertEqual(self.gripper.pressure(), PadPressure(3069, 123))
        self.connection.write.assert_called_once_with(b'0\n')

    def test_invalid_pressure_rejected(self):
        for reply in (b'FSR1:3070 / FSR2:0\n', b'garbage\n', b'FSR1:-1 / FSR2:0\n'):
            with self.subTest(reply=reply):
                self.connection.readline.return_value = reply
                with self.assertRaises(GripperError):
                    self.gripper.pressure()

    def test_missing_partial_and_unexpected_replies_never_retry(self):
        for reply in (b'', b'Tight', b'Loose\n', b'\xff\n'):
            with self.subTest(reply=reply):
                self.connection.write.reset_mock()
                self.connection.readline.return_value = reply
                with self.assertRaises(GripperError):
                    self.gripper.tight()
                self.connection.write.assert_called_once_with(b'1\n')

    def test_serial_error_wrapped_without_retry(self):
        self.connection.readline.side_effect = OSError('disconnected')
        with self.assertRaises(GripperError):
            self.gripper.tight()
        self.connection.write.assert_called_once_with(b'1\n')

    def test_partial_write_rejected(self):
        self.connection.write.side_effect = None
        self.connection.write.return_value = 1
        with self.assertRaises(GripperError):
            self.gripper.open()
        self.connection.readline.assert_not_called()

    def test_context_exit_does_not_release(self):
        with self.gripper:
            pass
        self.connection.close.assert_called_once()
        self.connection.write.assert_not_called()


if __name__ == '__main__':
    unittest.main()
