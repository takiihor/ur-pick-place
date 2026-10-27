import signal
import unittest

from ur_pick_place.shutdown import stop_on_termination


class TerminationTests(unittest.TestCase):
    def test_sigterm_uses_interrupt_path_and_restores_handler(self):
        previous = signal.getsignal(signal.SIGTERM)
        with self.assertRaises(KeyboardInterrupt), stop_on_termination():
            signal.raise_signal(signal.SIGTERM)
        self.assertEqual(signal.getsignal(signal.SIGTERM), previous)
