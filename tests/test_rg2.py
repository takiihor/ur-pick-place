import threading
import time
import unittest
from unittest.mock import Mock, patch
from xmlrpc.server import SimpleXMLRPCRequestHandler, SimpleXMLRPCServer

from ur_pick_place.grippers.rg2_driver import GripperError, RG2Gripper, _Transport

def state(width=100.0, grip=False, busy=False, fault=False):
    return dict(width_mm=width, grip_detected=grip, busy=busy, safety_failed=fault)


class RG2Tests(unittest.TestCase):
    def test_read_only_status_does_not_command_jaws(self):
        client = RG2Gripper('unused')
        client._rpc = Mock()
        client._rpc.rg_get_width.return_value = 101.1
        client._rpc.rg_get_busy.return_value = False
        client._rpc.rg_get_grip_detected.return_value = False
        client._rpc.rg_get_safety_failed.return_value = False
        self.assertEqual(client.status(), state(101.1))
        client._rpc.rg_grip.assert_not_called()

    def simulate(self, states, **command_options):
        client = RG2Gripper('unused', timeout=0.5)
        client._rpc = Mock()
        client._rpc.rg_grip.return_value = 0
        samples = iter(states)
        last = [states[-1]]
        def next_state():
            last[0] = next(samples, last[0])
            return last[0]
        client.status = next_state
        now = [0.0]
        def advance(seconds):
            now[0] += seconds
        with patch('ur_pick_place.grippers.rg2_driver.time.monotonic', side_effect=lambda: now[0]), \
             patch('ur_pick_place.grippers.rg2_driver.time.sleep', side_effect=advance):
            result = client.move(20.0, 10.0, **command_options)
        return result, client

    def test_grip_requires_detected_object(self):
        with self.assertRaisesRegex(GripperError, 'timeout'):
            self.simulate([state(), state(20)], require_grip=True)
        result, client = self.simulate([state(), state(35, grip=True)], require_grip=True)
        self.assertTrue(result['grip_detected'])
        client._rpc.rg_grip.assert_called_once_with(0, 20.0, 10.0)

    def test_release_requires_no_grip_and_target_width(self):
        with self.assertRaises(GripperError):
            self.simulate([state(35, grip=True), state(20, grip=True)], require_release=True)
        with self.assertRaises(GripperError):
            self.simulate([state(), state(40)], require_release=True)
        result, _ = self.simulate([state(), state(20)], require_release=True)
        self.assertFalse(result['grip_detected'])

    def test_fault_and_invalid_targets_send_no_command(self):
        client = RG2Gripper('unused')
        client._rpc = Mock()
        client.status = lambda: state(fault=True)
        with self.assertRaises(GripperError):
            client.move(20, 10)
        client._rpc.rg_grip.assert_not_called()
        for width, force in ((float('nan'), 10), (111, 10), (20, 41), (20, 0)):
            with self.subTest(width=width, force=force), self.assertRaises(ValueError):
                client.move(width, force)
        client._rpc.rg_grip.assert_not_called()

    def test_release_reports_latched_S2_fault_and_sends_no_jaw_command(self):
        client = RG2Gripper('unused')
        client.status = lambda: state(41.6, grip=True, fault=True)
        client._rpc = Mock()
        client._rpc.rg_get_status.return_value = 98
        client._rpc.rg_get_s1_pushed.return_value = False
        client._rpc.rg_get_s1_triggered.return_value = False
        client._rpc.rg_get_s2_pushed.return_value = False
        client._rpc.rg_get_s2_triggered.return_value = True
        with self.assertRaisesRegex(GripperError, 'S2_triggered=True'):
            client.move(100, 10, require_release=True)
        client._rpc.rg_grip.assert_not_called()
        self.assertTrue(client.status()['safety_failed'])

    def test_probe_safety_check_is_bounded_and_never_commands_or_resets(self):
        client = RG2Gripper('unused')
        client._rpc = Mock()
        original_timeout = client._transport.timeout
        def healthy(index):
            self.assertLessEqual(client._transport.timeout, .1)
            return False
        client._rpc.rg_get_safety_failed.side_effect = healthy
        client.check_safety()
        self.assertEqual(client._transport.timeout, original_timeout)
        self.assertEqual([call[0] for call in client._rpc.method_calls], ['rg_get_safety_failed'])
        client._rpc.rg_get_safety_failed.side_effect = TimeoutError('lost reply')
        with self.assertRaises(GripperError):
            client.check_safety()
        self.assertEqual(client._transport.timeout, original_timeout)
        client._rpc.rg_grip.assert_not_called()

    def test_probe_safety_fault_is_not_normal_contact(self):
        client = RG2Gripper('unused')
        client._rpc = Mock()
        client._rpc.rg_get_safety_failed.return_value = True
        with self.assertRaisesRegex(GripperError, 'contact is not accepted'):
            client.check_safety()
        client._rpc.rg_grip.assert_not_called()

    def test_command_transport_does_not_retry(self):
        transport = _Transport(1)
        transport.single_request = Mock(side_effect=ConnectionResetError)
        with patch.object(transport, 'close') as close:
            with self.assertRaises(ConnectionResetError):
                transport.request('unused', '/', b'data')
            self.assertEqual(close.call_count, 2)
        transport.single_request.assert_called_once()

    def test_status_survives_server_closing_keepalive_connections(self):
        class ClosingHandler(SimpleXMLRPCRequestHandler):
            protocol_version = 'HTTP/1.1'

            def do_POST(self):
                super().do_POST()
                # Mimic a daemon closing a cached connection unexpectedly,
                # without advertising Connection: close in its response.
                self.close_connection = True

        class CountingServer(SimpleXMLRPCServer):
            accepted_connections = 0

            def get_request(self):
                request = super().get_request()
                self.accepted_connections += 1
                return request

        server = CountingServer(('127.0.0.1', 0), requestHandler=ClosingHandler,
                                logRequests=False)
        server.register_function(lambda index: 99.6, 'rg_get_width')
        for name in ('rg_get_busy', 'rg_get_grip_detected', 'rg_get_safety_failed'):
            server.register_function(lambda index: False, name)
        jaw_command = Mock(return_value=0)
        server.register_function(jaw_command, 'rg_grip')
        thread = threading.Thread(target=server.serve_forever,
                                  kwargs={'poll_interval': 0.01}, daemon=True)
        thread.start()
        client = RG2Gripper('127.0.0.1', port=server.server_address[1])
        try:
            self.assertEqual(client.status(), state(99.6))
            self.assertEqual(client.status(), state(99.6))
            self.assertEqual(server.accepted_connections, 8)
            jaw_command.assert_not_called()
        finally:
            client.close()
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
