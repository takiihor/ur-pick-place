import http.client
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

from ur_pick_place.manual import ManualService, ManualSettings
from ur_pick_place.poses import PoseStore
from ur_pick_place.ui import ControlServer


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.service = ManualService(ManualSettings(), PoseStore(Path(self.directory.name)/'poses.json'), demo=True)
        self.server = ControlServer(('127.0.0.1', 0), self.service)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval':.01}, daemon=True)
        self.thread.start()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)
        self.service.close()
        self.directory.cleanup()

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_address[1], timeout=2)
        connection.request(method, path, json.dumps(body) if body is not None else None, headers or {})
        response = connection.getresponse()
        result = response.status, response.read(), response.getheaders()
        connection.close()
        return result

    def test_assets_and_preview_state_work_without_hardware(self):
        for route in ('/', '/waypoints', '/testing'):
            with self.subTest(route=route):
                status, html, _ = self.request('GET', route)
                self.assertEqual(status, 200)
                self.assertIn(self.server.token.encode(), html)
        for path in ('/app.js', '/style.css'):
            self.assertEqual(self.request('GET', path)[0], 200)
        status, state, _ = self.request('GET', '/api/state')
        self.assertFalse(json.loads(state)['connected'])
        self.assertTrue(json.loads(state)['demo'])

    def test_missing_token_cross_origin_and_unknown_host_are_rejected(self):
        self.assertEqual(self.request('POST', '/api/stop', {})[0], 403)
        headers = {'Content-Type':'application/json', 'X-Control-Token':self.server.token, 'Origin':'http://example.com'}
        self.assertEqual(self.request('POST', '/api/stop', {}, headers)[0], 403)
        self.assertEqual(self.request('GET', '/api/state', headers={'Host':'attacker.example'})[0], 403)

    def test_authorized_commands_and_bad_input(self):
        headers = {'Content-Type':'application/json', 'X-Control-Token':self.server.token}
        self.assertEqual(self.request('POST', '/api/heartbeat', {}, headers)[0], 200)
        self.assertEqual(self.request('POST', '/api/poses', {'name':'home'}, headers)[0], 400)
        self.assertEqual(self.request('POST', '/api/connect', {}, headers)[0], 202)

    def test_export_is_portable_json(self):
        status, content, headers = self.request('GET', '/api/poses/export')
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(content), {'version':1, 'poses':[]})
        self.assertIn('Content-Disposition', dict(headers))

    def test_existing_waypoint_can_be_updated_by_name_or_id(self):
        headers = {'Content-Type':'application/json', 'X-Control-Token':self.server.token}
        self.request('POST', '/api/connect', {}, headers)
        deadline = time.monotonic()+2
        while not self.service.state()['connected'] or self.service.state()['pending']:
            if time.monotonic() > deadline:
                self.fail('Timed out waiting for connection')
            time.sleep(.01)
        status, body, _ = self.request('POST', '/api/poses', {'name':'pick'}, headers)
        self.assertEqual(status, 201)
        original = json.loads(body)
        self.service.control.pose[0] += .01
        deadline = time.monotonic()+2
        while self.service.state()['feedback']['tcp_pose_m_rad'][0] == original['tcp_pose_m_rad'][0]:
            if time.monotonic() > deadline:
                self.fail('Timed out waiting for feedback')
            time.sleep(.01)
        status, body, _ = self.request('POST', '/api/poses', {'name':' PICK '}, headers)
        self.assertEqual(status, 201)
        updated = json.loads(body)
        self.assertEqual(updated['id'], original['id'])
        self.assertNotEqual(updated['tcp_pose_m_rad'], original['tcp_pose_m_rad'])
        status, body, _ = self.request('POST', '/api/poses', {'id':original['id']}, headers)
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)['id'], original['id'])
        self.assertEqual(len(self.service.store.list()), 1)
        self.assertEqual(self.request('POST', '/api/poses', {'id':'missing'}, headers)[0], 400)

    def test_held_jog_route_heartbeat_and_stop(self):
        headers = {'Content-Type':'application/json', 'X-Control-Token':self.server.token}

        def wait_until(predicate):
            deadline = time.monotonic()+2
            while not predicate():
                if time.monotonic() > deadline:
                    self.fail('Timed out waiting for worker')
                time.sleep(.01)

        self.request('POST', '/api/connect', {}, headers)
        wait_until(lambda: self.service.state()['connected'] and not self.service.state()['pending'])
        payload = {'axis':'x', 'speed':1, 'jog_id':'http-held'}
        self.assertEqual(self.request('POST', '/api/jog/start', payload, headers)[0], 202)
        wait_until(lambda: self.service.state()['moving'] and not self.service.state()['pending'])
        for _ in range(4):
            self.assertEqual(self.request('POST', '/api/heartbeat', {'jog_id':'http-held'}, headers)[0], 200)
            time.sleep(.2)
        self.assertTrue(self.service.state()['moving'])
        self.assertEqual(self.request('POST', '/api/stop', {}, headers)[0], 202)
        wait_until(lambda: not self.service.state()['moving'])
        self.assertTrue(self.service.state()['connected'])

    def test_sequence_preview_routes_do_not_move_hardware(self):
        headers = {'Content-Type':'application/json', 'X-Control-Token':self.server.token}
        status, response, _ = self.request('POST', '/api/sequence/demo-poses', {}, headers)
        self.assertEqual(status, 200)
        selection = json.loads(response)
        status, response, _ = self.request('POST', '/api/sequence/preview', selection, headers)
        self.assertEqual(status, 200)
        preview = json.loads(response)
        self.assertEqual(preview['gripper'], 'rg2')
        self.assertFalse(self.service.state()['connected'])
        self.assertEqual(self.request('POST', '/api/sequence/run', {'preview_id':preview['id']}, headers)[0], 400)
