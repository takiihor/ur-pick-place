import http.client
import json
from pathlib import Path
import tempfile
import threading
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
        status, html, _ = self.request('GET', '/')
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
