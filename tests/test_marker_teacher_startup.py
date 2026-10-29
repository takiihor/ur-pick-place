"""Repeated launch reuses this app without stopping it or unrelated listeners."""
from contextlib import redirect_stderr, redirect_stdout
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import shutil
import subprocess
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

try:
    from ur_pick_place.vision.teach_ui import (APP_ID, TeachingServer, TeachingService, existing_teacher, main)
except ImportError as exc:
    raise unittest.SkipTest(f'vision extra not installed: {exc}') from None


class OtherHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        body = json.dumps({'app_id': 'different-application', 'version': '0.1.0'}).encode()
        self.send_response(200)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class MarkerTeacherStartupTests(unittest.TestCase):
    def start_server(self, server):
        thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .02}, daemon=True)
        thread.start()
        def cleanup():
            server.shutdown()
            thread.join(timeout=2)
            server.server_close()
        self.addCleanup(cleanup)
        return f'http://127.0.0.1:{server.server_address[1]}'

    def teacher(self):
        service = TeachingService(Path.cwd())
        self.addCleanup(service.close)
        server = TeachingServer(('127.0.0.1', 0), service)
        return server, self.start_server(server)

    @unittest.skipUnless(shutil.which('node'), 'Node.js is needed for the frontend state test')
    def test_wizard_navigation_and_step_gates(self):
        script = Path(__file__).with_name('marker_teacher_ui.cjs')
        result = subprocess.run(['node', str(script)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_repeated_headless_launch_reuses_healthy_server(self):
        server, url = self.teacher()
        before = server.service.state()
        output = io.StringIO()
        with patch('ur_pick_place.vision.teach_ui.webbrowser.open') as browser, redirect_stdout(output):
            self.assertEqual(main(['--port', str(server.server_address[1]), '--no-browser']), 0)
        browser.assert_not_called()
        self.assertIn('already running', output.getvalue())
        self.assertIn(url, output.getvalue())
        self.assertEqual(server.service.state(), before)
        self.assertEqual(existing_teacher(url)['app_id'], APP_ID)

    def test_repeated_normal_launch_opens_existing_ui(self):
        server, url = self.teacher()
        with patch('ur_pick_place.vision.teach_ui.webbrowser.open') as browser, redirect_stdout(io.StringIO()):
            self.assertEqual(main(['--port', str(server.server_address[1])]), 0)
        browser.assert_called_once_with(url)
        self.assertFalse(server.service.closed)

    def test_different_application_gets_clear_error_and_is_not_stopped(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), OtherHandler)
        url = self.start_server(server)
        error = io.StringIO()
        with redirect_stderr(error), self.assertRaises(SystemExit) as caught:
            main(['--port', str(server.server_address[1]), '--no-browser'])
        self.assertEqual(caught.exception.code, 1)
        self.assertIn('another application', error.getvalue())
        self.assertIn('--port 8767', error.getvalue())
        self.assertNotIn('Traceback', error.getvalue())
        self.assertIsNone(existing_teacher(url))  # unrelated server is still responding

    def test_project_argument_does_not_replace_existing_work(self):
        server, url = self.teacher()
        with tempfile.TemporaryDirectory() as directory:
            project = Path(directory)
            (project / 'project.json').write_text(json.dumps({'settings': {}}))
            output = io.StringIO()
            with redirect_stdout(output):
                self.assertEqual(main(['--port', str(server.server_address[1]), '--no-browser',
                                       '--project', str(project)]), 0)
            self.assertIn('Open saved project', output.getvalue())
            self.assertIsNone(server.service.project)
            self.assertEqual(existing_teacher(url)['app_id'], APP_ID)


if __name__ == '__main__':
    unittest.main()
