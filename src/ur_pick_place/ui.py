"""Local browser UI and JSON backend for RTDE teaching/manual movement."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
import json
from pathlib import Path
import secrets
import signal
import threading
from urllib.parse import urlsplit
import webbrowser

from .config import Recipe, construct
from .manual import ManualService, ManualSettings
from .poses import PoseStore


class ControlServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, service):
        self.service = service
        self.token = secrets.token_urlsafe(32)
        super().__init__(address, Handler)
        port = self.server_address[1]
        self.allowed_hosts = {f'127.0.0.1:{port}', f'localhost:{port}'}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _host_ok(self):
        return self.headers.get('Host') in self.server.allowed_hosts

    def _reply(self, value, status=200, content_type='application/json; charset=utf-8', download=False):
        body = json.dumps(value, allow_nan=False).encode() if content_type.startswith('application/json') else value
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
        if download:
            self.send_header('Content-Disposition', 'attachment; filename="poses.json"')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if not self._host_ok():
            return self._reply({'error': 'Invalid host'}, 403)
        path = urlsplit(self.path).path
        if path == '/api/state':
            return self._reply(self.server.service.state())
        if path == '/api/poses/export':
            return self._reply(dict(version=1, poses=self.server.service.store.list()), download=True)
        assets = {'/': ('index.html', 'text/html; charset=utf-8'),
                  '/waypoints': ('index.html', 'text/html; charset=utf-8'),
                  '/waypoints/': ('index.html', 'text/html; charset=utf-8'),
                  '/testing': ('index.html', 'text/html; charset=utf-8'),
                  '/testing/': ('index.html', 'text/html; charset=utf-8'),
                  '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                  '/style.css': ('style.css', 'text/css; charset=utf-8')}
        if path not in assets:
            return self._reply({'error': 'Not found'}, 404)
        name, mime = assets[path]
        body = files('ur_pick_place').joinpath('web', name).read_bytes()
        if name == 'index.html':
            body = body.replace(b'__CONTROL_TOKEN__', self.server.token.encode())
        self._reply(body, content_type=mime)

    def do_POST(self):
        host = self.headers.get('Host')
        origin = self.headers.get('Origin')
        token = self.headers.get('X-Control-Token', '')
        if (not self._host_ok() or origin is not None and origin != f'http://{host}'
                or not secrets.compare_digest(token, self.server.token)):
            return self._reply({'error': 'Request not authorized for this local UI'}, 403)
        if self.headers.get_content_type() != 'application/json':
            return self._reply({'error': 'Use application/json'}, 415)
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 32768:
                raise ValueError('Request size is invalid')
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError('Expected a JSON object')
            service = self.server.service
            path = urlsplit(self.path).path
            commands = {'/api/connect': 'connect', '/api/disconnect': 'disconnect',
                        '/api/jog': 'jog', '/api/jog/start': 'jog_start',
                        '/api/move-tcp': 'move_tcp', '/api/move-saved': 'move_saved',
                        '/api/sequence/run': 'run_sequence'}
            if path in commands:
                service.submit(commands[path], body)
                return self._reply({'accepted': True}, 202)
            if path == '/api/stop':
                service.stop()
                return self._reply({'accepted': True}, 202)
            if path == '/api/heartbeat':
                service.heartbeat(body.get('jog_id'))
                return self._reply({'ok': True})
            if path == '/api/poses':
                return self._reply(service.record(body.get('name'), ident=body.get('id')), 200 if body.get('id') else 201)
            if path == '/api/poses/rename':
                return self._reply(service.store.rename(body.get('id'), body.get('name')))
            if path == '/api/settings':
                return self._reply(service.update_settings(body))
            if path == '/api/sequence/preview':
                return self._reply(service.preview_sequence(body))
            if path == '/api/sequence/demo-poses':
                return self._reply(service.seed_demo_poses())
            return self._reply({'error': 'Not found'}, 404)
        except (ValueError, TypeError) as exc:
            self._reply({'error': str(exc)}, 400)
        except Exception as exc:
            self._reply({'error': str(exc)}, 500)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765, help='Local HTTP port (default 8765)')
    parser.add_argument('--config', type=Path, help='Initialize IP/workspace/motion limits from a pick/place recipe')
    parser.add_argument('--data-dir', type=Path, help='Settings and named pose storage directory')
    parser.add_argument('--demo', action='store_true', help='Simulate the robot; never connect hardware')
    parser.add_argument('--no-browser', action='store_true', help='Print URL without opening a browser')
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error('--port must be 0..65535')
    directory = args.data_dir or Path('data') / ('demo' if args.demo else 'live')
    settings_path = directory / 'settings.json'
    try:
        recipe = None
        if args.config:
            recipe = Recipe.load(args.config)
            settings = ManualSettings(recipe.robot_ip, recipe.workspace_min_m, recipe.workspace_max_m,
                                      recipe.motion.transit_speed_mm_s, recipe.motion.tcp_accel_m_s2,
                                      recipe.motion.joint_speed_rad_s, recipe.motion.joint_accel_rad_s2)
        elif settings_path.exists():
            settings = construct(ManualSettings, json.loads(settings_path.read_text()))
        else:
            settings = ManualSettings()
        service = ManualService(settings, PoseStore(directory / 'poses.json'), demo=args.demo,
                                settings_path=settings_path, recipe=recipe,
                                sequence_path=directory / 'sequence.json')
        try:
            server = ControlServer(('127.0.0.1', args.port), service)
        except BaseException:
            service.close()
            raise
        url = f'http://127.0.0.1:{server.server_address[1]}'
        print(f'Robot Bench: {url} ({"DEMO — no hardware" if args.demo else "RTDE — connect from the UI"})', flush=True)
        print(f'Pose storage: {(directory / "poses.json").resolve()}', flush=True)
        if not args.no_browser:
            timer = threading.Timer(.3, webbrowser.open, args=(url,))
            timer.daemon = True
            timer.start()
        previous_term = signal.getsignal(signal.SIGTERM)
        def terminate(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, terminate)
        try:
            server.serve_forever(poll_interval=.2)
        except KeyboardInterrupt:
            pass
        finally:
            signal.signal(signal.SIGTERM, previous_term)
            server.server_close()
            service.close()
        return 0
    except Exception as exc:
        parser.exit(1, f'Backend error: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
