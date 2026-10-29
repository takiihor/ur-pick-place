"""Separate local Marker Pick Teacher app. No movement/control endpoints."""
import argparse
import errno
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
import json
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import threading
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, build_opener
import webbrowser

from .teaching import TeachingProject, VERSION, defaults, read_json

APP_ID = 'ur-pick-place.marker-pick-teacher'


def existing_teacher(url):
    """Identify our listener without treating another local service as this app."""
    try:
        opener = build_opener(ProxyHandler({}))
        with opener.open(url + '/api/identity', timeout=2) as response:
            identity = json.loads(response.read(8192))
        if isinstance(identity, dict) and identity.get('app_id') == APP_ID:
            return identity
    except (OSError, ValueError):
        pass
    return None


class TeachingService:
    def __init__(self, repo):
        self.repo = Path(repo)
        self.project = None
        self.busy = None
        self.error = ''
        self.completed = None
        self.failed = None
        self.lock = threading.RLock()
        self.process = None
        self.closed = False
        self.prefill = None

    def state(self):
        with self.lock:
            return {'version': VERSION, 'busy': self.busy, 'error': self.error, 'completed': self.completed, 'failed': self.failed,
                    'project': None if self.project is None else self.project.summary()}

    def configure(self, body):
        with self.lock:
            if self.busy:
                raise ValueError('Wait for the current step to finish')
            if body.get('tcp_confirmed') is not True:
                raise ValueError('Confirm the entered TCP offset matches the active gripper TCP on the pendant')
            self.project = TeachingProject(body)
            self.completed, self.error, self.failed = 'configure', '', None
            return self.state()

    def submit(self, action, body):
        if action not in ('capture', 'teach', 'locate'):
            raise ValueError('Unknown teaching step')
        with self.lock:
            if self.closed or self.busy:
                raise ValueError('Teacher is closed or a step is already running')
            if self.project is None:
                raise ValueError('Complete setup first')
            if action == 'teach' and body.get('marker_unchanged') is not True:
                raise ValueError('Confirm the marker/object stayed fixed after capture')
            if action == 'teach' and not self.project.flow['captured']:
                raise ValueError('Capture the marker first')
            if action == 'locate' and not self.project.flow['taught']:
                raise ValueError('Save a picking position first')
            self.project._invalidate_export()
            self.project._persist()
            self.busy, self.error, self.completed, self.failed = action, '', None, None
            settings = dict(self.project.settings, project=str(self.project.directory))
            thread = threading.Thread(target=self._run, args=(action, settings), daemon=True)
            thread.start()

    def _run(self, action, settings):
        try:
            # SDK/network calls run in a bounded child process. Timeout or app exit
            # releases camera/RTDE handles even if a driver call fails to return.
            with self.lock:
                if self.closed:
                    return
                process = self.process = subprocess.Popen(
                    [sys.executable, '-m', 'ur_pick_place.vision.teach_ui', '--worker', action, '--project', settings['project']],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                stdout, stderr = process.communicate(timeout=90)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
                raise ValueError('Step timed out after 90 seconds; connections closed. Check robot/network/camera and retry.')
            if process.returncode:
                raise ValueError(stderr.strip() or stdout.strip() or 'Teaching step failed')
            with self.lock:
                self.project = TeachingProject(settings)
                # Fresh export is only marked ready in the process that produced it.
                if action == 'locate':
                    self.project.exported = read_json(self.project.directory / 'pick_target.json')
                    self.project.flow['exported'] = True
                self.completed = action
        except Exception as exc:
            with self.lock:
                self.error, self.failed = str(exc), action
                try:
                    self.project = TeachingProject(settings)
                except Exception:
                    self.project = None
        finally:
            with self.lock:
                self.busy, self.process = None, None

    def close(self):
        with self.lock:
            self.closed = True
            if self.process is not None and self.process.poll() is None:
                self.process.kill()


class TeachingServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, service):
        self.service = service
        self.token = secrets.token_urlsafe(32)
        super().__init__(address, Handler)
        self.allowed_hosts = {f'127.0.0.1:{self.server_address[1]}', f'localhost:{self.server_address[1]}'}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, value, status=200, mime='application/json; charset=utf-8', filename=None):
        body = json.dumps(value, allow_nan=False).encode() if mime.startswith('application/json') else value
        self.send_response(status)
        for key, val in {'Content-Type': mime, 'Content-Length': str(len(body)), 'Cache-Control': 'no-store',
                         'X-Content-Type-Options': 'nosniff',
                         'Content-Security-Policy': "default-src 'self'; script-src 'self'; style-src 'self'; frame-ancestors 'none'; base-uri 'none'"}.items():
            self.send_header(key, val)
        if filename:
            self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.headers.get('Host') not in self.server.allowed_hosts:
            return self.reply({'error': 'Invalid host'}, 403)
        path = urlsplit(self.path).path
        service = self.server.service
        if path == '/api/identity':
            return self.reply({'app_id': APP_ID, 'version': VERSION})
        if path == '/api/state':
            return self.reply(service.state())
        if path == '/api/defaults':
            value = defaults(service.repo)
            if service.prefill is not None:
                value['settings'] = service.prefill
            return self.reply(value)
        if path == '/api/image':
            with service.lock:
                image = service.project.directory / 'capture.jpg' if service.project else None
                if image is None or not image.exists():
                    return self.reply({'error': 'Capture a view first'}, 404)
                return self.reply(image.read_bytes(), mime='image/jpeg')
        if path.startswith('/api/download/'):
            name = path.rsplit('/', 1)[-1]
            allowed = {'vision.json', 'grasp_pick.json', 'target.json', 'pick_target.json', 'target.script'}
            with service.lock:
                project = service.project
                if (name not in allowed or project is None or service.busy
                        or not project.flow['taught']
                        or name in {'target.json', 'pick_target.json', 'target.script'} and not project.flow['exported']):
                    return self.reply({'error': 'Complete the corresponding step first'}, 409)
                file = project.directory / name
                if name.endswith('.json'):
                    return self.reply(read_json(file), filename=name)
                return self.reply(file.read_bytes(), mime='text/plain; charset=utf-8', filename=name)
        assets = {'/': ('teacher.html', 'text/html; charset=utf-8'),
                  '/teacher.js': ('teacher.js', 'text/javascript; charset=utf-8'),
                  '/teacher.css': ('teacher.css', 'text/css; charset=utf-8')}
        if path not in assets:
            return self.reply({'error': 'Not found'}, 404)
        name, mime = assets[path]
        content = files('ur_pick_place').joinpath('web', name).read_bytes()
        if name.endswith('.html'):
            content = content.replace(b'__TEACH_TOKEN__', self.server.token.encode())
        return self.reply(content, mime=mime)

    def do_POST(self):
        host = self.headers.get('Host')
        if (host not in self.server.allowed_hosts
                or self.headers.get('Origin') not in (None, f'http://{host}')
                or not secrets.compare_digest(self.headers.get('X-Teach-Token', ''), self.server.token)):
            return self.reply({'error': 'Request not authorized for this local UI'}, 403)
        try:
            if self.headers.get_content_type() != 'application/json':
                raise ValueError('Use application/json')
            size = int(self.headers.get('Content-Length', 0))
            if not 0 < size <= 32768:
                raise ValueError('Invalid request size')
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError('Expected a JSON object')
            path = urlsplit(self.path).path
            if path == '/api/open':
                project = Path(body['project']).expanduser().resolve()
                saved = read_json(project / 'project.json')
                return self.reply(dict(saved['settings'], project=str(project)))
            if path == '/api/configure':
                return self.reply(self.server.service.configure(body))
            if path in ('/api/capture', '/api/teach', '/api/locate'):
                self.server.service.submit(path.rsplit('/', 1)[-1], body)
                return self.reply({'accepted': True}, 202)
            return self.reply({'error': 'Not found'}, 404)
        except Exception as exc:
            self.reply({'error': str(exc)}, 400)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--project', type=Path, help='Open a previously saved teaching project')
    parser.add_argument('--worker', choices=('capture', 'teach', 'locate'), help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        try:
            saved = read_json(args.project / 'project.json')
            project = TeachingProject(dict(saved['settings'], project=str(args.project)))
            if args.worker == 'teach':
                project.save_pick(marker_unchanged=True)
            else:
                getattr(project, args.worker)()
            return 0
        except Exception as exc:
            print(str(exc), file=sys.stderr)
            return 1
    if not 0 <= args.port <= 65535:
        parser.error('--port must be 0..65535')
    service = TeachingService(Path(__file__).resolve().parents[3])
    if args.project:
        path = args.project.expanduser().resolve()
        service.prefill = dict(read_json(path / 'project.json')['settings'], project=str(path))
    try:
        server = TeachingServer(('127.0.0.1', args.port), service)
    except OSError as exc:
        service.close()
        if exc.errno != errno.EADDRINUSE:
            parser.exit(1, f'Cannot start Marker Pick Teacher: {exc}\n')
        url = f'http://127.0.0.1:{args.port}'
        identity = existing_teacher(url)
        if identity is None:
            parser.exit(1, f'Port {args.port} is in use by another application. '
                        'Choose another port, for example:\n'
                        './start_marker_teach.sh --port 8767\n')
        print(f'Marker Pick Teacher v{identity.get("version", "unknown")} is already running: {url}', flush=True)
        if identity.get('version') != VERSION:
            print(f'This checkout is v{VERSION}; restart the running app to apply the update.', flush=True)
        if args.project:
            print(f'Use Open saved project in the existing app to open {args.project.expanduser().resolve()}.', flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        return 0
    url = f'http://127.0.0.1:{server.server_address[1]}'
    print(f'Marker Pick Teacher v{VERSION}: {url}\nRead-only camera/RTDE; teach using the pendant.', flush=True)
    if not args.no_browser:
        timer = threading.Timer(.3, webbrowser.open, args=(url,))
        timer.daemon = True
        timer.start()
    previous = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        service.close()
        server.server_close()
        signal.signal(signal.SIGTERM, previous)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
