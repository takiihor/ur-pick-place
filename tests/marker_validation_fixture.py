"""Isolated test server: real teacher HTTP/workers, rendered markers, fake RTDE.

All paths live in a temporary directory. This file never opens hardware interfaces.
"""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from unittest.mock import patch
from urllib.parse import urlsplit

from vision_sim import MARKER_TCP, OBSERVE_PICK, TCP_OFFSET, calibration_dict, true_flange
import numpy as np
from test_vision_pipeline import FakeRuntime, World
from test_marker_validation import ValidationRobot
from ur_pick_place.poses import atomic_json
from ur_pick_place.vision import teach_ui
from ur_pick_place.vision.teaching import digest, read_json
from ur_pick_place.vision.transforms import invert, matrix_to_pose, pose_to_matrix


def run_worker(project_path, action):
    world = World()
    scene = read_json(project_path.parent/'scene.json')
    world.pick = np.asarray(scene['marker'], float)
    world.camera.scene[0] = (world.camera.scene[0][0], world.pick)
    world.rtde.q = list(OBSERVE_PICK)
    world.rtde.flange = (world.pick @ MARKER_TCP @ invert(pose_to_matrix(TCP_OFFSET))
                        if action == 'teach' else true_flange(OBSERVE_PICK))
    runtime = FakeRuntime(world.rtde, world.camera)
    with patch('ur_pick_place.vision.teaching.Runtime', return_value=runtime), \
            patch('ur_pick_place.vision.robot_state.nominal_flange', return_value=world.rtde.flange):
        return teach_ui.main(['--worker', action, '--project', str(project_path)])


class FixtureService(teach_ui.TeachingService):
    def __init__(self, root):
        super().__init__(root, motion_factory=self.connect_simulation)
        self.mode = 'normal'
        self.moves = []

    def connect_simulation(self, host):
        assert host == 'simulated-robot', 'Test fixture refuses hardware addresses'
        record = self.project.validation
        robot = ValidationRobot()
        robot.pose = list(record['capture_robot']['tcp_pose_m_rad'])
        robot.joints = list(record['capture_robot']['joints_rad'])
        robot._duration = 1 if self.mode == 'normal' else 10
        if self.mode == 'wrong_tcp':
            robot.tcp = [0]*6
        original_move = robot.moveL

        def move(*args):
            self.moves.append(list(args[0]))
            return original_move(*args)

        robot.moveL = move
        return robot, robot


class FixtureHandler(teach_ui.Handler):
    def do_GET(self):
        if urlsplit(self.path).path == '/__test__/metrics':
            project = self.server.service.project
            return self.reply({'moves': self.server.service.moves,
                'reference_sha256': digest(project.directory/'observation.json'),
                'grasp_sha256': digest(project.directory/'grasp_pick.json'),
                'target_sha256': digest(project.directory/'target.json')})
        return super().do_GET()

    def do_POST(self):
        if urlsplit(self.path).path == '/__test__/scene':
            if self.headers.get('X-Teach-Token') != self.server.token:
                return self.reply({'error': 'Invalid fixture token'}, 403)
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            world = World()
            world.move_pick(body.get('dx_mm', 0), body.get('dy_mm', 0), body.get('yaw_deg', 0))
            service = self.server.service
            service.mode = body.get('mode', 'normal')
            if body.get('expire_capture'):
                service.project.validation['timestamp'] = time.time()-61
            atomic_json(service.repo/'scene.json', {'marker': world.pick.tolist()})
            expected_pick = world.pick @ MARKER_TCP
            expected_approach = matrix_to_pose(expected_pick)
            expected_approach[2] += .1
            return self.reply({'expected_pick': matrix_to_pose(expected_pick), 'expected_approach': expected_approach})
        return super().do_POST()


def serve():
    real_popen = subprocess.Popen

    def simulated_worker(command, **kwargs):
        assert command[1:3] == ['-m', 'ur_pick_place.vision.teach_ui'], command
        return real_popen([sys.executable, str(Path(__file__).resolve()), *command[3:]], **kwargs)

    with tempfile.TemporaryDirectory(prefix='marker-validation-browser-') as temporary:
        root = Path(temporary)
        (root/'calibration.json').write_text(json.dumps(calibration_dict()))
        atomic_json(root/'scene.json', {'marker': World().pick.tolist()})
        settings = dict(project=str(root/'project'), calibration=str(root/'calibration.json'),
            robot_ip='simulated-robot', tcp=list(TCP_OFFSET), motion_model='planar', approach_mm=100,
            target=dict(kind='aruco', marker_id=7, dictionary='DICT_5X5_100', marker_length_mm=30))
        service = FixtureService(root)
        service.prefill = settings
        server = teach_ui.TeachingServer(('127.0.0.1', 0), service)
        server.RequestHandlerClass = FixtureHandler
        signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
        print(json.dumps({'url': f'http://127.0.0.1:{server.server_address[1]}'}), flush=True)
        try:
            with patch('ur_pick_place.vision.teach_ui.subprocess.Popen', side_effect=simulated_worker):
                server.serve_forever(poll_interval=.05)
        except KeyboardInterrupt:
            pass
        finally:
            service.close()
            server.server_close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker')
    parser.add_argument('--project', type=Path)
    args = parser.parse_args()
    raise SystemExit(run_worker(args.project, args.worker) if args.worker else serve())
