"""Read-only marker/pick teaching. Hardware access is receive + colour camera only.

A project stores a marker observation, a marker-relative TCP endpoint and reusable
vision.json. Exports are snapshots, never authorization for motion or force mode.
"""
from contextlib import contextmanager
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from ..config import number, vector
from ..poses import atomic_json
from .cli import Runtime, _close
from .config import Limits, VisionConfig
from .errors import VisionError
from .grasp import load_taught, target_pose, teach
from .handeye import load_handeye
from .observe import Observation, Observer, check_camera_identity
from .robot_state import ReceivePoseSource, require_stationary
from .targets import same_target, target_from_config
from .transforms import matrix_to_pose

VERSION = '0.1.0'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def defaults(repo):
    """Discover local inputs; no robot or camera is opened."""
    repo = Path(repo).resolve()
    calibrations = []
    for path in sorted((repo.parent / 'ur10e-handeye' / 'sessions').glob('*/calibration.json'), reverse=True):
        try:
            calibration = load_handeye(path)
            if calibration.simulated:
                continue
            raw = read_json(path)
            calibrations.append({'path': str(path), 'tcp': calibration.tcp_offset,
                                 'robot_ip': raw.get('robot', {}).get('host', '')})
        except (ValueError, OSError, VisionError):
            continue
    manifests = [str(p) for p in sorted((repo.parent / 'charuco-generator' / 'output').rglob('board_manifest.json'))]
    stage2 = next((p for p in manifests if '/calibration_stage2/' in p), manifests[0] if manifests else '')
    return {'calibrations': calibrations, 'manifests': manifests,
            'settings': {'project': str(repo / 'data' / 'marker-teaching' / datetime.now().strftime('%Y%m%d-%H%M%S')),
                         'calibration': calibrations[0]['path'] if calibrations else '',
                         'robot_ip': calibrations[0]['robot_ip'] if calibrations else '',
                         'tcp': calibrations[0]['tcp'] if calibrations else [0.0] * 6,
                         'target': {'kind': 'charuco', 'board_manifest': stage2, 'board': 'calibration_stage2'},
                         'motion_model': 'planar', 'approach_mm': 100.0, 'limits': asdict(Limits())}}


class PreviewCamera:
    def __init__(self, camera):
        self.camera, self.last = camera, None

    def read(self):
        frame = self.camera.read()
        self.last = frame.bgr.copy()
        return frame


class TeachingProject:
    def __init__(self, settings, *, runtime=None):
        self.runtime = runtime or Runtime()
        self.directory = Path(settings['project']).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        raw = {k: settings[k] for k in ('calibration', 'robot_ip', 'tcp', 'target', 'motion_model', 'approach_mm')}
        raw['calibration'] = str(Path(raw['calibration']).expanduser().resolve())
        raw['limits'] = settings.get('limits', asdict(Limits()))
        raw['tcp'] = vector(raw['tcp'], 6, 'Active gripper TCP offset (m/rad)')
        raw['approach_mm'] = number(raw['approach_mm'], 'Approach height (mm)', low=1, high=300)
        if not isinstance(raw['robot_ip'], str) or not raw['robot_ip'].strip() or any(c.isspace() for c in raw['robot_ip']):
            raise ValueError('Enter a robot IP address or hostname without spaces')
        if raw['target'].get('kind') == 'charuco':
            raw['target'] = dict(raw['target'], board_manifest=str(Path(raw['target']['board_manifest']).expanduser().resolve()))
        self.settings = raw
        self.calibration = load_handeye(raw['calibration'])
        self.calibration.require_hardware()
        self.target = target_from_config(raw['target'])
        # This app deliberately reuses the existing read-only TCP checks. Never infer
        # an active gripper TCP from the camera transform.
        from .robot_state import same_tcp
        if not same_tcp(raw['tcp'], self.calibration.tcp_offset):
            raise ValueError('This read-only workflow requires the active gripper TCP to match the TCP recorded '
                             'in calibration.json. Activate that TCP or make a calibration with your gripper TCP.')
        stream = read_json(raw['calibration']).get('camera', {}).get('stream', {})
        self.config = VisionConfig(handeye_calibration=raw['calibration'],
            camera={'serial': self.calibration.camera_serial, 'width': self.calibration.intrinsics['width'],
                    'height': self.calibration.intrinsics['height'], 'fps': stream.get('fps', 15)},
            robot={'active_tcp_offset': raw['tcp']},
            pick={'target': raw['target'], 'observe_joints': [0.0] * 6, 'grasp': 'grasp_pick.json',
                  'motion_model': raw['motion_model'], 'limits': raw['limits']}, base_dir=str(self.directory))
        self.inputs = {'calibration': self.calibration.sha256}
        if self.target.kind == 'charuco':
            self.inputs['manifest'] = digest(raw['target']['board_manifest'])
        self.flow = {'captured': False, 'taught': False, 'exported': False}
        self.observation = self.taught = self.exported = None
        path = self.directory / 'project.json'
        if path.exists():
            saved = read_json(path)
            if saved.get('schema_version') != 1 or saved.get('type') != 'marker_pick_project':
                raise ValueError('Unsupported teaching project')
            if saved['settings'] != raw or saved['inputs'] != self.inputs:
                raise ValueError('This folder belongs to different settings or changed input files. '
                                 'Choose a new project folder to teach again.')
            self.flow = saved['flow']
            if self.flow['captured']:
                self.observation = Observation.from_json(read_json(self.directory / 'observation.json'))
                if self.observation.calibration_sha256 != self.calibration.sha256 or not same_target(self.observation.target, self.target):
                    raise ValueError('Stored observation does not match the project inputs')
                self.config.pick.observe_joints = list(self.observation.robot['joints_rad'])
            if self.flow['taught']:
                self.taught = read_json(self.directory / 'grasp_pick.json')
                load_taught(self.directory / 'grasp_pick.json')
            # Exported targets are deliberately not restored as fresh/ready after restart.
            self.flow['exported'] = False
        self._persist()

    def _persist(self):
        config = asdict(self.config)
        config.pop('base_dir')
        atomic_json(self.directory / 'vision.json', config)
        atomic_json(self.directory / 'project.json', {'schema_version': 1, 'type': 'marker_pick_project',
                    'software_version': VERSION, 'settings': self.settings, 'inputs': self.inputs, 'flow': self.flow})

    def check_inputs(self):
        if digest(self.settings['calibration']) != self.inputs['calibration']:
            raise ValueError('Calibration changed. Start a new teaching project.')
        if 'manifest' in self.inputs and digest(self.settings['target']['board_manifest']) != self.inputs['manifest']:
            raise ValueError('Board manifest changed. Start a new teaching project.')

    @contextmanager
    def receiver(self):
        self.check_inputs()
        receive = self.runtime.receive(self.settings['robot_ip'])
        try:
            source = ReceivePoseSource(receive, self.calibration, self.settings['tcp'], self.config.robot)
            yield source
        finally:
            _close(receive)

    def _look(self):
        with self.receiver() as source:
            camera = self.runtime.camera(self.config.camera)
            preview = PreviewCamera(camera)
            observation = None
            try:
                check_camera_identity(camera, self.calibration)
                observation = Observer(self.config, self.calibration, preview, source).look('pick', allow_motion=False)
                return observation
            finally:
                _close(camera)
                if preview.last is not None:
                    image = preview.last
                    if observation is not None:
                        from .detection import camera_matrix
                        k, distortion = camera_matrix(self.calibration.intrinsics)
                        pose = matrix_to_pose(observation.camera_target)
                        cv2.drawFrameAxes(image, k, distortion, np.asarray(pose[3:]), np.asarray(pose[:3]), 0.03)
                    ok, encoded = cv2.imencode('.jpg', image)
                    if ok:
                        (self.directory / 'capture.jpg').write_bytes(encoded.tobytes())

    def _invalidate_export(self):
        self.flow['exported'] = False
        self.exported = None
        for name in ('target.json', 'pick_target.json', 'target.script'):
            (self.directory / name).unlink(missing_ok=True)

    def _archive_grasp(self):
        grasp = self.directory / 'grasp_pick.json'
        if grasp.exists():
            archive = self.directory / 'history'
            archive.mkdir(exist_ok=True)
            grasp.replace(archive / (str(time.time_ns()) + '-grasp_pick.json'))

    def capture(self):
        self._invalidate_export()
        self._archive_grasp()
        # Re-capturing replaces the teaching reference and invalidates its taught offset.
        self.flow = {'captured': False, 'taught': False, 'exported': False}
        self.observation = self.taught = self.exported = None
        self._persist()
        observation = self._look()
        self.check_inputs()
        self.observation = observation
        self.config.pick.observe_joints = list(observation.robot['joints_rad'])
        atomic_json(self.directory / 'observation.json', observation.to_json())
        self.flow['captured'] = True
        self._persist()
        return observation.summary()

    def save_pick(self, *, marker_unchanged=False):
        if self.observation is None or not self.flow['captured']:
            raise ValueError('Capture the marker first')
        if marker_unchanged is not True:
            raise ValueError('Confirm that the marker/object stayed fixed after capture')
        self._invalidate_export()
        self._archive_grasp()
        self.flow['taught'] = False
        self.taught = self.exported = None
        self._persist()
        with self.receiver() as source:
            before = source.snapshot()
            time.sleep(max(0.25, self.config.robot.settle_s))
            after = source.snapshot()
            require_stationary(before, after, self.config.robot.stationary_joint_rad)
            record = teach(self.observation, after.tcp_pose, role='pick', calibration=self.calibration,
                           limits=self.config.pick.limits, tcp_offset=after.tcp_offset)
        record['taught_tcp_source'] = 'Marker Pick Teacher: stationary RTDE receive'
        atomic_json(self.directory / 'grasp_pick.json', record)
        self.taught = record
        self.flow['taught'] = True
        self._persist()
        return record

    def locate(self):
        if not self.flow['taught']:
            raise ValueError('Save a picking position first')
        self._invalidate_export()
        self._persist()
        observation = self._look()
        self.check_inputs()
        model = load_taught(self.directory / 'grasp_pick.json')
        pose, report = target_pose(model, observation, self.config.pick,
            calibration_sha256=self.calibration.sha256, tcp_offset=observation.robot['tcp_offset'],
            target=self.target, strict=True)
        pick = matrix_to_pose(pose)
        approach = list(pick)
        approach[2] += self.settings['approach_mm'] / 1000.0  # BASE +Z, not marker or tool Z
        exported = {'schema_version': 1, 'type': 'marker_pick_target', 'software_version': VERSION,
                    'created_utc': datetime.now(timezone.utc).isoformat(), 'timestamp': time.time(),
                    'frame': 'robot_base', 'units': 'metres and UR rotation-vector radians',
                    'robot_ip': self.settings['robot_ip'], 'tcp_offset': list(self.settings['tcp']),
                    'calibration_sha256': self.calibration.sha256, 'target': observation.target,
                    'pick_tcp_pose_m_rad': pick, 'approach_tcp_pose_m_rad': approach,
                    'approach_axis': 'base +Z', 'approach_mm': self.settings['approach_mm'],
                    'descent_axis': 'base -Z', 'quality': observation.quality, 'validation': report,
                    'purpose': 'target snapshot only; executor must check freshness, TCP, reachability, '
                               'workspace, force parameters, travel/time bounds and grip feedback'}
        # target.json is the existing Robot Bench / CLI six-number interchange format.
        atomic_json(self.directory / 'target.json', pick)
        atomic_json(self.directory / 'pick_target.json', exported)
        atomic_json(self.directory / 'latest_observation.json', observation.to_json())
        (self.directory / 'target.script').write_text(self._script(pick, approach))
        self.exported = exported
        self.flow['exported'] = True
        self._persist()
        return exported

    @staticmethod
    def _script(pick, approach):
        def ur(pose):
            return 'p[' + ','.join(f'{v:.12g}' for v in pose) + ']'
        return ('# Marker Pick Teacher: values only; this file commands NO motion.\n'
                '# Base frame, metres / rotation-vector radians. Re-read marker each cycle.\n'
                '# Confirm the active gripper TCP matches the teaching project.\n'
                f'global vision_pick = {ur(pick)}\n'
                f'global vision_approach = {ur(approach)}\n'
                '# Executor: approach -> bounded base -Z force search -> end_force_mode()\n'
                '# -> grip/check -> lift from the ACTUAL contact pose.\n')

    def summary(self):
        return {'project': str(self.directory), 'settings': self.settings, 'flow': self.flow,
                'calibration': dict(self.calibration.summary(), camera_serial=self.calibration.camera_serial), 'target': self.target.label,
                'observation': None if self.observation is None else self.observation.summary(),
                'taught': self.taught, 'exported': self.exported,
                'observe_joints': self.config.pick.observe_joints,
                'files': ['project.json', 'vision.json', 'observation.json', 'grasp_pick.json',
                          'target.json', 'pick_target.json', 'target.script']}
