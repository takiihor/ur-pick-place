"""One worker owns RTDE; HTTP threads only submit commands/read snapshots."""
from dataclasses import asdict, dataclass, field
import math
from pathlib import Path
import queue
import threading
import time

from .config import construct, number, vector
from .geometry import rotate_base
from .poses import atomic_json


@dataclass
class ManualSettings:
    robot_ip: str = '192.168.0.10'
    workspace_min_m: list = field(default_factory=lambda: [-.9, -.9, .02])
    workspace_max_m: list = field(default_factory=lambda: [.9, .9, 1.2])
    tcp_speed_mm_s: float = 20.0
    tcp_accel_m_s2: float = .1
    joint_speed_rad_s: float = .1
    joint_accel_rad_s2: float = .2

    def __post_init__(self):
        if (not isinstance(self.robot_ip, str) or not self.robot_ip.strip()
                or len(self.robot_ip) > 253 or any(c.isspace() for c in self.robot_ip)):
            raise ValueError('Enter a valid robot IP address or hostname')
        self.robot_ip = self.robot_ip.strip()
        self.workspace_min_m = vector(self.workspace_min_m, 3, 'Workspace minimum')
        self.workspace_max_m = vector(self.workspace_max_m, 3, 'Workspace maximum')
        if any(a >= b for a, b in zip(self.workspace_min_m, self.workspace_max_m)):
            raise ValueError('Each workspace minimum must be below its maximum')
        number(self.tcp_speed_mm_s, 'TCP speed', positive=True, high=100)
        number(self.tcp_accel_m_s2, 'TCP acceleration', positive=True, high=1)
        number(self.joint_speed_rad_s, 'Joint speed', positive=True, high=.4)
        number(self.joint_accel_rad_s2, 'Joint acceleration', positive=True, high=.4)


def live_backend(host):
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
    control = RTDEControlInterface(host, 125.0)
    try:
        return control, RTDEReceiveInterface(host, 125.0)
    except BaseException:
        try:
            control.stopScript()
        finally:
            control.disconnect()
        raise


class ManualService:
    def __init__(self, settings, store, *, demo=False, settings_path=None, factory=None):
        self.settings, self.store, self.demo = settings, store, demo
        self.settings_path = Path(settings_path) if settings_path else None
        if factory is not None:
            self.factory = factory
        elif demo:
            from .demo import DemoRobot
            self.factory = lambda host: (lambda robot: (robot, robot))(DemoRobot())
        else:
            self.factory = live_backend
        self._lock = threading.RLock()
        self._queue = queue.Queue(maxsize=1)
        self._shutdown = threading.Event()
        self._stop_requested = threading.Event()
        self._pending = None
        self._motion = None
        self._connected = False
        self._feedback = None
        self._error = None
        self._message = 'Ready to connect'
        self._generation = 0
        self._heartbeat_at = time.monotonic()
        self._last_timestamp = None
        self._fresh_at = time.monotonic()
        self.control = self.receive = None
        self._worker = threading.Thread(target=self._run, name='rtde-ui-worker', daemon=True)
        self._worker.start()

    def state(self):
        with self._lock:
            result = dict(connected=self._connected, demo=self.demo,
                          pending=self._pending, moving=self._motion is not None,
                          message=self._message, error=self._error,
                          settings=asdict(self.settings), feedback=None)
            if self._feedback:
                result['feedback'] = dict(self._feedback)
                result['feedback']['age_s'] = time.monotonic() - result['feedback'].pop('received_at')
            result['poses'] = self.store.list()
            return result

    def heartbeat(self):
        with self._lock:
            self._heartbeat_at = time.monotonic()

    def stop(self):
        with self._lock:
            self._generation += 1  # Cancel any movement queued before this stop.
            self._message = 'Stop requested'
            self._stop_requested.set()

    def submit(self, command, payload=None):
        payload = {} if payload is None else payload
        if not isinstance(payload, dict):
            raise ValueError('Command must be a JSON object')
        if command not in ('connect', 'disconnect', 'jog', 'move_tcp', 'move_saved'):
            raise ValueError('Unknown robot command')
        with self._lock:
            if self._shutdown.is_set():
                raise ValueError('Backend is shutting down')
            if self._pending:
                raise ValueError('Wait for the current command to finish')
            if command == 'connect':
                if self._connected:
                    raise ValueError('Disconnect before changing the robot connection')
                # Validate before reserving the worker.
                proposed = construct(ManualSettings, dict(asdict(self.settings), robot_ip=payload.get('robot_ip', self.settings.robot_ip)))
                payload = {'robot_ip': proposed.robot_ip}
            elif not self._connected:
                raise ValueError('Connect to the robot first')
            elif command != 'disconnect' and self._motion:
                raise ValueError('A move is already running')
            self._error = None
            self._pending = command
            self._queue.put_nowait((command, payload, self._generation))

    def update_settings(self, values):
        with self._lock:
            if self._connected or self._pending:
                raise ValueError('Disconnect before editing connection and motion limits')
            updated = construct(ManualSettings, values)
            if self.settings_path:
                atomic_json(self.settings_path, asdict(updated))
            self.settings = updated
            return asdict(updated)

    def record(self, name):
        with self._lock:
            feedback = self._feedback
            if (not self._connected or self._pending or self._motion or not feedback
                    or time.monotonic() - feedback['received_at'] > .5):
                raise ValueError('Recording requires a connected, stationary robot with fresh feedback')
            return self.store.save(name, feedback['tcp_pose_m_rad'], feedback['joints_rad'],
                                   robot_ip=self.settings.robot_ip, demo=self.demo)

    def _workspace(self, pose):
        pose = vector(pose, 6, 'TCP target')
        if any(not a <= v <= b for a, v, b in zip(self.settings.workspace_min_m, pose[:3], self.settings.workspace_max_m)):
            raise ValueError('Target is outside the configured XYZ workspace')
        return pose

    def _read(self):
        if not self.control.isConnected() or not self.receive.isConnected():
            raise RuntimeError('RTDE connection lost')
        stamp = number(self.receive.getTimestamp(), 'RTDE timestamp')
        now = time.monotonic()
        if stamp != self._last_timestamp:
            self._last_timestamp, self._fresh_at = stamp, now
        elif now - self._fresh_at > .5:
            raise RuntimeError('RTDE feedback is stale')
        feedback = dict(tcp_pose_m_rad=vector(self.receive.getActualTCPPose(), 6, 'TCP feedback'),
                        joints_rad=vector(self.receive.getActualQ(), 6, 'Joint feedback'),
                        tcp_force_n_nm=vector(self.receive.getActualTCPForce(), 6, 'Force feedback'),
                        safety_mode=int(self.receive.getSafetyMode()), received_at=now)
        with self._lock:
            self._feedback = feedback
        return feedback

    def _execute(self, command, payload, generation):
        if command == 'connect':
            self._message = 'Connecting to RTDE…'
            self.settings.robot_ip = payload['robot_ip']
            self.control, self.receive = self.factory(payload['robot_ip'])
            self._last_timestamp = None
            self._fresh_at = time.monotonic()
            try:
                self._read()
            except BaseException:
                self._disconnect()
                raise
            with self._lock:
                self._connected = True
                self._message = 'Connected to demo robot' if self.demo else 'RTDE connected'
            if self.settings_path:
                atomic_json(self.settings_path, asdict(self.settings))
            return
        if command == 'disconnect':
            self._disconnect()
            return
        with self._lock:
            if generation != self._generation:
                self._message = 'Queued movement cancelled'
                return
            if self._stop_requested.is_set():
                raise ValueError('Stop is pending')
        feedback = self._read()
        if feedback['safety_mode'] not in (1, 2):
            raise ValueError('Robot must be in normal or reduced safety mode to move')
        # Starting outside the configured envelope is visible, but cannot be
        # bypassed by asking for a target inside it.
        self._workspace(feedback['tcp_pose_m_rad'])
        mode = 'tcp'
        if command == 'jog':
            axes = ('x', 'y', 'z', 'rx', 'ry', 'rz')
            axis = payload.get('axis')
            if axis not in axes:
                raise ValueError('Choose a valid jog axis')
            step = number(payload.get('step'), 'Jog increment', low=-10, high=10)
            if step == 0:
                raise ValueError('Jog increment cannot be zero')
            target = list(feedback['tcp_pose_m_rad'])
            index = axes.index(axis)
            if index < 3:
                target[index] += step / 1000
            else:
                target = rotate_base(target, index-3, step)
        elif command == 'move_tcp':
            values = vector(payload.get('pose_mm_deg'), 6, 'Target [mm, degrees]')
            target = [v/1000 for v in values[:3]] + [math.radians(v) for v in values[3:]]
        else:
            saved = self.store.get(payload.get('id'))
            if bool(saved.get('demo')) != self.demo:
                raise ValueError('Demo and live poses cannot be mixed')
            mode = payload.get('mode')
            if mode not in ('tcp', 'joint'):
                raise ValueError('Saved move mode must be tcp or joint')
            target = saved['tcp_pose_m_rad'] if mode == 'tcp' else saved['joints_rad']
        if mode == 'tcp':
            self._workspace(target)
            if self.control.isPoseWithinSafetyLimits(target) is not True:
                raise ValueError('TCP target is unreachable or outside robot safety limits')
        else:
            if self.control.isJointsWithinSafetyLimits(target) is not True:
                raise ValueError('Joint target is outside robot safety limits')
            self._workspace(self.control.getForwardKinematics(target))
        before = self.control.getAsyncOperationProgress()
        # Reserve motion before sending, so a lost acknowledgement still stops.
        with self._lock:
            self._motion = dict(mode=mode, before=before, started=False,
                                deadline=time.monotonic()+120)
            self._message = 'Moving TCP' if mode == 'tcp' else 'Moving saved joint pose'
        c = self.settings
        if mode == 'tcp':
            accepted = self.control.moveL(target, c.tcp_speed_mm_s/1000, c.tcp_accel_m_s2, True)
        else:
            accepted = self.control.moveJ(target, c.joint_speed_rad_s, c.joint_accel_rad_s2, True)
        if accepted is not True:
            raise RuntimeError('Robot rejected the movement')

    def _stop_motion(self):
        with self._lock:
            motion = self._motion
        if motion and self.control:
            if motion['mode'] == 'joint':
                self.control.stopJ(1.0)
            else:
                self.control.stopL(1.0)
            with self._lock:
                self._motion = None
                self._message = 'Movement stopped'

    def _disconnect(self):
        errors = []
        for operation in ([self._stop_motion, self.control.stopScript, self.control.disconnect] if self.control else []):
            try:
                operation()
            except Exception as exc:
                errors.append(str(exc))
        if self.receive:
            try:
                self.receive.disconnect()
            except Exception as exc:
                errors.append(str(exc))
        self.control = self.receive = None
        with self._lock:
            self._connected = False
            self._motion = self._feedback = None
            self._message = 'Disconnected'
        if errors:
            raise RuntimeError('Disconnect/stop failed: ' + '; '.join(errors))

    def _poll(self):
        self._read()
        with self._lock:
            motion = self._motion
            heartbeat = self._heartbeat_at
            safety = self._feedback['safety_mode']
        if motion:
            if safety not in (1, 2):
                raise RuntimeError('Robot safety mode changed during movement')
            if time.monotonic() - heartbeat > 2:
                raise RuntimeError('UI heartbeat lost; stopping movement')
            if time.monotonic() > motion['deadline']:
                raise RuntimeError('Movement timed out')
            progress = self.control.getAsyncOperationProgress()
            if progress >= 0:
                motion['started'] = True
            elif motion['started'] or progress != motion['before']:
                with self._lock:
                    self._motion = None
                    self._message = 'Movement complete'

    def _failed(self, error):
        try:
            self._stop_motion()
        except Exception as stop_error:
            try:
                self._disconnect()
            except Exception:
                pass
            error = RuntimeError(f'{error}; stop failed: {stop_error}')
        with self._lock:
            self._error = str(error)
            self._message = 'Command failed'

    def _run(self):
        try:
            while not self._shutdown.is_set():
                if self._stop_requested.is_set():
                    try:
                        had_motion = self._motion is not None
                        self._stop_motion()
                        if not had_motion:
                            with self._lock:
                                self._message = 'No active movement'
                    except Exception as exc:
                        self._failed(exc)
                    self._stop_requested.clear()
                try:
                    command, payload, generation = self._queue.get(timeout=.05)
                except queue.Empty:
                    pass
                else:
                    try:
                        self._execute(command, payload, generation)
                    except Exception as exc:
                        self._failed(exc)
                    finally:
                        with self._lock:
                            self._pending = None
                if self.control:
                    try:
                        self._poll()
                    except Exception as exc:
                        try:
                            self._disconnect()
                        except Exception as cleanup:
                            exc = RuntimeError(f'{exc}; {cleanup}')
                        self._failed(exc)
        finally:
            try:
                self._disconnect()
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)

    def close(self):
        self.stop()
        self._shutdown.set()
        self._worker.join(timeout=10)
        if self._worker.is_alive():
            raise RuntimeError('RTDE worker did not shut down; check robot state')
