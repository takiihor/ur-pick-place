"""Common grip/release verification interface for the sequence controller."""
from contextlib import contextmanager
import threading
import time

from ..controller import SequenceError


class RG2:
    def __init__(self, config, robot_ip):
        from .rg2_driver import RG2Gripper
        self.config = config
        self.driver = RG2Gripper(config.host or robot_ip, config.port,
                                 config.tool_index, config.timeout_s)

    def require_ready(self):
        state = self.driver.status()
        if state['busy'] or state['safety_failed'] or state['grip_detected']:
            raise SequenceError('Start with RG2 idle, fault-free, and empty')

    def open(self):
        c = self.config
        self.driver.move(c.open_width_mm, c.release_force_n, require_release=True)
        time.sleep(c.settle_s)

    def grip(self):
        c = self.config
        self.driver.move(c.grip_width_mm, c.grip_force_n, require_grip=True)
        time.sleep(c.settle_s)

    def require_holding(self):
        state = self.driver.status()
        if state['busy'] or state['safety_failed'] or not state['grip_detected']:
            raise SequenceError('RG2 does not confirm a held object')

    @contextmanager
    def probe_guard(self):
        # XML-RPC stays out of the RTDE loop. Failure is latched until teardown.
        self.driver.check_safety()
        stop = threading.Event()
        failures = []

        def monitor():
            while not stop.wait(0.05):
                try:
                    self.driver.check_safety()
                except Exception as exc:
                    failures.append(exc)
                    return

        def guard():
            if failures:
                raise SequenceError(f'RG2 probe guard failed: {failures[0]}')

        worker = threading.Thread(target=monitor, daemon=True)
        worker.start()
        try:
            yield guard
            guard()
        finally:
            stop.set()
            worker.join(timeout=0.5)
        if worker.is_alive():
            raise SequenceError('RG2 probe guard did not finish; grip blocked')
        guard()
        self.driver.check_safety()

    def close(self):
        self.driver.close()


class Arduino:
    def __init__(self, config, robot_ip=None):
        from .arduino_driver import ArduinoGripper
        config.require_live_ready()
        self.config = config
        self.driver = ArduinoGripper(config.serial_port, timeout_s=config.timeout_s,
                                     startup_wait_s=config.startup_wait_s)

    def _verify(self, holding):
        c = self.config
        limits = c.grip_min_raw if holding else c.release_max_raw
        for _ in range(c.pressure_samples):
            pads = self.driver.pressure()
            values = (pads.fsr1, pads.fsr2)
            valid = all(v >= limit if holding else v <= limit for v, limit in zip(values, limits))
            if not valid:
                label = 'grip' if holding else 'release'
                raise SequenceError(f'Arduino {label} check failed: {values}, thresholds={limits}')
            time.sleep(0.05)

    def require_ready(self):
        self._verify(False)

    def open(self):
        self.driver.open()
        time.sleep(self.config.settle_s)
        self._verify(False)

    def grip(self):
        getattr(self.driver, self.config.grip_command)()
        time.sleep(self.config.settle_s)
        self._verify(True)

    def require_holding(self):
        self._verify(True)

    @contextmanager
    def probe_guard(self):
        # This firmware provides no RG2-style safety-switch feedback.
        yield lambda: None

    def close(self):
        self.driver.close()


def connect_gripper(config, robot_ip):
    return (RG2 if config.kind == 'rg2' else Arduino)(config, robot_ip)
