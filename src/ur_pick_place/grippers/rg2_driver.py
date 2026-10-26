"""OnRobot URCap XML-RPC client for a wrist-connected RG2.

Uses the service verified on the robot at port 41414. This does not upload a
robot program or enable depth compensation. Constructing the client never moves
the jaws; only move() sends a grip command.
"""

import math
import logging
import threading
import time
import xmlrpc.client

logger = logging.getLogger(__name__)


class GripperError(RuntimeError):
    pass


class _Transport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        return connection

    def request(self, host, handler, request_body, verbose=False):
        # OnRobot closes idle keep-alive sockets while the arm is moving.
        # Start each RPC on a fresh connection instead of reusing that socket.
        # Do not automatically replay a jaw command after a disconnected reply.
        self.close()
        try:
            return self.single_request(host, handler, request_body, verbose)
        finally:
            self.close()


class RG2Gripper:
    def __init__(self, host, port=41414, tool_index=0, timeout=8.0):
        if tool_index not in (0, 1, 2):
            raise ValueError("RG2 tool index must be 0, 1, or 2")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("RG2 timeout must be positive and finite")
        self.tool_index = tool_index
        self.timeout = timeout
        self._transport = _Transport(min(timeout, 2.0))
        self._rpc = xmlrpc.client.ServerProxy(
            f"http://{host}:{port}/", transport=self._transport
        )
        self._lock = threading.RLock()

    def close(self):
        self._transport.close()

    @staticmethod
    def validate_target(width_mm, force_n):
        if not math.isfinite(width_mm) or not 0 <= width_mm <= 110:
            raise ValueError("RG2 width must be finite and in [0, 110] mm")
        if not math.isfinite(force_n) or not 3 <= force_n <= 40:
            raise ValueError("RG2 gripping force must be finite and in [3, 40] N")

    def status(self):
        with self._lock:
            try:
                width = float(self._rpc.rg_get_width(self.tool_index))
                busy = self._rpc.rg_get_busy(self.tool_index)
                grip = self._rpc.rg_get_grip_detected(self.tool_index)
                failed = self._rpc.rg_get_safety_failed(self.tool_index)
                if not math.isfinite(width) or not 0 <= width <= 120:
                    raise ValueError(f"Invalid RG2 width reading: {width}")
                if any(type(value) is not bool for value in (busy, grip, failed)):
                    raise ValueError("Invalid RG2 status flags")
                return dict(width_mm=width, busy=busy, grip_detected=grip,
                            safety_failed=failed)
            except Exception as exc:
                raise GripperError(f"Cannot read RG2 tool {self.tool_index}: {exc}") from exc

    def safety_details(self):
        """Read diagnostic flags only; never clear or reset a safety interlock."""
        details = {}
        methods = {
            'status_code': 'rg_get_status',
            'S1_pushed': 'rg_get_s1_pushed',
            'S1_triggered': 'rg_get_s1_triggered',
            'S2_pushed': 'rg_get_s2_pushed',
            'S2_triggered': 'rg_get_s2_triggered',
        }
        with self._lock:
            for key, method in methods.items():
                try:
                    details[key] = getattr(self._rpc, method)(self.tool_index)
                except Exception as exc:
                    details[key] = f'unavailable ({exc})'
        return details

    def check_safety(self):
        """Bounded, read-only check for use while the arm is force probing.

        This is a software guard, not a safety-rated robot stop. A lost reply
        aborts the probe; it is never treated as evidence of a healthy gripper.
        Keep the timeout short so this query cannot stall the probe for seconds.
        """
        with self._lock:
            timeout = self._transport.timeout
            self._transport.timeout = min(timeout, 0.1)
            try:
                failed = self._rpc.rg_get_safety_failed(self.tool_index)
            except Exception as exc:
                raise GripperError(f'Cannot verify RG2 safety during probe: {exc}') from exc
            finally:
                self._transport.timeout = timeout
        if type(failed) is not bool:
            raise GripperError('Invalid RG2 safety feedback during probe')
        if failed:
            raise GripperError(
                'RG2 safety interlock activated during pick probing; contact is not accepted. '
                'Check the S1/S2 switches and body-contact geometry before restarting.'
            )

    def _raise_safety_fault(self):
        details = ', '.join(f'{key}={value}' for key, value in self.safety_details().items())
        raise GripperError(
            f'RG2 safety fault; jaw command blocked. {details}. '
            'Inspect the gripper switches/clearance and resolve the OnRobot safety fault '
            'on the pendant before retrying; it is not cleared automatically.'
        )

    def move(self, width_mm, force_n, *, require_grip=False, require_release=False,
             process_events=lambda: None):
        """Wait for observed completion and the requested grip/release feedback.

        A release must reach the target width (within 1 mm) and report no grip.
        A grasp must report grip detection; reaching an empty closed position
        alone does not count as a successful pick. No command retry or automatic
        opening is performed on errors.
        """
        self.validate_target(width_mm, force_n)
        if require_grip and require_release:
            raise ValueError("Cannot require grip and release simultaneously")
        with self._lock:
            initial = self.status()
            if initial['safety_failed']:
                self._raise_safety_fault()
            if initial['busy']:
                raise GripperError(f"RG2 is busy; jaw command not sent. Status: {initial}")
            logger.info("Sending RG2 jaw command: tool=%d, width=%.1f mm, force=%.1f N",
                        self.tool_index, width_mm, force_n)
            try:
                result = self._rpc.rg_grip(
                    self.tool_index, float(width_mm), float(force_n)
                )
            except Exception as exc:
                raise GripperError(f"RG2 command reply failed; jaw state is unknown: {exc}") from exc
            if type(result) is not int or result < 0:
                raise GripperError(f"RG2 rejected command: {result!r}")

            started = time.monotonic()
            while time.monotonic() - started < self.timeout:
                process_events()
                state = self.status()
                if state['safety_failed']:
                    self._raise_safety_fault()
                # Allow the daemon to publish movement feedback before accepting
                # an idle sample. Completion also needs positive position/grip
                # evidence, not just the busy flag.
                if time.monotonic() - started >= 0.2 and not state['busy']:
                    at_width = abs(state['width_mm'] - width_mm) <= 1.0
                    if require_release:
                        complete = at_width and not state['grip_detected']
                    elif require_grip:
                        complete = state['grip_detected']
                    else:
                        complete = at_width or state['grip_detected']
                    if complete:
                        return state
                time.sleep(0.05)
            raise GripperError(
                "RG2 completion/feedback timeout; no further sequence movement will run"
            )
