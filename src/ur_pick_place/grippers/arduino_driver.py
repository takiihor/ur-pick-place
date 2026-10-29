"""Standalone serial driver for firmware/arduino_gripper.ino.

Requires only pyserial. Replies acknowledge command handling, not completed
motion or a detected grip. Pressure readings are raw sums, not calibrated N.
"""

import argparse
from dataclasses import dataclass
import re
import threading
import time


class GripperError(RuntimeError):
    """Communication failed; a sent jaw command may already have taken effect."""


@dataclass(frozen=True)
class PadPressure:
    """Each reading is the sum of three Arduino ADC samples (0..3069)."""

    fsr1: int
    fsr2: int


class ArduinoGripper:
    """9600 baud, 8N1, newline-delimited protocol; no automatic retries.

    Opening some Arduino serial ports resets the board. startup_wait_s allows
    the bootloader and setup() to finish before the first command is sent.
    Construction and close() never send a jaw command.
    """

    def __init__(self, port, *, timeout_s=2.0, startup_wait_s=2.0):
        import math

        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('timeout_s must be positive and finite')
        if not math.isfinite(startup_wait_s) or startup_wait_s < 0:
            raise ValueError('startup_wait_s must be nonnegative and finite')
        try:
            import serial
        except ImportError as exc:
            raise GripperError('Install pyserial: python -m pip install pyserial') from exc
        self._lock = threading.Lock()
        self.timeout_s = timeout_s
        try:
            self._serial = serial.Serial(
                port=port, baudrate=9600, bytesize=8, parity='N', stopbits=1,
                timeout=timeout_s, write_timeout=timeout_s,
                xonxoff=False, rtscts=False, dsrdtr=False,
            )
        except (OSError, serial.SerialException) as exc:
            raise GripperError(f'Cannot open gripper port {port}: {exc}') from exc
        try:
            time.sleep(startup_wait_s)
            self._serial.reset_input_buffer()
        except BaseException:
            self._serial.close()
            raise

    def _exchange(self, command):
        with self._lock:
            try:
                # Discard old replies so they cannot acknowledge this command.
                self._serial.reset_input_buffer()
                payload = f'{command}\n'.encode('ascii')
                if self._serial.write(payload) != len(payload):
                    raise GripperError('Incomplete command write; jaw state is unknown')
                reply = self._serial.readline(128)
                if not reply.endswith(b'\n'):
                    raise GripperError(
                        'No complete gripper reply before timeout; command was not retried'
                    )
                return reply.decode('ascii').strip()
            except GripperError:
                raise
            except (OSError, UnicodeError) as exc:
                raise GripperError(f'Gripper communication failed: {exc}') from exc

    def _move(self, command, expected):
        reply = self._exchange(command)
        if reply != expected:
            raise GripperError(f'Expected {expected!r}, received {reply!r}')
        return reply

    def tight(self):
        """Command servo angles 115/90 degrees; wait for 'Tight' reply."""
        return self._move(1, 'Tight')

    def loose(self):
        """Command servo angles 90/123 degrees; wait for 'Loose' reply."""
        return self._move(2, 'Loose')

    def open(self):
        """Command servo angles 40/165 degrees; wait for 'Open' reply."""
        return self._move(3, 'Open')

    def pressure(self):
        reply = self._exchange(0)
        match = re.fullmatch(r'FSR1:(\d+)\s*/\s*FSR2:(\d+)', reply)
        if match is None:
            raise GripperError(f'Invalid pressure reply: {reply!r}')
        values = tuple(map(int, match.groups()))
        if any(value > 3069 for value in values):
            raise GripperError(f'Pressure reading outside ADC sum range: {reply!r}')
        return PadPressure(*values)

    def close(self):
        """Close communication without releasing the jaws."""
        with self._lock:
            self._serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('ports', 'tight', 'loose', 'open', 'pressure'))
    parser.add_argument('--port', help='Serial device, e.g. /dev/ttyUSB0 or COM3')
    parser.add_argument('--timeout-s', type=float, default=2.0)
    parser.add_argument('--startup-wait-s', type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.action != 'ports' and not args.port:
        parser.error('--port is required for gripper commands')
    try:
        if args.action == 'ports':
            from serial.tools import list_ports
            ports = list(list_ports.comports())
            for port in ports:
                print(f'{port.device}: {port.description}')
            if not ports:
                print('No serial ports found.')
            return 0
        with ArduinoGripper(args.port, timeout_s=args.timeout_s,
                            startup_wait_s=args.startup_wait_s) as gripper:
            result = getattr(gripper, args.action)()
            if isinstance(result, PadPressure):
                print(f'FSR1:{result.fsr1} / FSR2:{result.fsr2} (raw sums of 3 ADC samples)')
            else:
                print(f'{result}: command acknowledged; motion completion is not reported')
        return 0
    except (GripperError, ValueError, OSError, ImportError) as exc:
        parser.exit(1, f'Gripper error: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
