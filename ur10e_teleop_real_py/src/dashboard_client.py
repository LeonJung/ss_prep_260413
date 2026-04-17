"""
dashboard_client.py — minimal client for UR Dashboard Server (port 29999).

Used to automate the Power On → Booting → Release Brakes sequence at
startup and the reverse (Brake Engage → Power Off) at shutdown, so the
operator doesn't have to step through the pendant by hand.

Reference: https://www.universal-robots.com/articles/ur/dashboard-server-e-series-port-29999/
"""

import socket
import time


# Robot mode enum (e-Series)
ROBOT_MODE_NO_CONTROLLER   = 0
ROBOT_MODE_DISCONNECTED    = 1
ROBOT_MODE_CONFIRM_SAFETY  = 2
ROBOT_MODE_BOOTING         = 3
ROBOT_MODE_POWER_OFF       = 4
ROBOT_MODE_POWER_ON        = 5
ROBOT_MODE_IDLE            = 6
ROBOT_MODE_RUNNING         = 7

_ROBOT_MODE_NAMES = {
    0: 'NO_CONTROLLER', 1: 'DISCONNECTED', 2: 'CONFIRM_SAFETY',
    3: 'BOOTING', 4: 'POWER_OFF', 5: 'POWER_ON', 6: 'IDLE', 7: 'RUNNING',
}


def robot_mode_name(m):
    return _ROBOT_MODE_NAMES.get(m, f'?{m}?')


class DashboardClient:
    """Blocking TCP client for port 29999. Not threadsafe; use from one thread."""

    DEFAULT_PORT = 29999

    def __init__(self, host, port=DEFAULT_PORT, timeout=5.0, logger=None):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.log = logger or (lambda s: print(f'[dashboard] {s}'))
        self._sock = None

    # ---- connection ---------------------------------------------------------

    def connect(self):
        if self._sock is not None:
            return True
        try:
            s = socket.create_connection((self.host, self.port), timeout=self.timeout)
            s.settimeout(self.timeout)
            # welcome banner — discard
            s.recv(4096)
            self._sock = s
            return True
        except Exception as e:
            self.log(f'connect({self.host}:{self.port}) failed: {e}')
            self._sock = None
            return False

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    # ---- primitive ----------------------------------------------------------

    def _ask(self, cmd):
        """Send one command line and return the single-line response."""
        if self._sock is None and not self.connect():
            return ''
        try:
            self._sock.sendall((cmd + '\n').encode('utf-8'))
            data = b''
            while b'\n' not in data:
                chunk = self._sock.recv(4096)
                if not chunk:
                    break
                data += chunk
            return data.decode('utf-8', errors='replace').strip()
        except Exception as e:
            self.log(f'_ask({cmd!r}) failed: {e}')
            self.close()
            return ''

    # ---- state queries ------------------------------------------------------

    def get_robot_mode(self):
        """Return integer robot mode (see ROBOT_MODE_* constants)."""
        resp = self._ask('robotmode')
        # response: "Robotmode: RUNNING"
        name = resp.rsplit(':', 1)[-1].strip() if ':' in resp else resp.strip()
        name_to_int = {v: k for k, v in _ROBOT_MODE_NAMES.items()}
        return name_to_int.get(name, -1)

    def get_safety_mode(self):
        return self._ask('safetymode')

    def is_in_remote_control(self):
        return self._ask('is in remote control').strip().lower().startswith('true')

    # ---- power + brakes -----------------------------------------------------

    def power_on(self):
        return self._ask('power on')

    def power_off(self):
        return self._ask('power off')

    def brake_release(self):
        return self._ask('brake release')

    def stop_program(self):
        return self._ask('stop')

    def unlock_protective_stop(self):
        return self._ask('unlock protective stop')

    # ---- helpers ------------------------------------------------------------

    def wait_for_mode(self, target_mode, timeout=30.0, poll_interval=0.25):
        """Poll robotmode until it reaches target_mode or timeout. Returns bool."""
        deadline = time.time() + timeout
        last = -1
        while time.time() < deadline:
            m = self.get_robot_mode()
            if m != last:
                self.log(f'robot_mode = {m} ({robot_mode_name(m)})')
                last = m
            if m == target_mode:
                return True
            time.sleep(poll_interval)
        self.log(f'wait_for_mode({target_mode}) timed out '
                 f'(last seen {last} = {robot_mode_name(last)})')
        return False

    # ---- high-level sequences ----------------------------------------------

    def power_up_sequence(self, timeout=45.0):
        """Power On → (BOOTING →) IDLE → brake release → RUNNING."""
        if not self.connect():
            return False

        if not self.is_in_remote_control():
            self.log('WARNING: robot is NOT in Remote Control mode — '
                     'power-cycle will not work remotely.')

        mode = self.get_robot_mode()
        self.log(f'current robot_mode = {mode} ({robot_mode_name(mode)})')

        if mode == ROBOT_MODE_RUNNING:
            self.log('already RUNNING — power-up sequence skipped')
            return True

        self.log('power on …')
        self.power_on()
        if not self.wait_for_mode(ROBOT_MODE_IDLE, timeout=timeout):
            return False

        self.log('brake release …')
        self.brake_release()
        if not self.wait_for_mode(ROBOT_MODE_RUNNING, timeout=timeout):
            return False

        self.log('power-up sequence complete (RUNNING)')
        return True

    def power_down_sequence(self, timeout=30.0):
        """Stop program → power off. (UR engages brakes on power-off.)"""
        if not self.connect():
            return False
        self.log('stop program …')
        self.stop_program()
        time.sleep(0.5)
        self.log('power off …')
        self.power_off()
        self.wait_for_mode(ROBOT_MODE_POWER_OFF, timeout=timeout)
        self.log('power-down sequence complete')
        return True
