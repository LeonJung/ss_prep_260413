"""
rtde_connection.py — RTDE binary protocol implementation over TCP.

Implements the UR RTDE (Real-Time Data Exchange) protocol for communication
with UR robots (or ur_server_dummy.py for testing).

Protocol reference:
  https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/

Packet format:
  [packet_size: uint16_be] [packet_type: uint8] [payload: bytes]
"""

import socket
import struct
import threading
import time

import numpy as np

# ---------------------------------------------------------------------------
# RTDE Protocol Constants
# ---------------------------------------------------------------------------
RTDE_PORT = 30004

# Packet types (client → server)
RTDE_REQUEST_PROTOCOL_VERSION = 86   # 'V'
RTDE_GET_URCONTROL_VERSION    = 118  # 'v'
RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79  # 'O'
RTDE_CONTROL_PACKAGE_SETUP_INPUTS  = 73  # 'I'
RTDE_CONTROL_PACKAGE_START  = 83  # 'S'
RTDE_CONTROL_PACKAGE_PAUSE  = 80  # 'P'

# Packet types (server → client)
RTDE_DATA_PACKAGE = 85  # 'U' — output data from robot

# Input data packet (client → server)
RTDE_DATA_PACKAGE_INPUT = 73  # reuse 'I' type for input data

# Protocol version
RTDE_PROTOCOL_VERSION = 2

# RTDE data types
RTDE_TYPE_DOUBLE  = 'DOUBLE'
RTDE_TYPE_UINT32  = 'UINT32'
RTDE_TYPE_INT32   = 'INT32'
RTDE_TYPE_VECTOR6D = 'VECTOR6D'
RTDE_TYPE_BOOL    = 'BOOL'

# Type sizes in bytes
_TYPE_SIZE = {
    RTDE_TYPE_DOUBLE: 8,
    RTDE_TYPE_UINT32: 4,
    RTDE_TYPE_INT32: 4,
    RTDE_TYPE_VECTOR6D: 48,  # 6 * 8
    RTDE_TYPE_BOOL: 1,
}

# Standard output recipe for bilateral teleop
DEFAULT_OUTPUT_RECIPE = [
    ('timestamp',       RTDE_TYPE_DOUBLE),
    ('actual_q',        RTDE_TYPE_VECTOR6D),
    ('actual_qd',       RTDE_TYPE_VECTOR6D),
    ('actual_current',  RTDE_TYPE_VECTOR6D),
    ('robot_mode',      RTDE_TYPE_INT32),
    ('safety_mode',     RTDE_TYPE_INT32),
]

# Standard input recipe (torque command)
DEFAULT_INPUT_RECIPE = [
    ('input_double_register_0', RTDE_TYPE_DOUBLE),
    ('input_double_register_1', RTDE_TYPE_DOUBLE),
    ('input_double_register_2', RTDE_TYPE_DOUBLE),
    ('input_double_register_3', RTDE_TYPE_DOUBLE),
    ('input_double_register_4', RTDE_TYPE_DOUBLE),
    ('input_double_register_5', RTDE_TYPE_DOUBLE),
    ('input_int_register_0',    RTDE_TYPE_INT32),
]

N = 6


def _compute_output_size(recipe):
    """Compute total payload size of an output recipe."""
    # recipe_id (uint8) + data fields
    return 1 + sum(_TYPE_SIZE[t] for _, t in recipe)


def _compute_input_size(recipe):
    return 1 + sum(_TYPE_SIZE[t] for _, t in recipe)


# ---------------------------------------------------------------------------
# RTDE Connection (client side)
# ---------------------------------------------------------------------------
class RTDEConnection:
    """Low-level RTDE binary protocol client."""

    def __init__(self, host: str, port: int = RTDE_PORT):
        self.host = host
        self.port = port
        self._sock = None
        self._connected = False
        self._output_recipe_id = 0
        self._input_recipe_id = 0
        self._output_recipe = DEFAULT_OUTPUT_RECIPE
        self._input_recipe = DEFAULT_INPUT_RECIPE
        self._recv_lock = threading.Lock()

    def connect(self) -> bool:
        """Connect to RTDE server and perform protocol negotiation."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(5.0)
            self._sock.connect((self.host, self.port))
            self._sock.settimeout(0.1)  # non-blocking for recv

            # 1. Protocol version negotiation
            if not self._negotiate_protocol_version():
                return False

            # 2. Setup output recipe
            self._output_recipe_id = self._setup_output_recipe()
            if self._output_recipe_id < 0:
                return False

            # 3. Setup input recipe
            self._input_recipe_id = self._setup_input_recipe()
            if self._input_recipe_id < 0:
                return False

            # 4. Start data streaming
            if not self._start_streaming():
                return False

            self._connected = True
            return True
        except Exception as e:
            print(f'[RTDEConnection] Connect failed: {e}')
            return False

    def disconnect(self):
        if self._sock:
            try:
                self._send_packet(RTDE_CONTROL_PACKAGE_PAUSE, b'')
                self._sock.close()
            except Exception:
                pass
        self._connected = False

    def send_input(self, torques: np.ndarray, control_mode: int = 1) -> bool:
        """Send torque command as input data package."""
        # Input data: recipe_id + 6 doubles + 1 int32
        payload = struct.pack('>B', self._input_recipe_id)
        for i in range(N):
            payload += struct.pack('>d', float(torques[i]))
        payload += struct.pack('>i', control_mode)
        return self._send_packet(RTDE_DATA_PACKAGE_INPUT, payload)

    def receive_output(self) -> dict:
        """Receive one output data package. Returns dict or None."""
        with self._recv_lock:
            ptype, payload = self._recv_packet()
        if ptype != RTDE_DATA_PACKAGE or payload is None:
            return None

        offset = 0
        recipe_id = payload[offset]; offset += 1

        result = {}
        for name, dtype in self._output_recipe:
            if dtype == RTDE_TYPE_DOUBLE:
                val = struct.unpack_from('>d', payload, offset)[0]
                offset += 8
                result[name] = val
            elif dtype == RTDE_TYPE_VECTOR6D:
                vals = struct.unpack_from('>6d', payload, offset)
                offset += 48
                result[name] = np.array(vals)
            elif dtype == RTDE_TYPE_INT32:
                val = struct.unpack_from('>i', payload, offset)[0]
                offset += 4
                result[name] = val
            elif dtype == RTDE_TYPE_UINT32:
                val = struct.unpack_from('>I', payload, offset)[0]
                offset += 4
                result[name] = val
        return result

    # ---- Protocol negotiation ----------------------------------------------

    def _negotiate_protocol_version(self) -> bool:
        payload = struct.pack('>H', RTDE_PROTOCOL_VERSION)
        self._send_packet(RTDE_REQUEST_PROTOCOL_VERSION, payload)
        ptype, resp = self._recv_packet()
        if ptype == RTDE_REQUEST_PROTOCOL_VERSION and resp:
            accepted = resp[0]
            return accepted == 1
        return False

    def _setup_output_recipe(self) -> int:
        variables = ','.join(name for name, _ in self._output_recipe)
        payload = struct.pack('>d', 500.0)  # output frequency
        payload += variables.encode('utf-8')
        self._send_packet(RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, payload)
        ptype, resp = self._recv_packet()
        if ptype == RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS and resp:
            recipe_id = resp[0]
            return recipe_id
        return -1

    def _setup_input_recipe(self) -> int:
        variables = ','.join(name for name, _ in self._input_recipe)
        self._send_packet(RTDE_CONTROL_PACKAGE_SETUP_INPUTS, variables.encode('utf-8'))
        ptype, resp = self._recv_packet()
        if ptype == RTDE_CONTROL_PACKAGE_SETUP_INPUTS and resp:
            recipe_id = resp[0]
            return recipe_id
        return -1

    def _start_streaming(self) -> bool:
        self._send_packet(RTDE_CONTROL_PACKAGE_START, b'')
        ptype, resp = self._recv_packet()
        if ptype == RTDE_CONTROL_PACKAGE_START and resp:
            return resp[0] == 1
        return False

    # ---- Low-level packet I/O ----------------------------------------------

    def _send_packet(self, ptype: int, payload: bytes) -> bool:
        size = 3 + len(payload)  # 2 (size) + 1 (type) + payload
        header = struct.pack('>HB', size, ptype)
        try:
            self._sock.sendall(header + payload)
            return True
        except Exception:
            return False

    def _recv_packet(self, timeout: float = 1.0) -> tuple:
        """Receive one RTDE packet. Returns (ptype, payload) or (None, None)."""
        try:
            self._sock.settimeout(timeout)
            header = self._recv_exact(3)
            if header is None:
                return None, None
            size, ptype = struct.unpack('>HB', header)
            payload_size = size - 3
            if payload_size > 0:
                payload = self._recv_exact(payload_size)
            else:
                payload = b''
            return ptype, payload
        except socket.timeout:
            return None, None
        except Exception:
            return None, None

    def _recv_exact(self, n: int) -> bytes:
        """Receive exactly n bytes."""
        data = b''
        while len(data) < n:
            chunk = self._sock.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data
