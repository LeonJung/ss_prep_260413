#!/usr/bin/env python3
"""
ur_server_dummy.py — RTDE server that simulates a UR robot.

Accepts RTDE binary protocol connections on port 30004 (default).
Simulates rigid-body dynamics and streams joint state at 500Hz.

Usage:
    python3 ur_server_dummy.py [--port 30004] [--robot ur10e]
"""

import argparse
import os
import socket
import struct
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from environment_sensing_data_emulator import EnvironmentEmulator
from rtde_connection import (
    RTDE_PORT,
    RTDE_REQUEST_PROTOCOL_VERSION,
    RTDE_GET_URCONTROL_VERSION,
    RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS,
    RTDE_CONTROL_PACKAGE_SETUP_INPUTS,
    RTDE_CONTROL_PACKAGE_START,
    RTDE_CONTROL_PACKAGE_PAUSE,
    RTDE_DATA_PACKAGE,
    RTDE_DATA_PACKAGE_INPUT,
    RTDE_PROTOCOL_VERSION,
    RTDE_TYPE_DOUBLE,
    RTDE_TYPE_VECTOR6D,
    RTDE_TYPE_INT32,
    N,
)

HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

_ROBOT_PARAMS = {
    'ur10e': {
        'inertia': np.array([2.8, 6.8, 2.2, 0.14, 0.11, 0.10]),
        'damping': np.array([10.0, 10.0, 8.0, 4.0, 4.0, 2.0]),
    },
    'ur3e': {
        'inertia': np.array([0.5, 1.2, 0.4, 0.03, 0.02, 0.02]),
        'damping': np.array([5.0, 5.0, 3.0, 1.0, 1.0, 0.5]),
    },
}


class URServerDummy:
    """RTDE server simulating a UR robot with simple dynamics."""

    def __init__(self, port: int = RTDE_PORT, robot_name: str = 'ur10e',
                 timestep: float = 0.002):
        self.port = port
        self.robot_name = robot_name
        self.timestep = timestep

        params = _ROBOT_PARAMS[robot_name]
        # Reflected through 100:1 gear — heavier + more friction (like real UR)
        self._inertia = params['inertia'].copy() * 50.0
        self._damping = params['damping'].copy() * 50.0

        self.emulator = EnvironmentEmulator(robot_name)

        self._q = HOME_QPOS.copy()
        self._dq = np.zeros(N)
        self._tau_cmd = np.zeros(N)
        self._cmd_received = False   # don't step until first command received
        self._lock = threading.Lock()

        self._server_sock = None
        self._client_sock = None
        self._stop = threading.Event()
        self._streaming = False

        # Recipe IDs
        self._output_recipe_id = 1
        self._input_recipe_id = 2
        self._output_variables = []
        self._input_variables = []

    def start(self):
        """Start TCP server and accept one client."""
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind(('0.0.0.0', self.port))
        self._server_sock.listen(1)
        self._server_sock.settimeout(1.0)
        print(f'[URServerDummy] Listening on port {self.port} ({self.robot_name})')

        while not self._stop.is_set():
            try:
                self._client_sock, addr = self._server_sock.accept()
                self._client_sock.settimeout(0.01)
                print(f'[URServerDummy] Client connected from {addr}')
                self._handle_client()
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop.is_set():
                    print(f'[URServerDummy] Error: {e}')
                break

    def stop(self):
        self._stop.set()
        if self._client_sock:
            try:
                self._client_sock.close()
            except Exception:
                pass
        if self._server_sock:
            try:
                self._server_sock.close()
            except Exception:
                pass
        print('[URServerDummy] Stopped.')

    def _handle_client(self):
        """Handle RTDE protocol for one client connection."""
        try:
            while not self._stop.is_set():
                if self._streaming:
                    self._step_and_send()
                    self._try_recv_input()
                    time.sleep(self.timestep)
                else:
                    # Wait for negotiation packets
                    self._process_negotiation()
        except (ConnectionResetError, BrokenPipeError):
            print('[URServerDummy] Client disconnected.')
        except Exception as e:
            if not self._stop.is_set():
                print(f'[URServerDummy] Client error: {e}')

    def _process_negotiation(self):
        """Process one negotiation packet from client."""
        ptype, payload = self._recv_packet(timeout=0.5)
        if ptype is None:
            return

        if ptype == RTDE_REQUEST_PROTOCOL_VERSION:
            version = struct.unpack('>H', payload)[0]
            accepted = 1 if version == RTDE_PROTOCOL_VERSION else 0
            self._send_packet(RTDE_REQUEST_PROTOCOL_VERSION,
                              struct.pack('>B', accepted))

        elif ptype == RTDE_GET_URCONTROL_VERSION:
            # Send fake version 5.22.0.0
            self._send_packet(RTDE_GET_URCONTROL_VERSION,
                              struct.pack('>IIII', 5, 22, 0, 0))

        elif ptype == RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
            freq = struct.unpack_from('>d', payload, 0)[0]
            variables = payload[8:].decode('utf-8').split(',')
            self._output_variables = variables
            # Respond with recipe ID + variable types
            resp = struct.pack('>B', self._output_recipe_id)
            for var in variables:
                vtype = self._get_variable_type(var)
                resp += vtype.encode('utf-8') + b','
            resp = resp[:-1]  # remove trailing comma
            self._send_packet(RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, resp)

        elif ptype == RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
            variables = payload.decode('utf-8').split(',')
            self._input_variables = variables
            resp = struct.pack('>B', self._input_recipe_id)
            for var in variables:
                vtype = self._get_variable_type(var)
                resp += vtype.encode('utf-8') + b','
            resp = resp[:-1]
            self._send_packet(RTDE_CONTROL_PACKAGE_SETUP_INPUTS, resp)

        elif ptype == RTDE_CONTROL_PACKAGE_START:
            self._streaming = True
            self._send_packet(RTDE_CONTROL_PACKAGE_START,
                              struct.pack('>B', 1))
            print('[URServerDummy] Streaming started.')

        elif ptype == RTDE_CONTROL_PACKAGE_PAUSE:
            self._streaming = False
            self._send_packet(RTDE_CONTROL_PACKAGE_PAUSE,
                              struct.pack('>B', 1))

    def _step_and_send(self):
        """Step dynamics and send output data package.

        Emulates UR firmware behavior: gravity is automatically compensated.
        The client's tau should NOT include gravity (same as real UR direct_torque).
        Applied: tau_applied = tau_cmd + gravity_comp
        Net:     ddq = (tau_applied + contact - gravity - damping*dq) / inertia
                     = (tau_cmd + contact - damping*dq) / inertia
        """
        with self._lock:
            if self._cmd_received:
                tau = self._tau_cmd.copy()
                q = self._q
                dq = self._dq

                tau_contact = self.emulator.compute_contact_forces(q)

                # Gravity cancels: firmware adds gravity, dynamics subtracts it.
                # Dynamics: M*ddq = tau + contact - damping*dq
                tau_net = tau + tau_contact - self._damping * dq
                ddq = tau_net / self._inertia

                # Velocity clamp to prevent numerical explosion
                dq += ddq * self.timestep
                dq[:] = np.clip(dq, -2.0, 2.0)
                q += dq * self.timestep

        # Build output data package
        payload = struct.pack('>B', self._output_recipe_id)
        for var in self._output_variables:
            if var == 'timestamp':
                payload += struct.pack('>d', time.time())
            elif var == 'actual_q':
                with self._lock:
                    payload += struct.pack('>6d', *self._q.tolist())
            elif var == 'actual_qd':
                with self._lock:
                    payload += struct.pack('>6d', *self._dq.tolist())
            elif var == 'actual_current':
                payload += struct.pack('>6d', *([0.0] * N))
            elif var == 'robot_mode':
                payload += struct.pack('>i', 7)  # RUNNING
            elif var == 'safety_mode':
                payload += struct.pack('>i', 1)  # NORMAL

        self._send_packet(RTDE_DATA_PACKAGE, payload)

    def _try_recv_input(self):
        """Try to receive input command from client (non-blocking)."""
        ptype, payload = self._recv_packet(timeout=0.001)
        if ptype == RTDE_DATA_PACKAGE_INPUT and payload:
            offset = 1  # skip recipe_id
            torques = np.zeros(N)
            for i in range(N):
                if offset + 8 <= len(payload):
                    torques[i] = struct.unpack_from('>d', payload, offset)[0]
                    offset += 8
            with self._lock:
                self._tau_cmd[:] = torques
                self._cmd_received = True
        elif ptype is not None and not self._streaming:
            # Could be a negotiation packet during streaming
            pass

    def _get_variable_type(self, var_name: str) -> str:
        """Map RTDE variable name to type string."""
        _MAP = {
            'timestamp': RTDE_TYPE_DOUBLE,
            'actual_q': RTDE_TYPE_VECTOR6D,
            'actual_qd': RTDE_TYPE_VECTOR6D,
            'actual_current': RTDE_TYPE_VECTOR6D,
            'robot_mode': RTDE_TYPE_INT32,
            'safety_mode': RTDE_TYPE_INT32,
        }
        if var_name in _MAP:
            return _MAP[var_name]
        if var_name.startswith('input_double_register'):
            return RTDE_TYPE_DOUBLE
        if var_name.startswith('input_int_register'):
            return RTDE_TYPE_INT32
        return RTDE_TYPE_DOUBLE

    # ---- Low-level packet I/O ----------------------------------------------

    def _send_packet(self, ptype: int, payload: bytes):
        size = 3 + len(payload)
        header = struct.pack('>HB', size, ptype)
        self._client_sock.sendall(header + payload)

    def _recv_packet(self, timeout: float = 0.5) -> tuple:
        try:
            self._client_sock.settimeout(timeout)
            header = self._recv_exact(3)
            if header is None:
                return None, None
            size, ptype = struct.unpack('>HB', header)
            payload_size = size - 3
            payload = self._recv_exact(payload_size) if payload_size > 0 else b''
            return ptype, payload
        except socket.timeout:
            return None, None
        except Exception:
            return None, None

    def _recv_exact(self, n: int) -> bytes:
        data = b''
        while len(data) < n:
            chunk = self._client_sock.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data


def main():
    parser = argparse.ArgumentParser(description='UR RTDE Server Dummy')
    parser.add_argument('--port', type=int, default=RTDE_PORT)
    parser.add_argument('--robot', default='ur10e', choices=['ur10e', 'ur3e'])
    args = parser.parse_args()

    server = URServerDummy(port=args.port, robot_name=args.robot)
    try:
        server.start()
    except KeyboardInterrupt:
        server.stop()


if __name__ == '__main__':
    main()
