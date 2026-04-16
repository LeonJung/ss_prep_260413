#!/usr/bin/env python3
"""
identify_robots.py — identify which UR model is at which IP.

Queries UR Dashboard Server (port 29999) for robot model, serial number,
and current state. No RTDE, no URScript upload — read-only diagnostic.

Usage:
    python3 identify_robots.py
    python3 identify_robots.py --ips 169.254.186.92 169.254.186.94
"""

import argparse
import socket
import sys


def ask(sock, cmd, timeout=3.0):
    """Send a dashboard command and return single-line response."""
    sock.settimeout(timeout)
    sock.sendall((cmd + '\n').encode('utf-8'))
    data = b''
    while b'\n' not in data:
        chunk = sock.recv(4096)
        if not chunk:
            break
        data += chunk
    return data.decode('utf-8', errors='replace').strip()


def query(ip, port=29999, timeout=3.0):
    try:
        s = socket.create_connection((ip, port), timeout=timeout)
        s.settimeout(timeout)
        # welcome banner
        banner = ''
        try:
            banner = s.recv(4096).decode('utf-8', errors='replace').strip()
        except socket.timeout:
            pass

        info = {'banner': banner}
        for key, cmd in [
            ('model',        'get robot model'),
            ('serial',       'get serial number'),
            ('polyscope',    'PolyscopeVersion'),
            ('safety_mode',  'safetymode'),
            ('robot_mode',   'robotmode'),
            ('program_state','programState'),
            ('is_in_remote', 'is in remote control'),
        ]:
            try:
                info[key] = ask(s, cmd)
            except Exception as e:
                info[key] = f'ERR {e}'
        s.close()
        return info
    except Exception as e:
        return {'error': str(e)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ips', nargs='+',
                        default=['169.254.186.92', '169.254.186.94'])
    parser.add_argument('--port', type=int, default=29999)
    args = parser.parse_args()

    print('=' * 64)
    print('  UR Robot Identification (Dashboard port 29999)')
    print('=' * 64)

    for ip in args.ips:
        print(f'\n--- {ip} ---')
        info = query(ip, port=args.port)
        if 'error' in info:
            print(f'  FAILED: {info["error"]}')
            continue
        # Banner often contains "Connected: Universal Robots Dashboard Server"
        if info.get('banner'):
            print(f'  banner:        {info["banner"]}')
        for k in ('model', 'serial', 'polyscope', 'robot_mode',
                  'safety_mode', 'program_state', 'is_in_remote'):
            if k in info:
                print(f'  {k:<14s} {info[k]}')

    print()


if __name__ == '__main__':
    sys.exit(main() or 0)
