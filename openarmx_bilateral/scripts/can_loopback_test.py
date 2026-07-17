#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCAN-USB Pro FD 2채널 루프백 통신 테스트 (의존성 없음 — raw SocketCAN)

목적:
  로봇을 안 붙이고 Pro FD 어댑터/드라이버/양 채널이 정상인지만 격리 검증.
  한 채널(A)에서 프레임을 쏘고, 다른 채널(B)에서 그대로 받는지 확인.
  → 통과하면 "어댑터는 정상", 로봇 무통신은 로봇측 종단/배선 문제로 좁혀짐.
  → 실패하면 어댑터/드라이버/종단 문제.

배선:
  Pro FD의 두 CAN 포트를 짧은 전선으로 직결 (2노드 CAN 버스를 만듦)
     CAN1 포트의 CAN_H  ── CAN2 포트의 CAN_H
     CAN1 포트의 CAN_L  ── CAN2 포트의 CAN_L
  ⚠ 종단: Pro FD 내장 종단은 기본 OFF이고 Linux(peak_usb)에선 못 켬.
     - 짧은 직결(<30cm)이면 무종단으로도 대개 통과함.
     - 실패하면 두 CAN_H/CAN_L 사이에 120Ω 1개(가능하면 2개, 양끝)를 물릴 것.

사전 준비 (두 채널 모두 1Mbps로 UP, Robstride=classic CAN):
  sudo ip link set can0 up type can bitrate 1000000
  sudo ip link set can1 up type can bitrate 1000000
  # (라벨과 OS 번호가 다를 수 있음 — 'ip -br link show | grep can' 로 확인)

실행:
  python3 can_loopback_test.py can0 can1
  (인자 생략 시 can0 can1 기본값)
"""

import socket
import struct
import select
import sys
import time

# struct can_frame { u32 can_id; u8 can_dlc; u8 pad; u8 res0; u8 len8_dlc; u8 data[8]; }
CAN_FRAME_FMT = "=IB3x8s"                 # id(4) dlc(1) +3pad + data(8) = 16 bytes
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


def open_can(iface):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    return s


def send(sock, can_id, data):
    data = bytes(data)
    frame = struct.pack(CAN_FRAME_FMT, can_id & 0x7FF, len(data), data.ljust(8, b"\x00"))
    sock.send(frame)


def recv(sock, timeout):
    r, _, _ = select.select([sock], [], [], timeout)
    if not r:
        return None
    frame = sock.recv(CAN_FRAME_SIZE)
    can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
    return (can_id & 0x1FFFFFFF, data[:dlc])


def operstate(iface):
    try:
        with open(f"/sys/class/net/{iface}/operstate") as f:
            return f.read().strip()
    except OSError:
        return "missing"


def one_shot(tx_name, rx_name, tx_sock, rx_sock, can_id, payload):
    # rx 버퍼 비우기
    while recv(rx_sock, 0) is not None:
        pass
    send(tx_sock, can_id, payload)
    got = recv(rx_sock, 0.5)
    if got is None:
        print(f"  ✗ {tx_name} → {rx_name} : RX 무응답(timeout)")
        return False
    rid, rdata = got
    ok = (rid == can_id and bytes(rdata) == bytes(payload))
    mark = "✓" if ok else "✗"
    print(f"  {mark} {tx_name} → {rx_name} : 보냄 id=0x{can_id:X} "
          f"data={bytes(payload).hex()} | 받음 id=0x{rid:X} data={bytes(rdata).hex()}")
    return ok


def main():
    a = sys.argv[1] if len(sys.argv) > 1 else "can0"
    b = sys.argv[2] if len(sys.argv) > 2 else "can1"

    for i in (a, b):
        st = operstate(i)
        if st == "missing":
            print(f"[ERROR] '{i}' 인터페이스 없음. 'ip -br link show | grep can' 확인.")
            return 1
        if st not in ("up", "unknown"):        # CAN iface는 UP이어도 operstate가 'unknown'일 수 있음
            print(f"[ERROR] '{i}' state={st}. 먼저: "
                  f"sudo ip link set {i} up type can bitrate 1000000")
            return 1

    print(f"[INFO] 루프백 테스트: {a} <-> {b}  (CAN_H-CAN_H, CAN_L-CAN_L 직결 확인)")
    sa, sb = open_can(a), open_can(b)
    passed = total = 0
    try:
        for n in range(5):
            payload = bytes([n, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, n])
            total += 1; passed += one_shot(a, b, sa, sb, 0x123, payload)
            total += 1; passed += one_shot(b, a, sb, sa, 0x321, payload)
            time.sleep(0.2)
    except OSError as e:
        print(f"[ERROR] 송수신 실패: {e} (인터페이스 down? bitrate 불일치?)")
        return 1
    finally:
        sa.close(); sb.close()

    print(f"\n결과: {passed}/{total} 성공")
    if passed == 0:
        print("→ 전부 실패: (1)두 채널 H-H/L-L 직결 확인, "
              "(2)120Ω 종단 추가, (3)양쪽 bitrate 1M 확인")
    elif passed < total:
        print("→ 부분 성공: 신호 마진 의심(120Ω 추가 권장)")
    else:
        print("→ 어댑터·드라이버·양 채널 정상. "
              "로봇 무통신은 로봇측 종단/배선 문제로 좁혀짐.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
