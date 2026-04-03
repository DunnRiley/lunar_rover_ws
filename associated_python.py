#!/usr/bin/env python3
"""
Send 0xDC using packed 15-bit distance + 1 direction bit.

Packet format:
    [0xAA] [0xDC] [HI] [LO] [0x55]

Packed 16-bit field:
    bit 15    = direction bit (0 = forward, 1 = reverse)
    bits 14:0 = distance units

This matches rover_bl_15bit_from_working.ino where:
    dist_mm = distance_units * DIST_UNIT_MM
Default DIST_UNIT_MM in that file is 1.0 mm/unit.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List

import serial
import serial.tools.list_ports

START = 0xAA
END = 0x55
CMD_DRIVE_DISTANCE = 0xDC
CMD_STOP = 0xFF
ACK = 0xAA
BAUD = 115200
ACK_TIMEOUT_S = 1.0

DIR_FWD = 0
DIR_REV = 1
DEFAULT_DIST_UNIT_MM = 1.0


def list_ports() -> List[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


def pick_port() -> str:
    ports = list_ports()
    if not ports:
        raise SystemExit("No serial ports found.")
    if len(ports) == 1:
        print(f"Auto-selecting {ports[0]}")
        return ports[0]
    for i, p in enumerate(ports):
        print(f"[{i}] {p}")
    while True:
        raw = input("Select port number: ").strip()
        if raw.isdigit() and 0 <= int(raw) < len(ports):
            return ports[int(raw)]
        print("Invalid choice.")


def build_packet(device: int, byte2: int, byte3: int) -> bytes:
    return bytes([START, device & 0xFF, byte2 & 0xFF, byte3 & 0xFF, END])


def wait_for_ack(ser: serial.Serial) -> bool:
    deadline = time.monotonic() + ACK_TIMEOUT_S
    buf = bytearray()
    while time.monotonic() < deadline:
        if ser.in_waiting:
            buf.extend(ser.read(ser.in_waiting))
            if ACK in buf:
                return True
        time.sleep(0.005)
    return False


def send_packet(ser: serial.Serial, pkt: bytes, label: str) -> bool:
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()
    ok = wait_for_ack(ser)
    print(("✓" if ok else "✗"), label, "ACK" if ok else "no ACK")
    return ok
