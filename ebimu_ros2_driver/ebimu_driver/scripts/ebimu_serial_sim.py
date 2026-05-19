#!/usr/bin/env python3
"""Small pseudo-terminal EBIMU simulator for local driver testing."""

from __future__ import annotations

import argparse
import math
import os
import pty
import select
import time


def build_ascii_sample(t: float, timestamp_ms: int) -> bytes:
    z = math.sin(t * 0.3) * 0.05
    y = math.sin(t * 0.2) * 0.03
    x = math.sin(t * 0.1) * 0.02
    w = math.sqrt(max(0.0, 1.0 - x * x - y * y - z * z))
    fields = [
        z,
        y,
        x,
        w,
        0.1,
        -0.2,
        0.3,
        0.0,
        0.0,
        1.0,
        timestamp_ms % 65536,
    ]
    return ("*" + ",".join(f"{v:.5f}" for v in fields) + "\r\n").encode("ascii")


def to_u16(value: int) -> tuple[int, int]:
    value &= 0xFFFF
    return (value >> 8) & 0xFF, value & 0xFF


def build_binary_sample(t: float, timestamp_ms: int) -> bytes:
    z = int(math.sin(t * 0.3) * 500)
    y = int(math.sin(t * 0.2) * 300)
    x = int(math.sin(t * 0.1) * 200)
    w = 10000
    values = [z, y, x, w, 1, -2, 3, 0, 0, 1000, timestamp_ms % 65536]
    data = bytearray([0x55, 0x55])
    for value in values:
        hi, lo = to_u16(value)
        data.extend([hi, lo])
    checksum = sum(data) & 0xFFFF
    data.extend(to_u16(checksum))
    return bytes(data)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["ascii", "binary"], default="ascii")
    parser.add_argument("--rate", type=float, default=100.0)
    args = parser.parse_args()

    master, slave = pty.openpty()
    print(os.ttyname(slave), flush=True)
    next_sample = time.monotonic()
    start = next_sample
    interval = 1.0 / args.rate

    while True:
        readable, _, _ = select.select([master], [], [], 0.001)
        if readable:
            cmd = os.read(master, 256)
            if b"<" in cmd or cmd == b">":
                os.write(master, b"<ok>")

        now = time.monotonic()
        if now >= next_sample:
            timestamp_ms = int((now - start) * 1000)
            if args.mode == "binary":
                os.write(master, build_binary_sample(now - start, timestamp_ms))
            else:
                os.write(master, build_ascii_sample(now - start, timestamp_ms))
            next_sample += interval


if __name__ == "__main__":
    main()
