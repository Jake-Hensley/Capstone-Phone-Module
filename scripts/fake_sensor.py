#!/usr/bin/env python3
"""
Fake PPG sensor: broadcasts BLE advertisements from this Linux laptop in the
format the bridge firmware expects (see plan.md, section 1).

Each advertisement (28 of 31 bytes):
    02 01 06                      Flags
    07 09 "BEACON"                Complete local name
    10 FF  FF FF  01  <12 B>      Manufacturer data: company 0xFFFF,
                                  version 0x01, one record:
                                  uint32 LE timestamp ms, red, ir

Usage:
    sudo python3 scripts/fake_sensor.py            # broadcast
    python3 scripts/fake_sensor.py --dry-run       # print packets only

Needs BlueZ's btmgmt and root (the kernel management socket requires it).
Ctrl+C stops it and removes the advertisement.
"""

import argparse
import math
import os
import pty
import struct
import subprocess
import sys
import time

NAME = b"BEACON"
COMPANY_ID = 0xFFFF
VERSION = 0x01
ADC_MAX = 0x3FFFF


def build_adv(ts_ms: int, red: int, ir: int) -> bytes:
    flags = bytes([0x02, 0x01, 0x06])
    name = bytes([len(NAME) + 1, 0x09]) + NAME
    mfr_payload = struct.pack("<HB", COMPANY_ID, VERSION) + struct.pack(
        "<III", ts_ms & 0xFFFFFFFF, red, ir)
    mfr = bytes([len(mfr_payload) + 1, 0xFF]) + mfr_payload
    adv = flags + name + mfr
    assert len(adv) <= 31, len(adv)
    return adv


def fake_ppg(t_s: float) -> tuple[int, int]:
    """Plausible-looking 18-bit PPG values: ~72 bpm pulse on a baseline."""
    pulse = math.sin(2 * math.pi * 1.2 * t_s)
    red = int(100_000 + 4_000 * pulse)
    ir = int(130_000 + 6_000 * pulse)
    return min(max(red, 0), ADC_MAX), min(max(ir, 0), ADC_MAX)


def btmgmt(*args: str) -> None:
    # btmgmt never exits when stdin is not a terminal (BlueZ 5.77), so give
    # it a pseudo-terminal.
    master, slave = pty.openpty()
    try:
        r = subprocess.run(["btmgmt", *args], stdin=slave,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           timeout=10)
    finally:
        os.close(slave)
        os.close(master)
    out = r.stdout.decode(errors="replace").strip()
    if r.returncode != 0 or "fail" in out.lower() or "error" in out.lower():
        raise RuntimeError(f"btmgmt {' '.join(args)} failed "
                           f"(rc {r.returncode}): {out}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--period", type=float, default=2.0,
                    help="seconds between new samples (default 2.0; the "
                         "kernel's default advertising interval is ~1.28 s)")
    ap.add_argument("--instance", type=int, default=5,
                    help="advertising instance id (default 5)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print packets without broadcasting")
    args = ap.parse_args()

    inst = str(args.instance)
    start = time.monotonic()

    if not args.dry_run:
        btmgmt("power", "on")

    try:
        while True:
            t = time.monotonic() - start
            ts_ms = int(t * 1000)
            red, ir = fake_ppg(t)
            adv = build_adv(ts_ms, red, ir)
            hexdata = adv.hex().upper()

            print(f"t={ts_ms:>8} ms  red={red:>6}  ir={ir:>6}  adv={hexdata}",
                  flush=True)

            if not args.dry_run:
                # Re-adding the same instance replaces its data. No -c flag,
                # no scan response -> non-connectable (ADV_NONCONN_IND).
                btmgmt("add-adv", "-d", hexdata, inst)

            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        if not args.dry_run:
            try:
                btmgmt("rm-adv", inst)
            except Exception as e:  # noqa: BLE001
                print(f"cleanup failed: {e}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
