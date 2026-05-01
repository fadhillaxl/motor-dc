#!/usr/bin/env python3
# coding: utf-8
"""
Scan/check WT901 RS485 addresses on one bus.

Contoh:
  python check_wt901_address.py
  python check_wt901_address.py --start 0x50 --end 0x5F
  python check_wt901_address.py --start 0x00 --end 0xFF --tries 2
"""

import argparse
import os
import platform
import sys
import time

# =============================
# PATH SDK
# =============================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "Python-SDK-WT901C485", "chs"))
sys.path.insert(0, SDK_CHS)

# =============================
# IMPORT SDK
# =============================
try:
    import lib.device_model as deviceModel
    from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
    from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver
except Exception as exc:
    print(f"ERROR import SDK: {exc}")
    print(f"Pastikan path SDK valid: {SDK_CHS}")
    sys.exit(1)


def parse_int_auto(value: str) -> int:
    return int(str(value).strip(), 0)


def build_device():
    try:
        return deviceModel.DeviceModel(
            "WT901C485",
            Protocol485Resolver(),
            JY901SDataProcessor(),
        )
    except TypeError:
        return deviceModel.DeviceModel(
            "WT901C485",
            Protocol485Resolver(),
            JY901SDataProcessor(),
            "ADDR_SCANNER",
        )


def read_one_packet(device, addr: int):
    device.ADDR = int(addr)

    # Trigger register read from sensor and use direct response payload.
    # This avoids stale cache false-positive from getDeviceData/get().
    if not hasattr(device, "readReg"):
        return None
    values = device.readReg(0x30, 41)
    if not values:
        return None
    return values


def probe_addr(device, addr: int, tries: int) -> tuple[bool, float | None]:
    def _to_signed16(v: int) -> int:
        v = int(v) & 0xFFFF
        return v - 0x10000 if v & 0x8000 else v

    for _ in range(max(1, tries)):
        try:
            vals = read_one_packet(device, addr)
            if vals:
                # angleZ register is 0x3F, scanned from 0x30 -> index 15.
                if len(vals) > 15:
                    raw = _to_signed16(vals[15])
                    angle_z = raw / 32768.0 * 180.0
                    return True, float(angle_z)
                # If payload exists but angle index unavailable, still count as found.
                return True, None
        except Exception:
            # ignore per-address read errors while scanning
            pass
        time.sleep(0.02)
    return False, None


def main():
    parser = argparse.ArgumentParser(description="Scan WT901 RS485 address")
    parser.add_argument("--port", default=None, help="Serial port (default auto)")
    parser.add_argument("--baud", type=int, default=9600, help="Baudrate (default: 9600)")
    parser.add_argument("--start", type=parse_int_auto, default=0x50, help="Address start (default: 0x50)")
    parser.add_argument("--end", type=parse_int_auto, default=0x5F, help="Address end (default: 0x5F)")
    parser.add_argument("--tries", type=int, default=3, help="Read attempts per address (default: 3)")
    args = parser.parse_args()

    start = int(args.start)
    end = int(args.end)
    if not (0 <= start <= 0xFF and 0 <= end <= 0xFF and start <= end):
        raise SystemExit("Range address invalid. Gunakan 0..255 dan start <= end.")

    port = args.port
    if not port:
        port = "/dev/ttyUSB0" if platform.system().lower() == "linux" else "/dev/tty.usbserial-1330"

    print("=" * 72)
    print(" WT901 RS485 ADDRESS CHECKER")
    print("=" * 72)
    print(f"[INFO] Port={port} Baud={args.baud} Scan=0x{start:02X}..0x{end:02X} tries={args.tries}")

    device = build_device()
    device.serialConfig.portName = port
    device.serialConfig.baud = int(args.baud)
    # Initial address doesn't matter for scan; will be overwritten each probe.
    device.ADDR = start

    found = []
    try:
        device.openDevice()
        time.sleep(0.5)

        for addr in range(start, end + 1):
            ok, angle_z = probe_addr(device, addr, args.tries)
            if ok:
                found.append((addr, angle_z))
                if angle_z is None:
                    print(f"[FOUND] addr=0x{addr:02X}")
                else:
                    print(f"[FOUND] addr=0x{addr:02X} angleZ={angle_z:.2f}")
            else:
                print(f"[---- ] addr=0x{addr:02X}")

    except KeyboardInterrupt:
        print("\n[STOP] scan dibatalkan user.")
    except Exception as exc:
        print(f"[ERR] scan gagal: {exc}")
        sys.exit(1)
    finally:
        try:
            # Avoid noisy SDK thread shutdown errors.
            device.isOpen = False
            time.sleep(0.05)
            if getattr(device, "serialPort", None) is not None:
                device.serialPort.close()
            device.serialPort = None
        except Exception:
            pass

    print("-" * 72)
    if found:
        addrs = ", ".join(f"0x{a:02X}" for a, _ in found)
        print(f"[OK] Address terdeteksi: {addrs}")
    else:
        print("[WARN] Tidak ada sensor terdeteksi pada range tersebut.")


if __name__ == "__main__":
    main()
