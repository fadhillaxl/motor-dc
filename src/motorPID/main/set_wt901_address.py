#!/usr/bin/env python3
# coding: utf-8
"""
Set WT901 RS485 address (single device on bus).

Contoh:
  python set_wt901_address.py --current 0x50 --new 0x51
  python set_wt901_address.py --current 80 --new 81 --port /dev/ttyUSB0
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


ADDR_REG = 0x50  # register address for slave ID on RS485 variants


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
            "ADDR_SETTER",
        )


def write_new_address(device, current_addr: int, new_addr: int):
    # Prefer API style from chs SDK if available.
    if hasattr(device, "unlock"):
        device.unlock()
        time.sleep(0.1)

    if hasattr(device, "writeReg"):
        device.writeReg(ADDR_REG, int(new_addr))
    elif hasattr(device, "write_register"):
        device.write_register(int(current_addr), ADDR_REG, int(new_addr))
    else:
        raise RuntimeError("SDK method writeReg/write_register tidak ditemukan.")

    time.sleep(0.1)
    if hasattr(device, "save"):
        device.save()
    time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description="Set WT901 RS485 address")
    parser.add_argument("--current", required=True, type=parse_int_auto, help="Alamat saat ini (contoh: 0x50)")
    parser.add_argument("--new", required=True, type=parse_int_auto, help="Alamat baru (contoh: 0x51)")
    parser.add_argument("--port", default=None, help="Serial port (default auto)")
    parser.add_argument("--baud", type=int, default=9600, help="Baudrate serial (default: 9600)")
    args = parser.parse_args()

    current_addr = int(args.current)
    new_addr = int(args.new)
    if not (0 <= current_addr <= 0xFF and 0 <= new_addr <= 0xFF):
        raise SystemExit("Alamat harus di range 0..255 (0x00..0xFF).")

    port = args.port
    if not port:
        port = "/dev/ttyUSB0" if platform.system().lower() == "linux" else "/dev/tty.usbserial-1330"

    print("=" * 72)
    print(" WT901 RS485 ADDRESS SETTER")
    print("=" * 72)
    print(f"[INFO] Port={port} Baud={args.baud} Current=0x{current_addr:02X} New=0x{new_addr:02X}")
    print("[INFO] Pastikan hanya 1 sensor terhubung saat set address.")

    device = build_device()
    device.ADDR = current_addr
    device.serialConfig.portName = port
    device.serialConfig.baud = int(args.baud)

    try:
        device.openDevice()
        time.sleep(0.6)
        write_new_address(device, current_addr=current_addr, new_addr=new_addr)
        print(f"[OK] Address berhasil ditulis: 0x{current_addr:02X} -> 0x{new_addr:02X}")
        print("[NOTE] Power-cycle sensor, lalu gunakan alamat baru untuk komunikasi.")
    except Exception as exc:
        print(f"[ERR] Gagal set address: {exc}")
        sys.exit(1)
    finally:
        try:
            device.closeDevice()
        except Exception:
            pass


if __name__ == "__main__":
    main()
