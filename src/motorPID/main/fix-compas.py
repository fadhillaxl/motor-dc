"""
WT901C485 - FINAL FIX (REAL DATA TUNED)
======================================
Fix:
- Sensor terbalik (menghadap bawah)
- Roll jadi positif
- Compass tidak lari ke 261°
- Stabil saat roll 90°
"""

import os
import sys
import time
import platform
import math

# =============================
# PATH SDK
# =============================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "Python-SDK-WT901C485", "chs"))
sys.path.insert(0, SDK_CHS)

# =============================
# IMPORT SDK
# =============================
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver

# =============================
# CONFIG
# =============================
INTERVAL = 0.1
TILT_THRESHOLD_DEG = 15.0
AZ_OFFSET_DEG = -104.0

# =============================
# DEVICE
# =============================
def buat_device_model():
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
            "EL_0",
        )

# =============================
# RESET
# =============================
def reset_zero_point(device):
    try:
        if hasattr(device, "write_register"):
            device.write_register(device.ADDR, 0x69, 0xB588)
            time.sleep(0.1)
            device.write_register(device.ADDR, 0x01, 0x0000)
        else:
            if hasattr(device, "unlock"):
                device.unlock()
                time.sleep(0.1)
            device.writeReg(0x01, 0x0000)
            if hasattr(device, "save"):
                time.sleep(0.1)
                device.save()
        time.sleep(0.3)
        print("[OK] Zero reset")
    except Exception as e:
        print("[WARN] Reset gagal:", e)

# =============================
# TILT COMPASS
# =============================
def tilt_compass(mx, my, mz, roll, pitch):
    try:
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)

        Xh = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
        Yh = (mx * math.sin(roll_rad) * math.sin(pitch_rad) +
              my * math.cos(roll_rad) -
              mz * math.sin(roll_rad) * math.cos(pitch_rad))

        heading = math.degrees(math.atan2(Yh, Xh))

        if heading < 0:
            heading += 360

        return heading
    except:
        return None

# =============================
# READ DATA
# =============================
def baca_sudut(device):
    try:
        if hasattr(device, "readReg"):
            device.readReg(0x30, 41)

        if hasattr(device, "get"):
            roll = device.get("AngleX")
            pitch = device.get("AngleY")
            yaw = device.get("AngleZ")
            mx = device.get("magX")
            my = device.get("magY")
            mz = device.get("magZ")
        else:
            roll = device.getDeviceData("angleX")
            pitch = device.getDeviceData("angleY")
            yaw = device.getDeviceData("angleZ")
            mx = device.getDeviceData("magX")
            my = device.getDeviceData("magY")
            mz = device.getDeviceData("magZ")

        if None in (roll, pitch, yaw):
            return None

        roll = float(roll)
        pitch = float(pitch)
        yaw = float(yaw) % 360

        # ======================================
        # FIX ORIENTASI SENSOR (FINAL DARI DATA)
        # ======================================
        roll = 180.0 - roll
        pitch = -pitch

        if roll > 180:
            roll -= 360
        if roll < -180:
            roll += 360

        # ======================================
        # FIX MAGNETOMETER (DARI DEBUG REAL)
        # ======================================
        if None not in (mx, my, mz):
            mx = float(mx)
            my = float(my)
            mz = float(mz)

            # swap + invert (hasil tuning dari data kamu)
            mx, my = my, mx
            mx = -mx

        # ======================================
        # COMPASS
        # ======================================
        compass = None
        if None not in (mx, my, mz):
            compass = tilt_compass(mx, my, mz, roll, pitch)

        # ======================================
        # CONVERT CW
        # ======================================
        yaw_cw = (360 - yaw + AZ_OFFSET_DEG) % 360
        compass_cw = (360 - compass + AZ_OFFSET_DEG) % 360 if compass is not None else None

        # ======================================
        # SMART SWITCH
        # ======================================
        if abs(roll) > TILT_THRESHOLD_DEG or abs(pitch) > TILT_THRESHOLD_DEG:
            az = compass_cw
            src = "COMPASS"
        else:
            az = yaw_cw
            src = "YAW"

        return roll, pitch, yaw_cw, compass_cw, az, src

    except:
        return None

# =============================
# UI
# =============================
def header():
    print("=" * 80)
    print(" WT901C485 - FINAL (FIXED & TUNED)")
    print("=" * 80)
    print(f"{'TIME':<10} {'ROLL':>8} {'PITCH':>8} {'YAW':>8} {'COMPASS':>10} {'AZ':>10} {'SRC':>8}")
    print("-" * 80)

# =============================
# MAIN
# =============================
def main():
    header()

    device = buat_device_model()
    device.ADDR = 0x50

    if platform.system().lower() == "linux":
        device.serialConfig.portName = "/dev/ttyUSB0"
    else:
        device.serialConfig.portName = "/dev/tty.usbserial-1330"

    device.serialConfig.baud = 9600
    device.openDevice()

    print("[OK] Connected\n")

    reset_zero_point(device)

    try:
        while True:
            data = baca_sudut(device)

            if data:
                roll, pitch, yaw, comp, az, src = data
                t = time.strftime("%H:%M:%S")

                print(f"{t:<10} {roll:>8.2f} {pitch:>8.2f} {yaw:>8.2f} "
                      f"{(comp if comp else 0):>10.2f} {az:>10.2f} {src:>8}")
            else:
                print("[WARN] No data")

            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n[STOP]")

    finally:
        device.closeDevice()
        print("[CLOSED]")

if __name__ == "__main__":
    main()