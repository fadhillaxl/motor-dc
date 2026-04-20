"""
WT901C485 - FINAL PRO (AZ_USED OUTPUT)
=====================================
- Sensor terbalik FIX
- Magnetometer tuning OK
- Tilt compensation OK
- BLENDING (cos roll)
- Output AZ_USED (final azimuth)
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
# MATH HELPER
# =============================
def normalize_angle(a):
    return (a + 360) % 360

def angle_blend(a, b, w):
    """
    Blend 2 sudut tanpa loncat 0/360
    """
    a_rad = math.radians(a)
    b_rad = math.radians(b)

    x = w * math.cos(a_rad) + (1 - w) * math.cos(b_rad)
    y = w * math.sin(a_rad) + (1 - w) * math.sin(b_rad)

    return normalize_angle(math.degrees(math.atan2(y, x)))

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
        return normalize_angle(heading)
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

        # =============================
        # FIX ORIENTASI SENSOR
        # =============================
        roll = 180.0 - roll
        pitch = -pitch

        if roll > 180:
            roll -= 360
        if roll < -180:
            roll += 360

        # =============================
        # FIX MAGNETOMETER (TUNED)
        # =============================
        compass = None
        if None not in (mx, my, mz):
            mx = float(mx)
            my = float(my)
            mz = float(mz)

            mx, my = my, mx
            mx = -mx

            compass = tilt_compass(mx, my, mz, roll, pitch)

        # =============================
        # CONVERT CW + OFFSET
        # =============================
        yaw_cw = normalize_angle(360 - yaw + AZ_OFFSET_DEG)
        compass_cw = normalize_angle(360 - compass + AZ_OFFSET_DEG) if compass is not None else None

        # =============================
        # BLENDING (FINAL AZ_USED)
        # =============================
        if compass_cw is not None:
            w = abs(math.cos(math.radians(roll)))  # weight 0-1
            az_used = angle_blend(compass_cw, yaw_cw, w)
            src = f"COMP:{w:.2f} YAW:{1-w:.2f}"
        else:
            az_used = yaw_cw
            src = "YAW_ONLY"

        return roll, pitch, yaw_cw, compass_cw, az_used, src

    except:
        return None

# =============================
# UI
# =============================
def header():
    print("=" * 95)
    print(" WT901C485 - FINAL PRO (AZ_USED)")
    print("=" * 95)
    print(f"{'TIME':<10} {'ROLL':>8} {'PITCH':>8} {'YAW':>8} {'COMPASS':>10} {'AZ_USED':>12} {'SRC':>15}")
    print("-" * 95)

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
        device.serialConfig.portName = "/dev/tty.usbserial-110"

    device.serialConfig.baud = 9600
    device.openDevice()

    print("[OK] Connected\n")

    reset_zero_point(device)

    try:
        while True:
            data = baca_sudut(device)

            if data:
                roll, pitch, yaw, comp, az_used, src = data
                t = time.strftime("%H:%M:%S")

                comp_str = f"{comp:.2f}" if comp is not None else "N/A"

                print(f"{t:<10} {roll:>8.2f} {pitch:>8.2f} {yaw:>8.2f} "
                      f"{comp_str:>10} {az_used:>12.2f} {src:>15}")
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