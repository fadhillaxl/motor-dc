"""
WT901C485 - FINAL AZIMUTH STABLE (NO JUMP VERSION)
=================================================
✔ Tilt compensation (mag + acc)
✔ Blending yaw + compass
✔ FIX angle wrap (0-360)
✔ Smooth azimuth (circular filter)
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
AZ_OFFSET_DEG = 0.0

alpha = 0.15
last_az = None

# =============================
# ANGLE SAFE FILTER
# =============================
def angle_diff(a, b):
    """selisih sudut terpendek (-180..180)"""
    return (a - b + 180) % 360 - 180

def angle_lerp(new, old):
    """smooth tanpa loncat 0/360"""
    if old is None:
        return new
    d = angle_diff(new, old)
    return (old + alpha * d) % 360

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
            "AZ",
        )

# =============================
# RESET ZERO
# =============================
def reset_zero_point(device):
    try:
        print("[INFO] Reset zero-point...")
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
        print("[OK] Zero reset\n")
    except Exception as e:
        print("[WARN]", e)

# =============================
# TILT COMPASS
# =============================
def tilt_compass(mx, my, mz, roll, pitch):
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)

    Xh = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
    Yh = (
        mx * math.sin(roll_rad) * math.sin(pitch_rad)
        + my * math.cos(roll_rad)
        - mz * math.sin(roll_rad) * math.cos(pitch_rad)
    )

    heading = math.degrees(math.atan2(Yh, Xh))
    return (heading + 360) % 360

# =============================
# READ DATA
# =============================
def baca_sudut(device):
    global last_az

    try:
        if hasattr(device, "readReg"):
            device.readReg(0x30, 41)

        # ambil data
        if hasattr(device, "get"):
            roll = device.get("AngleX")
            pitch = device.get("AngleY")
            yaw = device.get("AngleZ")
            accX = device.get("accX")
            accY = device.get("accY")
            accZ = device.get("accZ")
            magX = device.get("magX")
            magY = device.get("magY")
            magZ = device.get("magZ")
        else:
            roll = device.getDeviceData("angleX")
            pitch = device.getDeviceData("angleY")
            yaw = device.getDeviceData("angleZ")
            accX = device.getDeviceData("accX")
            accY = device.getDeviceData("accY")
            accZ = device.getDeviceData("accZ")
            magX = device.getDeviceData("magX")
            magY = device.getDeviceData("magY")
            magZ = device.getDeviceData("magZ")

        if None in (roll, pitch, yaw):
            return None, None, None, None, None, None

        roll = float(roll)
        pitch = float(pitch)
        yaw = float(yaw) % 360

        # =============================
        # TILT dari ACC (lebih stabil)
        # =============================
        roll_tilt = roll
        pitch_tilt = pitch

        if None not in (accX, accY, accZ):
            ax = float(accX)
            ay = float(accY)
            az = float(accZ)

            if abs(ay) + abs(az) > 1e-6:
                roll_tilt = math.degrees(math.atan2(ay, az))

            if abs(ax) + abs(az) > 1e-6:
                pitch_tilt = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

        # =============================
        # COMPASS
        # =============================
        compass = None
        if None not in (magX, magY, magZ):
            compass = tilt_compass(
                float(magX),
                float(magY),
                float(magZ),
                roll_tilt,
                pitch_tilt
            )

        # =============================
        # CONVERT CW
        # =============================
        yaw_cw = (360 - yaw + AZ_OFFSET_DEG) % 360
        compass_cw = (
            (360 - compass + AZ_OFFSET_DEG) % 360
            if compass is not None else None
        )

        # =============================
        # BLENDING
        # =============================
        az = yaw_cw
        src = "YAW"

        if compass_cw is not None:
            w = math.cos(math.radians(roll_tilt)) * math.cos(math.radians(pitch_tilt))
            w = max(0, w)

            az = (1 - w) * yaw_cw + w * compass_cw
            src = f"BLEND({w:.2f})"

        # =============================
        # 🔥 FIX SMOOTH (NO JUMP)
        # =============================
        az = angle_lerp(az, last_az)
        last_az = az

        return roll, pitch, yaw_cw, compass_cw, az, src

    except Exception as e:
        print("[ERR]", e)
        return None, None, None, None, None, None

# =============================
# MAIN
# =============================
def main():
    print("="*80)
    print(" WT901C485 - FINAL AZIMUTH NO JUMP")
    print("="*80)

    device = buat_device_model()
    device.ADDR = 0x50

    if platform.system().lower() == "linux":
        device.serialConfig.portName = "/dev/ttyUSB0"
    else:
        device.serialConfig.portName = "/dev/tty.usbserial-1330"

    device.serialConfig.baud = 9600
    device.openDevice()

    time.sleep(1)
    print("[OK] Connected\n")

    reset_zero_point(device)

    print("{:<10} {:>8} {:>8} {:>8} {:>10} {:>10} {:>10}".format(
        "TIME", "ROLL", "PITCH", "YAW", "COMPASS", "AZ", "SRC"
    ))
    print("-"*80)

    try:
        while True:
            data = baca_sudut(device)

            if data[0] is not None:
                roll, pitch, yaw, comp, az, src = data
                now = time.strftime("%H:%M:%S")

                print("{:<10} {:>8.2f} {:>8.2f} {:>8.2f} {:>10} {:>10.2f} {:>10}".format(
                    now,
                    roll,
                    pitch,
                    yaw,
                    f"{comp:.2f}" if comp else "-",
                    az,
                    src
                ))

            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n[STOP]")

    finally:
        try:
            device.closeDevice()
        except:
            pass

# =============================
# RUN
# =============================
if __name__ == "__main__":
    main()