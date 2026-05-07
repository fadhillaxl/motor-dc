"""
WT901C485 - AZIMUTH YAW ONLY (ABSOLUTE OFFSET)
==============================================
✔ AZ pakai YAW sensor AZ (0x51)
✔ Offset AZ untuk sinkron heading absolut
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
INTERVAL = 0.05
AZ_ADDR = 0x51
EL_ADDR = 0x50
AZ_OFFSET_DEG = 140.0
EL_OFFSET_DEG = 0.0

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


def map_roll_to_el(roll_deg, el_offset_deg=0.0):
    """
    Mapping EL dari roll:
    - roll ~= 90  -> EL = 0 (depan datar)
    - roll ~= 180 -> EL = 90 (menghadap atas)
    """
    el = (float(roll_deg) - 90.0) + float(el_offset_deg)
    return max(0.0, min(180.0, el))

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
def baca_sudut(device, addr, smooth_az=True):
    global last_az

    try:
        device.ADDR = int(addr)
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
            return None, None, None, None, None, None, None

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
        # AZIMUTH: YAW ONLY (ABSOLUTE)
        # =============================
        az = yaw_cw
        src = "YAW"

        # =============================
        # 🔥 FIX SMOOTH (NO JUMP)
        # =============================
        if smooth_az:
            az = angle_lerp(az, last_az)
            last_az = az
        el = map_roll_to_el(roll, EL_OFFSET_DEG)

        return roll, pitch, yaw_cw, compass_cw, az, el, src

    except Exception as e:
        print("[ERR]", e)
        return None, None, None, None, None, None, None

# =============================
# MAIN
# =============================
def main():
    print("="*80)
    print(" WT901C485 - AZIMUTH YAW ONLY (ABS OFFSET)")
    print("="*80)

    device = buat_device_model()
    device.ADDR = AZ_ADDR

    if platform.system().lower() == "linux":
        device.serialConfig.portName = "/dev/ttyUSB0"
    else:
        device.serialConfig.portName = "/dev/tty.usbserial-1330"

    device.serialConfig.baud = 9600
    device.openDevice()

    time.sleep(1)
    print("[OK] Connected\n")

    print(f"[INFO] Dual WT901 mode (no auto reset): AZ=0x{AZ_ADDR:02X}, EL=0x{EL_ADDR:02X}")
    print(f"[INFO] Source mapping: YAW/AZ <- 0x{AZ_ADDR:02X}, EL(roll) <- 0x{EL_ADDR:02X}")

    # Fail fast startup validation
    az_boot = None
    el_boot = None
    for _ in range(30):
        az_boot = baca_sudut(device, AZ_ADDR, smooth_az=False)
        if az_boot[0] is not None:
            break
        time.sleep(0.05)
    for _ in range(30):
        el_boot = baca_sudut(device, EL_ADDR, smooth_az=False)
        if el_boot[0] is not None:
            break
        time.sleep(0.05)
    if az_boot is None or az_boot[0] is None:
        raise RuntimeError(f"Gagal membaca sensor AZ di address 0x{AZ_ADDR:02X}")
    if el_boot is None or el_boot[0] is None:
        raise RuntimeError(f"Gagal membaca sensor EL di address 0x{EL_ADDR:02X}")

    print("{:<10} {:>8} {:>8} {:>8} {:>10} {:>10} {:>8} {:>8} {:>10} {:>6} {:>6}".format(
        "TIME", "R_AZ", "P_AZ", "YAW", "COMPASS", "AZ", "R_EL", "EL", "SRC", "AZ@", "EL@"
    ))
    print("-"*80)

    try:
        while True:
            az_data = baca_sudut(device, AZ_ADDR, smooth_az=True)
            el_data = baca_sudut(device, EL_ADDR, smooth_az=False)

            if az_data[0] is not None and el_data[0] is not None:
                az_roll, az_pitch, yaw, comp, az, _, src = az_data
                el_roll, _, _, _, _, el, _ = el_data
                now = time.strftime("%H:%M:%S")

                print("{:<10} {:>8.2f} {:>8.2f} {:>8.2f} {:>10} {:>10.2f} {:>8.2f} {:>8.2f} {:>10} {:>6} {:>6}".format(
                    now,
                    az_roll,
                    az_pitch,
                    yaw,
                    f"{comp:.2f}" if comp is not None else "-",
                    az,
                    el_roll,
                    el,
                    src,
                    f"0x{AZ_ADDR:02X}",
                    f"0x{EL_ADDR:02X}",
                ))
            else:
                print("[WARN] Read failed on AZ/EL address")

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
