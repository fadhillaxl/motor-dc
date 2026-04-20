import os
import sys
import math
import time
from datetime import datetime

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
PORT = "/dev/ttyUSB0"
BAUD = 9600
AZ_OFFSET_DEG = 0

alpha = 0.15
last_az = None


# =============================
# FILTER
# =============================
def lowpass(new, old):
    if new is None:
        return old
    if old is None:
        return new
    return old + alpha * (new - old)


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
    if heading < 0:
        heading += 360

    return heading


# =============================
# READ DATA
# =============================
def read_data(device):
    global last_az

    try:
        # ambil data dari SDK
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
        # FIX ORIENTASI SENSOR (TERBALIK)
        # =============================
        roll = 180.0 - roll
        pitch = -pitch

        if roll > 180:
            roll -= 360
        if roll < -180:
            roll += 360

        # =============================
        # FIX AXIS MAGNET
        # =============================
        if None not in (mx, my, mz):
            mx = float(mx)
            my = float(my)
            mz = float(mz)

            # hasil tuning kamu
            mx, my = my, mx
            mx = -mx

        # =============================
        # COMPASS
        # =============================
        compass = None
        if None not in (mx, my, mz):
            compass = tilt_compass(mx, my, mz, roll, pitch)

        # =============================
        # CONVERT CW AZIMUTH
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
            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)

            w = math.cos(roll_rad) * math.cos(pitch_rad)
            if w < 0:
                w = 0

            az = (1 - w) * yaw_cw + w * compass_cw
            src = f"BLEND({w:.2f})"

        # =============================
        # SMOOTHING
        # =============================
        az = lowpass(az, last_az)
        last_az = az

        return roll, pitch, yaw_cw, compass_cw, az, src

    except Exception as e:
        print("[ERROR]", e)
        return None


# =============================
# MAIN
# =============================
def main():
    print("=" * 80)
    print(" WT901C485 - SDK MODE (STABLE AZIMUTH)")
    print("=" * 80)

    # init device model
    device = deviceModel.DeviceModel(
        "WT901",
        Protocol485Resolver(),
        JY901SDataProcessor()
    )

    print("Opening serial...")
    device.openDevice(PORT, BAUD, 1)

    time.sleep(1)

    print("[OK] Connected\n")

    print("{:<10} {:>8} {:>8} {:>8} {:>10} {:>10} {:>10}".format(
        "TIME", "ROLL", "PITCH", "YAW", "COMPASS", "AZ", "SRC"
    ))
    print("-" * 80)

    while True:
        data = read_data(device)

        if data:
            roll, pitch, yaw, compass, az, src = data

            now = datetime.now().strftime("%H:%M:%S")

            print("{:<10} {:>8.2f} {:>8.2f} {:>8.2f} {:>10} {:>10.2f} {:>10}".format(
                now,
                roll,
                pitch,
                yaw,
                f"{compass:.2f}" if compass is not None else "-",
                az,
                src
            ))

        time.sleep(0.1)


# =============================
# RUN
# =============================
if __name__ == "__main__":
    main()