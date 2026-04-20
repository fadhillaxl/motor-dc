import math
import time
from datetime import datetime

# ==============================
# IMPORT LIBRARY WT901
# ==============================
from WitMotionSensor import WitMotionSensor

# ==============================
# CONFIG
# ==============================
PORT = "/dev/ttyUSB0"   # ganti sesuai device kamu
BAUD = 9600

AZ_OFFSET_DEG = 0       # kalibrasi arah (jika perlu)

# smoothing filter
alpha = 0.15
last_az = None


# ==============================
# LOW PASS FILTER
# ==============================
def lowpass(new, old):
    if new is None:
        return old
    if old is None:
        return new
    return old + alpha * (new - old)


# ==============================
# TILT COMPENSATED COMPASS
# ==============================
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


# ==============================
# READ SENSOR
# ==============================
def baca_sudut(device):
    global last_az

    try:
        device.readReg(0x30, 41)

        roll = device.get("AngleX")
        pitch = device.get("AngleY")
        yaw = device.get("AngleZ")

        mx = device.get("magX")
        my = device.get("magY")
        mz = device.get("magZ")

        if None in (roll, pitch, yaw):
            return None

        roll = float(roll)
        pitch = float(pitch)
        yaw = float(yaw) % 360

        # ==============================
        # FIX ORIENTASI (SENSOR TERBALIK)
        # ==============================
        roll = 180.0 - roll
        pitch = -pitch

        if roll > 180:
            roll -= 360
        if roll < -180:
            roll += 360

        # ==============================
        # FIX AXIS MAGNET
        # ==============================
        if None not in (mx, my, mz):
            mx = float(mx)
            my = float(my)
            mz = float(mz)

            mx, my = my, mx   # swap
            mx = -mx          # flip

        # ==============================
        # COMPASS
        # ==============================
        compass = None
        if None not in (mx, my, mz):
            compass = tilt_compass(mx, my, mz, roll, pitch)

        # ==============================
        # CONVERT TO AZIMUTH CW
        # ==============================
        yaw_cw = (360 - yaw + AZ_OFFSET_DEG) % 360
        compass_cw = (
            (360 - compass + AZ_OFFSET_DEG) % 360
            if compass is not None
            else None
        )

        # ==============================
        # 🔥 BLENDING (KUNCI STABIL)
        # ==============================
        az = yaw_cw
        src = "YAW"

        if compass_cw is not None:
            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)

            # weight berdasarkan tilt
            w = math.cos(roll_rad) * math.cos(pitch_rad)

            if w < 0:
                w = 0

            az = (1 - w) * yaw_cw + w * compass_cw
            src = f"BLEND({w:.2f})"

        # ==============================
        # 🔥 SMOOTHING
        # ==============================
        az = lowpass(az, last_az)
        last_az = az

        return roll, pitch, yaw_cw, compass_cw, az, src

    except Exception as e:
        print("[ERR]", e)
        return None


# ==============================
# MAIN
# ==============================
def main():
    print("=" * 80)
    print(" WT901C485 - FINAL (FIXED & STABLE)")
    print("=" * 80)

    device = WitMotionSensor(PORT, BAUD)
    device.openDevice()

    time.sleep(1)

    print("[OK] Connected\n")

    print("{:<12} {:>8} {:>8} {:>8} {:>10} {:>10} {:>10}".format(
        "TIME", "ROLL", "PITCH", "YAW", "COMPASS", "AZ", "SRC"
    ))
    print("-" * 80)

    while True:
        data = baca_sudut(device)

        if data:
            roll, pitch, yaw, compass, az, src = data

            now = datetime.now().strftime("%H:%M:%S")

            print("{:<12} {:>8.2f} {:>8.2f} {:>8.2f} {:>10} {:>10.2f} {:>10}".format(
                now,
                roll,
                pitch,
                yaw,
                f"{compass:.2f}" if compass is not None else "-",
                az,
                src
            ))

        time.sleep(0.1)


# ==============================
# RUN
# ==============================
if __name__ == "__main__":
    main()