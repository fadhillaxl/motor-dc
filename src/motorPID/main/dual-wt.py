"""
WT901C485 - DUAL SENSOR (Sensor 1: 0x01, Sensor 2: 0x02)
"""

import os
import sys
import time
import platform
import math

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "Python-SDK-WT901C485", "chs"))
sys.path.insert(0, SDK_CHS)

import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver

INTERVAL = 0.2
AZ_OFFSET_DEG = 0.0
alpha = 0.15

# =============================
# SEPARATE STATE PER SENSOR
# =============================
last_az = {0x01: None, 0x02: None}

def angle_diff(a, b):
    return (a - b + 180) % 360 - 180

def angle_lerp(new, old):
    if old is None:
        return new
    d = angle_diff(new, old)
    return (old + alpha * d) % 360

def map_roll_to_el(roll_deg, el_offset_deg=0.0):
    el = (float(roll_deg) - 90.0) + float(el_offset_deg)
    return max(0.0, min(180.0, el))

def tilt_compass(mx, my, mz, roll, pitch):
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    Xh = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
    Yh = (
        mx * math.sin(roll_rad) * math.sin(pitch_rad)
        + my * math.cos(roll_rad)
        - mz * math.sin(roll_rad) * math.cos(pitch_rad)
    )
    return (math.degrees(math.atan2(Yh, Xh)) + 360) % 360

# =============================
# CREATE ONE DEVICE PER SENSOR
# =============================
def buat_device(name, addr, port, baud=9600):
    try:
        dev = deviceModel.DeviceModel(
            name,
            Protocol485Resolver(),
            JY901SDataProcessor(),
        )
    except TypeError:
        dev = deviceModel.DeviceModel(
            name,
            Protocol485Resolver(),
            JY901SDataProcessor(),
            "AZ",
        )
    dev.ADDR = addr
    dev.serialConfig.portName = port
    dev.serialConfig.baud = baud
    return dev

# =============================
# READ DATA (addr-aware)
# =============================
def baca_sudut(device, addr):
    global last_az
    try:
        if hasattr(device, "readReg"):
            device.readReg(0x30, 41)

        def get(key, fallback):
            if hasattr(device, "get"):
                return device.get(key)
            return device.getDeviceData(fallback)

        roll  = get("AngleX", "angleX")
        pitch = get("AngleY", "angleY")
        yaw   = get("AngleZ", "angleZ")
        accX  = get("accX", "accX")
        accY  = get("accY", "accY")
        accZ  = get("accZ", "accZ")
        magX  = get("magX", "magX")
        magY  = get("magY", "magY")
        magZ  = get("magZ", "magZ")

        if None in (roll, pitch, yaw):
            return None, None, None, None, None, None, None

        roll  = float(roll)
        pitch = float(pitch)
        yaw   = float(yaw) % 360

        roll_tilt = roll
        pitch_tilt = pitch

        if None not in (accX, accY, accZ):
            ax, ay, az = float(accX), float(accY), float(accZ)
            if abs(ay) + abs(az) > 1e-6:
                roll_tilt = math.degrees(math.atan2(ay, az))
            if abs(ax) + abs(az) > 1e-6:
                pitch_tilt = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

        compass = None
        if None not in (magX, magY, magZ):
            compass = tilt_compass(float(magX), float(magY), float(magZ), roll_tilt, pitch_tilt)

        yaw_cw = (360 - yaw + AZ_OFFSET_DEG) % 360
        compass_cw = ((360 - compass + AZ_OFFSET_DEG) % 360) if compass is not None else None

        az  = yaw_cw
        src = "YAW"

        if compass_cw is not None:
            w = max(0, math.cos(math.radians(roll_tilt)) * math.cos(math.radians(pitch_tilt)))
            az  = (1 - w) * yaw_cw + w * compass_cw
            src = f"BLEND({w:.2f})"

        az = angle_lerp(az, last_az[addr])
        last_az[addr] = az
        el = map_roll_to_el(roll)

        return roll, pitch, yaw_cw, compass_cw, az, el, src

    except Exception as e:
        print(f"[ERR addr=0x{addr:02X}]", e)
        return None, None, None, None, None, None, None

# =============================
# MAIN
# =============================
def main():
    print("="*90)
    print(" WT901C485 - DUAL SENSOR")
    print("="*90)

    PORT = "/dev/ttyUSB0" if platform.system().lower() == "linux" else "/dev/tty.usbserial-1330"

    # Both sensors share same physical port (RS485 bus)
    sensor1 = buat_device("S1", addr=0x01, port=PORT)
    sensor2 = buat_device("S2", addr=0x02, port=PORT)

    sensor1.openDevice()
    time.sleep(0.5)
    sensor2.openDevice()
    time.sleep(0.5)

    print("[OK] Both sensors connected\n")

    hdr = "{:<6} {:<10} {:>8} {:>8} {:>8} {:>10} {:>10} {:>8} {:>10}"
    print(hdr.format("SENSOR", "TIME", "ROLL", "PITCH", "YAW", "COMPASS", "AZ", "EL", "SRC"))
    print("-"*90)

    row = "{:<6} {:<10} {:>8.2f} {:>8.2f} {:>8.2f} {:>10} {:>10.2f} {:>8.2f} {:>10}"

    try:
        while True:
            for label, dev, addr in [("S1", sensor1, 0x01), ("S2", sensor2, 0x02)]:
                data = baca_sudut(dev, addr)
                if data[0] is not None:
                    roll, pitch, yaw, comp, az, el, src = data
                    now = time.strftime("%H:%M:%S")
                    print(row.format(
                        label, now,
                        roll, pitch, yaw,
                        f"{comp:.2f}" if comp else "-",
                        az, el, src
                    ))
            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n[STOP]")
    finally:
        for dev in [sensor1, sensor2]:
            try:
                dev.closeDevice()
            except:
                pass

if __name__ == "__main__":
    main()