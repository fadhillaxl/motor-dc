"""
WT901C485 - DUAL SENSOR via address switching
One device object, one serial port, swap ADDR before each read
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

INTERVAL   = 0.3
AZ_OFFSET  = 0.0
alpha      = 0.15
last_az    = {0x51: None, 0x50: None}

# =============================
# MATH HELPERS
# =============================
def angle_diff(a, b):
    return (a - b + 180) % 360 - 180

def angle_lerp(new_val, old_val):
    if old_val is None:
        return new_val
    return (old_val + alpha * angle_diff(new_val, old_val)) % 360

def map_roll_to_el(roll):
    return max(0.0, min(180.0, float(roll) - 90.0))

def tilt_compass(mx, my, mz, roll, pitch):
    r, p = math.radians(roll), math.radians(pitch)
    Xh = mx * math.cos(p) + mz * math.sin(p)
    Yh = mx * math.sin(r)*math.sin(p) + my*math.cos(r) - mz*math.sin(r)*math.cos(p)
    return (math.degrees(math.atan2(Yh, Xh)) + 360) % 360

# =============================
# SINGLE DEVICE OBJECT
# =============================
def make_device(port, baud=9600):
    try:
        dev = deviceModel.DeviceModel("WT901C485", Protocol485Resolver(), JY901SDataProcessor())
    except TypeError:
        dev = deviceModel.DeviceModel("WT901C485", Protocol485Resolver(), JY901SDataProcessor(), "AZ")
    dev.serialConfig.portName = port
    dev.serialConfig.baud     = baud
    return dev

# =============================
# REQUEST DATA FOR ONE ADDRESS
# =============================
def request_sensor(device, addr):
    """Switch address, flush, request, wait for reply"""
    device.ADDR = addr
    try:
        if hasattr(device, "readReg"):
            device.readReg(0x30, 41)
        time.sleep(0.15)          # wait for async reply
    except Exception as e:
        print(f"[WARN readReg addr=0x{addr:02X}]", e)

# =============================
# PARSE CURRENT DEVICE DATA
# =============================
def parse_data(device, addr):
    global last_az
    try:
        def g(a, b):
            v = device.get(a) if hasattr(device, "get") else device.getDeviceData(b)
            return float(v) if v is not None else None

        roll  = g("AngleX", "angleX")
        pitch = g("AngleY", "angleY")
        yaw   = g("AngleZ", "angleZ")
        accX  = g("accX",   "accX")
        accY  = g("accY",   "accY")
        accZ  = g("accZ",   "accZ")
        magX  = g("magX",   "magX")
        magY  = g("magY",   "magY")
        magZ  = g("magZ",   "magZ")

        if None in (roll, pitch, yaw):
            return None

        yaw %= 360
        roll_t, pitch_t = roll, pitch

        if None not in (accX, accY, accZ):
            if abs(accY) + abs(accZ) > 1e-6:
                roll_t  = math.degrees(math.atan2(accY, accZ))
            if abs(accX) + abs(accZ) > 1e-6:
                pitch_t = math.degrees(math.atan2(-accX, math.sqrt(accY**2 + accZ**2)))

        compass_cw = None
        if None not in (magX, magY, magZ):
            compass_cw = (360 - tilt_compass(magX, magY, magZ, roll_t, pitch_t) + AZ_OFFSET) % 360

        yaw_cw = (360 - yaw + AZ_OFFSET) % 360
        az, src = yaw_cw, "YAW"

        if compass_cw is not None:
            w = max(0, math.cos(math.radians(roll_t)) * math.cos(math.radians(pitch_t)))
            az  = (1 - w) * yaw_cw + w * compass_cw
            src = f"BLEND({w:.2f})"

        az = angle_lerp(az, last_az[addr])
        last_az[addr] = az

        return dict(roll=roll, pitch=pitch, yaw=yaw_cw,
                    compass=compass_cw, az=az,
                    el=map_roll_to_el(roll), src=src)

    except Exception as e:
        print(f"[ERR parse addr=0x{addr:02X}]", e)
        return None

# =============================
# MAIN
# =============================
def main():
    print("=" * 90)
    print(" WT901C485 - DUAL SENSOR (address-switch mode)")
    print("=" * 90)

    PORT = "/dev/ttyUSB0" if platform.system().lower() == "linux" else "/dev/tty.usbserial-1330"
    SENSORS = [(0x01, "S1"), (0x02, "S2")]

    device = make_device(PORT)
    device.ADDR = 0x01          # default start address
    device.openDevice()
    time.sleep(1)
    print(f"[OK] Port {PORT} open\n")

    hdr = "{:<4} {:<10} {:>8} {:>8} {:>8} {:>10} {:>10} {:>8} {:>12}"
    print(hdr.format("SNS", "TIME", "ROLL", "PITCH", "YAW", "COMPASS", "AZ", "EL", "SRC"))
    print("-" * 90)

    row = "{:<4} {:<10} {:>8.2f} {:>8.2f} {:>8.2f} {:>10} {:>10.2f} {:>8.2f} {:>12}"

    try:
        while True:
            for addr, label in SENSORS:
                request_sensor(device, addr)
                d = parse_data(device, addr)
                if d:
                    print(row.format(
                        label,
                        time.strftime("%H:%M:%S"),
                        d["roll"], d["pitch"], d["yaw"],
                        f"{d['compass']:.2f}" if d["compass"] else "-",
                        d["az"], d["el"], d["src"]
                    ))
                else:
                    print(f"  [{label}] addr=0x{addr:02X} -- no data yet")

            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n[STOP]")
    finally:
        try:
            device.closeDevice()
        except:
            pass

if __name__ == "__main__":
    main()