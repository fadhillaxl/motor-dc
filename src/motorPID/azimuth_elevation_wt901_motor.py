# coding: UTF-8
import sys
import time
import threading

# =====================================================
# WT901 SDK PATH
# =====================================================
SDK_CHS = "/home/raspberrypi5/WitStandardModbus_WT901C485/Python/Python-SDK-WT901C485/chs"
sys.path.insert(0, SDK_CHS)

import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver

# =====================================================
# GPIO MOTOR
# =====================================================
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

AZ_EN, AZ_IN1, AZ_IN2 = 18, 23, 24
EL_EN, EL_IN1, EL_IN2 = 13, 5, 6

for p in [AZ_EN, AZ_IN1, AZ_IN2, EL_EN, EL_IN1, EL_IN2]:
    GPIO.setup(p, GPIO.OUT)

az_pwm = GPIO.PWM(AZ_EN, 1000)
el_pwm = GPIO.PWM(EL_EN, 1000)
az_pwm.start(0)
el_pwm.start(0)

# =====================================================
# LIMIT
# =====================================================
MIN_AZ, MAX_AZ = -180.0, 180.0
MIN_EL, MAX_EL = 0.0, 90.0

# =====================================================
# HELPER
# =====================================================
def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

def az_error_shortest(target, current):
    e = target - current
    if e > 180:
        e -= 360
    elif e < -180:
        e += 360
    return e

def deadzone(err, dz=0.1):
    return 0.0 if abs(err) < dz else err

def soft_scale(err, out):
    e = abs(err)
    if e < 0.8:
        return out * 0.9
    elif e < 2.0:
        return out * 1.1
    elif e < 5.0:
        return out * 1.2
    return out

def near_target_boost(err, out):
    e = abs(err)
    if 0.8 < e < 2.0:
        return out * 1.15
    return out

def min_pwm(out, m=18):
    if out == 0:
        return 0
    return out + m if out > 0 else out - m

def gravity_comp(el):
    return 12.0 * (el / 90.0)

def print_status(text):
    sys.stdout.write("\r" + text + " " * 10)
    sys.stdout.flush()

# =====================================================
# MOTOR DRIVER
# =====================================================
def motor_drive(pwm, in1, in2, speed):
    speed = clamp(speed, -100, 100)
    if speed > 0:
        GPIO.output(in1, 1)
        GPIO.output(in2, 0)
        pwm.ChangeDutyCycle(speed)
    elif speed < 0:
        GPIO.output(in1, 0)
        GPIO.output(in2, 1)
        pwm.ChangeDutyCycle(-speed)
    else:
        pwm.ChangeDutyCycle(0)

# =====================================================
# PID
# =====================================================
class PID:
    def __init__(self, kp, ki, kd, ilim=60):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()
        self.ilim = ilim

    def compute(self, err):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0:
            return 0

        self.integral += err * dt
        self.integral = clamp(self.integral, -self.ilim, self.ilim)

        d = (err - self.last_error) / dt
        self.last_error = err

        return self.kp * err + self.ki * self.integral + self.kd * d

# 🔧 FINAL PID (ANTI OVERSHOOT)
pid_az = PID(8.75, 0.05, 2.2)
pid_el = PID(9.0,  0.05, 2.4)

# =====================================================
# STATE
# =====================================================
raw_az = 0.0
raw_el = 0.0
az_offset = 0.0
el_offset = 0.0
TARGET_AZ = 0.0
TARGET_EL = 0.0

# =====================================================
# WT901 CALLBACK
# =====================================================
def onUpdate(dev):
    global raw_az, raw_el
    raw_az = dev.getDeviceData("angleZ")
    raw_el = dev.getDeviceData("angleY")

# =====================================================
# WT901 THREAD
# =====================================================
def wt901_loop(dev):
    while True:
        dev.readReg(0x30, 41)
        time.sleep(0.01)

# =====================================================
# KEYBOARD
# =====================================================
def keyboard_loop():
    global az_offset, el_offset, TARGET_AZ, TARGET_EL
    print("\nReady: c | t AZ EL | q")

    while True:
        sys.stdout.write("\n")
        sys.stdout.flush()
        cmd = input().strip().lower()

        if cmd == "c":
            az_offset = raw_az
            el_offset = raw_el
            print("CALIBRATED → AZ=0  EL=0")

        elif cmd.startswith("t"):
            try:
                _, a, e = cmd.split()
                TARGET_AZ = clamp(float(a), MIN_AZ, MAX_AZ)
                TARGET_EL = clamp(float(e), MIN_EL, MAX_EL)
                print(f"TARGET → AZ={TARGET_AZ:.1f}  EL={TARGET_EL:.1f}")
            except:
                print("Format: t -30 30")

        elif cmd == "q":
            sys.exit(0)

# =====================================================
# MAIN
# =====================================================
print("=== AZIMUTH / ELEVATION CONTROLLER ===")

device = deviceModel.DeviceModel(
    "WT901",
    Protocol485Resolver(),
    JY901SDataProcessor(),
    "51_0"
)

device.ADDR = 0x50
device.serialConfig.portName = "/dev/ttyUSB0"
device.serialConfig.baud = 9600

device.openDevice()
device.dataProcessor.onVarChanged.append(onUpdate)

threading.Thread(target=wt901_loop, args=(device,), daemon=True).start()
threading.Thread(target=keyboard_loop, daemon=True).start()

try:
    while True:
        az = clamp(raw_az - az_offset, MIN_AZ, MAX_AZ)
        el = clamp(raw_el - el_offset, MIN_EL, MAX_EL)

        az_err = deadzone(az_error_shortest(TARGET_AZ, az), 0.1)
        el_err = deadzone(TARGET_EL - el, 0.1)

        az_out = pid_az.compute(az_err)
        el_out = pid_el.compute(el_err)

        az_out = near_target_boost(az_err, soft_scale(az_err, az_out))
        el_out = near_target_boost(el_err, soft_scale(el_err, el_out))

        el_out += gravity_comp(el)

        az_out = min_pwm(az_out)
        el_out = min_pwm(el_out)

        # 🔒 SOFT LOCK (NO OVERSHOOT)
        if abs(az_err) < 0.2:
            az_out = 0
        if abs(el_err) < 0.2:
            el_out = 0

        az_out = clamp(az_out, -70, 70)
        el_out = clamp(el_out, -80, 80)

        motor_drive(az_pwm, AZ_IN1, AZ_IN2, az_out)
        motor_drive(el_pwm, EL_IN1, EL_IN2, el_out)

        print_status(
            f"STATUS → AZ={az:6.1f}  EL={el:6.1f}"
        )

        time.sleep(0.02)

except KeyboardInterrupt:
    pass
finally:
    motor_drive(az_pwm, AZ_IN1, AZ_IN2, 0)
    motor_drive(el_pwm, EL_IN1, EL_IN2, 0)
    device.closeDevice()
    az_pwm.stop()
    el_pwm.stop()
    GPIO.cleanup()
