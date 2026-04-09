# coding: UTF-8
import sys
import time
import threading
import platform

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

# ---------- AZIMUTH MOTOR ----------
AZ_EN  = 18
AZ_IN1 = 23
AZ_IN2 = 24

# ---------- ELEVATION MOTOR ----------
EL_EN  = 13
EL_IN1 = 5
EL_IN2 = 6

for p in [AZ_EN, AZ_IN1, AZ_IN2, EL_EN, EL_IN1, EL_IN2]:
    GPIO.setup(p, GPIO.OUT)

az_pwm = GPIO.PWM(AZ_EN, 1000)
el_pwm = GPIO.PWM(EL_EN, 1000)

az_pwm.start(0)
el_pwm.start(0)

# =====================================================
# MOTOR DRIVER
# =====================================================
def motor_drive(pwm, in1, in2, speed):
    speed = max(-100, min(100, speed))
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
# PID CONTROLLER
# =====================================================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0:
            return 0

        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error

        return (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

# ===== PID TUNING =====
pid_az = PID(kp=4.0, ki=0.0, kd=1.2)
pid_el = PID(kp=6.0, ki=0.0, kd=1.8)

# ===== TARGET =====
TARGET_AZ = 0.0
TARGET_EL = 0.0

# ===== RAW SENSOR =====
raw_az = 0.0
raw_el = 0.0

# ===== OFFSET (KALIBRASI) =====
az_offset = 0.0
el_offset = 0.0

# =====================================================
# WT901 CALLBACK
# =====================================================
def onUpdate(device):
    global raw_az, raw_el
    raw_az = device.getDeviceData("angleZ")  # Yaw
    raw_el = device.getDeviceData("angleY")  # Pitch

# =====================================================
# WT901 LOOP
# =====================================================
def wt901_loop(device):
    while True:
        device.readReg(0x30, 41)
        time.sleep(0.01)

# =====================================================
# KEYBOARD CALIBRATION THREAD
# =====================================================
def keyboard_loop():
    global az_offset, el_offset
    while True:
        cmd = input().strip().lower()
        if cmd == "c":
            az_offset = raw_az
            el_offset = raw_el
            print(f"\n>>> CALIBRATED: AZ=0°, EL=0° (offset set)")
        time.sleep(0.1)

# =====================================================
# MAIN
# =====================================================
print("=== AZIMUTH / ELEVATION CONTROL ===")
print("Press 'c' + Enter to CALIBRATE ZERO")

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
        # ===== APPLY OFFSET =====
        azimuth   = raw_az - az_offset
        elevation = raw_el - el_offset

        # ===== ERROR =====
        az_error = TARGET_AZ - azimuth
        el_error = TARGET_EL - elevation

        # ===== PID =====
        az_out = pid_az.compute(az_error)
        el_out = pid_el.compute(el_error)

        # ===== SAFETY =====
        az_out = max(-50, min(50, az_out))
        el_out = max(-60, min(60, el_out))

        # ===== DRIVE =====
        motor_drive(az_pwm, AZ_IN1, AZ_IN2, az_out)
        motor_drive(el_pwm, EL_IN1, EL_IN2, el_out)

        print(
            f"AZ={azimuth:6.2f}° PWM={az_out:6.1f} | "
            f"EL={elevation:6.2f}° PWM={el_out:6.1f}",
            end="\r"
        )

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nSTOP")

finally:
    motor_drive(az_pwm, AZ_IN1, AZ_IN2, 0)
    motor_drive(el_pwm, EL_IN1, EL_IN2, 0)
    device.closeDevice()
    az_pwm.stop()
    el_pwm.stop()
    GPIO.cleanup()
