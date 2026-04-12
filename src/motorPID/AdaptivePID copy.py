# coding: UTF-8
import sys, time, threading, json, os
import RPi.GPIO as GPIO

# ================= WT901 SDK =================
SDK_CHS = "/home/raspberrypi5/WitStandardModbus_WT901C485/Python/Python-SDK-WT901C485/chs"
sys.path.insert(0, SDK_CHS)

import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver

# ================= GPIO =================
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

# ================= LIMIT =================
MIN_AZ, MAX_AZ = -180, 180
MIN_EL, MAX_EL = 0, 90

# ================= HELPER =================
def clamp(v, a, b): return max(a, min(b, v))

def az_error_shortest(t, c):
    e = t - c
    return e - 360 if e > 180 else e + 360 if e < -180 else e

def min_pwm(out, m=18):
    return 0 if out == 0 else out + m if out > 0 else out - m

def gravity_comp(el):
    return 12 * (el / 90.0)

def print_status(s):
    sys.stdout.write("\r" + s + " " * 10)
    sys.stdout.flush()

# ================= ADAPTIVE PID =================
class AdaptivePID:
    def __init__(self):
        self.i = 0
        self.last_err = 0
        self.last_t = time.time()

    def gains(self, err):
        e = abs(err)
        if e > 10:
            return 9.0, 0.02, 1.2
        elif e > 3:
            return 8.0, 0.05, 2.0
        elif e > 0.8:
            return 6.0, 0.03, 3.0
        else:
            return 4.0, 0.0, 4.0   # LOCK ZONE

    def compute(self, err):
        now = time.time()
        dt = now - self.last_t
        self.last_t = now
        if dt <= 0: return 0

        kp, ki, kd = self.gains(err)

        self.i += err * dt
        self.i = clamp(self.i, -40, 40)

        d = (err - self.last_err) / dt
        self.last_err = err

        return kp * err + ki * self.i + kd * d

pid_az = AdaptivePID()
pid_el = AdaptivePID()

# ================= STATE =================
raw_az = raw_el = 0
az_off = el_off = 0
T_AZ = T_EL = 0
sdr_last = {}
_sdr_last_t = 0.0
_sdr_path = "/tmp/sdr_last.json"

def read_sdr_metrics():
    global sdr_last, _sdr_last_t
    t = time.time()
    if t - _sdr_last_t < 0.5:
        return
    _sdr_last_t = t
    try:
        if os.path.exists(_sdr_path):
            s = open(_sdr_path, "r").read()
            sdr_last = json.loads(s)
    except:
        pass

# ================= WT901 =================
def onUpdate(dev):
    global raw_az, raw_el
    raw_az = dev.getDeviceData("angleZ")
    raw_el = dev.getDeviceData("angleY")

def wt901_loop(dev):
    while True:
        dev.readReg(0x30, 41)
        time.sleep(0.01)

# ================= KEYBOARD =================
def keyboard():
    global az_off, el_off, T_AZ, T_EL
    print("\nCommands: c | t AZ EL | q")

    while True:
        print()
        cmd = input().strip().lower()

        if cmd == "c":
            az_off, el_off = raw_az, raw_el
            print("CALIBRATED")

        elif cmd.startswith("t"):
            try:
                _, a, e = cmd.split()
                T_AZ, T_EL = float(a), float(e)
                print(f"TARGET → AZ={T_AZ} EL={T_EL}")
            except:
                print("Format: t -30 30")

        elif cmd == "q":
            sys.exit(0)

# ================= MAIN =================
print("=== ADAPTIVE PID GIMBAL ===")

dev = deviceModel.DeviceModel(
    "WT901", Protocol485Resolver(),
    JY901SDataProcessor(), "51_0"
)
dev.ADDR = 0x50
dev.serialConfig.portName = "/dev/ttyUSB0"
dev.serialConfig.baud = 9600
dev.openDevice()
dev.dataProcessor.onVarChanged.append(onUpdate)

threading.Thread(target=wt901_loop, args=(dev,), daemon=True).start()
threading.Thread(target=keyboard, daemon=True).start()

try:
    while True:
        az = clamp(raw_az - az_off, MIN_AZ, MAX_AZ)
        el = clamp(raw_el - el_off, MIN_EL, MAX_EL)

        az_err = az_error_shortest(T_AZ, az)
        el_err = T_EL - el

        az_out = pid_az.compute(az_err)
        el_out = pid_el.compute(el_err)

        el_out += gravity_comp(el)

        az_out = min_pwm(clamp(az_out, -70, 70))
        el_out = min_pwm(clamp(el_out, -80, 80))

        if abs(az_err) < 0.2: az_out = 0
        if abs(el_err) < 0.2: el_out = 0

        GPIO.output(AZ_IN1, az_out > 0)
        GPIO.output(AZ_IN2, az_out < 0)
        az_pwm.ChangeDutyCycle(abs(az_out))

        GPIO.output(EL_IN1, el_out > 0)
        GPIO.output(EL_IN2, el_out < 0)
        el_pwm.ChangeDutyCycle(abs(el_out))

        read_sdr_metrics()
        sig = ""
        if sdr_last:
            pk = sdr_last.get("peak_power_db")
            pf = sdr_last.get("peak_freq_hz")
            sr = sdr_last.get("signal_strength_ratio")
            if pk is not None and pf is not None and sr is not None:
                sig = f"  SIG={pk:5.1f}dB @{pf/1e6:7.2f}MHz R={sr:0.2f}"
        print_status(f"STATUS → AZ={az:6.1f}  EL={el:6.1f}{sig}")
        time.sleep(0.02)

finally:
    GPIO.cleanup()
