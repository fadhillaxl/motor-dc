#!/usr/bin/env python3
# coding: utf-8
"""
TB6600 Stepper Keyboard Controller

Fitur utama:
- Kontrol STEP / DIR / ENABLE untuk driver TB6600.
- Dukungan microstepping: 1, 2, 4, 8, 16.
- Arah CW/CCW dengan kecepatan yang bisa diatur dari keyboard.
- Profil percepatan/deselerasi smooth (linear ramp).
- Safety:
  - Limit switch minimum dan maksimum.
  - Input proteksi arus berlebih (digital fault input).
  - Emergency stop dari keyboard dan dari pin GPIO.
- Feedback posisi:
  - Posisi disimpan dalam "full-step equivalent" agar tetap akurat walau
    microstepping diubah saat runtime.

Mapping pin sesuai request:
- Motor 1:
  - PUL+ -> GPIO17 (Pin 11)
  - DIR+ -> GPIO27 (Pin 13)
  - EN+  -> GPIO22 (Pin 15)
- Motor 2:
  - PUL+ -> GPIO23 (Pin 16)
  - DIR+ -> GPIO24 (Pin 18)
  - EN+  -> GPIO25 (Pin 22)
- PUL- / DIR- / EN- -> GND (sesuai wiring driver)

Catatan penting hardware:
- TB6600 biasanya memiliki pin ENA/EN+, ENA-, DIR+/DIR-, PUL+/PUL-.
- Sesuaikan wiring active-high / active-low melalui parameter *_active_high.
- Input overcurrent diasumsikan berasal dari sensor/comparator eksternal
  yang memberikan sinyal digital fault.
"""

import sys
import time
import tty
import termios
import threading
import os
import math
import select
import platform
from dataclasses import dataclass

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "Python-SDK-WT901C485", "chs"))
sys.path.insert(0, SDK_CHS)

import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver

try:
    import RPi.GPIO as GPIO
except Exception as exc:
    print(f"ERROR: gagal import RPi.GPIO: {exc}")
    print("Jalankan file ini di Raspberry Pi dengan library RPi.GPIO terpasang.")
    sys.exit(1)

INTERVAL = 0.1
AZ_OFFSET_DEG = 0.0
KEY_HOLD_TIMEOUT = 0.18

alpha = 0.15
last_az = None


def angle_diff(a, b):
    return (a - b + 180) % 360 - 180


def angle_lerp(new, old):
    if old is None:
        return new
    d = angle_diff(new, old)
    return (old + alpha * d) % 360


def tilt_compass(mx, my, mz, roll, pitch):
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)

    xh = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
    yh = (
        mx * math.sin(roll_rad) * math.sin(pitch_rad)
        + my * math.cos(roll_rad)
        - mz * math.sin(roll_rad) * math.cos(pitch_rad)
    )

    heading = math.degrees(math.atan2(yh, xh))
    return (heading + 360) % 360


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


def baca_sudut(device):
    global last_az

    try:
        if hasattr(device, "readReg"):
            device.readReg(0x30, 41)

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

        roll_tilt = roll
        pitch_tilt = pitch

        if None not in (accX, accY, accZ):
            ax = float(accX)
            ay = float(accY)
            az = float(accZ)

            if abs(ay) + abs(az) > 1e-6:
                roll_tilt = math.degrees(math.atan2(ay, az))

            if abs(ax) + abs(az) > 1e-6:
                pitch_tilt = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        compass = None
        if None not in (magX, magY, magZ):
            compass = tilt_compass(
                float(magX),
                float(magY),
                float(magZ),
                roll_tilt,
                pitch_tilt,
            )

        yaw_cw = (360 - yaw + AZ_OFFSET_DEG) % 360
        compass_cw = (360 - compass + AZ_OFFSET_DEG) % 360 if compass is not None else None

        az = yaw_cw
        src = "YAW"

        if compass_cw is not None:
            w = math.cos(math.radians(roll_tilt)) * math.cos(math.radians(pitch_tilt))
            w = max(0, w)
            az = (1 - w) * yaw_cw + w * compass_cw
            src = f"BLEND({w:.2f})"

        az = angle_lerp(az, last_az)
        last_az = az

        return roll, pitch, yaw_cw, compass_cw, az, src
    except Exception:
        return None, None, None, None, None, None


@dataclass
class StepperConfig:
    # TB6600 control pins
    step_pin: int = 18
    dir_pin: int = 23
    en_pin: int = 24

    # Optional microstep select pins (jika TB6600 board expose DIP via GPIO bridge)
    ms1_pin: int | None = None
    ms2_pin: int | None = None
    ms3_pin: int | None = None

    # Safety inputs (optional; None = nonaktif)
    limit_min_pin: int | None = None
    limit_max_pin: int | None = None
    overcurrent_pin: int | None = None
    estop_pin: int | None = None

    # Logic polarity
    en_active_high: bool = False
    dir_active_high: bool = True
    step_active_high: bool = True
    limit_active_low: bool = True
    overcurrent_active_low: bool = False
    estop_active_low: bool = True

    # Motor tuning
    steps_per_rev: int = 200  # full steps (1.8 deg/step)
    microstep: int = 8
    min_microstep: int = 1
    max_microstep: int = 16
    max_speed_sps: float = 2200.0  # pulses per second
    accel_sps2: float = 3000.0     # pulses per second^2
    pulse_width_us: int = 8

    # Soft-limit in degree (optional, None untuk nonaktif)
    soft_limit_min_deg: float | None = 0.0
    soft_limit_max_deg: float | None = 360.0


class TB6600Stepper:
    SUPPORTED_MICROSTEPS = (1, 2, 4, 8, 16)

    # Mapping umum DIP -> (MS1, MS2, MS3)
    # Bisa berbeda antar board, sesuaikan jika perlu.
    MICROSTEP_GPIO_MAP = {
        1:  (0, 0, 0),
        2:  (1, 0, 0),
        4:  (0, 1, 0),
        8:  (1, 1, 0),
        16: (1, 1, 1),
    }

    def __init__(self, cfg: StepperConfig):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._run = True
        self._fault_latched = False
        self._fault_msg = ""

        # Dynamic motion state
        self._target_speed_sps = 0.0  # signed, +CW / -CCW
        self._current_speed_sps = 0.0
        self._position_full_steps = 0.0

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self._setup_gpio()
        self.enable_driver(True)
        self.set_microstep(cfg.microstep)

        self._thread = threading.Thread(target=self._motion_loop, daemon=True)
        self._thread.start()

    # ---------------- GPIO and utility ----------------
    def _setup_gpio(self):
        GPIO.setup(self.cfg.step_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.cfg.dir_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.cfg.en_pin, GPIO.OUT, initial=GPIO.LOW)

        if self.cfg.ms1_pin is not None:
            GPIO.setup(self.cfg.ms1_pin, GPIO.OUT, initial=GPIO.LOW)
        if self.cfg.ms2_pin is not None:
            GPIO.setup(self.cfg.ms2_pin, GPIO.OUT, initial=GPIO.LOW)
        if self.cfg.ms3_pin is not None:
            GPIO.setup(self.cfg.ms3_pin, GPIO.OUT, initial=GPIO.LOW)

        if self.cfg.limit_min_pin is not None:
            GPIO.setup(self.cfg.limit_min_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if self.cfg.limit_max_pin is not None:
            GPIO.setup(self.cfg.limit_max_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if self.cfg.overcurrent_pin is not None:
            GPIO.setup(self.cfg.overcurrent_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        if self.cfg.estop_pin is not None:
            GPIO.setup(self.cfg.estop_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def _safe_input(self, pin: int) -> int:
        try:
            return GPIO.input(pin)
        except Exception:
            return 0

    def _is_active(self, raw: int, active_low: bool) -> bool:
        return (raw == GPIO.LOW) if active_low else (raw == GPIO.HIGH)

    def _set_output(self, pin: int, state: bool):
        GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)

    # ---------------- Driver controls ----------------
    def enable_driver(self, enabled: bool):
        # enabled=True -> aktifkan driver
        if self.cfg.en_active_high:
            self._set_output(self.cfg.en_pin, enabled)
        else:
            self._set_output(self.cfg.en_pin, not enabled)

    def set_microstep(self, microstep: int):
        if microstep not in self.SUPPORTED_MICROSTEPS:
            raise ValueError("Microstep harus salah satu dari: 1, 2, 4, 8, 16")
        with self._lock:
            self.cfg.microstep = microstep
            if (
                self.cfg.ms1_pin is not None
                and self.cfg.ms2_pin is not None
                and self.cfg.ms3_pin is not None
            ):
                s1, s2, s3 = self.MICROSTEP_GPIO_MAP[microstep]
                self._set_output(self.cfg.ms1_pin, bool(s1))
                self._set_output(self.cfg.ms2_pin, bool(s2))
                self._set_output(self.cfg.ms3_pin, bool(s3))

    def set_target_speed(self, speed_sps: float):
        with self._lock:
            lim = max(0.0, float(self.cfg.max_speed_sps))
            self._target_speed_sps = max(-lim, min(lim, float(speed_sps)))

    def stop_smooth(self):
        with self._lock:
            self._target_speed_sps = 0.0

    def emergency_stop(self, reason: str = "Emergency stop"):
        with self._lock:
            self._target_speed_sps = 0.0
            self._current_speed_sps = 0.0
            self._fault_latched = True
            self._fault_msg = reason
        self.enable_driver(False)

    def reset_fault(self):
        with self._lock:
            self._fault_latched = False
            self._fault_msg = ""
            self._target_speed_sps = 0.0
            self._current_speed_sps = 0.0
        self.enable_driver(True)

    def get_position_steps(self) -> float:
        with self._lock:
            return float(self._position_full_steps)

    def get_position_deg(self) -> float:
        with self._lock:
            return (self._position_full_steps / self.cfg.steps_per_rev) * 360.0

    def get_status(self) -> dict:
        with self._lock:
            return {
                "target_speed_sps": self._target_speed_sps,
                "current_speed_sps": self._current_speed_sps,
                "microstep": self.cfg.microstep,
                "position_deg": (self._position_full_steps / self.cfg.steps_per_rev) * 360.0,
                "fault_latched": self._fault_latched,
                "fault_msg": self._fault_msg,
            }

    # ---------------- Safety and motion ----------------
    def _check_hard_safety(self):
        if (
            self.cfg.estop_pin is not None
            and self._is_active(self._safe_input(self.cfg.estop_pin), self.cfg.estop_active_low)
        ):
            self.emergency_stop("E-STOP input aktif")
            return
        if (
            self.cfg.overcurrent_pin is not None
            and self._is_active(self._safe_input(self.cfg.overcurrent_pin), self.cfg.overcurrent_active_low)
        ):
            self.emergency_stop("Proteksi arus berlebih aktif")

    def _is_limit_triggered(self, moving_positive: bool) -> bool:
        if self.cfg.limit_min_pin is None or self.cfg.limit_max_pin is None:
            return False
        if moving_positive:
            return self._is_active(self._safe_input(self.cfg.limit_max_pin), self.cfg.limit_active_low)
        return self._is_active(self._safe_input(self.cfg.limit_min_pin), self.cfg.limit_active_low)

    def _soft_limit_reached(self, next_deg: float) -> bool:
        mn = self.cfg.soft_limit_min_deg
        mx = self.cfg.soft_limit_max_deg
        if mn is not None and next_deg < mn:
            return True
        if mx is not None and next_deg > mx:
            return True
        return False

    def _set_direction(self, cw: bool):
        out = cw if self.cfg.dir_active_high else (not cw)
        self._set_output(self.cfg.dir_pin, out)

    def _pulse_step(self):
        hi = self.cfg.step_active_high
        lo = not hi
        self._set_output(self.cfg.step_pin, hi)
        time.sleep(self.cfg.pulse_width_us / 1_000_000.0)
        self._set_output(self.cfg.step_pin, lo)

    def _motion_loop(self):
        last_t = time.perf_counter()
        next_pulse_t = last_t

        while self._run:
            now = time.perf_counter()
            dt = now - last_t
            last_t = now

            self._check_hard_safety()

            with self._lock:
                if self._fault_latched:
                    self._current_speed_sps = 0.0
                    time.sleep(0.01)
                    continue

                # Ramp speed menuju target
                a = max(1.0, float(self.cfg.accel_sps2))
                delta = a * dt
                if self._current_speed_sps < self._target_speed_sps:
                    self._current_speed_sps = min(self._current_speed_sps + delta, self._target_speed_sps)
                elif self._current_speed_sps > self._target_speed_sps:
                    self._current_speed_sps = max(self._current_speed_sps - delta, self._target_speed_sps)

                spd = self._current_speed_sps
                microstep = self.cfg.microstep

            if abs(spd) < 1e-3:
                time.sleep(0.001)
                continue

            cw = spd > 0.0
            self._set_direction(cw)

            # Interval antar pulsa berdasarkan speed aktual
            interval = 1.0 / abs(spd)
            if now < next_pulse_t:
                time.sleep(min(0.001, next_pulse_t - now))
                continue

            # Safety terhadap limit switch berdasarkan arah
            if self._is_limit_triggered(moving_positive=cw):
                self.emergency_stop("Limit switch terpicu")
                continue

            # Prediksi posisi berikut untuk soft-limit
            step_delta_full = (1.0 / float(microstep)) * (1.0 if cw else -1.0)
            next_deg = ((self.get_position_steps() + step_delta_full) / self.cfg.steps_per_rev) * 360.0
            if self._soft_limit_reached(next_deg):
                self.emergency_stop("Soft limit tercapai")
                continue

            try:
                self._pulse_step()
                with self._lock:
                    self._position_full_steps += step_delta_full
            except Exception as exc:
                self.emergency_stop(f"GPIO pulse gagal: {exc}")
                continue

            next_pulse_t = now + interval

    def close(self):
        self._run = False
        self.stop_smooth()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.enable_driver(False)


def get_key() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            ch += sys.stdin.read(2)  # arrow key sequence
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def get_key_nonblocking(timeout: float = 0.01) -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return ""
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            ready, _, _ = select.select([sys.stdin], [], [], 0.001)
            if ready:
                ch += sys.stdin.read(1)
            ready, _, _ = select.select([sys.stdin], [], [], 0.001)
            if ready:
                ch += sys.stdin.read(1)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    # Motor 1 (sesuai mapping user)
    cfg_m1 = StepperConfig(
        step_pin=17,  # PUL+
        dir_pin=27,   # DIR+
        en_pin=22,    # EN+
        steps_per_rev=200,
        microstep=8,
        max_speed_sps=2200.0,
        accel_sps2=3000.0,
        soft_limit_min_deg=None,
        soft_limit_max_deg=None,
    )
    # Motor 2 (sesuai mapping user)
    cfg_m2 = StepperConfig(
        step_pin=23,  # PUL+
        dir_pin=24,   # DIR+
        en_pin=25,    # EN+
        steps_per_rev=200,
        microstep=8,
        max_speed_sps=2200.0,
        accel_sps2=3000.0,
        soft_limit_min_deg=None,
        soft_limit_max_deg=None,
    )

    motor_1 = TB6600Stepper(cfg_m1)
    motor_2 = TB6600Stepper(cfg_m2)

    device = None
    try:
        device = buat_device_model()
        device.ADDR = 0x50
        if platform.system().lower() == "linux":
            device.serialConfig.portName = "/dev/ttyUSB0"
        else:
            device.serialConfig.portName = "/dev/tty.usbserial-1330"
        device.serialConfig.baud = 9600
        device.openDevice()
        time.sleep(1)
    except Exception as exc:
        print(f"[WARN] IMU tidak aktif: {exc}")
        device = None

    command_speed = 600.0
    last_report = 0.0
    hold_until_m1 = 0.0
    hold_until_m2 = 0.0
    hold_dir_m1 = 0.0
    hold_dir_m2 = 0.0

    print(
        "\n=== DUAL TB6600 KEYBOARD STEPPER CONTROL ===\n"
        "Motor 1 (GPIO17/27/22): Arrow Left/Right atau A/D\n"
        "Motor 2 (GPIO23/24/25): Arrow Up/Down  atau W/S\n"
        "Mode gerak        : Hold key = jalan, lepas key = stop\n"
        "Space            : Smooth stop kedua motor\n"
        "E                : Emergency stop kedua motor (latch)\n"
        "R                : Reset fault kedua motor\n"
        "+ / -           : Speed up / down\n"
        "1/2/3/4/5       : Set microstep kedua motor = 1/2/4/8/16\n"
        "Q               : Quit\n"
    )
    print("{:<10} {:>8} {:>8} {:>8} {:>10} {:>10} {:>10}".format(
        "TIME", "ROLL", "EL", "YAW", "COMPASS", "AZ", "SRC"
    ))
    print("-" * 80)

    try:
        while True:
            key = get_key_nonblocking(0.01)
            now = time.time()
            if key in ("\x1b[C", "d", "D"):
                hold_dir_m1 = 1.0
                hold_until_m1 = now + KEY_HOLD_TIMEOUT
            elif key in ("\x1b[D", "a", "A"):
                hold_dir_m1 = -1.0
                hold_until_m1 = now + KEY_HOLD_TIMEOUT
            elif key in ("\x1b[A", "w", "W"):
                hold_dir_m2 = 1.0
                hold_until_m2 = now + KEY_HOLD_TIMEOUT
            elif key in ("\x1b[B", "s", "S"):
                hold_dir_m2 = -1.0
                hold_until_m2 = now + KEY_HOLD_TIMEOUT
            elif key == " ":
                motor_1.stop_smooth()
                motor_2.stop_smooth()
                hold_dir_m1 = 0.0
                hold_dir_m2 = 0.0
                hold_until_m1 = 0.0
                hold_until_m2 = 0.0
            elif key in ("e", "E"):
                motor_1.emergency_stop("Emergency stop keyboard")
                motor_2.emergency_stop("Emergency stop keyboard")
                hold_dir_m1 = 0.0
                hold_dir_m2 = 0.0
                hold_until_m1 = 0.0
                hold_until_m2 = 0.0
            elif key in ("r", "R"):
                motor_1.reset_fault()
                motor_2.reset_fault()
            elif key == "+":
                command_speed = min(cfg_m1.max_speed_sps, command_speed + 100.0)
            elif key == "-":
                command_speed = max(50.0, command_speed - 100.0)
            elif key == "1":
                motor_1.set_microstep(1)
                motor_2.set_microstep(1)
            elif key == "2":
                motor_1.set_microstep(2)
                motor_2.set_microstep(2)
            elif key == "3":
                motor_1.set_microstep(4)
                motor_2.set_microstep(4)
            elif key == "4":
                motor_1.set_microstep(8)
                motor_2.set_microstep(8)
            elif key == "5":
                motor_1.set_microstep(16)
                motor_2.set_microstep(16)
            elif key in ("q", "Q"):
                break

            if now <= hold_until_m1 and hold_dir_m1 != 0.0:
                motor_1.set_target_speed(hold_dir_m1 * abs(command_speed))
            else:
                motor_1.stop_smooth()

            if now <= hold_until_m2 and hold_dir_m2 != 0.0:
                motor_2.set_target_speed(hold_dir_m2 * abs(command_speed))
            else:
                motor_2.stop_smooth()

            if now - last_report >= INTERVAL:
                st1 = motor_1.get_status()
                st2 = motor_2.get_status()
                moving = abs(st1["current_speed_sps"]) > 1e-3 or abs(st2["current_speed_sps"]) > 1e-3
                if moving and device is not None:
                    data = baca_sudut(device)
                    if data[0] is not None:
                        roll, pitch, yaw, comp, az, src = data
                        now_text = time.strftime("%H:%M:%S")
                        print("{:<10} {:>8.2f} {:>8.2f} {:>8.2f} {:>10} {:>10.2f} {:>10}".format(
                            now_text,
                            roll,
                            pitch,
                            yaw,
                            f"{comp:.2f}" if comp is not None else "-",
                            az,
                            src,
                        ))
                last_report = now
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"\nERROR runtime: {exc}")
    finally:
        print("\nShutdown controller...")
        motor_1.close()
        motor_2.close()
        if device is not None:
            try:
                device.closeDevice()
            except Exception:
                pass
        GPIO.cleanup()
        print("GPIO cleaned up.")


if __name__ == "__main__":
    main()
