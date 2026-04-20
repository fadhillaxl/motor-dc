#!/usr/bin/env python3
# coding: utf-8
"""
AZ/EL Controller (Gabungan)

Menggabungkan:
- Kontrol keyboard 2x stepper TB6600 (STEP/DIR/EN) dengan ramp speed.
- Pembacaan heading WT901C485 (yaw + tilt compensated compass) dari fix-compas.

Kontrol keyboard:
- Motor AZ (GPIO17/27/22): Arrow Left/Right atau A/D
- Motor EL (GPIO23/24/25): Arrow Up/Down atau W/S
- Space: smooth stop kedua motor
- E: emergency stop (latch), R: reset fault
- + / -: naik/turun command speed
- 1..5: microstep (1/2/4/8/16) kedua motor
- Z / X: offset azimuth -1 / +1 derajat
- 0: reset zero point WT901
- Q: keluar
"""

import math
import os
import platform
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass

try:
    import RPi.GPIO as GPIO
except Exception as exc:
    print(f"ERROR: gagal import RPi.GPIO: {exc}")
    print("Jalankan file ini di Raspberry Pi dengan library RPi.GPIO terpasang.")
    sys.exit(1)


# =============================
# WT901 SDK PATH + IMPORT
# =============================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.abspath(
    os.path.join(BASE_DIR, "..", "..", "Python-SDK-WT901C485", "chs")
)
if SDK_CHS not in sys.path:
    sys.path.insert(0, SDK_CHS)

try:
    import lib.device_model as deviceModel
    from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
    from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver
    WT901_IMPORT_OK = True
except Exception:
    WT901_IMPORT_OK = False
    deviceModel = None
    JY901SDataProcessor = None
    Protocol485Resolver = None


@dataclass
class StepperConfig:
    step_pin: int
    dir_pin: int
    en_pin: int

    ms1_pin: int | None = None
    ms2_pin: int | None = None
    ms3_pin: int | None = None

    limit_min_pin: int | None = None
    limit_max_pin: int | None = None
    overcurrent_pin: int | None = None
    estop_pin: int | None = None

    en_active_high: bool = False
    dir_active_high: bool = True
    step_active_high: bool = True
    limit_active_low: bool = True
    overcurrent_active_low: bool = False
    estop_active_low: bool = True

    steps_per_rev: int = 200
    microstep: int = 8
    max_speed_sps: float = 2200.0
    accel_sps2: float = 3000.0
    pulse_width_us: int = 8
    soft_limit_min_deg: float | None = None
    soft_limit_max_deg: float | None = None


class TB6600Stepper:
    SUPPORTED_MICROSTEPS = (1, 2, 4, 8, 16)
    MICROSTEP_GPIO_MAP = {
        1: (0, 0, 0),
        2: (1, 0, 0),
        4: (0, 1, 0),
        8: (1, 1, 0),
        16: (1, 1, 1),
    }

    def __init__(self, cfg: StepperConfig):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._run = True
        self._fault_latched = False
        self._fault_msg = ""
        self._target_speed_sps = 0.0
        self._current_speed_sps = 0.0
        self._position_full_steps = 0.0

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self._setup_gpio()
        self.enable_driver(True)
        self.set_microstep(cfg.microstep)

        self._thread = threading.Thread(target=self._motion_loop, daemon=True)
        self._thread.start()

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

    @staticmethod
    def _is_active(raw: int, active_low: bool) -> bool:
        return (raw == GPIO.LOW) if active_low else (raw == GPIO.HIGH)

    @staticmethod
    def _set_output(pin: int, state: bool):
        GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)

    @staticmethod
    def _safe_input(pin: int) -> int:
        try:
            return GPIO.input(pin)
        except Exception:
            return 0

    def enable_driver(self, enabled: bool):
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

    def _check_hard_safety(self):
        if (
            self.cfg.estop_pin is not None
            and self._is_active(self._safe_input(self.cfg.estop_pin), self.cfg.estop_active_low)
        ):
            self.emergency_stop("E-STOP input aktif")
            return

        if (
            self.cfg.overcurrent_pin is not None
            and self._is_active(
                self._safe_input(self.cfg.overcurrent_pin), self.cfg.overcurrent_active_low
            )
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
        self._set_output(self.cfg.step_pin, hi)
        time.sleep(self.cfg.pulse_width_us / 1_000_000.0)
        self._set_output(self.cfg.step_pin, not hi)

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

                accel = max(1.0, float(self.cfg.accel_sps2))
                delta = accel * dt
                if self._current_speed_sps < self._target_speed_sps:
                    self._current_speed_sps = min(
                        self._current_speed_sps + delta, self._target_speed_sps
                    )
                elif self._current_speed_sps > self._target_speed_sps:
                    self._current_speed_sps = max(
                        self._current_speed_sps - delta, self._target_speed_sps
                    )

                spd = self._current_speed_sps
                microstep = self.cfg.microstep

            if abs(spd) < 1e-3:
                time.sleep(0.001)
                continue

            cw = spd > 0.0
            self._set_direction(cw)
            interval = 1.0 / abs(spd)
            if now < next_pulse_t:
                time.sleep(min(0.001, next_pulse_t - now))
                continue

            if self._is_limit_triggered(moving_positive=cw):
                self.emergency_stop("Limit switch terpicu")
                continue

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


@dataclass
class WT901Config:
    addr: int = 0x50
    baud: int = 9600
    interval: float = 0.1
    tilt_threshold_deg: float = 15.0
    az_offset_deg: float = -104.0
    port_name: str | None = None


class WT901Reader:
    def __init__(self, cfg: WT901Config):
        self.cfg = cfg
        self._device = None
        self._run = False
        self._thread = None
        self._lock = threading.Lock()
        self._latest = None

    @staticmethod
    def _tilt_compass(mx: float, my: float, mz: float, roll: float, pitch: float) -> float:
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        xh = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
        yh = (
            mx * math.sin(roll_rad) * math.sin(pitch_rad)
            + my * math.cos(roll_rad)
            - mz * math.sin(roll_rad) * math.cos(pitch_rad)
        )
        heading = math.degrees(math.atan2(yh, xh))
        if heading < 0:
            heading += 360.0
        return heading

    def _build_device_model(self):
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
                "EL_0",
            )

    def start(self):
        if not WT901_IMPORT_OK:
            raise RuntimeError("SDK WT901 tidak tersedia di path proyek.")

        self._device = self._build_device_model()
        self._device.ADDR = self.cfg.addr
        if self.cfg.port_name:
            self._device.serialConfig.portName = self.cfg.port_name
        else:
            if platform.system().lower() == "linux":
                self._device.serialConfig.portName = "/dev/ttyUSB0"
            else:
                self._device.serialConfig.portName = "/dev/tty.usbserial-1330"
        self._device.serialConfig.baud = self.cfg.baud
        self._device.openDevice()
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def close(self):
        self._run = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._device is not None:
            try:
                self._device.closeDevice()
            except Exception:
                pass

    def reset_zero_point(self):
        if self._device is None:
            return
        try:
            if hasattr(self._device, "write_register"):
                self._device.write_register(self._device.ADDR, 0x69, 0xB588)
                time.sleep(0.1)
                self._device.write_register(self._device.ADDR, 0x01, 0x0000)
            else:
                if hasattr(self._device, "unlock"):
                    self._device.unlock()
                    time.sleep(0.1)
                self._device.writeReg(0x01, 0x0000)
                if hasattr(self._device, "save"):
                    time.sleep(0.1)
                    self._device.save()
            time.sleep(0.3)
        except Exception:
            pass

    def set_az_offset(self, offset_deg: float):
        with self._lock:
            self.cfg.az_offset_deg = float(offset_deg)

    def get_latest(self) -> dict | None:
        with self._lock:
            if self._latest is None:
                return None
            return dict(self._latest)

    def _read_once(self) -> dict | None:
        if self._device is None:
            return None
        try:
            if hasattr(self._device, "readReg"):
                self._device.readReg(0x30, 41)

            if hasattr(self._device, "get"):
                roll = self._device.get("AngleX")
                pitch = self._device.get("AngleY")
                yaw = self._device.get("AngleZ")
                mx = self._device.get("magX")
                my = self._device.get("magY")
                mz = self._device.get("magZ")
            else:
                roll = self._device.getDeviceData("angleX")
                pitch = self._device.getDeviceData("angleY")
                yaw = self._device.getDeviceData("angleZ")
                mx = self._device.getDeviceData("magX")
                my = self._device.getDeviceData("magY")
                mz = self._device.getDeviceData("magZ")

            if None in (roll, pitch, yaw):
                return None

            roll = float(roll)
            pitch = float(pitch)
            yaw = float(yaw) % 360.0

            # Sensor dipasang terbalik, gunakan tuning dari fix-compas.py
            roll = 180.0 - roll
            pitch = -pitch
            if roll > 180.0:
                roll -= 360.0
            if roll < -180.0:
                roll += 360.0

            compass = None
            if None not in (mx, my, mz):
                mx = float(mx)
                my = float(my)
                mz = float(mz)
                mx, my = my, mx
                mx = -mx
                compass = self._tilt_compass(mx, my, mz, roll, pitch)

            with self._lock:
                az_offset = self.cfg.az_offset_deg
                tilt_threshold = self.cfg.tilt_threshold_deg

            yaw_cw = (360.0 - yaw + az_offset) % 360.0
            compass_cw = (360.0 - compass + az_offset) % 360.0 if compass is not None else None
            if abs(roll) > tilt_threshold or abs(pitch) > tilt_threshold:
                az = compass_cw
                src = "COMPASS"
            else:
                az = yaw_cw
                src = "YAW"

            return {
                "roll": roll,
                "pitch": pitch,
                "yaw_cw": yaw_cw,
                "compass_cw": compass_cw,
                "az_deg": az,
                "src": src,
                "ok": True,
                "timestamp": time.time(),
            }
        except Exception:
            return None

    def _loop(self):
        while self._run:
            data = self._read_once()
            with self._lock:
                if data is not None:
                    self._latest = data
                elif self._latest is None:
                    self._latest = {"ok": False, "timestamp": time.time()}
            time.sleep(max(0.01, self.cfg.interval))


class KeyboardReader:
    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._old = None

    def __enter__(self):
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def get_key(self, timeout: float = 0.05) -> str:
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        if not readable:
            return ""
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            seq = ""
            readable2, _, _ = select.select([sys.stdin], [], [], 0.001)
            if readable2:
                seq += sys.stdin.read(1)
            readable3, _, _ = select.select([sys.stdin], [], [], 0.001)
            if readable3:
                seq += sys.stdin.read(1)
            return ch + seq
        return ch


def main():
    cfg_az = StepperConfig(step_pin=17, dir_pin=27, en_pin=22, microstep=8)
    cfg_el = StepperConfig(step_pin=23, dir_pin=24, en_pin=25, microstep=8)

    motor_az = TB6600Stepper(cfg_az)
    motor_el = TB6600Stepper(cfg_el)

    imu = WT901Reader(WT901Config())
    imu_ok = False
    try:
        imu.start()
        imu.reset_zero_point()
        imu_ok = True
        print("[OK] WT901 connected.")
    except Exception as exc:
        print(f"[WARN] WT901 tidak aktif: {exc}")

    command_speed = 600.0
    last_report = 0.0

    print(
        "\n=== AZ/EL CONTROLLER (TB6600 + WT901) ===\n"
        "AZ motor  (GPIO17/27/22): Arrow Left/Right atau A/D\n"
        "EL motor  (GPIO23/24/25): Arrow Up/Down  atau W/S\n"
        "Space : Smooth stop kedua motor\n"
        "E     : Emergency stop kedua motor (latch)\n"
        "R     : Reset fault kedua motor\n"
        "+ / - : Speed up / down\n"
        "1..5  : Microstep kedua motor = 1/2/4/8/16\n"
        "Z / X : AZ offset -1 / +1 derajat\n"
        "0     : Reset zero WT901\n"
        "Q     : Quit\n"
    )

    try:
        with KeyboardReader() as kb:
            while True:
                now = time.time()
                if now - last_report > 0.25:
                    st_az = motor_az.get_status()
                    st_el = motor_el.get_status()

                    fault_az = f" F_AZ={st_az['fault_msg']}" if st_az["fault_latched"] else ""
                    fault_el = f" F_EL={st_el['fault_msg']}" if st_el["fault_latched"] else ""

                    imu_data = imu.get_latest() if imu_ok else None
                    if imu_data and imu_data.get("ok") and imu_data.get("az_deg") is not None:
                        az_imu = f"{imu_data['az_deg']:7.2f}"
                        src = imu_data.get("src", "-")
                    else:
                        az_imu = "   N/A "
                        src = "-"

                    sys.stdout.write(
                        f"\rAZ POS={st_az['position_deg']:7.2f} SPD={st_az['current_speed_sps']:7.1f} "
                        f"| EL POS={st_el['position_deg']:7.2f} SPD={st_el['current_speed_sps']:7.1f} "
                        f"| IMU_AZ={az_imu} SRC={src:<7} "
                        f"| MS={st_az['microstep']:2d}{fault_az}{fault_el}      "
                    )
                    sys.stdout.flush()
                    last_report = now

                key = kb.get_key(timeout=0.05)
                if not key:
                    continue

                if key in ("\x1b[C", "d", "D"):
                    motor_az.set_target_speed(abs(command_speed))
                elif key in ("\x1b[D", "a", "A"):
                    motor_az.set_target_speed(-abs(command_speed))
                elif key in ("\x1b[A", "w", "W"):
                    motor_el.set_target_speed(abs(command_speed))
                elif key in ("\x1b[B", "s", "S"):
                    motor_el.set_target_speed(-abs(command_speed))
                elif key == " ":
                    motor_az.stop_smooth()
                    motor_el.stop_smooth()
                elif key in ("e", "E"):
                    motor_az.emergency_stop("Emergency stop keyboard")
                    motor_el.emergency_stop("Emergency stop keyboard")
                elif key in ("r", "R"):
                    motor_az.reset_fault()
                    motor_el.reset_fault()
                elif key == "+":
                    command_speed = min(cfg_az.max_speed_sps, command_speed + 100.0)
                elif key == "-":
                    command_speed = max(50.0, command_speed - 100.0)
                elif key == "1":
                    motor_az.set_microstep(1)
                    motor_el.set_microstep(1)
                elif key == "2":
                    motor_az.set_microstep(2)
                    motor_el.set_microstep(2)
                elif key == "3":
                    motor_az.set_microstep(4)
                    motor_el.set_microstep(4)
                elif key == "4":
                    motor_az.set_microstep(8)
                    motor_el.set_microstep(8)
                elif key == "5":
                    motor_az.set_microstep(16)
                    motor_el.set_microstep(16)
                elif key in ("z", "Z"):
                    current = imu.cfg.az_offset_deg
                    imu.set_az_offset(current - 1.0)
                elif key in ("x", "X"):
                    current = imu.cfg.az_offset_deg
                    imu.set_az_offset(current + 1.0)
                elif key == "0":
                    if imu_ok:
                        imu.reset_zero_point()
                elif key in ("q", "Q"):
                    break
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"\nERROR runtime: {exc}")
    finally:
        print("\nShutdown controller...")
        motor_az.close()
        motor_el.close()
        imu.close()
        GPIO.cleanup()
        print("GPIO cleaned up.")


if __name__ == "__main__":
    main()
