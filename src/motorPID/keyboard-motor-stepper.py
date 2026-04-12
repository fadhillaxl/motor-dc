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
import select
import tty
import termios
import threading
import argparse
import json
import os
import urllib.parse
import urllib.request
from dataclasses import dataclass
from az_ls_utils import az_ls_allows_motion, validate_az_ls
try:
    import tkinter as tk
    TK_AVAILABLE = True
except Exception:
    tk = None
    TK_AVAILABLE = False

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except Exception as exc:
    GPIO = None
    GPIO_AVAILABLE = False
    GPIO_IMPORT_ERROR = str(exc)

try:
    from skyfield.api import EarthSatellite, Topos, load
    SKYFIELD_AVAILABLE = True
except Exception:
    EarthSatellite = None
    Topos = None
    load = None
    SKYFIELD_AVAILABLE = False


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
    az_wrap_enabled: bool = False  # False: tidak boleh lintas 0/360 (sesuai rotator non-continous)
    az_offset_deg: float = 0.0     # Kalibrasi azimuth terhadap Utara kompas
    az_ls_deg: float = 0.0         # 0=full range, non-zero=blok jika lintas titik AZ LS
    az_ls_block_crossing: bool = False  # Default: AZ LS sebagai referensi, bukan hard-stop


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
        if not GPIO_AVAILABLE:
            raise RuntimeError(
                f"RPi.GPIO tidak tersedia ({GPIO_IMPORT_ERROR}). "
                "Gunakan mode simulasi: python keyboard-motor-stepper.py --sim"
            )
        self.cfg = cfg
        self.cfg.az_ls_deg = validate_az_ls(self.cfg.az_ls_deg)
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

    def _az_ls_reached(self, current_deg: float, next_deg: float, moving_positive: bool) -> bool:
        if not self.cfg.az_ls_block_crossing:
            return False
        if az_ls_allows_motion(current_deg, next_deg, self.cfg.az_ls_deg):
            return False
        with self._lock:
            self._current_speed_sps = 0.0
            if (moving_positive and self._target_speed_sps > 0) or ((not moving_positive) and self._target_speed_sps < 0):
                self._target_speed_sps = 0.0
            self._fault_msg = f"AZ LS boundary reached at {self.cfg.az_ls_deg:.2f} deg"
        return True

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

            # Safety terhadap limit switch berdasarkan arah:
            # jangan latch fault, cukup blok arah yang menabrak limit
            if self._is_limit_triggered(moving_positive=cw):
                with self._lock:
                    self._current_speed_sps = 0.0
                    # jika command masih menekan ke arah limit, nolkan target.
                    if (cw and self._target_speed_sps > 0) or ((not cw) and self._target_speed_sps < 0):
                        self._target_speed_sps = 0.0
                continue

            # Prediksi posisi berikut untuk soft-limit
            step_delta_full = (1.0 / float(microstep)) * (1.0 if cw else -1.0)
            current_deg = (self.get_position_steps() / self.cfg.steps_per_rev) * 360.0
            next_deg = ((self.get_position_steps() + step_delta_full) / self.cfg.steps_per_rev) * 360.0
            if self._soft_limit_reached(next_deg):
                with self._lock:
                    self._current_speed_sps = 0.0
                    # blok hanya arah yang melanggar soft-limit
                    if (cw and self._target_speed_sps > 0) or ((not cw) and self._target_speed_sps < 0):
                        self._target_speed_sps = 0.0
                continue
            if self._az_ls_reached(current_deg, next_deg, cw):
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


class SimStepper:
    """Simulator stepper tanpa GPIO, API mirip TB6600Stepper."""

    SUPPORTED_MICROSTEPS = (1, 2, 4, 8, 16)

    def __init__(self, cfg: StepperConfig, name: str = "SIM"):
        self.cfg = cfg
        self.cfg.az_ls_deg = validate_az_ls(self.cfg.az_ls_deg)
        self.name = name
        self._lock = threading.Lock()
        self._run = True
        self._fault_latched = False
        self._fault_msg = ""
        self._target_speed_sps = 0.0
        self._current_speed_sps = 0.0
        self._position_full_steps = 0.0
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_microstep(self, microstep: int):
        if microstep not in self.SUPPORTED_MICROSTEPS:
            raise ValueError("Microstep harus salah satu dari: 1, 2, 4, 8, 16")
        with self._lock:
            self.cfg.microstep = microstep

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

    def reset_fault(self):
        with self._lock:
            self._fault_latched = False
            self._fault_msg = ""
            self._target_speed_sps = 0.0
            self._current_speed_sps = 0.0

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

    def _check_soft_limit(self, next_deg: float):
        mn = self.cfg.soft_limit_min_deg
        mx = self.cfg.soft_limit_max_deg
        if mn is not None and next_deg < mn:
            # Untuk limit posisi, jangan latch fault. Cukup stop arah ini.
            with self._lock:
                self._current_speed_sps = 0.0
                if self._target_speed_sps < 0:
                    self._target_speed_sps = 0.0
            return True
        if mx is not None and next_deg > mx:
            with self._lock:
                self._current_speed_sps = 0.0
                if self._target_speed_sps > 0:
                    self._target_speed_sps = 0.0
            return True
        return False

    def _check_az_ls(self, current_deg: float, next_deg: float):
        if not self.cfg.az_ls_block_crossing:
            return False
        if az_ls_allows_motion(current_deg, next_deg, self.cfg.az_ls_deg):
            return False
        with self._lock:
            self._current_speed_sps = 0.0
            if self._target_speed_sps > 0:
                self._target_speed_sps = 0.0
            elif self._target_speed_sps < 0:
                self._target_speed_sps = 0.0
            self._fault_msg = f"AZ LS boundary reached at {self.cfg.az_ls_deg:.2f} deg"
        return True

    def _loop(self):
        last_t = time.perf_counter()
        while self._run:
            now = time.perf_counter()
            dt = now - last_t
            last_t = now
            with self._lock:
                if self._fault_latched:
                    time.sleep(0.01)
                    continue
                a = max(1.0, float(self.cfg.accel_sps2))
                delta = a * dt
                if self._current_speed_sps < self._target_speed_sps:
                    self._current_speed_sps = min(self._current_speed_sps + delta, self._target_speed_sps)
                elif self._current_speed_sps > self._target_speed_sps:
                    self._current_speed_sps = max(self._current_speed_sps - delta, self._target_speed_sps)
                spd = self._current_speed_sps
                ms = max(1, int(self.cfg.microstep))

            # Integrasi posisi berbasis kecepatan pulse/s
            if abs(spd) > 1e-3:
                step_delta_full = (spd * dt) / float(ms)
                current_deg = (self.get_position_steps() / self.cfg.steps_per_rev) * 360.0
                next_deg = ((self.get_position_steps() + step_delta_full) / self.cfg.steps_per_rev) * 360.0
                if not self._check_soft_limit(next_deg) and not self._check_az_ls(current_deg, next_deg):
                    with self._lock:
                        self._position_full_steps += step_delta_full
            time.sleep(0.001)

    def close(self):
        self._run = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


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


def get_key_nonblocking(timeout: float = 0.05):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    ch = sys.stdin.read(1)
    if ch == "\x1b":
        seq = ch
        r, _, _ = select.select([sys.stdin], [], [], 0.002)
        if r:
            seq += sys.stdin.read(1)
        r, _, _ = select.select([sys.stdin], [], [], 0.002)
        if r:
            seq += sys.stdin.read(1)
        return seq
    return ch


def clear_screen():
    sys.stdout.write("\x1b[2J\x1b[H")
    sys.stdout.flush()


def draw_sim_ui(st1, st2, command_speed):
    clear_screen()
    print("=== ROTATOR STEPPER SIMULATION UI ===")
    print("Konfigurasi: NEMA23 | TB6600 | Microstep 2 (400 pulse/rev) | Current 2.0A")
    print("")
    print("Kontrol:")
    print("- Motor 1 (AZ): Left/Right atau A/D")
    print("- Motor 2 (EL): Up/Down   atau W/S")
    print("- Space=Stop halus, E=E-Stop, R=Reset fault, +/- speed, 1..5 microstep, Q=Quit")
    print("")
    print(
        f"AZ  pos={st1['position_deg']:8.2f} deg  spd={st1['current_speed_sps']:8.1f} sps  "
        f"tgt={st1['target_speed_sps']:8.1f}  ms={st1['microstep']}"
    )
    print(
        f"EL  pos={st2['position_deg']:8.2f} deg  spd={st2['current_speed_sps']:8.1f} sps  "
        f"tgt={st2['target_speed_sps']:8.1f}  ms={st2['microstep']}"
    )
    if st1["fault_latched"] or st2["fault_latched"]:
        print(f"FAULT: {st1['fault_msg']} {st2['fault_msg']}")
    print(f"\nCommand speed: {command_speed:.1f} sps")


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def tle_fetch_collection(search_text: str = "", page: int = 1, page_size: int = 20):
    base = "https://tle.ivanstanojevic.me/api/tle"
    params = {"page": page, "page-size": page_size}
    if search_text.strip():
        params["search"] = search_text.strip()
    url = f"{base}/?{urllib.parse.urlencode(params)}"
    req = urllib.request.Request(url, headers={"User-Agent": "motor-dc-rotator/1.0"})
    with urllib.request.urlopen(req, timeout=10) as resp:
        data = json.loads(resp.read().decode("utf-8"))

    # Format API umum: {"member":[...], ...}
    if isinstance(data, dict):
        if isinstance(data.get("member"), list):
            return data["member"]
        if isinstance(data.get("results"), list):
            return data["results"]
    if isinstance(data, list):
        return data
    return []


def tle_extract_lines(item: dict):
    # fleksibel terhadap nama field yang berbeda
    name = item.get("name") or item.get("satellite") or item.get("satellite_name") or "UNKNOWN"
    l1 = item.get("line1") or item.get("tle_line1")
    l2 = item.get("line2") or item.get("tle_line2")
    sat_id = item.get("satelliteId") or item.get("norad_cat_id") or item.get("id")
    if not l1 or not l2:
        return None
    return {"name": name, "line1": l1, "line2": l2, "id": sat_id}


class SimGuiApp:
    def __init__(self, motor_1, motor_2, cfg_m1, cfg_m2):
        self.motor_1 = motor_1
        self.motor_2 = motor_2
        self.cfg_m1 = cfg_m1
        self.cfg_m2 = cfg_m2
        self.command_speed = 600.0
        self.az_pos_pressed = False
        self.az_neg_pressed = False
        self.el_pos_pressed = False
        self.el_neg_pressed = False
        self.selected_tle = None
        self.selected_sat_name = "-"
        self.sat_az = None
        self.sat_el = None
        self.az_ls_deg = 36.0
        self.ts = load.timescale() if SKYFIELD_AVAILABLE else None
        self.tracking_enabled = False
        self._pid_state = {
            "az": {"i": 0.0, "last_e": 0.0, "last_t": None},
            "el": {"i": 0.0, "last_e": 0.0, "last_t": None},
        }

        self.root = tk.Tk()
        self.root.title("Rotator Stepper Simulation (NEMA23 + TB6600)")
        self.root.geometry("860x560")
        self.root.minsize(760, 420)

        # Scrollable container untuk semua elemen GUI.
        self.scroll_canvas = tk.Canvas(self.root, highlightthickness=0)
        self.scrollbar = tk.Scrollbar(self.root, orient="vertical", command=self.scroll_canvas.yview)
        self.scroll_canvas.configure(yscrollcommand=self.scrollbar.set)
        self.scroll_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.content = tk.Frame(self.scroll_canvas)
        self.content_window = self.scroll_canvas.create_window((0, 0), window=self.content, anchor="nw")
        self.content.bind("<Configure>", self._on_content_configure)
        self.scroll_canvas.bind("<Configure>", self._on_canvas_configure)
        self._bind_mousewheel()

        self.lbl_title = tk.Label(
            self.content,
            text="Konfigurasi: NEMA23 | TB6600 | Microstep 2 (400 pulse/rev) | Current 2.0A",
            font=("Arial", 12, "bold"),
        )
        self.lbl_title.pack(pady=8)

        tle_frame = tk.Frame(self.content)
        tle_frame.pack(pady=4, fill="x")
        tk.Label(tle_frame, text="Search TLE:").pack(side=tk.LEFT, padx=4)
        self.ent_search = tk.Entry(tle_frame, width=24)
        self.ent_search.insert(0, "ISS")
        self.ent_search.pack(side=tk.LEFT, padx=4)
        tk.Button(tle_frame, text="Load", command=self._load_tle).pack(side=tk.LEFT, padx=4)

        tk.Label(tle_frame, text="Lat").pack(side=tk.LEFT, padx=(16, 2))
        self.ent_lat = tk.Entry(tle_frame, width=8)
        self.ent_lat.insert(0, "-6.2")
        self.ent_lat.pack(side=tk.LEFT, padx=2)
        tk.Label(tle_frame, text="Lon").pack(side=tk.LEFT, padx=(8, 2))
        self.ent_lon = tk.Entry(tle_frame, width=8)
        self.ent_lon.insert(0, "106.8")
        self.ent_lon.pack(side=tk.LEFT, padx=2)
        tk.Label(tle_frame, text="Alt(m)").pack(side=tk.LEFT, padx=(8, 2))
        self.ent_alt = tk.Entry(tle_frame, width=8)
        self.ent_alt.insert(0, "50")
        self.ent_alt.pack(side=tk.LEFT, padx=2)

        self.lst_tle = tk.Listbox(self.content, height=5)
        self.lst_tle.pack(fill="x", padx=10, pady=4)
        self.lst_tle.bind("<<ListboxSelect>>", self._on_select_tle)

        self.lbl_sat = tk.Label(self.content, text="SAT: - | AZ: - | EL: -", font=("Consolas", 11, "bold"))
        self.lbl_sat.pack(pady=4)
        self.btn_track = tk.Button(self.content, text="TRACK: OFF", width=14, command=self._toggle_tracking, bg="#5a5a5a", fg="white")
        self.btn_track.pack(pady=4)

        cal = tk.Frame(self.content)
        cal.pack(pady=4, fill="x")
        tk.Label(cal, text="AZ offset (deg)").pack(side=tk.LEFT, padx=(8, 2))
        self.ent_az_offset = tk.Entry(cal, width=8)
        self.ent_az_offset.insert(0, f"{self.cfg_m1.az_offset_deg:.2f}")
        self.ent_az_offset.pack(side=tk.LEFT, padx=2)

        limit_row = tk.Frame(self.content)
        limit_row.pack(pady=2, fill="x")
        tk.Label(limit_row, text="Limit Switch Location (deg)", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(8, 12))
        tk.Label(limit_row, text="AZ LS ref").pack(side=tk.LEFT, padx=(2, 2))
        self.ent_az_ls = tk.Entry(limit_row, width=7)
        self.ent_az_ls.insert(0, "36")
        self.ent_az_ls.pack(side=tk.LEFT, padx=2)
        tk.Label(limit_row, text="EL min").pack(side=tk.LEFT, padx=(8, 2))
        self.ent_el_min = tk.Entry(limit_row, width=7)
        self.ent_el_min.insert(0, "0")
        self.ent_el_min.pack(side=tk.LEFT, padx=2)
        tk.Label(limit_row, text="EL max").pack(side=tk.LEFT, padx=(8, 2))
        self.ent_el_max = tk.Entry(limit_row, width=7)
        self.ent_el_max.insert(0, "90")
        self.ent_el_max.pack(side=tk.LEFT, padx=2)
        tk.Button(limit_row, text="Apply Limit Degrees", command=self._apply_limit_offset).pack(side=tk.LEFT, padx=10)

        self.lbl_status = tk.Label(self.content, text="", justify="left", font=("Consolas", 11))
        self.lbl_status.pack(pady=6)

        self.canvas = tk.Canvas(self.content, width=820, height=260, bg="#101820")
        self.canvas.pack(pady=6)

        ctrl = tk.Frame(self.content)
        ctrl.pack(pady=8)
        btn_az_ccw = tk.Button(ctrl, text="AZ CCW (A/←)", width=14)
        btn_az_ccw.grid(row=0, column=0, padx=4, pady=4)
        btn_az_ccw.bind("<ButtonPress-1>", lambda e: self._press_axis("az_neg"))
        btn_az_ccw.bind("<ButtonRelease-1>", lambda e: self._release_axis("az_neg"))

        btn_az_cw = tk.Button(ctrl, text="AZ CW (D/→)", width=14)
        btn_az_cw.grid(row=0, column=1, padx=4, pady=4)
        btn_az_cw.bind("<ButtonPress-1>", lambda e: self._press_axis("az_pos"))
        btn_az_cw.bind("<ButtonRelease-1>", lambda e: self._release_axis("az_pos"))

        btn_el_down = tk.Button(ctrl, text="EL DOWN (S/↓)", width=14)
        btn_el_down.grid(row=0, column=2, padx=4, pady=4)
        btn_el_down.bind("<ButtonPress-1>", lambda e: self._press_axis("el_neg"))
        btn_el_down.bind("<ButtonRelease-1>", lambda e: self._release_axis("el_neg"))

        btn_el_up = tk.Button(ctrl, text="EL UP (W/↑)", width=14)
        btn_el_up.grid(row=0, column=3, padx=4, pady=4)
        btn_el_up.bind("<ButtonPress-1>", lambda e: self._press_axis("el_pos"))
        btn_el_up.bind("<ButtonRelease-1>", lambda e: self._release_axis("el_pos"))

        tk.Button(ctrl, text="STOP", width=10, command=self._smooth_stop).grid(row=1, column=0, padx=4, pady=4)
        tk.Button(ctrl, text="E-STOP", width=10, command=self._estop).grid(row=1, column=1, padx=4, pady=4)
        tk.Button(ctrl, text="RESET", width=10, command=self._reset).grid(row=1, column=2, padx=4, pady=4)
        tk.Button(ctrl, text="+ SPEED", width=10, command=self._speed_up).grid(row=1, column=3, padx=4, pady=4)
        tk.Button(ctrl, text="- SPEED", width=10, command=self._speed_down).grid(row=1, column=4, padx=4, pady=4)

        ms = tk.Frame(self.content)
        ms.pack()
        tk.Label(ms, text="Microstep:", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=8)
        for label, val in [("1", 1), ("2", 2), ("4", 4), ("8", 8), ("16", 16)]:
            tk.Button(ms, text=label, width=4, command=lambda v=val: self._set_micro(v)).pack(side=tk.LEFT, padx=2)

        self.root.bind("<KeyPress-Left>", lambda e: self._press_axis("az_neg"))
        self.root.bind("<KeyRelease-Left>", lambda e: self._release_axis("az_neg"))
        self.root.bind("<KeyPress-Right>", lambda e: self._press_axis("az_pos"))
        self.root.bind("<KeyRelease-Right>", lambda e: self._release_axis("az_pos"))
        self.root.bind("<KeyPress-Up>", lambda e: self._press_axis("el_pos"))
        self.root.bind("<KeyRelease-Up>", lambda e: self._release_axis("el_pos"))
        self.root.bind("<KeyPress-Down>", lambda e: self._press_axis("el_neg"))
        self.root.bind("<KeyRelease-Down>", lambda e: self._release_axis("el_neg"))
        self.root.bind("<space>", lambda e: self._smooth_stop())
        self.root.bind("<KeyPress-a>", lambda e: self._press_axis("az_neg"))
        self.root.bind("<KeyRelease-a>", lambda e: self._release_axis("az_neg"))
        self.root.bind("<KeyPress-d>", lambda e: self._press_axis("az_pos"))
        self.root.bind("<KeyRelease-d>", lambda e: self._release_axis("az_pos"))
        self.root.bind("<KeyPress-w>", lambda e: self._press_axis("el_pos"))
        self.root.bind("<KeyRelease-w>", lambda e: self._release_axis("el_pos"))
        self.root.bind("<KeyPress-s>", lambda e: self._press_axis("el_neg"))
        self.root.bind("<KeyRelease-s>", lambda e: self._release_axis("el_neg"))
        self.root.bind("<KeyPress-A>", lambda e: self._press_axis("az_neg"))
        self.root.bind("<KeyRelease-A>", lambda e: self._release_axis("az_neg"))
        self.root.bind("<KeyPress-D>", lambda e: self._press_axis("az_pos"))
        self.root.bind("<KeyRelease-D>", lambda e: self._release_axis("az_pos"))
        self.root.bind("<KeyPress-W>", lambda e: self._press_axis("el_pos"))
        self.root.bind("<KeyRelease-W>", lambda e: self._release_axis("el_pos"))
        self.root.bind("<KeyPress-S>", lambda e: self._press_axis("el_neg"))
        self.root.bind("<KeyRelease-S>", lambda e: self._release_axis("el_neg"))
        self.root.bind("e", lambda e: self._estop())
        self.root.bind("r", lambda e: self._reset())
        self.root.bind("+", lambda e: self._speed_up())
        self.root.bind("-", lambda e: self._speed_down())

        self.root.protocol("WM_DELETE_WINDOW", self.root.quit)
        self._load_tle()
        self._update_ui()

    def _on_content_configure(self, _event):
        self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        # Biar lebar konten mengikuti lebar viewport canvas.
        self.scroll_canvas.itemconfig(self.content_window, width=event.width)

    def _bind_mousewheel(self):
        # Windows/macOS
        self.root.bind_all("<MouseWheel>", self._on_mousewheel)
        # Linux/X11
        self.root.bind_all("<Button-4>", self._on_mousewheel_linux_up)
        self.root.bind_all("<Button-5>", self._on_mousewheel_linux_down)

    def _on_mousewheel(self, event):
        # event.delta biasanya kelipatan 120 (Windows) atau nilai kecil (macOS).
        step = int(-1 * (event.delta / 120)) if event.delta else 0
        if step != 0:
            self.scroll_canvas.yview_scroll(step, "units")

    def _on_mousewheel_linux_up(self, _event):
        self.scroll_canvas.yview_scroll(-1, "units")

    def _on_mousewheel_linux_down(self, _event):
        self.scroll_canvas.yview_scroll(1, "units")

    def _load_tle(self):
        q = self.ent_search.get().strip()
        self.lst_tle.delete(0, tk.END)
        try:
            items = tle_fetch_collection(q, page=1, page_size=20)
            parsed = []
            for it in items:
                r = tle_extract_lines(it)
                if r:
                    parsed.append(r)
            self._tle_cache = parsed
            if not parsed:
                self.lst_tle.insert(tk.END, "No data")
                return
            for i, it in enumerate(parsed):
                sat_id = it.get("id")
                text = f"{i+1:02d}. {it['name']}" + (f" [{sat_id}]" if sat_id else "")
                self.lst_tle.insert(tk.END, text)
            self.lst_tle.selection_clear(0, tk.END)
            self.lst_tle.selection_set(0)
            self._on_select_tle()
        except Exception as exc:
            self.lst_tle.insert(tk.END, f"Load error: {exc}")

    def _on_select_tle(self, _evt=None):
        sel = self.lst_tle.curselection()
        if not sel:
            return
        idx = int(sel[0])
        if not hasattr(self, "_tle_cache") or idx >= len(self._tle_cache):
            return
        self.selected_tle = self._tle_cache[idx]
        self.selected_sat_name = self.selected_tle["name"]

    def _calc_selected_az_el(self):
        if not self.selected_tle or not SKYFIELD_AVAILABLE:
            self.sat_az = None
            self.sat_el = None
            return
        try:
            lat = float(self.ent_lat.get().strip())
            lon = float(self.ent_lon.get().strip())
            alt_m = float(self.ent_alt.get().strip())
        except ValueError:
            self.sat_az = None
            self.sat_el = None
            return
        try:
            sat = EarthSatellite(self.selected_tle["line1"], self.selected_tle["line2"], self.selected_tle["name"], self.ts)
            t = self.ts.now()
            observer = Topos(latitude_degrees=lat, longitude_degrees=lon, elevation_m=alt_m)
            difference = sat - observer
            topocentric = difference.at(t)
            alt, az, _distance = topocentric.altaz()
            self.sat_az = az.degrees
            self.sat_el = alt.degrees
        except Exception:
            self.sat_az = None
            self.sat_el = None

    def _press_axis(self, axis: str):
        # Manual override otomatis mematikan tracking.
        if self.tracking_enabled:
            self._set_tracking(False)
        if axis == "az_pos":
            self.az_pos_pressed = True
        elif axis == "az_neg":
            self.az_neg_pressed = True
        elif axis == "el_pos":
            self.el_pos_pressed = True
        elif axis == "el_neg":
            self.el_neg_pressed = True
        self._apply_axis_motion()

    def _release_axis(self, axis: str):
        if axis == "az_pos":
            self.az_pos_pressed = False
        elif axis == "az_neg":
            self.az_neg_pressed = False
        elif axis == "el_pos":
            self.el_pos_pressed = False
        elif axis == "el_neg":
            self.el_neg_pressed = False
        self._apply_axis_motion()

    def _apply_axis_motion(self):
        spd = abs(self.command_speed)
        if self.az_pos_pressed and not self.az_neg_pressed:
            self.motor_1.set_target_speed(spd)
        elif self.az_neg_pressed and not self.az_pos_pressed:
            self.motor_1.set_target_speed(-spd)
        else:
            self.motor_1.stop_smooth()

        if self.el_pos_pressed and not self.el_neg_pressed:
            self.motor_2.set_target_speed(spd)
        elif self.el_neg_pressed and not self.el_pos_pressed:
            self.motor_2.set_target_speed(-spd)
        else:
            self.motor_2.stop_smooth()

    def _smooth_stop(self):
        self.az_pos_pressed = False
        self.az_neg_pressed = False
        self.el_pos_pressed = False
        self.el_neg_pressed = False
        self.motor_1.stop_smooth()
        self.motor_2.stop_smooth()

    def _estop(self):
        self.motor_1.emergency_stop("Emergency stop GUI")
        self.motor_2.emergency_stop("Emergency stop GUI")

    def _reset(self):
        self.motor_1.reset_fault()
        self.motor_2.reset_fault()

    def _speed_up(self):
        self.command_speed = min(self.cfg_m1.max_speed_sps, self.command_speed + 100.0)

    def _speed_down(self):
        self.command_speed = max(50.0, self.command_speed - 100.0)

    def _set_micro(self, v):
        self.motor_1.set_microstep(v)
        self.motor_2.set_microstep(v)

    def _apply_limit_offset(self):
        try:
            az_offset = float(self.ent_az_offset.get().strip())
            az_ls = validate_az_ls(float(self.ent_az_ls.get().strip()))
            el_min = float(self.ent_el_min.get().strip())
            el_max = float(self.ent_el_max.get().strip())
            if el_min >= el_max:
                raise ValueError("EL min must be < EL max")

            self.cfg_m1.az_offset_deg = az_offset
            self.az_ls_deg = az_ls
            self.cfg_m1.az_ls_deg = az_ls
            self.cfg_m1.soft_limit_min_deg = 0.0
            self.cfg_m1.soft_limit_max_deg = 360.0
            self.cfg_m2.soft_limit_min_deg = el_min
            self.cfg_m2.soft_limit_max_deg = el_max
            self.lbl_sat.config(
                text=(
                    f"SAT: {self.selected_sat_name} | AZ: {self.sat_az:.2f}° | EL: {self.sat_el:.2f}° "
                    if self.sat_az is not None and self.sat_el is not None
                    else f"SAT: {self.selected_sat_name} | AZ: - | EL: -"
                )
                + f"  [AZ LS ref={self.az_ls_deg:.2f} deg applied]"
            )
        except Exception as exc:
            self.lbl_sat.config(text=f"SAT: {self.selected_sat_name} | AZ: - | EL: -  [ERROR: {exc}]")

    def _toggle_tracking(self):
        self._set_tracking(not self.tracking_enabled)

    def _set_tracking(self, enabled: bool):
        self.tracking_enabled = enabled
        if enabled:
            self.btn_track.config(text="TRACK: ON", bg="#0b8f3a")
            now = time.time()
            for k in ("az", "el"):
                self._pid_state[k]["i"] = 0.0
                self._pid_state[k]["last_e"] = 0.0
                self._pid_state[k]["last_t"] = now
        else:
            self.btn_track.config(text="TRACK: OFF", bg="#5a5a5a")
            self.motor_1.stop_smooth()
            self.motor_2.stop_smooth()

    def _az_shortest_error(self, target_deg: float, current_deg: float) -> float:
        # Jika rotator tidak bisa muter bebas 360, jangan gunakan shortest-wrap.
        if not self.cfg_m1.az_wrap_enabled:
            return target_deg - current_deg
        e = target_deg - current_deg
        if e > 180.0:
            e -= 360.0
        elif e < -180.0:
            e += 360.0
        return e

    def _adaptive_gains(self, axis: str, err_abs_deg: float):
        # Gain adaptif: error besar -> respon agresif, error kecil -> halus
        if axis == "az":
            if err_abs_deg > 20:
                return 120.0, 0.20, 25.0
            if err_abs_deg > 5:
                return 85.0, 0.12, 18.0
            if err_abs_deg > 1:
                return 45.0, 0.06, 10.0
            return 20.0, 0.00, 6.0
        # EL biasanya lebih lambat/berbeban
        if err_abs_deg > 15:
            return 110.0, 0.18, 20.0
        if err_abs_deg > 4:
            return 80.0, 0.10, 15.0
        if err_abs_deg > 1:
            return 40.0, 0.05, 8.0
        return 18.0, 0.00, 5.0

    def _pid_speed_cmd(self, axis: str, err_deg: float, max_speed: float):
        st = self._pid_state[axis]
        now = time.time()
        last_t = st["last_t"]
        dt = 0.02 if last_t is None else max(0.001, now - last_t)
        st["last_t"] = now

        kp, ki, kd = self._adaptive_gains(axis, abs(err_deg))
        st["i"] += err_deg * dt
        st["i"] = max(-300.0, min(300.0, st["i"]))
        d = (err_deg - st["last_e"]) / dt
        st["last_e"] = err_deg

        out = (kp * err_deg) + (ki * st["i"]) + (kd * d)
        # Deadband supaya tidak hunting saat dekat target
        if abs(err_deg) < 0.08:
            out = 0.0
        out = max(-max_speed, min(max_speed, out))
        return out

    def _tracking_step(self):
        if not self.tracking_enabled:
            return
        if self.sat_az is None or self.sat_el is None:
            return

        st1 = self.motor_1.get_status()
        st2 = self.motor_2.get_status()
        cur_az = st1["position_deg"] % 360.0
        cur_el = st2["position_deg"]
        # Konversi azimuth satelit (kompas) ke frame mekanik rotator dengan offset kalibrasi.
        target_az = (self.sat_az + self.cfg_m1.az_offset_deg) % 360.0
        target_el = self.sat_el

        # Clamp target ke rentang mekanik agar tidak forcing ke luar limit.
        if self.cfg_m1.soft_limit_min_deg is not None:
            target_az = max(self.cfg_m1.soft_limit_min_deg, target_az)
        if self.cfg_m1.soft_limit_max_deg is not None:
            target_az = min(self.cfg_m1.soft_limit_max_deg, target_az)

        err_az = self._az_shortest_error(target_az, cur_az)
        err_el = target_el - cur_el

        cmd_az = self._pid_speed_cmd("az", err_az, self.cfg_m1.max_speed_sps)
        cmd_el = self._pid_speed_cmd("el", err_el, self.cfg_m2.max_speed_sps)
        self.motor_1.set_target_speed(cmd_az)
        self.motor_2.set_target_speed(cmd_el)

    def _draw_rotator(self, st1, st2):
        self.canvas.delete("all")
        cx, cy, r = 200, 130, 90
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#33ccff", width=2)
        self.canvas.create_text(cx, cy + 105, fill="white", text="AZ", font=("Arial", 10, "bold"))
        az = st1["position_deg"] % 360.0
        import math
        rad = math.radians(az - 90.0)
        x2 = cx + r * 0.85 * math.cos(rad)
        y2 = cy + r * 0.85 * math.sin(rad)
        self.canvas.create_line(cx, cy, x2, y2, fill="#ffd166", width=4, arrow=tk.LAST)

        x0, y0, w, h = 460, 60, 240, 160
        self.canvas.create_rectangle(x0, y0, x0 + w, y0 + h, outline="#33ccff", width=2)
        self.canvas.create_text(x0 + w / 2, y0 + h + 20, fill="white", text="EL", font=("Arial", 10, "bold"))
        el = max(0.0, min(90.0, st2["position_deg"]))
        bar_h = (el / 90.0) * (h - 20)
        self.canvas.create_rectangle(x0 + 20, y0 + h - 10 - bar_h, x0 + w - 20, y0 + h - 10, fill="#06d6a0")
        self.canvas.create_text(x0 + w / 2, y0 + h - bar_h - 20, fill="white", text=f"{el:.1f}°")

    def _update_ui(self):
        st1 = self.motor_1.get_status()
        st2 = self.motor_2.get_status()
        self._calc_selected_az_el()
        self._tracking_step()
        fault = ""
        if st1["fault_latched"] or st2["fault_latched"]:
            fault = f"\nFAULT: {st1['fault_msg']} {st2['fault_msg']}"

        if not SKYFIELD_AVAILABLE:
            self.lbl_sat.config(text=f"SAT: {self.selected_sat_name} | AZ: - | EL: -  (install: pip install skyfield)")
        elif self.sat_az is None or self.sat_el is None:
            self.lbl_sat.config(text=f"SAT: {self.selected_sat_name} | AZ: - | EL: -")
        else:
            self.lbl_sat.config(text=f"SAT: {self.selected_sat_name} | AZ: {self.sat_az:.2f}° | EL: {self.sat_el:.2f}°")

        self.lbl_status.config(
            text=(
                f"AZ: pos={st1['position_deg']:.2f}°  spd={st1['current_speed_sps']:.1f} sps  tgt={st1['target_speed_sps']:.1f}\n"
                f"EL: pos={st2['position_deg']:.2f}°  spd={st2['current_speed_sps']:.1f} sps  tgt={st2['target_speed_sps']:.1f}\n"
                f"Command speed={self.command_speed:.1f} sps | Microstep={st1['microstep']} | Track={'ON' if self.tracking_enabled else 'OFF'}{fault}"
            )
        )
        self._draw_rotator(st1, st2)
        self.root.after(50, self._update_ui)

    def run(self):
        self.root.mainloop()


def run_cli_mode(motor_1, motor_2, cfg_m1, cfg_m2):
    print("\n=== TB6600 ROTATOR CLI MODE ===")
    print("Type 'help' for commands.")

    command_speed = 600.0
    selected_tle = None
    selected_sat_name = "-"
    observer = {"lat": -6.2, "lon": 106.8, "alt_m": 50.0}
    tracking_enabled = False
    tle_cache = []
    pid_state = {
        "az": {"i": 0.0, "last_e": 0.0, "last_t": None},
        "el": {"i": 0.0, "last_e": 0.0, "last_t": None},
    }
    ts = load.timescale() if SKYFIELD_AVAILABLE else None

    def az_error(target_deg, current_deg):
        if not cfg_m1.az_wrap_enabled:
            return target_deg - current_deg
        e = target_deg - current_deg
        if e > 180.0:
            e -= 360.0
        elif e < -180.0:
            e += 360.0
        return e

    def gains(axis, eabs):
        if axis == "az":
            if eabs > 20: return 120.0, 0.20, 25.0
            if eabs > 5: return 85.0, 0.12, 18.0
            if eabs > 1: return 45.0, 0.06, 10.0
            return 20.0, 0.0, 6.0
        if eabs > 15: return 110.0, 0.18, 20.0
        if eabs > 4: return 80.0, 0.10, 15.0
        if eabs > 1: return 40.0, 0.05, 8.0
        return 18.0, 0.0, 5.0

    def pid(axis, err, max_speed):
        st = pid_state[axis]
        now = time.time()
        dt = 0.02 if st["last_t"] is None else max(0.001, now - st["last_t"])
        st["last_t"] = now
        kp, ki, kd = gains(axis, abs(err))
        st["i"] += err * dt
        st["i"] = max(-300.0, min(300.0, st["i"]))
        d = (err - st["last_e"]) / dt
        st["last_e"] = err
        out = kp * err + ki * st["i"] + kd * d
        if abs(err) < 0.08:
            out = 0.0
        return max(-max_speed, min(max_speed, out))

    def sat_az_el():
        if not selected_tle or not SKYFIELD_AVAILABLE:
            return None, None
        sat = EarthSatellite(selected_tle["line1"], selected_tle["line2"], selected_tle["name"], ts)
        t = ts.now()
        obs = Topos(latitude_degrees=observer["lat"], longitude_degrees=observer["lon"], elevation_m=observer["alt_m"])
        topocentric = (sat - obs).at(t)
        alt, az, _ = topocentric.altaz()
        return az.degrees, alt.degrees

    def tracking_step():
        if not tracking_enabled:
            return
        saz, sel = sat_az_el()
        if saz is None or sel is None:
            return
        st1 = motor_1.get_status()
        st2 = motor_2.get_status()
        cur_az = st1["position_deg"] % 360.0
        cur_el = st2["position_deg"]
        target_az = (saz + cfg_m1.az_offset_deg) % 360.0
        if cfg_m1.soft_limit_min_deg is not None:
            target_az = max(cfg_m1.soft_limit_min_deg, target_az)
        if cfg_m1.soft_limit_max_deg is not None:
            target_az = min(cfg_m1.soft_limit_max_deg, target_az)
        motor_1.set_target_speed(pid("az", az_error(target_az, cur_az), cfg_m1.max_speed_sps))
        motor_2.set_target_speed(pid("el", sel - cur_el, cfg_m2.max_speed_sps))

    def print_status():
        st1 = motor_1.get_status()
        st2 = motor_2.get_status()
        saz, sel = sat_az_el()
        sat_txt = f"{selected_sat_name} AZ={saz:.2f} EL={sel:.2f}" if saz is not None and sel is not None else selected_sat_name
        print(f"M1(AZ) pos={st1['position_deg']:.2f} spd={st1['current_speed_sps']:.1f} tgt={st1['target_speed_sps']:.1f}")
        print(f"M2(EL) pos={st2['position_deg']:.2f} spd={st2['current_speed_sps']:.1f} tgt={st2['target_speed_sps']:.1f}")
        print(f"speed={command_speed:.1f} ms={st1['microstep']} track={'ON' if tracking_enabled else 'OFF'} sat={sat_txt}")
        print(f"limits AZ[{cfg_m1.soft_limit_min_deg},{cfg_m1.soft_limit_max_deg}] EL[{cfg_m2.soft_limit_min_deg},{cfg_m2.soft_limit_max_deg}] offset={cfg_m1.az_offset_deg}")

    def run_arrow_control():
        nonlocal tracking_enabled, command_speed
        tracking_enabled = False
        print("\nArrow control mode:")
        print("  Right/D: AZ+   Left/A: AZ-")
        print("  Up/W: EL+      Down/S: EL-")
        print("  +/- speed, Space stop, Q exit arrow mode")
        print("  Hold key to move; release auto-stop.\n")
        idle_timeout = 0.15
        last_input_t = time.time()
        with RawTerminal():
            while True:
                key = get_key_nonblocking(0.03)
                now = time.time()
                if key is None:
                    if now - last_input_t > idle_timeout:
                        motor_1.stop_smooth()
                        motor_2.stop_smooth()
                    continue
                last_input_t = now
                if key in ("\x1b[C", "d", "D"):
                    motor_1.set_target_speed(abs(command_speed))
                elif key in ("\x1b[D", "a", "A"):
                    motor_1.set_target_speed(-abs(command_speed))
                elif key in ("\x1b[A", "w", "W"):
                    motor_2.set_target_speed(abs(command_speed))
                elif key in ("\x1b[B", "s", "S"):
                    motor_2.set_target_speed(-abs(command_speed))
                elif key == " ":
                    motor_1.stop_smooth(); motor_2.stop_smooth()
                elif key == "+":
                    command_speed = min(cfg_m1.max_speed_sps, command_speed + 100.0)
                    print(f"\nSpeed -> {command_speed:.1f} sps")
                elif key == "-":
                    command_speed = max(50.0, command_speed - 100.0)
                    print(f"\nSpeed -> {command_speed:.1f} sps")
                elif key in ("q", "Q"):
                    motor_1.stop_smooth(); motor_2.stop_smooth()
                    print("\nExit arrow mode.\n")
                    break

    while True:
        tracking_step()
        try:
            cmd = input("rotator> ").strip()
        except (KeyboardInterrupt, EOFError):
            print()
            break
        if not cmd:
            continue
        p = cmd.split()
        c = p[0].lower()
        try:
            if c in ("q", "quit", "exit"):
                break
            elif c == "help":
                print("status | az+ az- el+ el- stop estop reset")
                print("arrow  (direct keyboard arrows/WASD control)")
                print("speed <sps> | micro <1|2|4|8|16> | offset <deg>")
                print("limit az <min> <max> | limit el <min> <max>")
                print("obs <lat> <lon> <alt_m> | tle search <text> | tle select <idx> | track on|off")
            elif c == "status":
                print_status()
            elif c == "az+":
                tracking_enabled = False; motor_1.set_target_speed(abs(command_speed))
            elif c == "az-":
                tracking_enabled = False; motor_1.set_target_speed(-abs(command_speed))
            elif c == "el+":
                tracking_enabled = False; motor_2.set_target_speed(abs(command_speed))
            elif c == "el-":
                tracking_enabled = False; motor_2.set_target_speed(-abs(command_speed))
            elif c == "stop":
                tracking_enabled = False; motor_1.stop_smooth(); motor_2.stop_smooth()
            elif c == "estop":
                tracking_enabled = False; motor_1.emergency_stop("CLI E-STOP"); motor_2.emergency_stop("CLI E-STOP")
            elif c == "reset":
                motor_1.reset_fault(); motor_2.reset_fault()
            elif c == "speed" and len(p) == 2:
                command_speed = max(50.0, min(cfg_m1.max_speed_sps, float(p[1])))
            elif c == "micro" and len(p) == 2:
                v = int(p[1]); motor_1.set_microstep(v); motor_2.set_microstep(v)
            elif c == "offset" and len(p) == 2:
                cfg_m1.az_offset_deg = float(p[1])
            elif c == "limit" and len(p) == 4 and p[1].lower() == "az":
                mn, mx = float(p[2]), float(p[3]); 
                if mn >= mx: raise ValueError("AZ min must be < max")
                cfg_m1.soft_limit_min_deg, cfg_m1.soft_limit_max_deg = mn, mx
            elif c == "limit" and len(p) == 4 and p[1].lower() == "el":
                mn, mx = float(p[2]), float(p[3]); 
                if mn >= mx: raise ValueError("EL min must be < max")
                cfg_m2.soft_limit_min_deg, cfg_m2.soft_limit_max_deg = mn, mx
            elif c == "obs" and len(p) == 4:
                observer["lat"], observer["lon"], observer["alt_m"] = float(p[1]), float(p[2]), float(p[3])
            elif c == "tle" and len(p) >= 3 and p[1].lower() == "search":
                q = " ".join(p[2:])
                items = tle_fetch_collection(q, page=1, page_size=20)
                tle_cache = [x for x in (tle_extract_lines(i) for i in items) if x]
                if not tle_cache:
                    print("No TLE found")
                for i, it in enumerate(tle_cache, start=1):
                    sat_id = it.get("id")
                    suffix = f" [{sat_id}]" if sat_id is not None else ""
                    print(f"{i:02d}. {it['name']}{suffix}")
            elif c == "tle" and len(p) == 3 and p[1].lower() == "select":
                idx = int(p[2]) - 1
                if idx < 0 or idx >= len(tle_cache):
                    raise ValueError("invalid index")
                selected_tle = tle_cache[idx]; selected_sat_name = selected_tle["name"]
                print(f"Selected: {selected_sat_name}")
            elif c == "track" and len(p) == 2:
                tracking_enabled = (p[1].lower() == "on")
                if tracking_enabled:
                    now = time.time()
                    for k in ("az", "el"):
                        pid_state[k]["i"] = 0.0
                        pid_state[k]["last_e"] = 0.0
                        pid_state[k]["last_t"] = now
                else:
                    motor_1.stop_smooth(); motor_2.stop_smooth()
            elif c == "arrow":
                run_arrow_control()
            else:
                print("Unknown command. Type 'help'.")
        except Exception as exc:
            print(f"ERR: {exc}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", help="Jalankan mode simulasi UI (tanpa GPIO)")
    parser.add_argument("--sim-gui", action="store_true", help="Jalankan simulasi GUI (window)")
    parser.add_argument("--cli", action="store_true", help="Run interactive CLI mode")
    args = parser.parse_args()

    # Motor 1 (sesuai mapping user)
    cfg_m1 = StepperConfig(
        step_pin=17,  # PUL+
        dir_pin=27,   # DIR+
        en_pin=22,    # EN+
        steps_per_rev=200,
        microstep=2,   # sesuai konfigurasi: microstep 2/A
        max_speed_sps=2200.0,
        accel_sps2=3000.0,
        soft_limit_min_deg=0.0,
        soft_limit_max_deg=360.0,
    )
    # Motor 2 (sesuai mapping user)
    cfg_m2 = StepperConfig(
        step_pin=23,  # PUL+
        dir_pin=24,   # DIR+
        en_pin=25,    # EN+
        steps_per_rev=200,
        microstep=2,   # sesuai konfigurasi: microstep 2/A
        max_speed_sps=2200.0,
        accel_sps2=3000.0,
        soft_limit_min_deg=0.0,
        soft_limit_max_deg=90.0,
    )

    # --sim-gui semestinya selalu jalan di mode simulator.
    use_sim = args.sim or args.sim_gui or (not GPIO_AVAILABLE)
    use_sim_gui = args.sim_gui
    if use_sim:
        motor_1 = SimStepper(cfg_m1, "AZ")
        motor_2 = SimStepper(cfg_m2, "EL")
    else:
        motor_1 = TB6600Stepper(cfg_m1)
        motor_2 = TB6600Stepper(cfg_m2)

    command_speed = 600.0
    last_report = 0.0

    try:
        if args.cli:
            run_cli_mode(motor_1, motor_2, cfg_m1, cfg_m2)
        elif use_sim and use_sim_gui:
            if not TK_AVAILABLE:
                raise RuntimeError("tkinter tidak tersedia. Install tkinter atau jalankan tanpa --sim-gui.")
            if not os.environ.get("DISPLAY"):
                raise RuntimeError("DISPLAY tidak terdeteksi. Jalankan dari desktop Raspberry Pi atau pakai X11 forwarding.")
            app = SimGuiApp(motor_1, motor_2, cfg_m1, cfg_m2)
            app.run()
        elif use_sim:
            with RawTerminal():
                while True:
                    now = time.time()
                    if now - last_report > 0.1:
                        st1 = motor_1.get_status()
                        st2 = motor_2.get_status()
                        draw_sim_ui(st1, st2, command_speed)
                        last_report = now
                    key = get_key_nonblocking(0.03)
                    if key is None:
                        continue
                    if key in ("\x1b[C", "d", "D"):
                        motor_1.set_target_speed(abs(command_speed))
                    elif key in ("\x1b[D", "a", "A"):
                        motor_1.set_target_speed(-abs(command_speed))
                    elif key in ("\x1b[A", "w", "W"):
                        motor_2.set_target_speed(abs(command_speed))
                    elif key in ("\x1b[B", "s", "S"):
                        motor_2.set_target_speed(-abs(command_speed))
                    elif key == " ":
                        motor_1.stop_smooth()
                        motor_2.stop_smooth()
                    elif key in ("e", "E"):
                        motor_1.emergency_stop("Emergency stop keyboard")
                        motor_2.emergency_stop("Emergency stop keyboard")
                    elif key in ("r", "R"):
                        motor_1.reset_fault()
                        motor_2.reset_fault()
                    elif key == "+":
                        command_speed = min(cfg_m1.max_speed_sps, command_speed + 100.0)
                    elif key == "-":
                        command_speed = max(50.0, command_speed - 100.0)
                    elif key == "1":
                        motor_1.set_microstep(1); motor_2.set_microstep(1)
                    elif key == "2":
                        motor_1.set_microstep(2); motor_2.set_microstep(2)
                    elif key == "3":
                        motor_1.set_microstep(4); motor_2.set_microstep(4)
                    elif key == "4":
                        motor_1.set_microstep(8); motor_2.set_microstep(8)
                    elif key == "5":
                        motor_1.set_microstep(16); motor_2.set_microstep(16)
                    elif key in ("q", "Q"):
                        break
        else:
            print(
                "\n=== DUAL TB6600 KEYBOARD STEPPER CONTROL ===\n"
                "Motor 1 (GPIO17/27/22): Arrow Left/Right atau A/D\n"
                "Motor 2 (GPIO23/24/25): Arrow Up/Down  atau W/S\n"
                "Space            : Smooth stop kedua motor\n"
                "E                : Emergency stop kedua motor (latch)\n"
                "R                : Reset fault kedua motor\n"
                "+ / -           : Speed up / down\n"
                "1/2/3/4/5       : Set microstep kedua motor = 1/2/4/8/16\n"
                "Q               : Quit\n"
            )
            while True:
                now = time.time()
                if now - last_report > 0.25:
                    st1 = motor_1.get_status()
                    st2 = motor_2.get_status()
                    fault1 = f" F1={st1['fault_msg']}" if st1["fault_latched"] else ""
                    fault2 = f" F2={st2['fault_msg']}" if st2["fault_latched"] else ""
                    sys.stdout.write(
                        f"\rM1 POS={st1['position_deg']:7.2f} SPD={st1['current_speed_sps']:7.1f} "
                        f"| M2 POS={st2['position_deg']:7.2f} SPD={st2['current_speed_sps']:7.1f} "
                        f"| MS={st1['microstep']:2d}{fault1}{fault2}       "
                    )
                    sys.stdout.flush()
                    last_report = now

                key = get_key()

                if key in ("\x1b[C", "d", "D"):
                    motor_1.set_target_speed(abs(command_speed))
                elif key in ("\x1b[D", "a", "A"):
                    motor_1.set_target_speed(-abs(command_speed))
                elif key in ("\x1b[A", "w", "W"):
                    motor_2.set_target_speed(abs(command_speed))
                elif key in ("\x1b[B", "s", "S"):
                    motor_2.set_target_speed(-abs(command_speed))
                elif key == " ":
                    motor_1.stop_smooth()
                    motor_2.stop_smooth()
                elif key in ("e", "E"):
                    motor_1.emergency_stop("Emergency stop keyboard")
                    motor_2.emergency_stop("Emergency stop keyboard")
                elif key in ("r", "R"):
                    motor_1.reset_fault()
                    motor_2.reset_fault()
                elif key == "+":
                    command_speed = min(cfg_m1.max_speed_sps, command_speed + 100.0)
                elif key == "-":
                    command_speed = max(50.0, command_speed - 100.0)
                elif key == "1":
                    motor_1.set_microstep(1); motor_2.set_microstep(1)
                elif key == "2":
                    motor_1.set_microstep(2); motor_2.set_microstep(2)
                elif key == "3":
                    motor_1.set_microstep(4); motor_2.set_microstep(4)
                elif key == "4":
                    motor_1.set_microstep(8); motor_2.set_microstep(8)
                elif key == "5":
                    motor_1.set_microstep(16); motor_2.set_microstep(16)
                elif key in ("q", "Q"):
                    break
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"\nERROR runtime: {exc}")
    finally:
        print("\nShutdown controller...")
        motor_1.close()
        motor_2.close()
        if GPIO_AVAILABLE and not use_sim:
            GPIO.cleanup()
        print("GPIO cleaned up.")


if __name__ == "__main__":
    main()
