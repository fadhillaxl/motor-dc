#!/usr/bin/env python3
# coding: utf-8
"""
TB6600 Dual Stepper Controller + WT901 Closed-Loop AZ/EL Targeting

This script drives a dual-axis antenna system to a precise target:
- Azimuth target: 20 deg
- Elevation target: 94 deg

Key features:
- TB6600 control (STEP / DIR / ENABLE) with acceleration ramp.
- Safety checks (limit switches, overcurrent, estop).
- WT901 sensor acquisition/reset/error handling replicated from fix-compas.py.
- Closed-loop correction using AZ/EL feedback from WT901.
- Logging of start/target/end position and correction steps.
"""

import json
import logging
import math
import os
import platform
import socket
import sys
import threading
import time
import argparse
from dataclasses import dataclass

try:
    import RPi.GPIO as GPIO
except Exception as exc:
    print(f"ERROR: gagal import RPi.GPIO: {exc}")
    print("Jalankan file ini di Raspberry Pi dengan library RPi.GPIO terpasang.")
    sys.exit(1)

# =============================
# PATH SDK (same style as fix-compas.py)
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
    WT901_AVAILABLE = True
except Exception as exc:
    WT901_AVAILABLE = False
    print(f"ERROR: gagal import WT901 SDK: {exc}")
    print(f"Pastikan path SDK valid: {SDK_CHS}")
    sys.exit(1)


# =============================
# GLOBAL TARGETS / LIMITS
# =============================
POSITION_TOL_DEG = 0.5
CONTROL_INTERVAL_S = 0.05
CONTROL_TIMEOUT_S = 120.0
CONTROL_KP_AZ = 18.0
CONTROL_KP_EL = 18.0
CONTROL_MIN_SPS = 80.0
CONTROL_MAX_SPS_EL = 300.0
AZ_SOFT_LIMIT_DEG = 280.0
EL_MIN_DEG = 0.0
EL_MAX_DEG = 90.0
TRACKING_EPS_DEG = 0.1
LOG_FILE = os.path.join(BASE_DIR, "az_el_closed_loop.log")
ROTCTL_DEFAULT_HOST = "127.0.0.1"
ROTCTL_DEFAULT_PORT = 4533


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
    # AZ biasanya circular (0-360 wrap), jadi soft-limit linear bisa dinonaktifkan.
    circular_axis: bool = False


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

    def set_position_deg(self, deg: float):
        """Sync internal position estimate to an external absolute sensor."""
        with self._lock:
            self._position_full_steps = (float(deg) / 360.0) * float(self.cfg.steps_per_rev)

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
        if self.cfg.circular_axis:
            return False
        mn = self.cfg.soft_limit_min_deg
        mx = self.cfg.soft_limit_max_deg
        if mn is not None and next_deg < mn:
            return True
        if mx is not None and next_deg > mx:
            return True
        return False

    def _soft_stop_at_limit(self, moving_positive: bool):
        """
        Soft stop saat menyentuh batas software.
        Tidak melatch fault agar kontrol bisa recovery dan lanjut operasi.
        """
        with self._lock:
            self._target_speed_sps = 0.0
            self._current_speed_sps = 0.0
            boundary = self.cfg.soft_limit_max_deg if moving_positive else self.cfg.soft_limit_min_deg
            if boundary is not None:
                self._position_full_steps = (float(boundary) / 360.0) * float(self.cfg.steps_per_rev)

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
                self._soft_stop_at_limit(cw)
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


def angle_diff(a: float, b: float) -> float:
    """Shortest circular angle diff in degree (-180..180)."""
    return (a - b + 180.0) % 360.0 - 180.0


def angle_lerp(new: float, old: float | None, alpha: float) -> float:
    """Circular smoothing with wrap-safe interpolation."""
    if old is None:
        return new % 360.0
    d = angle_diff(new, old)
    return (old + alpha * d) % 360.0


def map_roll_to_el(roll_deg: float, el_offset_deg: float = 0.0) -> float:
    """
    Map roll to elevation with user convention:
    - roll ~= 90  -> EL = 0 (front)
    - roll ~= 180 -> EL = 90 (up)
    """
    el = (float(roll_deg) - 90.0) + float(el_offset_deg)
    return max(EL_MIN_DEG, min(EL_MAX_DEG, el))


class AZLimitSwitch:
    """Logika limit switch software AZ dengan batas crossing di sudut tertentu."""

    def __init__(self, limit_deg: float = AZ_SOFT_LIMIT_DEG):
        self.limit_deg = float(limit_deg) % 360.0

    @staticmethod
    def _norm360(deg: float) -> float:
        return float(deg) % 360.0

    def _crosses_limit(self, current_deg: float, target_deg: float, direction: int) -> bool:
        """Cek apakah lintasan CW/CCW menyeberang titik limit AZ."""
        c = self._norm360(current_deg)
        t = self._norm360(target_deg)
        b = self.limit_deg
        if direction > 0:
            t_u = t if t >= c else t + 360.0
            b_u = b if b >= c else b + 360.0
            return c < b_u <= t_u
        t_u = t if t <= c else t - 360.0
        b_u = b if b <= c else b - 360.0
        return t_u <= b_u < c

    def detectMovementDirection(self, prev_deg: float, curr_deg: float, eps_deg: float = TRACKING_EPS_DEG) -> int:
        """Deteksi arah gerak saat ini: +1 (CW), -1 (CCW), 0 (diam)."""
        d = angle_diff(curr_deg, prev_deg)
        if abs(d) <= eps_deg:
            return 0
        return 1 if d > 0 else -1

    def calculateShortestPath(self, current_deg: float, target_deg: float) -> dict:
        """
        Hitung lintasan AZ terbaik dengan mempertimbangkan batas 280°.
        Return direction + distance + status boleh/tidak.
        """
        c = self._norm360(current_deg)
        t = self._norm360(target_deg)
        cw_dist = (t - c) % 360.0
        ccw_dist = (c - t) % 360.0
        cw_cross = self._crosses_limit(c, t, +1)
        ccw_cross = self._crosses_limit(c, t, -1)

        options = []
        if not cw_cross:
            options.append((cw_dist, +1))
        if not ccw_cross:
            options.append((ccw_dist, -1))
        options.sort(key=lambda item: item[0])
        if options:
            dist, direction = options[0]
            return {
                "allowed": True,
                "direction": direction,
                "distance_deg": dist,
                "cw_distance_deg": cw_dist,
                "ccw_distance_deg": ccw_dist,
                "cw_cross_limit": cw_cross,
                "ccw_cross_limit": ccw_cross,
                "reason": "shortest_allowed_path",
            }
        return {
            "allowed": False,
            "direction": 0,
            "distance_deg": 0.0,
            "cw_distance_deg": cw_dist,
            "ccw_distance_deg": ccw_dist,
            "cw_cross_limit": cw_cross,
            "ccw_cross_limit": ccw_cross,
            "reason": "both_paths_cross_limit",
        }

    @staticmethod
    def reverseDirection(current_direction: int) -> int:
        """Paksa arah berlawanan saat mencapai batas."""
        if current_direction > 0:
            return -1
        if current_direction < 0:
            return 1
        return 0

    def validateMovement(self, current_deg: float, target_deg: float, current_direction: int = 0) -> dict:
        """
        Validasi request gerak AZ:
        - tentukan lintasan terpendek yang diperbolehkan
        - jika arah aktif menabrak limit, paksa reverse
        """
        decision = self.calculateShortestPath(current_deg, target_deg)
        if decision["allowed"]:
            decision["reverse_required"] = bool(
                current_direction != 0 and decision["direction"] != current_direction
            )
            return decision
        reverse_dir = self.reverseDirection(current_direction)
        decision["reverse_required"] = reverse_dir != 0
        decision["forced_direction"] = reverse_dir
        return decision


class ELLimitSwitch:
    """Validasi software limit switch EL dalam rentang 0..90 derajat."""

    def __init__(self, min_deg: float = EL_MIN_DEG, max_deg: float = EL_MAX_DEG):
        self.min_deg = float(min_deg)
        self.max_deg = float(max_deg)

    def validateElevation(self, target_el_deg: float, current_el_deg: float | None = None) -> dict:
        """
        Validasi command EL sebelum dieksekusi.
        - target harus di [0, 90]
        - soft stop jika posisi sudah di batas dan command mendorong keluar batas
        """
        target = float(target_el_deg)
        if target < self.min_deg or target > self.max_deg:
            return {
                "allowed": False,
                "clamped_target_deg": max(self.min_deg, min(self.max_deg, target)),
                "reason": "target_out_of_range",
            }

        if current_el_deg is None:
            return {"allowed": True, "clamped_target_deg": target, "reason": "ok"}

        curr = float(current_el_deg)
        if curr <= self.min_deg + TRACKING_EPS_DEG and target < curr:
            return {"allowed": False, "clamped_target_deg": self.min_deg, "reason": "soft_stop_lower"}
        if curr >= self.max_deg - TRACKING_EPS_DEG and target > curr:
            return {"allowed": False, "clamped_target_deg": self.max_deg, "reason": "soft_stop_upper"}
        return {"allowed": True, "clamped_target_deg": target, "reason": "ok"}


class LimitRecoveryManager:
    """
    Manajer recovery limit:
    - Tidak langsung menghentikan task saat limit tercapai
    - Menentukan strategi koreksi (reverse escape, re-route, clamp target)
    - Menyesuaikan parameter (speed scale) saat limit berulang
    - Menyediakan notifikasi untuk monitoring
    """

    def __init__(
        self,
        az_limit: AZLimitSwitch,
        el_limit: ELLimitSwitch,
        logger: logging.Logger,
        notifier=None,
    ):
        self.az_limit = az_limit
        self.el_limit = el_limit
        self.logger = logger
        self.notifier = notifier
        self.az_limit_hits = 0
        self.el_limit_hits = 0
        self.az_recovery_target = None

    def _notify(self, event: str, message: str, payload: dict):
        self.logger.warning("[LIMIT:%s] %s | %s", event, message, json.dumps(payload, separators=(",", ":")))
        if callable(self.notifier):
            try:
                self.notifier(event, message, payload)
            except Exception as exc:
                self.logger.warning("Notifier failed: %s", exc)

    def on_cycle_ok(self):
        self.az_limit_hits = max(0, self.az_limit_hits - 1)
        self.el_limit_hits = max(0, self.el_limit_hits - 1)
        if self.az_limit_hits == 0:
            self.az_recovery_target = None

    def az_speed_scale(self) -> float:
        # Adaptive speed downscale saat limit berulang.
        if self.az_limit_hits >= 6:
            return 0.4
        if self.az_limit_hits >= 3:
            return 0.6
        return 1.0

    def recover_az(self, current_az: float, target_az: float, current_direction: int, az_decision: dict) -> dict:
        """
        Recovery AZ saat movement tidak valid:
        - Jika forced_direction tersedia -> gunakan
        - Jika tidak, pilih arah escape dari posisi saat ini
        - Set temporary recovery target agar sistem keluar dari zona kritis
        """
        if az_decision["allowed"]:
            return {
                "recovered": False,
                "target_az": target_az,
                "az_dir": int(az_decision["direction"]),
                "err_az": float(az_decision["distance_deg"]) * float(az_decision["direction"]),
                "reason": az_decision["reason"],
            }

        forced_dir = int(az_decision.get("forced_direction", 0))
        if forced_dir == 0:
            # fallback heuristic: tentukan arah berdasarkan posisi terhadap boundary.
            rel = (float(current_az) - self.az_limit.limit_deg + 360.0) % 360.0
            forced_dir = 1 if rel < 180.0 else -1
            if current_direction != 0:
                forced_dir = AZLimitSwitch.reverseDirection(current_direction)

        recovery_target = (float(current_az) + (forced_dir * 8.0)) % 360.0
        self.az_recovery_target = recovery_target
        self.az_limit_hits += 1

        payload = {
            "current_az": round(float(current_az), 3),
            "requested_target_az": round(float(target_az), 3),
            "recovery_target_az": round(float(recovery_target), 3),
            "forced_direction": forced_dir,
            "hit_count": self.az_limit_hits,
            "reason": az_decision.get("reason", "unknown"),
        }
        self._notify("AZ_RECOVERY", "AZ limit tercapai, alihkan ke jalur alternatif.", payload)

        return {
            "recovered": True,
            "target_az": recovery_target,
            "az_dir": forced_dir,
            "err_az": float(forced_dir) * 8.0,
            "reason": "recovery_reverse_escape",
        }

    def recover_el(self, target_el: float, current_el: float, el_decision: dict) -> dict:
        """
        Recovery EL:
        - Clamp target ke range 0..90
        - Soft-stop command jika mendorong keluar batas
        """
        effective_target = float(el_decision["clamped_target_deg"])
        if el_decision["allowed"]:
            return {
                "recovered": False,
                "target_el": effective_target,
                "allow_motion": True,
                "reason": "ok",
            }

        self.el_limit_hits += 1
        payload = {
            "current_el": round(float(current_el), 3),
            "requested_target_el": round(float(target_el), 3),
            "effective_target_el": round(effective_target, 3),
            "hit_count": self.el_limit_hits,
            "reason": el_decision.get("reason", "unknown"),
        }
        self._notify("EL_RECOVERY", "EL limit aktif, lakukan soft-stop / clamp target.", payload)
        return {
            "recovered": True,
            "target_el": effective_target,
            "allow_motion": False,
            "reason": el_decision.get("reason", "unknown"),
        }


class WT901AxisReader:
    """WT901 reader that mirrors the acquisition flow from fix-compas.py."""

    def __init__(
        self,
        label: str,
        addr: int,
        az_offset_deg: float = 0.0,
        el_offset_deg: float = 0.0,
        alpha: float = 0.15,
    ):
        self.label = label
        self.addr = addr
        self.az_offset_deg = az_offset_deg
        self.el_offset_deg = el_offset_deg
        self.alpha = alpha
        self.last_az = None
        self.device = self._buat_device_model()
        self.device.ADDR = addr

        if platform.system().lower() == "linux":
            self.device.serialConfig.portName = "/dev/ttyUSB0"
        else:
            self.device.serialConfig.portName = "/dev/tty.usbserial-1330"
        self.device.serialConfig.baud = 9600

    @staticmethod
    def _buat_device_model():
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

    def open(self):
        self.device.openDevice()
        time.sleep(1.0)

    def close(self):
        try:
            # SDK thread closes cleaner if we drop isOpen before closing the file descriptor.
            if hasattr(self.device, "isOpen"):
                self.device.isOpen = False
            time.sleep(0.1)
            sp = getattr(self.device, "serialPort", None)
            if sp is not None:
                try:
                    sp.close()
                except Exception:
                    pass
                try:
                    self.device.serialPort = None
                except Exception:
                    pass
        except Exception:
            pass

    def reset_zero_point(self):
        # Replicated behavior from fix-compas.py
        try:
            print(f"[INFO][{self.label}] Reset zero-point...")
            if hasattr(self.device, "write_register"):
                self.device.write_register(self.device.ADDR, 0x69, 0xB588)
                time.sleep(0.1)
                self.device.write_register(self.device.ADDR, 0x01, 0x0000)
            else:
                if hasattr(self.device, "unlock"):
                    self.device.unlock()
                    time.sleep(0.1)
                self.device.writeReg(0x01, 0x0000)
                if hasattr(self.device, "save"):
                    time.sleep(0.1)
                    self.device.save()
            time.sleep(0.3)
            print(f"[OK][{self.label}] Zero reset")
        except Exception as exc:
            print(f"[WARN][{self.label}] {exc}")

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
        return (heading + 360.0) % 360.0

    def read(self) -> dict | None:
        # Replicated acquisition + error handling style from fix-compas.py
        try:
            if hasattr(self.device, "readReg"):
                self.device.readReg(0x30, 41)

            if hasattr(self.device, "get"):
                roll = self.device.get("AngleX")
                pitch = self.device.get("AngleY")
                yaw = self.device.get("AngleZ")
                accX = self.device.get("accX")
                accY = self.device.get("accY")
                accZ = self.device.get("accZ")
                magX = self.device.get("magX")
                magY = self.device.get("magY")
                magZ = self.device.get("magZ")
            else:
                roll = self.device.getDeviceData("angleX")
                pitch = self.device.getDeviceData("angleY")
                yaw = self.device.getDeviceData("angleZ")
                accX = self.device.getDeviceData("accX")
                accY = self.device.getDeviceData("accY")
                accZ = self.device.getDeviceData("accZ")
                magX = self.device.getDeviceData("magX")
                magY = self.device.getDeviceData("magY")
                magZ = self.device.getDeviceData("magZ")

            if None in (roll, pitch, yaw):
                return None

            roll = float(roll)
            pitch = float(pitch)
            yaw = float(yaw) % 360.0

            # TILT dari ACC (samakan dengan fix-compas.py)
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
                compass = self._tilt_compass(
                    float(magX),
                    float(magY),
                    float(magZ),
                    roll_tilt,
                    pitch_tilt,
                )

            yaw_cw = (360.0 - yaw + self.az_offset_deg) % 360.0
            compass_cw = (
                (360.0 - compass + self.az_offset_deg) % 360.0
                if compass is not None
                else None
            )

            az = yaw_cw
            src = "YAW"
            if compass_cw is not None:
                w = math.cos(math.radians(roll_tilt)) * math.cos(math.radians(pitch_tilt))
                w = max(0.0, w)
                az = (1.0 - w) * yaw_cw + w * compass_cw
                src = f"BLEND({w:.2f})"

            az = angle_lerp(az, self.last_az, self.alpha)
            self.last_az = az

            # EL pakai mapping ROLL -> EL: depan=0, atas=90.
            el = map_roll_to_el(roll, self.el_offset_deg)

            return {
                "roll": roll,
                "pitch": pitch,
                "roll_tilt": roll_tilt,
                "pitch_tilt": pitch_tilt,
                "el_roll": el,
                "yaw_cw": yaw_cw,
                "compass_cw": compass_cw,
                "az": az,
                "el": el,
                "src": src,
            }
        except Exception as exc:
            print(f"[ERR][{self.label}] {exc}")
            return None

    def read_with_retry(self, attempts: int = 30, delay_s: float = 0.05) -> dict | None:
        """Retry wrapper for unstable serial startup/first reads."""
        for _ in range(max(1, int(attempts))):
            pkt = self.read()
            if pkt is not None:
                return pkt
            time.sleep(max(0.0, float(delay_s)))
        return None


def setup_logger() -> logging.Logger:
    logger = logging.getLogger("az_el_closed_loop")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")

    sh = logging.StreamHandler(sys.stdout)
    sh.setLevel(logging.INFO)
    sh.setFormatter(formatter)
    logger.addHandler(sh)

    fh = logging.FileHandler(LOG_FILE, mode="a", encoding="utf-8")
    fh.setLevel(logging.INFO)
    fh.setFormatter(formatter)
    logger.addHandler(fh)
    return logger


class ClosedLoopAzElController:
    def __init__(
        self,
        motor_az: TB6600Stepper,
        motor_el: TB6600Stepper,
        wt: WT901AxisReader,
        logger: logging.Logger,
        notifier=None,
    ):
        self.motor_az = motor_az
        self.motor_el = motor_el
        self.wt = wt
        self.logger = logger
        self.corrections: list[dict] = []
        self.az_limit = AZLimitSwitch(AZ_SOFT_LIMIT_DEG)
        self.el_limit = ELLimitSwitch(EL_MIN_DEG, EL_MAX_DEG)
        self.recovery = LimitRecoveryManager(self.az_limit, self.el_limit, logger, notifier=notifier)
        self._last_cmd_az_dir = 0

    @staticmethod
    def _speed_from_error(err_deg: float, kp: float, max_sps: float) -> float:
        cmd = kp * err_deg
        if abs(cmd) < CONTROL_MIN_SPS and abs(err_deg) > 0.05:
            cmd = CONTROL_MIN_SPS if cmd >= 0 else -CONTROL_MIN_SPS
        return max(-max_sps, min(max_sps, cmd))

    def _read_azel(self) -> tuple[float, float, dict, dict] | None:
        # Use one WT901 packet for both AZ and EL to avoid multi-access on serial port.
        data = self.wt.read_with_retry(attempts=8, delay_s=0.03)
        if data is None:
            return None
        return data["az"], data["el"], data, data

    def drive_to_target(
        self,
        target_az_deg: float,
        target_el_deg: float,
        tolerance_deg: float = POSITION_TOL_DEG,
        timeout_s: float = CONTROL_TIMEOUT_S,
    ) -> tuple[bool, dict]:
        self.corrections.clear()
        start_packet = self._read_azel()
        if start_packet is None:
            raise RuntimeError("Gagal membaca posisi awal AZ/EL dari WT901.")

        start_az, start_el, start_az_raw, start_el_raw = start_packet
        self.logger.info(
            "START position | az=%.3f el=%.3f | target az=%.3f el=%.3f",
            start_az,
            start_el,
            target_az_deg,
            target_el_deg,
        )
        self.logger.info(
            "START raw | az_src=%s az_yaw=%.3f az_compass=%s el_roll=%.3f",
            start_az_raw["src"],
            start_az_raw["yaw_cw"],
            "-" if start_az_raw["compass_cw"] is None else f"{start_az_raw['compass_cw']:.3f}",
            start_el_raw["el_roll"],
        )

        stable_hits = 0
        t0 = time.time()
        max_speed_az = float(self.motor_az.cfg.max_speed_sps)
        max_speed_el = min(float(self.motor_el.cfg.max_speed_sps), CONTROL_MAX_SPS_EL)
        sensor_fail_count = 0

        while True:
            # Latch validation from hard safety / limit switches / estop
            st_az = self.motor_az.get_status()
            st_el = self.motor_el.get_status()
            if st_az["fault_latched"] or st_el["fault_latched"]:
                self.motor_az.emergency_stop("Fault latched during closed-loop move")
                self.motor_el.emergency_stop("Fault latched during closed-loop move")
                self.logger.error("FAULT LATCHED | az=%s | el=%s", st_az["fault_msg"], st_el["fault_msg"])
                break

            pkt = self._read_azel()
            if pkt is None:
                sensor_fail_count += 1
                self.logger.warning("Sensor read failed (%d).", sensor_fail_count)
                if sensor_fail_count > 20:
                    self.motor_az.emergency_stop("Too many WT901 read failures")
                    self.motor_el.emergency_stop("Too many WT901 read failures")
                    break
                time.sleep(CONTROL_INTERVAL_S)
                continue
            sensor_fail_count = 0

            curr_az, curr_el, _, _ = pkt
            # Keep internal motor position estimate aligned with absolute sensor feedback.
            # This prevents false soft-limit trips when step->deg model differs from mechanics.
            self.motor_az.set_position_deg(curr_az)
            self.motor_el.set_position_deg(curr_el)

            az_decision = self.az_limit.validateMovement(curr_az, target_az_deg, self._last_cmd_az_dir)
            el_decision = self.el_limit.validateElevation(target_el_deg, curr_el)
            effective_target_el = float(el_decision["clamped_target_deg"])

            az_recovery = self.recovery.recover_az(curr_az, target_az_deg, self._last_cmd_az_dir, az_decision)
            el_recovery = self.recovery.recover_el(target_el_deg, curr_el, el_decision)
            az_dir = int(az_recovery["az_dir"])
            err_az = float(az_recovery["err_az"])
            err_el = float(el_recovery["target_el"]) - curr_el

            in_tol = abs(err_az) <= tolerance_deg and abs(err_el) <= tolerance_deg
            if in_tol:
                stable_hits += 1
                self.motor_az.stop_smooth()
                self.motor_el.stop_smooth()
                self.recovery.on_cycle_ok()
            else:
                stable_hits = 0
                cmd_az = self._speed_from_error(err_az, CONTROL_KP_AZ, max_speed_az)
                cmd_az *= self.recovery.az_speed_scale()
                cmd_el = self._speed_from_error(err_el, CONTROL_KP_EL, max_speed_el)
                if not el_recovery["allow_motion"]:
                    cmd_el = 0.0
                    self.logger.warning("EL limited by software stop: %s", el_recovery["reason"])
                self.motor_az.set_target_speed(cmd_az)
                self.motor_el.set_target_speed(cmd_el)
                self._last_cmd_az_dir = 1 if cmd_az > 0 else (-1 if cmd_az < 0 else 0)

                corr = {
                    "t": round(time.time() - t0, 3),
                    "curr_az": round(curr_az, 3),
                    "curr_el": round(curr_el, 3),
                    "err_az": round(err_az, 3),
                    "err_el": round(err_el, 3),
                    "az_dir": az_dir,
                    "az_reason": az_recovery["reason"],
                    "el_reason": el_recovery["reason"],
                    "cmd_az_sps": round(cmd_az, 3),
                    "cmd_el_sps": round(cmd_el, 3),
                }
                self.corrections.append(corr)
                self.logger.info("CORR %s", json.dumps(corr, separators=(",", ":")))

            if stable_hits >= 5:
                self.logger.info("Stable target lock reached.")
                break
            if (time.time() - t0) > timeout_s:
                self.logger.error("Timeout while moving to target.")
                break

            time.sleep(CONTROL_INTERVAL_S)

        self.motor_az.stop_smooth()
        self.motor_el.stop_smooth()
        time.sleep(0.5)

        end_packet = self._read_azel()
        if end_packet is None:
            raise RuntimeError("Gagal membaca posisi akhir AZ/EL dari WT901.")
        end_az, end_el, _, _ = end_packet

        end_err_az = angle_diff(target_az_deg, end_az)
        end_err_el = target_el_deg - end_el
        success = abs(end_err_az) <= tolerance_deg and abs(end_err_el) <= tolerance_deg

        report = {
            "start_position": {"az": start_az, "el": start_el},
            "target_position": {"az": target_az_deg, "el": target_el_deg},
            "actual_end_position": {"az": end_az, "el": end_el},
            "final_error": {"az": end_err_az, "el": end_err_el},
            "tolerance_deg": tolerance_deg,
            "success": success,
            "correction_steps": self.corrections,
        }
        self.logger.info("END report %s", json.dumps(report, separators=(",", ":")))
        return success, report


class RealtimeAzElController:
    """
    Continuous AZ/EL closed-loop controller used by rotctl server mode.
    """

    def __init__(
        self,
        motor_az: TB6600Stepper,
        motor_el: TB6600Stepper,
        wt: WT901AxisReader,
        logger: logging.Logger,
        notifier=None,
    ):
        self.motor_az = motor_az
        self.motor_el = motor_el
        self.wt = wt
        self.logger = logger

        self._lock = threading.Lock()
        self._run = False
        self._thread = None

        self._target_az = 0.0
        self._target_el = 0.0
        self._curr_az = 0.0
        self._curr_el = 0.0
        self._has_position = False
        self._last_cmd_az_dir = 0
        self.az_limit = AZLimitSwitch(AZ_SOFT_LIMIT_DEG)
        self.el_limit = ELLimitSwitch(EL_MIN_DEG, EL_MAX_DEG)
        self.recovery = LimitRecoveryManager(self.az_limit, self.el_limit, logger, notifier=notifier)

    @staticmethod
    def _speed_from_error(err_deg: float, kp: float, max_sps: float) -> float:
        cmd = kp * err_deg
        if abs(cmd) < CONTROL_MIN_SPS and abs(err_deg) > 0.05:
            cmd = CONTROL_MIN_SPS if cmd >= 0 else -CONTROL_MIN_SPS
        return max(-max_sps, min(max_sps, cmd))

    def _read_azel(self) -> tuple[float, float] | None:
        data = self.wt.read_with_retry(attempts=8, delay_s=0.03)
        if data is None:
            return None
        return data["az"], data["el"]

    def start(self):
        boot = self._read_azel()
        if boot is None:
            raise RuntimeError("Unable to read initial AZ/EL for realtime control.")
        az, el = boot
        with self._lock:
            self._curr_az = az
            self._curr_el = el
            self._target_az = az
            self._target_el = el
            self._has_position = True
        self.motor_az.set_position_deg(az)
        self.motor_el.set_position_deg(el)

        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.logger.info("Realtime control loop started at az=%.2f el=%.2f", az, el)

    def stop(self):
        self._run = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.motor_az.stop_smooth()
        self.motor_el.stop_smooth()

    def set_target(self, az_deg: float, el_deg: float) -> tuple[bool, str]:
        el_decision = self.el_limit.validateElevation(float(el_deg), self._curr_el)
        if not el_decision["allowed"] and el_decision["reason"] == "target_out_of_range":
            return False, "target_el_out_of_range"
        with self._lock:
            self._target_az = float(az_deg) % 360.0
            self._target_el = float(el_decision["clamped_target_deg"])
        self.logger.info("Target updated | az=%.2f el=%.2f", self._target_az, self._target_el)
        return True, "ok"

    def stop_motion(self):
        with self._lock:
            self._target_az = self._curr_az
            self._target_el = self._curr_el
        self.motor_az.stop_smooth()
        self.motor_el.stop_smooth()
        self.logger.info("Stop motion requested.")

    def get_position(self) -> tuple[float, float]:
        with self._lock:
            return self._curr_az, self._curr_el

    def get_target(self) -> tuple[float, float]:
        with self._lock:
            return self._target_az, self._target_el

    def _loop(self):
        max_speed_az = float(self.motor_az.cfg.max_speed_sps)
        max_speed_el = min(float(self.motor_el.cfg.max_speed_sps), CONTROL_MAX_SPS_EL)
        sensor_fail_count = 0
        last_log_t = 0.0

        while self._run:
            st_az = self.motor_az.get_status()
            st_el = self.motor_el.get_status()
            if st_az["fault_latched"] or st_el["fault_latched"]:
                self.motor_az.stop_smooth()
                self.motor_el.stop_smooth()
                self.logger.error("Fault latched in realtime loop | az=%s el=%s", st_az["fault_msg"], st_el["fault_msg"])
                time.sleep(0.1)
                continue

            pkt = self._read_azel()
            if pkt is None:
                sensor_fail_count += 1
                if sensor_fail_count % 10 == 0:
                    self.logger.warning("Realtime sensor read failed x%d", sensor_fail_count)
                time.sleep(CONTROL_INTERVAL_S)
                continue
            sensor_fail_count = 0

            curr_az, curr_el = pkt
            with self._lock:
                self._curr_az = curr_az
                self._curr_el = curr_el
                target_az = self._target_az
                target_el = self._target_el
                self._has_position = True

            self.motor_az.set_position_deg(curr_az)
            self.motor_el.set_position_deg(curr_el)

            az_decision = self.az_limit.validateMovement(curr_az, target_az, self._last_cmd_az_dir)
            el_decision = self.el_limit.validateElevation(target_el, curr_el)
            az_recovery = self.recovery.recover_az(curr_az, target_az, self._last_cmd_az_dir, az_decision)
            el_recovery = self.recovery.recover_el(target_el, curr_el, el_decision)
            err_az = float(az_recovery["err_az"])
            effective_target_el = float(el_recovery["target_el"])
            err_el = effective_target_el - curr_el
            in_tol = abs(err_az) <= POSITION_TOL_DEG and abs(err_el) <= POSITION_TOL_DEG
            if in_tol:
                self.motor_az.stop_smooth()
                self.motor_el.stop_smooth()
                self.recovery.on_cycle_ok()
            else:
                cmd_az = self._speed_from_error(err_az, CONTROL_KP_AZ, max_speed_az)
                cmd_az *= self.recovery.az_speed_scale()
                cmd_el = self._speed_from_error(err_el, CONTROL_KP_EL, max_speed_el)
                if not el_recovery["allow_motion"]:
                    cmd_el = 0.0
                    self.logger.warning("EL limited in realtime loop: %s", el_recovery["reason"])
                self.motor_az.set_target_speed(cmd_az)
                self.motor_el.set_target_speed(cmd_el)
                self._last_cmd_az_dir = 1 if cmd_az > 0 else (-1 if cmd_az < 0 else 0)

            now = time.time()
            if now - last_log_t >= 1.0:
                last_log_t = now
                self.logger.info(
                    "RT state | az=%.2f el=%.2f | target=%.2f/%.2f | err=%.2f/%.2f",
                    curr_az,
                    curr_el,
                    target_az,
                    effective_target_el,
                    err_az,
                    err_el,
                )
            time.sleep(CONTROL_INTERVAL_S)


class RotctlServer:
    """
    Minimal rotctld-compatible TCP server for Gpredict.
    Supports commands:
    - p / \\get_pos
    - P <az> <el> / \\set_pos <az> <el>
    - S / \\stop
    - q
    """

    def __init__(self, controller: RealtimeAzElController, host: str, port: int, logger: logging.Logger):
        self.controller = controller
        self.host = host
        self.port = int(port)
        self.logger = logger
        self._sock = None
        self._run = False

    def serve_forever(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(5)
        self._sock.settimeout(1.0)
        self._run = True
        self.logger.info("rotctl server listening on %s:%d", self.host, self.port)

        while self._run:
            try:
                conn, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True).start()

    def stop(self):
        self._run = False
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass

    def _handle_client(self, conn: socket.socket, addr):
        self.logger.info("rotctl client connected: %s", addr)
        try:
            with conn:
                file_obj = conn.makefile("rwb", buffering=0)
                while self._run:
                    line = file_obj.readline()
                    if not line:
                        break
                    req = line.decode("utf-8", errors="ignore").strip()
                    if not req:
                        continue
                    resp, should_close = self._process_command(req)
                    if resp:
                        file_obj.write(resp.encode("utf-8"))
                    if should_close:
                        break
        except Exception as exc:
            self.logger.warning("rotctl client error (%s): %s", addr, exc)
        finally:
            self.logger.info("rotctl client disconnected: %s", addr)

    def _process_command(self, req: str) -> tuple[str, bool]:
        parts = req.split()
        cmd = parts[0]

        # Hamlib style aliases
        if cmd in ("p", "\\get_pos"):
            az, el = self.controller.get_position()
            return f"{az:.2f}\n{el:.2f}\n", False

        if cmd in ("P", "\\set_pos"):
            if len(parts) < 3:
                return "RPRT -1\n", False
            try:
                az = float(parts[1]) % 360.0
                el = float(parts[2])
            except ValueError:
                return "RPRT -1\n", False
            ok, reason = self.controller.set_target(az, el)
            if not ok:
                self.logger.warning("Reject set_pos az=%.2f el=%.2f: %s", az, el, reason)
                return "RPRT -1\n", False
            return "RPRT 0\n", False

        if cmd in ("S", "\\stop"):
            self.controller.stop_motion()
            return "RPRT 0\n", False

        if cmd in ("q", "\\quit"):
            return "RPRT 0\n", True

        if cmd in ("_", "\\get_info"):
            return "AZ/EL WT901 TB6600 rotctl bridge\n", False

        return "RPRT -11\n", False


def build_default_motors() -> tuple[TB6600Stepper, TB6600Stepper]:
    # Motor 1 (AZ)
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
        circular_axis=True,
    )
    # Motor 2 (EL)
    cfg_m2 = StepperConfig(
        step_pin=23,  # PUL+
        dir_pin=24,   # DIR+
        en_pin=25,    # EN+
        steps_per_rev=200,
        microstep=8,
        max_speed_sps=2200.0,
        accel_sps2=3000.0,
        soft_limit_min_deg=EL_MIN_DEG,
        soft_limit_max_deg=EL_MAX_DEG,
    )
    return TB6600Stepper(cfg_m1), TB6600Stepper(cfg_m2)


def validate_target_move(
    controller: ClosedLoopAzElController,
    logger: logging.Logger,
    target_az_deg: float,
    target_el_deg: float,
) -> bool:
    logger.info("Validation run start.")
    logger.info("Move to target az=%.2f el=%.2f", target_az_deg, target_el_deg)
    ok_target, report_target = controller.drive_to_target(
        target_az_deg,
        target_el_deg,
        tolerance_deg=POSITION_TOL_DEG,
    )
    if not ok_target:
        logger.error("Target validation failed. report=%s", json.dumps(report_target))
        return False

    logger.info(
        "Validation PASS: final position within +/-%.2f deg of target.",
        POSITION_TOL_DEG,
    )
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Closed-loop AZ/EL target move with WT901 feedback.",
    )
    parser.add_argument(
        "--az",
        type=float,
        default=20.0,
        help="Target azimuth in degree (default: 20.0).",
    )
    parser.add_argument(
        "--el",
        type=float,
        default=70.0,
        help="Target elevation in degree (default: 70.0).",
    )
    parser.add_argument(
        "--mode",
        choices=("target", "rotctl"),
        default="target",
        help="target: one-shot move, rotctl: run TCP server for Gpredict",
    )
    parser.add_argument(
        "--rotctl-host",
        type=str,
        default=ROTCTL_DEFAULT_HOST,
        help=f"rotctl bind host (default: {ROTCTL_DEFAULT_HOST})",
    )
    parser.add_argument(
        "--rotctl-port",
        type=int,
        default=ROTCTL_DEFAULT_PORT,
        help=f"rotctl bind port (default: {ROTCTL_DEFAULT_PORT})",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    target_az_deg = float(args.az) % 360.0
    target_el_deg = max(EL_MIN_DEG, min(EL_MAX_DEG, float(args.el)))

    logger = setup_logger()
    logger.info("=== AZ/EL CLOSED-LOOP CONTROL START ===")
    logger.info("Mode | %s", args.mode)
    if args.mode == "target":
        logger.info("Target | az=%.2f el=%.2f", target_az_deg, target_el_deg)
    else:
        logger.info("rotctl mode: hold current position, waiting target from Gpredict (P az el).")

    motor_az = None
    motor_el = None
    wt = None

    try:
        motor_az, motor_el = build_default_motors()

        # Single WT901 reader for both AZ and EL from one sensor packet.
        wt = WT901AxisReader(label="AZEL", addr=0x50, az_offset_deg=0.0, el_offset_deg=0.0)
        wt.open()
        logger.info("WT901 connected.")

        # Calibration routine replicated from fix-compas.py flow.
        wt.reset_zero_point()

        # Sync internal motor positions from absolute sensor to avoid false soft-limit trips.
        boot_data = wt.read_with_retry(attempts=40, delay_s=0.05)
        if boot_data is None:
            raise RuntimeError("Gagal membaca AZ/EL untuk sinkronisasi posisi awal.")
        motor_az.set_position_deg(boot_data["az"])
        motor_el.set_position_deg(boot_data["el"])
        logger.info(
            "Motor position synced from sensor | az=%.3f el=%.3f",
            boot_data["az"],
            boot_data["el"],
        )

        if args.mode == "target":
            controller = ClosedLoopAzElController(
                motor_az=motor_az,
                motor_el=motor_el,
                wt=wt,
                logger=logger,
            )
            ok = validate_target_move(
                controller,
                logger,
                target_az_deg=target_az_deg,
                target_el_deg=target_el_deg,
            )
            if not ok:
                logger.error("Validation FAILED.")
                sys.exit(2)
            logger.info("Validation completed successfully.")
            sys.exit(0)

        rt_controller = RealtimeAzElController(
            motor_az=motor_az,
            motor_el=motor_el,
            wt=wt,
            logger=logger,
        )
        rt_controller.start()
        # In rotctl mode, do not auto-move on startup.
        # Wait for external command from Gpredict ("P <az> <el>").

        server = RotctlServer(
            controller=rt_controller,
            host=args.rotctl_host,
            port=args.rotctl_port,
            logger=logger,
        )
        try:
            server.serve_forever()
        finally:
            server.stop()
            rt_controller.stop()
        sys.exit(0)
    except KeyboardInterrupt:
        logger.warning("Interrupted by user.")
        sys.exit(130)
    except Exception as exc:
        logger.exception("Runtime error: %s", exc)
        sys.exit(1)
    finally:
        logger.info("Shutting down...")
        if motor_az is not None:
            motor_az.close()
        if motor_el is not None:
            motor_el.close()
        if wt is not None:
            wt.close()
        try:
            GPIO.cleanup()
        except Exception:
            pass
        logger.info("GPIO cleanup done.")


if __name__ == "__main__":
    main()
