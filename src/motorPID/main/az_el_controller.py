#!/usr/bin/env python3
# coding: utf-8
"""
Integrated AZ & EL Controller with WT901C485 Feedback and TB6600 Steppers.
Features:
- Dual axis control (AZ 0-360°, EL 0-90°).
- Absolute position reading from WT901C485.
- Hardware and Simulation mode (set SIMULATION_MODE = True/False).
- Keyboard control interface.
- Zero calibration (software offset + hardware reset).
- Data logging for debugging.
- Soft-limit validation.
"""

import sys
import os
import time
import tty
import termios
import threading
import logging
import math
import platform
import argparse
import select
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

# ================= CONFIGURATION =================
# SIMULATION_MODE diatur melalui argumen command line (--sim)
BASE_DIR = Path(__file__).resolve().parent
LOG_FILE = BASE_DIR / "az_el_system.log"
STATE_FILE = BASE_DIR / "az_el_eeprom_state.json"
FAULT_FILE = BASE_DIR / "az_el_fault_state.json"
# =================================================

# Setup logging
logging.basicConfig(
    filename=str(LOG_FILE),
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.ERROR)
logging.getLogger().addHandler(console_handler)

GPIO = None
deviceModel = None
JY901SDataProcessor = None
Protocol485Resolver = None
TILT_THRESHOLD_DEG = 15.0
AZ_OFFSET_DEG = -104.0


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def format_ts() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


def configure_sensor_heading(
    *,
    az_offset_deg: Optional[float] = None,
    tilt_threshold_deg: Optional[float] = None,
):
    """Apply runtime heading config used by AbsoluteSensor."""
    global AZ_OFFSET_DEG, TILT_THRESHOLD_DEG
    if az_offset_deg is not None:
        AZ_OFFSET_DEG = float(az_offset_deg)
    if tilt_threshold_deg is not None:
        TILT_THRESHOLD_DEG = max(0.0, float(tilt_threshold_deg))


class PersistentStateStore:
    """JSON-backed persistence for Raspberry Pi/Python deployments."""

    def __init__(self, state_path: Path = STATE_FILE, fault_path: Path = FAULT_FILE):
        self.state_path = state_path
        self.fault_path = fault_path
        self._lock = threading.Lock()

    def _default_state(self) -> dict:
        return {"updated_at": format_ts(), "axes": {}, "fault": None}

    def _read_state(self) -> dict:
        if not self.state_path.exists():
            return self._default_state()
        try:
            return json.loads(self.state_path.read_text())
        except Exception:
            return self._default_state()

    def _write_state(self, data: dict):
        data["updated_at"] = format_ts()
        self.state_path.write_text(json.dumps(data, indent=2, sort_keys=True))

    def save_axis_position(self, axis_name: str, position_deg: float, reason: str):
        with self._lock:
            data = self._read_state()
            data.setdefault("axes", {})[axis_name] = {
                "position_deg": round(position_deg, 3),
                "reason": reason,
                "saved_at": format_ts(),
            }
            self._write_state(data)

    def load_axis_position(self, axis_name: str) -> Optional[float]:
        with self._lock:
            axis = self._read_state().get("axes", {}).get(axis_name)
            if not axis:
                return None
            try:
                return float(axis["position_deg"])
            except Exception:
                return None

    def save_fault(self, axis_name: str, code: str, message: str):
        payload = {
            "timestamp": format_ts(),
            "axis": axis_name,
            "code": code,
            "message": message,
        }
        with self._lock:
            data = self._read_state()
            data["fault"] = payload
            self._write_state(data)
            self.fault_path.write_text(json.dumps(payload, indent=2, sort_keys=True))

    def clear_fault(self):
        with self._lock:
            data = self._read_state()
            data["fault"] = None
            self._write_state(data)
            self.fault_path.write_text(
                json.dumps({"timestamp": format_ts(), "fault": None}, indent=2, sort_keys=True)
            )


class FaultNotifier:
    def __init__(self, store: PersistentStateStore):
        self.store = store

    def notify(self, axis_name: str, code: str, message: str):
        logging.error("%s %s: %s", axis_name, code, message)
        self.store.save_fault(axis_name, code, message)

    def clear(self):
        self.store.clear_fault()


class GainScheduledPID:
    """Adaptive PID ringan berbasis gain scheduling."""

    def __init__(
        self,
        axis_name: str,
        integral_limit: float = 40.0,
        deadband_deg: float = 0.2,
        derivative_alpha: float = 0.25,
    ):
        self.axis_name = axis_name
        self.integral_limit = integral_limit
        self.deadband_deg = deadband_deg
        self.derivative_alpha = derivative_alpha
        self.integral = 0.0
        self.last_error = 0.0
        self.filtered_derivative = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.filtered_derivative = 0.0
        self.initialized = False

    def gains(self, error_deg: float) -> tuple[float, float, float]:
        mag = abs(error_deg)
        if self.axis_name == "EL":
            if mag > 10.0:
                return 7.5, 0.02, 0.8
            if mag > 3.0:
                return 6.0, 0.04, 1.2
            if mag > 0.3:
                return 4.0, 0.015, 1.8
            return 2.2, 0.0, 2.0

        if mag > 10.0:
            return 8.5, 0.02, 1.0
        if mag > 3.0:
            return 7.0, 0.04, 1.6
        if mag > 0.3:
            return 5.0, 0.02, 2.0
        return 2.6, 0.0, 2.2

    def compute(self, error_deg: float, dt: float) -> float:
        if abs(error_deg) <= self.deadband_deg:
            self.integral = 0.0
            self.last_error = error_deg
            self.initialized = True
            return 0.0

        kp, ki, kd = self.gains(error_deg)
        self.integral = clamp(
            self.integral + error_deg * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        derivative = 0.0 if not self.initialized else (error_deg - self.last_error) / max(dt, 1e-3)
        derivative = clamp(derivative, -120.0, 120.0)
        self.filtered_derivative = (
            self.derivative_alpha * derivative
            + (1.0 - self.derivative_alpha) * self.filtered_derivative
        )
        self.last_error = error_deg
        self.initialized = True
        output = (kp * error_deg) + (ki * self.integral) + (kd * self.filtered_derivative)
        if output * error_deg < 0.0:
            return 0.0
        return output

def setup_hardware(is_sim: bool) -> bool:
    """Setup hardware dependencies, returns fallback simulation mode if hardware not found"""
    global GPIO, deviceModel, JY901SDataProcessor, Protocol485Resolver

    if is_sim:
        return True
        
    try:
        import RPi.GPIO as gpio_module
        GPIO = gpio_module
    except ImportError:
        logging.error("RPi.GPIO not found. Switching to Simulation Mode.")
        return True

    # Setup WITMOTION SDK path
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    SDK_CHS = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "Python-SDK-WT901C485", "chs"))
    if SDK_CHS not in sys.path:
        sys.path.insert(0, SDK_CHS)

    try:
        import lib.device_model as device_model_module
        from lib.data_processor.roles.jy901s_dataProcessor import (
            JY901SDataProcessor as jy901_processor,
        )
        from lib.protocol_resolver.roles.protocol_485_resolver import (
            Protocol485Resolver as protocol_resolver,
        )

        deviceModel = device_model_module
        JY901SDataProcessor = jy901_processor
        Protocol485Resolver = protocol_resolver
    except ImportError as e:
        logging.error(f"WITMOTION SDK not found: {e}. Switching to Simulation Mode.")
        return True
        
    return False

# ================= MOTOR CLASSES =================

@dataclass
class StepperConfig:
    name: str = "Motor"
    step_pin: int = 0
    dir_pin: int = 0
    en_pin: int = 0
    limit_min_pin: Optional[int] = None
    limit_max_pin: Optional[int] = None

    # Motor tuning
    steps_per_rev: int = 200
    microstep: int = 8
    max_speed_sps: float = 2200.0
    accel_sps2: float = 3000.0
    pulse_width_us: int = 8
    home_speed_sps: float = 280.0
    home_timeout_s: float = 20.0
    limit_nc: bool = True
    limit_debounce_ms: int = 50
    persist_interval_s: float = 0.5

    # Soft-limits
    soft_limit_min_deg: float = 0.0
    soft_limit_max_deg: float = 360.0
    wrap_position: bool = False

class MotorController:
    def __init__(
        self,
        cfg: StepperConfig,
        is_sim: bool,
        store: PersistentStateStore,
        notifier: FaultNotifier,
    ):
        self.cfg = cfg
        self.is_sim = is_sim
        self.store = store
        self.notifier = notifier
        self._lock = threading.RLock()
        self._run = True
        self._target_speed_sps = 0.0
        self._current_speed_sps = 0.0
        self._position_full_steps = 0.0
        self._fault_latched = False
        self._fault_msg = ""
        self._last_persist_ts = 0.0
        self._homing = False
        self._home_limit = "min"
        self._home_event = threading.Event()
        self._limit_state = {
            "min": {"stable": False, "last_raw": False, "last_change": time.monotonic()},
            "max": {"stable": False, "last_raw": False, "last_change": time.monotonic()},
        }

        if not self.is_sim:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.cfg.step_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.cfg.dir_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.cfg.en_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.output(self.cfg.en_pin, GPIO.HIGH)
            if self.cfg.limit_min_pin is not None:
                GPIO.setup(self.cfg.limit_min_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            if self.cfg.limit_max_pin is not None:
                GPIO.setup(self.cfg.limit_max_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        restored = self.store.load_axis_position(self.cfg.name)
        if restored is not None:
            self.set_position_deg(restored, persist=False)

        self._thread = threading.Thread(target=self._motion_loop, daemon=True, name=cfg.name)
        self._thread.start()
        logging.info("%s Motor initialized. SIM: %s", cfg.name, is_sim)

    def _axis_deg_to_steps(self, value_deg: float) -> float:
        return (value_deg / 360.0) * float(self.cfg.steps_per_rev)

    def _steps_to_axis_deg(self, steps: float) -> float:
        return (steps / float(self.cfg.steps_per_rev)) * 360.0

    def set_position_deg(self, value_deg: float, persist: bool = True):
        bounded = clamp(value_deg, self.cfg.soft_limit_min_deg, self.cfg.soft_limit_max_deg)
        with self._lock:
            self._position_full_steps = self._axis_deg_to_steps(bounded)
        if persist:
            self.store.save_axis_position(self.cfg.name, bounded, "set_position")

    def set_target_speed(self, speed_sps: float):
        with self._lock:
            if self._fault_latched and not self._homing:
                return
            lim = max(0.0, float(self.cfg.max_speed_sps))
            self._target_speed_sps = max(-lim, min(lim, float(speed_sps)))

    def stop_smooth(self):
        with self._lock:
            self._target_speed_sps = 0.0

    def clear_fault(self):
        with self._lock:
            self._fault_latched = False
            self._fault_msg = ""
            self._target_speed_sps = 0.0
            self._current_speed_sps = 0.0
        self.notifier.clear()

    def get_internal_deg(self) -> float:
        with self._lock:
            return clamp(
                self._steps_to_axis_deg(self._position_full_steps),
                self.cfg.soft_limit_min_deg,
                self.cfg.soft_limit_max_deg,
            )

    def _raw_limit_active(self, which: str) -> bool:
        if self.is_sim:
            pos = self.get_internal_deg()
            eps = self._steps_to_axis_deg(1.0 / float(self.cfg.microstep))
            if which == "min":
                return pos <= self.cfg.soft_limit_min_deg + eps
            return pos >= self.cfg.soft_limit_max_deg - eps

        pin = self.cfg.limit_min_pin if which == "min" else self.cfg.limit_max_pin
        if pin is None:
            return False
        raw = GPIO.input(pin)
        return raw == GPIO.HIGH if self.cfg.limit_nc else raw == GPIO.LOW

    def _debounced_limit_active(self, which: str, now: Optional[float] = None) -> bool:
        if now is None:
            now = time.monotonic()
        raw = self._raw_limit_active(which)
        if self.is_sim:
            return raw
        state = self._limit_state[which]
        if raw != state["last_raw"]:
            state["last_raw"] = raw
            state["last_change"] = now
        elif raw != state["stable"]:
            if now - state["last_change"] >= self.cfg.limit_debounce_ms / 1000.0:
                state["stable"] = raw
        return state["stable"]

    def _limit_name_for_speed(self, speed_sps: float) -> str:
        return "max" if speed_sps > 0 else "min"

    def _limit_position(self, which: str) -> float:
        return self.cfg.soft_limit_min_deg if which == "min" else self.cfg.soft_limit_max_deg

    def _persist_position(self, reason: str):
        self.store.save_axis_position(self.cfg.name, self.get_internal_deg(), reason)
        self._last_persist_ts = time.monotonic()

    def _latch_fault(self, code: str, message: str):
        with self._lock:
            self._fault_latched = True
            self._fault_msg = message
            self._target_speed_sps = 0.0
            self._current_speed_sps = 0.0
        self._persist_position(code)
        self.notifier.notify(self.cfg.name, code, message)

    def home_to_min_limit(self) -> bool:
        self.clear_fault()
        now = time.monotonic()
        if self._debounced_limit_active("min", now):
            self.set_position_deg(self.cfg.soft_limit_min_deg)
            return True

        with self._lock:
            self._homing = True
            self._home_limit = "min"
        self._home_event.clear()
        self.set_target_speed(-abs(self.cfg.home_speed_sps))
        success = self._home_event.wait(timeout=self.cfg.home_timeout_s)
        self.stop_smooth()
        with self._lock:
            self._homing = False
        if not success:
            self._latch_fault("HOME_TIMEOUT", f"Homing timeout on {self.cfg.name}")
            return False
        self.set_position_deg(self.cfg.soft_limit_min_deg)
        self._persist_position("home")
        return True

    def get_status(self) -> dict:
        now = time.monotonic()
        return {
            "target_speed": self._target_speed_sps,
            "current_speed": self._current_speed_sps,
            "pos_deg": self.get_internal_deg(),
            "fault_latched": self._fault_latched,
            "fault_msg": self._fault_msg,
            "limit_min": self._debounced_limit_active("min", now),
            "limit_max": self._debounced_limit_active("max", now),
            "homing": self._homing,
        }

    def _motion_loop(self):
        last_t = time.perf_counter()
        next_pulse_t = last_t

        while self._run:
            now = time.perf_counter()
            dt = now - last_t
            last_t = now
            mono_now = time.monotonic()

            with self._lock:
                if self._fault_latched and not self._homing:
                    time.sleep(0.01)
                    continue
                accel = max(1.0, float(self.cfg.accel_sps2))
                delta = accel * dt
                if self._current_speed_sps < self._target_speed_sps:
                    self._current_speed_sps = min(self._current_speed_sps + delta, self._target_speed_sps)
                elif self._current_speed_sps > self._target_speed_sps:
                    self._current_speed_sps = max(self._current_speed_sps - delta, self._target_speed_sps)
                speed = self._current_speed_sps

            if abs(speed) < 1e-3:
                if mono_now - self._last_persist_ts > self.cfg.persist_interval_s:
                    self._persist_position("idle")
                time.sleep(0.005)
                continue

            if now < next_pulse_t:
                time.sleep(max(0.0001, min(0.005, next_pulse_t - now)))
                continue

            moving_limit = self._limit_name_for_speed(speed)
            if self._debounced_limit_active(moving_limit, mono_now):
                limit_pos = self._limit_position(moving_limit)
                self.set_position_deg(limit_pos)
                if self._homing and moving_limit == self._home_limit:
                    with self._lock:
                        self._target_speed_sps = 0.0
                        self._current_speed_sps = 0.0
                    self._home_event.set()
                    time.sleep(0.01)
                    continue
                self._latch_fault(
                    f"LIMIT_{moving_limit.upper()}",
                    f"{self.cfg.name} limit switch {moving_limit} aktif",
                )
                time.sleep(0.01)
                continue

            step_delta_full = (1.0 / float(self.cfg.microstep)) * (1.0 if speed > 0 else -1.0)
            next_deg = self.get_internal_deg() + self._steps_to_axis_deg(step_delta_full)
            if next_deg < self.cfg.soft_limit_min_deg or next_deg > self.cfg.soft_limit_max_deg:
                self._latch_fault(
                    "SOFT_LIMIT",
                    f"{self.cfg.name} soft limit tercapai pada {next_deg:.2f}°",
                )
                time.sleep(0.01)
                continue

            if not self.is_sim:
                GPIO.output(self.cfg.dir_pin, GPIO.HIGH if speed > 0 else GPIO.LOW)
                GPIO.output(self.cfg.step_pin, GPIO.HIGH)
                time.sleep(self.cfg.pulse_width_us / 1_000_000.0)
                GPIO.output(self.cfg.step_pin, GPIO.LOW)

            with self._lock:
                self._position_full_steps += step_delta_full
                bounded = clamp(
                    self._steps_to_axis_deg(self._position_full_steps),
                    self.cfg.soft_limit_min_deg,
                    self.cfg.soft_limit_max_deg,
                )
                self._position_full_steps = self._axis_deg_to_steps(bounded)

            if mono_now - self._last_persist_ts >= self.cfg.persist_interval_s:
                self._persist_position("motion")

            next_pulse_t = now + (1.0 / abs(speed))

    def close(self):
        self._run = False
        self.stop_smooth()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._persist_position("shutdown")
        if not self.is_sim:
            GPIO.output(self.cfg.en_pin, GPIO.LOW)

# ================= SENSOR CLASS =================

class AbsoluteSensor:
    def __init__(
        self,
        is_sim: bool,
        az_motor: MotorController,
        el_motor: MotorController,
        port_name: Optional[str] = None,
        baud_rate: int = 9600,
    ):
        self.is_sim = is_sim
        self.az_motor = az_motor
        self.el_motor = el_motor
        self.device = None
        self.offset_az = 0.0
        self.offset_el = 0.0
        self.port_name = port_name
        self.baud_rate = int(baud_rate)
        
        if not self.is_sim:
            try:
                try:
                    self.device = deviceModel.DeviceModel("WT901C485", Protocol485Resolver(), JY901SDataProcessor())
                except TypeError:
                    self.device = deviceModel.DeviceModel("WT901C485", Protocol485Resolver(), JY901SDataProcessor(), "EL_0")
                
                self.device.ADDR = 0x50
                if self.port_name:
                    self.device.serialConfig.portName = self.port_name
                elif platform.system().lower() == "linux":
                    self.device.serialConfig.portName = "/dev/ttyUSB0"
                else:
                    self.device.serialConfig.portName = "/dev/tty.usbserial-1330"
                self.device.serialConfig.baud = self.baud_rate
                self.device.openDevice()
                logging.info(f"Sensor connected on {self.device.serialConfig.portName}")
            except Exception as e:
                logging.error(f"Failed to open sensor: {e}")
                self.is_sim = True
                
    def zero_calibration(self):
        """Zero out the current readings using software offset and hardware reset if available."""
        abs_az, abs_el = self._read_raw()
        if abs_az is not None and abs_el is not None:
            self.offset_az = abs_az
            self.offset_el = abs_el
            logging.info(f"Zero calibration done. Offsets AZ:{self.offset_az:.2f}, EL:{self.offset_el:.2f}")
        
        if not self.is_sim and self.device:
            try:
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
                logging.info("Hardware zero-point reset sent.")
            except Exception as e:
                logging.warning(f"Failed hardware reset: {e}")

    def _convert_roll_to_el_deg(self, roll_deg: float) -> float:
        """
        Roll sudah dalam orientasi mekanik final:
        - sekitar 0° saat datar
        - sekitar 90° saat elevasi maksimum
        """
        return clamp(float(roll_deg), 0.0, 90.0)

    def _sensor_get(self, key: str):
        """Baca nilai sensor lintas variasi SDK (getDeviceData/get + variasi key)."""
        variants = [key, key[0].upper() + key[1:]]
        for getter_name in ("getDeviceData", "get"):
            getter = getattr(self.device, getter_name, None)
            if getter is None:
                continue
            for k in variants:
                try:
                    value = getter(k)
                except Exception:
                    continue
                if value is not None:
                    return value
        return None

    def _read_raw(self) -> Tuple[Optional[float], Optional[float]]:
        if self.is_sim:
            return self.az_motor.get_internal_deg(), self.el_motor.get_internal_deg()
            
        try:
            if hasattr(self.device, "readReg"):
                self.device.readReg(0x30, 41)
            roll = self._sensor_get("angleX")
            pitch = self._sensor_get("angleY")
            yaw = self._sensor_get("angleZ")
            magX = self._sensor_get("magX")
            magY = self._sensor_get("magY")
            magZ = self._sensor_get("magZ")

            if roll is None:
                return None, None

            # Koreksi orientasi sesuai tuning lapangan di fix-compas.py.
            roll_deg = 180.0 - float(roll)
            pitch_deg = -(float(pitch) if pitch is not None else 0.0)
            if roll_deg > 180.0:
                roll_deg -= 360.0
            elif roll_deg < -180.0:
                roll_deg += 360.0
            el = self._convert_roll_to_el_deg(roll_deg)

            # Heading YAW (CW 0..360) dengan offset manual.
            yaw_cw = None
            if yaw is not None:
                yaw_norm = float(yaw) % 360.0
                yaw_cw = (360.0 - yaw_norm + AZ_OFFSET_DEG) % 360.0

            azimuth = None
            compass_cw = None
            if magX is not None and magY is not None and magZ is not None:
                try:
                    mx = float(magX)
                    my = float(magY)
                    mz = float(magZ)
                    # Koreksi orientasi magnetometer dari tuning real-data.
                    mx, my = my, mx
                    mx = -mx

                    r_rad = math.radians(roll_deg)
                    p_rad = math.radians(pitch_deg)
                    X_h = mx * math.cos(p_rad) + mz * math.sin(p_rad)
                    Y_h = (
                        mx * math.sin(r_rad) * math.sin(p_rad)
                        + my * math.cos(r_rad)
                        - mz * math.sin(r_rad) * math.cos(p_rad)
                    )
                    heading = math.degrees(math.atan2(Y_h, X_h))
                    if heading < 0.0:
                        heading += 360.0
                    compass = heading
                    compass_cw = (360.0 - compass + AZ_OFFSET_DEG) % 360.0
                except Exception:
                    pass

            # Saat tilt besar, utamakan compass tilt-compensated.
            tilt_large = abs(roll_deg) > TILT_THRESHOLD_DEG or abs(pitch_deg) > TILT_THRESHOLD_DEG
            if tilt_large and compass_cw is not None:
                azimuth = compass_cw
            elif yaw_cw is not None:
                azimuth = yaw_cw
            elif compass_cw is not None:
                azimuth = compass_cw

            # If azimuth still fails, fallback to internal motor position for azimuth.
            if azimuth is None:
                azimuth = self.az_motor.get_internal_deg()
                
            return azimuth, el
        except Exception:
            return None, None

    def get_angles(self) -> Tuple[float, float]:
        raw_az, raw_el = self._read_raw()
        if raw_az is None or raw_el is None:
            return 0.0, 0.0 # Fallback on error
        
        calib_az = (raw_az - self.offset_az + 360) % 360
        calib_el = clamp(raw_el - self.offset_el, 0.0, 90.0)
        return float(calib_az), float(calib_el)
        
    def close(self):
        if not self.is_sim and self.device:
            try:
                if hasattr(self.device, "closeDevice"):
                    self.device.closeDevice()
                else:
                    self.device.close()
            except:
                pass

# ================= KEYBOARD & MAIN =================

class TerminalInput:
    """Context manager untuk input keyboard non-blocking tanpa echo."""

    def __init__(self, stream):
        self.stream = stream
        self.fd = stream.fileno()
        self._old = None

    def __enter__(self):
        self._old = termios.tcgetattr(self.fd)
        new = termios.tcgetattr(self.fd)
        new[3] &= ~(termios.ICANON | termios.ECHO)
        new[6][termios.VMIN] = 0
        new[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, new)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old)

    def read_key(self, timeout: float = 0.05) -> Optional[str]:
        ready, _, _ = select.select([self.stream], [], [], timeout)
        if not ready:
            return None

        ch = self.stream.read(1)
        if ch != "\x1b":
            return ch or None

        parts = [ch]
        for _ in range(7):
            ready, _, _ = select.select([self.stream], [], [], 0.01)
            if not ready:
                break
            nxt = self.stream.read(1)
            if not nxt:
                break
            parts.append(nxt)
        return "".join(parts)


def normalize_key(key: Optional[str]) -> Optional[str]:
    mapping = {
        "\x1b[A": "\x1b[A",
        "\x1bOA": "\x1b[A",
        "\x1b[B": "\x1b[B",
        "\x1bOB": "\x1b[B",
        "\x1b[C": "\x1b[C",
        "\x1bOC": "\x1b[C",
        "\x1b[D": "\x1b[D",
        "\x1bOD": "\x1b[D",
    }
    return mapping.get(key, key)


def format_key_label(key: Optional[str]) -> str:
    key = normalize_key(key)
    mapping = {
        "\x1b[A": "UP",
        "\x1b[B": "DOWN",
        "\x1b[C": "RIGHT",
        "\x1b[D": "LEFT",
        " ": "SPACE",
        None: "-",
    }
    return mapping.get(key, key.upper() if isinstance(key, str) and len(key) == 1 else repr(key))


def axis_hold_label(speed: float) -> str:
    if speed > 0:
        return "POS"
    if speed < 0:
        return "NEG"
    return "STOP"


def bounded_az_error_deg(target_deg: float, current_deg: float) -> float:
    return clamp(target_deg, 0.0, 360.0) - clamp(current_deg, 0.0, 360.0)


def gravity_comp(el_deg: float) -> float:
    return 120.0 * clamp(el_deg / 90.0, 0.0, 1.0)


def fine_settle_speed_limit(abs_error_deg: float, command_speed: float) -> tuple[float, str]:
    if abs_error_deg <= 0.05:
        return min(command_speed, 8.0), "HOLD"
    if abs_error_deg <= 0.15:
        return min(command_speed, 18.0), "FINE2"
    if abs_error_deg <= 0.5:
        return min(command_speed, 40.0), "FINE1"
    if abs_error_deg <= 2.0:
        return min(command_speed, 120.0), "APPROACH"
    return command_speed, "COARSE"


def compute_tracking_command(
    axis_name: str,
    pid: GainScheduledPID,
    error_deg: float,
    dt: float,
    command_speed: float,
    current_el_deg: float = 0.0,
) -> tuple[float, float, float, str, bool]:
    pid_cmd = pid.compute(error_deg, dt)
    speed_limit, phase = fine_settle_speed_limit(abs(error_deg), command_speed)
    feedforward = gravity_comp(current_el_deg) if axis_name == "EL" and pid_cmd > 0.0 else 0.0
    command = clamp(pid_cmd + feedforward, -speed_limit, speed_limit)
    settled = abs(error_deg) <= 0.05
    return command, pid_cmd, feedforward, phase, settled


def tracking_speed_command(
    error_deg: float,
    max_speed: float,
    *,
    tolerance_deg: float = 0.3,
    kp: float = 8.0,
    min_tracking_speed: float = 12.0,
    boost_error_deg: float = 8.0,
    boost_min_speed: float = 80.0,
) -> float:

    if abs(error_deg) <= tolerance_deg:
        return 0.0

    speed = abs(error_deg) * kp
    speed = max(speed, min_tracking_speed)
    if abs(error_deg) >= boost_error_deg:
        speed = max(speed, boost_min_speed)
    speed = min(speed, max_speed)
    return speed if error_deg > 0 else -speed


def restore_positions(az_motor: MotorController, el_motor: MotorController, store: PersistentStateStore):
    az_saved = store.load_axis_position("AZ")
    el_saved = store.load_axis_position("EL")
    if az_saved is not None:
        az_motor.set_position_deg(az_saved, persist=False)
    if el_saved is not None:
        el_motor.set_position_deg(el_saved, persist=False)


def auto_home_axes(az_motor: MotorController, el_motor: MotorController, sensor: AbsoluteSensor) -> bool:
    print("[INFO] Menjalankan homing otomatis AZ -> 0°")
    if not az_motor.home_to_min_limit():
        return False
    print("[INFO] Menjalankan homing otomatis EL -> 0°")
    if not el_motor.home_to_min_limit():
        return False
    sensor.zero_calibration()
    print("[INFO] Homing selesai. AZ/EL di-set ke home 0°.")
    return True


def wait_until(predicate, timeout_s: float, poll_s: float = 0.02) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(poll_s)
    return False


def run_limit_self_test(az_motor: MotorController, el_motor: MotorController) -> bool:
    print("[TEST] Menjalankan functional test overtravel: AZ 370° dan EL 100°")
    az_motor.clear_fault()
    el_motor.clear_fault()
    az_motor.set_position_deg(359.0)
    el_motor.set_position_deg(89.0)
    az_motor.set_target_speed(220.0)
    el_motor.set_target_speed(220.0)

    az_ok = wait_until(lambda: az_motor.get_status()["fault_latched"], 3.0)
    el_ok = wait_until(lambda: el_motor.get_status()["fault_latched"], 3.0)

    az_status = az_motor.get_status()
    el_status = el_motor.get_status()
    print(
        "[TEST] AZ requested 370° -> stop di %.2f° | fault=%s"
        % (az_status["pos_deg"], az_status["fault_msg"])
    )
    print(
        "[TEST] EL requested 100° -> stop di %.2f° | fault=%s"
        % (el_status["pos_deg"], el_status["fault_msg"])
    )
    return az_ok and el_ok and az_status["pos_deg"] >= 359.9 and el_status["pos_deg"] >= 89.9


class AzElTrackerService:
    """Reusable AZ/EL controller service for CLI and Hamlib bridge."""

    def __init__(
        self,
        *,
        sim: bool = False,
        target_az: float = 0.0,
        target_el: float = 0.0,
        auto_home: bool = True,
        sensor_port: Optional[str] = None,
        sensor_baud: int = 9600,
    ):
        self.simulation_mode = setup_hardware(sim)
        self.store = PersistentStateStore()
        self.notifier = FaultNotifier(self.store)
        self.command_speed = 600.0
        self.target_az = clamp(target_az, 0.0, 360.0)
        self.target_el = clamp(target_el, 0.0, 90.0)
        self.tracking_enabled = False
        self.last_key = "-"
        self._lock = threading.RLock()
        self._run = True
        self._last_control_ts = time.monotonic()
        self._last_debug = {
            "az_error": 0.0,
            "el_error": 0.0,
            "az_pid_cmd": 0.0,
            "el_base_cmd": 0.0,
            "el_gravity_ff": 0.0,
        }
        self._last_comm_log_ts = 0.0

        cfg_az = StepperConfig(
            name="AZ",
            step_pin=17,
            dir_pin=27,
            en_pin=22,
            limit_min_pin=5,
            limit_max_pin=6,
            soft_limit_min_deg=0.0,
            soft_limit_max_deg=360.0,
        )
        cfg_el = StepperConfig(
            name="EL",
            step_pin=23,
            dir_pin=24,
            en_pin=25,
            limit_min_pin=12,
            limit_max_pin=16,
            soft_limit_min_deg=0.0,
            soft_limit_max_deg=90.0,
        )
        self.az_motor = MotorController(cfg_az, self.simulation_mode, self.store, self.notifier)
        self.el_motor = MotorController(cfg_el, self.simulation_mode, self.store, self.notifier)
        self.sensor = AbsoluteSensor(
            self.simulation_mode,
            self.az_motor,
            self.el_motor,
            port_name=sensor_port,
            baud_rate=sensor_baud,
        )
        self.az_pid = GainScheduledPID("AZ", integral_limit=40.0, deadband_deg=0.03, derivative_alpha=0.18)
        self.el_pid = GainScheduledPID("EL", integral_limit=35.0, deadband_deg=0.03, derivative_alpha=0.18)

        if auto_home:
            if not auto_home_axes(self.az_motor, self.el_motor, self.sensor):
                raise RuntimeError("Auto-home gagal. Periksa limit switch atau koneksi sensor.")
        else:
            restore_positions(self.az_motor, self.el_motor, self.store)

        self._thread = threading.Thread(target=self._loop, daemon=True, name="az-el-tracker")
        self._thread.start()

    def _loop(self):
        while self._run:
            control_now = time.monotonic()
            control_dt = clamp(control_now - self._last_control_ts, 0.001, 0.2)
            self._last_control_ts = control_now

            az_angle, el_angle = self.sensor.get_angles()
            az_st = self.az_motor.get_status()
            el_st = self.el_motor.get_status()

            if az_st["fault_latched"] or el_st["fault_latched"]:
                with self._lock:
                    self.tracking_enabled = False
                self.az_pid.reset()
                self.el_pid.reset()
                time.sleep(0.02)
                continue

            with self._lock:
                tracking_enabled = self.tracking_enabled
                target_az = self.target_az
                target_el = self.target_el
                command_speed = self.command_speed

            az_error = 0.0
            el_error = 0.0
            az_pid_cmd = 0.0
            el_base_cmd = 0.0
            el_gravity_ff = 0.0
            az_phase = "IDLE"
            el_phase = "IDLE"
            az_settled = False
            el_settled = False

            if tracking_enabled:
                az_error = bounded_az_error_deg(target_az, az_angle)
                el_error = target_el - el_angle
                az_cmd, az_pid_cmd, _, az_phase, az_settled = compute_tracking_command(
                    "AZ",
                    self.az_pid,
                    az_error,
                    control_dt,
                    command_speed,
                )
                el_cmd, el_base_cmd, el_gravity_ff, el_phase, el_settled = compute_tracking_command(
                    "EL",
                    self.el_pid,
                    el_error,
                    control_dt,
                    command_speed,
                    current_el_deg=el_angle,
                )
                self.az_motor.set_target_speed(az_cmd)
                self.el_motor.set_target_speed(el_cmd)
            else:
                self.az_pid.reset()
                self.el_pid.reset()
                self.az_motor.stop_smooth()
                self.el_motor.stop_smooth()

            self._last_debug = {
                "az_error": az_error,
                "el_error": el_error,
                "az_pid_cmd": az_pid_cmd,
                "el_base_cmd": el_base_cmd,
                "el_gravity_ff": el_gravity_ff,
                "az_phase": az_phase,
                "el_phase": el_phase,
                "az_settled": az_settled,
                "el_settled": el_settled,
            }
            time.sleep(0.02)

    def set_target(self, az: float, el: float):
        with self._lock:
            self.target_az = clamp(float(az), 0.0, 360.0)
            self.target_el = clamp(float(el), 0.0, 90.0)
            self.tracking_enabled = True
        logging.info("HAMLIB TARGET -> AZ=%.3f EL=%.3f", self.target_az, self.target_el)

    def get_position(self) -> tuple[float, float]:
        az, el = self.sensor.get_angles()
        return float(az), float(el)

    def get_debug_snapshot(self) -> dict:
        az, el = self.get_position()
        az_st = self.az_motor.get_status()
        el_st = self.el_motor.get_status()
        data = {
            "az": az,
            "el": el,
            "az_fault": az_st["fault_msg"],
            "el_fault": el_st["fault_msg"],
            **self._last_debug,
        }
        return data

    def stop(self):
        with self._lock:
            self.tracking_enabled = False
        self.az_pid.reset()
        self.el_pid.reset()
        self.az_motor.stop_smooth()
        self.el_motor.stop_smooth()
        logging.info("HAMLIB STOP -> hold current position")

    def reset_fault(self):
        self.az_motor.clear_fault()
        self.el_motor.clear_fault()
        self.az_pid.reset()
        self.el_pid.reset()

    def close(self):
        self._run = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.az_motor.close()
        self.el_motor.close()
        self.sensor.close()
        if not self.simulation_mode and GPIO is not None:
            GPIO.cleanup()

def main():
    parser = argparse.ArgumentParser(description="Integrated AZ/EL Controller")
    parser.add_argument("--sim", action="store_true", help="Run in Simulation Mode (without hardware)")
    parser.add_argument("--target-az", type=float, default=250.0, help="Tracking target azimuth in degree")
    parser.add_argument("--target-el", type=float, default=70.0, help="Tracking target elevation in degree")
    parser.add_argument("--track", action="store_true", help="Start in automatic tracking mode")
    parser.add_argument("--no-auto-home", action="store_true", help="Skip automatic homing at startup")
    parser.add_argument("--self-test-limits", action="store_true", help="Run AZ/EL overtravel limit self-test")
    args = parser.parse_args()

    # Determine simulation mode based on arguments and hardware availability
    SIMULATION_MODE = setup_hardware(args.sim)
    store = PersistentStateStore()
    notifier = FaultNotifier(store)

    print(f"Starting AZ/EL Controller... SIMULATION_MODE = {SIMULATION_MODE}")
    logging.info(f"System started. SIMULATION_MODE={SIMULATION_MODE}")
    
    cfg_az = StepperConfig(
        name="AZ",
        step_pin=17,
        dir_pin=27,
        en_pin=22,
        limit_min_pin=5,
        limit_max_pin=6,
        soft_limit_min_deg=0.0,
        soft_limit_max_deg=360.0,
    )
    cfg_el = StepperConfig(
        name="EL",
        step_pin=23,
        dir_pin=24,
        en_pin=25,
        limit_min_pin=12,
        limit_max_pin=16,
        soft_limit_min_deg=0.0,
        soft_limit_max_deg=90.0,
    )
    
    az_motor = MotorController(cfg_az, SIMULATION_MODE, store, notifier)
    el_motor = MotorController(cfg_el, SIMULATION_MODE, store, notifier)
    sensor = AbsoluteSensor(SIMULATION_MODE, az_motor, el_motor)

    if args.no_auto_home:
        restore_positions(az_motor, el_motor, store)
        print("[INFO] Auto-home dilewati. Posisi dipulihkan dari state persisten jika tersedia.")
    else:
        if not auto_home_axes(az_motor, el_motor, sensor):
            print("[ERROR] Homing gagal. Periksa limit switch dan file fault state.")
            az_motor.close()
            el_motor.close()
            sensor.close()
            sys.exit(1)

    if args.self_test_limits:
        if not SIMULATION_MODE:
            print("[ERROR] Self-test limits hanya aman pada mode simulasi.")
            sys.exit(1)
        test_ok = run_limit_self_test(az_motor, el_motor)
        print("[TEST] Result:", "PASS" if test_ok else "FAIL")
        az_motor.close()
        el_motor.close()
        sensor.close()
        sys.exit(0 if test_ok else 1)
    
    command_speed = 600.0
    last_report = 0.0
    last_log = 0.0
    last_key = "-"
    target_az = clamp(args.target_az, 0.0, 360.0)
    target_el = clamp(args.target_el, 0.0, 90.0)
    tracking_enabled = bool(args.track)
    az_pid = GainScheduledPID("AZ", integral_limit=40.0, deadband_deg=0.03, derivative_alpha=0.18)
    el_pid = GainScheduledPID("EL", integral_limit=35.0, deadband_deg=0.03, derivative_alpha=0.18)
    last_control_ts = time.monotonic()
    hold_timeout_s = 0.18
    az_hold_until = 0.0
    el_hold_until = 0.0
    az_hold_speed = 0.0
    el_hold_speed = 0.0
    
    print(
        "\n=== AZ/EL INTEGRATED CONTROLLER ===\n"
        "AZ Motor (hold A/D or Left/Right) : 0-360 deg\n"
        "EL Motor (hold W/S or Up/Down)    : 0-90 deg\n"
        f"Target tracking default           : AZ {target_az:.1f} / EL {target_el:.1f}\n"
        "Space                             : Stop both motors\n"
        "R                                 : Reset fault latch\n"
        "T                                 : Track target AZ/EL\n"
        "Z                                 : Zero Calibration\n"
        "+ / -                             : Speed up / down\n"
        "Q                                 : Quit\n"
        "Limit switch NC + debounce 50 ms aktif.\n"
        "Tracking target memakai adaptive PID + gravity compensation EL.\n"
        "Motor jalan saat key ditekan/ditahan dan stop saat key release.\n"
        "Input keyboard disembunyikan; cek status limit dan LastKey.\n"
    )
    
    try:
        with TerminalInput(sys.stdin) as keyboard:
            while True:
                now = time.time()
                control_now = time.monotonic()
                control_dt = clamp(control_now - last_control_ts, 0.001, 0.2)
                last_control_ts = control_now
                az_angle, el_angle = sensor.get_angles()
                az_st = az_motor.get_status()
                el_st = el_motor.get_status()
                az_error = 0.0
                el_error = 0.0
                az_pid_cmd = 0.0
                el_base_cmd = 0.0
                el_gravity_ff = 0.0
                az_phase = "IDLE"
                el_phase = "IDLE"
                az_settled = False
                el_settled = False

                if az_st["fault_latched"] or el_st["fault_latched"]:
                    tracking_enabled = False
                    az_hold_speed = 0.0
                    el_hold_speed = 0.0
                    az_pid.reset()
                    el_pid.reset()

                if tracking_enabled:
                    az_error = bounded_az_error_deg(target_az, az_angle)
                    el_error = target_el - el_angle
                    az_hold_speed, az_pid_cmd, _, az_phase, az_settled = compute_tracking_command(
                        "AZ",
                        az_pid,
                        az_error,
                        control_dt,
                        command_speed,
                    )
                    el_hold_speed, el_base_cmd, el_gravity_ff, el_phase, el_settled = compute_tracking_command(
                        "EL",
                        el_pid,
                        el_error,
                        control_dt,
                        command_speed,
                        current_el_deg=el_angle,
                    )
                    az_motor.set_target_speed(az_hold_speed)
                    el_motor.set_target_speed(el_hold_speed)
                else:
                    az_pid.reset()
                    el_pid.reset()
                    if now > az_hold_until and az_hold_speed != 0.0:
                        az_hold_speed = 0.0
                        az_motor.stop_smooth()
                    if now > el_hold_until and el_hold_speed != 0.0:
                        el_hold_speed = 0.0
                        el_motor.stop_smooth()

                if now - last_report > 0.1:
                    line_main = (
                        f"AZ:{az_angle:6.1f}° M:{az_st['pos_deg']:6.1f}° V:{az_st['current_speed']:6.0f} {axis_hold_label(az_hold_speed):>4} "
                        f"L:{int(az_st['limit_min'])}/{int(az_st['limit_max'])} | "
                        f"EL:{el_angle:6.1f}° M:{el_st['pos_deg']:6.1f}° V:{el_st['current_speed']:6.0f} {axis_hold_label(el_hold_speed):>4} "
                        f"L:{int(el_st['limit_min'])}/{int(el_st['limit_max'])} | "
                        f"Tgt:{target_az:5.1f}/{target_el:4.1f} {'TRK' if tracking_enabled else 'MAN'} | "
                        f"Cmd:{command_speed:4.0f} | LastKey:{last_key:<6}"
                    )
                    line_debug = (
                        f"DBG AZ_ERR:{az_error:7.3f} EL_ERR:{el_error:7.3f} "
                        f"AZ_PID:{az_pid_cmd:8.3f} EL_PID:{el_base_cmd:8.3f} EL_FF:{el_gravity_ff:8.3f} "
                        f"AZ_PH:{az_phase:<8} EL_PH:{el_phase:<8} ST:{int(az_settled)}/{int(el_settled)}"
                    )
                    out = (
                        "\r\033[2K"
                        f"{line_main}\n"
                        "\033[2K"
                        f"{line_debug}"
                        "\033[F"
                    )
                    sys.stdout.write(out)
                    sys.stdout.flush()

                    if now - last_log > 1.0:
                        logging.info(
                            "POS -> AZ: %.2f, EL: %.2f, KEY: %s, TARGET: %.2f/%.2f, TRACK: %s, "
                            "AZ_ERR: %.3f, EL_ERR: %.3f, AZ_PID: %.3f, EL_PID: %.3f, EL_FF: %.3f, "
                            "AZ_PHASE: %s, EL_PHASE: %s, AZ_SETTLED: %s, EL_SETTLED: %s, "
                            "FAULT_AZ: %s, FAULT_EL: %s",
                            az_angle,
                            el_angle,
                            last_key,
                            target_az,
                            target_el,
                            tracking_enabled,
                            az_error,
                            el_error,
                            az_pid_cmd,
                            el_base_cmd,
                            el_gravity_ff,
                            az_phase,
                            el_phase,
                            az_settled,
                            el_settled,
                            az_st["fault_msg"],
                            el_st["fault_msg"],
                        )
                        last_log = now

                    last_report = now

                key = keyboard.read_key(timeout=0.02)
                if not key:
                    continue

                key = normalize_key(key)
                last_key = format_key_label(key)

                if key in ("\x1b[C", "d", "D"):
                    tracking_enabled = False
                    az_pid.reset()
                    el_pid.reset()
                    az_hold_speed = command_speed
                    az_hold_until = now + hold_timeout_s
                    az_motor.set_target_speed(az_hold_speed)
                elif key in ("\x1b[D", "a", "A"):
                    tracking_enabled = False
                    az_pid.reset()
                    el_pid.reset()
                    az_hold_speed = -command_speed
                    az_hold_until = now + hold_timeout_s
                    az_motor.set_target_speed(az_hold_speed)
                elif key in ("\x1b[A", "w", "W"):
                    tracking_enabled = False
                    az_pid.reset()
                    el_pid.reset()
                    el_hold_speed = command_speed
                    el_hold_until = now + hold_timeout_s
                    el_motor.set_target_speed(el_hold_speed)
                elif key in ("\x1b[B", "s", "S"):
                    tracking_enabled = False
                    az_pid.reset()
                    el_pid.reset()
                    el_hold_speed = -command_speed
                    el_hold_until = now + hold_timeout_s
                    el_motor.set_target_speed(el_hold_speed)
                elif key == " ":
                    tracking_enabled = False
                    az_pid.reset()
                    el_pid.reset()
                    az_hold_speed = 0.0
                    el_hold_speed = 0.0
                    az_hold_until = 0.0
                    el_hold_until = 0.0
                    az_motor.stop_smooth()
                    el_motor.stop_smooth()
                elif key in ("r", "R"):
                    az_pid.reset()
                    el_pid.reset()
                    az_motor.clear_fault()
                    el_motor.clear_fault()
                    sys.stdout.write("\n[INFO] Fault latch reset.\n")
                    sys.stdout.flush()
                elif key in ("t", "T"):
                    az_pid.reset()
                    el_pid.reset()
                    tracking_enabled = True
                    sys.stdout.write(
                        f"\n[INFO] Tracking enabled -> target AZ {target_az:.1f}, EL {target_el:.1f}\n"
                    )
                    sys.stdout.flush()
                elif key in ("z", "Z"):
                    sensor.zero_calibration()
                    sys.stdout.write("\n[INFO] Zero Calibration Applied.\n")
                    sys.stdout.flush()
                elif key == "+":
                    command_speed = min(2200.0, command_speed + 100.0)
                elif key == "-":
                    command_speed = max(100.0, command_speed - 100.0)
                elif key in ("q", "Q"):
                    break
                
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        logging.error(f"Runtime error: {exc}", exc_info=True)
    finally:
        print("\nShutting down...")
        az_motor.close()
        el_motor.close()
        sensor.close()
        if not SIMULATION_MODE:
            try:
                import RPi.GPIO as GPIO
                GPIO.cleanup()
            except ImportError:
                pass
        print("Done.")

if __name__ == "__main__":
    main()
