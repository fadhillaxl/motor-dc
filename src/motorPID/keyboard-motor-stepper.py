#!/usr/bin/env python3
from __future__ import annotations
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
import csv
import datetime
import platform
import logging
import math
import urllib.parse
import urllib.request
from collections import deque
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

FALLBACK_TLE = [
    {
        "name": "ISS (ZARYA)",
        "line1": "1 25544U 98067A   25067.51892361  .00015867  00000+0  28631-3 0  9991",
        "line2": "2 25544  51.6393  20.9756 0004985  44.5630  78.1897 15.50033015442444",
        "id": 25544,
    },
    {
        "name": "NOAA 15",
        "line1": "1 25338U 98030A   25067.47435356  .00000099  00000+0  74015-4 0  9995",
        "line2": "2 25338  98.7545  92.6464 0010719 338.6616  21.3905 14.26090814391464",
        "id": 25338,
    },
]

# =====================================================
# PYTHON PATH -> folder chs lokal project (samakan pola read_wt901.py)
# =====================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.abspath(os.path.join(BASE_DIR, "..", "Python-SDK-WT901C485", "chs"))
if SDK_CHS not in sys.path:
    sys.path.insert(0, SDK_CHS)

try:
    import lib.device_model as wt901_deviceModel
    from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
    from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver
    WT901_SDK_AVAILABLE = True
    WT901_SDK_IMPORT_ERROR = ""
except Exception as exc:
    wt901_deviceModel = None
    JY901SDataProcessor = None
    Protocol485Resolver = None
    WT901_SDK_AVAILABLE = False
    WT901_SDK_IMPORT_ERROR = str(exc)


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
    el_offset_deg: float = 0.0     # Kalibrasi elevasi terhadap referensi mekanik
    axis_gear_ratio: float = 1.0   # >1 jika ada reduksi gear (motor:output)
    az_ls_deg: float = 0.0         # 0=full range, non-zero=blok jika lintas titik AZ LS
    az_ls_block_crossing: bool = False  # Default: AZ LS sebagai referensi, bukan hard-stop


@dataclass
class WT901Config:
    enabled: bool = False
    interface: str = "uart"  # uart|i2c
    port_name: str = ""
    baud: int = 9600
    address: int = 0x50
    sample_rate_hz: float = 50.0
    timeout_s: float = 0.08
    retry_limit: int = 5
    reconnect_delay_s: float = 0.5
    buffer_size: int = 512
    moving_avg_window: int = 5
    outlier_deg_threshold: float = 35.0
    declination_deg: float = 0.0
    log_level: str = "INFO"


@dataclass
class WT901Sample:
    timestamp: float
    az_deg: float
    el_deg: float
    compass_deg: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    accel_g: tuple[float, float, float]
    gyro_dps: tuple[float, float, float]
    mag: tuple[float, float, float]
    temperature_c: float | None
    source: str = "sdk"


@dataclass
class ImuAzElHoldConfig:
    control_rate_hz: float = 50.0
    log_rate_hz: float = 10.0
    dropout_timeout_s: float = 0.25
    az_kp: float = 65.0
    az_ki: float = 0.18
    az_kd: float = 12.0
    el_kp: float = 55.0
    el_ki: float = 0.16
    el_kd: float = 10.0
    nudge_step_deg: float = 0.5
    log_path: str = ""


class WT901Reader:
    """WT901 IMU service for hardware mode with retry, filtering, and buffering.
    
    Fitur dan Parameter Baru (berdasarkan absolutEL.py):
    1. Azimuth (AZ): Dihitung dari magnetometer (Compass) menggunakan Tilt Compensation.
       - Satuan: Derajat (°)
       - Range Nilai: 0° hingga 360°
    2. Elevasi Absolut (EL): Dibaca dari sensor Roll setelah kalibrasi gravitasi bumi.
       - Satuan: Derajat (°)
       - Range Nilai: -90° hingga +90° (-90 tegak ke bawah, +90 tegak ke atas, 0 datar)
    
    Prosedur Kalibrasi:
    1. Zero-point (Gravitasi): Dipanggil otomatis saat terkoneksi (`reset_zero_point`).
       Pastikan alat diletakkan pada orientasi default/sejajar sebelum dihubungkan 
       jika ingin mengatur elevasi 0° terhadap posisi saat ini.
    2. Kompas (Medan Magnet): Panggil `begin_field_calibration()`, lalu putar perangkat 
        360° perlahan di sumbu X, Y, dan Z. Setelah selesai, panggil `end_field_calibration()`.
        (Di CLI: `imu magcal start` dan `imu magcal end`)
    """

    def __init__(self, cfg: WT901Config):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._run = False
        self._thread = None
        self._device = None
        self._connected = False
        self._last_error = ""
        self._retry_count = 0
        self._last_ok_t = None
        self._latest: WT901Sample | None = None
        self._buffer = deque(maxlen=max(64, int(cfg.buffer_size)))
        self._yaw_hist = deque(maxlen=max(3, int(cfg.moving_avg_window)))
        self._pitch_hist = deque(maxlen=max(3, int(cfg.moving_avg_window)))
        self._compass_hist = deque(maxlen=max(3, int(cfg.moving_avg_window)))
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._hard_iron = [0.0, 0.0, 0.0]
        self._soft_iron = [1.0, 1.0, 1.0]
        self._logger = logging.getLogger("motorPID.wt901")
        self._logger.setLevel(getattr(logging, str(cfg.log_level).upper(), logging.INFO))

    @staticmethod
    def _default_port() -> str:
        return "/dev/ttyUSB0" if platform.system().lower() == "linux" else "/dev/tty.usbserial-1330"

    @staticmethod
    def _wrap_360(v: float) -> float:
        x = float(v) % 360.0
        return x if x >= 0.0 else x + 360.0

    @staticmethod
    def _to_deg_rad(v_deg: float) -> tuple[float, float]:
        return float(v_deg), math.radians(float(v_deg))

    @staticmethod
    def _moving_average(hist: deque) -> float:
        if not hist:
            return 0.0
        return float(sum(hist)) / float(len(hist))

    @staticmethod
    def _decode_int16_le(lo: int, hi: int) -> int:
        raw = ((int(hi) & 0xFF) << 8) | (int(lo) & 0xFF)
        return raw - 65536 if raw >= 32768 else raw

    @classmethod
    def parse_uart_frame(cls, frame: bytes) -> dict | None:
        """
        Parse 11-byte WT901 UART frame (0x55 + type + 8 data bytes + checksum).
        Includes checksum and little-endian int16 conversion.
        """
        if len(frame) != 11 or frame[0] != 0x55:
            return None
        checksum = sum(frame[:10]) & 0xFF
        if checksum != frame[10]:
            return None
        frame_type = frame[1]
        vals = [
            cls._decode_int16_le(frame[2], frame[3]),
            cls._decode_int16_le(frame[4], frame[5]),
            cls._decode_int16_le(frame[6], frame[7]),
            cls._decode_int16_le(frame[8], frame[9]),
        ]
        return {"type": frame_type, "values_i16": vals, "checksum_ok": True}

    def _new_device(self):
        if not WT901_SDK_AVAILABLE:
            raise RuntimeError(f"WT901 SDK tidak tersedia: {WT901_SDK_IMPORT_ERROR}")
        dev = wt901_deviceModel.DeviceModel(
            "WT901",
            Protocol485Resolver(),
            JY901SDataProcessor(),
            "51_0",
        )
        dev.ADDR = int(self.cfg.address) & 0xFF
        port_name = self.cfg.port_name.strip() if self.cfg.port_name else self._default_port()
        if self.cfg.interface.lower() == "uart":
            dev.serialConfig.portName = port_name
            dev.serialConfig.baud = int(self.cfg.baud)
            if hasattr(dev.serialConfig, "timeout"):
                dev.serialConfig.timeout = float(self.cfg.timeout_s)
        elif self.cfg.interface.lower() == "i2c":
            # SDK ini berbasis Protocol485Resolver; i2c disediakan sebagai mode degrade.
            # Jika object i2cConfig tersedia, set param dasar; jika tidak, fallback error.
            if hasattr(dev, "i2cConfig"):
                if hasattr(dev.i2cConfig, "address"):
                    dev.i2cConfig.address = int(self.cfg.address) & 0x7F
                if hasattr(dev.i2cConfig, "bus"):
                    dev.i2cConfig.bus = 1
            else:
                raise RuntimeError("I2C mode belum didukung oleh SDK WT901C485 pada project ini.")
        else:
            raise ValueError(f"WT901 interface unsupported: {self.cfg.interface}")
        return dev

    def initialize(self):
        if not self.cfg.enabled:
            self._logger.info("WT901 disabled by config.")
            return
        self._connect_with_retry()
        self._run = True
        period = 1.0 / max(50.0, float(self.cfg.sample_rate_hz))
        self._thread = threading.Thread(target=self._acquire_loop, args=(period,), daemon=True)
        self._thread.start()

    def _connect_with_retry(self):
        for attempt in range(1, max(1, int(self.cfg.retry_limit)) + 1):
            try:
                dev = self._new_device()
                dev.openDevice()
                vals = dev.readReg(0x02, 3)
                if len(vals) <= 0:
                    raise RuntimeError("No response on readReg(0x02,3)")
                
                self._device = dev
                self._connected = True
                self._retry_count = 0
                self._last_error = ""
                
                # (3) Kalibrasi Sensor: Reset zero-point saat terkoneksi agar EL absolut terhadap gravitasi
                self.reset_zero_point()
                
                self._logger.info("WT901 connected on attempt %d", attempt)
                return
            except Exception as exc:
                self._last_error = f"connect fail attempt {attempt}: {exc}"
                self._retry_count += 1
                self._connected = False
                self._logger.error("WT901 connect error: %s", self._last_error)
                time.sleep(float(self.cfg.reconnect_delay_s))
        raise RuntimeError(f"WT901 gagal konek setelah {self.cfg.retry_limit} percobaan.")

    def reset_zero_point(self):
        """
        Menghapus zero-point yang tersimpan di sensor (Elevasi Absolut).
        Ini memastikan roll/EL selalu mengacu pada gravitasi, bukan posisi saat dinyalakan.
        """
        if self._device is None:
            return
        self._logger.info("Mereset zero-point ke default (sudut absolut)...")
        try:
            REG_KEY = 0x69
            if hasattr(self._device, "write_register"):
                self._device.write_register(self._device.ADDR, REG_KEY, 0xB588)
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
            self._logger.info("Zero-point berhasil direset. Sensor menggunakan gravitasi sebagai referensi.")
        except Exception as e:
            self._logger.warning("Gagal reset zero-point: %s", e)

    def _safe_get(self, key: str) -> float | None:
        try:
            val = self._device.getDeviceData(key)
            return float(val) if val is not None else None
        except Exception:
            return None

    def _validate_register_payload(self, regs):
        if not isinstance(regs, (list, tuple, bytes, bytearray)):
            raise ValueError("WT901 register payload type invalid")
        if len(regs) <= 0:
            raise TimeoutError("WT901 register payload kosong")
        ints = list(regs)
        for v in ints:
            if not isinstance(v, int):
                raise ValueError("WT901 register payload non-int value")
            if v < -32768 or v > 65535:
                raise ValueError(f"WT901 register payload out of range: {v}")

        # Jika payload terlihat seperti stream frame UART 11-byte, validasi checksum per frame.
        if len(ints) % 11 == 0:
            frame_like = True
            for i in range(0, len(ints), 11):
                b0 = ints[i] & 0xFF
                if b0 != 0x55:
                    frame_like = False
                    break
            if frame_like:
                for i in range(0, len(ints), 11):
                    frame = bytes((ints[i + j] & 0xFF) for j in range(11))
                    parsed = self.parse_uart_frame(frame)
                    if parsed is None:
                        raise ValueError("WT901 UART checksum/frame invalid")

    def _read_sample_from_sdk(self) -> WT901Sample:
        if self._device is None:
            raise RuntimeError("WT901 device belum diinisialisasi")
        regs = self._device.readReg(0x30, 41)
        self._validate_register_payload(regs)

        roll = self._safe_get("angleX")
        pitch = self._safe_get("angleY")
        yaw = self._safe_get("angleZ")
        ax = self._safe_get("accX")
        ay = self._safe_get("accY")
        az_acc = self._safe_get("accZ")
        gx = self._safe_get("gyroX")
        gy = self._safe_get("gyroY")
        gz = self._safe_get("gyroZ")
        mx = self._safe_get("magX")
        my = self._safe_get("magY")
        mz = self._safe_get("magZ")
        temp_c = self._safe_get("temperature")

        # Validasi (4) penanganan error untuk data sensor yang tidak valid
        if any(v is None for v in (roll, pitch, yaw, ax, ay, az_acc, mx, my, mz)):
            raise ValueError("Data sudut/sensor tidak lengkap (None)")

        gx -= self._gyro_bias[0]
        gy -= self._gyro_bias[1]
        gz -= self._gyro_bias[2]
        mx = (mx - self._hard_iron[0]) * self._soft_iron[0]
        my = (my - self._hard_iron[1]) * self._soft_iron[1]
        mz = (mz - self._hard_iron[2]) * self._soft_iron[2]

        # (1) Integrasi pembacaan magnetometer untuk azimuth dengan kompensasi kemiringan (tilt compensation)
        try:
            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)

            # Transformasi NED/NWD asumsi standar
            X_h = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
            Y_h = mx * math.sin(roll_rad) * math.sin(pitch_rad) + my * math.cos(roll_rad) - mz * math.sin(roll_rad) * math.cos(pitch_rad)

            azimuth_rad = math.atan2(-Y_h, X_h)
            azimuth = math.degrees(azimuth_rad)
            if azimuth < 0:
                azimuth += 360.0
            
            compass = self._wrap_360(azimuth + self.cfg.declination_deg)
        except Exception:
            compass = self._wrap_360(yaw + self.cfg.declination_deg)

        # Azimuth utama sekarang menggunakan compass (magnetometer)
        az = compass
        
        # (2) Kalkulasi elevasi absolut menggunakan accelerometer. 
        # (Meskipun WT901 sudah memberikan pitch yang difusi gravitasi setelah reset_zero_point,
        # kita tambahkan verifikasi accelerometer murni sebagai cadangan / log).
        # el_acc = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az_acc*az_acc)))
        
        el = max(-90.0, min(90.0, roll))
        sample = WT901Sample(
            timestamp=time.time(),
            az_deg=az,
            el_deg=el,
            compass_deg=compass,
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=yaw,
            accel_g=(ax, ay, az_acc),
            gyro_dps=(gx, gy, gz),
            mag=(mx, my, mz),
            temperature_c=temp_c,
        )
        self._validate_ranges(sample)
        return self._filter_sample(sample)

    def _validate_ranges(self, smp: WT901Sample):
        if not (-180.0 <= smp.roll_deg <= 180.0):
            raise ValueError(f"roll out of range: {smp.roll_deg}")
        if not (-180.0 <= smp.pitch_deg <= 180.0):
            raise ValueError(f"pitch out of range: {smp.pitch_deg}")
        if not (-360.0 <= smp.yaw_deg <= 360.0):
            raise ValueError(f"yaw out of range: {smp.yaw_deg}")
        for g in smp.gyro_dps:
            if not (-5000.0 <= g <= 5000.0):
                raise ValueError(f"gyro out of range: {g}")

    def _filter_sample(self, smp: WT901Sample) -> WT901Sample:
        self._yaw_hist.append(self._wrap_360(smp.az_deg))
        self._pitch_hist.append(float(smp.el_deg))
        self._compass_hist.append(self._wrap_360(smp.compass_deg))

        az_avg = self._moving_average(self._yaw_hist)
        el_avg = self._moving_average(self._pitch_hist)
        comp_avg = self._moving_average(self._compass_hist)

        if self._latest is not None:
            delta = abs(az_avg - self._latest.az_deg)
            if delta > 180.0:
                delta = 360.0 - delta
            if delta > float(self.cfg.outlier_deg_threshold):
                az_avg = self._latest.az_deg
                comp_avg = self._latest.compass_deg

        smp.az_deg = self._wrap_360(az_avg)
        smp.el_deg = max(-90.0, min(90.0, el_avg))
        smp.compass_deg = self._wrap_360(comp_avg)
        return smp

    def _acquire_loop(self, period_s: float):
        while self._run:
            t0 = time.time()
            try:
                smp = self._read_sample_from_sdk()
                with self._lock:
                    self._latest = smp
                    self._buffer.append(smp)
                with self._status_lock:
                    self._last_ok_t = smp.timestamp
                    self._connected = True
                    self._retry_count = 0
                    self._last_error = ""
            except Exception as exc:
                with self._status_lock:
                    self._last_error = str(exc)
                    self._retry_count += 1
                self._logger.error("WT901 read error: %s", exc)
                if self._retry_count >= max(1, int(self.cfg.retry_limit)):
                    self._reset_connection()
            dt = time.time() - t0
            time.sleep(max(0.0, period_s - dt))

    def _reset_connection(self):
        self._logger.warning("WT901 reset connection (retry=%d)", self._retry_count)
        try:
            if self._device is not None:
                self._device.closeDevice()
        except Exception:
            pass
        self._device = None
        self._connected = False
        time.sleep(float(self.cfg.reconnect_delay_s))
        try:
            self._connect_with_retry()
        except Exception as exc:
            self._last_error = f"reconnect failed: {exc}"
            self._logger.error("WT901 reconnect failed: %s", exc)

    def get_latest(self) -> WT901Sample | None:
        with self._lock:
            return self._latest

    def get_latest_dict(self) -> dict:
        smp = self.get_latest()
        if smp is None:
            return {}
        az_deg, az_rad = self._to_deg_rad(smp.az_deg)
        el_deg, el_rad = self._to_deg_rad(smp.el_deg)
        cp_deg, cp_rad = self._to_deg_rad(smp.compass_deg)
        return {
            "timestamp": smp.timestamp,
            "azimuth_deg": az_deg,
            "azimuth_rad": az_rad,
            "elevation_deg": el_deg,
            "elevation_rad": el_rad,
            "compass_deg": cp_deg,
            "compass_rad": cp_rad,
            "roll_deg": smp.roll_deg,
            "pitch_deg": smp.pitch_deg,
            "yaw_deg": smp.yaw_deg,
            "accel_g": smp.accel_g,
            "gyro_dps": smp.gyro_dps,
            "mag": smp.mag,
            "temperature_c": smp.temperature_c,
            "source": smp.source,
        }

    def get_buffer_snapshot(self) -> list[dict]:
        with self._lock:
            data = list(self._buffer)
        out = []
        for smp in data:
            out.append(
                {
                    "timestamp": smp.timestamp,
                    "azimuth_deg": smp.az_deg,
                    "elevation_deg": smp.el_deg,
                    "compass_deg": smp.compass_deg,
                }
            )
        return out

    def get_status(self) -> dict:
        with self._status_lock:
            return {
                "enabled": self.cfg.enabled,
                "connected": self._connected,
                "interface": self.cfg.interface,
                "port": self.cfg.port_name or self._default_port(),
                "baud": self.cfg.baud,
                "sample_rate_hz": max(50.0, float(self.cfg.sample_rate_hz)),
                "last_ok_timestamp": self._last_ok_t,
                "retry_count": self._retry_count,
                "last_error": self._last_error,
                "buffer_len": len(self._buffer),
            }

    def set_declination(self, decl_deg: float):
        self.cfg.declination_deg = float(decl_deg)

    def set_hard_iron_offset(self, mx: float, my: float, mz: float):
        self._hard_iron = [float(mx), float(my), float(mz)]

    def set_soft_iron_scale(self, sx: float, sy: float, sz: float):
        self._soft_iron = [max(1e-6, float(sx)), max(1e-6, float(sy)), max(1e-6, float(sz))]

    def calibrate_gyro_bias(self, duration_s: float = 2.0):
        duration_s = max(0.2, float(duration_s))
        t_end = time.time() + duration_s
        acc = [0.0, 0.0, 0.0]
        n = 0
        while time.time() < t_end:
            smp = self.get_latest()
            if smp is not None:
                acc[0] += smp.gyro_dps[0]
                acc[1] += smp.gyro_dps[1]
                acc[2] += smp.gyro_dps[2]
                n += 1
            time.sleep(0.01)
        if n > 0:
            self._gyro_bias = [acc[0] / n, acc[1] / n, acc[2] / n]
            self._logger.info("WT901 gyro bias calibrated: %s", self._gyro_bias)

    def begin_field_calibration(self):
        if self._device is None:
            raise RuntimeError("WT901 belum connected")
        
        if hasattr(self._device, "unlock"):
            self._device.unlock()
            time.sleep(0.1)

        if hasattr(self._device, "BeginFiledCalibration"):
            self._device.BeginFiledCalibration()
        else:
            self._device.writeReg(0x01, 0x07)
        self._logger.info("WT901 field calibration started. Putar sensor 360 derajat di semua sumbu.")

    def end_field_calibration(self):
        if self._device is None:
            raise RuntimeError("WT901 belum connected")

        if hasattr(self._device, "unlock"):
            self._device.unlock()
            time.sleep(0.1)

        if hasattr(self._device, "EndFiledCalibration"):
            self._device.EndFiledCalibration()
        else:
            self._device.writeReg(0x01, 0x00)
            if hasattr(self._device, "save"):
                time.sleep(0.1)
                self._device.save()
        self._logger.info("WT901 field calibration ended and saved.")

    def close(self):
        self._run = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        try:
            if self._device is not None:
                self._device.closeDevice()
        except Exception:
            pass
        self._connected = False


@dataclass
class ELImuFusionConfig:
    update_rate_hz: float = 100.0
    complementary_alpha: float = 0.98
    imu_pitch_weight: float = 0.18
    stale_timeout_s: float = 0.2
    still_gyro_dps: float = 0.3
    still_acc_g_tol: float = 0.08
    el_min_deg: float = 0.0
    el_max_deg: float = 90.0
    absolute_gravity_mode: bool = True
    debug_log: bool = False


class ELImuFusionTracker:
    """IMU-only EL estimator (gyro integration + accel correction + drift compensation)."""

    def __init__(self, cfg: ELImuFusionConfig, imu_reader: WT901Reader):
        self.cfg = cfg
        self.imu_reader = imu_reader
        self._lock = threading.Lock()
        self._logger = logging.getLogger("motorPID.el_imu")
        self._run = False
        self._thread = None
        self._last_imu_ts = None
        self._last_proc_t = None
        self._el_est_deg = 0.0
        self._gyro_bias_y = 0.0
        self._zero_offset_deg = 0.0
        self._last_error = ""
        self._samples = 0
        self._last_rate_t = time.time()
        self._rate_hz = 0.0
        self._initialized = False
        self._debug_snapshot = {}

    def start(self):
        self._run = True
        period = 1.0 / max(100.0, float(self.cfg.update_rate_hz))
        self._thread = threading.Thread(target=self._loop, args=(period,), daemon=True)
        self._thread.start()

    @staticmethod
    def _accel_to_pitch_deg(ax: float, ay: float, az: float) -> float:
        # Pitch estimate from gravity vector.
        denom = math.sqrt((ay * ay) + (az * az))
        if denom < 1e-9:
            return 0.0
        return math.degrees(math.atan2(ax, denom))

    def _loop(self, period_s: float):
        while self._run:
            t0 = time.time()
            try:
                imu = self.imu_reader.get_latest_dict()
                if not imu:
                    time.sleep(period_s)
                    continue
                imu_ts = float(imu["timestamp"])
                if self._last_imu_ts is not None and imu_ts <= self._last_imu_ts:
                    time.sleep(period_s)
                    continue
                self._last_imu_ts = imu_ts

                ax, ay, az = imu.get("accel_g", (0.0, 0.0, 1.0))
                gx, gy, gz = imu.get("gyro_dps", (0.0, 0.0, 0.0))
                imu_pitch = float(imu.get("elevation_deg", 0.0))
                now = time.time()
                dt = 0.01 if self._last_proc_t is None else max(0.001, now - self._last_proc_t)
                self._last_proc_t = now

                acc_pitch = self._accel_to_pitch_deg(float(ax), float(ay), float(az))
                with self._lock:
                    if not self._initialized:
                        # Bootstrap from IMU fused pitch to avoid startup lock at near-zero accel tilt.
                        self._el_est_deg = imu_pitch
                        self._initialized = True
                    gyro_term = self._el_est_deg + (float(gy) - self._gyro_bias_y) * dt
                    alpha = max(0.7, min(0.999, float(self.cfg.complementary_alpha)))
                    acc_fused = alpha * gyro_term + (1.0 - alpha) * acc_pitch
                    w_imu = max(0.02, min(0.6, float(self.cfg.imu_pitch_weight)))
                    innovation = imu_pitch - acc_fused
                    # Increase correction when estimator diverges far from IMU fused pitch.
                    w_imu_dyn = min(0.65, w_imu + min(0.35, abs(innovation) / 180.0))
                    fused = (1.0 - w_imu_dyn) * acc_fused + w_imu_dyn * imu_pitch
                    self._el_est_deg = fused

                    acc_norm = math.sqrt((float(ax) * float(ax)) + (float(ay) * float(ay)) + (float(az) * float(az)))
                    still = (abs(float(gy)) < float(self.cfg.still_gyro_dps)) and (
                        abs(acc_norm - 1.0) < float(self.cfg.still_acc_g_tol)
                    )
                    if still:
                        # Slow bias learning while stationary.
                        self._gyro_bias_y = (0.995 * self._gyro_bias_y) + (0.005 * float(gy))

                    self._samples += 1
                    if now - self._last_rate_t >= 1.0:
                        self._rate_hz = self._samples / max(1e-6, (now - self._last_rate_t))
                        self._samples = 0
                        self._last_rate_t = now
                    self._debug_snapshot = {
                        "acc_pitch_deg": acc_pitch,
                        "imu_pitch_deg": imu_pitch,
                        "gyro_y_dps": float(gy),
                        "gyro_term_deg": gyro_term,
                        "innovation_deg": innovation,
                        "imu_weight_dyn": w_imu_dyn,
                        "acc_norm_g": acc_norm,
                    }
                    self._last_error = ""
                    if self.cfg.debug_log and (self._samples % 20 == 0):
                        self._logger.info(
                            "EL fusion dbg est=%.2f imu=%.2f acc=%.2f gy=%.3f bias=%.3f w=%.3f",
                            self._el_est_deg,
                            imu_pitch,
                            acc_pitch,
                            float(gy),
                            self._gyro_bias_y,
                            w_imu_dyn,
                        )
            except Exception as exc:
                with self._lock:
                    self._last_error = str(exc)
                self._logger.error("EL IMU fusion error: %s", exc)
            dt_loop = time.time() - t0
            time.sleep(max(0.0, period_s - dt_loop))

    def calibrate_level_zero(self, duration_s: float = 2.0):
        # Mode absolut gravitasi: jangan ubah nol referensi EL.
        if self.cfg.absolute_gravity_mode:
            self._logger.info("EL absolute gravity mode aktif: zero calibration diabaikan.")
            return
        duration_s = max(0.2, float(duration_s))
        t_end = time.time() + duration_s
        vals = []
        while time.time() < t_end:
            vals.append(self.get_el_raw_deg())
            time.sleep(0.01)
        if vals:
            avg = sum(vals) / float(len(vals))
            with self._lock:
                self._zero_offset_deg = -avg

    def calibrate_gyro_bias(self, duration_s: float = 2.0):
        duration_s = max(0.2, float(duration_s))
        t_end = time.time() + duration_s
        vals = []
        while time.time() < t_end:
            imu = self.imu_reader.get_latest_dict()
            if imu:
                vals.append(float(imu.get("gyro_dps", (0.0, 0.0, 0.0))[1]))
            time.sleep(0.01)
        if vals:
            with self._lock:
                self._gyro_bias_y = sum(vals) / float(len(vals))

    def get_el_raw_deg(self) -> float:
        with self._lock:
            return float(self._el_est_deg)

    def get_el_deg(self) -> float | None:
        with self._lock:
            last_imu_ts = self._last_imu_ts
            if self.cfg.absolute_gravity_mode:
                el = self._el_est_deg
            else:
                el = self._el_est_deg + self._zero_offset_deg
        if last_imu_ts is None:
            return None
        if (time.time() - float(last_imu_ts)) > float(self.cfg.stale_timeout_s):
            return None
        el = max(float(self.cfg.el_min_deg), min(float(self.cfg.el_max_deg), float(el)))
        return el

    def get_status(self) -> dict:
        with self._lock:
            return {
                "rate_hz": self._rate_hz,
                "gyro_bias_y_dps": self._gyro_bias_y,
                "zero_offset_deg": self._zero_offset_deg,
                "absolute_gravity_mode": self.cfg.absolute_gravity_mode,
                "initialized": self._initialized,
                "last_imu_ts": self._last_imu_ts,
                "last_error": self._last_error,
                "debug": dict(self._debug_snapshot),
            }

    def close(self):
        self._run = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)


class ImuAzElPositionController:
    """Closed-loop AZ/EL hold controller using WT901 orientation feedback."""

    def __init__(self, cfg: ImuAzElHoldConfig, imu_reader: WT901Reader, motor_1, motor_2, cfg_m1: StepperConfig, cfg_m2: StepperConfig):
        self.cfg = cfg
        self.imu_reader = imu_reader
        self.motor_1 = motor_1
        self.motor_2 = motor_2
        self.cfg_m1 = cfg_m1
        self.cfg_m2 = cfg_m2
        self._lock = threading.Lock()
        self._logger = logging.getLogger("motorPID.imu_hold")
        self._zero_az_abs = 0.0
        self._zero_el_abs = 0.0
        self._target_az_rel = 0.0
        self._target_el_rel = 0.0
        self._i_az = 0.0
        self._i_el = 0.0
        self._last_e_az = 0.0
        self._last_e_el = 0.0
        self._last_t = None
        self._last_control = {}
        self._dropout = False
        self._next_log_t = 0.0
        self._log_file = None
        self._log_writer = None
        self._open_log_file()

    @staticmethod
    def _az_error_deg(target_abs: float, actual_abs: float, az_wrap_enabled: bool) -> float:
        e = float(target_abs) - float(actual_abs)
        if not az_wrap_enabled:
            return e
        if e > 180.0:
            e -= 360.0
        elif e < -180.0:
            e += 360.0
        return e

    def _clamp_abs_targets(self, az_abs: float, el_abs: float) -> tuple[float, float]:
        if self.cfg_m1.soft_limit_min_deg is not None:
            az_abs = max(float(self.cfg_m1.soft_limit_min_deg), az_abs)
        if self.cfg_m1.soft_limit_max_deg is not None:
            az_abs = min(float(self.cfg_m1.soft_limit_max_deg), az_abs)
        if self.cfg_m2.soft_limit_min_deg is not None:
            el_abs = max(float(self.cfg_m2.soft_limit_min_deg), el_abs)
        if self.cfg_m2.soft_limit_max_deg is not None:
            el_abs = min(float(self.cfg_m2.soft_limit_max_deg), el_abs)
        return az_abs, el_abs

    def _open_log_file(self):
        path = self.cfg.log_path.strip() or datetime.datetime.now().strftime("imu_azel_hold_%Y%m%d_%H%M%S.csv")
        self._log_file = open(path, "w", newline="")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow(
            [
                "timestamp", "cmd_az_rel_deg", "cmd_el_rel_deg", "cmd_az_abs_deg", "cmd_el_abs_deg",
                "act_az_rel_deg", "act_el_rel_deg", "act_az_abs_deg", "act_el_abs_deg",
                "err_az_deg", "err_el_deg", "cmd_az_speed_sps", "cmd_el_speed_sps", "imu_stale_s", "dropout",
            ]
        )
        self._logger.info("IMU hold log file: %s", path)

    def _append_log(self, row: dict):
        if self._log_writer is None:
            return
        self._log_writer.writerow(
            [
                f"{row['timestamp']:.6f}", f"{row['cmd_az_rel']:.4f}", f"{row['cmd_el_rel']:.4f}",
                f"{row['cmd_az_abs']:.4f}", f"{row['cmd_el_abs']:.4f}",
                f"{row['act_az_rel']:.4f}", f"{row['act_el_rel']:.4f}",
                f"{row['act_az_abs']:.4f}", f"{row['act_el_abs']:.4f}",
                f"{row['err_az']:.4f}", f"{row['err_el']:.4f}",
                f"{row['cmd_az_speed']:.4f}", f"{row['cmd_el_speed']:.4f}",
                f"{row['imu_stale_s']:.4f}", int(bool(row["dropout"])),
            ]
        )
        self._log_file.flush()

    def calibrate_zero_reference(self):
        imu = self.imu_reader.get_latest_dict()
        if not imu:
            raise RuntimeError("IMU sample belum tersedia untuk zeroing.")
        with self._lock:
            self._zero_az_abs = float(imu["azimuth_deg"])
            self._zero_el_abs = float(imu["elevation_deg"])
            self._target_az_rel = 0.0
            self._target_el_rel = 0.0
            self._i_az = 0.0
            self._i_el = 0.0
            self._last_e_az = 0.0
            self._last_e_el = 0.0
            self._last_t = None
        self._logger.info("IMU zero reference set: az=%.3f el=%.3f", self._zero_az_abs, self._zero_el_abs)

    def nudge_target(self, axis: str, delta_deg: float):
        with self._lock:
            if axis == "az":
                self._target_az_rel += float(delta_deg)
            elif axis == "el":
                self._target_el_rel += float(delta_deg)
            else:
                raise ValueError("axis must be az or el")
            az_abs = self._target_az_rel + self._zero_az_abs
            el_abs = self._target_el_rel + self._zero_el_abs
            az_abs, el_abs = self._clamp_abs_targets(az_abs, el_abs)
            self._target_az_rel = az_abs - self._zero_az_abs
            self._target_el_rel = el_abs - self._zero_el_abs

    def hold_current(self):
        imu = self.imu_reader.get_latest_dict()
        if not imu:
            return
        with self._lock:
            self._target_az_rel = float(imu["azimuth_deg"]) - self._zero_az_abs
            self._target_el_rel = float(imu["elevation_deg"]) - self._zero_el_abs
            self._i_az = 0.0
            self._i_el = 0.0

    def get_targets_rel(self) -> tuple[float, float]:
        with self._lock:
            return float(self._target_az_rel), float(self._target_el_rel)

    def get_zero_offsets(self) -> tuple[float, float]:
        with self._lock:
            return float(self._zero_az_abs), float(self._zero_el_abs)

    def update(self, now: float | None = None) -> dict:
        if now is None:
            now = time.time()
        imu = self.imu_reader.get_latest_dict()
        if not imu:
            self.motor_1.stop_smooth()
            self.motor_2.stop_smooth()
            self._dropout = True
            return {"dropout": True, "reason": "no imu sample"}
        imu_ts = float(imu["timestamp"])
        stale_s = max(0.0, now - imu_ts)
        if stale_s > float(self.cfg.dropout_timeout_s):
            self.motor_1.stop_smooth()
            self.motor_2.stop_smooth()
            self._dropout = True
            return {"dropout": True, "reason": "imu stale", "imu_stale_s": stale_s}
        self._dropout = False

        act_az_abs = float(imu["azimuth_deg"])
        act_el_abs = float(imu["elevation_deg"])
        with self._lock:
            cmd_az_abs = self._target_az_rel + self._zero_az_abs
            cmd_el_abs = self._target_el_rel + self._zero_el_abs
            cmd_az_abs, cmd_el_abs = self._clamp_abs_targets(cmd_az_abs, cmd_el_abs)
            self._target_az_rel = cmd_az_abs - self._zero_az_abs
            self._target_el_rel = cmd_el_abs - self._zero_el_abs
            dt = 0.02 if self._last_t is None else max(0.001, now - self._last_t)
            self._last_t = now
            e_az = self._az_error_deg(cmd_az_abs, act_az_abs, self.cfg_m1.az_wrap_enabled)
            e_el = cmd_el_abs - act_el_abs
            self._i_az = max(-300.0, min(300.0, self._i_az + e_az * dt))
            self._i_el = max(-300.0, min(300.0, self._i_el + e_el * dt))
            d_az = (e_az - self._last_e_az) / dt
            d_el = (e_el - self._last_e_el) / dt
            self._last_e_az = e_az
            self._last_e_el = e_el
            cmd_az_speed = self.cfg.az_kp * e_az + self.cfg.az_ki * self._i_az + self.cfg.az_kd * d_az
            cmd_el_speed = self.cfg.el_kp * e_el + self.cfg.el_ki * self._i_el + self.cfg.el_kd * d_el
            if abs(e_az) < 0.06:
                cmd_az_speed = 0.0
            if abs(e_el) < 0.06:
                cmd_el_speed = 0.0
            cmd_az_speed = max(-self.cfg_m1.max_speed_sps, min(self.cfg_m1.max_speed_sps, cmd_az_speed))
            cmd_el_speed = max(-self.cfg_m2.max_speed_sps, min(self.cfg_m2.max_speed_sps, cmd_el_speed))
            cmd_el_speed = gate_el_speed_by_position(
                cmd_el_speed,
                act_el_abs,
                el_min=(0.0 if self.cfg_m2.soft_limit_min_deg is None else float(self.cfg_m2.soft_limit_min_deg)),
                el_max=(90.0 if self.cfg_m2.soft_limit_max_deg is None else float(self.cfg_m2.soft_limit_max_deg)),
            )

        self.motor_1.set_target_speed(cmd_az_speed)
        self.motor_2.set_target_speed(cmd_el_speed)
        row = {
            "timestamp": now, "cmd_az_rel": self._target_az_rel, "cmd_el_rel": self._target_el_rel,
            "cmd_az_abs": cmd_az_abs, "cmd_el_abs": cmd_el_abs,
            "act_az_rel": act_az_abs - self._zero_az_abs, "act_el_rel": act_el_abs - self._zero_el_abs,
            "act_az_abs": act_az_abs, "act_el_abs": act_el_abs, "err_az": e_az, "err_el": e_el,
            "cmd_az_speed": cmd_az_speed, "cmd_el_speed": cmd_el_speed, "imu_stale_s": stale_s, "dropout": False,
        }
        if now >= self._next_log_t:
            self._append_log(row)
            self._next_log_t = now + (1.0 / max(1.0, float(self.cfg.log_rate_hz)))
        self._last_control = row
        return row

    def get_last_control(self) -> dict:
        return dict(self._last_control)

    def close(self):
        try:
            self.motor_1.stop_smooth()
            self.motor_2.stop_smooth()
        except Exception:
            pass
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None


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
            gear = max(1e-9, float(self.cfg.axis_gear_ratio))
            return ((self._position_full_steps / self.cfg.steps_per_rev) * 360.0) / gear

    def set_position_deg(self, position_deg: float):
        """Software calibration point: set current axis angle without moving motor."""
        with self._lock:
            gear = max(1e-9, float(self.cfg.axis_gear_ratio))
            self._position_full_steps = (float(position_deg) * gear / 360.0) * float(self.cfg.steps_per_rev)

    def get_status(self) -> dict:
        with self._lock:
            gear = max(1e-9, float(self.cfg.axis_gear_ratio))
            pos_deg = ((self._position_full_steps / self.cfg.steps_per_rev) * 360.0) / gear
            return {
                "target_speed_sps": self._target_speed_sps,
                "current_speed_sps": self._current_speed_sps,
                "microstep": self.cfg.microstep,
                "position_deg": pos_deg,
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
            gear = max(1e-9, float(self.cfg.axis_gear_ratio))
            current_deg = ((self.get_position_steps() / self.cfg.steps_per_rev) * 360.0) / gear
            next_deg = (((self.get_position_steps() + step_delta_full) / self.cfg.steps_per_rev) * 360.0) / gear
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
            gear = max(1e-9, float(self.cfg.axis_gear_ratio))
            return ((self._position_full_steps / self.cfg.steps_per_rev) * 360.0) / gear

    def set_position_deg(self, position_deg: float):
        """Software calibration point: set current axis angle without moving motor."""
        with self._lock:
            gear = max(1e-9, float(self.cfg.axis_gear_ratio))
            self._position_full_steps = (float(position_deg) * gear / 360.0) * float(self.cfg.steps_per_rev)

    def get_status(self) -> dict:
        with self._lock:
            gear = max(1e-9, float(self.cfg.axis_gear_ratio))
            pos_deg = ((self._position_full_steps / self.cfg.steps_per_rev) * 360.0) / gear
            return {
                "target_speed_sps": self._target_speed_sps,
                "current_speed_sps": self._current_speed_sps,
                "microstep": self.cfg.microstep,
                "position_deg": pos_deg,
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
                gear = max(1e-9, float(self.cfg.axis_gear_ratio))
                current_deg = ((self.get_position_steps() / self.cfg.steps_per_rev) * 360.0) / gear
                next_deg = (((self.get_position_steps() + step_delta_full) / self.cfg.steps_per_rev) * 360.0) / gear
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


def parse_float_safe(value, default=None):
    try:
        if value is None:
            return default
        if isinstance(value, str):
            s = value.strip().replace(",", ".")
            if s == "":
                return default
            return float(s)
        return float(value)
    except Exception:
        return default


def command_el_with_bounds(
    motor_2,
    command_speed: float,
    direction: int,
    el_min: float = 0.0,
    el_max: float = 90.0,
    current_el_deg: float | None = None,
):
    """
    Keyboard EL command with explicit 0..90 boundary guard.
    direction: +1 (up), -1 (down), 0 (stop)
    """
    if direction == 0:
        motor_2.stop_smooth()
        return
    if current_el_deg is None:
        st2 = motor_2.get_status()
        cur_el = parse_float_safe(st2.get("position_deg"), 0.0)
    else:
        cur_el = parse_float_safe(current_el_deg, 0.0)
    if direction > 0 and cur_el >= el_max - 1e-3:
        motor_2.stop_smooth()
        return
    if direction < 0 and cur_el <= el_min + 1e-3:
        motor_2.stop_smooth()
        return
    motor_2.set_target_speed(float(direction) * abs(float(command_speed)))


def gate_el_speed_by_position(
    cmd_el_speed: float,
    current_el_deg: float | None,
    el_min: float = 0.0,
    el_max: float = 90.0,
) -> float:
    """
    EL guard:
    - jika posisi <= el_min, blok command negatif (turun)
    - jika posisi >= el_max, blok command positif (naik)
    """
    cur = parse_float_safe(current_el_deg, None)
    cmd = parse_float_safe(cmd_el_speed, 0.0)
    if cur is None:
        return float(cmd)
    if cur <= float(el_min) + 1e-3 and cmd < 0.0:
        return 0.0
    if cur >= float(el_max) - 1e-3 and cmd > 0.0:
        return 0.0
    return float(cmd)


def _adaptive_gains_common(axis: str, err_abs_deg: float):
    if axis == "az":
        if err_abs_deg > 20:
            return 120.0, 0.20, 25.0
        if err_abs_deg > 5:
            return 85.0, 0.12, 18.0
        if err_abs_deg > 1:
            return 45.0, 0.06, 10.0
        return 20.0, 0.0, 6.0
    if err_abs_deg > 15:
        return 110.0, 0.18, 20.0
    if err_abs_deg > 4:
        return 80.0, 0.10, 15.0
    if err_abs_deg > 1:
        return 40.0, 0.05, 8.0
    return 18.0, 0.0, 5.0


def _pid_speed_common(axis: str, err_deg: float, state: dict, max_speed: float) -> float:
    now = time.time()
    dt = 0.02 if state.get("last_t") is None else max(0.001, now - float(state["last_t"]))
    state["last_t"] = now
    kp, ki, kd = _adaptive_gains_common(axis, abs(err_deg))
    state["i"] = float(state.get("i", 0.0)) + (err_deg * dt)
    state["i"] = max(-300.0, min(300.0, state["i"]))
    d = (err_deg - float(state.get("last_e", 0.0))) / dt
    state["last_e"] = err_deg
    out = (kp * err_deg) + (ki * state["i"]) + (kd * d)
    if abs(err_deg) < 0.08:
        out = 0.0
    return max(-max_speed, min(max_speed, out))


def set_azimuth(target_az_deg: float, current_az_deg: float, pid_state: dict, max_speed: float, wrap_enabled: bool = True) -> float:
    err = float(target_az_deg) - float(current_az_deg)
    if wrap_enabled:
        if err > 180.0:
            err -= 360.0
        elif err < -180.0:
            err += 360.0
    return _pid_speed_common("az", err, pid_state, max_speed)


def set_elevation(target_el_deg: float, current_el_deg: float, pid_state: dict, max_speed: float) -> float:
    err = float(target_el_deg) - float(current_el_deg)
    return _pid_speed_common("el", err, pid_state, max_speed)


def draw_sim_ui(st1, st2, command_speed):
    clear_screen()
    az_disp = st1["position_deg"] % 360.0
    print("=== ROTATOR STEPPER SIMULATION UI ===")
    print("Konfigurasi: NEMA23 | TB6600 | Microstep 2 (400 pulse/rev) | Current 2.0A")
    print("")
    print("Kontrol:")
    print("- Motor 1 (AZ): Left/Right atau A/D")
    print("- Motor 2 (EL): Up/Down   atau W/S")
    print("- Space=Stop halus, E=E-Stop, R=Reset fault, +/- speed, 1..5 microstep, Q=Quit")
    print("")
    print(
        f"AZ  pos={az_disp:8.2f} deg  spd={st1['current_speed_sps']:8.1f} sps  "
        f"tgt={st1['target_speed_sps']:8.1f}  ms={st1['microstep']}"
    )
    print(
        f"EL  pos={st2['position_deg']:8.2f} deg  spd={st2['current_speed_sps']:8.1f} sps  "
        f"tgt={st2['target_speed_sps']:8.1f}  ms={st2['microstep']}"
    )
    if st1["fault_latched"] or st2["fault_latched"]:
        print(f"FAULT: {st1['fault_msg']} {st2['fault_msg']}")
    print(f"\nCommand speed: {command_speed:.1f} sps")


def run_imu_azel_keyboard_mode(
    motor_1,
    motor_2,
    cfg_m1: StepperConfig,
    cfg_m2: StepperConfig,
    imu_reader: WT901Reader,
    imu_ctrl: ImuAzElPositionController,
):
    print(
        "\n=== IMU AZ/EL POSITION HOLD MODE ===\n"
        "Target dikunci ke IMU feedback. Keyboard override (nudge):\n"
        "  A/Left  : AZ target -\n"
        "  D/Right : AZ target +\n"
        "  W/Up    : EL target +\n"
        "  S/Down  : EL target -\n"
        "  Z       : Zero IMU reference frame\n"
        "  Space   : Hold current orientation\n"
        "  E       : Emergency stop, R reset fault\n"
        "  +/-     : Ubah nudge step\n"
        "  Q       : Quit\n"
    )
    nudge = max(0.05, float(imu_ctrl.cfg.nudge_step_deg))
    report_period_s = 0.1
    last_report = 0.0
    ctrl_period_s = 1.0 / max(10.0, float(imu_ctrl.cfg.control_rate_hz))

    with RawTerminal():
        while True:
            now = time.time()
            key = get_key_nonblocking(0.01)
            if key in ("q", "Q"):
                motor_1.stop_smooth()
                motor_2.stop_smooth()
                print("\nExit IMU hold mode.")
                break
            elif key in ("\x1b[C", "d", "D"):
                imu_ctrl.nudge_target("az", +nudge)
            elif key in ("\x1b[D", "a", "A"):
                imu_ctrl.nudge_target("az", -nudge)
            elif key in ("\x1b[A", "w", "W"):
                imu_ctrl.nudge_target("el", +nudge)
            elif key in ("\x1b[B", "s", "S"):
                imu_ctrl.nudge_target("el", -nudge)
            elif key in ("z", "Z"):
                imu_ctrl.calibrate_zero_reference()
            elif key == " ":
                imu_ctrl.hold_current()
            elif key in ("e", "E"):
                motor_1.emergency_stop("Emergency stop keyboard")
                motor_2.emergency_stop("Emergency stop keyboard")
            elif key in ("r", "R"):
                motor_1.reset_fault()
                motor_2.reset_fault()
            elif key == "+":
                nudge = min(15.0, nudge + 0.1)
            elif key == "-":
                nudge = max(0.05, nudge - 0.1)

            ctrl = imu_ctrl.update(now)

            if now - last_report >= report_period_s:
                st1 = motor_1.get_status()
                st2 = motor_2.get_status()
                imu = imu_reader.get_latest_dict()
                taz, tel = imu_ctrl.get_targets_rel()
                z_az, z_el = imu_ctrl.get_zero_offsets()
                if imu:
                    az_rel = imu["azimuth_deg"] - z_az
                    el_rel = imu["elevation_deg"] - z_el
                    msg_imu = (
                        f"IMU rel AZ={az_rel:7.2f} EL={el_rel:7.2f} "
                        f"abs AZ={imu['azimuth_deg']:7.2f} EL={imu['elevation_deg']:7.2f}"
                    )
                else:
                    msg_imu = "IMU unavailable"
                msg_ctrl = (
                    f"TGT rel AZ={taz:7.2f} EL={tel:7.2f} "
                    f"SPDcmd AZ={st1['target_speed_sps']:7.1f} EL={st2['target_speed_sps']:7.1f}"
                )
                msg_dropout = ""
                if ctrl.get("dropout"):
                    msg_dropout = f" DROPOUT={ctrl.get('reason', '-')}"
                sys.stdout.write(f"\r{msg_imu} | {msg_ctrl} | nudge={nudge:.2f}{msg_dropout}      ")
                sys.stdout.flush()
                last_report = now

            dt = time.time() - now
            if dt < ctrl_period_s:
                time.sleep(ctrl_period_s - dt)


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
    def __init__(
        self,
        motor_1,
        motor_2,
        cfg_m1,
        cfg_m2,
        imu_reader: WT901Reader | None = None,
        el_imu_tracker: ELImuFusionTracker | None = None,
        imu_hold_ctrl: ImuAzElPositionController | None = None,
    ):
        self.motor_1 = motor_1
        self.motor_2 = motor_2
        self.cfg_m1 = cfg_m1
        self.cfg_m2 = cfg_m2
        self.imu_reader = imu_reader
        self.el_imu_tracker = el_imu_tracker
        self.imu_hold_ctrl = imu_hold_ctrl
        self.imu_hold_enabled = imu_hold_ctrl is not None
        self.command_speed = 600.0
        self.az_pos_pressed = False
        self.az_neg_pressed = False
        self.el_pos_pressed = False
        self.el_neg_pressed = False
        self.selected_tle = None
        self.selected_sat_name = "-"
        self.sat_az = None
        self.sat_el = None
        self.az_ls_deg = 0.0
        self.ts = load.timescale() if SKYFIELD_AVAILABLE else None
        self.tracking_enabled = False
        self.tracking_diag = "init"
        self._last_valid_sat_az = None
        self._last_valid_sat_el = None
        self._tracking_dbg_counter = 0
        self._tracking_logger = logging.getLogger("motorPID.sim_gui_tracking")
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
        tk.Label(cal, text="EL offset (deg)").pack(side=tk.LEFT, padx=(10, 2))
        self.ent_el_offset = tk.Entry(cal, width=8)
        self.ent_el_offset.insert(0, f"{self.cfg_m2.el_offset_deg:.2f}")
        self.ent_el_offset.pack(side=tk.LEFT, padx=2)

        limit_row = tk.Frame(self.content)
        limit_row.pack(pady=2, fill="x")
        tk.Label(limit_row, text="Limit Switch Location (deg)", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=(8, 12))
        tk.Label(limit_row, text="AZ LS ref").pack(side=tk.LEFT, padx=(2, 2))
        self.ent_az_ls = tk.Entry(limit_row, width=7)
        self.ent_az_ls.insert(0, "0")
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
        tk.Button(ctrl, text="CAL AZ=0", width=10, command=lambda: self._calibrate_axis("az", 0.0)).grid(row=1, column=3, padx=4, pady=4)
        tk.Button(ctrl, text="CAL EL=0", width=10, command=lambda: self._calibrate_axis("el", 0.0)).grid(row=1, column=4, padx=4, pady=4)
        tk.Button(ctrl, text="+ SPEED", width=10, command=self._speed_up).grid(row=1, column=5, padx=4, pady=4)
        tk.Button(ctrl, text="- SPEED", width=10, command=self._speed_down).grid(row=1, column=6, padx=4, pady=4)
        self.btn_imu_hold = tk.Button(ctrl, text="IMU HOLD: OFF", width=14, command=self._toggle_imu_hold, bg="#5a5a5a", fg="white")
        self.btn_imu_hold.grid(row=2, column=0, padx=4, pady=4)
        tk.Button(ctrl, text="IMU ZERO", width=10, command=self._imu_zero).grid(row=2, column=1, padx=4, pady=4)
        tk.Button(ctrl, text="HOLD HERE", width=10, command=self._imu_hold_here).grid(row=2, column=2, padx=4, pady=4)

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
        self.root.bind("z", lambda e: self._imu_zero())
        self.root.bind("Z", lambda e: self._imu_zero())

        self.root.protocol("WM_DELETE_WINDOW", self.root.quit)
        self._refresh_imu_hold_button()
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
                # Fallback agar tracking simulasi tetap berjalan saat API gagal/no data.
                self._tle_cache = list(FALLBACK_TLE)
                self.lst_tle.insert(tk.END, "No API data, fallback TLE loaded")
                parsed = self._tle_cache
            for i, it in enumerate(parsed):
                sat_id = it.get("id")
                text = f"{i+1:02d}. {it['name']}" + (f" [{sat_id}]" if sat_id else "")
                self.lst_tle.insert(tk.END, text)
            self.lst_tle.selection_clear(0, tk.END)
            self.lst_tle.selection_set(0)
            self._on_select_tle()
        except Exception as exc:
            self._tle_cache = list(FALLBACK_TLE)
            self.lst_tle.insert(tk.END, f"Load error: {exc} -> fallback TLE")
            for i, it in enumerate(self._tle_cache):
                sat_id = it.get("id")
                text = f"{i+1:02d}. {it['name']}" + (f" [{sat_id}]" if sat_id else "")
                self.lst_tle.insert(tk.END, text)
            self.lst_tle.selection_clear(0, tk.END)
            self.lst_tle.selection_set(1 if len(self._tle_cache) > 1 else 0)
            self._on_select_tle()

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
            self.tracking_diag = "no_tle_or_skyfield"
            return
        try:
            lat = float(self.ent_lat.get().strip())
            lon = float(self.ent_lon.get().strip())
            alt_m = float(self.ent_alt.get().strip())
        except ValueError:
            self.sat_az = None
            self.sat_el = None
            self.tracking_diag = "invalid_observer_latlonalt"
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
            self._last_valid_sat_az = self.sat_az
            self._last_valid_sat_el = self.sat_el
            self.tracking_diag = "sat_ok"
        except Exception as exc:
            # Jangan langsung drop target ke None agar tracking tidak terlihat "mati" saat hiccup sesaat.
            if self._last_valid_sat_az is not None and self._last_valid_sat_el is not None:
                self.sat_az = self._last_valid_sat_az
                self.sat_el = self._last_valid_sat_el
                self.tracking_diag = f"sat_calc_error_using_last_valid: {exc}"
            else:
                self.sat_az = None
                self.sat_el = None
                self.tracking_diag = f"sat_calc_error_no_valid: {exc}"

    def set_azimuth(self, target_az_deg: float):
        st1 = self.motor_1.get_status()
        cur_az = float(st1["position_deg"]) % 360.0
        target_az = float(target_az_deg) % 360.0
        err_az = self._az_shortest_error(target_az, cur_az)
        cmd_az = self._pid_speed_cmd("az", err_az, self.cfg_m1.max_speed_sps)
        self.motor_1.set_target_speed(cmd_az)
        return {"cur_az": cur_az, "target_az": target_az, "err_az": err_az, "cmd_az": cmd_az}

    def set_elevation(self, target_el_deg: float):
        st2 = self.motor_2.get_status()
        imu = self.imu_reader.get_latest_dict() if self.imu_reader is not None else {}
        cur_el = parse_float_safe(imu.get("elevation_deg"), None) if imu else None
        if cur_el is None:
            cur_el = parse_float_safe(st2.get("position_deg"), 0.0)
        target_el = max(0.0, min(90.0, float(target_el_deg)))
        err_el = target_el - cur_el
        cmd_el = self._pid_speed_cmd("el", err_el, self.cfg_m2.max_speed_sps)
        cmd_el = gate_el_speed_by_position(
            cmd_el,
            cur_el,
            el_min=(0.0 if self.cfg_m2.soft_limit_min_deg is None else float(self.cfg_m2.soft_limit_min_deg)),
            el_max=(90.0 if self.cfg_m2.soft_limit_max_deg is None else float(self.cfg_m2.soft_limit_max_deg)),
        )
        self.motor_2.set_target_speed(cmd_el)
        return {"cur_el": cur_el, "target_el": target_el, "err_el": err_el, "cmd_el": cmd_el}

    def _press_axis(self, axis: str):
        if self.imu_hold_enabled and self.imu_hold_ctrl is not None:
            step = max(0.05, float(self.imu_hold_ctrl.cfg.nudge_step_deg))
            if axis == "az_pos":
                self.imu_hold_ctrl.nudge_target("az", +step)
            elif axis == "az_neg":
                self.imu_hold_ctrl.nudge_target("az", -step)
            elif axis == "el_pos":
                self.imu_hold_ctrl.nudge_target("el", +step)
            elif axis == "el_neg":
                self.imu_hold_ctrl.nudge_target("el", -step)
            return
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

        el_min = 0.0 if self.cfg_m2.soft_limit_min_deg is None else float(self.cfg_m2.soft_limit_min_deg)
        el_max = 90.0 if self.cfg_m2.soft_limit_max_deg is None else float(self.cfg_m2.soft_limit_max_deg)
        imu = self.imu_reader.get_latest_dict() if self.imu_reader is not None else {}
        if imu:
            el_cur = parse_float_safe(imu.get("elevation_deg"), None)
        else:
            el_cur = self.el_imu_tracker.get_el_deg() if self.el_imu_tracker is not None else None
        if self.el_pos_pressed and not self.el_neg_pressed:
            command_el_with_bounds(self.motor_2, spd, +1, el_min=el_min, el_max=el_max, current_el_deg=el_cur)
        elif self.el_neg_pressed and not self.el_pos_pressed:
            command_el_with_bounds(self.motor_2, spd, -1, el_min=el_min, el_max=el_max, current_el_deg=el_cur)
        else:
            command_el_with_bounds(self.motor_2, spd, 0, el_min=el_min, el_max=el_max, current_el_deg=el_cur)

    def _smooth_stop(self):
        if self.imu_hold_enabled and self.imu_hold_ctrl is not None:
            self.imu_hold_ctrl.hold_current()
            return
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

    def _calibrate_axis(self, axis: str, deg: float):
        try:
            if axis == "az":
                self.motor_1.set_position_deg(float(deg))
            elif axis == "el":
                self.motor_2.set_position_deg(float(deg))
            else:
                raise ValueError("axis must be az|el")
        except Exception as exc:
            self.lbl_sat.config(text=f"SAT: {self.selected_sat_name} | AZ: - | EL: -  [CAL ERR: {exc}]")

    def _apply_limit_offset(self):
        try:
            az_offset = float(self.ent_az_offset.get().strip())
            el_offset = float(self.ent_el_offset.get().strip())
            az_ls = validate_az_ls(float(self.ent_az_ls.get().strip()))
            el_min = float(self.ent_el_min.get().strip())
            el_max = float(self.ent_el_max.get().strip())
            if el_min >= el_max:
                raise ValueError("EL min must be < EL max")

            self.cfg_m1.az_offset_deg = az_offset
            self.cfg_m2.el_offset_deg = el_offset
            self.az_ls_deg = az_ls
            self.cfg_m1.az_ls_deg = az_ls
            # Aktifkan blok crossing AZ LS saat user mengisi nilai non-zero/non-360.
            self.cfg_m1.az_ls_block_crossing = not (abs(az_ls) < 1e-9 or abs(az_ls - 360.0) < 1e-9)
            # Untuk mode AZ LS, nonaktifkan soft-limit AZ 0/360 agar 0 derajat
            # bukan batas lagi; batas AZ ditentukan oleh crossing AZ LS saja.
            self.cfg_m1.soft_limit_min_deg = None
            self.cfg_m1.soft_limit_max_deg = None
            self.cfg_m2.soft_limit_min_deg = el_min
            self.cfg_m2.soft_limit_max_deg = el_max
            self.lbl_sat.config(
                text=(
                    f"SAT: {self.selected_sat_name} | AZ: {self.sat_az:.2f}° | EL: {self.sat_el:.2f}° "
                    if self.sat_az is not None and self.sat_el is not None
                    else f"SAT: {self.selected_sat_name} | AZ: - | EL: -"
                )
                + (
                    f"  [AZ LS ref={self.az_ls_deg:.2f} deg | crossing block ON]"
                    if self.cfg_m1.az_ls_block_crossing
                    else f"  [AZ LS ref={self.az_ls_deg:.2f} deg | crossing block OFF]"
                )
            )
        except Exception as exc:
            self.lbl_sat.config(text=f"SAT: {self.selected_sat_name} | AZ: - | EL: -  [ERROR: {exc}]")

    def _toggle_tracking(self):
        if self.imu_hold_enabled:
            self._set_imu_hold(False)
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

    def _refresh_imu_hold_button(self):
        if self.imu_hold_ctrl is None:
            self.btn_imu_hold.config(text="IMU HOLD: N/A", bg="#777777", state=tk.DISABLED)
            return
        if self.imu_hold_enabled:
            self.btn_imu_hold.config(text="IMU HOLD: ON", bg="#0b8f3a", state=tk.NORMAL)
        else:
            self.btn_imu_hold.config(text="IMU HOLD: OFF", bg="#5a5a5a", state=tk.NORMAL)

    def _set_imu_hold(self, enabled: bool):
        if self.imu_hold_ctrl is None:
            self.imu_hold_enabled = False
            self._refresh_imu_hold_button()
            return
        self.imu_hold_enabled = bool(enabled)
        if self.imu_hold_enabled:
            self._set_tracking(False)
            self.imu_hold_ctrl.hold_current()
        else:
            self.motor_1.stop_smooth()
            self.motor_2.stop_smooth()
        self._refresh_imu_hold_button()

    def _toggle_imu_hold(self):
        self._set_imu_hold(not self.imu_hold_enabled)

    def _imu_zero(self):
        if self.imu_hold_ctrl is None and self.el_imu_tracker is None:
            return
        try:
            if self.imu_hold_ctrl is not None:
                self.imu_hold_ctrl.calibrate_zero_reference()
            # EL sengaja tidak di-zero agar tetap absolut terhadap gravitasi.
            self.lbl_sat.config(text=f"SAT: {self.selected_sat_name} | AZ: - | EL: -  [IMU zeroed | EL absolute]")
        except Exception as exc:
            self.lbl_sat.config(text=f"SAT: {self.selected_sat_name} | AZ: - | EL: -  [IMU zero error: {exc}]")

    def _imu_hold_here(self):
        if self.imu_hold_ctrl is None:
            return
        self.imu_hold_ctrl.hold_current()

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
            self.tracking_diag = "tracking_off"
            return
        if self.sat_az is None or self.sat_el is None:
            self.tracking_diag = "sat_none_no_target"
            return
        imu = self.imu_reader.get_latest_dict() if self.imu_reader is not None else {}
        el_imu = self.el_imu_tracker.get_el_deg() if self.el_imu_tracker is not None else None
        if imu:
            cur_az_raw = parse_float_safe(imu.get("azimuth_deg"), None)
            cur_az = (cur_az_raw % 360.0) if cur_az_raw is not None else (self.motor_1.get_status()["position_deg"] % 360.0)
            cur_el = parse_float_safe(imu.get("elevation_deg"), None)
            if cur_el is None:
                cur_el = float(el_imu) if el_imu is not None else 0.0
        else:
            st1 = self.motor_1.get_status()
            st2 = self.motor_2.get_status()
            cur_az = st1["position_deg"] % 360.0
            cur_el = float(el_imu) if el_imu is not None else st2["position_deg"]
        # Konversi azimuth satelit (kompas) ke frame mekanik rotator dengan offset kalibrasi.
        target_az = (self.sat_az + self.cfg_m1.az_offset_deg) % 360.0
        target_el = self.sat_el + self.cfg_m2.el_offset_deg

        # Clamp target ke rentang mekanik agar tidak forcing ke luar limit.
        if self.cfg_m1.soft_limit_min_deg is not None:
            target_az = max(self.cfg_m1.soft_limit_min_deg, target_az)
        if self.cfg_m1.soft_limit_max_deg is not None:
            target_az = min(self.cfg_m1.soft_limit_max_deg, target_az)

        err_az = self._az_shortest_error(target_az, cur_az)
        err_el = target_el - cur_el

        cmd_az = self._pid_speed_cmd("az", err_az, self.cfg_m1.max_speed_sps)
        cmd_el = self._pid_speed_cmd("el", err_el, self.cfg_m2.max_speed_sps)
        cmd_el = gate_el_speed_by_position(
            cmd_el,
            cur_el,
            el_min=(0.0 if self.cfg_m2.soft_limit_min_deg is None else float(self.cfg_m2.soft_limit_min_deg)),
            el_max=(90.0 if self.cfg_m2.soft_limit_max_deg is None else float(self.cfg_m2.soft_limit_max_deg)),
        )
        self.motor_1.set_target_speed(cmd_az)
        self.motor_2.set_target_speed(cmd_el)
        self._tracking_dbg_counter += 1
        self.tracking_diag = (
            f"ok tAZ={target_az:.2f} tEL={target_el:.2f} "
            f"eAZ={err_az:.2f} eEL={err_el:.2f} cmdAZ={cmd_az:.1f} cmdEL={cmd_el:.1f}"
        )
        if self._tracking_dbg_counter % 20 == 0:
            self._tracking_logger.info("SIM-GUI tracking: %s", self.tracking_diag)

    def _draw_rotator(self, az_deg, el_deg):
        self.canvas.delete("all")
        cx, cy, r = 200, 130, 90
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#33ccff", width=2)
        self.canvas.create_text(cx, cy + 105, fill="white", text="AZ", font=("Arial", 10, "bold"))
        az = az_deg % 360.0
        import math
        rad = math.radians(az - 90.0)
        x2 = cx + r * 0.85 * math.cos(rad)
        y2 = cy + r * 0.85 * math.sin(rad)
        self.canvas.create_line(cx, cy, x2, y2, fill="#ffd166", width=4, arrow=tk.LAST)

        x0, y0, w, h = 460, 60, 240, 160
        self.canvas.create_rectangle(x0, y0, x0 + w, y0 + h, outline="#33ccff", width=2)
        self.canvas.create_text(x0 + w / 2, y0 + h + 20, fill="white", text="EL", font=("Arial", 10, "bold"))
        el = max(0.0, min(90.0, el_deg))
        bar_h = (el / 90.0) * (h - 20)
        self.canvas.create_rectangle(x0 + 20, y0 + h - 10 - bar_h, x0 + w - 20, y0 + h - 10, fill="#06d6a0")
        self.canvas.create_text(x0 + w / 2, y0 + h - bar_h - 20, fill="white", text=f"{el:.1f}°")

    def _update_ui(self):
        st1 = self.motor_1.get_status()
        st2 = self.motor_2.get_status()
        imu = self.imu_reader.get_latest_dict() if self.imu_reader is not None else {}
        el_imu = self.el_imu_tracker.get_el_deg() if self.el_imu_tracker is not None else None
        if imu:
            az_disp = parse_float_safe(imu.get("azimuth_deg"), None)
            if az_disp is None:
                az_disp = st1["position_deg"] % 360.0
            else:
                az_disp = az_disp % 360.0
            pose_src = "IMU_AZ/IMU_EL"
        else:
            az_disp = st1["position_deg"] % 360.0
            pose_src = "IMU_EL_ONLY" if el_imu is not None else "MOTOR"
        # EL prioritas dari IMU (sesuai permintaan: mirror AZ source), fallback ke estimator/stepper.
        if imu:
            el_disp = parse_float_safe(imu.get("elevation_deg"), None)
            if el_disp is None:
                el_disp = float(el_imu) if el_imu is not None else float(st2["position_deg"])
        else:
            el_disp = float(el_imu) if el_imu is not None else float(st2["position_deg"])
        self._calc_selected_az_el()
        self._tracking_step()
        imu_hold_row = None
        if self.imu_hold_enabled and self.imu_hold_ctrl is not None:
            imu_hold_row = self.imu_hold_ctrl.update()
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
                f"AZ: pos={az_disp:.2f}°  spd={st1['current_speed_sps']:.1f} sps  tgt={st1['target_speed_sps']:.1f}\n"
                f"EL: pos={el_disp:.2f}°  spd={st2['current_speed_sps']:.1f} sps  tgt={st2['target_speed_sps']:.1f}\n"
                f"Command speed={self.command_speed:.1f} sps | Microstep={st1['microstep']} | PoseSrc={pose_src} | "
                f"Track={'ON' if self.tracking_enabled else 'OFF'} | Diag={self.tracking_diag}{fault}"
            )
        )
        if self.imu_reader is not None:
            if imu:
                taz = tel = 0.0
                if self.imu_hold_ctrl is not None:
                    taz, tel = self.imu_hold_ctrl.get_targets_rel()
                hold_txt = "OFF"
                if self.imu_hold_ctrl is not None:
                    hold_txt = "ON" if self.imu_hold_enabled else "OFF"
                extra = ""
                if imu_hold_row and imu_hold_row.get("dropout"):
                    extra = f" DROP={imu_hold_row.get('reason', '-')}"
                self.lbl_status.config(
                    text=self.lbl_status.cget("text")
                    + (
                        f"\nIMU AZ={parse_float_safe(imu.get('azimuth_deg'), 0.0):.2f}° "
                        f"EL={parse_float_safe(imu.get('elevation_deg'), 0.0):.2f}° "
                        f"HDG={parse_float_safe(imu.get('compass_deg'), 0.0):.2f}° "
                        f"T={parse_float_safe(imu.get('temperature_c'), 0.0):.1f}C"
                        f" | IMU-HOLD={hold_txt} TGTrel AZ={taz:.2f} EL={tel:.2f}{extra}"
                    )
                )
        if self.el_imu_tracker is not None:
            st_el = self.el_imu_tracker.get_status()
            dbg = st_el.get("debug", {}) if isinstance(st_el, dict) else {}
            self.lbl_status.config(
                text=self.lbl_status.cget("text")
                + (
                    f"\nEL-IMU raw={self.el_imu_tracker.get_el_raw_deg():.2f}° est={el_disp:.2f}° "
                    f"rate={st_el.get('rate_hz', 0.0):.1f}Hz "
                    f"imuPitch={dbg.get('imu_pitch_deg', 0.0):.2f} "
                    f"accPitch={dbg.get('acc_pitch_deg', 0.0):.2f}"
                )
            )
        self._draw_rotator(az_disp, el_disp)
        self.root.after(50, self._update_ui)

    def run(self):
        self.root.mainloop()


def run_cli_mode(
    motor_1,
    motor_2,
    cfg_m1,
    cfg_m2,
    imu_reader: WT901Reader | None = None,
    el_imu_tracker: ELImuFusionTracker | None = None,
):
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
        imu = imu_reader.get_latest_dict() if imu_reader is not None else {}
        el_imu = el_imu_tracker.get_el_deg() if el_imu_tracker is not None else None
        if imu:
            cur_az_raw = parse_float_safe(imu.get("azimuth_deg"), None)
            cur_az = (cur_az_raw % 360.0) if cur_az_raw is not None else (motor_1.get_status()["position_deg"] % 360.0)
            cur_el = parse_float_safe(imu.get("elevation_deg"), None)
            if cur_el is None:
                cur_el = float(el_imu) if el_imu is not None else motor_2.get_status()["position_deg"]
        else:
            st1 = motor_1.get_status()
            st2 = motor_2.get_status()
            cur_az = st1["position_deg"] % 360.0
            cur_el = float(el_imu) if el_imu is not None else st2["position_deg"]
        target_az = (saz + cfg_m1.az_offset_deg) % 360.0
        target_el = sel + cfg_m2.el_offset_deg
        if cfg_m1.soft_limit_min_deg is not None:
            target_az = max(cfg_m1.soft_limit_min_deg, target_az)
        if cfg_m1.soft_limit_max_deg is not None:
            target_az = min(cfg_m1.soft_limit_max_deg, target_az)
        motor_1.set_target_speed(pid("az", az_error(target_az, cur_az), cfg_m1.max_speed_sps))
        cmd_el = pid("el", target_el - cur_el, cfg_m2.max_speed_sps)
        cmd_el = gate_el_speed_by_position(
            cmd_el,
            cur_el,
            el_min=(0.0 if cfg_m2.soft_limit_min_deg is None else float(cfg_m2.soft_limit_min_deg)),
            el_max=(90.0 if cfg_m2.soft_limit_max_deg is None else float(cfg_m2.soft_limit_max_deg)),
        )
        motor_2.set_target_speed(cmd_el)

    def print_status():
        st1 = motor_1.get_status()
        st2 = motor_2.get_status()
        imu = imu_reader.get_latest_dict() if imu_reader is not None else {}
        el_imu = el_imu_tracker.get_el_deg() if el_imu_tracker is not None else None
        if imu:
            az_raw = parse_float_safe(imu.get("azimuth_deg"), None)
            az_disp = (az_raw % 360.0) if az_raw is not None else (st1["position_deg"] % 360.0)
            pose_src = "IMU_AZ/IMU_EL"
        else:
            az_disp = st1["position_deg"] % 360.0
            pose_src = "IMU_EL_ONLY" if el_imu is not None else "MOTOR"
        el_disp = parse_float_safe(imu.get("elevation_deg"), None) if imu else None
        if el_disp is None:
            el_disp = float(el_imu) if el_imu is not None else st2["position_deg"]
        saz, sel = sat_az_el()
        sat_txt = f"{selected_sat_name} AZ={saz:.2f} EL={sel:.2f}" if saz is not None and sel is not None else selected_sat_name
        print(f"M1(AZ) pos={az_disp:.2f} spd={st1['current_speed_sps']:.1f} tgt={st1['target_speed_sps']:.1f}")
        print(f"M2(EL) pos={el_disp:.2f} spd={st2['current_speed_sps']:.1f} tgt={st2['target_speed_sps']:.1f}")
        print(f"speed={command_speed:.1f} ms={st1['microstep']} pose={pose_src} track={'ON' if tracking_enabled else 'OFF'} sat={sat_txt}")
        el_steps_per_deg = (cfg_m2.steps_per_rev * st2["microstep"] * max(1e-9, cfg_m2.axis_gear_ratio)) / 360.0
        print(
            f"EL conv steps/rev={cfg_m2.steps_per_rev} micro={st2['microstep']} "
            f"gear={cfg_m2.axis_gear_ratio:.4f} => {el_steps_per_deg:.4f} microsteps/deg"
        )
        print(
            f"limits AZ[{cfg_m1.soft_limit_min_deg},{cfg_m1.soft_limit_max_deg}] "
            f"EL[{cfg_m2.soft_limit_min_deg},{cfg_m2.soft_limit_max_deg}] "
            f"offset_az={cfg_m1.az_offset_deg} offset_el={cfg_m2.el_offset_deg}"
        )
        if imu_reader is not None:
            imu = imu_reader.get_latest_dict()
            if imu:
                print(
                    "imu "
                    f"AZ={parse_float_safe(imu.get('azimuth_deg'), 0.0):.2f}({parse_float_safe(imu.get('azimuth_rad'), 0.0):.4f} rad) "
                    f"EL={parse_float_safe(imu.get('elevation_deg'), 0.0):.2f}({parse_float_safe(imu.get('elevation_rad'), 0.0):.4f} rad) "
                    f"HDG={parse_float_safe(imu.get('compass_deg'), 0.0):.2f} "
                    f"GYRO={imu['gyro_dps']} MAG={imu['mag']}"
                )
            else:
                print(f"imu status={imu_reader.get_status()}")
        if el_imu_tracker is not None:
            print(f"elimu dbg={el_imu_tracker.get_status()}")

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
                    imu_cur = imu_reader.get_latest_dict() if imu_reader is not None else {}
                    command_el_with_bounds(
                        motor_2,
                        command_speed,
                        +1,
                        el_min=0.0,
                        el_max=90.0,
                        current_el_deg=(float(imu_cur["elevation_deg"]) if imu_cur else (el_imu_tracker.get_el_deg() if el_imu_tracker is not None else None)),
                    )
                elif key in ("\x1b[B", "s", "S"):
                    imu_cur = imu_reader.get_latest_dict() if imu_reader is not None else {}
                    command_el_with_bounds(
                        motor_2,
                        command_speed,
                        -1,
                        el_min=0.0,
                        el_max=90.0,
                        current_el_deg=(float(imu_cur["elevation_deg"]) if imu_cur else (el_imu_tracker.get_el_deg() if el_imu_tracker is not None else None)),
                    )
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
                print("speed <sps> | micro <1|2|4|8|16>")
                print("offset az <deg> | offset el <deg> | cal az <deg> | cal el <deg>")
                print("limit az <min> <max> | limit el <min> <max>")
                print("obs <lat> <lon> <alt_m> | tle search <text> | tle select <idx> | track on|off")
                print("imu status | imu decl <deg> | imu gyrocal <sec> | imu magcal start|end | imu zero")
                print("elimu status | elimu zero [sec] | elimu gyrocal [sec]")
            elif c == "status":
                print_status()
            elif c == "imu" and len(p) >= 2 and p[1].lower() == "status":
                if imu_reader is None:
                    print("IMU not enabled")
                else:
                    print(imu_reader.get_status())
                    print(imu_reader.get_latest_dict())
            elif c == "imu" and len(p) == 3 and p[1].lower() == "decl":
                if imu_reader is None:
                    print("IMU not enabled")
                else:
                    imu_reader.set_declination(float(p[2]))
                    print(f"IMU declination -> {float(p[2]):.3f} deg")
            elif c == "imu" and len(p) == 3 and p[1].lower() == "gyrocal":
                if imu_reader is None:
                    print("IMU not enabled")
                else:
                    imu_reader.calibrate_gyro_bias(float(p[2]))
                    print("IMU gyro bias calibration done")
            elif c == "imu" and len(p) == 3 and p[1].lower() == "magcal":
                if imu_reader is None:
                    print("IMU not enabled")
                elif p[2].lower() == "start":
                    imu_reader.begin_field_calibration()
                    print("IMU magnetic calibration started")
                elif p[2].lower() == "end":
                    imu_reader.end_field_calibration()
                    print("IMU magnetic calibration ended")
                else:
                    raise ValueError("imu magcal expects start|end")
            elif c == "imu" and len(p) >= 2 and p[1].lower() == "zero":
                if imu_reader is None:
                    print("IMU not enabled")
                else:
                    imu_reader.reset_zero_point()
                    print("IMU absolute elevation zero-point reset done")
            elif c == "elimu" and len(p) >= 2 and p[1].lower() == "status":
                if el_imu_tracker is None:
                    print("EL IMU tracker not enabled")
                else:
                    print(el_imu_tracker.get_status())
                    print(f"el_deg={el_imu_tracker.get_el_deg()}")
            elif c == "elimu" and len(p) >= 2 and p[1].lower() == "zero":
                if el_imu_tracker is None:
                    print("EL IMU tracker not enabled")
                else:
                    sec = 1.5 if len(p) < 3 else float(p[2])
                    el_imu_tracker.calibrate_level_zero(sec)
                    st = el_imu_tracker.get_status()
                    if st.get("absolute_gravity_mode", False):
                        print("EL absolute gravity mode aktif: zero calibration diabaikan")
                    else:
                        print("EL IMU zero calibration done")
            elif c == "elimu" and len(p) >= 2 and p[1].lower() == "gyrocal":
                if el_imu_tracker is None:
                    print("EL IMU tracker not enabled")
                else:
                    sec = 1.5 if len(p) < 3 else float(p[2])
                    el_imu_tracker.calibrate_gyro_bias(sec)
                    print("EL IMU gyro bias calibration done")
            elif c == "az+":
                tracking_enabled = False; motor_1.set_target_speed(abs(command_speed))
            elif c == "az-":
                tracking_enabled = False; motor_1.set_target_speed(-abs(command_speed))
            elif c == "el+":
                tracking_enabled = False
                imu_cur = imu_reader.get_latest_dict() if imu_reader is not None else {}
                command_el_with_bounds(
                    motor_2,
                    command_speed,
                    +1,
                    el_min=0.0,
                    el_max=90.0,
                    current_el_deg=(float(imu_cur["elevation_deg"]) if imu_cur else (el_imu_tracker.get_el_deg() if el_imu_tracker is not None else None)),
                )
            elif c == "el-":
                tracking_enabled = False
                imu_cur = imu_reader.get_latest_dict() if imu_reader is not None else {}
                command_el_with_bounds(
                    motor_2,
                    command_speed,
                    -1,
                    el_min=0.0,
                    el_max=90.0,
                    current_el_deg=(float(imu_cur["elevation_deg"]) if imu_cur else (el_imu_tracker.get_el_deg() if el_imu_tracker is not None else None)),
                )
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
                # Backward compatibility: offset <deg> = AZ offset.
                cfg_m1.az_offset_deg = float(p[1])
            elif c == "offset" and len(p) == 3 and p[1].lower() == "az":
                cfg_m1.az_offset_deg = float(p[2])
            elif c == "offset" and len(p) == 3 and p[1].lower() == "el":
                cfg_m2.el_offset_deg = float(p[2])
            elif c == "cal" and len(p) == 3 and p[1].lower() == "az":
                motor_1.set_position_deg(float(p[2]))
            elif c == "cal" and len(p) == 3 and p[1].lower() == "el":
                motor_2.set_position_deg(float(p[2]))
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


def run_tracking_regression_tests() -> bool:
    """
    Regression sederhana non-GUI untuk memastikan command tracking menggerakkan AZ/EL.
    """
    cfg_az = StepperConfig(steps_per_rev=200, microstep=2, max_speed_sps=2200.0, accel_sps2=3000.0, soft_limit_min_deg=None, soft_limit_max_deg=None)
    cfg_el = StepperConfig(steps_per_rev=200, microstep=2, max_speed_sps=2200.0, accel_sps2=3000.0, soft_limit_min_deg=0.0, soft_limit_max_deg=90.0)
    m1 = SimStepper(cfg_az, "AZ_TEST")
    m2 = SimStepper(cfg_el, "EL_TEST")

    def az_err(target_deg, current_deg):
        e = target_deg - current_deg
        if e > 180.0:
            e -= 360.0
        elif e < -180.0:
            e += 360.0
        return e

    scenarios = [
        (10.0, 5.0),
        (120.0, 30.0),
        (220.0, 60.0),
        (315.0, 10.0),
        (45.0, 80.0),
    ]
    passed = True
    try:
        for i, (taz, tel) in enumerate(scenarios, start=1):
            st1_init = m1.get_status()
            st2_init = m2.get_status()
            az_err_init = abs(az_err(taz, st1_init["position_deg"] % 360.0))
            el_err_init = abs(tel - st2_init["position_deg"])
            t0 = time.time()
            while time.time() - t0 < 2.4:
                st1 = m1.get_status()
                st2 = m2.get_status()
                eaz = az_err(taz, st1["position_deg"] % 360.0)
                eel = tel - st2["position_deg"]
                m1.set_target_speed(max(-cfg_az.max_speed_sps, min(cfg_az.max_speed_sps, 70.0 * eaz)))
                m2.set_target_speed(max(-cfg_el.max_speed_sps, min(cfg_el.max_speed_sps, 70.0 * eel)))
                time.sleep(0.01)
            st1 = m1.get_status()
            st2 = m2.get_status()
            az_err_final = abs(az_err(taz, st1["position_deg"] % 360.0))
            el_err_final = abs(tel - st2["position_deg"])
            az_ok = (az_err_final <= az_err_init) and ((az_err_init - az_err_final) >= 5.0 or az_err_final < 15.0)
            el_ok = (el_err_final <= el_err_init) and ((el_err_init - el_err_final) >= 5.0 or el_err_final < 8.0)
            print(
                f"[selftest] scenario#{i} target=({taz:.1f},{tel:.1f}) pos=({st1['position_deg']%360.0:.1f},{st2['position_deg']:.1f}) "
                f"errAZ={az_err_init:.1f}->{az_err_final:.1f} errEL={el_err_init:.1f}->{el_err_final:.1f} az_ok={az_ok} el_ok={el_ok}"
            )
            passed = passed and az_ok and el_ok
        return passed
    finally:
        m1.close()
        m2.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", help="Jalankan mode simulasi UI (tanpa GPIO)")
    parser.add_argument("--gui", action="store_true", help="Jalankan GUI (bisa hardware atau simulasi)")
    parser.add_argument("--sim-gui", action="store_true", help="Jalankan simulasi GUI (window)")
    parser.add_argument("--cli", action="store_true", help="Run interactive CLI mode")
    parser.add_argument("--imu", action="store_true", help="Aktifkan akuisisi WT901 IMU (hardware mode)")
    parser.add_argument("--imu-interface", default="uart", choices=["uart", "i2c"], help="Interface WT901")
    parser.add_argument("--imu-port", default="", help="Serial port WT901, default auto")
    parser.add_argument("--imu-baud", type=int, default=9600, help="Baudrate WT901 UART")
    parser.add_argument("--imu-addr", type=lambda x: int(x, 0), default=0x50, help="WT901 device address (e.g. 0x50)")
    parser.add_argument("--imu-rate-hz", type=float, default=50.0, help="Sampling rate WT901 (>=50Hz)")
    parser.add_argument("--imu-declination", type=float, default=0.0, help="Magnetic declination compensation (deg)")
    parser.add_argument("--imu-log-level", default="INFO", help="WT901 log level (DEBUG/INFO/WARN/ERROR)")
    parser.add_argument("--imu-azel-hold", action="store_true", help="Aktifkan closed-loop hold AZ/EL berbasis IMU")
    parser.add_argument("--imu-control-rate-hz", type=float, default=50.0, help="Control-loop rate untuk IMU hold")
    parser.add_argument("--imu-dropout-timeout", type=float, default=0.25, help="Timeout dropout IMU (detik)")
    parser.add_argument("--imu-nudge-deg", type=float, default=0.5, help="Besar nudge keyboard per langkah (deg)")
    parser.add_argument("--imu-log-path", default="", help="Path CSV log cmd-vs-actual AZ/EL (10Hz)")
    parser.add_argument("--el-imu-only", action="store_true", help="Gunakan estimasi EL IMU secara eksklusif untuk tracking/control")
    parser.add_argument("--el-imu-rate-hz", type=float, default=100.0, help="Update rate EL IMU fusion (>=100Hz)")
    parser.add_argument("--el-imu-alpha", type=float, default=0.98, help="Complementary filter alpha EL fusion")
    parser.add_argument("--el-imu-weight", type=float, default=0.18, help="Weight koreksi sudut pitch IMU pada EL fusion")
    parser.add_argument("--el-imu-debug", action="store_true", help="Aktifkan debug log EL fusion")
    parser.add_argument("--el-gear-ratio", type=float, default=1.0, help="Rasio gear EL (motor:output), contoh 50 berarti 50:1")
    parser.add_argument("--selftest-tracking", action="store_true", help="Jalankan 5 skenario regression tracking non-GUI")
    args = parser.parse_args()

    if args.selftest_tracking:
        ok = run_tracking_regression_tests()
        print(f"[selftest] result={'PASS' if ok else 'FAIL'}")
        return

    logging.basicConfig(
        level=getattr(logging, str(args.imu_log_level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    # Motor 1 (sesuai mapping user)
    cfg_m1 = StepperConfig(
        step_pin=17,  # PUL+
        dir_pin=27,   # DIR+
        en_pin=22,    # EN+
        steps_per_rev=200,
        microstep=2,   # sesuai konfigurasi: microstep 2/A
        max_speed_sps=2200.0,
        accel_sps2=3000.0,
        # Default AZ dibuka full-rotation; batasi via perintah "limit az" jika diperlukan.
        soft_limit_min_deg=None,
        soft_limit_max_deg=None,
        az_wrap_enabled=True,
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
        axis_gear_ratio=max(1e-9, float(args.el_gear_ratio)),
    )
    logging.getLogger("motorPID.cfg").info(
        "EL conversion: steps_per_rev=%d microstep=%d gear=%.4f => %.4f microsteps/deg",
        cfg_m2.steps_per_rev,
        cfg_m2.microstep,
        cfg_m2.axis_gear_ratio,
        (cfg_m2.steps_per_rev * cfg_m2.microstep * cfg_m2.axis_gear_ratio) / 360.0,
    )

    # --sim-gui tetap memaksa mode simulator (kompatibilitas).
    # --gui hanya mengaktifkan window; mode motor ditentukan oleh use_sim.
    use_sim = args.sim or args.sim_gui or (not GPIO_AVAILABLE)
    use_gui = args.gui or args.sim_gui
    if use_sim:
        motor_1 = SimStepper(cfg_m1, "AZ")
        motor_2 = SimStepper(cfg_m2, "EL")
    else:
        motor_1 = TB6600Stepper(cfg_m1)
        motor_2 = TB6600Stepper(cfg_m2)
    imu_reader = None
    if args.imu and not use_sim:
        imu_cfg = WT901Config(
            enabled=True,
            interface=args.imu_interface,
            port_name=args.imu_port,
            baud=int(args.imu_baud),
            address=int(args.imu_addr),
            sample_rate_hz=max(50.0, float(args.imu_rate_hz)),
            declination_deg=float(args.imu_declination),
            log_level=str(args.imu_log_level).upper(),
        )
        try:
            imu_reader = WT901Reader(imu_cfg)
            imu_reader.initialize()
        except Exception as exc:
            logging.getLogger("motorPID.wt901").error("IMU disabled due to init failure: %s", exc)
            imu_reader = None
    imu_hold_ctrl = None
    if args.imu_azel_hold and not use_sim:
        if imu_reader is None:
            logging.getLogger("motorPID.imu_hold").error("IMU AZ/EL hold but IMU is not available.")
        else:
            ctrl_cfg = ImuAzElHoldConfig(
                control_rate_hz=max(10.0, float(args.imu_control_rate_hz)),
                log_rate_hz=10.0,
                dropout_timeout_s=max(0.05, float(args.imu_dropout_timeout)),
                nudge_step_deg=max(0.05, float(args.imu_nudge_deg)),
                log_path=str(args.imu_log_path).strip(),
            )
            imu_hold_ctrl = ImuAzElPositionController(ctrl_cfg, imu_reader, motor_1, motor_2, cfg_m1, cfg_m2)
            # Tunggu sample awal agar zeroing valid.
            for _ in range(40):
                if imu_reader.get_latest_dict():
                    break
                time.sleep(0.05)
            if imu_reader.get_latest_dict():
                imu_hold_ctrl.calibrate_zero_reference()
            else:
                logging.getLogger("motorPID.imu_hold").warning("IMU sample belum ada, hold akan menunggu data.")
    el_imu_tracker = None
    # EL tracking/control source hanya aktif saat diminta eksplisit.
    # Ini mencegah EL keyboard terasa "stuck" jika EL IMU startup belum valid.
    enable_el_imu = args.el_imu_only and args.imu and not use_sim
    if enable_el_imu and imu_reader is not None:
        el_cfg = ELImuFusionConfig(
            update_rate_hz=max(100.0, float(args.el_imu_rate_hz)),
            complementary_alpha=max(0.7, min(0.999, float(args.el_imu_alpha))),
            imu_pitch_weight=max(0.02, min(0.8, float(args.el_imu_weight))),
            el_min_deg=0.0,
            el_max_deg=90.0,
            debug_log=bool(args.el_imu_debug),
        )
        el_imu_tracker = ELImuFusionTracker(el_cfg, imu_reader)
        el_imu_tracker.start()
        # EL position determination no longer relies on step count boundaries.
        cfg_m2.soft_limit_min_deg = None
        cfg_m2.soft_limit_max_deg = None

    command_speed = 600.0
    last_report = 0.0

    try:
        if args.cli:
            run_cli_mode(motor_1, motor_2, cfg_m1, cfg_m2, imu_reader=imu_reader, el_imu_tracker=el_imu_tracker)
        elif use_gui:
            if not TK_AVAILABLE:
                raise RuntimeError("tkinter tidak tersedia. Install tkinter atau jalankan tanpa --gui/--sim-gui.")
            if not os.environ.get("DISPLAY"):
                raise RuntimeError("DISPLAY tidak terdeteksi. Jalankan dari desktop Raspberry Pi atau pakai X11 forwarding.")
            app = SimGuiApp(
                motor_1,
                motor_2,
                cfg_m1,
                cfg_m2,
                imu_reader=imu_reader,
                el_imu_tracker=el_imu_tracker,
                imu_hold_ctrl=imu_hold_ctrl,
            )
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
                        imu_cur = imu_reader.get_latest_dict() if imu_reader is not None else {}
                        command_el_with_bounds(
                            motor_2,
                            command_speed,
                            +1,
                            el_min=0.0,
                            el_max=90.0,
                            current_el_deg=(float(imu_cur["elevation_deg"]) if imu_cur else (el_imu_tracker.get_el_deg() if el_imu_tracker is not None else None)),
                        )
                    elif key in ("\x1b[B", "s", "S"):
                        imu_cur = imu_reader.get_latest_dict() if imu_reader is not None else {}
                        command_el_with_bounds(
                            motor_2,
                            command_speed,
                            -1,
                            el_min=0.0,
                            el_max=90.0,
                            current_el_deg=(float(imu_cur["elevation_deg"]) if imu_cur else (el_imu_tracker.get_el_deg() if el_imu_tracker is not None else None)),
                        )
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
            if imu_hold_ctrl is not None:
                run_imu_azel_keyboard_mode(motor_1, motor_2, cfg_m1, cfg_m2, imu_reader, imu_hold_ctrl)
                return
            print(
                "\n=== DUAL TB6600 KEYBOARD STEPPER CONTROL ===\n"
                "Motor 1 (GPIO17/27/22): Arrow Left/Right atau A/D\n"
                "Motor 2 (GPIO23/24/25): Arrow Up/Down  atau W/S\n"
                "AZ default: full rotation (tanpa soft-limit)\n"
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
                    imu = imu_reader.get_latest_dict() if imu_reader is not None else {}
                    el_imu = el_imu_tracker.get_el_deg() if el_imu_tracker is not None else None
                    if imu:
                        az_raw = parse_float_safe(imu.get("azimuth_deg"), None)
                        az_pos = (az_raw % 360.0) if az_raw is not None else (st1["position_deg"] % 360.0)
                        pose_tag = "IMU_AZ/IMU_EL"
                    else:
                        az_pos = st1["position_deg"] % 360.0
                        pose_tag = "IMU_EL_ONLY" if el_imu is not None else "MOTOR"
                    el_pos = parse_float_safe(imu.get("elevation_deg"), None) if imu else None
                    if el_pos is None:
                        el_pos = float(el_imu) if el_imu is not None else st2["position_deg"]
                    fault1 = f" F1={st1['fault_msg']}" if st1["fault_latched"] else ""
                    fault2 = f" F2={st2['fault_msg']}" if st2["fault_latched"] else ""
                    imu_txt = ""
                    if imu:
                        imu_txt = f" | HDG={parse_float_safe(imu.get('compass_deg'), 0.0):6.2f}"
                    sys.stdout.write(
                        f"\rM1 POS={az_pos:7.2f} SPD={st1['current_speed_sps']:7.1f} "
                        f"| M2 POS={el_pos:7.2f} SPD={st2['current_speed_sps']:7.1f} "
                        f"| SRC={pose_tag} MS={st1['microstep']:2d}{fault1}{fault2}{imu_txt}       "
                    )
                    sys.stdout.flush()
                    last_report = now

                key = get_key()

                if key in ("\x1b[C", "d", "D"):
                    motor_1.set_target_speed(abs(command_speed))
                elif key in ("\x1b[D", "a", "A"):
                    motor_1.set_target_speed(-abs(command_speed))
                elif key in ("\x1b[A", "w", "W"):
                    imu_cur = imu_reader.get_latest_dict() if imu_reader is not None else {}
                    command_el_with_bounds(
                        motor_2,
                        command_speed,
                        +1,
                        el_min=0.0,
                        el_max=90.0,
                        current_el_deg=(float(imu_cur["elevation_deg"]) if imu_cur else (el_imu_tracker.get_el_deg() if el_imu_tracker is not None else None)),
                    )
                elif key in ("\x1b[B", "s", "S"):
                    imu_cur = imu_reader.get_latest_dict() if imu_reader is not None else {}
                    command_el_with_bounds(
                        motor_2,
                        command_speed,
                        -1,
                        el_min=0.0,
                        el_max=90.0,
                        current_el_deg=(float(imu_cur["elevation_deg"]) if imu_cur else (el_imu_tracker.get_el_deg() if el_imu_tracker is not None else None)),
                    )
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
        if imu_hold_ctrl is not None:
            imu_hold_ctrl.close()
        if el_imu_tracker is not None:
            el_imu_tracker.close()
        if imu_reader is not None:
            imu_reader.close()
        motor_1.close()
        motor_2.close()
        if GPIO_AVAILABLE and not use_sim:
            GPIO.cleanup()
        print("GPIO cleaned up.")


if __name__ == "__main__":
    main()
