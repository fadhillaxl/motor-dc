# coding: UTF-8
import argparse
import configparser
import logging
import math
import os
import platform
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except Exception as exc:
    GPIO = None
    GPIO_AVAILABLE = False
    GPIO_IMPORT_ERROR = str(exc)

# ================= WT901 SDK =================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.abspath(os.path.join(BASE_DIR, "..", "Python-SDK-WT901C485", "chs"))
sys.path.insert(0, SDK_CHS)

# ================= LIMIT =================
MIN_AZ, MAX_AZ = -180.0, 180.0
MIN_EL, MAX_EL = 0.0, 90.0

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
LOGGER = logging.getLogger("AdaptivePID")
DEFAULT_CONFIG_PATH = os.path.join(BASE_DIR, "config-stepper.conf")

DEFAULT_CONFIG_TEMPLATE = """# AdaptivePID Stepper Configuration
# Edit values below, then run: python AdaptivePID.py --config config-stepper.conf

[stepper_az]
step_pin = 17
dir_pin = 27
en_pin = 22
microstep = 2
max_speed_sps = 2200.0
accel_sps2 = 3000.0
soft_limit_min_deg = 0.0
soft_limit_max_deg = 360.0
limit_min_pin =
limit_max_pin =
limit_active_low = true
dir_sign = 1
offset_deg = 0.0

[stepper_el]
step_pin = 23
dir_pin = 24
en_pin = 25
microstep = 2
max_speed_sps = 2200.0
accel_sps2 = 3000.0
soft_limit_min_deg = 0.0
soft_limit_max_deg = 90.0
limit_min_pin =
limit_max_pin =
limit_active_low = true
dir_sign = 1
offset_deg = 0.0

[limit_switch]
enabled = true
use_hw_switch = true
safety_margin_deg = 0.5
az_min_deg = 0.0
az_max_deg = 360.0
el_min_deg = 0.0
el_max_deg = 90.0
"""


def clamp(v, a, b):
    return max(a, min(b, v))


def az_error_shortest(t, c):
    e = t - c
    return e - 360 if e > 180 else e + 360 if e < -180 else e


def gravity_comp(el):
    return 120.0 * (el / 90.0)


def is_finite_number(v):
    try:
        return math.isfinite(float(v))
    except Exception:
        return False


def print_status(s):
    sys.stdout.write("\r" + s + " " * 10)
    sys.stdout.flush()


def _parse_optional_int(v: str) -> Optional[int]:
    s = str(v).strip()
    if s == "":
        return None
    return int(s)


def _as_bool(parser: configparser.ConfigParser, section: str, key: str, fallback: bool) -> bool:
    if parser.has_option(section, key):
        return parser.getboolean(section, key)
    return fallback


def ensure_default_config(path: str):
    if os.path.exists(path):
        return
    with open(path, "w", encoding="utf-8") as f:
        f.write(DEFAULT_CONFIG_TEMPLATE)
    LOGGER.info("Created default config: %s", path)


def load_config_stepper(path: str):
    ensure_default_config(path)
    cp = configparser.ConfigParser()
    cp.read(path, encoding="utf-8")

    if "stepper_az" not in cp or "stepper_el" not in cp or "limit_switch" not in cp:
        raise ValueError("config-stepper.conf must contain [stepper_az], [stepper_el], and [limit_switch].")

    saz = cp["stepper_az"]
    sel = cp["stepper_el"]
    lsw = cp["limit_switch"]

    cfg_az = StepperConfig(
        step_pin=int(saz.get("step_pin", "17")),
        dir_pin=int(saz.get("dir_pin", "27")),
        en_pin=int(saz.get("en_pin", "22")),
        microstep=int(saz.get("microstep", "2")),
        max_speed_sps=float(saz.get("max_speed_sps", "2200.0")),
        accel_sps2=float(saz.get("accel_sps2", "3000.0")),
        soft_limit_min_deg=float(saz.get("soft_limit_min_deg", "0.0")),
        soft_limit_max_deg=float(saz.get("soft_limit_max_deg", "360.0")),
        limit_min_pin=_parse_optional_int(saz.get("limit_min_pin", "")),
        limit_max_pin=_parse_optional_int(saz.get("limit_max_pin", "")),
        limit_active_low=_as_bool(cp, "stepper_az", "limit_active_low", True),
    )
    cfg_el = StepperConfig(
        step_pin=int(sel.get("step_pin", "23")),
        dir_pin=int(sel.get("dir_pin", "24")),
        en_pin=int(sel.get("en_pin", "25")),
        microstep=int(sel.get("microstep", "2")),
        max_speed_sps=float(sel.get("max_speed_sps", "2200.0")),
        accel_sps2=float(sel.get("accel_sps2", "3000.0")),
        soft_limit_min_deg=float(sel.get("soft_limit_min_deg", "0.0")),
        soft_limit_max_deg=float(sel.get("soft_limit_max_deg", "90.0")),
        limit_min_pin=_parse_optional_int(sel.get("limit_min_pin", "")),
        limit_max_pin=_parse_optional_int(sel.get("limit_max_pin", "")),
        limit_active_low=_as_bool(cp, "stepper_el", "limit_active_low", True),
    )

    ls_cfg = LimitSwitchConfig(
        enabled=_as_bool(cp, "limit_switch", "enabled", True),
        az_min_deg=float(lsw.get("az_min_deg", "0.0")),
        az_max_deg=float(lsw.get("az_max_deg", "360.0")),
        el_min_deg=float(lsw.get("el_min_deg", "0.0")),
        el_max_deg=float(lsw.get("el_max_deg", "90.0")),
        az_offset_deg=float(saz.get("offset_deg", "0.0")),
        el_offset_deg=float(sel.get("offset_deg", "0.0")),
        az_dir_sign=int(saz.get("dir_sign", "1")),
        el_dir_sign=int(sel.get("dir_sign", "1")),
        safety_margin_deg=float(lsw.get("safety_margin_deg", "0.5")),
        use_hw_switch=_as_bool(cp, "limit_switch", "use_hw_switch", True),
    )
    ls_cfg.validate()
    return cfg_az, cfg_el, ls_cfg


def normalize_360(deg: float) -> float:
    v = float(deg) % 360.0
    if v < 0:
        v += 360.0
    return v


def az_range_contains(az_deg: float, min_deg: float, max_deg: float) -> bool:
    """Range azimuth mendukung wrap-around (contoh 350..20)."""
    a = normalize_360(az_deg)
    mn = normalize_360(min_deg)
    mx = normalize_360(max_deg)
    if mn <= mx:
        return mn <= a <= mx
    return a >= mn or a <= mx


def validate_limit_range(min_deg: float, max_deg: float, axis: str):
    if not (is_finite_number(min_deg) and is_finite_number(max_deg)):
        raise ValueError(f"Range {axis} harus angka valid.")
    if axis == "az":
        if min_deg < 0 or min_deg > 360 or max_deg < 0 or max_deg > 360:
            raise ValueError("Range AZ harus dalam 0..360.")
        return
    if min_deg < -90 or min_deg > 90 or max_deg < -90 or max_deg > 90:
        raise ValueError("Range EL harus dalam -90..90.")
    if min_deg >= max_deg:
        raise ValueError("EL min harus lebih kecil dari EL max.")


@dataclass
class LimitSwitchConfig:
    enabled: bool = True
    az_min_deg: float = 0.0
    az_max_deg: float = 360.0
    el_min_deg: float = 0.0
    el_max_deg: float = 90.0
    az_offset_deg: float = 0.0
    el_offset_deg: float = 0.0
    az_dir_sign: int = 1
    el_dir_sign: int = 1
    safety_margin_deg: float = 0.5
    home_az_deg: float = 0.0
    home_el_deg: float = 0.0
    use_hw_switch: bool = True

    def validate(self):
        validate_limit_range(self.az_min_deg, self.az_max_deg, "az")
        validate_limit_range(self.el_min_deg, self.el_max_deg, "el")
        if self.az_dir_sign not in (-1, 1) or self.el_dir_sign not in (-1, 1):
            raise ValueError("Arah rotasi harus -1 atau 1.")
        if self.safety_margin_deg < 0 or self.safety_margin_deg > 20:
            raise ValueError("Safety margin harus pada rentang 0..20 derajat.")


# ================= ADAPTIVE PID (dipertahankan) =================
class AdaptivePID:
    def __init__(self):
        self.i = 0.0
        self.last_err = 0.0
        self.last_t = time.time()

    def gains(self, err):
        e = abs(err)
        if e > 10:
            return 9.0, 0.02, 1.2
        if e > 3:
            return 8.0, 0.05, 2.0
        if e > 0.8:
            return 6.0, 0.03, 3.0
        return 4.0, 0.0, 4.0

    def compute(self, err):
        now = time.time()
        dt = now - self.last_t
        self.last_t = now
        if dt <= 0:
            return 0.0

        kp, ki, kd = self.gains(err)
        self.i += err * dt
        self.i = clamp(self.i, -40.0, 40.0)
        d = (err - self.last_err) / dt
        self.last_err = err
        return kp * err + ki * self.i + kd * d


@dataclass
class StepperConfig:
    step_pin: int
    dir_pin: int
    en_pin: int
    steps_per_rev: int = 200
    microstep: int = 2
    max_speed_sps: float = 2200.0
    accel_sps2: float = 3000.0
    pulse_width_us: int = 8
    en_active_high: bool = False
    dir_active_high: bool = True
    step_active_high: bool = True
    soft_limit_min_deg: Optional[float] = None
    soft_limit_max_deg: Optional[float] = None
    limit_min_pin: Optional[int] = None
    limit_max_pin: Optional[int] = None
    limit_active_low: bool = True


class TB6600Stepper:
    SUPPORTED_MICROSTEPS = (1, 2, 4, 8, 16)

    def __init__(self, cfg: StepperConfig):
        if not GPIO_AVAILABLE:
            raise RuntimeError(
                f"RPi.GPIO tidak tersedia ({GPIO_IMPORT_ERROR}). "
                "Gunakan --sim untuk mode simulasi."
            )
        self.cfg = cfg
        self._lock = threading.Lock()
        self._run = True
        self._fault_msg = ""
        self._target_speed_sps = 0.0
        self._current_speed_sps = 0.0
        self._position_full_steps = 0.0

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.cfg.step_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.cfg.dir_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.cfg.en_pin, GPIO.OUT, initial=GPIO.LOW)
        if self.cfg.limit_min_pin is not None:
            GPIO.setup(self.cfg.limit_min_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if self.cfg.limit_max_pin is not None:
            GPIO.setup(self.cfg.limit_max_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.enable_driver(True)
        self._thread = threading.Thread(target=self._motion_loop, daemon=True)
        self._thread.start()

    def _set_output(self, pin, state):
        GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)

    def _safe_input(self, pin: int) -> int:
        try:
            return GPIO.input(pin)
        except Exception:
            return GPIO.LOW

    def _is_active(self, raw: int) -> bool:
        return (raw == GPIO.LOW) if self.cfg.limit_active_low else (raw == GPIO.HIGH)

    def enable_driver(self, enabled: bool):
        if self.cfg.en_active_high:
            self._set_output(self.cfg.en_pin, enabled)
        else:
            self._set_output(self.cfg.en_pin, not enabled)

    def set_target_speed(self, speed_sps: float):
        with self._lock:
            lim = max(0.0, float(self.cfg.max_speed_sps))
            self._target_speed_sps = clamp(float(speed_sps), -lim, lim)

    def stop_smooth(self):
        with self._lock:
            self._target_speed_sps = 0.0

    def get_status(self):
        with self._lock:
            return {
                "target_speed_sps": self._target_speed_sps,
                "current_speed_sps": self._current_speed_sps,
                "position_deg": (self._position_full_steps / self.cfg.steps_per_rev) * 360.0,
                "fault_msg": self._fault_msg,
                "limit_min_active": self.get_limit_inputs_status()["min_active"],
                "limit_max_active": self.get_limit_inputs_status()["max_active"],
            }

    def get_position_steps(self):
        with self._lock:
            return self._position_full_steps

    def set_position_deg(self, deg: float):
        with self._lock:
            self._position_full_steps = (float(deg) / 360.0) * self.cfg.steps_per_rev

    def get_limit_inputs_status(self):
        min_active = False
        max_active = False
        if self.cfg.limit_min_pin is not None:
            min_active = self._is_active(self._safe_input(self.cfg.limit_min_pin))
        if self.cfg.limit_max_pin is not None:
            max_active = self._is_active(self._safe_input(self.cfg.limit_max_pin))
        return {"min_active": bool(min_active), "max_active": bool(max_active)}

    def _set_direction(self, cw: bool):
        out = cw if self.cfg.dir_active_high else (not cw)
        self._set_output(self.cfg.dir_pin, out)

    def _pulse_step(self):
        hi = self.cfg.step_active_high
        lo = not hi
        self._set_output(self.cfg.step_pin, hi)
        time.sleep(self.cfg.pulse_width_us / 1_000_000.0)
        self._set_output(self.cfg.step_pin, lo)

    def _soft_limit_reached(self, next_deg):
        mn = self.cfg.soft_limit_min_deg
        mx = self.cfg.soft_limit_max_deg
        if mn is not None and next_deg < mn:
            return True
        if mx is not None and next_deg > mx:
            return True
        return False

    def _motion_loop(self):
        last_t = time.perf_counter()
        next_pulse_t = last_t

        while self._run:
            now = time.perf_counter()
            dt = now - last_t
            last_t = now

            with self._lock:
                a = max(1.0, float(self.cfg.accel_sps2))
                delta = a * dt
                if self._current_speed_sps < self._target_speed_sps:
                    self._current_speed_sps = min(self._current_speed_sps + delta, self._target_speed_sps)
                elif self._current_speed_sps > self._target_speed_sps:
                    self._current_speed_sps = max(self._current_speed_sps - delta, self._target_speed_sps)
                spd = self._current_speed_sps
                microstep = max(1, int(self.cfg.microstep))

            if abs(spd) < 1e-3:
                time.sleep(0.001)
                continue

            cw = spd > 0.0
            self._set_direction(cw)
            interval = 1.0 / abs(spd)
            if now < next_pulse_t:
                time.sleep(min(0.001, next_pulse_t - now))
                continue

            step_delta_full = (1.0 / float(microstep)) * (1.0 if cw else -1.0)
            next_deg = ((self.get_position_steps() + step_delta_full) / self.cfg.steps_per_rev) * 360.0
            hw = self.get_limit_inputs_status()
            if (cw and hw["max_active"]) or ((not cw) and hw["min_active"]):
                with self._lock:
                    self._current_speed_sps = 0.0
                    self._target_speed_sps = 0.0
                    self._fault_msg = "Hardware limit switch triggered"
                continue
            if self._soft_limit_reached(next_deg):
                with self._lock:
                    self._current_speed_sps = 0.0
                    self._target_speed_sps = 0.0
                    self._fault_msg = "Soft limit reached"
                continue

            try:
                self._pulse_step()
                with self._lock:
                    self._position_full_steps += step_delta_full
            except Exception as exc:
                with self._lock:
                    self._current_speed_sps = 0.0
                    self._target_speed_sps = 0.0
                    self._fault_msg = f"GPIO pulse failed: {exc}"

            next_pulse_t = now + interval

    def close(self):
        self._run = False
        self.stop_smooth()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.enable_driver(False)


class SimStepper:
    def __init__(self, cfg: StepperConfig, name="SIM"):
        self.cfg = cfg
        self.name = name
        self._lock = threading.Lock()
        self._run = True
        self._fault_msg = ""
        self._target_speed_sps = 0.0
        self._current_speed_sps = 0.0
        self._position_full_steps = 0.0
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_target_speed(self, speed_sps):
        with self._lock:
            lim = max(0.0, float(self.cfg.max_speed_sps))
            self._target_speed_sps = clamp(float(speed_sps), -lim, lim)

    def stop_smooth(self):
        with self._lock:
            self._target_speed_sps = 0.0

    def get_status(self):
        with self._lock:
            return {
                "target_speed_sps": self._target_speed_sps,
                "current_speed_sps": self._current_speed_sps,
                "position_deg": (self._position_full_steps / self.cfg.steps_per_rev) * 360.0,
                "fault_msg": self._fault_msg,
            }

    def get_position_steps(self):
        with self._lock:
            return self._position_full_steps

    def set_position_deg(self, deg: float):
        with self._lock:
            self._position_full_steps = (float(deg) / 360.0) * self.cfg.steps_per_rev

    def get_limit_inputs_status(self):
        return {"min_active": False, "max_active": False}

    def _soft_limit_reached(self, next_deg):
        mn = self.cfg.soft_limit_min_deg
        mx = self.cfg.soft_limit_max_deg
        if mn is not None and next_deg < mn:
            return True
        if mx is not None and next_deg > mx:
            return True
        return False

    def _loop(self):
        last_t = time.perf_counter()
        while self._run:
            now = time.perf_counter()
            dt = now - last_t
            last_t = now
            with self._lock:
                a = max(1.0, float(self.cfg.accel_sps2))
                delta = a * dt
                if self._current_speed_sps < self._target_speed_sps:
                    self._current_speed_sps = min(self._current_speed_sps + delta, self._target_speed_sps)
                elif self._current_speed_sps > self._target_speed_sps:
                    self._current_speed_sps = max(self._current_speed_sps - delta, self._target_speed_sps)
                spd = self._current_speed_sps
                ms = max(1, int(self.cfg.microstep))

            if abs(spd) > 1e-3:
                step_delta_full = (spd * dt) / float(ms)
                next_deg = ((self.get_position_steps() + step_delta_full) / self.cfg.steps_per_rev) * 360.0
                if self._soft_limit_reached(next_deg):
                    with self._lock:
                        self._current_speed_sps = 0.0
                        self._target_speed_sps = 0.0
                        self._fault_msg = "Soft limit reached"
                else:
                    with self._lock:
                        self._position_full_steps += step_delta_full
            time.sleep(0.001)

    def close(self):
        self._run = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


class WT901Reader:
    def __init__(self, port_name=None, baud=9600):
        self.port_name = port_name
        self.baud = baud
        self.dev = None
        self._lock = threading.Lock()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_update_ts = 0.0

    def _read_config(self):
        vals = self.dev.readReg(0x02, 3)
        print("Config 0x02.. :", vals if len(vals) > 0 else "no response")
        vals = self.dev.readReg(0x23, 2)
        print("Config 0x23.. :", vals if len(vals) > 0 else "no response")

    def _on_update(self, dev):
        try:
            ax = dev.getDeviceData("angleX")
            ay = dev.getDeviceData("angleY")
            az = dev.getDeviceData("angleZ")
            if not (is_finite_number(ax) and is_finite_number(ay) and is_finite_number(az)):
                return
            with self._lock:
                self.roll = float(ax)
                self.pitch = float(ay)
                self.yaw = float(az)
                self.last_update_ts = time.time()
        except Exception:
            pass

    def _loop(self):
        while True:
            self.dev.readReg(0x30, 41)
            time.sleep(0.01)

    def start(self):
        try:
            import lib.device_model as deviceModel
            from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
            from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver
        except Exception as exc:
            raise RuntimeError(
                "Gagal import SDK WT901. Pastikan dependency terpasang (contoh: pyserial) "
                f"dan path SDK benar. Detail: {exc}"
            ) from exc

        self.dev = deviceModel.DeviceModel(
            "WT901",
            Protocol485Resolver(),
            JY901SDataProcessor(),
            "51_0",
        )
        self.dev.ADDR = 0x50
        if self.port_name:
            self.dev.serialConfig.portName = self.port_name
        else:
            # Default lintas-platform: Windows pakai COM82, selain itu /dev/ttyUSB0
            if platform.system().lower().startswith("win"):
                self.dev.serialConfig.portName = "COM82"
            else:
                self.dev.serialConfig.portName = "/dev/ttyUSB0"
        self.dev.serialConfig.baud = self.baud
        try:
            self.dev.openDevice()
        except Exception as exc:
            port = self.dev.serialConfig.portName
            baud = self.dev.serialConfig.baud
            raise RuntimeError(
                f"Gagal membuka port WT901 ({port}, baud={baud}). "
                "Periksa kabel/port, atau gunakan --imu-port yang sesuai."
            ) from exc
        self._read_config()
        self.dev.dataProcessor.onVarChanged.append(self._on_update)
        threading.Thread(target=self._loop, daemon=True).start()

    def get_angles(self):
        with self._lock:
            return self.roll, self.pitch, self.yaw, self.last_update_ts

    def close(self):
        if self.dev is not None:
            self.dev.closeDevice()


class AdaptiveStepperController:
    def __init__(
        self,
        use_sim=False,
        imu_port=None,
        ls_cfg: Optional[LimitSwitchConfig] = None,
        az_cfg: Optional[StepperConfig] = None,
        el_cfg: Optional[StepperConfig] = None,
    ):
        self.use_sim = use_sim
        self.stop_requested = False
        self._api_lock = threading.Lock()
        self.az_off = 0.0
        self.el_off = 0.0
        self.target_az = 0.0
        self.target_el = 0.0
        self._last_az = 0.0
        self._last_el = 0.0

        self.pid_az = AdaptivePID()
        self.pid_el = AdaptivePID()
        self.ls_cfg = ls_cfg if ls_cfg is not None else LimitSwitchConfig()
        self.ls_fault_msg = ""
        self._last_ls_log = 0.0
        self.imu_ready = False

        cfg_m1 = az_cfg if az_cfg is not None else StepperConfig(step_pin=17, dir_pin=27, en_pin=22)
        cfg_m2 = el_cfg if el_cfg is not None else StepperConfig(step_pin=23, dir_pin=24, en_pin=25)

        if use_sim:
            self.motor_az = SimStepper(cfg_m1, "AZ")
            self.motor_el = SimStepper(cfg_m2, "EL")
        else:
            self.motor_az = TB6600Stepper(cfg_m1)
            self.motor_el = TB6600Stepper(cfg_m2)

        self.imu = WT901Reader(port_name=imu_port)
        try:
            self.imu.start()
            self.imu_ready = True
            LOGGER.info("WT901 connected and running.")
        except Exception as exc:
            if self.use_sim:
                self.imu_ready = False
                LOGGER.warning("WT901 tidak tersedia, fallback ke feedback simulasi motor. Detail: %s", exc)
            else:
                raise

    def _get_feedback_angles(self):
        # Jika IMU tersedia, pakai data WT901.
        if self.imu_ready:
            return self.imu.get_angles()

        # Fallback simulasi: gunakan posisi motor sebagai pseudo-feedback.
        st_az = self.motor_az.get_status()
        st_el = self.motor_el.get_status()
        yaw = st_az["position_deg"]
        if yaw > 180.0:
            yaw -= 360.0
        pitch = st_el["position_deg"]
        roll = 0.0
        return roll, pitch, yaw, time.time()

    def set_target(self, az: float, el: float):
        """API untuk rotctl bridge (gpredict): set target az/el."""
        with self._api_lock:
            self.target_az = clamp(float(az), MIN_AZ, MAX_AZ)
            self.target_el = clamp(float(el), MIN_EL, MAX_EL)

    def get_position(self):
        """API untuk rotctl bridge (gpredict): baca posisi az/el terbaru."""
        with self._api_lock:
            return float(self._last_az), float(self._last_el)

    def stop(self):
        """API untuk rotctl bridge (gpredict): hold posisi saat ini."""
        with self._api_lock:
            hold_az = self._last_az
            hold_el = self._last_el
            self.target_az = hold_az
            self.target_el = hold_el
        self.motor_az.stop_smooth()
        self.motor_el.stop_smooth()

    def _limit_status_summary(self, az_deg_360: float, el_deg: float):
        hw_az = self.motor_az.get_limit_inputs_status()
        hw_el = self.motor_el.get_limit_inputs_status()
        margin = self.ls_cfg.safety_margin_deg

        az_ok = az_range_contains(
            az_deg_360,
            self.ls_cfg.az_min_deg + margin,
            self.ls_cfg.az_max_deg - margin,
        )
        el_ok = (self.ls_cfg.el_min_deg + margin) <= el_deg <= (self.ls_cfg.el_max_deg - margin)

        return {
            "az_ok": bool(az_ok),
            "el_ok": bool(el_ok),
            "hw_az_min": hw_az["min_active"],
            "hw_az_max": hw_az["max_active"],
            "hw_el_min": hw_el["min_active"],
            "hw_el_max": hw_el["max_active"],
        }

    def _apply_ls_config_to_motors(self):
        # Soft-limit stepper tetap aktif sebagai lapis safety tambahan.
        self.motor_az.cfg.soft_limit_min_deg = normalize_360(self.ls_cfg.az_min_deg)
        self.motor_az.cfg.soft_limit_max_deg = normalize_360(self.ls_cfg.az_max_deg)
        self.motor_el.cfg.soft_limit_min_deg = self.ls_cfg.el_min_deg
        self.motor_el.cfg.soft_limit_max_deg = self.ls_cfg.el_max_deg
        LOGGER.info(
            "LS applied: enabled=%s AZ=[%.2f..%.2f] EL=[%.2f..%.2f] margin=%.2f dir=(%d,%d) offset=(%.2f,%.2f)",
            self.ls_cfg.enabled,
            self.ls_cfg.az_min_deg,
            self.ls_cfg.az_max_deg,
            self.ls_cfg.el_min_deg,
            self.ls_cfg.el_max_deg,
            self.ls_cfg.safety_margin_deg,
            self.ls_cfg.az_dir_sign,
            self.ls_cfg.el_dir_sign,
            self.ls_cfg.az_offset_deg,
            self.ls_cfg.el_offset_deg,
        )

    def _set_home_position(self, az_deg: float, el_deg: float):
        self.ls_cfg.home_az_deg = normalize_360(az_deg)
        self.ls_cfg.home_el_deg = clamp(float(el_deg), -90.0, 90.0)
        LOGGER.info("Home set -> AZ=%.2f EL=%.2f", self.ls_cfg.home_az_deg, self.ls_cfg.home_el_deg)

    def _go_home(self):
        self.target_az = clamp(self.ls_cfg.home_az_deg if self.ls_cfg.home_az_deg <= 180 else self.ls_cfg.home_az_deg - 360, MIN_AZ, MAX_AZ)
        self.target_el = clamp(self.ls_cfg.home_el_deg, MIN_EL, MAX_EL)
        LOGGER.info("Go home target -> AZ=%.2f EL=%.2f", self.target_az, self.target_el)

    def _calibrate_home_from_current(self):
        _, pitch, yaw, _ = self.imu.get_angles()
        az_sensor = normalize_360((yaw - self.az_off) * self.ls_cfg.az_dir_sign + self.ls_cfg.az_offset_deg)
        el_sensor = (pitch - self.el_off) * self.ls_cfg.el_dir_sign + self.ls_cfg.el_offset_deg
        self._set_home_position(az_sensor, el_sensor)

    def _handle_ls_breach(self, reason: str):
        self.motor_az.stop_smooth()
        self.motor_el.stop_smooth()
        self.ls_fault_msg = reason
        now = time.time()
        if now - self._last_ls_log > 0.5:
            LOGGER.warning(reason)
            self._last_ls_log = now

    def _evaluate_ls_safety(self, az_deg_360: float, el_deg: float, az_cmd: float, el_cmd: float):
        if not self.ls_cfg.enabled:
            return az_cmd, el_cmd, ""

        stat = self._limit_status_summary(az_deg_360, el_deg)
        margin = self.ls_cfg.safety_margin_deg
        reason = ""

        if not stat["az_ok"]:
            reason = f"AZ out of limit: AZ={az_deg_360:.2f} range={self.ls_cfg.az_min_deg + margin:.2f}..{self.ls_cfg.az_max_deg - margin:.2f}"
            az_cmd = 0.0
        if not stat["el_ok"]:
            reason = f"EL out of limit: EL={el_deg:.2f} range={self.ls_cfg.el_min_deg + margin:.2f}..{self.ls_cfg.el_max_deg - margin:.2f}"
            el_cmd = 0.0

        if self.ls_cfg.use_hw_switch:
            if (az_cmd > 0 and stat["hw_az_max"]) or (az_cmd < 0 and stat["hw_az_min"]):
                reason = "AZ hardware limit switch triggered"
                az_cmd = 0.0
            if (el_cmd > 0 and stat["hw_el_max"]) or (el_cmd < 0 and stat["hw_el_min"]):
                reason = "EL hardware limit switch triggered"
                el_cmd = 0.0

        return az_cmd, el_cmd, reason

    def _print_help(self):
        print(
            "\nCommands:\n"
            "  c                                 : calibrate zero sensor\n"
            "  t <AZ> <EL>                       : set target\n"
            "  ls on|off                         : enable/disable limit switch logic\n"
            "  ls set az <min> <max>             : set AZ limit (0..360, support wrap)\n"
            "  ls set el <min> <max>             : set EL limit (-90..90)\n"
            "  ls offset <az_off> <el_off>       : set limit offset (deg)\n"
            "  ls dir <az_sign> <el_sign>        : set axis sign (-1 or 1)\n"
            "  ls margin <deg>                   : set safety margin\n"
            "  ls hw on|off                      : enable/disable hardware switch check\n"
            "  ls status                          : print LS status\n"
            "  home set <AZ> <EL>                : set home position\n"
            "  home go                            : move target to home\n"
            "  home calib                         : set home from current sensor\n"
            "  q                                 : quit\n"
        )

    def _handle_keyboard_command(self, cmd: str):
        p = cmd.split()
        if not p:
            return
        c = p[0].lower()
        try:
            if c == "help":
                self._print_help()
            elif c == "c":
                _, pitch, yaw, _ = self.imu.get_angles()
                self.az_off = yaw
                self.el_off = pitch
                LOGGER.info("Sensor zero calibrated")
            elif c == "t" and len(p) == 3:
                self.target_az = clamp(float(p[1]), MIN_AZ, MAX_AZ)
                self.target_el = clamp(float(p[2]), MIN_EL, MAX_EL)
                LOGGER.info("Target set -> AZ=%.2f EL=%.2f", self.target_az, self.target_el)
            elif c == "ls" and len(p) == 2 and p[1] in ("on", "off"):
                self.ls_cfg.enabled = (p[1] == "on")
                LOGGER.info("LS enabled=%s", self.ls_cfg.enabled)
            elif c == "ls" and len(p) == 5 and p[1] == "set" and p[2] == "az":
                mn = float(p[3]); mx = float(p[4])
                validate_limit_range(mn, mx, "az")
                self.ls_cfg.az_min_deg = mn
                self.ls_cfg.az_max_deg = mx
                self._apply_ls_config_to_motors()
            elif c == "ls" and len(p) == 5 and p[1] == "set" and p[2] == "el":
                mn = float(p[3]); mx = float(p[4])
                validate_limit_range(mn, mx, "el")
                self.ls_cfg.el_min_deg = mn
                self.ls_cfg.el_max_deg = mx
                self._apply_ls_config_to_motors()
            elif c == "ls" and len(p) == 4 and p[1] == "offset":
                self.ls_cfg.az_offset_deg = float(p[2])
                self.ls_cfg.el_offset_deg = float(p[3])
                self.ls_cfg.validate()
                self._apply_ls_config_to_motors()
            elif c == "ls" and len(p) == 4 and p[1] == "dir":
                self.ls_cfg.az_dir_sign = int(p[2])
                self.ls_cfg.el_dir_sign = int(p[3])
                self.ls_cfg.validate()
                self._apply_ls_config_to_motors()
            elif c == "ls" and len(p) == 3 and p[1] == "margin":
                self.ls_cfg.safety_margin_deg = float(p[2])
                self.ls_cfg.validate()
                self._apply_ls_config_to_motors()
            elif c == "ls" and len(p) == 3 and p[1] == "hw" and p[2] in ("on", "off"):
                self.ls_cfg.use_hw_switch = (p[2] == "on")
                LOGGER.info("LS hardware check=%s", self.ls_cfg.use_hw_switch)
            elif c == "ls" and len(p) == 2 and p[1] == "status":
                _, pitch, yaw, _ = self.imu.get_angles()
                az_deg_360 = normalize_360((yaw - self.az_off) * self.ls_cfg.az_dir_sign + self.ls_cfg.az_offset_deg)
                el_deg = (pitch - self.el_off) * self.ls_cfg.el_dir_sign + self.ls_cfg.el_offset_deg
                st = self._limit_status_summary(az_deg_360, el_deg)
                LOGGER.info(
                    "LS STATUS enabled=%s az=%.2f el=%.2f az_ok=%s el_ok=%s hw(az_min=%s az_max=%s el_min=%s el_max=%s)",
                    self.ls_cfg.enabled,
                    az_deg_360,
                    el_deg,
                    st["az_ok"],
                    st["el_ok"],
                    st["hw_az_min"],
                    st["hw_az_max"],
                    st["hw_el_min"],
                    st["hw_el_max"],
                )
            elif c == "home" and len(p) == 4 and p[1] == "set":
                self._set_home_position(float(p[2]), float(p[3]))
            elif c == "home" and len(p) == 2 and p[1] == "go":
                self._go_home()
            elif c == "home" and len(p) == 2 and p[1] == "calib":
                self._calibrate_home_from_current()
            elif c == "q":
                self.stop_requested = True
            else:
                LOGGER.info("Unknown command. Type: help")
        except Exception as exc:
            LOGGER.error("Command error: %s", exc)

    def keyboard_loop(self):
        self._print_help()
        while not self.stop_requested:
            try:
                cmd = input().strip()
            except EOFError:
                self.stop_requested = True
                break
            self._handle_keyboard_command(cmd)

    def run(self, enable_keyboard: bool = True, status_output: bool = True):
        self.ls_cfg.validate()
        self._apply_ls_config_to_motors()
        if enable_keyboard:
            threading.Thread(target=self.keyboard_loop, daemon=True).start()
        if status_output:
            print("=== ADAPTIVE PID STEPPER + WT901 ===")
            print("Type 'help' to show commands.")

        while not self.stop_requested:
            roll, pitch, yaw, ts = self._get_feedback_angles()
            if time.time() - ts > 1.0:
                self.motor_az.stop_smooth()
                self.motor_el.stop_smooth()
                if status_output:
                    print_status("WAITING IMU DATA...")
                time.sleep(0.05)
                continue

            az = clamp(yaw - self.az_off, MIN_AZ, MAX_AZ)
            el = clamp(pitch - self.el_off, MIN_EL, MAX_EL)
            with self._api_lock:
                self._last_az = az
                self._last_el = el
            az_deg_360 = normalize_360(az * self.ls_cfg.az_dir_sign + self.ls_cfg.az_offset_deg)
            el_ls = (el * self.ls_cfg.el_dir_sign) + self.ls_cfg.el_offset_deg
            az_err = az_error_shortest(self.target_az, az)
            el_err = self.target_el - el

            az_out = self.pid_az.compute(az_err)
            el_out = self.pid_el.compute(el_err)
            el_out += gravity_comp(el)

            az_cmd = clamp(az_out, -self.motor_az.cfg.max_speed_sps, self.motor_az.cfg.max_speed_sps)
            el_cmd = clamp(el_out, -self.motor_el.cfg.max_speed_sps, self.motor_el.cfg.max_speed_sps)

            if abs(az_err) < 0.2:
                az_cmd = 0.0
            if abs(el_err) < 0.2:
                el_cmd = 0.0

            az_cmd, el_cmd, ls_reason = self._evaluate_ls_safety(az_deg_360, el_ls, az_cmd, el_cmd)
            if ls_reason:
                self._handle_ls_breach(ls_reason)
            else:
                self.ls_fault_msg = ""

            self.motor_az.set_target_speed(az_cmd)
            self.motor_el.set_target_speed(el_cmd)

            if status_output:
                st_az = self.motor_az.get_status()
                st_el = self.motor_el.get_status()
                print_status(
                    f"AZ={az:6.2f}({az_deg_360:6.2f}) EL={el:6.2f} | "
                    f"AZ_ERR={az_err:6.2f} EL_ERR={el_err:6.2f} | "
                    f"AZ_SPD={st_az['current_speed_sps']:7.1f} EL_SPD={st_el['current_speed_sps']:7.1f} "
                    f"LS={'ON' if self.ls_cfg.enabled else 'OFF'}"
                )
            time.sleep(0.02)

    def close(self):
        self.stop_requested = True
        self.motor_az.close()
        self.motor_el.close()
        self.imu.close()
        if GPIO_AVAILABLE and (not self.use_sim):
            GPIO.cleanup()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", help="Run simulated stepper without GPIO")
    parser.add_argument("--imu-port", type=str, default=None, help="WT901 serial port, e.g. /dev/ttyUSB0")
    parser.add_argument(
        "--config",
        type=str,
        default=DEFAULT_CONFIG_PATH,
        help="Path to config-stepper.conf",
    )
    args = parser.parse_args()

    use_sim = args.sim or (not GPIO_AVAILABLE)
    if (not GPIO_AVAILABLE) and (not args.sim):
        print(f"RPi.GPIO tidak tersedia ({GPIO_IMPORT_ERROR}), otomatis masuk mode --sim.")

    az_cfg, el_cfg, ls_cfg = load_config_stepper(args.config)
    LOGGER.info("Loaded config from %s", args.config)
    ctl = AdaptiveStepperController(
        use_sim=use_sim,
        imu_port=args.imu_port,
        ls_cfg=ls_cfg,
        az_cfg=az_cfg,
        el_cfg=el_cfg,
    )
    try:
        ctl.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutdown controller...")
        ctl.close()
        print("Done.")


if __name__ == "__main__":
    main()
