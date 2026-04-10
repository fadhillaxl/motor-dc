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
from dataclasses import dataclass

try:
    import RPi.GPIO as GPIO
except Exception as exc:
    print(f"ERROR: gagal import RPi.GPIO: {exc}")
    print("Jalankan file ini di Raspberry Pi dengan library RPi.GPIO terpasang.")
    sys.exit(1)


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

    command_speed = 600.0
    last_report = 0.0

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

    try:
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
        GPIO.cleanup()
        print("GPIO cleaned up.")


if __name__ == "__main__":
    main()
