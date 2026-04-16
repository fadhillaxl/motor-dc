#!/usr/bin/env python3
# coding: UTF-8
import argparse
import importlib.util
import os
import queue
import select
import sys
import termios
import threading
import time
import types
import tty

from rotctl_server import RotctlServer
from controller import MotorController, AdaptivePIDBridgeController
from telemetry_sdr import TelemetrySDR


def _repo_root() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def _default_imu_port() -> str:
    # Keep default behavior aligned with src/motorPID/read_wt901.py
    # Linux: /dev/ttyUSB0, others: /dev/tty.usbserial-1330
    if sys.platform.startswith("linux"):
        return "/dev/ttyUSB0"
    return "/dev/tty.usbserial-1330"


def _load_legacy_keyboard_module():
    path = os.path.join(_repo_root(), "src", "motorPID", "keyboard-motor-stepper.py")
    motorpid_dir = os.path.dirname(path)
    if motorpid_dir not in sys.path:
        sys.path.insert(0, motorpid_dir)
    if sys.version_info < (3, 10):
        mod_name = "keyboard_motor_stepper"
        mod = types.ModuleType(mod_name)
        mod.__name__ = mod_name
        mod.__file__ = path
        mod.__package__ = ""
        sys.modules[mod_name] = mod
        with open(path, "r", encoding="utf-8") as f:
            src = f.read()
        # Python <3.10: avoid runtime evaluation of PEP604 annotations (int | None).
        src = "from __future__ import annotations\n" + src
        code = compile(src, path, "exec")
        exec(code, mod.__dict__)
        return mod
    spec = importlib.util.spec_from_file_location("keyboard_motor_stepper", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load legacy module from {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class KeyboardInputAdapter:
    """
    Non-blocking keyboard shim for rotator bridge.
    It maps familiar keys into controller.set_target() calls and runs in a daemon thread.
    """

    def __init__(self, controller, speed_deg_step: float = 1.0, poll_timeout: float = 0.03):
        self.controller = controller
        self.speed_deg_step = float(speed_deg_step)
        self.poll_timeout = float(poll_timeout)
        self._q: "queue.Queue[str]" = queue.Queue()
        self._stop_evt = threading.Event()
        self._th_read = threading.Thread(target=self._reader_loop, daemon=True)
        self._th_apply = threading.Thread(target=self._apply_loop, daemon=True)

    def start(self):
        self._th_read.start()
        self._th_apply.start()

    def stop(self):
        self._stop_evt.set()

    def _reader_loop(self):
        if not sys.stdin.isatty():
            return
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not self._stop_evt.is_set():
                r, _, _ = select.select([sys.stdin], [], [], self.poll_timeout)
                if not r:
                    continue
                ch = sys.stdin.read(1)
                if ch == "\x1b":
                    seq = ch
                    r, _, _ = select.select([sys.stdin], [], [], 0.002)
                    if r:
                        seq += sys.stdin.read(1)
                    r, _, _ = select.select([sys.stdin], [], [], 0.002)
                    if r:
                        seq += sys.stdin.read(1)
                    self._q.put(seq)
                else:
                    self._q.put(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _apply_loop(self):
        while not self._stop_evt.is_set():
            try:
                key = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                az, el = self.controller.get_position()
                step = self.speed_deg_step
                if key in ("w", "W", "\x1b[A"):
                    self.controller.set_target(az, el + step)
                elif key in ("s", "S", "\x1b[B"):
                    self.controller.set_target(az, el - step)
                elif key in ("a", "A", "\x1b[D"):
                    self.controller.set_target(az - step, el)
                elif key in ("d", "D", "\x1b[C"):
                    self.controller.set_target(az + step, el)
                elif key in (" ",):
                    self.controller.stop()
                elif key in ("q", "Q"):
                    self._stop_evt.set()
            except Exception:
                # Shim should never kill the bridge loop.
                pass


def _run_legacy_keyboard_stepper(argv):
    mod = _load_legacy_keyboard_module()
    old_argv = sys.argv[:]
    try:
        sys.argv = [sys.argv[0]] + argv
        mod.main()
    finally:
        sys.argv = old_argv


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=4533)
    p.add_argument("--backend", choices=["mock", "adaptive"], default="mock")
    p.add_argument("--mock", action="store_true")
    p.add_argument("--sim", action="store_true", help="Run adaptive backend in simulation mode")
    p.add_argument("--imu-port", type=str, default=None, help="WT901 serial port for adaptive backend")
    p.add_argument(
        "--config",
        type=str,
        default=os.path.join("src", "motorPID", "config-stepper.conf"),
        help="Path to config-stepper.conf (used by adaptive backend)",
    )
    p.add_argument("--interval", type=float, default=0.5)
    p.add_argument("--keyboard-shim", action="store_true", help="Enable non-blocking WASD/arrow control shim")
    p.add_argument("--keyboard-step", type=float, default=1.0, help="Degree step per key event for keyboard shim")
    p.add_argument(
        "--legacy-keyboard-stepper",
        action="store_true",
        help="Run src/motorPID/keyboard-motor-stepper.py main() from this launcher",
    )
    p.add_argument(
        "--legacy-args",
        nargs=argparse.REMAINDER,
        default=[],
        help="Arguments forwarded to keyboard-motor-stepper.py when --legacy-keyboard-stepper is set",
    )
    args = p.parse_args()

    if args.legacy_keyboard_stepper:
        _run_legacy_keyboard_stepper(args.legacy_args)
        return

    backend = "adaptive" if args.backend == "adaptive" else "mock"
    if args.mock:
        backend = "mock"

    imu_port = args.imu_port if args.imu_port else _default_imu_port()
    if backend == "adaptive":
        ctrl = AdaptivePIDBridgeController(
            sim=args.sim,
            imu_port=imu_port,
            config_path=args.config,
        )
    else:
        ctrl = MotorController(mock=True)

    shim = None
    if args.keyboard_shim:
        shim = KeyboardInputAdapter(ctrl, speed_deg_step=args.keyboard_step)
        shim.start()
        print("Keyboard shim enabled: A/D=AZ, W/S=EL, arrows supported, Space=stop, Q=quit shim")

    srv = RotctlServer(ctrl, port=args.port)
    srv.start()
    tel = TelemetrySDR(interval=args.interval)

    print(f"Rotator Bridge listening on port {args.port} (backend={backend})")
    try:
        while True:
            t = tel.poll()
            if t:
                pk = t.get("peak_power_db")
                pf = t.get("peak_freq_hz")
                sr = t.get("signal_strength_ratio")
                if pk is not None and pf is not None and sr is not None:
                    pass
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        if shim is not None:
            shim.stop()
        try:
            if hasattr(ctrl, "close"):
                ctrl.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
