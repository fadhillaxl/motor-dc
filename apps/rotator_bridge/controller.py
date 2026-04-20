import time
import threading
import os
import sys
import importlib.util
from typing import Optional


def _get_repo_root() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def _add_motorpid_to_path():
    motorpid_dir = os.path.join(_get_repo_root(), "src", "motorPID")
    if motorpid_dir not in sys.path:
        sys.path.insert(0, motorpid_dir)

class MotorController:
    def __init__(self, mock=True):
        self._mock = mock
        self._target_az = 0.0
        self._target_el = 0.0
        self._az = 0.0
        self._el = 0.0
        self._lock = threading.Lock()
        self._stop = False
        threading.Thread(target=self._loop, daemon=True).start()

    def set_target(self, az: float, el: float):
        with self._lock:
            self._target_az = float(az)
            self._target_el = float(el)

    def get_position(self):
        with self._lock:
            return float(self._az), float(self._el)

    def stop(self):
        with self._lock:
            self._target_az = self._az
            self._target_el = self._el
            self._stop = True

    def _loop(self):
        dt = 0.02
        while True:
            time.sleep(dt)
            with self._lock:
                if self._mock:
                    s = 20.0 * dt
                    if abs(self._target_az - self._az) > s:
                        self._az += s if self._target_az > self._az else -s
                    else:
                        self._az = self._target_az
                    if abs(self._target_el - self._el) > s:
                        self._el += s if self._target_el > self._el else -s
                    else:
                        self._el = self._target_el
                else:
                    pass


class AdaptivePIDBridgeController:
    """
    Adapter agar rotctl_server bisa memakai AdaptiveStepperController.
    """

    def __init__(self, sim=False, imu_port=None, config_path=None):
        _add_motorpid_to_path()
        from AdaptivePID import AdaptiveStepperController, load_config_stepper

        if config_path is None:
            config_path = os.path.join(_get_repo_root(), "src", "motorPID", "config-stepper.conf")

        az_cfg, el_cfg, ls_cfg = load_config_stepper(config_path)
        self._ctl = AdaptiveStepperController(
            use_sim=bool(sim),
            imu_port=imu_port,
            ls_cfg=ls_cfg,
            az_cfg=az_cfg,
            el_cfg=el_cfg,
        )
        self._thread = threading.Thread(
            target=self._ctl.run,
            kwargs={"enable_keyboard": False, "status_output": False},
            daemon=True,
        )
        self._thread.start()

    def set_target(self, az: float, el: float):
        self._ctl.set_target(az, el)

    def get_position(self):
        return self._ctl.get_position()

    def stop(self):
        self._ctl.stop()

    def close(self):
        self._ctl.close()


class AzElBridgeController:
    """
    Adapter agar rotctl_server bisa memakai src/motorPID/main/az_el_controller.py.
    """

    def __init__(
        self,
        sim: bool = False,
        sensor_port: Optional[str] = None,
        sensor_baud: int = 9600,
        auto_home: bool = True,
    ):
        repo_root = _get_repo_root()
        azel_path = os.path.join(repo_root, "src", "motorPID", "main", "az_el_controller.py")
        spec = importlib.util.spec_from_file_location("az_el_controller_bridge", azel_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Unable to load az_el_controller module from {azel_path}")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        self._ctl = mod.AzElTrackerService(
            sim=bool(sim),
            auto_home=bool(auto_home),
            sensor_port=sensor_port,
            sensor_baud=int(sensor_baud),
        )

    def set_target(self, az: float, el: float):
        self._ctl.set_target(az, el)

    def get_position(self):
        return self._ctl.get_position()

    def get_debug_snapshot(self):
        return self._ctl.get_debug_snapshot()

    def stop(self):
        self._ctl.stop()

    def reset_fault(self):
        self._ctl.reset_fault()

    def close(self):
        self._ctl.close()
