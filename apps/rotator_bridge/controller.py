import time
import threading

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
