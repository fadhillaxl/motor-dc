import json
import os
import time

class TelemetrySDR:
    def __init__(self, path="/tmp/sdr_last.json", interval=0.5):
        self.path = path
        self.interval = interval
        self.last = {}
        self._last_t = 0.0

    def poll(self):
        t = time.time()
        if t - self._last_t < self.interval:
            return self.last
        self._last_t = t
        try:
            if os.path.exists(self.path):
                s = open(self.path, "r").read()
                self.last = json.loads(s)
        except Exception:
            pass
        return self.last
