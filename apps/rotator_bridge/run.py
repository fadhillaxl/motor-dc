import argparse
import time
from rotctl_server import RotctlServer
from controller import MotorController
from telemetry_sdr import TelemetrySDR

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=4533)
    p.add_argument("--mock", action="store_true")
    p.add_argument("--interval", type=float, default=0.5)
    args = p.parse_args()

    ctrl = MotorController(mock=args.mock)
    srv = RotctlServer(ctrl, port=args.port)
    srv.start()
    tel = TelemetrySDR(interval=args.interval)

    print(f"Rotator Bridge listening on port {args.port} (mock={args.mock})")
    try:
        while True:
            t = tel.poll()
            if t:
                pk = t.get("peak_power_db"); pf = t.get("peak_freq_hz"); sr = t.get("signal_strength_ratio")
                if pk is not None and pf is not None and sr is not None:
                    # print(f"SDR SIG {pk:.1f} dB @{pf/1e6:.2f} MHz R={sr:.2f}")
                    pass
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
