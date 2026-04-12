import argparse
import os
import time
from rotctl_server import RotctlServer
from controller import MotorController, AdaptivePIDBridgeController
from telemetry_sdr import TelemetrySDR

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
    args = p.parse_args()

    backend = "mock"
    if args.backend == "adaptive":
        backend = "adaptive"
    elif args.mock:
        backend = "mock"

    if backend == "adaptive":
        ctrl = AdaptivePIDBridgeController(
            sim=args.sim,
            imu_port=args.imu_port,
            config_path=args.config,
        )
    else:
        ctrl = MotorController(mock=True)

    srv = RotctlServer(ctrl, port=args.port)
    srv.start()
    tel = TelemetrySDR(interval=args.interval)

    print(f"Rotator Bridge listening on port {args.port} (backend={backend})")
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
    finally:
        try:
            if hasattr(ctrl, "close"):
                ctrl.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
