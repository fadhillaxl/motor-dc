import argparse
import json
import time
from pathlib import Path
import numpy as np
import sys

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--device", choices=["rtl", "pluto"], required=True)
    p.add_argument("--freq", type=float, required=True)
    p.add_argument("--rate", type=float, default=2.048e6)
    p.add_argument("--gain", default="auto")
    p.add_argument("--n", type=int, default=1024 * 1024)
    p.add_argument("--nfft", type=int, default=1024)
    p.add_argument("--plot", action="store_true")
    p.add_argument("--out", default="/tmp/sdr_last.json")
    p.add_argument("--uri", default=None, help="URI untuk Pluto (mis. ip:192.168.2.1)")
    p.add_argument("--mock", action="store_true")
    p.add_argument("--tone_offset_hz", type=float, default=100e3)
    p.add_argument("--snr_db", type=float, default=20.0)
    p.add_argument("--continuous", action="store_true")
    p.add_argument("--interval", type=float, default=0.5)
    p.add_argument("--iterations", type=int, default=None)
    args = p.parse_args()

    def once(x):
        from sdr_signal.analysis.metrics import compute_metrics
        m = compute_metrics(x, args.rate, args.freq, args.nfft)
        rec = {
            "timestamp": time.time(),
            "device": args.device,
            "center_freq_hz": args.freq,
            "sample_rate_hz": args.rate,
            **{k: m[k] for k in ["average_power_db", "peak_power_db", "peak_freq_hz", "noise_floor_db", "signal_strength_ratio"]},
        }
        s = json.dumps(rec)
        print(s)
        Path(args.out).write_text(s)
        if args.plot and not args.continuous:
            import matplotlib.pyplot as plt
            P, f = m["psd_db"], m["psd_freq_hz"]
            plt.figure()
            plt.plot(np.asarray(f) / 1e6, P)
            plt.xlabel("Frequency (MHz)")
            plt.ylabel("Relative power (dB)")
            plt.show()

    if not args.continuous:
        if args.mock:
            from sdr_signal.sources.mock_reader import gen_tone_noise
            x = gen_tone_noise(args.n, args.rate, args.tone_offset_hz, args.snr_db)
        else:
            if args.device == "rtl":
                from sdr_signal.sources.rtl_sdr_reader import read_samples
                x = read_samples(args.freq, args.rate, args.gain, args.n)
            else:
                from sdr_signal.sources.pluto_reader import read_samples
                gain_db = float(args.gain) if isinstance(args.gain, str) else args.gain
                x = read_samples(args.freq, args.rate, gain_db, args.n, args.uri)
        once(x)
        return

    it = 0
    try:
        if args.mock:
            from sdr_signal.sources.mock_reader import gen_tone_noise
            while True:
                x = gen_tone_noise(args.n, args.rate, args.tone_offset_hz, args.snr_db)
                once(x)
                it += 1
                if args.iterations and it >= args.iterations:
                    break
                time.sleep(args.interval)
        elif args.device == "rtl":
            from rtlsdr import RtlSdr
            sdr = RtlSdr()
            sdr.sample_rate = args.rate
            sdr.center_freq = args.freq
            sdr.gain = args.gain
            try:
                while True:
                    x = sdr.read_samples(args.n)
                    x = np.asarray(x)
                    once(x)
                    it += 1
                    if args.iterations and it >= args.iterations:
                        break
                    time.sleep(args.interval)
            finally:
                sdr.close()
        else:
            import adi
            def open_pluto(uri):
                if uri and uri != "auto":
                    return adi.Pluto(uri=uri)
                for u in ["ip:192.168.2.1", "ip:pluto.local", "usb", "local:"]:
                    try:
                        return adi.Pluto(uri=u)
                    except Exception as e:
                        print(f"Pluto connect failed: {u}: {e}", file=sys.stderr)
                        continue
                raise Exception("No Pluto device found")
            sdr = open_pluto(args.uri if args.uri else "auto")
            sdr.rx_lo = int(args.freq)
            sdr.sample_rate = int(args.rate)
            sdr.rx_hardwaregain = float(args.gain) if isinstance(args.gain, str) else args.gain
            while True:
                x = sdr.rx()
                x = np.asarray(x).astype(np.complex64)
                if len(x) >= args.n:
                    x = x[:args.n]
                once(x)
                it += 1
                if args.iterations and it >= args.iterations:
                    break
                time.sleep(args.interval)
    except KeyboardInterrupt:
        pass

    from sdr_signal.analysis.metrics import compute_metrics, psd_arrays
    m = compute_metrics(x, args.rate, args.freq, args.nfft)
    rec = {
        "timestamp": time.time(),
        "device": args.device,
        "center_freq_hz": args.freq,
        "sample_rate_hz": args.rate,
        **{k: m[k] for k in ["average_power_db", "peak_power_db", "peak_freq_hz", "noise_floor_db", "signal_strength_ratio"]},
    }
    s = json.dumps(rec)
    print(s)
    Path(args.out).write_text(s)

    if args.plot:
        import matplotlib.pyplot as plt
        P, f = m["psd_db"], m["psd_freq_hz"]
        plt.figure()
        plt.plot(np.asarray(f) / 1e6, P)
        plt.xlabel("Frequency (MHz)")
        plt.ylabel("Relative power (dB)")
        plt.show()

if __name__ == "__main__":
    main()
