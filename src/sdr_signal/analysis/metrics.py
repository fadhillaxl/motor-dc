import math
import numpy as np

def average_power_db(x: np.ndarray) -> float:
    return 10.0 * math.log10(float(np.var(x)) + 1e-12)

def psd_arrays(x: np.ndarray, fs_hz: float, fc_hz: float, nfft: int):
    m = min(len(x), nfft)
    w = np.hanning(m)
    X = np.fft.fftshift(np.fft.fft(x[:m] * w, nfft))
    P = 10.0 * np.log10((np.abs(X) ** 2) / float(np.sum(w ** 2)) + 1e-12)
    f = np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0 / fs_hz)) + fc_hz
    return P, f

def psd_peak(x: np.ndarray, fs_hz: float, fc_hz: float, nfft: int = 1024):
    P, f = psd_arrays(x, fs_hz, fc_hz, nfft)
    i = int(np.argmax(P))
    return float(P[i]), float(f[i]), P, f

def compute_metrics(x: np.ndarray, fs_hz: float, fc_hz: float, nfft: int = 1024):
    avg_db = average_power_db(x)
    peak_db, peak_f_hz, P, f = psd_peak(x, fs_hz, fc_hz, nfft)
    floor_db = float(np.percentile(P, 10.0))
    ratio = float(np.clip((peak_db - floor_db) / max(1.0, abs(floor_db)), 0.0, 1.0))
    return {
        "average_power_db": float(avg_db),
        "peak_power_db": float(peak_db),
        "peak_freq_hz": float(peak_f_hz),
        "noise_floor_db": floor_db,
        "signal_strength_ratio": ratio,
        "psd_db": P.tolist(),
        "psd_freq_hz": f.tolist(),
    }
