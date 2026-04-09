import numpy as np

def gen_tone_noise(n: int, fs_hz: float, tone_offset_hz: float = 100e3, snr_db: float = 20.0):
    t = np.arange(n, dtype=np.float32) / float(fs_hz)
    a_sig = 10.0 ** (snr_db / 20.0)
    tone = a_sig * np.exp(1j * 2.0 * np.pi * tone_offset_hz * t)
    noise = (np.random.randn(n).astype(np.float32) + 1j * np.random.randn(n).astype(np.float32))
    return tone + noise
