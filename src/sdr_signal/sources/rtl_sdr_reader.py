import numpy as np
from rtlsdr import RtlSdr

def read_samples(freq_hz: float, rate_hz: float, gain, n: int) -> np.ndarray:
    sdr = RtlSdr()
    sdr.sample_rate = rate_hz
    sdr.center_freq = freq_hz
    sdr.gain = gain
    x = sdr.read_samples(n)
    sdr.close()
    return np.asarray(x)
