import numpy as np
import adi

def read_samples(freq_hz: float, rate_hz: float, gain_db: float, n: int, uri: str | None = None) -> np.ndarray:
    sdr = adi.Pluto(uri=uri) if uri else adi.Pluto()
    sdr.rx_lo = int(freq_hz)
    sdr.sample_rate = int(rate_hz)
    sdr.rx_hardwaregain = gain_db
    x = sdr.rx()
    x = np.asarray(x).astype(np.complex64)
    if len(x) >= n:
        return x[:n]
    return x
