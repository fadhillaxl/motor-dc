## Tujuan
- Menambahkan modul SDR terpisah untuk menghitung peak signal, level dB, dan signal strength
- Mendukung dua sumber: RTL‑SDR Blog v4 dan Pluto+ (ADALM‑Pluto)
- Menyediakan CLI dan API sederhana untuk integrasi dengan AdaptivePID.py

## Struktur Folder
- src/sdr_signal/
  - sources/rtl_sdr_reader.py
  - sources/pluto_reader.py
  - analysis/metrics.py
  - cli/scan_peak.py
  - __init__.py

## Dependensi
- RTL‑SDR: numpy, matplotlib, pyrtlsdr
- Pluto+: numpy, matplotlib, pyadi‑iio (butuh libiio di sistem)
- Opsi headless: nonaktifkan plotting, hanya hitung PSD dan metrik

## Implementasi Inti
- Reader RTL‑SDR
```python
import numpy as np, math
from rtlsdr import RtlSdr

def read_rtl(freq_hz, rate_hz, gain, n):
    sdr = RtlSdr(); sdr.sample_rate = rate_hz; sdr.center_freq = freq_hz; sdr.gain = gain
    x = sdr.read_samples(n); sdr.close(); return np.asarray(x)
```
- Reader Pluto+
```python
import numpy as np
import adi

def read_pluto(freq_hz, rate_hz, gain_db, n):
    sdr = adi.Pluto(); sdr.rx_lo = int(freq_hz); sdr.sample_rate = int(rate_hz); sdr.rx_hardwaregain = gain_db
    x = sdr.rx()[:n]; return np.asarray(x)
```
- Hitung Metrik
```python
import numpy as np, math

def average_power_db(x):
    return 10 * math.log10(np.var(x) + 1e-12)

def psd_peak(x, fs_hz, fc_hz, nfft=1024):
    w = np.hanning(min(len(x), nfft))
    X = np.fft.fftshift(np.fft.fft(x[:len(w)] * w, nfft))
    P = 10 * np.log10((np.abs(X)**2) / np.sum(w**2) + 1e-12)
    f = np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0/fs_hz)) + fc_hz
    i = int(np.argmax(P)); return float(P[i]), float(f[i])

def strength_ratio_db(p_db, floor_db):
    return float(np.clip((p_db - floor_db) / max(1.0, abs(floor_db)), 0.0, 1.0))
```

## CLI & Output
- cli/scan_peak.py: argumen --device {rtl,pluto} --freq --rate --gain --n --nfft --plot
- Cetak JSON ke stdout dan simpan ke file (mis. /tmp/sdr_last.json):
  - timestamp, device, center_freq_hz, sample_rate_hz
  - average_power_db, peak_power_db, peak_freq_hz, signal_strength_ratio
- Plot opsional PSD jika --plot diaktifkan

## Integrasi ke AdaptivePID.py
- API: from sdr_signal.analysis.metrics import average_power_db, psd_peak
- Mode terpisah: jalankan scan_peak.py sebagai proses latar dan baca /tmp/sdr_last.json di loop AdaptivePID untuk telemetri
- Tidak mengubah logika PID; hanya menambah tampilan/telemetri agar operator bisa memaksimalkan sinyal selama tracking

## Konfigurasi Default
- RTL‑SDR: rate=2.048e6, gain='auto', n=1024*1024, nfft=1024
- Pluto+: rate=2.048e6, gain_db=30, n=262144, nfft=2048
- Frekuensi diatur via argumen (contoh 100e6 untuk FM)

## Validasi
- Uji RTL‑SDR: band FM, verifikasi peak di sekitar stasiun lokal
- Uji Pluto+: verifikasi pengambilan sampel dan psd peak dengan antena yang sama
- Headless: jalankan tanpa plot dan cek file JSON

## Contoh Pemakaian
```bash
python src/sdr_signal/cli/scan_peak.py --device rtl --freq 100e6 --rate 2.048e6 --gain auto --n 1048576 --nfft 1024 --plot
python src/sdr_signal/cli/scan_peak.py --device pluto --freq 100e6 --rate 2.048e6 --gain 30 --n 262144 --nfft 2048
```

## Langkah Setelah Disetujui
- Buat folder dan file sesuai struktur
- Implementasi reader, metrik, dan CLI
- Tambah contoh integrasi ringan ke AdaptivePID.py untuk membaca JSON dan mencetak metrik