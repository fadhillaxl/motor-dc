# Rotator Bridge (SDR + Motor + Gpredict)

## Ikhtisar
- Layanan rotator yang menggabungkan kontrol gimbal AZ/EL dan telemetri SDR.
- Kompatibel dengan Gpredict melalui protokol Hamlib (rotctld subset).
- Folder ini berdiri sendiri; folder lama tidak diubah.

## Dependensi & venv
- Buat venv:
  - `python3 -m venv .venv`
  - `source .venv/bin/activate`
  - `pip install -r requirements.txt`
- Catatan:
  - macOS: pasang `libiio` (Pluto) dan `librtlsdr` (RTL‑SDR) via Homebrew/Conda bila perlu.
  - Raspberry Pi: `RPi.GPIO` tersedia; jalankan layanan di Pi untuk kontrol motor riil.

## Menjalankan Telemetri SDR (opsional)
- Jalankan continuous scan untuk menulis `/tmp/sdr_last.json`:
  - RTL‑SDR: `python src/sdr_signal/cli/scan_peak.py --device rtl --freq 100e6 --rate 2.048e6 --gain auto --n 1048576 --nfft 1024 --continuous --interval 0.5`
  - Pluto+: `python src/sdr_signal/cli/scan_peak.py --device pluto --uri auto --freq 100e6 --rate 2.048e6 --gain 30 --n 262144 --nfft 2048 --continuous --interval 0.5`
  - Mock: `python src/sdr_signal/cli/scan_peak.py --device pluto --mock --freq 100e6 --rate 2.048e6 --n 131072 --nfft 1024 --continuous`

## Menjalankan Layanan Rotator
- `python apps/rotator_bridge/run.py --port 4533` (mock default di macOS)
- Opsi:
  - `--mock`: gunakan posisi sintetis (uji tanpa hardware)
  - `--interval`: periode pembaruan status (detik)

## Konfigurasi Gpredict
- Add Rotator → Hamlib NET rotctld
  - Host: `127.0.0.1`
  - Port: `4533`
  - Mode: `AZ/EL`
- Gpredict akan mengirim `P <az> <el>` dan polling `p`.

## Protokol (subset)
- `P <az> <el>`: set target; balasan `RPRT 0` bila sukses.
- `p`: get posisi, balasan:
  - `Azimuth: <az>` (baris 1)
  - `Elevation: <el>` (baris 2)
- `S`: stop/hold (target = posisi saat ini), balasan `RPRT 0`.
- `Q`: tutup koneksi, balasan `RPRT 0`.
- Kesalahan: `RPRT -n` (n negatif).

## Troubleshooting
- Pluto timeout/“No device found”: pastikan `libiio`, gunakan `--uri auto` atau `usb`, cek interface USB‑Ethernet (`192.168.2.x`).
- RTL‑SDR I/O error: uji `rtl_test -t`, tutup aplikasi lain (Gqrx), periksa kabel/port USB.
- NumPy/OpenBLAS (macOS): pasang `libgfortran` via conda; gunakan venv terpisah.

