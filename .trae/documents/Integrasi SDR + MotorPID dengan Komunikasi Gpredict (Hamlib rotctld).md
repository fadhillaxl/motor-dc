## Tujuan
- Membuat integrasi SDR + motorPID + Gpredict dalam folder baru agar folder lama tetap utuh
- Menyediakan requirements.txt dan panduan venv untuk isolasi dependensi

## Struktur Folder Baru
- apps/rotator_bridge/
  - controller.py: kontrol AZ/EL (WT901 + PID), driver GPIO/PWM, tanpa mengubah kode lama
  - rotctl_server.py: TCP server kompatibel Hamlib (subset rotctld) untuk Gpredict
  - telemetry_sdr.py: pembaca /tmp/sdr_last.json (output dari sdr_signal/scan_peak)
  - run.py: entrypoint yang menjalankan controller + rotctl_server + telemetry
  - requirements.txt: daftar dependensi
  - README.md: dokumentasi penggunaan dan konfigurasi

## Protokol rotctl (subset minimal)
- P <az> <el>: set target derajat
- p: get posisi (balasan dua baris: “Azimuth: X” dan “Elevation: Y”)
- S: stop (target = posisi saat ini), balasan “RPRT 0”
- Q: tutup koneksi, balasan “RPRT 0”
- Kesalahan: “RPRT x” (x negatif)

## Controller (tanpa ganggu folder lama)
- Implementasi mandiri mirip AdaptivePID: pembacaan WT901 via SDK, loop PID untuk AZ/EL, kompensasi gravitasi, GPIO/PWM
- API: set_target(az, el), get_position() -> (az, el), stop()
- Opsi --mock untuk uji tanpa hardware (menghasilkan posisi sintetis)

## Telemetri SDR
- telemetry_sdr.py membaca /tmp/sdr_last.json dan menampilkan metrik (peak dB, frekuensi) di log
- Tidak memodifikasi sdr_signal; cukup jalankan scan_peak (continuous) terpisah

## requirements.txt
- numpy
- matplotlib (opsional plotting)
- pyrtlsdr (opsional jika pakai RTL‑SDR)
- pyadi-iio (opsional jika pakai Pluto)
- RPi.GPIO (untuk Raspberry Pi; di macOS diabaikan)
- pyserial (bila perlu untuk SDK WT901)
- (catatan kompatibilitas arm64/macOS vs Raspberry Pi)

## venv
- Cara membuat lingkungan terisolasi di folder apps/rotator_bridge:
  - python3 -m venv .venv
  - source .venv/bin/activate
  - pip install -r requirements.txt

## Dokumentasi
- README.md berisi:
  - Ikhtisar arsitektur dan diagram alur
  - Instalasi dependensi (macOS/Raspberry Pi), pembuatan venv
  - Menjalankan run.py (opsi port, mock)
  - Menjalankan sdr_signal/cli/scan_peak.py (continuous) untuk telemetri
  - Konfigurasi Gpredict: Hamlib NET rotctld, host 127.0.0.1, port 4533, mode AZ/EL
  - Contoh pengujian dengan rotctl (P/p/S/Q)
  - Troubleshooting (Pluto timeout, RTL‑SDR I/O, NumPy/BLAS di macOS)

## Pengujian
- Test rotctl server: kirim perintah P/p/S/Q dan verifikasi format respons
- Test controller mock: posisi sintetis dan respon p stabil
- Test integrasi real (Pi): WT901 + PID + motor, Gpredict mengendalikan target

## Setelah Disetujui
- Buat folder apps/rotator_bridge dengan file/controller/server/telemetry/run sesuai di atas
- Tambahkan requirements.txt + README.md
- Tidak menyentuh src/sdr_signal dan src/motorPID agar kompatibel
