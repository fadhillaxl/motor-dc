# Plan Implementasi: Stabilisasi HEADING Compass-Only

## Ringkasan
- Tujuan: menghentikan loncatan liar `HEADING(°)` pada `check_wt901_address.py` saat sensor diam.
- Scope: hanya file `src/motorPID/main/check_wt901_address.py`.
- Keputusan user:
  - Mode heading: **compass-only + filter** (tanpa blend yaw IMU).
  - Target sukses: **loncatan antar-sampel maksimal 5°** saat sensor diam.

## Analisis Kondisi Saat Ini
- Alur saat ini:
  - `heading_tilt_compensated()` menghitung heading langsung dari magnetometer + tilt compensation.
  - Di loop utama, jika magnetometer gagal baru fallback ke yaw IMU.
- Masalah teramati dari log:
  - `EL` stabil, tetapi `HEADING` melompat besar antar-sampel (contoh 292 → 257 → 247 → 207 ...).
- Fakta teknis repo:
  - `compass_calibration.json` belum ada, sehingga script memakai default kalibrasi.
  - Belum ada mekanisme anti-spike/outlier, circular smoothing, atau rate limiting khusus heading.

## Perubahan yang Diusulkan

### 1) Tambah utilitas filter sudut circular (di file yang sama)
- Tambah helper:
  - `_angle_diff_deg(a, b)`: selisih sudut terpendek `(-180..180)`.
  - `_angle_ema_deg(new, old, alpha)`: EMA yang aman untuk wrap 0/360.
  - `_clamp(v, lo, hi)`.
- Alasan:
  - Operasi sudut tidak boleh difilter linear biasa karena discontinuity di 0/360.

### 2) Tambah stateful heading filter khusus compass-only
- Tambah kelas `CompassHeadingFilter` dengan state internal:
  - `last_output`, `last_raw`, `warmup_count`.
- Parameter konfigurasi (konstanta di bagian konfigurasi):
  - `HEADING_MAX_STEP_DEG = 5.0`  (sesuai target user)
  - `HEADING_OUTLIER_DEG = 35.0`
  - `HEADING_EMA_ALPHA = 0.20`
  - `HEADING_WARMUP_SAMPLES = 5`
- Algoritma `update(raw_heading)`:
  - Warmup awal: keluarkan raw langsung sampai sampel cukup.
  - Setelah warmup:
    - Hitung `delta = angle_diff(raw, last_output)`.
    - Jika `abs(delta) > HEADING_OUTLIER_DEG`, perlakukan sebagai spike dan **hold** `last_output`.
    - Jika valid, batasi perubahan per sampel ke `±HEADING_MAX_STEP_DEG`.
    - Terapkan EMA circular pada hasil terbatasi.
- Alasan:
  - Kombinasi outlier reject + slew-rate limit + EMA circular paling efektif untuk loncatan acak tanpa merusak arah umum.

### 3) Ubah loop utama menjadi compass-only murni
- Di `main()`:
  - Inisialisasi `heading_filter = CompassHeadingFilter(...)` sebelum loop.
  - Hilangkan fallback ke yaw IMU sebagai jalur normal.
  - Perhitungan heading:
    - Jika magnetometer valid: hitung `heading_raw = heading_tilt_compensated(...)`, lalu `heading = heading_filter.update(heading_raw)`.
    - Jika magnetometer invalid: `heading = heading_filter.hold()` (nilai terakhir), cetak status degradasi.
- Alasan:
  - Sesuai preferensi user: “langsung pakai compass”.
  - Saat data magnetometer putus sesaat, output tidak loncat.

### 4) Tambah indikator diagnostik minimal di output
- Tambah info singkat pada baris output:
  - status sumber: `CMP` / `HOLD`.
  - delta antar output (opsional 1 kolom) untuk memverifikasi batas 5°.
- Alasan:
  - Mempermudah validasi cepat bahwa filter benar-benar bekerja.

### 5) Perbaiki warning kalibrasi agar eksplisit
- Tetap izinkan jalan tanpa file kalibrasi (sesuai keputusan “opsional tapi disarankan” dari diskusi awal).
- Perjelas warning saat startup:
  - belum ada file kalibrasi → heading bisa drift/noisy.
  - instruksi `--calibrate`.
- Alasan:
  - Mengurangi false expectation tanpa memblokir eksekusi.

## Asumsi dan Keputusan Terkunci
- Tidak mengubah arsitektur komunikasi serial atau SDK.
- Tidak menyentuh file lain (`fix-compas.py`, `az_el_controller.py`, dll).
- Tidak menambah dependency eksternal.
- Fokus patch pada stabilitas output heading, bukan absolut akurasi arah geografis.

## Langkah Verifikasi
1. Jalankan script normal tanpa argumen.
2. Uji kondisi diam ±60 detik:
   - verifikasi loncatan antar-sampel `<= 5°` (lihat kolom delta jika ditambahkan).
   - tidak ada loncatan besar acak seperti log awal.
3. Uji gangguan singkat (mis. pembacaan magnetometer gagal sementara):
   - status berubah ke `HOLD`, nilai heading tidak meloncat.
4. Jalankan mode kalibrasi `--calibrate`, ulangi uji diam:
   - bandingkan kestabilan sebelum/sesudah kalibrasi.
5. Cek diagnostics/lint untuk file yang diedit dan pastikan tidak ada error baru.

## Kriteria Diterima
- Mode heading berjalan compass-only + filter.
- Pada sensor diam, loncatan output heading antar-sampel tidak melebihi 5°.
- Saat data magnetometer invalid sementara, output tetap stabil (hold), bukan loncat liar.
