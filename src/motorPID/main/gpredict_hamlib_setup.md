# Gpredict Hamlib Setup

## Tujuan

Dokumen ini menjelaskan cara menghubungkan `az_el_controller.py` ke Gpredict melalui server rotctld-compatible baru di folder `main`.

File utama:

- [rotctl_server_gpredict.py](file:///Users/mm/GitHub/motor-dc/src/motorPID/main/rotctl_server_gpredict.py)
- [az_el_controller.py](file:///Users/mm/GitHub/motor-dc/src/motorPID/main/az_el_controller.py)
- [start_rotctl_gpredict.sh](file:///Users/mm/GitHub/motor-dc/src/motorPID/main/start_rotctl_gpredict.sh)
- [test_rotctl_gpredict_e2e.py](file:///Users/mm/GitHub/motor-dc/src/motorPID/main/test_rotctl_gpredict_e2e.py)
- [gpredict_rotctl_client.py](file:///Users/mm/GitHub/motor-dc/src/motorPID/main/gpredict_rotctl_client.py)

## Menjalankan Server

### Simulasi

```bash
cd src/motorPID/main
python3 rotctl_server_gpredict.py -gpredict --sim --port 4533 --no-auto-home
```

### Hardware

```bash
cd src/motorPID/main
python3 rotctl_server_gpredict.py -gpredict -m rotator -r /dev/ttyUSB0 -s 9600 --port 4533
```

## Script Otomatisasi

```bash
cd src/motorPID/main
DEVICE_PORT=/dev/ttyUSB0 BAUD_RATE=9600 LISTEN_PORT=4533 ./start_rotctl_gpredict.sh
```

Mode simulasi:

```bash
cd src/motorPID/main
SIM_FLAG=--sim AUTO_HOME_FLAG=--no-auto-home ./start_rotctl_gpredict.sh
```

## Konfigurasi Gpredict

Tambahkan rotator baru:

- Backend: `Hamlib NET rotctld`
- Host: `127.0.0.1`
- Port: `4533`
- Mode: `AZ/EL`

Aktifkan tracking pada Gpredict setelah koneksi berhasil.

## Command yang Didukung

- `P <az> <el>`: set target tracking
- `p`: baca posisi saat ini
- `S`: stop / hold
- `R`: reset fault latch
- `Q`: tutup koneksi

## Simulasi Client Gpredict

Untuk mensimulasikan koneksi Gpredict ke rotctl server secara manual, gunakan:

```bash
cd src/motorPID/main
python3 gpredict_rotctl_client.py --host 127.0.0.1 --port 4533 --az 250 --el 70
```

Jika ingin sekaligus menyalakan server simulasi lokal:

```bash
cd src/motorPID/main
python3 gpredict_rotctl_client.py --spawn-sim-server --az 250 --el 70
```

Script ini akan:

- connect ke rotctl server
- mengirim command `P <az> <el>`
- polling posisi dengan command `p`
- mengirim `R` untuk reset fault
- mengirim `S` untuk stop
- mengirim `Q` untuk menutup sesi

Contoh output:

```text
[INFO] Connected ke 127.0.0.1:4533
>>> P 250.000 70.000
<<< RPRT 0
>>> p
<<< 20.700000
<<< 16.900000
```

## Logging

File log:

- `az_el_system.log`: log kontrol motor, PID, dan limit switch
- `rotctl_gpredict.log`: log komunikasi Hamlib TCP
- `az_el_fault_state.json`: fault terakhir untuk integrasi sistem utama

Contoh data yang dicatat:

- target `AZ/EL`
- posisi aktual `AZ/EL`
- `AZ_ERR`
- `EL_ERR`
- `AZ_PID`
- `EL_PID`
- `EL_FF`
- fault koneksi dan fault gerak

## Error Handling

Implementasi menangani:

- putus koneksi client TCP
- command Hamlib tidak valid
- fault limit switch / soft limit
- kegagalan homing
- reset fault melalui command `R`

## Testing End-to-End

Jalankan server otomatis dalam simulasi dan uji command TCP:

```bash
cd src/motorPID/main
python3 test_rotctl_gpredict_e2e.py --spawn
```

Test ini memverifikasi:

- server aktif di port `4533`
- command `P 250 70` diterima
- command `p` mengembalikan nilai `AZ/EL` valid
- command `S` dan `Q` bekerja

## Testing Manual Tambahan

Urutan uji manual yang disarankan:

1. Jalankan server:

```bash
cd src/motorPID/main
python3 rotctl_server_gpredict.py -gpredict --sim --port 4533 --no-auto-home
```

2. Di terminal lain, jalankan client simulasi:

```bash
cd src/motorPID/main
python3 gpredict_rotctl_client.py --host 127.0.0.1 --port 4533 --az 250 --el 70 --poll-count 5
```

3. Verifikasi:

- server membalas `RPRT 0` untuk command target
- nilai `AZ/EL` hasil polling berubah mendekati target
- log di `rotctl_gpredict.log` mencatat command dan response
- log di `az_el_system.log` mencatat `AZ_ERR`, `EL_ERR`, `AZ_PID`, `EL_PID`, dan `EL_FF`
