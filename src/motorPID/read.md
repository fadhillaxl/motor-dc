🧭 WT901 Adaptive PID Azimuth–Elevation Controller

Kontroller 2-axis (Azimuth & Elevation) berbasis WT901 IMU (RS485) dan motor DC (L298N / sejenis) menggunakan Adaptive PID untuk pergerakan cepat, stabil, dan presisi (overshoot < 0.2°).

Cocok untuk:

📡 Antenna / satellite tracker

🎥 DIY gimbal kamera

🛰️ Auto-level platform

🤖 Pan–tilt presisi

✨ Fitur Utama

✅ Adaptive PID

Gain otomatis menyesuaikan jarak ke target

Cepat saat jauh, halus saat dekat

✅ Overshoot sangat kecil (≤ 0.2°)

✅ Gravity compensation (Elevation axis)

✅ Soft-lock dekat target

✅ Realtime 1-line status (tidak spam log)

✅ Kalibrasi nol (zero-offset)

✅ Target AZ/EL via keyboard

✅ Stabil & gimbal-grade

🧱 Arsitektur Sistem
WT901 (RS485 IMU)
        ↓
   Adaptive PID
        ↓
 Motor Driver (L298N)
        ↓
   DC Motor AZ / EL


Azimuth : angleZ (WT901)

Elevation: angleY (WT901)

📂 Struktur Folder
motor-dc/
├── azimuth_elevation_wt901_motor.py   # main program
├── README.md
└── venv/                              # (optional)


WT901 SDK (clone terpisah):

/home/raspberrypi5/
└── WitStandardModbus_WT901C485/
    └── Python/Python-SDK-WT901C485/chs/

🔧 Hardware yang Digunakan

Raspberry Pi 5

WT901C485 (RS485 → USB / TTL)

Motor DC x2

Driver motor (L298N / BTS7960 / sejenis)

Power supply motor terpisah (disarankan)

📦 Dependency
pip install pyserial


Library bawaan:

RPi.GPIO

WT901 Python SDK (official)

⚙️ Konfigurasi Pin (Default)
# Azimuth motor
AZ_EN  = 18
AZ_IN1 = 23
AZ_IN2 = 24

# Elevation motor
EL_EN  = 13
EL_IN1 = 5
EL_IN2 = 6


Ubah sesuai wiring kamu.

▶️ Cara Menjalankan
cd ~/motor-dc
python3 azimuth_elevation_wt901_motor.py

⌨️ Perintah Keyboard
Perintah	Fungsi
c	Kalibrasi posisi saat ini menjadi 0°
t AZ EL	Set target azimuth & elevation
q	Keluar program
Contoh
t -30 30

📊 Output Realtime

Hanya 1 baris yang update, tidak mengganggu input:

STATUS → AZ=-29.9  EL=29.8

🧠 Adaptive PID Strategy

PID tidak statis, tapi menyesuaikan error:

Error	Mode	Perilaku
>10°	FAST	Sangat agresif
3–10°	NORMAL	Cepat & stabil
0.8–3°	PRECISION	Halus
<0.8°	LOCK	Anti overshoot
🎯 Akurasi

Overshoot: ≤ 0.2°

Tidak hunting

Tidak lambat di 1–2° terakhir

Motor berhenti tenang

⚠️ Catatan Penting

Gunakan power motor terpisah

Jangan colok WT901 & motor ke supply yang sama tanpa ground bersama

Pastikan RS485 address = 0x50

Pastikan /dev/ttyUSB0 benar

🚀 Pengembangan Selanjutnya (Opsional)

🔄 Auto-scan azimuth

🎥 Camera follow / tracking

🌍 Auto-level mode

🌐 Web UI / joystick

🧠 Auto PID tuning

📜 Lisensi

Bebas digunakan untuk riset & proyek DIY.
WT901 SDK mengikuti lisensi resmi WitMotion.