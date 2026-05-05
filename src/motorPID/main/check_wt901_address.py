"""
Baca 2 sensor WT901C485 pada satu bus RS485.

- 0x50: sensor EL (elevasi/pitch)
- 0x51: sensor AZ (compass/yaw)
"""

import os
import sys
import time
import platform
import math

# PYTHON PATH -> folder chs lokal project
# =====================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.join(BASE_DIR, "..", "..", "Python-SDK-WT901C485", "chs")
sys.path.insert(0, SDK_CHS)

# ─── Import dari SDK WITMOTION ─────────────────────────────────────────────
try:
    import lib.device_model as deviceModel
    from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
    from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver
except ImportError as e:
    print(f"[ERROR] Library WITMOTION tidak ditemukan: {e}")
    print("Pastikan struktur folder SDK sudah benar:")
    print(f"  {SDK_CHS}/lib/device_model.py")
    print(f"  {SDK_CHS}/lib/data_processor/roles/jy901s_dataProcessor.py")
    print(f"  {SDK_CHS}/lib/protocol_resolver/roles/protocol_485_resolver.py")
    sys.exit(1)

# ─── Konfigurasi ──────────────────────────────────────────────────────────
INTERVAL = 0.5  # Interval baca dalam detik
ADDR_EL = 0x50
ADDR_AZ = 0x51

# ─── Konstanta Register ───────────────────────────────────────────────────
REG_KEY = 0x69        # Register kunci untuk operasi tulis
REG_ANGLE = 0x3D      # AngleX, AngleY, AngleZ (3 register)
REG_MAG = 0x3A        # MagX, MagY, MagZ (3 register)

# ─── Kompas / Heading AZ ──────────────────────────────────────────────────
# Sesuaikan jika hasil heading terbalik/bergeser karena orientasi pemasangan sensor.
DECLINATION_DEG = 0.0   # Koreksi deklinasi magnetik lokasi (opsional)
AZ_OFFSET_DEG = 0.0     # Offset heading manual setelah kalibrasi lapangan
MAG_SWAP_XY = False     # True jika sumbu X/Y tertukar
MAG_INV_X = False       # True jika sumbu X terbalik
MAG_INV_Y = False       # True jika sumbu Y terbalik

def _raw_to_angle(raw):
    """Konversi register 16-bit ke derajat (-180..180)."""
    if raw > 32767:
        raw -= 65536
    return raw / 32768.0 * 180.0


def _map_el_roll_to_90_0(el_roll_deg):
    """Map EL dari roll: |roll| 0..90 menjadi EL 90..0."""
    roll_abs_clamped = max(0.0, min(90.0, abs(float(el_roll_deg))))
    return 90.0 - roll_abs_clamped


def _normalize_azimuth_0_360(yaw_deg):
    """Normalisasi azimuth ke rentang 0..360."""
    return float(yaw_deg) % 360.0


def _raw_to_signed16(raw):
    """Konversi register uint16 ke int16."""
    val = int(raw) & 0xFFFF
    if val >= 0x8000:
        val -= 0x10000
    return val


def _heading_from_magnetometer(mag_x, mag_y):
    """
    Hitung heading kompas dari magnetometer (sumbu horizontal), hasil 0..360 derajat.
    """
    x = float(mag_x)
    y = float(mag_y)
    if MAG_SWAP_XY:
        x, y = y, x
    if MAG_INV_X:
        x = -x
    if MAG_INV_Y:
        y = -y

    heading = math.degrees(math.atan2(y, x))
    heading += DECLINATION_DEG + AZ_OFFSET_DEG
    return _normalize_azimuth_0_360(heading)


def reset_zero_point(device, addr):
    """
    Menghapus zero-point yang tersimpan di sensor.
    Ini memastikan EL selalu mengacu pada gravitasi (sudut absolut),
    bukan posisi saat dinyalakan.
    """
    print("[INFO] Mereset zero-point ke default (sudut absolut)...")
    try:
        device.ADDR = addr
        # Unlock register untuk penulisan
        device.writeReg(REG_KEY, 0xB588)
        time.sleep(0.1)
        device.writeReg(0x01, 0x0000)
        time.sleep(0.3)

        print("[OK] Zero-point berhasil direset. Sensor sekarang menggunakan gravitasi sebagai referensi.\n")
    except Exception as e:
        print(f"[WARN] Gagal reset zero-point: {e}")
        print("[WARN] Melanjutkan tanpa reset (sudut mungkin memiliki offset).\n")


def baca_sudut(device, addr):
    """
    Membaca sudut Roll, Pitch (EL), dan Yaw dari sensor.
    Mengembalikan tuple (roll, pitch, yaw) dalam derajat.
    Data diambil dari readReg(0x3D, 3) lalu dikonversi ke derajat.
    """
    try:
        device.ADDR = addr
        vals = device.readReg(REG_ANGLE, 3)
        if not vals or len(vals) < 3:
            return None, None, None
        roll = _raw_to_angle(vals[0])
        pitch = _raw_to_angle(vals[1])
        yaw = _raw_to_angle(vals[2])
        return float(roll), float(pitch), float(yaw)
    except Exception:
        return None, None, None


def baca_azimuth_kompas(device, addr):
    """
    Baca heading kompas dari register magnetometer AZ (0x3A..0x3C).
    """
    try:
        device.ADDR = addr
        vals = device.readReg(REG_MAG, 3)
        if not vals or len(vals) < 2:
            return None
        mag_x = _raw_to_signed16(vals[0])
        mag_y = _raw_to_signed16(vals[1])
        return _heading_from_magnetometer(mag_x, mag_y)
    except Exception:
        return None


def tampilkan_header():
    print("=" * 60)
    print("  WT901C485 - Dual Sensor (EL 0x50 + AZ 0x51)")
    print("=" * 60)
    print("Penjelasan sudut:")
    print("  ROLL  (X) : Kemiringan kiri-kanan")
    print("  PITCH (Y) : Kemiringan depan-belakang [ELEVASI/EL]")
    print("  AZ_YAW    : Heading kompas 0..360 derajat")
    print()
    print("Referensi: GRAVITASI BUMI (sudut absolut)")
    print("  0°   = Sensor sejajar dengan tanah (datar)")
    print("  90°  = Sensor berdiri tegak")
    print("  -90° = Sensor terbalik tegak")
    print()
    print("Tekan Ctrl+C untuk berhenti.")
    print("-" * 60)
    print(f"{'Waktu':<10} {'EL_FROM_ROLL(°)':>15} {'AZ_YAW(°)':>10} {'EL_ROLL(°)':>11} {'AZ_ROLL(°)':>11}")
    print("-" * 60)


def main():
    tampilkan_header()

    # Inisialisasi koneksi ke sensor
    try:
        device = deviceModel.DeviceModel(
            "WT901C485-DUAL",
            Protocol485Resolver(),
            JY901SDataProcessor(),
            lambda *_: None
        )
        device.ADDR = ADDR_EL
        if platform.system().lower() == "linux":
            device.serialConfig.portName = "/dev/ttyUSB0"
        else:
            device.serialConfig.portName = "/dev/tty.usbserial-1330"
        device.serialConfig.baud = 9600
        device.openDevice()
        print(f"[OK] Terhubung ke {device.serialConfig.portName} @ {device.serialConfig.baud} baud")
        print(f"[OK] Polling alamat sensor: EL={hex(ADDR_EL)}, AZ={hex(ADDR_AZ)}\n")
    except Exception as e:
        print(f"[ERROR] Tidak bisa membuka port: {e}")
        print("Pastikan:")
        print("  1. Sensor terhubung ke komputer")
        print("  2. Port sudah benar (Linux: /dev/ttyUSB0 | Mac: /dev/tty.usbserial-xxxx)")
        print("  3. Tidak ada aplikasi lain yang menggunakan port ini")
        sys.exit(1)

    # Reset zero-point untuk masing-masing sensor
    reset_zero_point(device, ADDR_EL)
    reset_zero_point(device, ADDR_AZ)

    # Loop pembacaan data
    try:
        while True:
            el_roll, el_pitch, _ = baca_sudut(device, ADDR_EL)
            az_roll, _, az_yaw_fused = baca_sudut(device, ADDR_AZ)
            az_yaw_compass = baca_azimuth_kompas(device, ADDR_AZ)

            if el_pitch is None or (az_yaw_compass is None and az_yaw_fused is None):
                print("[WARN] Gagal membaca data dari salah satu sensor, mencoba lagi...")
            else:
                waktu = time.strftime("%H:%M:%S")
                el_from_roll = _map_el_roll_to_90_0(el_roll)
                az_yaw_360 = az_yaw_compass if az_yaw_compass is not None else _normalize_azimuth_0_360(az_yaw_fused)

                if el_from_roll < 5:
                    status_el = "TEGAK"
                elif el_from_roll > 85:
                    status_el = "DATAR"
                else:
                    status_el = f"EL {el_from_roll:.1f}°"

                print(
                    f"{waktu:<10} {el_from_roll:>15.2f} {az_yaw_360:>10.2f} "
                    f"{el_roll:>11.2f} {az_roll:>11.2f}   [EL:{status_el}]"
                )

            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n" + "-" * 60)
        print("[INFO] Program dihentikan oleh pengguna.")

    finally:
        try:
            device.closeDevice()
            print("[INFO] Koneksi serial ditutup.")
        except Exception:
            pass


if __name__ == "__main__":
    main()
