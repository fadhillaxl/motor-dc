"""
Baca Elevasi Absolut Terhadap Gravitasi - WT901C485
=====================================================
Script ini membaca sudut EL (Elevation/Roll) dari sensor WT901C485
menggunakan referensi gravitasi bumi (sudut absolut).

Cara penggunaan:
1. Pastikan library WITMOTION sudah di-download dari:
   https://github.com/WITMOTION/WitStandardModbus_WT901C485/tree/main/Python/Python-SDK-WT901C485/chs
2. Letakkan file ini di folder yang SAMA dengan file SDK (device.py, dll)
3. Jalankan: python baca_elevasi_absolut.py

Dependensi:
    pip install pyserial
"""

import os
import sys
import time
import platform
import math

# PYTHON PATH -> folder chs lokal project
# =====================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "Python-SDK-WT901C485", "chs"))
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
INTERVAL  = 0.5          # Interval baca dalam detik
TILT_THRESHOLD_DEG = 15.0  # Ambang tilt untuk beralih dari YAW ke COMPASS
AZ_OFFSET_DEG = 19.5        # Offset heading manual (derajat), contoh: 28.7

# ─── Konstanta Reset Zero-Point ───────────────────────────────────────────
REG_KEY   = 0x69          # Register kunci untuk operasi tulis

def buat_device_model():
    """
    Membuat objek DeviceModel yang kompatibel dengan beberapa versi SDK.
    Sebagian versi membutuhkan argumen dataUpdateListener tambahan.
    """
    try:
        # Versi SDK lama: 3 argumen
        return deviceModel.DeviceModel(
            "WT901C485",
            Protocol485Resolver(),
            JY901SDataProcessor(),
        )
    except TypeError:
        # Versi SDK baru: 4 argumen (dataUpdateListener)
        return deviceModel.DeviceModel(
            "WT901C485",
            Protocol485Resolver(),
            JY901SDataProcessor(),
            "EL_0",
        )


def reset_zero_point(device):
    """
    Menghapus zero-point yang tersimpan di sensor.
    Ini memastikan EL (Roll) selalu mengacu pada gravitasi (sudut absolut),
    bukan posisi saat dinyalakan.
    """
    print("[INFO] Mereset zero-point ke default (sudut absolut)...")
    try:
        # Mendukung dua varian API SDK: write_register(...) dan writeReg(...)
        if hasattr(device, "write_register"):
            device.write_register(device.ADDR, REG_KEY, 0xB588)
            time.sleep(0.1)
            device.write_register(device.ADDR, 0x01, 0x0000)
        else:
            if hasattr(device, "unlock"):
                device.unlock()
                time.sleep(0.1)
            device.writeReg(0x01, 0x0000)
            if hasattr(device, "save"):
                time.sleep(0.1)
                device.save()
        time.sleep(0.3)

        print("[OK] Zero-point berhasil direset. Sensor sekarang menggunakan gravitasi sebagai referensi.\n")
    except Exception as e:
        print(f"[WARN] Gagal reset zero-point: {e}")
        print("[WARN] Melanjutkan tanpa reset (sudut mungkin memiliki offset).\n")


def baca_sudut(device):
    """
    Membaca sudut Roll (EL), Pitch, Yaw, dan menghitung Azimuth Kompas murni.
    Mengembalikan tuple (roll, pitch, yaw, compass, az_used, az_source) dalam derajat.
    Menggunakan data register yang diparse oleh JY901SDataProcessor.
    """
    try:
        # Untuk SDK 485, pembacaan register ini akan mengisi deviceData
        # termasuk angleX/angleY/angleZ dan magX/magY/magZ.
        if hasattr(device, "readReg"):
            device.readReg(0x30, 41)

        if hasattr(device, "get"):
            roll = device.get("AngleX")
            pitch = device.get("AngleY")
            yaw = device.get("AngleZ")
            accX = device.get("accX")
            accY = device.get("accY")
            accZ = device.get("accZ")
            magX = device.get("magX")
            magY = device.get("magY")
            magZ = device.get("magZ")
        else:
            roll = device.getDeviceData("angleX")
            pitch = device.getDeviceData("angleY")
            yaw = device.getDeviceData("angleZ")
            accX = device.getDeviceData("accX")
            accY = device.getDeviceData("accY")
            accZ = device.getDeviceData("accZ")
            magX = device.getDeviceData("magX")
            magY = device.getDeviceData("magY")
            magZ = device.getDeviceData("magZ")

        if roll is None or pitch is None or yaw is None:
            return None, None, None, None, None, None

        # Hitung sudut tilt berbasis accelerometer agar kompensasi tilt
        # benar-benar mengikuti orientasi gravitasi saat sensor miring.
        roll_tilt = float(roll)
        pitch_tilt = float(pitch)
        if accX is not None and accY is not None and accZ is not None:
            try:
                ax = float(accX)
                ay = float(accY)
                az = float(accZ)
                den_roll = math.sqrt(ay * ay + az * az)
                den_pitch = math.sqrt(ax * ax + az * az)
                if den_roll > 1e-9 and den_pitch > 1e-9:
                    roll_tilt = math.degrees(math.atan2(ay, az))
                    pitch_tilt = math.degrees(math.atan2(-ax, den_roll))
            except Exception:
                pass

        # Menghitung arah utara dari Medan Magnet dengan tilt compensation
        # memakai roll/pitch dari accelerometer.
        compass = None
        if magX is not None and magY is not None and magZ is not None:
            try:
                # Konversi sudut ke radian untuk kompensasi kemiringan (tilt compensation)
                roll_rad = math.radians(roll_tilt)
                pitch_rad = math.radians(pitch_tilt)

                # Kompensasi kemiringan (menggunakan sumbu standar NED/NWD)
                # Bergantung pada sistem koordinat WT901, asumsi standar:
                X_h = magX * math.cos(pitch_rad) + magZ * math.sin(pitch_rad)
                Y_h = magX * math.sin(roll_rad) * math.sin(pitch_rad) + magY * math.cos(roll_rad) - magZ * math.sin(roll_rad) * math.cos(pitch_rad)

                # Hitung sudut arah (heading/compass)
                compass_rad = math.atan2(-Y_h, X_h)
                compass = math.degrees(compass_rad)
                
                # Normalisasi ke 0-360 derajat
                if compass < 0:
                    compass += 360
            except Exception:
                pass

        yaw_norm = float(yaw) % 360.0
        # Konversi ke arah kanan (clockwise/CW) 0..360
        # 0 -> 0, 10 -> 350, 90 -> 270, 270 -> 90
        yaw_cw = (360.0 - yaw_norm + AZ_OFFSET_DEG) % 360.0
        roll_f = float(roll)
        pitch_f = float(pitch)
        compass_f = float(compass) if compass is not None else None
        compass_cw = (360.0 - compass_f + AZ_OFFSET_DEG) % 360.0 if compass_f is not None else None

        # Sesuai catatan: YAW akurat saat level; saat tilt besar, gunakan compass tilt-compensated.
        tilt_large = abs(roll_f) > TILT_THRESHOLD_DEG or abs(pitch_f) > TILT_THRESHOLD_DEG
        if tilt_large and compass_cw is not None:
            az_used = compass_cw
            az_source = "COMPASS_CW"
        else:
            az_used = yaw_cw
            az_source = "YAW_CW"

        return roll_f, pitch_f, yaw_cw, compass_cw, az_used, az_source
    except Exception:
        return None, None, None, None, None, None


def tampilkan_header():
    print("=" * 75)
    print("  WT901C485 - Elevasi Absolut & Azimuth Kompas")
    print("=" * 75)
    print("Penjelasan sudut:")
    print("  ROLL  (X) : Kemiringan kiri-kanan [ELEVASI/EL]")
    print("  PITCH (Y) : Kemiringan depan-belakang")
    print("  YAW   (Z) : Rotasi Z (Giroskop/Fusi)")
    print("                *CATATAN: YAW/Heading WT901 hanya akurat jika sensor datar (level).")
    print("                *Jika Pitch/Roll besar, heading bisa drift tanpa Tilt Compensation.")
    print("  COMPASS   : Arah hadap kompas murni (Medan Magnet dengan Tilt Compensation)")
    print("  AZ_USED   : Azimuth final adaptif (YAW saat level, COMPASS saat tilt besar)")
    print()
    print("Referensi: GRAVITASI BUMI (sudut absolut)")
    print("  0°   = Sensor sejajar dengan tanah (datar)")
    print("  90°  = Sensor berdiri tegak")
    print("  -90° = Sensor terbalik tegak")
    print()
    print("Tekan Ctrl+C untuk berhenti.")
    print("-" * 75)
    print(f"{'Waktu':<12} {'ROLL/EL (°)':>14} {'PITCH (°)':>10} {'YAW (°)':>10} {'COMPASS (°)':>14} {'AZ_USED (°)':>12} {'SRC':>7}")
    print("-" * 75)


def main():
    tampilkan_header()

    # Inisialisasi koneksi ke sensor
    try:
        device = buat_device_model()
        device.ADDR = 0x50
        if platform.system().lower() == "linux":
            device.serialConfig.portName = "/dev/ttyUSB0"
        else:
            device.serialConfig.portName = "/dev/tty.usbserial-1330"
        device.serialConfig.baud = 9600
        device.openDevice()
        print(f"[OK] Terhubung ke {device.serialConfig.portName} @ {device.serialConfig.baud} baud\n")
    except Exception as e:
        print(f"[ERROR] Tidak bisa membuka port: {e}")
        print("Pastikan:")
        print("  1. Sensor terhubung ke komputer")
        print("  2. Port sudah benar (Linux: /dev/ttyUSB0 | Mac: /dev/tty.usbserial-xxxx)")
        print("  3. Tidak ada aplikasi lain yang menggunakan port ini")
        sys.exit(1)

    # Reset zero-point untuk memastikan sudut absolut
    reset_zero_point(device)

    # Loop pembacaan data
    try:
        while True:
            roll, pitch, yaw, compass, az_used, az_source = baca_sudut(device)

            if roll is None:
                print(f"[WARN] Gagal membaca data, mencoba lagi...")
            else:
                waktu = time.strftime("%H:%M:%S")

                # Interpretasi elevasi
                if abs(roll) < 5:
                    status = "DATAR"
                elif roll > 0:
                    status = f"MIRING DEPAN {roll:.1f}°"
                else:
                    status = f"MIRING BELAKANG {abs(roll):.1f}°"

                comp_str = f"{compass:>14.2f}" if compass is not None else "           N/A"
                az_used_str = f"{az_used:>12.2f}" if az_used is not None else "         N/A"
                src_str = f"{az_source:>7}" if az_source is not None else "    N/A"
                print(f"{waktu:<12} {roll:>14.2f} {pitch:>10.2f} {yaw:>10.2f} {comp_str} {az_used_str} {src_str}   [{status}]")

            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n" + "-" * 75)
        print("[INFO] Program dihentikan oleh pengguna.")

    finally:
        try:
            if hasattr(device, "closeDevice"):
                device.closeDevice()
            else:
                device.close()
            print("[INFO] Koneksi serial ditutup.")
        except Exception:
            pass


if __name__ == "__main__":
    main()
