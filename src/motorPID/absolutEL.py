"""
Baca Elevasi Absolut Terhadap Gravitasi - WT901C485
=====================================================
Script ini membaca sudut EL (Elevation/Pitch) dari sensor WT901C485
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

# PYTHON PATH -> folder chs lokal project
# =====================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.abspath(os.path.join(BASE_DIR, "..", "Python-SDK-WT901C485", "chs"))
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

# ─── Konstanta Reset Zero-Point ───────────────────────────────────────────
REG_KEY   = 0x69          # Register kunci untuk operasi tulis

def reset_zero_point(device):
    """
    Menghapus zero-point yang tersimpan di sensor.
    Ini memastikan EL selalu mengacu pada gravitasi (sudut absolut),
    bukan posisi saat dinyalakan.
    """
    print("[INFO] Mereset zero-point ke default (sudut absolut)...")
    try:
        # Unlock register untuk penulisan
        device.write_register(device.ADDR, REG_KEY, 0xB588)
        time.sleep(0.1)
        device.write_register(device.ADDR, 0x01, 0x0000)
        time.sleep(0.3)

        print("[OK] Zero-point berhasil direset. Sensor sekarang menggunakan gravitasi sebagai referensi.\n")
    except Exception as e:
        print(f"[WARN] Gagal reset zero-point: {e}")
        print("[WARN] Melanjutkan tanpa reset (sudut mungkin memiliki offset).\n")


def baca_sudut(device):
    """
    Membaca sudut Roll, Pitch (EL), dan Yaw dari sensor.
    Mengembalikan tuple (roll, pitch, yaw) dalam derajat.
    Menggunakan get() dari deviceModel yang sudah di-parse oleh JY901SDataProcessor.
    """
    try:
        roll  = device.get("AngleX")   # Roll  (kemiringan kiri-kanan)
        pitch = device.get("AngleY")   # Pitch (elevasi depan-belakang)
        yaw   = device.get("AngleZ")   # Yaw   (rotasi horizontal)

        if roll is None or pitch is None or yaw is None:
            return None, None, None

        return float(roll), float(pitch), float(yaw)
    except Exception:
        return None, None, None


def tampilkan_header():
    print("=" * 60)
    print("  WT901C485 - Elevasi Absolut Terhadap Gravitasi")
    print("=" * 60)
    print("Penjelasan sudut:")
    print("  ROLL  (X) : Kemiringan kiri-kanan")
    print("  PITCH (Y) : Kemiringan depan-belakang [ELEVASI/EL]")
    print("  YAW   (Z) : Rotasi horizontal (kompas)")
    print()
    print("Referensi: GRAVITASI BUMI (sudut absolut)")
    print("  0°   = Sensor sejajar dengan tanah (datar)")
    print("  90°  = Sensor berdiri tegak")
    print("  -90° = Sensor terbalik tegak")
    print()
    print("Tekan Ctrl+C untuk berhenti.")
    print("-" * 60)
    print(f"{'Waktu':<12} {'ROLL (°)':>10} {'PITCH/EL (°)':>14} {'YAW (°)':>10}")
    print("-" * 60)


def main():
    tampilkan_header()

    # Inisialisasi koneksi ke sensor
    try:
        device = deviceModel.DeviceModel(
            "WT901C485",
            Protocol485Resolver(),
            JY901SDataProcessor()
        )
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
            roll, pitch, yaw = baca_sudut(device)

            if pitch is None:
                print(f"[WARN] Gagal membaca data, mencoba lagi...")
            else:
                waktu = time.strftime("%H:%M:%S")

                # Interpretasi elevasi
                if abs(pitch) < 5:
                    status = "DATAR"
                elif pitch > 0:
                    status = f"MIRING DEPAN {pitch:.1f}°"
                else:
                    status = f"MIRING BELAKANG {abs(pitch):.1f}°"

                print(f"{waktu:<12} {roll:>10.2f} {pitch:>14.2f} {yaw:>10.2f}   [{status}]")

            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n" + "-" * 60)
        print("[INFO] Program dihentikan oleh pengguna.")

    finally:
        try:
            device.close()
            print("[INFO] Koneksi serial ditutup.")
        except Exception:
            pass


if __name__ == "__main__":
    main()