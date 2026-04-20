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
    pip install pyserial numpy
"""

import os
import sys
import time
import platform
import math
import numpy as np

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
    sys.exit(1)

# ─── Konfigurasi ──────────────────────────────────────────────────────────
INTERVAL            = 0.5     # Interval baca dalam detik
TILT_THRESHOLD_DEG  = 15.0    # Ambang tilt untuk beralih dari YAW ke COMPASS
REG_KEY             = 0x69    # Register kunci untuk operasi tulis

# ─── Kalibrasi Hard Iron ──────────────────────────────────────────────────
# Diisi otomatis saat menjalankan mode kalibrasi (tekan 'k' saat program jalan)
# Format: [offset_x, offset_y, offset_z]
HARD_IRON_OFFSET = [0.0, 0.0, 0.0]

# ─── Kalibrasi Soft Iron (opsional, default identity matrix) ──────────────
# Diisi otomatis saat menjalankan mode kalibrasi
SOFT_IRON_MATRIX = np.eye(3)

# ─── Magnetic Declination (opsional, default 0) ───────────────────────────
# Cek deklinasi magnetik untuk lokasi kamu di: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
# Yogyakarta: sekitar +0.9° (positif = east)
MAGNETIC_DECLINATION_DEG = 0.9


# ══════════════════════════════════════════════════════════════════════════════
#  KALIBRASI HARD IRON & SOFT IRON
# ══════════════════════════════════════════════════════════════════════════════

def jalankan_kalibrasi(device, durasi_detik=30):
    """
    Kalibrasi hard iron dengan mengumpulkan data magnetometer saat sensor
    diputar ke semua arah (minimal 1 putaran penuh di setiap sumbu).
    
    Hard iron offset = pusat ellipsoid dari titik-titik data mag.
    Soft iron matrix = normalisasi ellipsoid ke sphere.
    
    Hasilnya DISIMPAN ke variabel global dan dicetak untuk disalin ke kode.
    """
    global HARD_IRON_OFFSET, SOFT_IRON_MATRIX

    print("\n" + "=" * 60)
    print("  MODE KALIBRASI MAGNETOMETER")
    print("=" * 60)
    print(f"Putar sensor ke SEMUA ARAH selama {durasi_detik} detik.")
    print("Gerakkan seperti angka 8 di udara (figure-8) di semua bidang.")
    print("Mulai sekarang...\n")

    samples = []
    t_start = time.time()

    while time.time() - t_start < durasi_detik:
        sisa = durasi_detik - (time.time() - t_start)
        if hasattr(device, "readReg"):
            device.readReg(0x30, 41)

        mx = _get(device, "magX")
        my = _get(device, "magY")
        mz = _get(device, "magZ")

        if mx is not None and my is not None and mz is not None:
            samples.append([float(mx), float(my), float(mz)])
            print(f"\r  Sampel: {len(samples):4d}  | Sisa: {sisa:5.1f}s  | mag=({mx:.1f}, {my:.1f}, {mz:.1f})", end="")

        time.sleep(0.05)

    print(f"\n\n[INFO] Total sampel terkumpul: {len(samples)}")

    if len(samples) < 50:
        print("[WARN] Sampel terlalu sedikit, kalibrasi tidak valid.")
        return

    data = np.array(samples)

    # ── Hard Iron: offset = titik tengah (midpoint min-max per sumbu) ──────
    hi_offset = np.array([
        (data[:, 0].max() + data[:, 0].min()) / 2.0,
        (data[:, 1].max() + data[:, 1].min()) / 2.0,
        (data[:, 2].max() + data[:, 2].min()) / 2.0,
    ])

    # ── Soft Iron: normalisasi radius per sumbu ke sphere ──────────────────
    data_corrected = data - hi_offset
    radius = np.array([
        (data_corrected[:, 0].max() - data_corrected[:, 0].min()) / 2.0,
        (data_corrected[:, 1].max() - data_corrected[:, 1].min()) / 2.0,
        (data_corrected[:, 2].max() - data_corrected[:, 2].min()) / 2.0,
    ])
    avg_radius = np.mean(radius)
    scale = avg_radius / radius  # scale per sumbu
    si_matrix = np.diag(scale)   # diagonal soft iron matrix

    HARD_IRON_OFFSET = hi_offset.tolist()
    SOFT_IRON_MATRIX = si_matrix

    print("\n[OK] Kalibrasi selesai!")
    print(f"  Hard Iron Offset : [{hi_offset[0]:.4f}, {hi_offset[1]:.4f}, {hi_offset[2]:.4f}]")
    print(f"  Soft Iron Matrix :")
    print(f"    [{si_matrix[0,0]:.4f},  0.0000,  0.0000]")
    print(f"    [ 0.0000, {si_matrix[1,1]:.4f},  0.0000]")
    print(f"    [ 0.0000,  0.0000, {si_matrix[2,2]:.4f}]")
    print("\n  Salin nilai di atas ke variabel HARD_IRON_OFFSET dan SOFT_IRON_MATRIX")
    print("  di bagian atas kode untuk hasil permanen.\n")


# ══════════════════════════════════════════════════════════════════════════════
#  HELPER
# ══════════════════════════════════════════════════════════════════════════════

def _get(device, key):
    """Helper baca data dari device (support dua versi SDK)."""
    try:
        if hasattr(device, "get"):
            return device.get(key)
        return device.getDeviceData(key)
    except Exception:
        return None


def buat_device_model():
    try:
        return deviceModel.DeviceModel(
            "WT901C485", Protocol485Resolver(), JY901SDataProcessor()
        )
    except TypeError:
        return deviceModel.DeviceModel(
            "WT901C485", Protocol485Resolver(), JY901SDataProcessor(), "EL_0"
        )


def reset_zero_point(device):
    print("[INFO] Mereset zero-point ke default (sudut absolut)...")
    try:
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
        print("[OK] Zero-point direset. Sensor menggunakan gravitasi sebagai referensi.\n")
    except Exception as e:
        print(f"[WARN] Gagal reset zero-point: {e}\n")


# ══════════════════════════════════════════════════════════════════════════════
#  KOREKSI MAGNETOMETER
# ══════════════════════════════════════════════════════════════════════════════

def koreksi_magnetometer(mx, my, mz):
    """
    Terapkan hard iron + soft iron correction ke data magnetometer mentah.
    
    1. Hard iron  : hilangkan offset permanen (besi statis di dekat sensor)
    2. Soft iron  : normalisasi ellipsoid → sphere (besi lunak / distorsi)
    
    Hasilnya adalah vektor medan magnet yang sudah bersih.
    """
    raw = np.array([mx, my, mz])
    corrected = raw - np.array(HARD_IRON_OFFSET)       # Hard iron
    corrected = SOFT_IRON_MATRIX @ corrected            # Soft iron
    return corrected[0], corrected[1], corrected[2]


# ══════════════════════════════════════════════════════════════════════════════
#  TILT COMPENSATION COMPASS
# ══════════════════════════════════════════════════════════════════════════════

def hitung_compass_tilt_compensated(roll_deg, pitch_deg, mx, my, mz):
    """
    Menghitung heading kompas dengan tilt compensation yang benar.

    Referensi: STMicroelectronics AN3192 - Tilt Compensated Compass

    Sumbu WT901:
        angleX = Roll  (rotasi sumbu X, kiri-kanan)
        angleY = Pitch (rotasi sumbu Y, depan-belakang)
        angleZ = Yaw   (rotasi sumbu Z, atas-bawah)

    Konvensi positif WT901:
        Roll  positif = sisi kanan sensor turun
        Pitch positif = depan sensor naik (nose up)

    Langkah:
        1. Koreksi hard iron + soft iron
        2. Rotasi vektor mag dari body frame → NED horizontal plane
           menggunakan rotation matrix Ry(pitch) * Rx(roll)
        3. Hitung heading dari komponen horizontal
        4. Tambahkan magnetic declination
    """
    # Koreksi hard/soft iron
    mx_c, my_c, mz_c = koreksi_magnetometer(mx, my, mz)

    # Konversi ke radian
    roll_r  = math.radians(roll_deg)
    pitch_r = math.radians(pitch_deg)

    # ── Tilt compensation (rotasi ke horizontal plane) ─────────────────────
    # Menggunakan rotation matrix standar:
    #   Bx_h = Bx*cos(pitch) + Bz*sin(pitch)
    #   By_h = Bx*sin(roll)*sin(pitch) + By*cos(roll) - Bz*sin(roll)*cos(pitch)
    #
    # Catatan: sign pada By_h mengikuti konvensi WT901 di mana
    # Roll (+) = sisi kanan turun (sama dengan NED right-hand convention)

    Bx_h = (mx_c * math.cos(pitch_r)
            + mz_c * math.sin(pitch_r))

    By_h = (mx_c * math.sin(roll_r) * math.sin(pitch_r)
            + my_c * math.cos(roll_r)
            - mz_c * math.sin(roll_r) * math.cos(pitch_r))

    # ── Heading ───────────────────────────────────────────────────────────
    # atan2(-By, Bx) → arah utara magnetik (CW dari North = positif)
    heading_rad = math.atan2(-By_h, Bx_h)
    heading_deg = math.degrees(heading_rad)

    # Tambahkan deklinasi magnetik
    heading_deg += MAGNETIC_DECLINATION_DEG

    # Normalisasi 0..360
    heading_deg %= 360.0

    return heading_deg


# ══════════════════════════════════════════════════════════════════════════════
#  BACA SUDUT UTAMA
# ══════════════════════════════════════════════════════════════════════════════

def baca_sudut(device):
    """
    Membaca Roll, Pitch, Yaw (fusi onboard), dan menghitung heading kompas
    dengan tilt compensation penuh.

    Return: (roll, pitch, yaw_fusi, compass_tc, az_used, az_source)
        - roll/pitch     : dari sensor langsung (absolut vs gravitasi)
        - yaw_fusi       : yaw dari algoritma fusi WT901 (akurat saat level)
        - compass_tc     : heading kompas dengan tilt compensation (dari magnetometer)
        - az_used        : heading final adaptif
        - az_source      : sumber yang dipakai ("YAW_FUSI" atau "COMPASS_TC")
    """
    try:
        if hasattr(device, "readReg"):
            device.readReg(0x30, 41)

        roll  = _get(device, "AngleX")
        pitch = _get(device, "AngleY")
        yaw   = _get(device, "AngleZ")
        accX  = _get(device, "accX")
        accY  = _get(device, "accY")
        accZ  = _get(device, "accZ")
        magX  = _get(device, "magX")
        magY  = _get(device, "magY")
        magZ  = _get(device, "magZ")

        if roll is None or pitch is None or yaw is None:
            return None, None, None, None, None, None

        roll_f  = float(roll)
        pitch_f = float(pitch)
        yaw_f   = float(yaw)

        # ── Roll/Pitch dari accelerometer (lebih akurat untuk tilt compensation) ──
        roll_acc = roll_f
        pitch_acc = pitch_f
        if accX is not None and accY is not None and accZ is not None:
            try:
                ax, ay, az = float(accX), float(accY), float(accZ)
                # Rumus accelerometer-based roll/pitch (NED convention)
                roll_acc  = math.degrees(math.atan2(ay, az))
                pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
            except Exception:
                pass

        # ── Compass dengan Tilt Compensation ──────────────────────────────────
        compass_tc = None
        if magX is not None and magY is not None and magZ is not None:
            try:
                compass_tc = hitung_compass_tilt_compensated(
                    roll_acc, pitch_acc,
                    float(magX), float(magY), float(magZ)
                )
            except Exception:
                pass

        # ── Yaw dari fusi WT901 (normalisasi 0..360 CW) ───────────────────────
        # WT901 yaw: CCW positif → konversi ke CW (heading konvensional)
        yaw_cw = (360.0 - (yaw_f % 360.0)) % 360.0

        # ── Pilih sumber heading adaptif ──────────────────────────────────────
        tilt_besar = abs(roll_f) > TILT_THRESHOLD_DEG or abs(pitch_f) > TILT_THRESHOLD_DEG
        if tilt_besar and compass_tc is not None:
            az_used   = compass_tc
            az_source = "COMPASS_TC"
        else:
            az_used   = yaw_cw
            az_source = "YAW_FUSI"

        return roll_f, pitch_f, yaw_cw, compass_tc, az_used, az_source

    except Exception:
        return None, None, None, None, None, None


# ══════════════════════════════════════════════════════════════════════════════
#  TAMPILAN
# ══════════════════════════════════════════════════════════════════════════════

def arah_mata_angin(derajat):
    """Konversi derajat ke nama arah mata angin (16 arah)."""
    arah = ["U", "UTL", "TL", "TTL", "T", "TTG", "TG", "UTG",
            "S", "STG", "BD", "BBD", "B", "BBL", "BL", "SBL"]
    idx = int((derajat + 11.25) / 22.5) % 16
    return arah[idx]


def tampilkan_header():
    print("=" * 85)
    print("  WT901C485 - Elevasi Absolut & Azimuth Kompas (Tilt Compensated)")
    print("=" * 85)
    print("Penjelasan:")
    print("  ROLL  (X)  : Kemiringan kiri-kanan [ELEVASI/EL]")
    print("  PITCH (Y)  : Kemiringan depan-belakang")
    print("  YAW_FUSI   : Heading dari algoritma fusi WT901 (akurat saat datar)")
    print("  COMPASS_TC : Heading dari magnetometer + Tilt Compensation penuh")
    print("  AZ_USED    : Heading final adaptif (YAW saat datar, COMPASS_TC saat miring)")
    print()
    print("  [!] Jalankan kalibrasi (Ctrl+K) sebelum pakai untuk compass akurat!")
    print("  [!] Magnetic declination Yogyakarta: +0.9° (sudah diterapkan)")
    print()
    print("Tekan Ctrl+C untuk berhenti. Tekan Ctrl+K untuk kalibrasi magnetometer.")
    print("-" * 85)
    print(f"{'Waktu':<10} {'ROLL(°)':>9} {'PITCH(°)':>9} {'YAW_FUSI':>10} {'COMPASS_TC':>12} {'AZ_USED':>10} {'ARAH':>6} {'SRC':>10}")
    print("-" * 85)


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    # Cek apakah mode kalibrasi diminta via argumen
    mode_kalibrasi = "--kalibrasi" in sys.argv or "-k" in sys.argv

    tampilkan_header()

    # ── Inisialisasi koneksi sensor ────────────────────────────────────────
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
        sys.exit(1)

    reset_zero_point(device)

    # ── Mode kalibrasi ─────────────────────────────────────────────────────
    if mode_kalibrasi:
        jalankan_kalibrasi(device, durasi_detik=30)
        print("[INFO] Kalibrasi selesai. Jalankan ulang program tanpa --kalibrasi untuk mulai baca data.")
        device.closeDevice() if hasattr(device, "closeDevice") else device.close()
        return

    # ── Loop pembacaan ─────────────────────────────────────────────────────
    try:
        while True:
            roll, pitch, yaw, compass, az_used, az_source = baca_sudut(device)

            if roll is None:
                print("[WARN] Gagal membaca data, mencoba lagi...")
            else:
                waktu = time.strftime("%H:%M:%S")

                if abs(roll) < 5:
                    status = "DATAR"
                elif roll > 0:
                    status = f"MIRING KA {roll:.1f}°"
                else:
                    status = f"MIRING KI {abs(roll):.1f}°"

                comp_str   = f"{compass:>12.2f}" if compass is not None else "          N/A"
                az_str     = f"{az_used:>10.2f}"  if az_used  is not None else "        N/A"
                arah_str   = arah_mata_angin(az_used) if az_used is not None else "---"
                src_str    = f"{az_source:>10}"

                print(f"{waktu:<10} {roll:>9.2f} {pitch:>9.2f} {yaw:>10.2f} {comp_str} {az_str} {arah_str:>6} {src_str}   [{status}]")

            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n" + "-" * 85)
        print("[INFO] Program dihentikan.")

    finally:
        try:
            device.closeDevice() if hasattr(device, "closeDevice") else device.close()
            print("[INFO] Koneksi serial ditutup.")
        except Exception:
            pass


if __name__ == "__main__":
    main()