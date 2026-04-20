"""
Baca Elevasi Absolut & Kompas Tilt Compensated - WT901C485
===========================================================
Perbaikan dari absolutEL.py:
- Tilt compensation formula diperbaiki (AN3192 standard)
- Hard iron + soft iron correction
- Tidak ada hardcoded AZ_OFFSET
- Magnetic declination Yogyakarta (+0.9°) otomatis
- Key SDK mengikuti yang terbukti jalan (getDeviceData / angleX lowercase)

Cara pakai:
    python fix-compas.py              # baca data normal
    python fix-compas.py --kalibrasi  # kalibrasi magnetometer dulu
"""

import os
import sys
import time
import platform
import math
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "Python-SDK-WT901C485", "chs"))
sys.path.insert(0, SDK_CHS)

try:
    import lib.device_model as deviceModel
    from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
    from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver
except ImportError as e:
    print(f"[ERROR] Library WITMOTION tidak ditemukan: {e}")
    sys.exit(1)

# ─── Konfigurasi ──────────────────────────────────────────────────────────
INTERVAL           = 0.5
TILT_THRESHOLD_DEG = 15.0
REG_KEY            = 0x69

# Magnetic declination Yogyakarta (positif = east)
MAGNETIC_DECLINATION_DEG = 0.9

# ─── Kalibrasi Hard Iron ──────────────────────────────────────────────────
# Isi setelah jalankan --kalibrasi, lalu salin hasilnya ke sini
HARD_IRON_OFFSET = [0.0, 0.0, 0.0]   # [offset_x, offset_y, offset_z]

# Soft iron matrix (default identity = tidak ada koreksi)
SOFT_IRON_MATRIX = np.eye(3)


# ══════════════════════════════════════════════════════════════════════════
#  HELPER — key SDK pakai getDeviceData (lowercase) sesuai SDK ini
# ══════════════════════════════════════════════════════════════════════════

def _get(device, key):
    """
    Baca satu nilai dari sensor.
    Prioritas: getDeviceData (SDK 485) → get (SDK generik)
    """
    val = None
    try:
        val = device.getDeviceData(key)
    except Exception:
        pass
    if val is None:
        try:
            val = device.get(key)
        except Exception:
            pass
    return val


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


def tutup_device(device):
    try:
        if hasattr(device, "closeDevice"):
            device.closeDevice()
        elif hasattr(device, "close"):
            device.close()
    except Exception:
        pass


# ══════════════════════════════════════════════════════════════════════════
#  KALIBRASI MAGNETOMETER
# ══════════════════════════════════════════════════════════════════════════

def jalankan_kalibrasi(device, durasi_detik=30):
    """
    Kalibrasi hard iron & soft iron.
    Putar sensor ke semua arah (figure-8) selama durasi_detik.
    """
    global HARD_IRON_OFFSET, SOFT_IRON_MATRIX

    print("\n" + "=" * 60)
    print("  MODE KALIBRASI MAGNETOMETER")
    print("=" * 60)
    print(f"Putar sensor figure-8 di semua bidang selama {durasi_detik} detik.")
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
            print(f"\r  Sampel: {len(samples):4d} | Sisa: {sisa:5.1f}s | ({float(mx):.1f}, {float(my):.1f}, {float(mz):.1f})", end="")

        time.sleep(0.05)

    print(f"\n\n[INFO] Sampel terkumpul: {len(samples)}")

    if len(samples) < 50:
        print("[WARN] Sampel kurang, kalibrasi tidak valid.")
        return

    data = np.array(samples)

    # Hard iron: midpoint min-max
    hi = np.array([
        (data[:, 0].max() + data[:, 0].min()) / 2.0,
        (data[:, 1].max() + data[:, 1].min()) / 2.0,
        (data[:, 2].max() + data[:, 2].min()) / 2.0,
    ])

    # Soft iron: normalisasi radius per sumbu
    dc = data - hi
    radius = np.array([
        (dc[:, 0].max() - dc[:, 0].min()) / 2.0,
        (dc[:, 1].max() - dc[:, 1].min()) / 2.0,
        (dc[:, 2].max() - dc[:, 2].min()) / 2.0,
    ])
    avg_r  = np.mean(radius)
    scale  = avg_r / np.where(radius > 1e-9, radius, 1e-9)
    si_mat = np.diag(scale)

    HARD_IRON_OFFSET = hi.tolist()
    SOFT_IRON_MATRIX = si_mat

    print("\n[OK] Kalibrasi selesai! Salin nilai berikut ke kode:\n")
    print(f"HARD_IRON_OFFSET = [{hi[0]:.4f}, {hi[1]:.4f}, {hi[2]:.4f}]")
    print(f"SOFT_IRON_MATRIX = np.diag([{scale[0]:.4f}, {scale[1]:.4f}, {scale[2]:.4f}])\n")


# ══════════════════════════════════════════════════════════════════════════
#  KOREKSI & TILT COMPENSATION
# ══════════════════════════════════════════════════════════════════════════

def koreksi_mag(mx, my, mz):
    """Hard iron + soft iron correction."""
    v = np.array([mx, my, mz]) - np.array(HARD_IRON_OFFSET)
    v = SOFT_IRON_MATRIX @ v
    return v[0], v[1], v[2]


def hitung_compass_tc(roll_deg, pitch_deg, mx, my, mz):
    """
    Compass heading dengan tilt compensation (STMicroelectronics AN3192).

    Konvensi WT901:
        angleX = Roll  (kiri-kanan)
        angleY = Pitch (depan-belakang, nose-up = positif)
        magX/Y/Z = raw magnetometer dalam body frame

    Rotation ke horizontal plane:
        Bx_h =  Bx·cos(P)  +  Bz·sin(P)
        By_h =  Bx·sin(R)·sin(P)  +  By·cos(R)  -  Bz·sin(R)·cos(P)

    Heading (CW dari North):
        heading = atan2(-By_h, Bx_h)
    """
    mx_c, my_c, mz_c = koreksi_mag(mx, my, mz)

    R = math.radians(roll_deg)
    P = math.radians(pitch_deg)

    Bx_h = mx_c * math.cos(P) + mz_c * math.sin(P)
    By_h = (mx_c * math.sin(R) * math.sin(P)
            + my_c * math.cos(R)
            - mz_c * math.sin(R) * math.cos(P))

    heading = math.degrees(math.atan2(-By_h, Bx_h))
    heading += MAGNETIC_DECLINATION_DEG
    heading %= 360.0
    return heading


# ══════════════════════════════════════════════════════════════════════════
#  BACA SUDUT
# ══════════════════════════════════════════════════════════════════════════

def baca_sudut(device):
    try:
        # Trigger baca register (wajib untuk SDK 485)
        if hasattr(device, "readReg"):
            device.readReg(0x30, 41)
        time.sleep(0.05)

        # Key lowercase sesuai SDK yang terbukti jalan
        roll  = _get(device, "angleX")
        pitch = _get(device, "angleY")
        yaw   = _get(device, "angleZ")
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

        # Roll/Pitch dari accelerometer (lebih akurat untuk tilt comp)
        roll_acc  = roll_f
        pitch_acc = pitch_f
        if None not in (accX, accY, accZ):
            try:
                ax, ay, az = float(accX), float(accY), float(accZ)
                den = math.sqrt(ay**2 + az**2)
                if abs(az) > 1e-9 and den > 1e-9:
                    roll_acc  = math.degrees(math.atan2(ay, az))
                    pitch_acc = math.degrees(math.atan2(-ax, den))
            except Exception:
                pass

        # Compass tilt compensated
        compass_tc = None
        if None not in (magX, magY, magZ):
            try:
                compass_tc = hitung_compass_tc(
                    roll_acc, pitch_acc,
                    float(magX), float(magY), float(magZ)
                )
            except Exception:
                pass

        # Yaw dari fusi WT901 (CCW) → konversi ke CW 0..360
        yaw_cw = (360.0 - (yaw_f % 360.0)) % 360.0

        # Pilih heading adaptif
        tilt_besar = abs(roll_f) > TILT_THRESHOLD_DEG or abs(pitch_f) > TILT_THRESHOLD_DEG
        if tilt_besar and compass_tc is not None:
            az_used   = compass_tc
            az_source = "COMPASS_TC"
        else:
            az_used   = yaw_cw
            az_source = "YAW_FUSI "

        return roll_f, pitch_f, yaw_cw, compass_tc, az_used, az_source

    except Exception as e:
        return None, None, None, None, None, None


# ══════════════════════════════════════════════════════════════════════════
#  TAMPILAN
# ══════════════════════════════════════════════════════════════════════════

MATA_ANGIN = ["U  ", "UTL", "TL ", "TTL", "T  ", "TTG", "TG ", "UTG",
              "S  ", "STG", "BD ", "BBD", "B  ", "BBL", "BL ", "SBL"]

def arah(deg):
    return MATA_ANGIN[int((deg + 11.25) / 22.5) % 16]


def tampilkan_header():
    print("=" * 85)
    print("  WT901C485 - Elevasi Absolut & Kompas Tilt Compensated")
    print("=" * 85)
    print("  ROLL  (X) : Kemiringan kiri-kanan [ELEVASI]")
    print("  PITCH (Y) : Kemiringan depan-belakang")
    print("  YAW_FUSI  : Heading fusi WT901 (akurat saat datar)")
    print("  COMPASS_TC: Heading magnetometer + Tilt Compensation (akurat saat miring)")
    print("  AZ_USED   : Heading final adaptif")
    print()
    if HARD_IRON_OFFSET == [0.0, 0.0, 0.0]:
        print("  [!] BELUM KALIBRASI! Jalankan: python fix-compas.py --kalibrasi")
        print("      Compass mungkin tidak akurat tanpa kalibrasi.")
    else:
        print("  [OK] Kalibrasi magnetometer aktif.")
    print()
    print("Ctrl+C untuk berhenti.")
    print("-" * 85)
    print(f"{'Waktu':<10} {'ROLL(°)':>9} {'PITCH(°)':>9} {'YAW_FUSI':>10} "
          f"{'COMPASS_TC':>12} {'AZ_USED':>10} {'ARAH':>5} {'SRC':>11}")
    print("-" * 85)


# ══════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════

def main():
    mode_kalibrasi = "--kalibrasi" in sys.argv or "-k" in sys.argv

    tampilkan_header()

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

    if mode_kalibrasi:
        jalankan_kalibrasi(device, durasi_detik=30)
        print("[INFO] Selesai kalibrasi. Salin nilai ke kode, lalu jalankan normal.")
        tutup_device(device)
        return

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
                    status = f"MRG-KA {roll:.1f}°"
                else:
                    status = f"MRG-KI {abs(roll):.1f}°"

                comp_str = f"{compass:>12.2f}" if compass is not None else "          N/A"
                az_str   = f"{az_used:>10.2f}" if az_used  is not None else "        N/A"
                ar_str   = arah(az_used)        if az_used  is not None else "---"
                src_str  = az_source if az_source else "N/A"

                print(f"{waktu:<10} {roll:>9.2f} {pitch:>9.2f} {yaw:>10.2f} "
                      f"{comp_str} {az_str} {ar_str:>5} {src_str:>11}  [{status}]")

            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n" + "-" * 85)
        print("[INFO] Program dihentikan.")

    finally:
        tutup_device(device)
        print("[INFO] Koneksi serial ditutup.")


if __name__ == "__main__":
    main()