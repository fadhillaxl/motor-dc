"""
Baca 2 sensor WT901C485 pada satu bus RS485.

- 0x50: sensor EL (elevasi/pitch)
- 0x51: sensor AZ (compass/yaw) — dengan kalibrasi hard-iron + tilt compensation

Fitur:
  ✓ Hard-iron calibration (offset X/Y/Z) — disimpan ke file, persistent mati/nyala
  ✓ Soft-iron calibration (skala X/Y/Z) — kompensasi elips jadi lingkaran
  ✓ Tilt compensation — heading tetap benar meski sensor miring (seperti kompas HP)
  ✓ Magnetic declination — koreksi deklinasi lokasi
  ✓ Mode kalibrasi interaktif — putar sensor 360° lalu simpan otomatis
"""

import os
import sys
import time
import math
import json
import platform
import threading
import select

# ─── PATH SETUP ────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.join(BASE_DIR, "..", "..", "Python-SDK-WT901C485", "chs")
sys.path.insert(0, SDK_CHS)

# ─── Import SDK WITMOTION ──────────────────────────────────────────────────
try:
    import lib.device_model as deviceModel
    from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
    from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver
except ImportError as e:
    print(f"[ERROR] Library WITMOTION tidak ditemukan: {e}")
    sys.exit(1)

# ─── Konfigurasi ──────────────────────────────────────────────────────────
INTERVAL        = 0.1          # Interval baca (detik)
ADDR_EL         = 0x50         # Sensor elevasi
ADDR_AZ         = 0x51         # Sensor azimuth / kompas

# Deklinasi magnetik kota Anda (cek: https://www.magnetic-declination.com/)
# Yogyakarta, Indonesia ≈ +0.97° (positif = timur)
DECLINATION_DEG = 0.97
AZ_OFFSET_DEG = 0.00   # Offset azimuth manual (+/- derajat) untuk fine-tuning arah

# File penyimpanan kalibrasi (persistent lintas sesi)
CALIB_FILE = os.path.join(BASE_DIR, "compass_calibration.json")

# Konfigurasi stabilisasi heading (compass-only)
HEADING_MAX_STEP_DEG = 1.0
HEADING_OUTLIER_DEG = 35.0
HEADING_EMA_ALPHA = 0.08
HEADING_WARMUP_SAMPLES = 5

# ─── Register ─────────────────────────────────────────────────────────────
REG_KEY   = 0x69
REG_ANGLE = 0x3D    # Roll, Pitch, Yaw
REG_MAG   = 0x3A    # MagX, MagY, MagZ
REG_ACC   = 0x34    # AccX, AccY, AccZ (opsional, tidak dipakai di sini)

# ═══════════════════════════════════════════════════════════════════════════
#   KALIBRASI — Hard-iron + Soft-iron
# ═══════════════════════════════════════════════════════════════════════════

DEFAULT_CALIB = {
    # Offset hard-iron (bias magnet internal): geser pusat elips ke 0,0,0
    "offset_x": 0.0,
    "offset_y": 0.0,
    "offset_z": 0.0,
    # Skala soft-iron: normalkan jari-jari elips jadi lingkaran
    "scale_x": 1.0,
    "scale_y": 1.0,
    "scale_z": 1.0,
}


def load_calibration() -> dict:
    """Muat kalibrasi dari file. Jika tidak ada, pakai default."""
    if os.path.exists(CALIB_FILE):
        try:
            with open(CALIB_FILE, "r") as f:
                data = json.load(f)
            # Validasi key lengkap
            for k in DEFAULT_CALIB:
                if k not in data:
                    data[k] = DEFAULT_CALIB[k]
            print(f"[OK] Kalibrasi dimuat dari {CALIB_FILE}")
            print(f"     offset=({data['offset_x']:.1f}, {data['offset_y']:.1f}, {data['offset_z']:.1f})")
            print(f"     scale=({data['scale_x']:.4f}, {data['scale_y']:.4f}, {data['scale_z']:.4f})")
            return data
        except Exception as e:
            print(f"[WARN] Gagal membaca kalibrasi: {e} — pakai default")
    else:
        print(f"[WARN] File kalibrasi tidak ditemukan ({CALIB_FILE})")
        print("       Heading tetap berjalan, tapi bisa drift/noisy tanpa kalibrasi.")
        print("       Jalankan mode kalibrasi dengan argumen: python script.py --calibrate")
    return dict(DEFAULT_CALIB)


def save_calibration(calib: dict):
    """Simpan kalibrasi ke file JSON."""
    with open(CALIB_FILE, "w") as f:
        json.dump(calib, f, indent=2)
    print(f"[OK] Kalibrasi disimpan ke {CALIB_FILE}")


def compute_calibration_from_samples(samples: list) -> dict:
    """
    Hitung parameter kalibrasi dari kumpulan sampel (mx, my, mz).

    Hard-iron offset  = (max + min) / 2  per sumbu
    Soft-iron scale   = avg_radius / radius_sumbu  (normalisasi)
    """
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    zs = [s[2] for s in samples]

    offset_x = (max(xs) + min(xs)) / 2.0
    offset_y = (max(ys) + min(ys)) / 2.0
    offset_z = (max(zs) + min(zs)) / 2.0

    # Jari-jari tiap sumbu setelah dikurangi offset
    r_x = (max(xs) - min(xs)) / 2.0
    r_y = (max(ys) - min(ys)) / 2.0
    r_z = (max(zs) - min(zs)) / 2.0

    avg_r = (r_x + r_y + r_z) / 3.0

    scale_x = avg_r / r_x if r_x > 0 else 1.0
    scale_y = avg_r / r_y if r_y > 0 else 1.0
    scale_z = avg_r / r_z if r_z > 0 else 1.0

    return {
        "offset_x": offset_x,
        "offset_y": offset_y,
        "offset_z": offset_z,
        "scale_x": scale_x,
        "scale_y": scale_y,
        "scale_z": scale_z,
    }


# ═══════════════════════════════════════════════════════════════════════════
#   KONVERSI DATA RAW
# ═══════════════════════════════════════════════════════════════════════════

def _raw_to_angle(raw) -> float:
    """Register 16-bit → derajat (-180 .. +180)."""
    val = int(raw)
    if val > 32767:
        val -= 65536
    return val / 32768.0 * 180.0


def _raw_to_signed16(raw) -> int:
    """uint16 → int16."""
    val = int(raw) & 0xFFFF
    if val >= 0x8000:
        val -= 0x10000
    return val


def _normalize_0_360(deg: float) -> float:
    return float(deg) % 360.0


def _angle_diff_deg(a: float, b: float) -> float:
    """Selisih sudut terpendek a-b dalam derajat (-180..180)."""
    return (float(a) - float(b) + 180.0) % 360.0 - 180.0


def _angle_ema_deg(new_deg: float, old_deg: float, alpha: float) -> float:
    """EMA sudut yang aman di wrap 0/360."""
    if old_deg is None:
        return _normalize_0_360(new_deg)
    d = _angle_diff_deg(new_deg, old_deg)
    return _normalize_0_360(float(old_deg) + float(alpha) * d)


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(val)))


def _map_el_roll_90_180_to_0_90(roll_deg: float) -> float:
    """Roll 90–180° → EL 0–90°."""
    return _clamp(float(roll_deg) - 90.0, 0.0, 90.0)


class CompassHeadingFilter:
    """Filter heading compass-only: outlier reject + step limiter + EMA circular."""

    def __init__(self, max_step_deg: float, outlier_deg: float, ema_alpha: float, warmup_samples: int):
        self.max_step_deg = float(max_step_deg)
        self.outlier_deg = float(outlier_deg)
        self.ema_alpha = float(ema_alpha)
        self.warmup_samples = int(warmup_samples)
        self.last_output = None
        self.last_raw = None
        self.warmup_count = 0

    def hold(self):
        """Kembalikan heading terakhir saat input invalid."""
        if self.last_output is None:
            return None
        return float(self.last_output)

    def update(self, raw_heading: float) -> float:
        raw = _normalize_0_360(raw_heading)
        self.last_raw = raw

        if self.last_output is None:
            self.last_output = raw
            self.warmup_count = 1
            return self.last_output

        if self.warmup_count < self.warmup_samples:
            self.last_output = raw
            self.warmup_count += 1
            return self.last_output

        delta = _angle_diff_deg(raw, self.last_output)
        if abs(delta) > self.outlier_deg:
            return self.last_output

        limited = _normalize_0_360(self.last_output + _clamp(delta, -self.max_step_deg, self.max_step_deg))
        self.last_output = _angle_ema_deg(limited, self.last_output, self.ema_alpha)
        return self.last_output


# ═══════════════════════════════════════════════════════════════════════════
#   PEMBACAAN SENSOR
# ═══════════════════════════════════════════════════════════════════════════

def baca_sudut(device, addr):
    """
    Baca Roll, Pitch, Yaw dari sensor.
    Return: (roll, pitch, yaw) dalam derajat, atau (None, None, None) jika gagal.
    """
    try:
        device.ADDR = addr
        vals = device.readReg(REG_ANGLE, 3)
        if not vals or len(vals) < 3:
            return None, None, None
        return (
            float(_raw_to_angle(vals[0])),
            float(_raw_to_angle(vals[1])),
            float(_raw_to_angle(vals[2])),
        )
    except Exception:
        return None, None, None


def baca_magnetometer(device, addr):
    """
    Baca MagX, MagY, MagZ (raw int16) dari sensor.
    Return: (mx, my, mz) atau (None, None, None) jika gagal.
    """
    try:
        device.ADDR = addr
        vals = device.readReg(REG_MAG, 3)
        if not vals or len(vals) < 3:
            return None, None, None
        return (
            _raw_to_signed16(vals[0]),
            _raw_to_signed16(vals[1]),
            _raw_to_signed16(vals[2]),
        )
    except Exception:
        return None, None, None


# ═══════════════════════════════════════════════════════════════════════════
#   ALGORITMA HEADING — Tilt-Compensated + Hard/Soft-Iron Correction
# ═══════════════════════════════════════════════════════════════════════════

def heading_tilt_compensated(
    mx_raw, my_raw, mz_raw,
    roll_deg, pitch_deg,
    calib: dict,
) -> float:
    """
    Hitung heading kompas dengan kompensasi kemiringan sensor (seperti kompas HP).

    Langkah:
      1. Koreksi hard-iron (geser ke pusat) + soft-iron (normalisasi elips)
      2. Tilt compensation menggunakan roll & pitch dari IMU
      3. atan2 → heading 0..360° + deklinasi magnetik + offset AZ

    Args:
        mx_raw, my_raw, mz_raw : nilai magnetometer mentah (int16)
        roll_deg, pitch_deg    : kemiringan sensor dari IMU (derajat)
        calib                  : dict kalibrasi (offset + scale)

    Returns:
        Heading 0..360 derajat terhadap Utara Magnetik
    """
    # ── 1. Koreksi Hard-iron & Soft-iron ────────────────────────────────
    mx = (float(mx_raw) - calib["offset_x"]) * calib["scale_x"]
    my = (float(my_raw) - calib["offset_y"]) * calib["scale_y"]
    mz = (float(mz_raw) - calib["offset_z"]) * calib["scale_z"]

    # ── 2. Tilt Compensation ─────────────────────────────────────────────
    # Referensi: https://www.nxp.com/docs/en/application-note/AN4248.pdf
    #
    # Sistem koordinat WT901C:
    #   Roll  = rotasi di sumbu X (kiri-kanan)
    #   Pitch = rotasi di sumbu Y (depan-belakang)
    #
    # Tanpa kompensasi ini, heading berubah saat sensor miring → tidak akurat!
    roll  = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)

    # Proyeksi magnetometer ke bidang horizontal virtual
    #   Xh = Mx·cos(pitch) + Mz·sin(pitch)
    #   Yh = Mx·sin(roll)·sin(pitch) + My·cos(roll) − Mz·sin(roll)·cos(pitch)
    cos_roll  = math.cos(roll)
    sin_roll  = math.sin(roll)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)

    xh = mx * cos_pitch + mz * sin_pitch
    yh = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch

    # ── 3. Hitung Heading ─────────────────────────────────────────────────
    #   atan2(Yh, Xh) → sudut dari Utara
    #   Konvensi: Utara=0°, Timur=90°, Selatan=180°, Barat=270°
    heading = math.degrees(math.atan2(yh, xh))

    # ── 4. Koreksi Deklinasi + Offset AZ ──────────────────────────────────
    heading += DECLINATION_DEG
    heading += AZ_OFFSET_DEG

    return _normalize_0_360(heading)


# ═══════════════════════════════════════════════════════════════════════════
#   RESET ZERO-POINT
# ═══════════════════════════════════════════════════════════════════════════

def reset_zero_point(device, addr):
    """Reset zero-point sensor → sudut mengacu ke gravitasi (absolut)."""
    print(f"[INFO] Reset zero-point sensor {hex(addr)}...")
    try:
        device.ADDR = addr
        device.writeReg(REG_KEY, 0xB588)
        time.sleep(0.1)
        device.writeReg(0x01, 0x0000)
        time.sleep(0.3)
        print(f"[OK]   Zero-point {hex(addr)} direset.\n")
    except Exception as e:
        print(f"[WARN] Gagal reset zero-point {hex(addr)}: {e}\n")


# ═══════════════════════════════════════════════════════════════════════════
#   MODE KALIBRASI INTERAKTIF
# ═══════════════════════════════════════════════════════════════════════════

def run_calibration_mode(device):
    """
    Mode kalibrasi hard-iron + soft-iron.

    Pengguna diminta memutar sensor AZ (0x51) perlahan ke semua arah
    selama ±30 detik (pola angka 8 / figure-8).
    Program mengumpulkan sampel min/max lalu menghitung & menyimpan kalibrasi.
    """
    print()
    print("=" * 60)
    print("  MODE KALIBRASI KOMPAS")
    print("=" * 60)
    print("""
Instruksi:
  1. Pegang sensor AZ (0x51) di tangan.
  2. Putar perlahan membentuk ANGKA 8 di udara (figure-8),
     miring ke kiri, kanan, atas, bawah — semua arah.
  3. Lakukan selama ~30 detik hingga hitungan mundur selesai.
  4. Kalibrasi akan disimpan otomatis ke file JSON.

Tekan ENTER untuk mulai...
""")
    input()

    try:
        device.ADDR = ADDR_AZ
    except Exception:
        pass

    DURASI = 30   # detik
    samples = []

    print(f"[▶] Mulai kalibrasi — putar sensor sekarang! ({DURASI} detik)")
    t_start = time.time()
    counter = 0

    while True:
        elapsed = time.time() - t_start
        sisa = DURASI - elapsed
        if sisa <= 0:
            break

        mx, my, mz = baca_magnetometer(device, ADDR_AZ)
        if mx is not None:
            samples.append((mx, my, mz))
            counter += 1

        # Tampilkan progress
        bar = "█" * int((elapsed / DURASI) * 30)
        print(f"\r  [{bar:<30}] {sisa:.0f}s tersisa  |  {counter} sampel", end="", flush=True)
        time.sleep(0.1)

    print(f"\n\n[OK] Selesai. Total sampel: {len(samples)}")

    if len(samples) < 20:
        print("[ERROR] Sampel terlalu sedikit. Coba lagi.")
        return

    calib = compute_calibration_from_samples(samples)

    print(f"""
Hasil Kalibrasi:
  Hard-iron offset:
    X = {calib['offset_x']:.2f}
    Y = {calib['offset_y']:.2f}
    Z = {calib['offset_z']:.2f}
  Soft-iron scale:
    X = {calib['scale_x']:.4f}
    Y = {calib['scale_y']:.4f}
    Z = {calib['scale_z']:.4f}
""")

    save_calibration(calib)
    print("Kalibrasi selesai! Jalankan script tanpa argumen untuk mulai membaca.\n")


# ═══════════════════════════════════════════════════════════════════════════
#   TAMPILAN
# ═══════════════════════════════════════════════════════════════════════════

def arah_mata_angin(heading: float) -> str:
    """Konversi heading derajat → nama mata angin (N/NE/E/SE/S/SW/W/NW)."""
    arah = ["U", "UL", "T", "TG", "S", "BD", "B", "UB"]
    full = ["Utara", "Utara-Timur", "Timur", "Timur-Selatan",
            "Selatan", "Barat-Selatan", "Barat", "Barat-Utara"]
    idx = int((heading + 22.5) / 45.0) % 8
    return f"{arah[idx]} ({full[idx]})"


def tampilkan_header():
    print("=" * 70)
    print("  WT901C485 — Dual Sensor | Kompas Tilt-Compensated")
    print(f"  Deklinasi: {DECLINATION_DEG:+.2f}°  |  AZ Offset: {AZ_OFFSET_DEG:+.2f}°")
    print(f"  Kalibrasi: {CALIB_FILE}")
    print("=" * 70)
    print(f"  EL sensor: {hex(ADDR_EL)}  |  AZ/Kompas sensor: {hex(ADDR_AZ)}")
    print()
    print("  EL_FROM_ROLL : Elevasi dari roll (0°=datar, 90°=tegak)")
    print("  HEADING      : Arah kompas 0..360° (Utara=0°, Timur=90°)")
    print()
    print("  Tekan Ctrl+C untuk berhenti.")
    print("-" * 70)
    print(
        f"{'Waktu':<10} "
        f"{'EL(°)':>8} "
        f"{'HEADING(°)':>12} "
        f"{'ARAH':>20} "
        f"{'SRC':>6} "
        f"{'ΔHDG':>8} "
        f"{'EL_ROLL(°)':>11} "
        f"{'AZ_ROLL(°)':>11}"
    )
    print("-" * 70)


# ═══════════════════════════════════════════════════════════════════════════
#   MAIN
# ═══════════════════════════════════════════════════════════════════════════

def buka_device():
    """Inisialisasi dan buka koneksi serial ke sensor."""
    try:
        device = deviceModel.DeviceModel(
            "WT901C485-DUAL",
            Protocol485Resolver(),
            JY901SDataProcessor(),
            lambda *_: None,
        )
        device.ADDR = ADDR_EL
        if platform.system().lower() == "linux":
            device.serialConfig.portName = "/dev/ttyUSB0"
        else:
            device.serialConfig.portName = "/dev/tty.usbserial-1330"
        device.serialConfig.baud = 9600
        device.openDevice()
        print(f"[OK] Terhubung ke {device.serialConfig.portName} @ {device.serialConfig.baud} baud\n")
        return device
    except Exception as e:
        print(f"[ERROR] Tidak bisa membuka port: {e}")
        print("Pastikan sensor terhubung dan port benar.")
        sys.exit(1)


def main():
    # ── Cek mode kalibrasi ──────────────────────────────────────────────
    if len(sys.argv) > 1 and sys.argv[1] == "--calibrate":
        device = buka_device()
        run_calibration_mode(device)
        try:
            device.closeDevice()
        except Exception:
            pass
        return

    # ── Muat kalibrasi dari file ────────────────────────────────────────
    calib = load_calibration()
    print()

    # ── Buka device ────────────────────────────────────────────────────
    device = buka_device()

    # ── Reset zero-point kedua sensor ──────────────────────────────────
    reset_zero_point(device, ADDR_EL)
    reset_zero_point(device, ADDR_AZ)

    tampilkan_header()
    heading_filter = CompassHeadingFilter(
        max_step_deg=HEADING_MAX_STEP_DEG,
        outlier_deg=HEADING_OUTLIER_DEG,
        ema_alpha=HEADING_EMA_ALPHA,
        warmup_samples=HEADING_WARMUP_SAMPLES,
    )
    prev_heading = None

    # ── Loop pembacaan ─────────────────────────────────────────────────
    try:
        while True:
            # Baca sensor EL
            el_roll, el_pitch, _ = baca_sudut(device, ADDR_EL)

            # Baca sensor AZ: sudut + magnetometer
            az_roll, az_pitch, az_yaw_raw = baca_sudut(device, ADDR_AZ)
            mx, my, mz = baca_magnetometer(device, ADDR_AZ)

            if el_roll is None or az_roll is None:
                print("[WARN] Gagal baca sensor, mencoba lagi...")
                time.sleep(INTERVAL)
                continue

            waktu = time.strftime("%H:%M:%S")

            # EL dari roll sensor EL
            el_from_roll = _map_el_roll_90_180_to_0_90(el_roll)

            # Heading compass-only + filter anti-jump
            if mx is not None:
                heading_raw = heading_tilt_compensated(
                    mx, my, mz,
                    az_roll, az_pitch,   # gunakan roll/pitch sensor AZ untuk kompensasi
                    calib,
                )
                heading = heading_filter.update(heading_raw)
                src_heading = "CMP"
            else:
                heading = heading_filter.hold()
                if heading is None:
                    print("[WARN] Magnetometer belum valid, menunggu data pertama...")
                    time.sleep(INTERVAL)
                    continue
                src_heading = "HOLD"

            delta_hdg = 0.0 if prev_heading is None else abs(_angle_diff_deg(heading, prev_heading))
            prev_heading = heading

            arah = arah_mata_angin(heading)

            # Status EL
            if el_from_roll < 5:
                status_el = "DATAR"
            elif el_from_roll > 85:
                status_el = "TEGAK"
            else:
                status_el = f"{el_from_roll:.1f}°"

            print(
                f"{waktu:<10} "
                f"{el_from_roll:>8.2f} "
                f"{heading:>12.2f} "
                f"{arah:>20} "
                f"{src_heading:>6} "
                f"{delta_hdg:>8.2f} "
                f"{el_roll:>11.2f} "
                f"{az_roll:>11.2f}"
                f"  [EL:{status_el}]"
            )

            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n" + "-" * 70)
        print("[INFO] Program dihentikan.")
    finally:
        try:
            device.closeDevice()
            print("[INFO] Koneksi serial ditutup.")
        except Exception:
            pass


if __name__ == "__main__":
    main()
