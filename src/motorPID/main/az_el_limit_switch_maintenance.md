# AZ/EL Limit Switch Maintenance

## Ringkasan

Dokumen ini menjelaskan wiring, parameter, dan prosedur maintenance untuk limit switch AZ/EL pada `az_el_controller.py`.

Implementasi saat ini memakai:

- Limit switch mekanik tipe `roller lever`
- Konfigurasi `normally closed (NC)` untuk fail-safe
- Debounce software `50 ms`
- Penyimpanan posisi persisten file-backed pada `az_el_eeprom_state.json`
- Notifikasi fault ke sistem kontrol utama melalui `az_el_fault_state.json`

Catatan:

- Pada deployment Python Raspberry Pi ini, penyimpanan persisten memakai file JSON sebagai emulator EEPROM.
- Jika sistem dipindah ke mikrokontroler yang punya EEPROM/NVS asli, backend penyimpanan dapat diganti tanpa mengubah logika limit switch.

## Wiring Diagram

### Pin Motor

- `AZ STEP` -> `GPIO17`
- `AZ DIR` -> `GPIO27`
- `AZ EN` -> `GPIO22`
- `EL STEP` -> `GPIO23`
- `EL DIR` -> `GPIO24`
- `EL EN` -> `GPIO25`

### Pin Limit Switch

- `AZ MIN / 0°` -> `GPIO5`
- `AZ MAX / 360°` -> `GPIO6`
- `EL MIN / 0°` -> `GPIO12`
- `EL MAX / 90°` -> `GPIO16`

### Wiring NC Fail-Safe

Gunakan skema berikut untuk setiap limit switch:

```text
GPIO INPUT ----+----[ internal pull-up 3.3V ]
               |
               +----[ NC roller lever switch ]---- GND
```

Perilaku:

- Kondisi normal: switch tertutup, input tertarik ke `LOW`
- Kondisi limit tertekan: switch membuka, input menjadi `HIGH`
- Kabel putus: input juga menjadi `HIGH`

Dengan pola ini, trigger limit dan kegagalan kabel sama-sama dianggap kondisi tidak aman.

## Posisi Mekanik

- `AZ MIN` dipasang pada batas `0°`
- `AZ MAX` dipasang pada batas `360°`
- `EL MIN` dipasang pada batas `0°`
- `EL MAX` dipasang pada batas `90°`

Rekomendasi pemasangan:

- Pastikan roller menyentuh cam/stop mekanik sebelum kabel mulai menegang.
- Sisakan margin mekanik 1-2 derajat antara titik trigger dan hard stop fisik.
- Gunakan bracket yang kaku agar titik trigger tidak bergeser saat vibrasi.

## Parameter Konfigurasi

Parameter penting di `StepperConfig`:

- `limit_min_pin`
- `limit_max_pin`
- `limit_nc = True`
- `limit_debounce_ms = 50`
- `home_speed_sps = 280.0`
- `home_timeout_s = 20.0`
- `soft_limit_min_deg`
- `soft_limit_max_deg`
- `persist_interval_s = 0.5`

## Logika Kontrol

Saat limit switch aktif:

- Motor dihentikan
- Posisi terakhir disimpan ke `az_el_eeprom_state.json`
- Fault ditulis ke `az_el_fault_state.json`
- Pesan fault dicatat ke `az_el_system.log`
- Tracking otomatis dimatikan sampai fault di-reset

Kode fault yang dipakai:

- `LIMIT_MIN`
- `LIMIT_MAX`
- `SOFT_LIMIT`
- `HOME_TIMEOUT`

## Auto Homing

Startup default menjalankan homing otomatis:

1. Sumbu `AZ` bergerak ke arah negatif sampai `AZ MIN` aktif
2. Posisi `AZ` di-set ke `0°`
3. Sumbu `EL` bergerak ke arah negatif sampai `EL MIN` aktif
4. Posisi `EL` di-set ke `0°`
5. Sensor absolut di-zero-kan ulang

Jika ingin melewati homing dan memakai posisi terakhir tersimpan:

```bash
python az_el_controller.py --no-auto-home
```

## Self-Test Fungsional

Untuk simulasi overtravel:

```bash
python az_el_controller.py --sim --self-test-limits
```

Self-test melakukan:

- Simulasi request `AZ 370°`
- Simulasi request `EL 100°`
- Verifikasi bahwa `AZ` berhenti di `360°`
- Verifikasi bahwa `EL` berhenti di `90°`
- Verifikasi bahwa fault yang muncul adalah `LIMIT_MAX`

## Operasi Harian

- `T` mengaktifkan tracking target
- `R` me-reset fault latch setelah inspeksi aman
- `Space` menghentikan kedua motor
- `Z` menjalankan zero calibration sensor

## Checklist Maintenance

- Periksa roller lever bergerak bebas
- Periksa kabel limit switch tidak putus atau longgar
- Verifikasi status `LOW` saat normal dan `HIGH` saat switch ditekan
- Jalankan self-test simulasi setelah perubahan software
- Jalankan uji homing nyata setelah perubahan mekanik
- Periksa file `az_el_fault_state.json` setelah insiden limit

