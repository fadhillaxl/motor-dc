## Masalah

* BrokenPipeError saat menulis balasan (contoh RPRT -8) ketika klien menutup koneksi lebih cepat.

* Format balasan `p` sebaiknya nilai numerik per baris (tanpa label) sesuai protokol Hamlib sederhana; label "Azimuth:" bisa membuat klien bingung.

## Perubahan yang Diusulkan

* Tambah util `_safe_write(sock_or_file, bytes)` yang membungkus `write/send` dengan try/except dan menutup koneksi jika terjadi BrokenPipe/ConnectionReset.

* Ubah respon `p` menjadi dua baris numerik saja: `f"{az:.3f}\n{el:.3f}\n"`.

* Ketika perintah tidak dikenal, kirim `RPRT -8` memakai `_safe_write`; jika koneksi sudah terputus, cukup hentikan handler tanpa exception.

* Terima baris perintah dengan CRLF (`\r\n`) dan abaikan spasi berlebih.

## Implementasi

* Edit `apps/rotator_bridge/rotctl_server.py`:

  * Tambah `_safe_write` dan gunakan pada semua jalur keluaran (P/p/S/Q/error).

  * Ubah `p` untuk mengirim nilai numerik dua baris.

  * Cek baris kosong/EOF dan segera keluar dari loop klien.

## Validasi

* Tes lokal dengan nc: `printf "p\n" | nc -w 1 localhost 4533` → dua baris angka.

* Tes Hamlib: `rotctl -m 2 -r localhost:4533 p`, `P 35 10`, `S`, `Q`.

* Pastikan tidak ada thread exception saat klien disconnect mendadak.

## Dampak

* Tidak mengubah folder lama; perubahan hanya di apps/rotator\_bridge.

* Kompatibilitas protokol meningkat untuk Gpredict/Hamlib; telemetri SDR tetap berjalan.

