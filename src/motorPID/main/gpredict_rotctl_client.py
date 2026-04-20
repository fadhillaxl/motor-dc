#!/usr/bin/env python3
# coding: utf-8
"""
Simulasi client Gpredict ke rotctl server Hamlib-compatible.

Fungsi:
- Connect ke rotctl server yang sudah aktif
- Optional spawn server lokal mode simulasi
- Kirim command Hamlib: P, p, S, R, Q
- Tampilkan request/response agar mudah debug
"""

import argparse
import socket
import subprocess
import sys
import time
from pathlib import Path


def wait_for_port(host: str, port: int, timeout_s: float = 8.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.2)
    return False


def send_line(sock: socket.socket, line: str):
    print(f">>> {line}")
    sock.sendall((line + "\n").encode("ascii"))


def recv_line(sock: socket.socket) -> str:
    data = b""
    while not data.endswith(b"\n"):
        chunk = sock.recv(1)
        if not chunk:
            break
        data += chunk
    text = data.decode("ascii", errors="ignore").strip()
    print(f"<<< {text}")
    return text


def main():
    parser = argparse.ArgumentParser(description="Simulasi koneksi Gpredict ke rotctl server")
    parser.add_argument("--host", default="127.0.0.1", help="Host rotctl server")
    parser.add_argument("--port", type=int, default=4533, help="Port rotctl server")
    parser.add_argument("--az", type=float, default=250.0, help="Target azimuth")
    parser.add_argument("--el", type=float, default=70.0, help="Target elevation")
    parser.add_argument("--wait", type=float, default=1.0, help="Waktu tunggu setelah kirim target")
    parser.add_argument("--poll-count", type=int, default=3, help="Berapa kali polling posisi")
    parser.add_argument("--poll-interval", type=float, default=0.7, help="Jeda antar polling posisi")
    parser.add_argument("--spawn-sim-server", action="store_true", help="Spawn server lokal mode simulasi")
    parser.add_argument("--no-stop", action="store_true", help="Jangan kirim command stop")
    args = parser.parse_args()

    proc = None
    script_dir = Path(__file__).resolve().parent

    if args.spawn_sim_server:
        proc = subprocess.Popen(
            [
                sys.executable,
                str(script_dir / "rotctl_server_gpredict.py"),
                "-gpredict",
                "--sim",
                "--no-auto-home",
                "--port",
                str(args.port),
            ],
            cwd=str(script_dir),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        print(f"[INFO] Spawn server simulasi di port {args.port}")

    try:
        if not wait_for_port(args.host, args.port):
            raise RuntimeError(f"rotctl server tidak tersedia di {args.host}:{args.port}")

        with socket.create_connection((args.host, args.port), timeout=3.0) as sock:
            print(f"[INFO] Connected ke {args.host}:{args.port}")

            send_line(sock, f"P {args.az:.3f} {args.el:.3f}")
            reply = recv_line(sock)
            if reply != "RPRT 0":
                raise RuntimeError(f"Target command gagal: {reply}")

            time.sleep(max(0.0, args.wait))

            for idx in range(args.poll_count):
                send_line(sock, "p")
                az_now = recv_line(sock)
                el_now = recv_line(sock)
                print(f"[INFO] Poll #{idx + 1}: AZ={az_now} EL={el_now}")
                if idx < args.poll_count - 1:
                    time.sleep(max(0.0, args.poll_interval))

            send_line(sock, "R")
            recv_line(sock)

            if not args.no_stop:
                send_line(sock, "S")
                recv_line(sock)

            send_line(sock, "Q")
            recv_line(sock)

        print("[INFO] Simulasi Gpredict selesai.")
    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()


if __name__ == "__main__":
    main()
