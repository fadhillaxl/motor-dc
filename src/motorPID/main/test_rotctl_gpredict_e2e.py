#!/usr/bin/env python3
# coding: utf-8

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
    sock.sendall((line + "\n").encode("ascii"))


def recv_line(sock: socket.socket) -> str:
    data = b""
    while not data.endswith(b"\n"):
        chunk = sock.recv(1)
        if not chunk:
            break
        data += chunk
    return data.decode("ascii", errors="ignore").strip()


def main():
    parser = argparse.ArgumentParser(description="End-to-end test for rotctl_server_gpredict.py")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4533)
    parser.add_argument("--spawn", action="store_true", help="Spawn local server in simulation mode for test")
    args = parser.parse_args()

    proc = None
    script_dir = Path(__file__).resolve().parent
    if args.spawn:
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

    try:
        if not wait_for_port(args.host, args.port):
            raise RuntimeError(f"rotctl server tidak tersedia di {args.host}:{args.port}")

        with socket.create_connection((args.host, args.port), timeout=3.0) as sock:
            send_line(sock, "P 250 70")
            reply = recv_line(sock)
            if reply != "RPRT 0":
                raise RuntimeError(f"reply target tidak valid: {reply}")

            time.sleep(0.8)

            send_line(sock, "p")
            az = float(recv_line(sock))
            el = float(recv_line(sock))
            if not (0.0 <= az <= 360.0 and 0.0 <= el <= 90.0):
                raise RuntimeError(f"posisi di luar batas: az={az}, el={el}")

            send_line(sock, "S")
            reply = recv_line(sock)
            if reply != "RPRT 0":
                raise RuntimeError(f"reply stop tidak valid: {reply}")

            send_line(sock, "Q")
            recv_line(sock)

        print(f"PASS: az={az:.3f} el={el:.3f}")
    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()


if __name__ == "__main__":
    main()
