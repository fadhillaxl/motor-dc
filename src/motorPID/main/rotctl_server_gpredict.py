#!/usr/bin/env python3
# coding: utf-8
"""
Hamlib rotctld-compatible server for Gpredict using az_el_controller backend.

Supported command subset:
- P <az> <el> : set target
- p           : get current position
- S           : stop/hold
- R           : reset fault latch
- Q           : close client
"""

import argparse
import logging
import socket
import threading
import time
from contextlib import closing
from pathlib import Path
from typing import Optional

from az_el_controller import AzElTrackerService


BASE_DIR = Path(__file__).resolve().parent
LOG_FILE = BASE_DIR / "rotctl_gpredict.log"

logging.basicConfig(
    filename=str(LOG_FILE),
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)


class GpredictRotctlServer:
    def __init__(self, controller: AzElTrackerService, host: str = "0.0.0.0", port: int = 4533):
        self.controller = controller
        self.host = host
        self.port = int(port)
        self._srv: Optional[socket.socket] = None
        self._stop = threading.Event()

    def start(self):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(5)
        threading.Thread(target=self._accept_loop, daemon=True, name="rotctl-accept").start()
        logging.info("rotctl server listening on %s:%d", self.host, self.port)

    def close(self):
        self._stop.set()
        if self._srv is not None:
            try:
                self._srv.close()
            except Exception:
                pass

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, addr = self._srv.accept()
            except OSError:
                break
            logging.info("client connected from %s:%s", *addr[:2])
            threading.Thread(
                target=self._handle_client,
                args=(conn, addr),
                daemon=True,
                name=f"rotctl-client-{addr[0]}:{addr[1]}",
            ).start()

    def _handle_client(self, conn: socket.socket, addr):
        with closing(conn):
            f = conn.makefile("rwb", buffering=0)
            try:
                while not self._stop.is_set():
                    try:
                        line = f.readline()
                    except OSError as exc:
                        logging.warning("socket read error from %s:%s -> %s", addr[0], addr[1], exc)
                        break
                    if not line:
                        logging.info("client disconnected %s:%s", addr[0], addr[1])
                        break

                    cmd = line.decode("ascii", errors="ignore").strip()
                    if not cmd:
                        continue
                    logging.info("RX %s:%s -> %s", addr[0], addr[1], cmd)

                    if cmd.startswith("P"):
                        self._cmd_set_target(f, cmd)
                    elif cmd == "p":
                        self._cmd_get_position(f, addr)
                    elif cmd == "S":
                        self._cmd_stop(f)
                    elif cmd == "R":
                        self._cmd_reset_fault(f)
                    elif cmd == "Q":
                        f.write(b"RPRT 0\n")
                        break
                    else:
                        f.write(b"RPRT -8\n")
            finally:
                try:
                    f.close()
                except Exception:
                    pass

    def _cmd_set_target(self, f, cmd: str):
        try:
            _, az_s, el_s = cmd.split()
            az = float(az_s)
            el = float(el_s)
            self.controller.set_target(az, el)
            dbg = self.controller.get_debug_snapshot()
            logging.info(
                "TX RPRT 0 | TARGET az=%.3f el=%.3f | ACTUAL az=%.3f el=%.3f | "
                "AZ_ERR=%.3f EL_ERR=%.3f AZ_PID=%.3f EL_PID=%.3f EL_FF=%.3f",
                az,
                el,
                dbg["az"],
                dbg["el"],
                dbg["az_error"],
                dbg["el_error"],
                dbg["az_pid_cmd"],
                dbg["el_base_cmd"],
                dbg["el_gravity_ff"],
            )
            f.write(b"RPRT 0\n")
        except Exception as exc:
            logging.exception("failed to handle target command: %s", exc)
            f.write(b"RPRT -1\n")

    def _cmd_get_position(self, f, addr):
        try:
            az, el = self.controller.get_position()
            dbg = self.controller.get_debug_snapshot()
            payload = f"{az:.6f}\n{el:.6f}\n".encode("ascii")
            f.write(payload)
            logging.info(
                "TX %s:%s <- az=%.3f el=%.3f | faults az='%s' el='%s'",
                addr[0],
                addr[1],
                az,
                el,
                dbg["az_fault"],
                dbg["el_fault"],
            )
        except Exception as exc:
            logging.exception("failed to read position: %s", exc)
            f.write(b"RPRT -1\n")

    def _cmd_stop(self, f):
        try:
            self.controller.stop()
            logging.info("TX RPRT 0 | stop")
            f.write(b"RPRT 0\n")
        except Exception as exc:
            logging.exception("failed to stop controller: %s", exc)
            f.write(b"RPRT -1\n")

    def _cmd_reset_fault(self, f):
        try:
            self.controller.reset_fault()
            logging.info("TX RPRT 0 | reset fault")
            f.write(b"RPRT 0\n")
        except Exception as exc:
            logging.exception("failed to reset fault: %s", exc)
            f.write(b"RPRT -1\n")


def main():
    parser = argparse.ArgumentParser(description="Hamlib rotctld-compatible server for Gpredict")
    parser.add_argument("-gpredict", "--gpredict", action="store_true", help="Enable Gpredict/Hamlib mode")
    parser.add_argument("-m", "--mode", default="rotator", help="Rotator mode label for automation scripts")
    parser.add_argument("-r", "--device-port", default=None, help="Serial/USB device port")
    parser.add_argument("-s", "--baud-rate", type=int, default=9600, help="Serial baud rate")
    parser.add_argument("--host", default="0.0.0.0", help="Listen host")
    parser.add_argument("--port", type=int, default=4533, help="Listen port (Hamlib default 4533)")
    parser.add_argument("--sim", action="store_true", help="Run backend in simulation mode")
    parser.add_argument("--no-auto-home", action="store_true", help="Skip automatic homing")
    args = parser.parse_args()

    controller = AzElTrackerService(
        sim=args.sim,
        auto_home=not args.no_auto_home,
        sensor_port=args.device_port,
        sensor_baud=args.baud_rate,
    )
    server = GpredictRotctlServer(controller, host=args.host, port=args.port)
    server.start()

    print(
        f"rotctld-compatible server aktif di {args.host}:{args.port} "
        f"(mode={args.mode}, gpredict={args.gpredict}, sim={args.sim}, "
        f"device={args.device_port}, baud={args.baud_rate})"
    )
    print("Gunakan Gpredict -> Hamlib NET rotctld -> host 127.0.0.1 port 4533")

    try:
        while True:
            dbg = controller.get_debug_snapshot()
            logging.info(
                "MONITOR az=%.3f el=%.3f AZ_ERR=%.3f EL_ERR=%.3f AZ_PID=%.3f EL_PID=%.3f EL_FF=%.3f",
                dbg["az"],
                dbg["el"],
                dbg["az_error"],
                dbg["el_error"],
                dbg["az_pid_cmd"],
                dbg["el_base_cmd"],
                dbg["el_gravity_ff"],
            )
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        controller.close()


if __name__ == "__main__":
    main()
