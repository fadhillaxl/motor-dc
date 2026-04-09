import socket
import threading

class RotctlServer:
    def __init__(self, controller, host="0.0.0.0", port=4533):
        self.controller = controller
        self.host = host
        self.port = port
        self._srv = None

    def start(self):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(5)
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        while True:
            conn, addr = self._srv.accept()
            threading.Thread(target=self._handle_client, args=(conn,), daemon=True).start()

    def _handle_client(self, conn: socket.socket):
        f = conn.makefile("rwb", buffering=0)
        try:
            while True:
                try:
                    line = f.readline()
                except OSError:
                    break
                if not line:
                    break
                cmd = line.decode("ascii", errors="ignore").strip()
                if not cmd:
                    continue
                if cmd.startswith("P"):
                    try:
                        parts = cmd.split()
                        az = float(parts[1]); el = float(parts[2])
                        curr_az, curr_el = self.controller.get_position()
                        print(f"[GPREDICT] TARGET: Az={az:.2f}, El={el:.2f} | ACTUAL: Az={curr_az:.2f}, El={curr_el:.2f}")
                        self.controller.set_target(az, el)
                        f.write(b"RPRT 0\n")
                    except Exception:
                        f.write(b"RPRT -1\n")
                elif cmd == "p":
                    az, el = self.controller.get_position()
                    f.write(f"{az:.6f}\n{el:.6f}\n".encode("ascii"))
                elif cmd == "S":
                    try:
                        self.controller.stop()
                        f.write(b"RPRT 0\n")
                    except Exception:
                        f.write(b"RPRT -1\n")
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
            try:
                conn.close()
            except Exception:
                pass
