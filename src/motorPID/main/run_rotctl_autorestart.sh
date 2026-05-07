#!/usr/bin/env bash

set -u

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-$BASE_DIR/venv/bin/python}"
HOST="${ROTCTL_HOST:-0.0.0.0}"
PORT="${ROTCTL_PORT:-4533}"
RESTART_DELAY="${RESTART_DELAY:-2}"

if [[ ! -x "$PYTHON_BIN" ]]; then
  PYTHON_BIN="$(command -v python3)"
fi

if [[ -z "${PYTHON_BIN:-}" ]]; then
  echo "[FATAL] Python tidak ditemukan."
  exit 1
fi

echo "[INFO] Auto-restart rotctl mode"
echo "[INFO] Script     : $BASE_DIR/az_el_controller.py"
echo "[INFO] Python     : $PYTHON_BIN"
echo "[INFO] Host/Port  : $HOST:$PORT"
echo "[INFO] Delay      : ${RESTART_DELAY}s"

while true; do
  echo "[$(date '+%F %T')] [START] az_el_controller rotctl"
  "$PYTHON_BIN" "$BASE_DIR/az_el_controller.py" --mode rotctl --rotctl-host "$HOST" --rotctl-port "$PORT" "$@"
  EXIT_CODE=$?
  echo "[$(date '+%F %T')] [WARN] process exit code=$EXIT_CODE, restart in ${RESTART_DELAY}s..."
  sleep "$RESTART_DELAY"
done
