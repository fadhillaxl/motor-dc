#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

DEVICE_PORT="${DEVICE_PORT:-/dev/ttyUSB0}"
BAUD_RATE="${BAUD_RATE:-9600}"
LISTEN_PORT="${LISTEN_PORT:-4533}"
ROTATOR_MODE="${ROTATOR_MODE:-rotator}"
SIM_FLAG="${SIM_FLAG:-}"
AUTO_HOME_FLAG="${AUTO_HOME_FLAG:-}"

python3 rotctl_server_gpredict.py \
  -gpredict \
  -m "${ROTATOR_MODE}" \
  -r "${DEVICE_PORT}" \
  -s "${BAUD_RATE}" \
  --port "${LISTEN_PORT}" \
  ${SIM_FLAG} \
  ${AUTO_HOME_FLAG}
