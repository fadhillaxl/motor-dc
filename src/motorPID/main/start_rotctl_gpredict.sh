#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

LISTEN_PORT="${LISTEN_PORT:-4533}"
LISTEN_HOST="${LISTEN_HOST:-0.0.0.0}"
TARGET_AZ="${TARGET_AZ:-20.0}"
TARGET_EL="${TARGET_EL:-70.0}"

python3 az_el_controller.py \
  --mode rotctl \
  --rotctl-host "${LISTEN_HOST}" \
  --rotctl-port "${LISTEN_PORT}" \
  --az "${TARGET_AZ}" \
  --el "${TARGET_EL}"
