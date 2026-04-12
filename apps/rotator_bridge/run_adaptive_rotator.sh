#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

python apps/rotator_bridge/run.py \
  --backend adaptive \
  --port 4533 \
  --imu-port /dev/ttyUSB0 \
  --config src/motorPID/config-stepper.conf
