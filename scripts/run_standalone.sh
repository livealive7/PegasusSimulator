#!/usr/bin/env bash
# Runs a Pegasus standalone example (default: examples/12_multi_backend_standalone.py) inside the
# Isaac Sim docker container, detached, with stdout/stderr captured to a timestamped log file.
#
# Meant to be invoked via `docker exec -d isim-peterson-isaac-sim-1 ...` (or directly inside the
# container) so a remote SSH session can kick it off and disconnect without killing the process -
# see scripts/tail_standalone.sh to follow the log afterwards.
#
# Usage:
#   scripts/run_standalone.sh [example_relative_path] [extra args passed to the example]
#
# Examples:
#   scripts/run_standalone.sh
#   scripts/run_standalone.sh examples/12_multi_backend_standalone.py --headless
#   scripts/run_standalone.sh examples/8_camera_vehicle.py --headless

set -euo pipefail

# Auto-detect rather than hardcode /Volume/PegasusSimulator -- the
# streaming-vs-standalone Isaac Sim containers mount this repo at different
# paths (/Volume/PegasusSimulator vs /workspace/PegasusSimulator), and this
# script needs to work unmodified in both.
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=./ros2_env.sh
source "${REPO_DIR}/scripts/ros2_env.sh"

EXAMPLE="${1:-examples/12_multi_backend_standalone.py}"
shift || true

LOG_DIR="${REPO_DIR}/logs"
mkdir -p "${LOG_DIR}"

TS="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/standalone_${TS}.log"

cd "${REPO_DIR}"
echo "[run_standalone] launching ${EXAMPLE} $* -> ${LOG_FILE}"

# Record the PID so tail_standalone.sh / a remote session can check liveness and kill it if needed
nohup /isaac-sim/python.sh "${EXAMPLE}" "$@" > "${LOG_FILE}" 2>&1 &
echo $! > "${LOG_DIR}/standalone.pid"
ln -sf "${LOG_FILE}" "${LOG_DIR}/latest.log"

echo "[run_standalone] pid=$(cat "${LOG_DIR}/standalone.pid")"
