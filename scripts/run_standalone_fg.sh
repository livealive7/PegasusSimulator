#!/usr/bin/env bash
# Runs a Pegasus standalone example in the FOREGROUND, streaming its output live to your terminal
# until you hit Ctrl+C - and, at the same time, mirrors that same output to logs/latest.log so a
# Claude Code session (this one or a separate one, local or over SSH) can inspect what's currently
# happening without needing your terminal:
#
#   docker exec isim-peterson-isaac-sim-1 /Volume/PegasusSimulator/scripts/tail_standalone.sh 200
#
# For a detached/background run instead (e.g. scripted benchmarks with --duration), use
# scripts/run_standalone.sh.
#
# Usage:
#   docker exec -it isim-peterson-isaac-sim-1 /Volume/PegasusSimulator/scripts/run_standalone_fg.sh \
#       [example_relative_path] [extra args passed to the example]
#
# Examples:
#   docker exec -it isim-peterson-isaac-sim-1 /Volume/PegasusSimulator/scripts/run_standalone_fg.sh
#   docker exec -it isim-peterson-isaac-sim-1 /Volume/PegasusSimulator/scripts/run_standalone_fg.sh \
#       examples/12_multi_backend_standalone.py --headless
#
# Note: needs `-it` on `docker exec` (a real tty) for Ctrl+C to be delivered as SIGINT and stop the
# simulation cleanly instead of just detaching your terminal from a still-running process.

set -uo pipefail

REPO_DIR="/Volume/PegasusSimulator"
# shellcheck source=./ros2_env.sh
source "${REPO_DIR}/scripts/ros2_env.sh"

EXAMPLE="${1:-examples/12_multi_backend_standalone.py}"
shift || true

LOG_DIR="${REPO_DIR}/logs"
mkdir -p "${LOG_DIR}"

TS="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/standalone_${TS}.log"
ln -sf "${LOG_FILE}" "${LOG_DIR}/latest.log"

cd "${REPO_DIR}"
echo "[run_standalone_fg] launching ${EXAMPLE} $* -> ${LOG_FILE}"
echo "[run_standalone_fg] press Ctrl+C to stop"

# Run as a background job of this script (not `nohup`'d - deliberately still tied to this
# terminal/session) so we can capture its PID for tail_standalone.sh, while `tee` mirrors output
# to both this terminal and the log file. `wait` blocks here until it exits or Ctrl+C is delivered.
/isaac-sim/python.sh "${EXAMPLE}" "$@" > >(tee "${LOG_FILE}") 2>&1 &
PID=$!
echo "${PID}" > "${LOG_DIR}/standalone.pid"

wait "${PID}"
STATUS=$?
echo "[run_standalone_fg] exited with code ${STATUS}"
exit "${STATUS}"
