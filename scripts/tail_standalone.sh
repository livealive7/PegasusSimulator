#!/usr/bin/env bash
# Reads back the log of the most recently launched scripts/run_standalone.sh process (inside the
# Isaac Sim container), and reports whether it's still running.
#
# Usage:
#   scripts/tail_standalone.sh [num_lines]

set -euo pipefail

REPO_DIR="/Volume/PegasusSimulator"
LOG_DIR="${REPO_DIR}/logs"
N="${1:-100}"

if [ ! -e "${LOG_DIR}/latest.log" ]; then
    echo "[tail_standalone] no run yet (${LOG_DIR}/latest.log not found)"
    exit 1
fi

if [ -f "${LOG_DIR}/standalone.pid" ] && kill -0 "$(cat "${LOG_DIR}/standalone.pid")" 2>/dev/null; then
    echo "[tail_standalone] process is RUNNING (pid=$(cat "${LOG_DIR}/standalone.pid"))"
else
    echo "[tail_standalone] process is NOT running (exited or was never started this way)"
fi

echo "[tail_standalone] last ${N} lines of $(readlink -f "${LOG_DIR}/latest.log")"
echo "----------------------------------------------------------------------"
tail -n "${N}" "${LOG_DIR}/latest.log"
