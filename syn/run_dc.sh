#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

mkdir -p "${SCRIPT_DIR}/logs" "${SCRIPT_DIR}/reports" "${SCRIPT_DIR}/mapped" "${SCRIPT_DIR}/work"

: "${TOP:=mac_dma_rob}"
: "${CLK_PERIOD:=10.0}"
: "${TARGET_LIB:=/fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db}"

cd "${REPO_ROOT}"

echo "TOP=${TOP}"
echo "CLK_PERIOD=${CLK_PERIOD} ns"
echo "TARGET_LIB=${TARGET_LIB}"

dc_shell -f "${SCRIPT_DIR}/dc_rob.tcl" | tee "${SCRIPT_DIR}/logs/${TOP}_dc.log"
