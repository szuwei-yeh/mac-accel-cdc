#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

: "${TOP:=mac_dma_rob}"
: "${MAX_OUTSTANDING:=8}"
: "${CLK_PERIOD:=10.0}"
: "${TARGET_LIB:=/fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db}"
: "${RUN_ID:=${TOP}_mo${MAX_OUTSTANDING}_${CLK_PERIOD}ns}"

case "${TOP}" in
    mac_dma_rob|axi_read_engine_rob) ;;
    *) echo "Unsupported TOP: ${TOP}" >&2; exit 2 ;;
esac

case "${MAX_OUTSTANDING}" in
    2|4|8) ;;
    *) echo "MAX_OUTSTANDING must be 2, 4, or 8" >&2; exit 2 ;;
esac

if ! command -v dc_shell >/dev/null 2>&1; then
    echo "dc_shell is not available on PATH" >&2
    exit 2
fi

if [[ ! -f "${TARGET_LIB}" ]]; then
    echo "Standard-cell library not found: ${TARGET_LIB}" >&2
    exit 2
fi

RUN_ROOT="${SCRIPT_DIR}/runs/${RUN_ID}"
if [[ -e "${RUN_ROOT}" ]]; then
    echo "Refusing to reuse existing synthesis run directory: ${RUN_ROOT}" >&2
    echo "Choose a new RUN_ID or move the old directory first." >&2
    exit 2
fi

mkdir -p "${RUN_ROOT}"

if git -C "${REPO_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    GIT_HEAD="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
    GIT_STATUS="$(git -C "${REPO_ROOT}" status --short)"
else
    GIT_HEAD="not-a-git-worktree"
    GIT_STATUS="unavailable"
fi

{
    echo "run_id=${RUN_ID}"
    echo "top=${TOP}"
    echo "max_outstanding=${MAX_OUTSTANDING}"
    echo "clock_period_ns=${CLK_PERIOD}"
    echo "target_lib=${TARGET_LIB}"
    echo "dc_shell=$(command -v dc_shell)"
    echo "git_head=${GIT_HEAD}"
    echo "git_status_begin"
    printf '%s\n' "${GIT_STATUS}"
    echo "git_status_end"
    sha256sum \
        "${REPO_ROOT}/rtl/axi_read_engine_rob.v" \
        "${REPO_ROOT}/rtl/mac_dma_rob.v" \
        "${SCRIPT_DIR}/constraints_rob.sdc" \
        "${SCRIPT_DIR}/dc_baseline.tcl"
} > "${RUN_ROOT}/manifest.txt"

export TOP MAX_OUTSTANDING CLK_PERIOD TARGET_LIB RUN_ROOT
cd "${REPO_ROOT}"
dc_shell -f "${SCRIPT_DIR}/dc_baseline.tcl" 2>&1 | tee "${RUN_ROOT}/dc.log"
