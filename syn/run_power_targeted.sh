#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
: "${MODE:?MODE must be baseline or gated}"
: "${ACTIVITY_RUN:?ACTIVITY_RUN must name the immutable activity run}"
: "${RUN_ID:=mac_dma_rob_mo8_power_${MODE}_$(date +%Y%m%d_%H%M%S)}"
case "${MODE}" in baseline|gated) ;; *) echo "Invalid MODE=${MODE}" >&2; exit 2;; esac

RUN_ROOT="${SCRIPT_DIR}/runs/${RUN_ID}"
ACTIVITY_ROOT="${SCRIPT_DIR}/runs/${ACTIVITY_RUN}"
ACTIVE_SAIF="${ACTIVITY_ROOT}/activity/active.saif"
IDLE_SAIF="${ACTIVITY_ROOT}/activity/idle.saif"
if [[ -e "${RUN_ROOT}" ]]; then
    echo "Refusing to reuse immutable power directory: ${RUN_ROOT}" >&2
    exit 2
fi
for input in "${ACTIVE_SAIF}" "${IDLE_SAIF}"; do
    [[ -f "${input}" ]] || { echo "Missing ${input}" >&2; exit 2; }
done
for expected in \
    "top=mac_dma_rob" \
    "max_outstanding=8" \
    "clock_period_ns=10.0" \
    "workload=deterministic_len64_active_then_512_cycle_idle"; do
    grep -Fqx "${expected}" "${ACTIVITY_ROOT}/manifest.txt" || {
        echo "Activity manifest does not match canonical configuration: ${expected}" >&2
        exit 2
    }
done
mkdir -p "${RUN_ROOT}/inputs"
cp "${REPO_ROOT}/rtl/axi_read_engine_rob.v" "${RUN_ROOT}/inputs/"
cp "${REPO_ROOT}/rtl/mac_dma_rob.v" "${RUN_ROOT}/inputs/"
cp "${SCRIPT_DIR}/constraints_rob.sdc" "${RUN_ROOT}/inputs/"
cp "${SCRIPT_DIR}/dc_power_targeted.tcl" "${RUN_ROOT}/inputs/"
cp "${SCRIPT_DIR}/run_power_targeted.sh" "${RUN_ROOT}/inputs/"

{
    echo "run_id=${RUN_ID}"
    echo "mode=${MODE}"
    echo "top=mac_dma_rob"
    echo "max_outstanding=8"
    echo "clock_period_ns=10.0"
    echo "compile=$([[ ${MODE} == gated ]] && echo 'compile_ultra -gate_clock' || echo compile_ultra)"
    echo "activity_run=${ACTIVITY_RUN}"
    echo "active_saif=${ACTIVE_SAIF}"
    echo "idle_saif=${IDLE_SAIF}"
    echo "target_lib=/fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db"
    echo "dc_shell=$(command -v dc_shell)"
    git -C "${REPO_ROOT}" rev-parse HEAD | sed 's/^/git_head=/'
    echo "input_sha256_begin"
    sha256sum "${RUN_ROOT}"/inputs/* "${ACTIVE_SAIF}" "${IDLE_SAIF}"
    echo "input_sha256_end"
} > "${RUN_ROOT}/manifest.txt"

export MODE RUN_ROOT ACTIVE_SAIF IDLE_SAIF
cd "${REPO_ROOT}"
dc_shell -f "${SCRIPT_DIR}/dc_power_targeted.tcl" 2>&1 | tee "${RUN_ROOT}/dc.log"
