#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
: "${RUN_ID:=mac_dma_rob_mo8_power_activity_$(date +%Y%m%d_%H%M%S)}"

RUN_ROOT="${SCRIPT_DIR}/runs/${RUN_ID}"
if [[ -e "${RUN_ROOT}" ]]; then
    echo "Refusing to reuse immutable activity directory: ${RUN_ROOT}" >&2
    exit 2
fi
mkdir -p "${RUN_ROOT}/inputs" "${RUN_ROOT}/logs" "${RUN_ROOT}/activity" "${RUN_ROOT}/build"

cp "${REPO_ROOT}/rtl/axi_read_engine_rob.v" "${RUN_ROOT}/inputs/"
cp "${REPO_ROOT}/rtl/mac_dma_rob.v" "${RUN_ROOT}/inputs/"
cp "${REPO_ROOT}/sim/axi_read_mem_model_ooo.v" "${RUN_ROOT}/inputs/"
cp "${REPO_ROOT}/sim/tb_mac_dma_rob_power.v" "${RUN_ROOT}/inputs/"
cp "${SCRIPT_DIR}/run_power_activity.sh" "${RUN_ROOT}/inputs/"

{
    echo "run_id=${RUN_ID}"
    echo "top=mac_dma_rob"
    echo "max_outstanding=8"
    echo "clock_period_ns=10.0"
    echo "workload=deterministic_len64_active_then_512_cycle_idle"
    echo "memory_latency_cycles=40"
    echo "ooo_mode=enabled_fixed_lfsr_seed_ACE1"
    echo "backpressure=deterministic_cycle_modulo_pattern"
    echo "vcs=$(command -v vcs)"
    echo "vcd2saif=$(command -v vcd2saif)"
    git -C "${REPO_ROOT}" rev-parse HEAD | sed 's/^/git_head=/'
    echo "input_sha256_begin"
    sha256sum "${RUN_ROOT}"/inputs/*
    echo "input_sha256_end"
} > "${RUN_ROOT}/manifest.txt"

cd "${RUN_ROOT}/build"
vcs -full64 -sverilog +v2k -debug_access+all +memcbk \
    +define+POWER_MAX_OUT=8 +define+POWER_ACTIVITY_DUMP \
    "${RUN_ROOT}/inputs/axi_read_engine_rob.v" \
    "${RUN_ROOT}/inputs/mac_dma_rob.v" \
    "${RUN_ROOT}/inputs/axi_read_mem_model_ooo.v" \
    "${RUN_ROOT}/inputs/tb_mac_dma_rob_power.v" \
    -top tb_mac_dma_rob_power -o simv \
    2>&1 | tee "${RUN_ROOT}/logs/vcs_compile.log"

./simv 2>&1 | tee "${RUN_ROOT}/logs/rtl_activity_sim.log"
mv mac_dma_rob_power.vpd "${RUN_ROOT}/activity/full.vpd"
vpd2vcd +includemda "${RUN_ROOT}/activity/full.vpd" "${RUN_ROOT}/activity/full.vcd" \
    2>&1 | tee "${RUN_ROOT}/logs/vpd2vcd.log"

grep -q "POWER_REGRESSION_PASS" "${RUN_ROOT}/logs/rtl_activity_sim.log"
ACTIVE_START="$(sed -n 's/.*POWER_ACTIVE_START_PS=\([0-9][0-9]*\).*/\1/p' "${RUN_ROOT}/logs/rtl_activity_sim.log" | tail -1)"
ACTIVE_END="$(sed -n 's/.*POWER_ACTIVE_END_PS=\([0-9][0-9]*\).*/\1/p' "${RUN_ROOT}/logs/rtl_activity_sim.log" | tail -1)"
IDLE_START="$(sed -n 's/.*POWER_IDLE_START_PS=\([0-9][0-9]*\).*/\1/p' "${RUN_ROOT}/logs/rtl_activity_sim.log" | tail -1)"
IDLE_END="$(sed -n 's/.*POWER_IDLE_END_PS=\([0-9][0-9]*\).*/\1/p' "${RUN_ROOT}/logs/rtl_activity_sim.log" | tail -1)"
for value in ACTIVE_START ACTIVE_END IDLE_START IDLE_END; do
    if [[ -z "${!value}" ]]; then
        echo "Missing ${value} marker" >&2
        exit 2
    fi
done

vcd2saif -input "${RUN_ROOT}/activity/full.vcd" \
    -output "${RUN_ROOT}/activity/active.saif" \
    -instance tb_mac_dma_rob_power/dut \
    -time "${ACTIVE_START}" "${ACTIVE_END}" \
    2>&1 | tee "${RUN_ROOT}/logs/vcd2saif_active.log"
vcd2saif -input "${RUN_ROOT}/activity/full.vcd" \
    -output "${RUN_ROOT}/activity/idle.saif" \
    -instance tb_mac_dma_rob_power/dut \
    -time "${IDLE_START}" "${IDLE_END}" \
    2>&1 | tee "${RUN_ROOT}/logs/vcd2saif_idle.log"

{
    echo "active_start_ps=${ACTIVE_START}"
    echo "active_end_ps=${ACTIVE_END}"
    echo "active_duration_ps=$((ACTIVE_END-ACTIVE_START))"
    echo "idle_start_ps=${IDLE_START}"
    echo "idle_end_ps=${IDLE_END}"
    echo "idle_duration_ps=$((IDLE_END-IDLE_START))"
    echo "vcd_bytes=$(wc -c < "${RUN_ROOT}/activity/full.vcd")"
    echo "vpd_bytes=$(wc -c < "${RUN_ROOT}/activity/full.vpd")"
    echo "active_saif_bytes=$(wc -c < "${RUN_ROOT}/activity/active.saif")"
    echo "idle_saif_bytes=$(wc -c < "${RUN_ROOT}/activity/idle.saif")"
    sha256sum "${RUN_ROOT}/activity/active.saif" "${RUN_ROOT}/activity/idle.saif"
} > "${RUN_ROOT}/activity/windows_and_hashes.txt"

echo "Activity capture complete: ${RUN_ROOT}"
