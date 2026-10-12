#!/usr/bin/env python3
"""Run fast Icarus tests in a temporary directory; validate printed scoreboards."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
TESTS = [('tb_mac_dma_pending_tail', None, 2, ['rtl/mac_dma.v']),
 ('tb_mac_dma_rob_pending_tail', None, 2, ['rtl/mac_dma_rob.v', 'rtl/axi_read_engine_rob.v']),
 ('tb_mac_accel_dma_axil_stream', None, 4,
 ['rtl/mac_accel_dma_top.v', 'rtl/mac_dma.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v']),
 ('tb_mac_accel_dma_rob_axil_stream', None, 4,
 ['rtl/mac_accel_dma_rob_top.v', 'rtl/mac_dma_rob.v', 'rtl/axi_read_engine_rob.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v']),
 ('tb_mac_accel_axi_wstrb', None, 18,
 ['rtl/mac_accel_axi.v', 'rtl/mac_accel.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v']),
 ('tb_mac_accel_dma_axil_wstrb', None, 26,
 ['rtl/mac_accel_dma_top.v', 'rtl/mac_dma.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v']),
 ('tb_mac_accel_axi_closure', None, 12,
 ['rtl/mac_accel_axi.v', 'rtl/mac_accel.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v']),
 ('tb_mac_accel_dma_closure', None, 12,
 ['rtl/mac_accel_dma_top.v', 'rtl/mac_dma.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v', 'sim/axi_read_mem_model_ooo.v']),
 ('tb_mac_accel_dma_rob_closure', None, 12,
 ['rtl/mac_accel_dma_rob_top.v', 'rtl/mac_dma_rob.v', 'rtl/axi_read_engine_rob.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v', 'sim/axi_read_mem_model_ooo.v']),
 ('tb_mac_accel_axi_protocol',
  None,
  8,
  ['rtl/mac_accel_axi.v', 'rtl/mac_accel.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v']),
 ('tb_mac_accel_consecutive', None, 3, ['rtl/mac_accel.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v']),
 ('tb_axi_read_engine_rob_completion', None, 1, ['rtl/axi_read_engine_rob.v']),
 ('tb_mac_accel_dma_rob_error',
  None,
  12,
  ['rtl/mac_accel_dma_rob_top.v',
   'rtl/mac_dma_rob.v',
   'rtl/axi_read_engine_rob.v',
   'rtl/mac_pe.v',
   'rtl/mac_fifo_async.v',
   'sim/axi_read_mem_model_ooo.v']),
 ('tb_mac_accel_dma_rob_busy',
  None,
  15,
  ['rtl/mac_accel_dma_rob_top.v',
   'rtl/mac_dma_rob.v',
   'rtl/axi_read_engine_rob.v',
   'rtl/mac_pe.v',
   'rtl/mac_fifo_async.v',
   'sim/axi_read_mem_model_ooo.v']),
 ('tb_mac_accel_dma_rob_length',
  None,
  13,
  ['rtl/mac_accel_dma_rob_top.v',
   'rtl/mac_dma_rob.v',
   'rtl/axi_read_engine_rob.v',
   'rtl/mac_pe.v',
   'rtl/mac_fifo_async.v',
   'sim/axi_read_mem_model_ooo.v']),
 ('tb_mac_accel_dma_rob_axil_wstrb',
  None,
  26,
  ['rtl/mac_accel_dma_rob_top.v',
   'rtl/mac_dma_rob.v',
   'rtl/axi_read_engine_rob.v',
   'rtl/mac_pe.v',
   'rtl/mac_fifo_async.v']),
 ('tb_mac_accel_dma_rob_top',
  'V3_MAX_OUT=8',
  7,
  ['rtl/mac_accel_dma_rob_top.v',
   'rtl/mac_dma_rob.v',
   'rtl/axi_read_engine_rob.v',
   'rtl/mac_pe.v',
   'rtl/mac_fifo_async.v',
   'sim/axi_read_mem_model_ooo.v']),
 ('tb_axi_read_engine_rob_ooo',
  'ENG_MAX_OUT=8',
  12,
  ['rtl/axi_read_engine_rob.v', 'sim/axi_read_mem_model_ooo.v']),
 ('tb_mac_dma_rob',
  'DMA_ROB_MAX_OUT=8',
  11,
  ['rtl/mac_dma_rob.v', 'rtl/axi_read_engine_rob.v', 'sim/axi_read_mem_model_ooo.v']),
 ('tb_mac_accel_cdc', None, 8, ['rtl/mac_accel.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v']),
 ('tb_mac_accel_axi_stream',
  None,
  4,
  ['rtl/mac_accel_axi.v', 'rtl/mac_accel.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v']),
 ('tb_axi_read_engine_rob',
  'ENG_MAX_OUT=8',
  4,
  ['rtl/axi_read_engine_rob.v', 'sim/axi_read_mem_model_mo.v']),
 ('tb_mac_dma_rob_power',
  'POWER_MAX_OUT=8',
  4,
  ['rtl/mac_dma_rob.v', 'rtl/axi_read_engine_rob.v', 'sim/axi_read_mem_model_ooo.v']),
 ('tb_mac_accel_dma',
  None,
  7,
  ['rtl/mac_accel_dma_top.v', 'rtl/mac_dma.v', 'rtl/mac_pe.v', 'rtl/mac_fifo_async.v']),
 ('tb_mac_accel_dma_bp',
  None,
  3,
  ['rtl/mac_accel_dma_top.v', 'rtl/mac_dma.v', 'rtl/mac_pe.v', 'rtl/mac_fifo_async.v']),
 ('tb_mac_dma_baseline',
  None,
  5,
  ['rtl/mac_accel_dma_top.v',
   'rtl/mac_dma.v',
   'rtl/mac_pe.v',
   'rtl/mac_fifo_async.v',
   'sim/axi_read_mem_model.v']),
 ('tb_mac_dma_rob_sweep',
  'DMA_ROB_MAX_OUT=8',
  8,
  ['rtl/mac_dma_rob.v', 'rtl/axi_read_engine_rob.v', 'sim/axi_read_mem_model_ooo.v'])]


def check_scoreboard(log, expected):
    """$finish may exit zero on failure; require the complete expected summary."""
    summaries = re.findall(r"TOTAL:\s*(\d+) PASS\s*/\s*(\d+) FAIL", log)
    failures = re.search(r"\b(?:FATAL|ERROR)\b|\bFAIL\s*:|timeout|abort", log, re.I)
    if summaries != [(str(expected), "0")] or failures:
        raise RuntimeError("Missing/incorrect scoreboard summary or failure diagnostic")


def check_power_scoreboard(log, expected):
    """This existing workload prints POWER_TOTAL, not the generic TOTAL colon."""
    if len(re.findall(r"^POWER_REGRESSION_PASS$", log, re.M)) != 1:
        raise RuntimeError("Missing/duplicate power workload completion marker")
    if len(re.findall(r"^POWER_JOB_PASS\b", log, re.M)) != expected:
        raise RuntimeError("Incomplete power workload jobs")
    normalized = re.sub(r"^POWER_TOTAL\s+", "TOTAL: ", log, flags=re.M)
    check_scoreboard(normalized, expected)


def check_sweep_scoreboard(log, expected):
    """Validate the existing CSV sweep's rows and exact summary, fail closed."""
    summaries = re.findall(r"^SWEEP_TOTAL,(\d+),(\d+)$", log, re.M)
    rows = [line.split(',') for line in log.splitlines() if line.startswith('SWEEP,')]
    failures = re.search(r"\b(?:FATAL|ERROR)\b|\bFAIL\s*:|timeout|abort", log, re.I)
    if (summaries != [(str(expected), '0')] or failures or len(rows) != expected
            or any(len(row) != 10 or row[-1] != 'PASS' for row in rows)
            or re.search(r"TOTAL:", log)):
        raise RuntimeError("Missing/incorrect sweep scoreboard or failed/incomplete rows")


def run(argv, cwd):
    result = subprocess.run(argv, cwd=cwd, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, timeout=120)
    print(result.stdout, end="", flush=True)
    result.check_returncode()
    return result.stdout


def main():
    run(["iverilog", "-V"], ROOT)
    with tempfile.TemporaryDirectory(prefix="vector-mac-ci-") as tmp:
        work = Path(tmp)
        for top, define, expected, sources in TESTS:
            case = work / top
            case.mkdir()
            binary = case / "simv"
            command = ["iverilog", "-g2012", "-s", top, "-o", str(binary)]
            if define:
                command.append("-D" + define)
            command.extend(str(ROOT / name) for name in ["sim/" + top + ".v"] + sources)
            print("\nRunning " + top, flush=True)
            run(command, case)
            log = run(["vvp", str(binary)], case)
            checker = {"tb_mac_dma_rob_power": check_power_scoreboard,
                       "tb_mac_dma_rob_sweep": check_sweep_scoreboard}.get(top, check_scoreboard)
            checker(log, expected)
            print(f"CI_PASS {top}: {expected} PASS / 0 FAIL", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
