#!/usr/bin/env python3
"""Run fast Icarus tests in a temporary directory; validate printed scoreboards."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
TESTS = [
    ("tb_mac_accel_dma_rob_top", "V3_MAX_OUT=8", 7,
     ["rtl/mac_accel_dma_rob_top.v", "rtl/mac_dma_rob.v",
      "rtl/axi_read_engine_rob.v", "rtl/mac_pe.v", "rtl/mac_fifo_async.v",
      "sim/axi_read_mem_model_ooo.v"]),
    ("tb_axi_read_engine_rob_ooo", "ENG_MAX_OUT=8", 12,
     ["rtl/axi_read_engine_rob.v", "sim/axi_read_mem_model_ooo.v"]),
    ("tb_mac_dma_rob", "DMA_ROB_MAX_OUT=8", 11,
     ["rtl/mac_dma_rob.v", "rtl/axi_read_engine_rob.v",
      "sim/axi_read_mem_model_ooo.v"]),
    ("tb_mac_accel_cdc", None, 8,
     ["rtl/mac_accel.v", "rtl/mac_fifo_async.v", "rtl/mac_pe.v"]),
]


def check_scoreboard(log, expected):
    """$finish may exit zero on failure; require the complete expected summary."""
    summaries = re.findall(r"TOTAL:\s*(\d+) PASS\s*/\s*(\d+) FAIL", log)
    failures = re.search(r"\b(?:FATAL|ERROR)\b|\bFAIL\s*:|timeout|abort", log, re.I)
    if summaries != [(str(expected), "0")] or failures:
        raise RuntimeError("Missing/incorrect scoreboard summary or failure diagnostic")


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
            check_scoreboard(run(["vvp", str(binary)], case), expected)
            print(f"CI_PASS {top}: {expected} PASS / 0 FAIL", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
