#!/usr/bin/env python3
"""Bounded reproducible closure stress; logs/hashes/counters live outside the repo."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

from run_ci_rtl import ROOT, TESTS, check_scoreboard

SEEDS = (1, 19, 2345)
MAC_PERIODS = ("7.5", "30", "3.5")
TOPS = ("tb_mac_accel_axi_closure", "tb_mac_accel_dma_closure",
        "tb_mac_accel_dma_rob_closure")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    work = Path(tempfile.mkdtemp(prefix="vector-mac-closure-stress-"))
    print("WORKSPACE=" + str(work), flush=True)
    inputs = {"scripts/run_rtl_closure_stress.py", "scripts/run_ci_rtl.py"}
    for top in TOPS:
        entry = next(t for t in TESTS if t[0] == top)
        inputs.update(["sim/" + top + ".v"] + entry[3])
    hashes = {n: sha(ROOT / n) for n in sorted(inputs)}
    results = []
    for top in TOPS:
        _, define, expected, sources = next(t for t in TESTS if t[0] == top)
        assert define is None and expected == 12
        names = ["sim/" + top + ".v"] + sources
        inputs.update(names)
        case = work / top
        case.mkdir()
        binary = case / "simv"
        argv = ["iverilog", "-g2012", "-s", top, "-o", str(binary)]
        argv += [str(ROOT / n) for n in names]
        compile_result = subprocess.run(argv, capture_output=True, text=True, timeout=120)
        (case / "compile.log").write_text(compile_result.stdout + compile_result.stderr)
        compile_result.check_returncode()
        for seed, period in zip(SEEDS, MAC_PERIODS):
            run = case / ("seed-" + str(seed))
            run.mkdir()
            argv = ["vvp", str(binary), "+SEED=" + str(seed), "+MAC_PERIOD=" + period]
            result = subprocess.run(argv, cwd=run, capture_output=True, text=True, timeout=120)
            log = result.stdout + result.stderr
            (run / "simulation.log").write_text(log)
            result.check_returncode()
            check_scoreboard(log, expected)
            marker = "V1_CLOSURE_COUNTERS" if "axi_closure" in top else "CLOSURE_COUNTERS"
            rows = [s for s in log.splitlines() if s.startswith(marker + " ")]
            if len(rows) != 1:
                raise RuntimeError("Missing/duplicate stress counters")
            counters = {k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", rows[0])}
            if counters.get("seed") != seed or counters.get("jobs") != 12:
                raise RuntimeError("Wrong seed/job count")
            results.append(dict(top=top, seed=seed, mac_period_ns=period,
                                jobs=expected, status="PASS", counters=counters,
                                argv=argv, log=str(run / "simulation.log")))
            print(top + " seed=" + str(seed) + " mac=" + period + ": 12 jobs PASS", flush=True)
    if any(sha(ROOT / n) != h for n, h in hashes.items()):
        raise RuntimeError("Stress input changed during execution")
    data = dict(status="PASS", jobs=sum(r["jobs"] for r in results), results=results,
                source_sha256=hashes, seeds=SEEDS, mac_periods_ns=MAC_PERIODS,
                reset_contract="coordinated startup; zero inter-job resets")
    (work / "results.json").write_text(json.dumps(data, indent=2) + "\n")
    print("RTL_CLOSURE_STRESS_PASS jobs=" + str(data["jobs"]), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
