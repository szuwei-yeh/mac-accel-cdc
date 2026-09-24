#!/usr/bin/env python3
"""Isolated whole-top DC runs. Never overwrites a previous run or block output."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time

parser = argparse.ArgumentParser()
parser.add_argument("--stage", choices=["inspect", "constraints", "synthesis", "readback"], required=True)
parser.add_argument("--mapped-from", help="Prior synthesis run directory, required for readback")
parser.add_argument("--run-root", help="New absolute directory; default: unique /tmp directory")
args = parser.parse_args()
if (args.stage == "readback") != bool(args.mapped_from):
    parser.error("--mapped-from is required only for --stage readback")
repo = Path(__file__).resolve().parents[2]
if args.run_root:
    run = Path(args.run_root)
    if not run.is_absolute():
        parser.error("--run-root must be absolute")
    run.mkdir(parents=True, exist_ok=False)
else:
    run = Path(tempfile.mkdtemp(prefix="vectormac-whole-top-" + args.stage + "-", dir="/tmp"))
inputs = run / "inputs"
sources = [repo / "rtl" / name for name in
           ("mac_accel_dma_rob_top.v", "mac_dma_rob.v", "axi_read_engine_rob.v",
            "mac_fifo_async.v", "mac_pe.v")]
sources += sorted((repo / "syn/whole_top").glob("*.tcl"))
sources += sorted((repo / "syn/whole_top").glob("*.sdc"))
sources += [Path(__file__).resolve()]
hashes = {}
for src in sources:
    rel = src.relative_to(repo)
    dst = inputs / rel
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(str(src), str(dst))
    hashes[str(rel)] = hashlib.sha256(dst.read_bytes()).hexdigest()
parent_manifest = None
if args.mapped_from:
    parent = Path(args.mapped_from).resolve()
    parent_manifest = json.loads((parent / "manifest.json").read_text())
    for name in ("mac_accel_dma_rob_top_mapped.v", "mac_accel_dma_rob_top.sdc"):
        src = parent / "mapped" / name
        dst = inputs / "mapped" / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(str(src), str(dst))
        hashes["mapped/" + name] = hashlib.sha256(dst.read_bytes()).hexdigest()
tool = "/ece/synopsys/syn/R-2020.09-SP4/bin/dc_shell"
library = Path("/fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db")
manifest = {
    "stage": args.stage, "run_root": str(run), "source_root": str(repo),
    "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=str(repo), universal_newlines=True).strip(),
    "git_status": subprocess.check_output(["git", "status", "--short"], cwd=str(repo), universal_newlines=True),
    "source_sha256": hashes, "library": str(library),
    "parent_synthesis_manifest": parent_manifest,
    "library_sha256": hashlib.sha256(library.read_bytes()).hexdigest(),
    "command": [tool, "-no_home_init", "-f", str(inputs / "syn/whole_top/dc_flow.tcl")],
}
(run / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
env = os.environ.copy()
env.setdefault("LM_LICENSE_FILE", "1781@license.ece.ucsb.edu")
env.update(RUN_ROOT=str(run), INPUT_ROOT=str(inputs), WTOP_STAGE=args.stage)
print("RUN_ROOT=" + str(run), flush=True)
start = time.time()
with (run / "dc.log").open("w") as log:
    result = subprocess.run(manifest["command"], cwd=str(run), env=env,
                            stdout=log, stderr=subprocess.STDOUT, timeout=1800)
log = (run / "dc.log").read_text(errors="replace")
errors = re.findall(r"^Error:.*$", log, re.MULTILINE)
for report in sorted((run / "reports").glob("*.rpt")):
    errors += [report.name + ": " + msg for msg in
               re.findall(r"^Error:.*$", report.read_text(errors="replace"), re.MULTILINE)]
passed = result.returncode == 0 and not errors and "WTOP_STAGE_COMPLETE=" + args.stage in log
status = {"tool_return_code": result.returncode, "errors": errors,
          "stage_complete": passed, "wall_seconds": round(time.time() - start, 3)}
(run / "status.json").write_text(json.dumps(status, indent=2) + "\n")
print(json.dumps(status, indent=2))
raise SystemExit(0 if passed else 1)
