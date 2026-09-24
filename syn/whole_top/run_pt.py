#!/usr/bin/env python3
"""PrimeTime on the accepted whole-top mapped pair; never reruns synthesis."""
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
parser.add_argument("--mapped-from", help="Accepted Step-4 artifact bundle")
parser.add_argument("--run-root", help="New absolute directory; default: unique /tmp path")
args = parser.parse_args()
repo = Path(__file__).resolve().parents[2]
parent = Path(args.mapped_from).resolve() if args.mapped_from else repo / "syn/runs/whole_top_mo8_bus10_mac7p5_20260924_dc"
accept = json.loads((parent / "ACCEPTANCE.json").read_text())
if accept["status"] != "MAPPED_PAIR_VERIFIED_BY_INDEPENDENT_DC_READBACK":
    parser.error("Mapped pair must have accepted DC readback evidence")
for name in ("mac_accel_dma_rob_top_mapped.v", "mac_accel_dma_rob_top.sdc"):
    if hashlib.sha256((parent / "mapped" / name).read_bytes()).hexdigest() != accept["mapped_sha256"][name]:
        parser.error("Mapped input hash mismatch: " + name)
if args.run_root:
    run = Path(args.run_root)
    if not run.is_absolute():
        parser.error("--run-root must be absolute")
    run.mkdir(parents=True, exist_ok=False)
else:
    run = Path(tempfile.mkdtemp(prefix="vectormac-whole-top-pt-", dir="/tmp"))
inputs = run / "inputs"
hashes = {}


def snapshot(src, name):
    dst = inputs / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(str(src), str(dst))
    hashes[name] = hashlib.sha256(dst.read_bytes()).hexdigest()


for name in ("mac_accel_dma_rob_top_mapped.v", "mac_accel_dma_rob_top.sdc"):
    snapshot(parent / "mapped" / name, "mapped/" + name)
for name in ("ACCEPTANCE.json", "manifest.json"):
    snapshot(parent / name, "dc_reference/" + name)
for report in (parent / "readback/reports").glob("*.rpt"):
    snapshot(report, "dc_reference/" + report.name)
for name in ("profile.tcl", "cdc_constraints.tcl", "dc_constraints.tcl",
             "dc_synthesis.tcl", "pt_flow.tcl", "pt_checks.tcl", "run_pt.py"):
    snapshot(repo / "syn/whole_top" / name, "scripts/" + name)
library = Path("/fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db")
snapshot(library, "library.db")
tool = "/ece/synopsys/prime/R-2020.09-SP5-1/bin/pt_shell"
command = [tool, "-f", str(inputs / "scripts/pt_flow.tcl")]
manifest = {
    "top": "mac_accel_dma_rob_top", "max_outstanding": 8,
    "configuration_provenance": "Step-4 elaboration manifest and accepted mapped-pair hashes; no elaboration in PT",
    "parent_bundle": str(parent), "run_root": str(run),
    "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=str(repo), universal_newlines=True).strip(),
    "git_status": subprocess.check_output(["git", "status", "--short"], cwd=str(repo), universal_newlines=True),
    "command": command, "library_source": str(library), "input_sha256": hashes,
}
(run / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
env = os.environ.copy()
env.setdefault("LM_LICENSE_FILE", "1781@license.ece.ucsb.edu")
env.update(WTOP_PT_RUN=str(run))
print("RUN_ROOT=" + str(run), flush=True)
start = time.time()
with (run / "pt.log").open("w") as log:
    result = subprocess.run(command, cwd=str(run), env=env, stdout=log,
                            stderr=subprocess.STDOUT, timeout=900)
errors = []
known_diagnostics = []
for path in [run / "pt.log"] + sorted((run / "reports").glob("*.rpt")):
    for line in re.findall(r"^Error:.*$", path.read_text(errors="replace"), re.MULTILINE):
        target = known_diagnostics if "(PT-063)" in line else errors
        target.append(path.name + ": " + line)
log = (run / "pt.log").read_text(errors="replace")
passed = result.returncode == 0 and not errors and "WTOP_PT_FLOW_COMPLETE" in log
status = {"tool_return_code": result.returncode, "errors": errors,
          "known_nonblocking_diagnostics": known_diagnostics,
          "flow_complete": passed, "wall_seconds": round(time.time() - start, 3),
          "meaning": "Tool/flow status, not a claim of timing closure; inspect reports"}
(run / "status.json").write_text(json.dumps(status, indent=2) + "\n")
print(json.dumps(status, indent=2))
raise SystemExit(0 if passed else 1)
