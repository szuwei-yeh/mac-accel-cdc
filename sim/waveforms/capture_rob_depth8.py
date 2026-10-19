#!/usr/bin/env python3
"""Reproduce and audit the directed depth-8 standalone ROB viewer evidence."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tempfile

from capture_rob_waveform import TB, require, vcd_edges

ROOT = Path(__file__).resolve().parents[2]
PROFILE = Path(__file__).resolve().parent / "depth8"
SOURCES = ["rtl/axi_read_engine_rob.v", "sim/axi_read_mem_model_ooo.v",
           "sim/tb_axi_read_engine_rob_ooo.v"]
STIMULUS_MARKER = ("    //----------------------------------------------------\n"
                   "    // Stimulus\n"
                   "    //----------------------------------------------------\n")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(path):
    """Check actual handshakes before each edge, then settled DUT updates.

    Keep the observation top name shared with the depth-4 capture so both use
    the existing Icarus VCD parser. The two wrappers compile in isolation.
    All required signals are mandatory; Python -O cannot disable these checks.
    """
    requests, completions, retirements, outputs = [], [], [], []
    received, blocked = {}, []
    peak, previous_rid, interleave, previous = 0, None, 0, None
    final_head, final_occupancy = None, None
    for stamp, before, after in vcd_edges(path):
        def get(name, post=False):
            key = TB + name
            state = after if post else before
            require(key in state and state[key] is not None,
                    "Missing/unknown VCD signal: " + key)
            return state[key]

        if get("resetn") != 1:
            continue
        require(get("engine_err") == 0, "DUT error")
        require(get("dut.MAX_OUTSTANDING") == 8, "DUT is not depth 8")
        alias = "tb_axi_read_engine_rob_waveform.e_complete"
        expected_alias = sum(get(f"dut.e_complete[{i}]", True) << i for i in range(8))
        require(after.get(alias) == expected_alias, "Packed observation alias differs from DUT")
        if previous:
            for valid, ready, payload in [
                    ("ARVALID", "ARREADY", ["ARID", "ARADDR", "ARLEN"]),
                    ("RVALID", "RREADY", ["RID", "RDATA", "RRESP", "RLAST"]),
                    ("out_valid", "out_ready_tb", ["out_data", "out_last"])]:
                if previous[TB + valid] == 1 and previous[TB + ready] == 0:
                    require(get(valid) == 1 and all(get(n) == previous[TB + n] for n in payload),
                            "Unstable stalled channel: " + valid)
        previous = before
        ns = stamp / 1000
        if get("ARVALID") and get("ARREADY"):
            entry, beats = get("ARID"), get("ARLEN") + 1
            require(entry == len(requests) and beats == 16 and get("ARADDR") == entry * 64,
                    "Allocation sequence/geometry mismatch")
            require(get("dut.M_AXI_ARSIZE") == 2 and get("dut.M_AXI_ARBURST") == 1,
                    "Unexpected AXI request size/burst")
            require(get(f"dut.e_valid[{entry}]") == 0, "Reallocated a live entry")
            requests.append(dict(time_ns=ns, id=entry, beats=beats, address=get("ARADDR")))
            received[entry] = 0
            require(get("dut.alloc_ptr", True) == (entry + 1) % 8, "Allocation pointer mismatch")
        if get("RVALID") and get("RREADY"):
            entry = get("RID")
            require(entry in received, "Response without allocation")
            beat = received[entry]
            index = entry * 16 + beat
            require(beat < 16 and get("RDATA") == index + 1 and get("RRESP") == 0,
                    "Response data/order/status mismatch")
            require(get("RLAST") == int(beat == 15), "Incorrect RLAST")
            require(get("dut.rob_widx") == index and
                    get(f"dut.rob_data[{index}]", True) == index + 1,
                    "RID-indexed ROB storage mismatch")
            require(get(f"dut.e_recv[{entry}]", True) == beat + 1, "Receive count mismatch")
            received[entry] += 1
            interleave += int(previous_rid is not None and previous_rid != entry)
            previous_rid = entry
            if get("RLAST"):
                require(get(f"dut.e_complete[{entry}]", True) == 1, "Completion flag not set")
                completions.append(dict(time_ns=ns, id=entry))
        accepted = bool(get("out_valid") and get("out_ready_tb"))
        entry, beat = get("dut.head_ptr"), get("dut.drain_idx")
        drained = get("dut.entry_drained")
        require(drained == int(accepted and beat == get(f"dut.e_exp[{entry}]") - 1),
                "entry_drained does not match final accepted beat")
        if accepted:
            index = len(outputs)
            require(entry == index // 16 and beat == index % 16,
                    "Missing/duplicate/interleaved output entry or beat")
            require(get(f"dut.e_valid[{entry}]") == get(f"dut.e_complete[{entry}]") == 1,
                    "Output from an invalid/incomplete entry")
            require(get("out_data") == index + 1 and get("out_last") == int(index == 127),
                    "Output data/command-last scoreboard mismatch")
            outputs.append(dict(time_ns=ns, entry=entry, beat=beat, data=get("out_data"),
                                out_last=get("out_last"), entry_drained=drained))
        if drained:
            require(get("dut.head_ptr", True) == (entry + 1) % 8 and
                    get("dut.drain_idx", True) == 0, "Retirement pointer/index mismatch")
            retirements.append(dict(time_ns=ns, id=entry, head_after=get("dut.head_ptr", True),
                                    data=get("out_data"), accepted_beats=beat + 1))
        elif not get("dut.cmd_valid"):
            require(get("dut.head_ptr", True) == entry, "Head advanced without retirement")
        for i in range(8):
            require(get(f"dut.e_valid[{i}]", True) == int(i < len(requests) and i >= len(retirements)),
                    "Entry freed before its final handshake or not freed at retirement")
        final_head, final_occupancy = get("dut.head_ptr", True), get("dut.occupancy", True)
        peak = max(peak, final_occupancy)
        require(final_occupancy == len(requests) - len(retirements), "Occupancy/credit mismatch")
        if final_head == 0 and get("dut.e_complete[0]", True) == 0:
            ready = [i for i in range(1, 8) if get(f"dut.e_complete[{i}]", True)]
            if len(ready) >= 2:
                require(get("out_valid", True) == 0, "Non-head completion bypassed head")
                blocked.append(dict(time_ns=ns + 1, head_ptr=0, completed=ready,
                                    e_complete_7_0=f"{expected_alias:08b}", out_valid=0,
                                    occupancy=final_occupancy))
    require(len(requests) == 8 and peak == 8 and len(outputs) == 128,
            "Did not allocate eight entries / transfer 128 output beats")
    require(all(count == 16 for count in received.values()), "Missing response beats")
    require(sorted(x["id"] for x in completions) == list(range(8)) and
            completions[-1]["id"] == 0, "Head did not complete after all seven later entries")
    require([x["id"] for x in retirements] == list(range(8)), "Retirement order mismatch")
    require(final_head == final_occupancy == 0, "Final head did not wrap / entries remain live")
    require(requests[-1]["time_ns"] < completions[0]["time_ns"],
            "Eight requests were not outstanding before first burst completion")
    require(interleave > 0, "Response interleaving was not exercised")
    cursor = next((x for x in blocked if len(x["completed"]) == 7), None)
    require(cursor is not None, "Missing blocked-head snapshot with seven complete entries")
    summary = []
    for entry in range(8):
        beats = [x for x in outputs if x["entry"] == entry]
        summary.append(dict(entry=entry, allocation_ns=requests[entry]["time_ns"],
                            completion_ns=next(x["time_ns"] for x in completions if x["id"] == entry),
                            first_output_handshake_ns=beats[0]["time_ns"],
                            retirement_ns=beats[-1]["time_ns"], beats=len(beats),
                            data_first=beats[0]["data"], data_last=beats[-1]["data"]))
    return dict(status="DEPTH8_WAVEFORM_PASS", scope="Standalone ROB test, depth 8",
                max_outstanding=peak, requests=requests, completions=completions,
                retirements=retirements, blocked_cursor=cursor, summary=summary,
                output_beats=len(outputs), response_beats=sum(received.values()),
                interleave_events=interleave, final_head=final_head,
                final_occupancy=final_occupancy), outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="New directory outside the repository")
    args = parser.parse_args()
    provenance = json.loads((PROFILE / "provenance.json").read_text())
    for name, expected in provenance["reproduction_input_sha256"].items():
        require(sha(ROOT / name) == expected, "Recorded reproduction input drift: " + name)
    require(sha(ROOT / provenance["image"]) == provenance["image_sha256"],
            "Published depth-8 screenshot drift")
    work = args.output.expanduser().resolve() if args.output else Path(
        tempfile.mkdtemp(prefix="vector-mac-rob-depth8-"))
    require(not work.is_relative_to(ROOT), "Keep generated artifacts outside the repository")
    if args.output:
        work.mkdir(parents=True, exist_ok=False)
    original = (ROOT / "sim/tb_axi_read_engine_rob_ooo.v").read_text()
    latency = ".ar_latency(mem_latency)"
    require(original.count(STIMULUS_MARKER) == original.count(latency) == 1,
            "Expected exactly one stimulus block and model latency connection")
    adapted = original.split(STIMULUS_MARKER)[0].replace(
        latency, ".ar_latency(mem_latency + ((ARID == 0) ? 16'd200 : 16'd0))")
    adapted += (PROFILE / "stimulus.vh").read_text() + "\nendmodule\n"
    generated = work / "directed_tb.v"
    generated.write_text(adapted)
    wrapper = PROFILE / "tb_axi_read_engine_rob_waveform.v"
    commands = [
        ["iverilog", "-g2012", "-DENG_MAX_OUT=8", "-s", "tb_axi_read_engine_rob_waveform",
         "-o", str(work / "rob_depth8.vvp"), str(wrapper), str(generated)] +
        [str(ROOT / name) for name in SOURCES[:2]],
        ["vvp", str(work / "rob_depth8.vvp"), "+ROB_VCD=" + str(work / "rob_depth8.vcd")]]
    logs = []
    for command in commands:
        print(shlex.join(command), flush=True)
        result = subprocess.run(command, cwd=work, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=120)
        logs.append(result.stdout)
        (work / "simulation.log").write_text("".join(logs))
        result.check_returncode()
    require(len(re.findall(r"^\s*PASS\s+in-order output;", logs[-1], re.M)) == 1 and
            "DEPTH8_SCOREBOARD: 1 PASS / 0 FAIL" in logs[-1] and
            not re.search(r"\b(?:FAIL:|FATAL:)", logs[-1]), "Dedicated scoreboard did not pass")
    report, outputs = audit(work / "rob_depth8.vcd")
    # Bind the caption to measured events; never use these values to drive RTL.
    for name, expected in provenance["measured_trace"].items():
        require(report[name] == expected, "Measured trace differs from README capture: " + name)
    report.update(configuration=provenance["configuration"], scoreboard="1 PASS / 0 FAIL",
                  vcd_sha256=sha(work / "rob_depth8.vcd"), generated_testbench_sha256=sha(generated),
                  source_sha256={str(p.relative_to(ROOT)): sha(p) for p in
                                 [ROOT / x for x in SOURCES] + [wrapper, PROFILE / "stimulus.vh", Path(__file__)]},
                  commands=commands)
    (work / "trace_summary.json").write_text(json.dumps(report, indent=2) + "\n")
    for name, rows in [("output_handshakes.csv", outputs), ("entry_timing.csv", report["summary"])]:
        with (work / name).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    for name in ["rob_depth8_surfer.ron", "overview.sucl", "blocked.sucl"]:
        shutil.copyfile(PROFILE / name, work / name)
    (work / ".surfer").mkdir()
    shutil.copyfile(PROFILE / "config.toml", work / ".surfer/config.toml")
    print("1 PASS / 0 FAIL; DEPTH8_WAVEFORM_PASS; max outstanding=8; 128/128 output beats")
    for name in ["requests", "completions", "retirements"]:
        print(name + ": " + " -> ".join(str(x["id"]) for x in report[name]))
    print("blocked cursor: 2276 ns; complete[7:0]=11111110; head=0; out_valid=0; occupancy=8")
    print("final: head=0, occupancy=0; artifacts: " + str(work))
    print("View (from output directory): " + shlex.join([
        "surfer", "rob_depth8.vcd", "--state-file", "rob_depth8_surfer.ron",
        "--command-file", "overview.sucl"]))


if __name__ == "__main__":
    main()
