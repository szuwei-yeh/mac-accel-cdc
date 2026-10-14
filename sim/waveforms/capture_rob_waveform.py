#!/usr/bin/env python3
"""Capture and audit the first phase of the unchanged standalone ROB regression."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TB = "tb_axi_read_engine_rob_waveform.tb."
SOURCES = ["sim/waveforms/tb_axi_read_engine_rob_waveform.v",
           "sim/tb_axi_read_engine_rob_ooo.v", "rtl/axi_read_engine_rob.v",
           "sim/axi_read_mem_model_ooo.v"]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def vcd_edges(path):
    """Yield pre-edge and settled post-edge snapshots, including NBA updates.

    This intentionally handles Icarus's scalar/binary VCD subset only. Sampling
    settled values as handshake payloads would read the *next* cycle's signals.
    """
    names, scopes, values, changes = {}, [], {}, {}
    stamp, header = 0, True

    def edge():
        before = {name: values.get(code) for name, code in names.items()}
        after_values = values | changes
        after = {name: after_values.get(code) for name, code in names.items()}
        if before.get(TB + "clk") == 0 and after.get(TB + "clk") == 1:
            return stamp, before, after
        return None

    text = path.read_text()
    require(re.search(r"\$timescale\s+1ps\s+\$end", text), "Expected 1ps VCD units")
    for line in text.splitlines():
        fields = line.split()
        if not fields:
            continue
        if header:
            if fields[0] == "$scope":
                scopes.append(fields[2])
            elif fields[0] == "$upscope":
                scopes.pop()
            elif fields[0] == "$var":
                name = fields[4].lstrip("\\")
                names[".".join(scopes + [name])] = fields[3]
            elif fields[0] == "$enddefinitions":
                header = False
        elif line.startswith("#"):
            result = edge()
            if result:
                yield result
            values.update(changes)
            changes = {}
            stamp = int(line[1:])
        elif line[0] in "01xz":
            changes[line[1:]] = int(line[0]) if line[0] in "01" else None
        elif line[0] == "b":
            bits, code = fields
            changes[code] = None if re.search("[xz]", bits) else int(bits[1:], 2)
    result = edge()
    if result:
        yield result


def audit(path):
    requests, completions, retirements, non_head = [], [], [], []
    received, outputs, previous_rid, interleave = {}, [], None, 0
    previous = None
    response_beats = 0
    for stamp, before, after in vcd_edges(path):
        def get(name, post=False):
            key = TB + name
            state = after if post else before
            require(key in state, "Missing VCD signal: " + key)
            return state[key]

        if get("resetn") != 1:
            continue
        require(get("engine_err") == 0, "DUT error")
        if previous:
            for valid, ready, payload in [
                    ("ARVALID", "ARREADY", ["ARID", "ARADDR", "ARLEN"]),
                    ("RVALID", "RREADY", ["RID", "RDATA", "RRESP", "RLAST"]),
                    ("out_valid", "out_ready_tb", ["out_data", "out_last"])]:
                if previous[TB + valid] == 1 and previous[TB + ready] == 0:
                    require(get(valid) == 1 and all(
                        get(n) == previous[TB + n] for n in payload),
                        "Unstable stalled channel: " + valid)
        previous = before
        if get("ARVALID") == 1 and get("ARREADY") == 1:
            rid, address, beats = get("ARID"), get("ARADDR"), get("ARLEN") + 1
            require(rid == len(requests) and address == len(requests) * 64 and beats == 16,
                    "First-phase AR sequence/geometry mismatch")
            requests.append(dict(time_ns=stamp / 1000, id=rid, address=address, beats=beats))
            received[rid] = 0
            require(get("dut.alloc_ptr", True) == (rid + 1) % 4, "Allocation pointer mismatch")
        if get("RVALID") == 1 and get("RREADY") == 1:
            rid, data = get("RID"), get("RDATA")
            require(rid in received, "Response RID has no allocation")
            beat = received[rid]
            require(beat < 16 and get("RRESP") == 0 and data == rid * 16 + beat + 1,
                    "Response payload/order/status mismatch")
            require(get("RLAST") == int(beat == 15), "Incorrect RLAST")
            index = rid * 16 + beat
            require(get("dut.rob_widx") == index, "Incorrect RID-indexed write address")
            require(get(f"dut.rob_data[{index}]", True) == data, "ROB storage mismatch")
            require(get(f"dut.e_recv[{rid}]", True) == beat + 1, "ROB receive count mismatch")
            received[rid] += 1
            response_beats += 1
            interleave += int(previous_rid is not None and previous_rid != rid)
            previous_rid = rid
            if get("RLAST"):
                require(get(f"dut.e_complete[{rid}]", True) == 1, "Completion flag not set")
                completions.append(dict(time_ns=stamp / 1000, id=rid))
                head = get("dut.head_ptr")
                if rid != head and get(f"dut.e_complete[{head}]") == 0:
                    require(get("out_valid") == 0 and get("dut.head_ptr", True) == head,
                            "Non-head response bypassed incomplete head")
                    non_head.append(dict(time_ns=stamp / 1000, id=rid, blocked_head=head))
        if get("out_valid") == 1 and get("out_ready_tb") == 1:
            index = len(outputs)
            require(get("out_data") == index + 1 and get("out_last") == int(index == 63),
                    "Output scoreboard mismatch")
            require(get("dut.head_ptr") == index // 16 and get("dut.drain_idx") == index % 16,
                    "Output did not drain in allocation order")
            require(get(f"dut.e_complete[{index // 16}]") == 1,
                    "Output retired an incomplete ROB entry")
            outputs.append(dict(time_ns=stamp / 1000, data=get("out_data")))
            if index % 16 == 15:
                retirements.append(dict(time_ns=stamp / 1000, id=index // 16))
                require(get("dut.head_ptr", True) == (index // 16 + 1) % 4,
                        "Head did not advance after final output beat")
        require(get("dut.occupancy", True) == len(requests) - len(retirements),
                "ROB occupancy/credit mismatch")
    completion_ids = [event["id"] for event in completions]
    require(len(requests) == 4 and response_beats == 64 and len(outputs) == 64,
            "Incomplete first-phase trace")
    require(all(count == 16 for count in received.values()), "Missing response beats")
    require(sorted(completion_ids) == list(range(4)) and completion_ids != list(range(4)),
            "No out-of-order burst completion")
    require(non_head and interleave > 0, "Vacuous non-head completion or interleaving")
    return dict(status="WAVEFORM_PASS", scope="standalone axi_read_engine_rob",
                configuration=dict(outstanding=4, command_beats=64, burst_beats=16,
                                   memory_latency_cycles=100, ooo_en=1, backpressure=0,
                                   clock_period_ns=10, model_reset_lfsr="0xACE1"),
                requests=requests, completions=completions, retirements=retirements,
                blocked_non_head_completions=non_head, interleave_events=interleave,
                response_beats=response_beats, output_beats=len(outputs),
                first_output=outputs[0], last_output=outputs[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="New directory outside the repository")
    args = parser.parse_args()
    work = args.output.expanduser().resolve() if args.output else Path(
        tempfile.mkdtemp(prefix="vector-mac-rob-waveform-output-"))
    require(not work.is_relative_to(ROOT), "Keep generated artifacts outside the repository")
    if args.output:
        work.mkdir(parents=True, exist_ok=False)
    commands = [
        ["iverilog", "-g2012", "-DENG_MAX_OUT=4", "-s", "tb_axi_read_engine_rob_waveform",
         "-o", str(work / "rob.vvp")] + [str(ROOT / name) for name in SOURCES],
        ["vvp", str(work / "rob.vvp"), "+ROB_VCD=" + str(work / "rob_ooo.vcd")]]
    logs = []
    for command in commands:
        print(shlex.join(command), flush=True)
        result = subprocess.run(command, cwd=work, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=120)
        logs.append(result.stdout)
        (work / "simulation.log").write_text("".join(logs))
        result.check_returncode()
    sys.path.insert(0, str(ROOT / "scripts"))
    from run_ci_rtl import check_scoreboard
    check_scoreboard(logs[-1], 12)
    report = audit(work / "rob_ooo.vcd")
    report["scoreboard"] = "12 PASS / 0 FAIL"
    report["source_sha256"] = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                               for name in SOURCES + ["sim/waveforms/rob_ooo.gtkw", "sim/waveforms/capture_rob_waveform.py"]}
    report["vcd_sha256"] = hashlib.sha256((work / "rob_ooo.vcd").read_bytes()).hexdigest()
    report["commands"] = commands
    (work / "trace_summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print("12 PASS / 0 FAIL; WAVEFORM_PASS: RID storage, non-head completion, ordered output")
    for key in ("requests", "completions", "retirements"):
        print(key + ": " + " -> ".join(str(event["id"]) for event in report[key]))
    print(f"interleave events: {report['interleave_events']}; artifacts: {work}")
    print("Open: " + shlex.join(["gtkwave", str(work / "rob_ooo.vcd"), str(ROOT / "sim/waveforms/rob_ooo.gtkw")]))


if __name__ == "__main__":
    main()
