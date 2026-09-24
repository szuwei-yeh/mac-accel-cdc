# Whole-top Design Compiler flow

This flow targets `mac_accel_dma_rob_top` with all eight parameters explicitly
passed to elaboration, including MAX_OUTSTANDING=8. It is independent of
`syn/dc_baseline.tcl` and does not replace the canonical DMA/ROB block result.
Timing assumptions and operating limits are in [README.md](README.md).

Run on the UCSB server from the repository:

```sh
python3 syn/whole_top/run_dc.py --stage inspect
python3 syn/whole_top/run_dc.py --stage constraints
python3 syn/whole_top/run_dc.py --stage synthesis
# Verify a previously generated mapped pair without running compile again:
python3 syn/whole_top/run_dc.py --stage readback --mapped-from /absolute/synthesis/run
```

Each invocation creates a unique `/tmp/vectormac-whole-top-<stage>-*` directory
and prints its location. Alternatively supply `--run-root /absolute/new/path`;
an existing directory is refused. Each stage starts from source; `synthesis`
includes elaboration and constraint checks. Python 3.6+ and DC
`/ece/synopsys/syn/R-2020.09-SP4/bin/dc_shell` are required. The launcher uses
the existing license environment, or the audited UCSB license server default.
It does not modify a shell startup file or a system library.

Every run snapshots RTL/scripts, Git state, source hashes and the library
hash. DC executes inside that run directory, with WORK and ALIB caches there.
`dc.log`, `status.json`, `reports/`, `mapped/` and `inputs/` belong to the run;
only publish a result after reviewing its status and evidence. A successful
tool execution does not itself mean timing closure or CDC signoff.

## Stages and acceptance

- `inspect`: analyze, elaborate, rename the parameter-suffixed design to the
  declared top, link and record hierarchy, library attributes and sequential
  register names. Generic Verilog is inspection evidence, not a gate netlist.
- `constraints`: resolve exact sequential-cell bindings, apply the unchanged
  Step-3 profile, report clocks/I/O/exceptions and classify cross-domain
  sequential cones. The unexpanded FIFO read `MUX_OP` prevents a complete
  precompile payload cone check. That known representation must be present,
  with 528 bus-domain source bits and 33 MAC-domain captures tracing to it;
  the report explicitly marks payload connectivity deferred, not passing.
- `synthesis`: `compile_ultra -no_autoungroup` retains hierarchy for CDC
  traceability. This new whole-top setting is recorded; no retiming or clock
  gating option is added. Save mapped Verilog/DDC/resolved SDC and reports,
  then rebind actual registers and check mapped CDC connectivity. Read the
  exported Verilog/SDC into a fresh DC design and repeat checks. No PT,
  placement/routing, extraction, equivalence or gate-level simulation is run.
- `readback`: start a separate DC process, snapshot the prior run's mapped
  Verilog/SDC and its manifest, then perform link, report, CDC and exception
  visibility checks without compile. This also works on the saved artifact
  bundle containing top-level `manifest.json` and `mapped/`.

The mapped audit must inspect **all** bus↔MAC sequential crossing endpoints,
reject unclassified sources, verify one-to-one Gray/result/latency bit
connections, verify FIFO payload bit-to-word connectivity and retain all 13
two-flop synchronizer chains. It also rejects unclocked or multiply-clocked
sequential cells. Register names/counts are deliberately strict: if mapping
shares, removes or renames registers, investigate the saved netlist rather
than silently weakening the checks.

The GTECH inventory itself is not CDC signoff: unmapped arithmetic/mux
operators can stop graph traversal. The post-mapping check is mandatory.
Max/min timing reports also check that bounded crossings have active max
checks, asynchronous hold checks are excluded, all 13 stage-1→stage-2 paths
remain timed, and MAC read-pointer→payload capture retains same-domain timing.
DC explicitly queried false paths can appear as `(Path is unconstrained)`;
the check distinguishes that from a path with a required time and slack.
Graph connectivity does not establish metastability robustness, reset safety,
equivalence or implementation skew.

## Error handling and interpretation

DC's default `sh_continue_on_error=true` is overridden **inside this tool
session** so sourcing errors stop the flow. The launcher also checks process
return code, completion marker and `Error:` messages in logs/reports. Files
can exist after a failed audit; do not treat them as accepted outputs.

Library attributes verify 1 ns units and typical/1.8 V/25 C. `report_lib` is
not required: on this server it calls an unavailable Library Compiler helper,
although direct `.db` loading works. No dependency workaround is applied.

The profile intentionally excludes external reset timing and leaves drive,
load and clock uncertainty unspecified. Any TIM-216 reset-input warning must
remain documented. Ideal pre-layout clocks, no extracted RC, one typical
corner and the portfolio I/O assumptions limit all resulting slack/area
interpretations. Review design warnings, ignored exceptions and fanout
violations separately from setup/hold slack.

## Package for PrimeTime

After reviewing the synthesis and independent readback reports, package their
matching mapped pair. For example, choose new run directories:

```sh
python3 syn/whole_top/run_dc.py --stage synthesis --run-root /tmp/wtop-dc-new
python3 syn/whole_top/run_dc.py --stage readback \
  --mapped-from /tmp/wtop-dc-new --run-root /tmp/wtop-readback-new
python3 syn/whole_top/package_dc.py \
  --synthesis /tmp/wtop-dc-new --readback /tmp/wtop-readback-new \
  --output syn/runs/whole_top_accepted_new
python3 syn/whole_top/run_pt.py --mapped-from syn/runs/whole_top_accepted_new
```

The packager requires both runs to have completed, checks input snapshot hashes,
the parent synthesis manifest, library hash and exact netlist/SDC identity, and
refuses an existing destination. It copies the pair, snapshots and reports,
then writes `ACCEPTANCE.json` last. It does not launch EDA tools or establish
timing closure. WORK/ALIB caches and optional DDC are not required by PT and are
left in the original synthesis run. Keep those original runs if needed.

The historical 2026-09-24 run used a reviewed recovery: compile/export succeeded,
but reporting helpers initially failed; a corrected independent readback passed
on the same pair without recompiling. Its original failed status was retained.
The public packager deliberately rejects incomplete runs; it does not automate
that historical recovery or rewrite its status.
