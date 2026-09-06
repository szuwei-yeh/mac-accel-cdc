# Synopsys Design Compiler Flow

This directory contains the synthesis scripts for the ROB/outstanding-read
subsystem. The canonical flow creates one self-contained, immutable directory
per run so reports from different parameters cannot overwrite each other.

## Directory layout

```text
syn/
├── README.md                 This guide
├── constraints_rob.sdc      Shared clock and I/O constraints
├── dc_baseline.tcl          Canonical parameterized DC flow
├── run_baseline_dc.sh       Canonical launcher and manifest generator
├── dc_power_targeted.tcl    Symmetric baseline/gated power synthesis flow
├── run_power_activity.sh    Deterministic VCS-to-SAIF activity flow
├── run_power_targeted.sh    Immutable targeted-power launcher
├── dc_rob.tcl               Original compatibility flow
├── run_dc.sh                Original compatibility launcher
└── runs/                    Generated output; ignored by Git
    ├── mac_dma_rob_mo4_10ns/
    ├── mac_dma_rob_mo8_10ns/
    ├── mac_dma_rob_mo8_10ns_araddr_inc_v2/
    └── legacy_2026-07_default_mo4/
        ├── logs/
        ├── mapped/
        ├── reports/
        ├── root_artifacts/
        └── work/
```

`runs/` is the only home for retained synthesis output. Each canonical run has:

- `manifest.txt`: parameters, tool/library paths, Git state, and SHA-256 input
  hashes;
- `inputs/`: the exact RTL and synthesis input snapshot used for archived runs;
- `reports/`: QoR, area, hierarchy, design checks, constraints, and detailed
  timing paths;
- `mapped/`: DDC, mapped Verilog, and output SDC;
- `dc.log`: the complete Design Compiler transcript.

## Canonical baseline flow

Run from the repository root on the UCSB ECE server:

```bash
cd /fs/student/szu-wei/work/mac-accel-cdc

MAX_OUTSTANDING=8 \
RUN_ID=mac_dma_rob_mo8_10ns \
bash syn/run_baseline_dc.sh
```

The launcher refuses to reuse an existing `RUN_ID`. Choose a unique name for a
new experiment, for example:

```bash
MAX_OUTSTANDING=4 CLK_PERIOD=8.0 \
RUN_ID=mac_dma_rob_mo4_8ns_addr_pipe_v1 \
bash syn/run_baseline_dc.sh
```

Supported overrides are:

| Variable | Default | Allowed/use |
|---|---:|---|
| `TOP` | `mac_dma_rob` | `mac_dma_rob` or `axi_read_engine_rob` |
| `MAX_OUTSTANDING` | `8` | `2`, `4`, or `8` |
| `CLK_PERIOD` | `10.0` | Clock period in ns |
| `TARGET_LIB` | UCSB OSU 0.18 um library | Path to a Synopsys `.db` |
| `RUN_ID` | Derived from the fields above | Unique output directory name |

The current UCSB setup uses:

```text
DC:      /ece/synopsys/syn/R-2020.09-SP4/bin/dc_shell
Library: /fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db
```

The synthesis top `mac_dma_rob` reads these RTL sources:

```text
rtl/axi_read_engine_rob.v
rtl/mac_dma_rob.v
```

## Current clean baselines

Both runs use a 10 ns clock, 1 ns input delay, 1 ns output delay, and maximum
fanout 32.

| Run | Critical path | Slack | Logic levels | Total cell area | Leaf cells |
|---|---:|---:|---:|---:|---:|
| `mac_dma_rob_mo4_10ns` | 5.4506 ns | 3.5494 ns | 31 | 968,782 | 22,396 |
| `mac_dma_rob_mo8_10ns` | 5.4973 ns | 3.5027 ns | 31 | 1,160,928 | 26,868 |

The depth-8 critical path starts at
`u_read_engine/issue_elem_reg[0]` and ends at `M_AXI_ARADDR[31]`. It implements
the address expression `cmd_addr_r + (issue_elem << 2)` through a long carry
chain. The shift itself has zero cell delay in the timing report.

The depth-4 result exactly reproduces the retained July 2026 default result.
Changing the ROB depth from 4 to 8 increases total cell area by 192,146
(19.83%) while critical-path delay changes by 0.0467 ns (0.86%). Detailed
methodology and interpretation are kept in the private local work log.

## First synthesis-driven optimization

`mac_dma_rob_mo8_10ns_araddr_inc_v2` uses the same depth-8 configuration,
constraints, library, and `compile_ultra` flow as the canonical depth-8
baseline. It replaces `cmd_addr_r + (issue_elem << 2)` at the AR output with an
incrementing address register that advances only on an accepted AR request.

| Metric | Depth-8 baseline | Incrementing AR address | Change |
|---|---:|---:|---:|
| Worst-path data arrival | 5.4973 ns | 5.0480 ns | -0.4493 ns (-8.17%) |
| Slack | 3.5027 ns | 4.7923 ns | +1.2896 ns |
| QoR critical-path length | 5.50 ns | 4.05 ns | -1.45 ns (-26.36%) |
| Logic levels | 31 | 11 | -20 (-64.52%) |
| Total cell area | 1,160,928 | 1,160,580 | -348 (-0.030%) |

The new global worst path starts at the `rst` input and terminates at
`u_read_engine/retire_elem_reg[14]/D`. No `M_AXI_ARADDR` endpoint appears in
the optimized global top-10 report. The worst ARADDR path is now only the
address register's clock-to-Q plus wiring: 0.2221 ns arrival and 8.7779 ns
slack. The worst path that updates the address register has 4.5311 ns arrival
and 5.3096 ns slack, so the address adder did not become the new global
bottleneck. The private local work log retains the full implementation,
verification, and path interpretation record.

A read-only follow-up characterization of the optimized mapped DDC found that
all global top-20 paths are synchronous-reset/control paths into `retire_elem`
or `issue_elem`. After excluding only the external `rst` port for diagnostic
reporting, the worst functional path is
`u_read_engine/head_ptr_reg[1] -> buf_a_reg[127][12]`, through the flat ROB read
mux and wrapper capture logic, with 4.6982 ns arrival and +5.1322 ns slack. The
isolated `retire_elem` Q-to-D arithmetic path is 2.0926 ns with +7.7290 ns
slack. With more than half a cycle of functional margin, no additional
microarchitectural timing optimization is currently justified.

## Clock-period sweep

The optimized depth-8 RTL was independently recompiled at 10, 8, 6, and 5 ns.
Only the clock period changed; every run retained an input snapshot, manifest,
hashes, DC log, mapped outputs, and detailed timing/area reports.

| Period | Frequency | Setup | WNS | Critical length | Main path class |
|---:|---:|---|---:|---:|---|
| 10 ns | 100 MHz | PASS | +4.7923 ns | 4.05 ns | reset/control |
| 8 ns | 125 MHz | PASS | +1.3094 ns | 6.52 ns | address-state update mapping |
| 6 ns | 166.7 MHz | PASS | +1.1860 ns | 4.64 ns | ROB read/data selection |
| 5 ns | 200 MHz | PASS | +0.0389 ns | 4.80 ns | ROB read/data selection |

At 5 ns the global and functional worst path is:

```text
head_ptr
  -> flat ROB read mux
  -> engine_out_data
  -> wrapper buf_a/stream_b capture
```

The 200 MHz point closes in the current pre-layout DC/library/ideal-clock
model, but 38.9 ps is not useful physical-design margin. It is not a
post-layout frequency claim. The canonical target remains 100 MHz.

## Targeted `buf_a` clock-gating experiment

The power flow compares ordinary `compile_ultra` with a targeted
`compile_ultra -gate_clock` run. It does not modify functional RTL. Before
compile, every register outside `buf_a_reg[*]` is explicitly excluded with
`set_clock_gating_objects`; the 4096 `buf_a` bits are the only included scope.

The exact style is:

```tcl
set_clock_gating_style \
    -sequential_cell latch:osu018_stdcells/LATCH \
    -positive_edge_logic {and:osu018_stdcells/AND2X1} \
    -minimum_bitwidth 16 \
    -max_fanout 16 \
    -no_sharing \
    -control_point none
```

This produced 256 independent 16-bit banks and gated exactly 4096 registers.
No ROB, control, CDC, or other buffer register was gated. The correct wording
is **synthesis-inserted latch-based clock gating using discrete LATCH + AND
cells**. `osu018_stdcells.db` has no dedicated integrated clock-gating cell.

### Deterministic activity source

`run_power_activity.sh` builds a dedicated `MAX_OUTSTANDING=8`, 100 MHz VCS
workload with:

- fixed 40-cycle memory latency;
- fixed OOO-response selection state;
- deterministic AR and output backpressure;
- one measured length-64 active job;
- a separate 512-cycle idle window;
- reset/idle wake-up and consecutive length 1/16/33 follow-up jobs.

VCS VPD memory tracing and `vpd2vcd +includemda` preserve all unpacked `buf_a`
and ROB memory words. `vcd2saif` then creates independent active and idle SAIF
files. Baseline and gated synthesis consume the same files and reject missing
activity inputs. The accepted run achieved 100% annotation of ports, all 4232
RTL-invariant sequential objects, and all 4096 `buf_a` bits.

### Reproduction

Run from a clean checkout in the UCSB ECE Synopsys environment:

```bash
RUN_ID=mac_dma_rob_mo8_power_activity_final \
bash syn/run_power_activity.sh

MODE=baseline \
ACTIVITY_RUN=mac_dma_rob_mo8_power_activity_final \
RUN_ID=mac_dma_rob_mo8_power_baseline_final \
bash syn/run_power_targeted.sh

MODE=gated \
ACTIVITY_RUN=mac_dma_rob_mo8_power_activity_final \
RUN_ID=mac_dma_rob_mo8_power_gated_final \
bash syn/run_power_targeted.sh
```

The launchers use repository-relative inputs, do not read private notes, and
do not require a temporary working path. They refuse to reuse a run directory.
The canonical OSU database path and tool executables must be available in the
UCSB environment.

### Result

| SAIF-driven pre-layout metric | Baseline | Gated | Change |
|---|---:|---:|---:|
| Active dynamic power | 76.1114 mW | 35.3225 mW | -53.59% |
| Active total power | 76.1134 mW | 35.3242 mW | -53.59% |
| Idle dynamic power | 71.6315 mW | 30.6423 mW | -57.22% |
| Energy per measured job | 221.870 nJ | 102.970 nJ | -53.59% |
| Setup WNS at 10 ns | +4.7923 ns | +2.4278 ns | PASS |

The gated worst path is a half-cycle enable path from ROB/head output-valid
control through the A-write word decode to a gate latch D pin. Its arrival is
2.5722 ns against a 5 ns requirement, leaving +2.4278 ns slack. Separate
enable-path hold timing also passes.

The library assigns zero area to its generic `LATCH`, so the reported mapped
area delta has no credible physical interpretation and is not used as a result.
Clock-network power also cannot be compared directly because the baseline
source clock is ideal and is classified differently from the inserted discrete
gate network.

This is **SAIF-driven pre-layout power analysis**. It is not production ICG,
CTS, post-layout, PrimeTime PX, physical-implementation, or signoff power.
Mapped GLS remains inconclusive because the OSU cell model leaves some
synchronous-reset state unknown; the independently compiled baseline mapped
design reproduces the same behavior. GLS and LEC are not claimed as passing.

## Original compatibility flow

`run_dc.sh` and `dc_rob.tcl` are retained so earlier commands still work. This
flow writes shared `syn/logs/`, `syn/reports/`, `syn/mapped/`, and `syn/work/`
directories and elaborates the RTL default parameter value. Use the canonical
baseline flow for comparisons because its outputs are isolated and its manifest
captures the run inputs.

The output under `runs/legacy_2026-07_default_mo4/` preserves the original
shared logs, reports, mapped files, WORK library, and DC root artifacts moved
out of the directory root during repository cleanup. It is historical reference
data rather than a canonical run with a manifest and input snapshot.

## Interpretation limits

This is a relative synthesis baseline. The SDC has an ideal clock and does not
set clock uncertainty, input drive cells, output loads, or a wire-load model.
Consequently, `Total cell area` is useful for comparisons, while physical total
area is reported as undefined. Treat these results as synthesis guidance rather
than signoff timing or post-layout estimates.
