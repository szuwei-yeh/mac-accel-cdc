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
