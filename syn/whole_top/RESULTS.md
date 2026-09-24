# Whole-top DC / PrimeTime results

Observed on 2026-09-24 using `mac_accel_dma_rob_top`, MAX_OUTSTANDING=8,
DATA_WIDTH=16, AXI_DATA_WIDTH=32, AXI_ADDR_WIDTH=32, AXI_ID_WIDTH=4,
S_AXI_ADDR_WIDTH=8, MAX_LEN=256 and BURST_LEN=16. This is separate from the
canonical `mac_dma_rob` block timing/power benchmark.

DC R-2020.09-SP4 used `compile_ultra -no_autoungroup`. PrimeTime
R-2020.09-SP5-1 independently read the mapped Verilog and resolved SDC.
Read, link, SDC load and timing update passed. The library was
`osu018_stdcells.db` (TSMC 0.18 um), typical/process 1/1.8 V/25 C, same corner
for min and max. Bus/MAC clocks were asynchronous and ideal, 10/7.5 ns;
functional bus I/O delays were min 0/max 1 ns, max fanout 32.

## Timing

All values below are ns. Worst positive slack is reported separately from
violation-only WNS, which is zero for every group; TNS is also zero.

| Domain/check | DC worst slack | PT worst slack | PT arrival | PT required | Violating endpoints |
|---|---:|---:|---:|---:|---:|
| Bus setup | +5.1322 | +5.114273 | 4.698183 | 9.812455 | 0 |
| MAC setup | +0.0207 | +0.020727 | 7.381683 | 7.402410 | 0 |
| Bus hold | +0.0522 | +0.049906 | 0.054068 | 0.004162 | 0 |
| MAC hold | +0.2147 | +0.214744 | 0.223478 | 0.008734 | 0 |

| PT path | Startpoint | Endpoint |
|---|---|---|
| Bus setup | `u_dma/u_read_engine/head_ptr_reg[1]` | `u_dma/buf_a_reg[31][12]` |
| MAC setup | `u_pe/a_in_r_reg[1]` | `u_pe/mul_reg_reg[31]` |
| Bus hold | `S_AXI_RREADY` | `S_AXI_RVALID_reg` |
| MAC hold | `start_sync_mac_reg[0]` | `start_sync_mac_reg[1]` |

DC selected bus endpoint `buf_a_reg[127][12]`. Querying that same path in PT
gave arrival 4.698183, required 9.812598 and slack +5.114416. The data-path
point sequence matched DC exactly and arrival agreed within DC's four-decimal
rounding. DC's library setup requirement was 0.1696; PT's was 0.187402.
The observed slack difference is in the endpoint timing check. The exact
internal evaluation/default/slew cause was not isolated; neither tool is
declared incorrect. MAC setup and hold agree within DC report precision.
Bus hold likewise has matching arrival but a different library hold check.

## Constraint interpretation and coverage

- All 7,473 functional register `/CLK` pins had exactly one expected clock:
  7,250 bus and 223 MAC. No unresolved references were found.
- Eight CDC groups, 39 bus-to-MAC and 71 MAC-to-bus endpoints, and all 13
  two-flop chains passed mapped connectivity checks. Bounded crossing max paths
  remained timed, intentional cross-domain hold exclusions took effect, and
  synchronous stage-1-to-stage-2 timing remained active.
- `check_timing` reported 316 no-clock pins. Each was an inactive DFFSR preset
  `/S` tied to logic 1, not an unclocked functional `/CLK` pin.
- Coverage: 32,703 checks total, 30,139 met, 0 violated, **2,564 untested**.
  Untested reasons were constant_disabled 1,580; no_startpoint_clock 632;
  no_clock 316; no_paths 20; false_paths 16. Reset/preset checks, ten constant
  outputs and intentional first-stage CDC exceptions account for these items.
- All 1,906 disabled entries were propagated constants: 1,896 preset-related
  arcs plus ten output ports. No user-disabled or loop-breaking entries were
  found. Reset release/recovery/removal coverage remains outside this profile.

PrimeTime emitted startup **Error PT-063** for an unset Library Compiler path.
Direct `.db` load, link, timing update and reports worked. The runner retains
that exact known diagnostic and fails on other `Error:` messages. It does not
silently claim an error-free log. PTE-016's internal 30 ns common analysis cycle
did not change the two clock periods. The exported diagnostic SDC's WSCR-002
warning is retained; that export is not substituted for the accepted input.

## Provenance and interpretation limits

Source baseline HEAD: `17b4e36484482289e716c3761396514bdaca1f84`.
No RTL or existing constraints changed during whole-top STA.

| Accepted input | SHA-256 |
|---|---|
| `mac_accel_dma_rob_top_mapped.v` | `f1f501125c1760699349964ff6a51c42825cf63cae6d4a3694177af0b2ec053c` |
| `mac_accel_dma_rob_top.sdc` | `091a6510813b135b62899b29c7fad18188cde50f31ac404e49be620c126c85b4` |

DC and PT library bytes matched. Raw logs, mapped artifacts, manifests and
full comparison reports are retained privately; this repository publishes the
authored scripts and result summary without vendor library files. New runs
record their own hashes and may differ with tool/library versions.

The initial DC compile/export completed but reporting helpers failed. Corrected
independent DC readback verified the same saved pair without a second compile;
the original failed status remains in the historical evidence. PT query-helper
bring-up failures were likewise retained before the final successful run.

No extracted RC, CTS, drive/load model or clock uncertainty was used. PT reports
`on_chip_variation` analysis type but both sides use the same typical corner,
without configured early/late factors. This is not multi-corner OCV signoff.
The MAC's **20.727 ps** pre-layout setup margin is too small to infer physical
closure. Digital CDC simulation and structural checks do not establish analog
metastability robustness, physical skew, reset safety, LEC or mapped GLS.

For regeneration, follow [DC_FLOW.md](DC_FLOW.md), [PT_FLOW.md](PT_FLOW.md), and
the [CDC regression instructions](../../sim/CDC_REGRESSION.md).
