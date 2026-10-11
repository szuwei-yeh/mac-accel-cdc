# Verification methodology and limits

## Evidence chain

The evidence and provenance audit rehashed the accepted toolchain-qualification, pilot and full-top synthesis/physical reports/manifests/checkpoints, 63 original source files, 70 platform files and 21 installation entries. Exact final netlist/GDS/DEF/ODB/SDC/SPEF hashes are in `summary.json`; source/PDK/tool revision pins are in `pins.json`. The accepted V3 source is commit `38d6353712ac7ba680868f0729cf9006e7bff93d`.

The post-synthesis flow has an initial and a qualified-final coverage evaluator. Their hashes differ intentionally and both original snapshots are preserved. The provenance audit resolved the initial manifest to its initial archived copy, and the accepted evaluator to its qualified-final manifest. No evidence file was edited to make hashes agree.
[Historical archive identifiers](PROVENANCE.md) remain unchanged for traceability.

## Mapping and parameter checks

Read elaboration parameters independently, verify the eight ROB ownership entries and buffer/FIFO dimensions, then check mapped cells against the selected Liberty library. No unresolved generic or unintended black-box cells remain. The same 7,503 sequential identities retain bus/MAC clock assignments (7,281 / 222) through physical optimization. Yosys netlist readback reconciles OpenDB bracket escaping and traces clock roots rather than assuming all `CLKBUF` masters are clock-tree cells.

## Clock and I/O contract

Two asynchronous clocks: bus 10 ns, MAC 7.5 ns; external clock source latency 0. Functional I/O: min 0 / max 1 ns relative to bus, input transition 0.1 ns, output capacitance 5 fF, maximum fanout 32. No added jitter or clock uncertainty is included. Coverage checks account for 104 functional input bits, 93 output bits and all functional endpoints. Reset ports `S_AXI_ARESETN` and `mac_rst` are excluded; safe external reset release is assumed. Recovery/removal, RDC and analog metastability are not covered.

Asynchronous grouping uses `-allow_paths`; there is no broad bus↔MAC false path. Source→first-stage event paths for start/done/busy are specifically excluded. Every synchronizer interstage remains timed for both setup and hold.

## CDC groups

| Group | Crossing count | Maximum data delay | Hold treatment |
|---|---:|---:|---|
| Write Gray pointers | 5 | 7.5 ns, ignore clock latency | Targeted cross-domain hold exclusion |
| Read Gray pointers | 5 | 7.5 ns, ignore clock latency | Targeted cross-domain hold exclusion |
| FIFO operand payload | 528 | 7.5 ns, ignore clock latency | Targeted cross-domain hold exclusion |
| Result | 32 | 10 ns, ignore clock latency | Targeted cross-domain hold exclusion |
| Latency | 32 | 10 ns, ignore clock latency | Targeted cross-domain hold exclusion |
| Start / done / busy event stage 1 | 3 | Specific asynchronous exception | Specific exception |

This yields **602 bounded + 3 event = 605 source/destination pairs**, not 602 synchronizer chains. The **13 chains** comprise start/done/busy and five write/five read Gray bits: 26 retained two-stage registers. The evidence audit checks each direct stage-1→stage-2 fanin, clock identity, active setup/hold timing, and physical positions. Gray bit 4 may be named `*_ptr_bin[4]` because it is also the Gray MSB; the fifth bit is retained.

Final worst bounded slacks (ns): write Gray +7.357976, read Gray +7.348492, FIFO payload +7.100448, result +9.811222, latency +9.814015. Measured Gray source clock-to-Q plus data delays span 0.102561–0.108950 ns (write) and 0.110415–0.116680 ns (read), with spreads 6.389 / 6.265 ps. These are measured timing spreads, not analog MTBF or a replacement for protocol stability assumptions. Result/latency are a stable bundled bus captured after synchronized done; hold-stability depends on the RTL protocol.

## Functional and physical checks

Post-Synthesis: 66 RTL cases/jobs, seven selected formal tasks and seven mapped functional cases pass. Formal properties use scoped/reduced configurations and legal AXI assumptions; full depth-8 accelerator formal equivalence is not established. Post-CTS and exact final Post-Route functional netlists each pass the seven targeted integration cases, with the same preserved stimuli/assertions and complete qualified cell models.

The final regression is **zero-delay gate-level simulation**. It exercises integration, out-of-order ROB responses, stalls and CDC behavior; it is not SDF timing simulation or exhaustive equivalence. Standalone OpenSTA independently reads final Verilog/SDC/SPEF, matches all four domain WNS/TNS results, all 605 crossing classifications and all 26 interstage slack values. Active annotation coverage is checked against the library port graph; 8,302 raw unannotated drivers are unused, not missing functional extracted nets.

Placement legality, PG connectivity and route opens/shorts checks pass. KLayout runs all enabled FEOL/BEOL/OFFGRID/ANTENNA groups using the existing FreePDK45 deck in flat mode with two workers. Execution mode/thread count change; rule thresholds/groups do not. No marker waiver is used. The deck documents missing WELL.2 / POLY.2 / POLY.3 and reliability coverage. Native LEF antenna results are inconclusive because the antenna model is incomplete. Qualified LVS infrastructure is absent: **UNSUPPORTED**, never PASS.

## Achieved scope

This is a routed, extracted, single-corner research implementation. Foundry-certified DRC/LVS, multi-corner/OCV timing, realistic clock jitter/uncertainty, SDF regression, LEC, CDC MTBF/RDC, dynamic IR/EM, package integration and silicon validation remain unestablished. No power measurement is added for Nangate45. OSU block timing and SAIF power claims retain their original independent scope.
