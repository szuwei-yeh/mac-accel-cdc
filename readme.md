# Vector MAC Accelerator with CDC and Multi-Outstanding AXI Read DMA

A parameterized dot-product accelerator written in Verilog. The project evolves
from a CPU-fed AXI4-Lite peripheral into a dual-clock accelerator whose DMA can
issue multiple AXI read bursts, accept out-of-order/interleaved responses, and
retire data in order through a RID-indexed reorder buffer.

The canonical synthesis and power-analysis target is:

```text
TOP               = mac_dma_rob
MAX_OUTSTANDING   = 8
Bus clock target  = 10 ns / 100 MHz
DC                 = R-2020.09-SP4
Library            = osu018_stdcells.db
Compile            = compile_ultra
```

V1 was also validated end to end on a Digilent Nexys A7-100T with MicroBlaze.
V2 and V3 use behavioral AXI memory models for simulation and formal models for
control-safety properties.

## Results at a glance

| Area | Result |
|---|---|
| Multi-outstanding DMA | Up to 8 outstanding bursts validated at standalone ROB, DMA-wrapper, and V3 full-top levels |
| Reordering | Unique ARIDs, RID-indexed response storage, strictly in-order retirement |
| CDC | Async FIFO with Gray-code pointers plus toggle and 2-FF synchronizers |
| Timing optimization | Critical-path length reduced from about 5.50 ns to 4.05 ns, a 26.36% improvement |
| Canonical 100 MHz timing | Setup WNS +4.7923 ns |
| Clock sweep | 100, 125, 166.7, and 200 MHz all pass pre-layout DC setup timing |
| 200 MHz edge | WNS +0.0389 ns; functional limiter is the flat ROB read/capture path |
| Targeted power experiment | 4096 `buf_a` bits gated in 256 16-bit banks |
| SAIF-driven active power | Dynamic and total power both reduced by 53.59% |
| SAIF-driven idle power | Dynamic power reduced by 57.22% |
| Measured-job energy | Reduced by 53.59% |

The 200 MHz result uses a pre-layout Design Compiler model with ideal clocks and
no extracted parasitics. It is not a post-layout frequency claim. The project
target remains 100 MHz.

The power result is **SAIF-driven pre-layout power analysis**. Clock gating is
**synthesis-inserted latch-based clock gating using discrete LATCH + AND
cells**. The OSU library has no dedicated production ICG cell.

## Engineering evolution

| Stage | Engineering change | Evidence |
|---|---|---|
| 1. Original Vector MAC | Two-stage signed multiply/accumulate pipeline behind AXI4-Lite | Simulation and FPGA validation |
| 2. CDC architecture | Separate bus/MAC clocks, async FIFO, toggle synchronizers, stable multi-bit capture | Dual-clock regression and FIFO formal properties |
| 3. AXI read DMA | Operand vectors fetched through AXI INCR bursts | DMA regression with a reference memory model |
| 4. Multi-outstanding reads | Credit-paced AR issue with unique IDs | Standalone engine and utilization tests |
| 5. RID-indexed ROB | Responses stored by RID and retired by allocation order | OOO/interleaved-response regressions |
| 6. Verification and formal | Scoreboards, protocol stability checks, BMC, prove, and cover | Simulation plus SymbiYosys/IC3-PDR results |
| 7. Canonical synthesis | Immutable parameterized DC runs with manifests and hashes | `mac_dma_rob`, depth 8, 10 ns |
| 8. Address timing optimization | Replaced the long combinational ARADDR expression with handshake-driven address state | 26.36% critical-path improvement |
| 9. Frequency characterization | Recompiled at 10/8/6/5 ns without RTL changes | All setup PASS; 5 ns has 38.9 ps margin |
| 10. Targeted power optimization | Gated only the large A-operand buffer through Power Compiler | Same-workload SAIF baseline/gated comparison |

## Architecture

```text
bus_clk domain                                      mac_clk domain
────────────────────────────────────────────────────────────────────────
AXI4-Lite CSR                                            ┌─────────────┐
     │                                                   │   mac_pe    │
     ▼                                                   │ multiply +  │
mac_accel_dma_rob_top                                    │ accumulate  │
     │                                                   └──────▲──────┘
     ├── mac_dma_rob ── AXI4 AR/R ── memory model               │
     │      │                                                   │
     │      └── axi_read_engine_rob                             │
     │             ├── multi-outstanding AR issue               │
     │             ├── RID-indexed response storage             │
     │             └── in-order retirement                      │
     │                                                           │
     └── {last,a,b} ── mac_fifo_async ───────────────────────────┘
                        Gray pointers + 2-FF synchronizers
```

### Implementations

- **V1 — `mac_accel_axi`**: AXI4-Lite control and CPU-fed operand data.
- **V2 — `mac_accel_dma_top`**: AXI4-Lite control plus a single-outstanding
  AXI read DMA.
- **V3 — `mac_accel_dma_rob_top`**: AXI4-Lite control plus a parameterized,
  multi-outstanding AXI read DMA and ROB.

V3 preserves the A-then-B schedule. The first read phase fills `buf_a`; the
second returns B operands and pairs them with the stored A operands before
crossing the async FIFO into the MAC clock domain.

Each accepted AR request uses `ARID=alloc_ptr`. R beats update the ROB entry
selected by `RID`. Only the completed `head_ptr` entry can drive the output,
which restores allocation order even when responses arrive out of order or are
interleaved across IDs.

### CDC strategy

| Signal | Direction | Method |
|---|---|---|
| Start event | bus to MAC | Toggle, 2-FF synchronization, edge detection |
| Operand stream | bus to MAC | Async FIFO with Gray-code pointers |
| Done event | MAC to bus | Toggle, 2-FF synchronization, edge detection |
| Busy level | MAC to bus | 2-FF synchronizer |
| Result/latency | MAC to bus | Stable bus captured when synchronized done arrives |

Synchronizer chains and FIFO pointer logic are intentionally excluded from the
clock-gating experiment.

## Verification evidence

Final simulation results:

| Regression | Configuration | Result |
|---|---|---:|
| Original dual-clock MAC/CDC | Non-integer clock ratio | 8 PASS / 0 FAIL |
| Single-outstanding DMA | Behavioral AXI memory | 7 PASS / 0 FAIL |
| Standalone ROB | `MAX_OUTSTANDING=8` | 12 PASS / 0 FAIL |
| ROB DMA wrapper | `MAX_OUTSTANDING=8` | 11 PASS / 0 FAIL |
| V3 full top | `MAX_OUTSTANDING=8` | 7 PASS / 0 FAIL |
| Deterministic power workload | `MAX_OUTSTANDING=8` | 4 PASS / 0 FAIL |

The scoreboards and reference models check:

- accepted AR address sequence, ARLEN geometry, and request/beat conservation;
- AR payload stability under backpressure;
- no duplicate, skipped, or over-issued data;
- RID-indexed response placement and in-order retirement;
- output payload/last stability while stalled;
- OOO and interleaved RID responses;
- lengths 1 and 16 plus multi-burst/cross-burst transfers;
- reset, idle wake-up, consecutive jobs, and FIFO/stream backpressure.

### Formal verification

| Property set | Main properties | Result |
|---|---|---|
| `mac_dma_bp` | DMA no-drop behavior and AR-channel stability under adversarial backpressure | BMC + prove PASS |
| `mac_fifo_gray` | Gray pointer transitions, no overflow, no underflow | BMC + prove PASS |
| `axi_read_engine_rob` | No overflow/tag reuse, legal retirement, in-order output, burst/address/count model, sticky errors | BMC + IC3/PDR prove PASS; cover PASS |

The ROB proof uses a small bounded parameter configuration and a legal AXI read
slave model. Readiness and error behavior remain adversarial within the model's
protocol assumptions. See [`formal/STUDY_GUIDE.md`](formal/STUDY_GUIDE.md).

Mapped gate-level simulation is inconclusive because the OSU Verilog cell model
leaves some synchronous-reset state unknown. The independently compiled
baseline mapped design reproduces the same behavior, so it has not been
identified as a clock-gating-specific failure. GLS, LEC, CTS, and signoff are
not claimed as passing evidence.

## Timing optimization and frequency characterization

The original depth-8 critical path computed ARADDR directly from
`cmd_addr_r + (issue_elem << 2)`. The optimized RTL keeps the accepted-request
address in `current_ar_addr_r`, drives ARADDR directly from that register, and
updates it only after an AR handshake.

| Metric | Original depth-8 | Optimized depth-8 | Change |
|---|---:|---:|---:|
| QoR critical-path length | 5.50 ns | 4.05 ns | -26.36% |
| Setup WNS at 10 ns | +3.5027 ns | +4.7923 ns | +1.2896 ns |
| Logic levels | 31 | 11 | -20 |

ARADDR no longer appears in the optimized global top ten. At 10 ns, the global
worst path is synchronous reset/control logic into `retire_elem`; the worst
active functional path is the ROB read and wrapper-capture cone.

No-RTL-change clock sweep:

| Period | Frequency | Setup | WNS | Critical length | Main path class |
|---:|---:|---|---:|---:|---|
| 10 ns | 100 MHz | PASS | +4.7923 ns | 4.05 ns | reset/control |
| 8 ns | 125 MHz | PASS | +1.3094 ns | 6.52 ns | address-state update mapping |
| 6 ns | 166.7 MHz | PASS | +1.1860 ns | 4.64 ns | ROB read/data selection |
| 5 ns | 200 MHz | PASS | +0.0389 ns | 4.80 ns | ROB read/data selection |

At 200 MHz the functional limiter is:

```text
head_ptr
  -> flat ROB read mux
  -> engine_out_data
  -> wrapper buf_a/stream_b capture
```

The 38.9 ps margin is useful frequency characterization, not robust physical
margin. The project therefore keeps 100 MHz as its canonical target and stops
further timing optimization.

## Targeted `buf_a` power optimization

Power Compiler is restricted to `buf_a_reg[*]`. Every other register is
explicitly excluded before `compile_ultra -gate_clock`.

```text
Gated storage              4096 register bits
Organization               256 x 16-bit word banks
Inserted gate banks        256
Implementation per bank    generic LATCH + AND2X1 (+ clock inverter)
Other gated registers      0
Gated setup WNS @ 10 ns    +2.4278 ns
```

The deterministic workload uses fixed 40-cycle memory latency, fixed OOO
selection state, deterministic AR/stream backpressure, one measured length-64
job, a 512-cycle idle interval, and follow-up length 1/16/33 jobs. VCS records
unpacked memory activity through VPD; `vpd2vcd +includemda` preserves all
`buf_a` and ROB words before `vcd2saif` creates separate active and idle SAIF
windows.

Both mapped designs consume the same two SAIF files. Ports, all 4232
RTL-invariant sequential objects, and all 4096 `buf_a` bits are user annotated.

| SAIF-driven pre-layout metric | Baseline | Gated | Change |
|---|---:|---:|---:|
| Active dynamic power | 76.1114 mW | 35.3225 mW | -53.59% |
| Active total power | 76.1134 mW | 35.3242 mW | -53.59% |
| Idle dynamic power | 71.6315 mW | 30.6423 mW | -57.22% |
| Energy per measured job | 221.870 nJ | 102.970 nJ | -53.59% |

The library reports zero area for its generic `LATCH`, so the mapped area delta
has no credible physical interpretation and is intentionally omitted as a
headline result.

## Reproducing the results

### RTL regressions with Icarus Verilog

```bash
# Standalone ROB, depth 8
iverilog -g2012 -DENG_MAX_OUT=8 -o sim_ooo \
  sim/tb_axi_read_engine_rob_ooo.v rtl/axi_read_engine_rob.v \
  sim/axi_read_mem_model_ooo.v
vvp sim_ooo

# ROB DMA wrapper, depth 8
iverilog -g2012 -DDMA_ROB_MAX_OUT=8 -o sim_dma_rob \
  sim/tb_mac_dma_rob.v rtl/mac_dma_rob.v rtl/axi_read_engine_rob.v \
  sim/axi_read_mem_model_ooo.v
vvp sim_dma_rob

# V3 full top, depth 8
iverilog -g2012 -DV3_MAX_OUT=8 -o sim_dma_rob_top \
  sim/tb_mac_accel_dma_rob_top.v rtl/mac_accel_dma_rob_top.v \
  rtl/mac_dma_rob.v rtl/axi_read_engine_rob.v rtl/mac_pe.v \
  rtl/mac_fifo_async.v sim/axi_read_mem_model_ooo.v
vvp sim_dma_rob_top
```

### Formal

With OSS CAD Suite on `PATH`:

```bash
sby -f formal/mac_dma_bp.sby
sby -f formal/mac_fifo_gray.sby
sby -f formal/axi_read_engine_rob.sby
```

### Canonical synthesis

On the UCSB ECE Synopsys environment:

```bash
MAX_OUTSTANDING=8 CLK_PERIOD=10.0 \
RUN_ID=mac_dma_rob_mo8_10ns_final \
bash syn/run_baseline_dc.sh
```

### Activity and targeted clock-gating comparison

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

Each launcher refuses to overwrite an existing run directory and stores input
snapshots, hashes, logs, mapped outputs, timing/area reports, clock-gating
reports, SAIF coverage, and power reports under `syn/runs/`.

## Register map

| Offset | Register | Access | Description |
|---:|---|---|---|
| `0x00` | CTRL | R/W | Start; busy, done, and DMA-error status |
| `0x04` | SRC_A_ADDR | W | Vector A byte address |
| `0x08` | SRC_B_ADDR | W | Vector B byte address |
| `0x0C` | LENGTH | W | Elements per vector, 1 through `MAX_LEN` |
| `0x10` | RESULT | R | Signed dot-product result |
| `0x14` | LATENCY | R | MAC-clock cycles from start to done |

Each operand occupies the low 16 bits of one 32-bit memory word. The DMA emits
INCR bursts of up to 16 beats and chains bursts for longer vectors.

## Limitations

- AXI verification uses behavioral/formal memory models rather than a
  commercial interconnect or memory controller.
- The implemented master is the read-only AR/R subset needed by this project;
  this is not a complete commercial AXI subsystem.
- Synthesis timing is pre-layout with ideal clocks, a wire-load model, and no
  CTS or extracted parasitics.
- The OSU library has no dedicated ICG; the power experiment uses discrete
  generic latch and logic cells.
- Power results are SAIF-driven pre-layout estimates, not post-layout, CTS,
  PrimeTime PX, or signoff power.
- Mapped GLS is inconclusive because of OSU cell-model unknown-state behavior;
  the baseline netlist reproduces it.
- Formality/LEC and PrimeTime signoff were not available.
- The RTL top remains parameterized with a default outstanding depth of 4; the
  final ROB, DMA, and V3 full-top regressions explicitly select depth 8.

## Interview summary

- Built a parameterized AXI read DMA that keeps up to eight bursts in flight
  using unique IDs and credit-based issue control.
- Designed a RID-indexed circular ROB that accepts interleaved/out-of-order R
  responses while preserving strict in-order output.
- Implemented the bus-to-MAC CDC with a Gray-pointer async FIFO and toggle/2-FF
  event synchronizers.
- Removed a 31-level AR address path and improved synthesized critical-path
  length by 26.36% while preserving protocol behavior.
- Characterized 100–200 MHz targets; 200 MHz passes the pre-layout model by
  38.9 ps and exposes the flat ROB read/capture cone as the real limiter.
- Built a deterministic VCS-to-SAIF power flow and demonstrated a 53.59%
  active-dynamic reduction by gating only the 4096-bit A-operand buffer.

## Repository structure

```text
rtl/       V1/V2/V3 accelerator, DMA, ROB, FIFO, and MAC RTL
sim/       Behavioral memory models, functional regressions, power workload
formal/    SymbiYosys property sets and study guide
syn/       Canonical timing and targeted power flows
```

See [`syn/README.md`](syn/README.md) for synthesis methodology and report
interpretation.
