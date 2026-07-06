# Vector MAC Accelerator with AXI4-Lite + AXI4 Master DMA

A parameterized, pipelined **dot-product MAC accelerator** implemented in Verilog, featuring a fully CDC-safe architecture with an Asynchronous FIFO, AXI4-Lite control, AXI4 master DMA, and an experimental outstanding-transaction read path:

- **V1 — `mac_accel_axi`**: AXI4-Lite slave with MMIO data path (CPU writes every element). Validated end-to-end on a Digilent Nexys A7-100T FPGA with MicroBlaze.
- **V2 — `mac_accel_dma_top`**: AXI4-Lite slave (control) **+ AXI4 master DMA** (data). CPU writes only `SRC_A_ADDR / SRC_B_ADDR / LENGTH` and kicks `CTRL`; the accelerator burst-reads its operands directly from system memory. Verified in simulation against a behavioural AXI memory model.
- **V3 experimental — `axi_read_engine_rob` + `mac_dma_rob`**: unique-ARID AXI4 read engine with up to 4 outstanding bursts by default and a reorder buffer (ROB) that restores in-order phase output from out-of-order/interleaved R responses. Integrated at DMA level as an experimental path; the verified V2 top remains untouched.

---

## Features

- **Parameterized design** — configurable `DATA_WIDTH` and `VEC_LEN`
- **2-stage pipelined MAC** — signed multiply then accumulate, fully registered
- **Dual-clock architecture** — separate `bus_clk` (AXI/MicroBlaze) and `mac_clk` (compute core)
- **Three CDC techniques** — each signal type uses the correct crossing strategy (see below)
- **Async FIFO** — Gray-code pointer + 2-FF synchronizer for vector data transfer
- **True AXI4-Lite slave** — full AWVALID/WVALID/BVALID/ARVALID/RVALID handshake, independent AW and W channel latching
- **FPGA validated (V1)** — end-to-end verified on Nexys A7-100T at 100 MHz with MicroBlaze MMIO
- **AXI4 master DMA (V2)** — read-only burst master (INCR, up to 16 beats/burst) with multi-burst transactions, sticky `RRESP` error capture, and FIFO-paced back-pressure
- **Outstanding read engine (V3 experimental)** — credit-paced issue, unique `ARID=alloc_ptr`, circular ROB indexed by `RID`, in-order retirement, out-of-order response testbench, and integrated DMA-level utilization sweep
- **Formal verification** — async-FIFO CDC properties, DMA no-drop / AR-stability properties, and ROB safety properties proven on a bounded configuration with SymbiYosys / IC3-PDR

---

## Architecture

```
  bus_clk domain                            mac_clk domain
  ─────────────────────────────────────────────────────────────────
  ┌──────────────────┐                    ┌──────────────────────┐
  │  mac_accel_axi   │                    │                      │
  │  (AXI4-Lite      │                    │      mac_pe          │
  │   slave wrapper) │                    │  (2-stage pipeline)  │
  └────────┬─────────┘                    │  stage1: a × b       │
           │ we/re/addr/wdata             │  stage2: acc += mul  │
  ┌────────▼─────────┐  toggle+2FF+edge   │                      │
  │                  │ ──── start ──────► │                      │
  │   mac_accel      │                    └───────────┬──────────┘
  │   (CDC + control)│  Async FIFO                    │
  │                  │ ──{last,a,b}──────►            │
  │                  │                    ┌───────────▼──────────┐
  │                  │  toggle+2FF+edge   │   done / result      │
  │                  │ ◄──── done ────────│   (mac_clk domain)   │
  └──────────────────┘                    └──────────────────────┘
```

### CDC Strategy — Three Techniques

| Signal | Direction | CDC Method | Why |
|--------|-----------|------------|-----|
| `start` | bus → mac | Toggle register → 2-FF sync → edge detect | Pulse: toggling makes it level-safe across clock domains |
| `vecA`, `vecB` | bus → mac | Asynchronous FIFO (Gray-code pointers) | Multi-bit data: FIFO is the only safe method |
| `done` | mac → bus | Toggle register → 2-FF sync → XOR edge detect | Pulse: single mac_clk cycle, would be missed by direct 2-FF sync |
| `busy` | mac → bus | 2-FF synchronizer | Level signal: direct 2-FF sync is safe |
| `result`, `latency` | mac → bus | Latched on `done_pulse_bus` rising edge | Multi-bit, output-only: stable when done fires, latch in bus domain is safe |

> **Key insight:** `done_mac` is a single mac_clk cycle pulse. At 133 MHz mac_clk vs 100 MHz bus_clk, a direct 2-FF sync on a 1-cycle pulse has a high probability of being missed entirely. Using a toggle synchronizer converts the pulse into a level change, which is guaranteed to be captured regardless of clock phase.

---

## Module Hierarchy

### V1 — MMIO data path
```
mac_accel_axi          (AXI4-Lite slave wrapper)
└── mac_accel          (top-level CDC + control logic)
    ├── mac_fifo_async (dual-clock async FIFO, Gray-code pointers)
    └── mac_pe         (2-stage pipelined signed MAC)
```

### V2 — AXI4 master DMA data path
```
mac_accel_dma_top      (AXI4-Lite slave + AXI4 master + CDC + register file)
├── mac_dma            (AXI4 master DMA engine, burst INCR reads)
├── mac_fifo_async     (dual-clock async FIFO, Gray-code pointers)
└── mac_pe             (2-stage pipelined signed MAC)
```

In V2 the AXI4-Lite slave only carries a small control surface; the DMA engine fetches operands directly from memory and pushes paired `{last, a, b}` samples through the same async FIFO that V1 uses, so `mac_pe` and the FIFO are shared verbatim.

### V3 — Experimental outstanding + ROB read path
```
mac_dma_rob                  (experimental DMA wrapper; not wired into V2 top)
└── axi_read_engine_rob      (unique-ID AXI read issue + ROB + in-order output)
```

The V3 path preserves the V2 A-then-B operand schedule: one active read phase at a time. The same `axi_read_engine_rob` first fills `buf_a` with in-order A beats, then reads B with outstanding transactions and pairs the in-order B stream with buffered A before emitting `{last, a, b}`.

The ROB is a circular buffer with `MAX_OUTSTANDING` entries. Each accepted AR uses `ARID = alloc_ptr`; R beats fill entries by `RID`; only the `head_ptr` entry retires, so the output stream is strictly in allocation order even when the AXI slave returns interleaved or out-of-order responses.

---

## Register Map

### V1 — `mac_accel_axi` (MMIO data path)

| Byte Offset | Name | R/W | Description |
|-------------|------|-----|-------------|
| `0x00` | CTRL | R/W | Write Bit[0]=1 to start; Read: Bit[2]=done, Bit[1]=busy |
| `0x04` | AIN | W | Write next element of vector A (auto-increment index) |
| `0x08` | BIN | W | Write next element of vector B (auto-increment index) |
| `0x0C` | MASK | W | Bit mask to enable which vector elements to update; resets load index |
| `0x10` | RESULT | R | Signed dot-product result |
| `0x14` | LATENCY | R | Cycle count (mac_clk) from start to done |

### V2 — `mac_accel_dma_top` (DMA data path)

| Byte Offset | Name | R/W | Description |
|-------------|------|-----|-------------|
| `0x00` | CTRL | R/W | Write Bit[0]=1 to start; Read: Bit[1]=busy, Bit[2]=done, Bit[3]=dma_err |
| `0x04` | SRC_A_ADDR | W | Base address of vector A in memory (4-byte aligned) |
| `0x08` | SRC_B_ADDR | W | Base address of vector B in memory (4-byte aligned) |
| `0x0C` | LENGTH | W | Number of elements per vector (1..`MAX_LEN`) |
| `0x10` | RESULT | R | Signed dot-product result (latched on done) |
| `0x14` | LATENCY | R | Cycle count (mac_clk) from start to done |

V2 memory layout: each vector element occupies one 32-bit word (low 16 bits = signed value); the DMA issues INCR bursts of up to 16 beats, automatically chaining multiple bursts when `LENGTH > BURST_LEN`.

---

## Simulation

Uses independent non-integer-ratio clocks to stress all CDC paths including toggle synchronizers and async FIFO gray-code logic.

### Run V1 (CDC stress)

```bash
iverilog -o sim_cdc3 sim/tb_mac_accel_cdc_v3.v rtl/mac_accel.v rtl/mac_pe.v rtl/mac_fifo_async.v && vvp sim_cdc3
```

### Run V2 (DMA + behavioural AXI memory model)

```bash
iverilog -g2012 -o sim_dma sim/tb_mac_accel_dma.v rtl/mac_accel_dma_top.v rtl/mac_dma.v rtl/mac_pe.v rtl/mac_fifo_async.v && vvp sim_dma
```

### Run V3 experimental ROB read engine / DMA integration

Standalone out-of-order ROB engine test:

```bash
iverilog -g2012 -o sim_ooo sim/tb_axi_read_engine_rob_ooo.v rtl/axi_read_engine_rob.v sim/axi_read_mem_model_ooo.v && vvp sim_ooo
```

Experimental DMA wrapper integration:

```bash
iverilog -g2012 -o sim_dma_rob sim/tb_mac_dma_rob.v rtl/mac_dma_rob.v rtl/axi_read_engine_rob.v sim/axi_read_mem_model_ooo.v && vvp sim_dma_rob
```

Utilization sweep:

```bash
iverilog -g2012 -DDMA_ROB_MAX_OUT=4 -o sweep_4 sim/tb_mac_dma_rob_sweep.v rtl/mac_dma_rob.v rtl/axi_read_engine_rob.v sim/axi_read_mem_model_ooo.v && vvp sweep_4
```

The V2 testbench includes a behavioural AXI4 slave memory model (4 KB, INCR bursts) and covers:

| Test | Length | Notes |
|------|--------|-------|
| 1 | 4 | basic single-burst dot-product |
| 2 | 4 | negative operands |
| 3 | 16 | exactly one burst (boundary) |
| 4 | 20 | multi-burst, 16 + 4 |
| 5 | 33 | multi-burst, 16 + 16 + 1 |
| 6 | 1 | degenerate single-beat burst |
| 7 | 4 | back-to-back run with reset between |


Expected output (V1):

```
========================================
  mac_accel TRUE CDC Testbench
  bus_clk=100MHz  mac_clk=133MHz
========================================
---- Test 1: Basic ----
  A=[1,2,3,4] B=[10,20,30,40] expect=300
  HW RESULT=300  LATENCY=12 cycles (mac_clk)
  PASS
...
========================================
  TOTAL: 8 PASS / 0 FAIL
========================================
```

Expected output (V2):

```
========================================
  mac_accel_dma_top Testbench (V2)
  bus_clk=100MHz  mac_clk=133MHz
========================================
---- Test 1: basic length=4 ----
  length=4  src_a=0x00000000  src_b=0x00000100  expect=300
  HW RESULT=300  LATENCY=26 mac_clk cycles
  PASS
...
========================================
  TOTAL: 7 PASS / 0 FAIL
========================================
```

Expected output (V3 DMA ROB, default `MAX_OUTSTANDING=4`):

```
==================================================
  mac_dma_rob integration test
  A phase + B phase use axi_read_engine_rob, OOO memory responses
  MAX_OUTSTANDING=4 BURST_LEN=16
==================================================
----            headline len=64 L=100 ----
  PASS pairs=64 beats=128 AR=8 sum=4160
  T_active=461  U_mac=13.88 %  U_bus=27.76 %
...
==================================================
  TOTAL: 4 PASS / 0 FAIL
==================================================
```

### V3 measured utilization

Baseline V2 at `N=256`, `L=100` cycles:

| Design | Outstanding | U_mac | U_bus |
|--------|-------------|-------|-------|
| V2 single-outstanding DMA | 1 | 6.77% | 13.55% |
| V3 ROB DMA | 2 | 11.64% | 23.28% |
| V3 ROB DMA | 4 | 20.49% | 40.99% |
| V3 ROB DMA | 8 | 27.20% | 54.41% |

The conservative headline result is `MAX_OUTSTANDING=4`: integrated MAC feed utilization improves from **6.77% to 20.49%** at 100-cycle memory latency on 256-element vectors (**3.03x**).

---

## Synopsys VCS / Design Compiler

The V2 and V3 RTL were also checked on the UCSB ECE Synopsys flow. VCS
`V-2023.12-SP2` passes the original V2 DMA regression (`7 PASS / 0 FAIL`) and
the experimental ROB DMA integration regression (`4 PASS / 0 FAIL`).

Design Compiler `R-2020.09-SP4` was run with the OSU 0.18 um standard-cell
library (`osu018_stdcells.db`) at a 10 ns clock. Both synthesis targets meet
timing with zero setup/hold violating paths and no timing/design-rule constraint
violations:

| Target | Critical path | Slack @ 10 ns | Leaf cells | Cell area |
|--------|---------------|---------------|------------|-----------|
| `axi_read_engine_rob` | 5.45 ns | +3.55 ns | 9,455 | 400,254 |
| `mac_dma_rob` | 5.45 ns | +3.55 ns | 22,396 | 968,782 |

The longest reported path is the AXI AR address-generation path from
`issue_elem` through the address adder to `M_AXI_ARADDR`, which is expected for
this design.

---

## Formal Verification (SymbiYosys)

Three property sets are machine-checked with SymbiYosys. The FIFO and V2 DMA proofs use BMC + k-induction; the ROB proof uses BMC + IC3/PDR for an unbounded safety proof on a small bounded configuration.
Run with `oss-cad-suite` on `PATH`:

```bash
sby -f formal/mac_dma_bp.sby      # DMA back-pressure no-drop
sby -f formal/mac_fifo_gray.sby   # async-FIFO Gray-code invariants
sby -f formal/axi_read_engine_rob.sby
```

| Proof | Module | Property checked | Result |
|-------|--------|------------------|--------|
| `mac_dma_bp` | `mac_dma.v` | AXI-Stream VALID-stability / no-drop under adversarial back-pressure; AR-channel handshake stability | bmc + prove PASS |
| `mac_fifo_gray` | `mac_fifo_async.v` | Gray pointers change by ≤1 bit per clock (the CDC safety property); no overflow; no underflow | bmc + prove PASS |
| `axi_read_engine_rob` | `axi_read_engine_rob.v` | ROB no-overflow, no tag reuse while valid, no retire before complete, in-order retirement, output/AR stability, sticky error capture | bmc + IC3/PDR prove PASS; cover PASS |

`stream_ready` / `M_AXI_ARREADY` and the two clocks are free inputs the solver drives
adversarially in the existing proofs, so every back-pressure pattern and clock relationship is covered. The ROB proof models a legal AXI read slave with assumptions on `RID`, per-ID beat order, and `RLAST`, while leaving readiness/error signals free. See [`formal/STUDY_GUIDE.md`](formal/STUDY_GUIDE.md) for details.

> The Gray-code one-bit-change proof is the formal counterpart of the CDC design
> claim: because consecutive pointer values differ in a single bit, a pointer
> sampled mid-flight by the other domain's 2-FF synchronizer always resolves to
> the old *or* new value — never a corrupt intermediate.

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DATA_WIDTH` | 16 | Bit width of each vector element (signed) |
| `VEC_LEN` | 4 | Number of elements per vector |

---

## File Structure

```
.
├── rtl/
│   ├── mac_accel_axi.v       # V1: AXI4-Lite slave wrapper (MMIO path)
│   ├── mac_accel.v           # V1: top-level CDC + control logic
│   ├── mac_accel_dma_top.v   # V2: AXI4-Lite slave + AXI4 master DMA top
│   ├── mac_dma.v             # V2: AXI4 master DMA engine (INCR bursts)
│   ├── axi_read_engine_rob.v # V3 experimental: unique-ID read issue + ROB
│   ├── mac_dma_rob.v         # V3 experimental: A/B DMA wrapper around ROB engine
│   ├── mac_pe.v              # 2-stage pipelined signed MAC core (shared)
│   └── mac_fifo_async.v      # Dual-clock async FIFO, Gray-code (shared)
├── formal/
│   ├── axi_read_engine_rob.sby # ROB safety proof
│   ├── mac_dma_bp.sby        # DMA back-pressure no-drop + AR stability proof
│   ├── mac_fifo_gray.sby     # async-FIFO Gray-code / no over-underflow proof
│   └── STUDY_GUIDE.md        # line-by-line formal walkthrough
├── sim/
│   ├── tb_mac_accel_cdc_v3.v # V1: true dual-clock CDC testbench (8 cases)
│   ├── tb_mac_accel_dma.v    # V2: DMA + AXI memory model testbench (7 cases)
│   ├── tb_axi_read_engine_rob_ooo.v # V3: standalone ROB OOO response test
│   ├── tb_mac_dma_rob.v      # V3: experimental DMA wrapper integration test
│   ├── tb_mac_dma_rob_sweep.v # V3: utilization sweep harness
│   ├── axi_read_mem_model.v  # Step 0: single-outstanding latency model
│   ├── axi_read_mem_model_mo.v # Step 1: multi-outstanding in-order model
│   └── axi_read_mem_model_ooo.v # Step 2+: out-of-order/interleaved model
└── syn/
    ├── run_dc.sh             # Design Compiler wrapper
    ├── dc_rob.tcl            # synthesis script
    ├── constraints_rob.sdc   # portable single-clock constraints
    └── README.md             # Synopsys DC flow notes
```
