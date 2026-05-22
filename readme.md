# Vector MAC Accelerator with AXI4-Lite + AXI4 Master DMA

A parameterized, pipelined **dot-product MAC accelerator** implemented in Verilog, featuring a fully CDC-safe architecture with an Asynchronous FIFO and **two top-level integrations**:

- **V1 — `mac_accel_axi`**: AXI4-Lite slave with MMIO data path (CPU writes every element). Validated end-to-end on a Digilent Nexys A7-100T FPGA with MicroBlaze.
- **V2 — `mac_accel_dma_top`**: AXI4-Lite slave (control) **+ AXI4 master DMA** (data). CPU writes only `SRC_A_ADDR / SRC_B_ADDR / LENGTH` and kicks `CTRL`; the accelerator burst-reads its operands directly from system memory. Verified in simulation against a behavioural AXI memory model.

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
│   ├── mac_pe.v              # 2-stage pipelined signed MAC core (shared)
│   └── mac_fifo_async.v      # Dual-clock async FIFO, Gray-code (shared)
├── sim/
│   ├── tb_mac_accel_cdc_v3.v # V1: true dual-clock CDC testbench (8 cases)
│   └── tb_mac_accel_dma.v    # V2: DMA + AXI memory model testbench (7 cases)
└── sim_0/                     # Earlier testbench iterations (reference only)
    ├── tb_mac_accel_cdc.v
    ├── tb_mac_accel_cdc_v2.v
    ├── tb_mac_accel_only.v
    └── tb_mac_pe.v
```