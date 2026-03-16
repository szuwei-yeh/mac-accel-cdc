# Vector MAC Accelerator with AXI-Lite Interface

A parameterized, pipelined **dot-product MAC accelerator** implemented in Verilog, featuring a fully CDC-safe architecture with an Asynchronous FIFO, wrapped in a true AXI4-Lite slave interface, and validated end-to-end on a Digilent Nexys A7-100T FPGA with MicroBlaze via MMIO.

---

## Features

- **Parameterized design** — configurable `DATA_WIDTH` and `VEC_LEN`
- **2-stage pipelined MAC** — signed multiply then accumulate, fully registered
- **Dual-clock architecture** — separate `bus_clk` (AXI/MicroBlaze) and `mac_clk` (compute core)
- **Three CDC techniques** — each signal type uses the correct crossing strategy (see below)
- **Async FIFO** — Gray-code pointer + 2-FF synchronizer for vector data transfer
- **True AXI4-Lite slave** — full AWVALID/WVALID/BVALID/ARVALID/RVALID handshake, independent AW and W channel latching
- **FPGA validated** — end-to-end verified on Nexys A7-100T at 100 MHz with MicroBlaze MMIO

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
  ┌────────▼─────────┐  toggle+2FF+edge  │                      │
  │                  │ ──── start ──────► │                      │
  │   mac_accel      │                    └──────────┬───────────┘
  │   (CDC + control)│  Async FIFO                   │
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
| `result`, `latency` | mac → bus | Latched on `done_pulse_bus` rising edge | Data stable when done fires; latch captures safely |

> **Key insight:** `done_mac` is a single mac_clk cycle pulse. At 133 MHz mac_clk vs 100 MHz bus_clk, a direct 2-FF sync on a 1-cycle pulse has a high probability of being missed entirely. Using a toggle synchronizer converts the pulse into a level change, which is guaranteed to be captured regardless of clock phase.

---

## Module Hierarchy

```
mac_accel_axi          (AXI4-Lite slave wrapper)
└── mac_accel          (top-level CDC + control logic)
    ├── mac_fifo_async (dual-clock async FIFO, Gray-code pointers)
    └── mac_pe         (2-stage pipelined signed MAC)
```

---

## Register Map

| Byte Offset | Name | R/W | Description |
|-------------|------|-----|-------------|
| `0x00` | CTRL | W | Bit[0]: write `1` to start computation |
| `0x00` | CTRL | R | Bit[2]: `done`, Bit[1]: `busy` |
| `0x04` | AIN | W | Write next element of vector A (auto-increment index) |
| `0x08` | BIN | W | Write next element of vector B (auto-increment index) |
| `0x0C` | MASK | W | Bit mask to enable which vector elements to update; resets load index |
| `0x10` | RESULT | R | Signed dot-product result |
| `0x14` | LATENCY | R | Cycle count (mac_clk) from start to done |

---

## Simulation

Uses independent non-integer-ratio clocks to stress all CDC paths including toggle synchronizers and async FIFO gray-code logic.

### Run

```bash
iverilog -o sim_cdc3 sim/tb_mac_accel_cdc_v3.v rtl/mac_accel.v rtl/mac_pe.v rtl/mac_fifo_async.v && vvp sim_cdc3
```

Expected output:

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
│   ├── mac_accel_axi.v      # AXI4-Lite slave wrapper (full handshake)
│   ├── mac_accel.v          # Top-level: CDC logic, FIFO control, register map
│   ├── mac_pe.v             # 2-stage pipelined signed MAC core
│   └── mac_fifo_async.v     # Dual-clock async FIFO (Gray-code pointers)
└── sim/
    └── tb_mac_accel_cdc_v3.v  # True dual-clock CDC testbench (8 test cases)
```