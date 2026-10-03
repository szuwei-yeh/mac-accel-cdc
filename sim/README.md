# Simulation map

Start with `tb_mac_accel_dma_rob_top.v` for V3 functional behavior or
`run_cdc_regression.py` for the V3 CDC scenario matrix. Tests of a DMA block or
read engine alone do not exercise the bus-to-MAC clock crossing.

Older Step 0/1/2 comments refer to development milestones, not the V1/V2/V3
accelerator architectures. Testbench names identify the DUT or the stimulus.

| Testbench / runner | DUT | Purpose |
|---|---|---|
| `tb_mac_accel_cdc.v` | `mac_accel` | CPU-fed core: 8 basic dual-clock cases, reset before each case |
| `tb_mac_accel_dma.v` | `mac_accel_dma_top` | V2 single-outstanding full-top functional regression |
| `tb_mac_accel_dma_bp.v` | `mac_accel_dma_top` | V2 slow-MAC backpressure stress |
| `tb_mac_dma_baseline.v` | `mac_accel_dma_top` | V2 latency/utilization baseline measurement |
| `tb_axi_read_engine_rob.v` | `axi_read_engine_rob` | Current unique-ID engine with in-order memory stimulus |
| `tb_axi_read_engine_rob_ooo.v` | `axi_read_engine_rob` | OOO/interleaved responses, scoreboards and protocol checks |
| `tb_mac_dma_rob.v` | `mac_dma_rob` | V3 DMA block and A/B pairing regression |
| `tb_mac_dma_rob_sweep.v` | `mac_dma_rob` | Outstanding-depth / memory-latency measurement |
| `tb_mac_dma_rob_power.v` | `mac_dma_rob` | Deterministic active/idle workload for SAIF power analysis |
| `tb_mac_accel_dma_rob_top.v` | `mac_accel_dma_rob_top` | V3 full-top functional regression |
| `tb_mac_accel_dma_rob_cdc.sv` | `mac_accel_dma_rob_top` | V3 payload/event/Gray checks over six jobs without inter-job reset |
| `run_cdc_regression.py` | Same V3 CDC testbench | Six clock/phase/reset-release scenarios on Icarus or VCS |

The CPU-fed core test was renamed from `tb_mac_accel_cdc_v3.v` to
**`tb_mac_accel_cdc.v`**, including its top module. Its old suffix described a
test revision, not the V3 DMA/ROB architecture. Update local source lists and
`-s`/`-top` arguments that used the old name.

## Memory models

These are behavioral test infrastructure, not synthesizable RTL.

| Model | Requests accepted | Response scheduling |
|---|---|---|
| `axi_read_mem_model.v` | One outstanding burst | Complete one burst before accepting another |
| `axi_read_mem_model_mo.v` | Multiple outstanding bursts | Complete bursts in allocation order; echo each request's ID |
| `axi_read_mem_model_ooo.v` | Multiple unique-ID bursts | Ready-slot scheduling; optional beat interleaving across IDs |

`mo` means **multi-outstanding**, not out-of-order. The `_ooo` model can reorder
completion even with `ooo_en=0`; that setting disables beat interleaving and
services a ready burst to completion. V2 functional tests also contain local
memory models in their testbench files.

## Basic CPU-fed core regression

Run from the repository root:

```bash
iverilog -g2012 -s tb_mac_accel_cdc -o /tmp/mac_accel_cdc \
  sim/tb_mac_accel_cdc.v rtl/mac_accel.v rtl/mac_fifo_async.v rtl/mac_pe.v
vvp /tmp/mac_accel_cdc
```

## In-order stimulus for the current ROB engine

```bash
iverilog -g2012 -DENG_MAX_OUT=8 -s tb_axi_read_engine_rob \
  -o /tmp/axi_read_engine_inorder sim/tb_axi_read_engine_rob.v \
  rtl/axi_read_engine_rob.v sim/axi_read_mem_model_mo.v
vvp /tmp/axi_read_engine_inorder
```

This does not reconstruct the historical shared-ID engine; it exercises the
current ROB RTL with an in-order response model.

## V3 CDC matrix

```bash
python3 sim/run_cdc_regression.py --simulator iverilog
```

See [CDC_REGRESSION.md](CDC_REGRESSION.md) for scenarios, recorded results and
coverage limits.

## Regression evidence

Recorded simulation results:

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

Supplemental CDC-matrix results and coverage are documented separately in
[CDC_REGRESSION.md](CDC_REGRESSION.md). The power workload and its active/idle
measurement windows are described in the [power flow](../syn/README.md#deterministic-activity-source).

## Utilization measurement

Simulated MAC-feed utilization is accepted `{last,a,b}` pairs at the DMA output
divided by bus-clock cycles with `dma_busy` asserted. This measures operand
delivery, not MAC PE activity or whole-accelerator speedup.

The published comparison uses 256 elements per vector, 16-beat bursts and
100-cycle first-beat memory latency:

| Design | Outstanding bursts | Accepted pairs | DMA-busy bus cycles | Feed utilization |
|---|---:|---:|---:|---:|
| V2 | 1 | 256 | 3777 | 6.78% |
| V3 | 8 | 256 | 941 | 27.21% |

The approximately 4.0× utilization increase comes from overlapping memory
requests while preserving in-order operand delivery. The
[V2 baseline](tb_mac_dma_baseline.v) and
[V3 depth sweep](tb_mac_dma_rob_sweep.v), compiled with `DMA_ROB_MAX_OUT=8`, contain
the measurement cases. The utilization workload's 100-cycle latency is separate
from the 40-cycle latency used by the SAIF power workload.

## Depth-8 ROB and DMA regressions

Run from the repository root with Icarus Verilog:

```bash
# Standalone ROB with OOO/interleaved responses
iverilog -g2012 -DENG_MAX_OUT=8 -s tb_axi_read_engine_rob_ooo \
  -o /tmp/axi_read_engine_ooo sim/tb_axi_read_engine_rob_ooo.v \
  rtl/axi_read_engine_rob.v sim/axi_read_mem_model_ooo.v
vvp /tmp/axi_read_engine_ooo

# ROB DMA wrapper and operand pairing
iverilog -g2012 -DDMA_ROB_MAX_OUT=8 -s tb_mac_dma_rob \
  -o /tmp/mac_dma_rob sim/tb_mac_dma_rob.v rtl/mac_dma_rob.v \
  rtl/axi_read_engine_rob.v sim/axi_read_mem_model_ooo.v
vvp /tmp/mac_dma_rob
```

The complete V3 accelerator command is the
[root README quick start](../readme.md#quick-start).
