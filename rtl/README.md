# RTL map

File names match Verilog module names. V1/V2/V3 describe accelerator
architectures, not testbench revisions or synthesis run IDs.

| Architecture | Top | Operand source | Read transactions |
|---|---|---|---|
| V1: CPU-fed | `mac_accel_axi` | CPU writes A/B through AXI4-Lite registers | No DMA |
| V2: DMA | `mac_accel_dma_top` | Read A, then B, from memory | Single outstanding burst |
| V3: ROB DMA | `mac_accel_dma_rob_top` | Read A, then B, from memory | Multiple unique-ID bursts; RID-indexed ROB |

The current complete accelerator is **`mac_accel_dma_rob_top`**. The canonical
block timing/power benchmark targets **`mac_dma_rob`**, which excludes the
AXI-Lite shell, async FIFO and MAC. These are different analysis scopes.

| File / module | Role | Instantiated by |
|---|---|---|
| `mac_accel.v` | CPU-fed dual-clock core; simplified register interface | `mac_accel_axi` |
| `mac_accel_axi.v` | V1 AXI4-Lite wrapper | V1 system integration |
| `mac_accel_dma_top.v` | V2 AXI-Lite + DMA + FIFO + MAC top | V2 system integration |
| `mac_accel_dma_rob_top.v` | V3 AXI-Lite + ROB DMA + FIFO + MAC top | V3 system integration |
| `mac_dma.v` | V2 single-outstanding read DMA and A/B pairing | `mac_accel_dma_top` |
| `mac_dma_rob.v` | V3 read-phase scheduling, A buffer and A/B pairing | `mac_accel_dma_rob_top` |
| `axi_read_engine_rob.v` | AR issue, RID-indexed response storage, in-order retirement | `mac_dma_rob` |
| `mac_fifo_async.v` | Dual-clock operand FIFO with Gray pointers | CPU-fed core and both DMA tops |
| `mac_pe.v` | Signed multiply/accumulate pipeline | CPU-fed core and both DMA tops |

All three architectures use `mac_fifo_async` and `mac_pe`. V2/V3 implement their
own control shell; they do not instantiate the CPU-fed `mac_accel` core.

The V3 RTL defaults to `MAX_OUTSTANDING=4`; published depth-8 results select
`MAX_OUTSTANDING=8` explicitly. Keep that selection in the testbench or synthesis
configuration when reproducing those results. The read engine requires
power-of-two outstanding depth (at least 2) and burst length.

DMA addresses use 4-byte beats. The DMA/read engine does not split bursts at
4 KB boundaries; callers must choose addresses and lengths so each burst stays
within a page. This is separate from ROB ID ownership and response ordering.

## V2/V3 register interface

| Offset | Register | Access | Description |
|---:|---|---|---|
| `0x00` | CTRL | R/W | Start; busy, done, and DMA-error status |
| `0x04` | SRC_A_ADDR | W | Vector A byte address |
| `0x08` | SRC_B_ADDR | W | Vector B byte address |
| `0x0C` | LENGTH | W | Elements per vector, 1 through `MAX_LEN` |
| `0x10` | RESULT | R | Signed dot-product result |
| `0x14` | LATENCY | R | MAC-clock cycles from start to done |

Each operand occupies the low 16 bits of one 32-bit memory word. The DMA emits
INCR bursts of up to 16 beats and chains bursts for longer vectors. V1 has a
CPU-fed operand interface instead; see `mac_accel_axi.v` for its register decode.

See the [simulation map](../sim/README.md), [whole-top CDC regression](../sim/CDC_REGRESSION.md)
and [block synthesis methodology](../syn/README.md).
