# Vector MAC Accelerator

A signed vector dot-product accelerator built to study AXI operand delivery,
out-of-order memory responses and clock-domain crossing. The project progresses
from a CPU-fed FPGA design to a multi-outstanding DMA accelerator with a complete
Nangate45 RTL-to-GDS research implementation.

| Version | Top module | Operand delivery and demonstrated scope |
|---|---|---|
| V1 — CPU-fed | `mac_accel_axi` | AXI4-Lite operand writes; historical Nexys A7-100T / MicroBlaze demonstration |
| V2 — DMA | `mac_accel_dma_top` | Single-outstanding AXI reads; behavioral memory models |
| V3 — ROB DMA | `mac_accel_dma_rob_top` | OOO/interleaved receive, ordered retirement; frozen RTL and new UCSB routed implementation |

## V3 architecture

![V3 bus-domain AXI-Lite control, DMA/ROB and operand pairing feed an async FIFO and the independent MAC domain.](docs/figures/architecture.svg)

AXI4-Lite programs the two source addresses and vector length. DMA first loads
vector A into `buf_a`, then reads B and pairs each beat with stored A. The
`{last,a,b}` stream crosses an asynchronous FIFO into the signed MAC pipeline;
completion returns the result and MAC-cycle latency to the control registers.
Storage arrays map to flip-flops; the implementation has no SRAM macro.

**Ordered retirement.** Each accepted AR request owns a circular ROB entry,
using `ARID=alloc_ptr`. R beats select storage by `RID`, so later bursts may
complete first and responses may interleave across IDs. Only a completed
`head_ptr` entry retires. Its credit returns after the **last output beat drains**,
preventing ID reuse while stalled output still owns the old entry. Per-ID beat
order follows the AXI contract; a completed later entry waits behind the head.

![Illustrative ROB completion order 1, 2, 0 is restored to output order 0, 1, 2.](docs/figures/rob_reordering.svg)

*Protocol illustration with two-beat bursts, not a recorded waveform.*

**CDC and backpressure.** FIFO full propagates stalls through operand pairing
and ordered retirement. Gray pointers cross through two-flop synchronizers;
FIFO storage carries payload under the pointer/handshake protocol. Start and
done use toggle events; busy uses level synchronization. Result/latency stay
stable until synchronized done captures them. Physical checks retain 13
synchronizer chains and audit 602 bounded crossing pairs plus three first-stage
event exceptions. These are digital protocol/structure/timing checks.

All AXI-Lite wrappers honor byte strobes and hold responses under stalls. V3
rejects invalid lengths and busy starts, snapshots each admitted job's config,
and retains DMA errors through result capture. CTRL DONE acknowledges final
MAC consumption/result capture; standalone DMA completion can precede acceptance
of its final pending stream beat. See the [supported interface contract](docs/RTL_SUPPORTED_CONTRACT.md).

## Verification

The frozen integrated candidate has **243/243 RTL cases across 27 testbenches**,
**108/108 multi-seed stress jobs**, **36/36 V3 CDC jobs**, and **31/31 closure-era
unit tests** passing. Counts are separate and overlap; they are not added into a
single unique-case total. Directed coverage includes split AW/W arrival, B/R
stalls, partial/zero WSTRB, consecutive jobs, final-beat errors, ID ownership,
non-head completion and pending final-output recovery.

| Layer | Evidence and boundary |
|---|---|
| RTL and stress | Independent signed reference sums, transaction/data-order checks, timeouts; seeds 1/19/2345 and MAC periods 7.5/30/3.5 ns |
| Dual-clock CDC | Six scenarios × six jobs, with payload/event/Gray-pointer checks and startup reset-release variations |
| Reduced formal | Seven source/config-bound tasks reused; scoped control properties under legal AXI assumptions, not full-top equivalence |
| Final mapped GLS | 7 top-level cases + 2 DMA error-recovery jobs; exact final routed netlist binding, zero-delay cell models, no SDF |
| Physical checks | Dual-engine extracted STA, mapped/clock/CDC identity, constraint coverage, router/PG connectivity and supported GDS rules |

[Current evidence summary](docs/INTEGRATION_RESULTS.md) ·
[Simulation map](sim/README.md) · [Formal scope](formal/STUDY_GUIDE.md)

## Latest post-route implementation

The **2026-10-12 frozen candidate** was implemented on UCSB with OpenROAD/ORFS
and Nangate45, from the newly qualified checkpoint through floorplan, placement,
CTS, detailed routing, OpenRCX extraction and GDS. The qualified profile is
DATA16, AXI data/address32, ID4, MAX_LEN256, BURST_LEN16, outstanding8;
bus/MAC clocks are **10 / 7.5 ns**. V3 defaults to depth4; these results explicitly
select depth8.

<p align="center"><img src="docs/figures/routed-20261012-overview.png" width="560" alt="New accepted routed GDS: complete 434.48 by 434.48 micrometre die, selected M2-M10 metal layers."></p>

*Actual final GDS, selected M2–M10 metal across the full die. Lower layers are
omitted for readability. [Routed-metal detail](docs/figures/routed-20261012-routing-detail.png).*

| Full V3 — typical 1.1 V / 25 °C | Final measured result |
|---|---:|
| Standard cells / FFs | **29,002 / 7,509**, excluding taps/fillers from the cell count |
| Standard-cell / core area | **69,034.714 / 174,468.868 µm²** |
| Die / standard-cell utilization | **434.48 × 434.48 µm / 39.5685%** |
| Bus setup / hold WNS | **+7.551700 ns / +74.356 ps** |
| MAC setup / hold WNS | **+5.978773 ns / +82.133 ps** |
| Setup/hold TNS / violating endpoints | **0 ns / 0**, both domains |
| Slew / fanout / capacitance violations | **0 / 0 / 0** |
| Router DRC / supported KLayout DRC / operative GDS antenna | **0 / 0 / 0**; 133 supported rule categories |

Timing uses **propagated clocks and routed OpenRCX SPEF**, with the original
I/O/CDC contract and zero clock uncertainty. Clock-input antenna segmentation
and data hold-buffer repair closed the recorded failures without changing RTL
or relaxing constraints. WNS is not a frequency limit.

[Latest results and reproduction](docs/ROUTED_RESULTS.md) ·
[Revision/source/artifact binding](evidence/routed-20261012/binding.json)

## Historical optimization studies

The following measurements belong to archived experiments, not the new routed
candidate. They remain useful evidence of design choices with different scopes.

| Study | Recorded result | Measurement scope |
|---|---|---|
| Overlap memory requests | 6.78% → 27.21% MAC-feed utilization, 4.0× | Historical V2/V3 simulation: 256 pairs, 16-beat bursts, 100-cycle first-beat latency; accepted pairs / DMA-busy bus cycles |
| Register AR address arithmetic | 5.50 → 4.05 ns, −26.36% | Synopsys/OSU DMA-block global synthesized QoR path; ideal clocks, pre-layout |
| Gate A-buffer clocks | 76.1114 → 35.3225 mW active dynamic, −53.59% | SAIF-driven Synopsys/OSU DMA-block estimate; 4,096 buf_a bits, 40-cycle latency workload |

![Historical simulation comparison of accepted operand pairs per DMA-busy bus cycle.](docs/figures/mac_feed_utilization.svg)

These are not whole-system speedup, matched-process PPA or routed Nangate45
power claims. The OSU generic zero-area latch prevents a credible gating-area
comparison. [Synthesis/power methodology](syn/README.md) and
[historical Nangate45 results](physical/RESULTS.md) retain their original scope
and identities; [figure provenance](docs/FIGURES.md) distinguishes both revisions.

## Run and reproduce

<a id="quick-start"></a>

With Icarus Verilog installed, run the full V3 integration test from the repo root:

```bash
iverilog -g2012 -DV3_MAX_OUT=8 -s tb_mac_accel_dma_rob_top \
  -o /tmp/mac_accel_dma_rob_top \
  sim/tb_mac_accel_dma_rob_top.v rtl/mac_accel_dma_rob_top.v \
  rtl/mac_dma_rob.v rtl/axi_read_engine_rob.v rtl/mac_pe.v \
  rtl/mac_fifo_async.v sim/axi_read_mem_model_ooo.v
vvp /tmp/mac_accel_dma_rob_top
```

Expected: **7 PASS / 0 FAIL**. Permanent regressions and document checks:

```bash
python3 scripts/run_ci_rtl.py
python3 scripts/run_rtl_closure_stress.py
python3 sim/run_cdc_regression.py --simulator iverilog
python3 scripts/check_publication.py
```

[CI instructions](docs/CI.md) cover dependencies and unit checks. Hosted CI
links there certify only their listed published commits; this uncommitted
candidate has no new hosted CI result. [Physical reproduction](docs/ROUTED_RESULTS.md#reproduction)
uses the externally archived package and pinned tools. Large netlists, ODB/GDS,
SPEF and raw logs stay outside the publication tree; exact hashes are committed
as reviewable records.

## Supported scope and limits

DMA sources must be four-byte aligned and each burst must stay within its 4 KB
page; the master does not split bursts. The published profile supports lengths
1–256. Arbitrary parameter combinations and MAX_BURST_LEN=1 are not qualified.
Reset requires coordinated initialization and running clocks; single-domain
reset/recovery is unsupported. See the [full contract](docs/RTL_SUPPORTED_CONTRACT.md)
and [parameter scope](docs/RTL_PARAMETER_SCOPE.md).

The physical result is **single-corner research evidence**. GLS is zero-delay;
native LEF antenna is **inconclusive**, and qualified **LVS is unsupported**.
Multi-corner/OCV, foundry signoff, full-top LEC, SDF, CDC MTBF/RDC, dynamic IR/EM
and signoff power remain outside the demonstrated scope. A fresh portable
end-to-end RTL-to-GDS replay has not been validated.
