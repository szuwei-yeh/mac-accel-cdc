# Portfolio figure sources

The README uses architecture/control diagrams, an accepted simulation comparison,
and measured physical artifacts. No layout, timing or benchmark result is
generated from an illustrative model.

| Figure | Source and scope |
|---|---|
| [Architecture](figures/architecture.svg) | V3 RTL data flow; depth 8 explicitly selected |
| [ROB reordering](figures/rob_reordering.svg) | RTL protocol schematic; illustrative two-beat bursts, not a recorded trace |
| [MAC-feed utilization](figures/mac_feed_utilization.svg) | Accepted [V2/V3 measurement table](../sim/README.md#utilization-measurement); 256 pairs / 3777 vs. 941 DMA-busy bus cycles |
| [OSU timing comparison](figures/timing_optimization.svg) | [Block synthesis metrics](../syn/README.md#first-synthesis-driven-optimization); global QoR paths, ideal clocks |
| [OSU power comparison](figures/power_comparison.svg) | [SAIF-driven block estimates](../syn/README.md#targeted-buf_a-clock-gating-experiment); no post-layout/Nangate45 power claim |
| [Final routed overview](../physical/figures/overview_dark.png) | Accepted P4 GDS; selected M3–M7 layers; [P9 copy provenance](../physical/figures/p9_provenance.json) |
| [P2–P4 timing](../physical/figures/timing-comparison.png) | Nangate45 [measured stage results](../physical/RESULTS.md#setup-and-hold); estimated/extracted RC as captioned |
| [Area/cell-count evolution](../physical/figures/area-comparison.png) | Accepted OpenDB inventories; taps/fillers excluded |
| [Hold decomposition](../physical/figures/hold-decomposition.png) | [Matched-path analysis](../physical/HOLD_ANALYSIS.md); estimated/extracted RC, changed worst endpoint |

Original SVGs, accepted PNG/PDF charts and their physical provenance are
preserved byte-for-byte. The main README uses one sized layout overview and
landscape charts; routed-metal, placement and power-grid detail stays in
[physical documentation](../physical/README.md).

## Regenerate the new utilization chart

With Python and Matplotlib 3.9.2, run from the repository root:

```bash
python3 scripts/render_mac_feed_utilization.py

# Generate into a separate directory for review instead.
python3 scripts/render_mac_feed_utilization.py --output /tmp/vector-mac-feed-preview
```

The generator reads the accepted table in `sim/README.md`, calculates
`100 × accepted pairs / DMA-busy bus cycles`, and checks agreement with its
published two-decimal percentages. It writes an SVG and
[source manifest](figures/mac_feed_utilization.json), including source/output
SHA-256s, counts, conditions and Matplotlib version. No simulation is rerun.
SVG timestamps are omitted and IDs use a fixed salt; the checked-in SVG is
reproducible with the recorded renderer version.

MAC-feed utilization measures operand delivery under a 100-cycle memory-latency
model. It is not whole-system speedup, MAC PE activity, or physical PPA. The
power experiment's separate 40-cycle workload is not used by this chart.

The original SVG generator remains
`python3 scripts/render_readme_figures.py`. It is unchanged and was not rerun
during this redesign. Accepted physical chart and layout regeneration requires
the inputs and tools documented in [REPRODUCE.md](../physical/REPRODUCE.md).
