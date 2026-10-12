# Figure provenance

## Current publication figures

| Figure | Identity and scope |
|---|---|
| [Architecture](figures/architecture.svg) | Existing V3 dataflow schematic, depth8 explicitly selected; not physical geometry |
| [ROB timeline](figures/rob_reordering.svg) | Illustrative completion order1,2,0 / retirement0,1,2; two-beat bursts, not measured waveform |
| [New routed overview](figures/routed-20261012-overview.png) | Final 2026-10-12 frozen candidate GDS; full 434.48 × 434.48 µm die, M2–M10 |
| [New routed detail](figures/routed-20261012-routing-detail.png) | Same final GDS; M1–M10, centered 30 × 30 µm window |
| [MAC-feed chart](figures/mac_feed_utilization.svg) | Historical accepted V2/V3 simulation counts; operand delivery, not routed power or system speedup |

[Current figure binding](../evidence/routed-20261012/binding.json) records exact
image/diagram SHA-256s and binds both new PNGs to GDS
`b7dec1a5900ff5ca5816f6a2141bbd3a3a75788b118fbb6fd681fc42cdf3fdb0`.
The PNGs are byte-identical to the final external package; the recorded KLayout
renderer is retained there as `evidence/driver/render_candidate_gds.py`.
No geometry was invented or modified. Raw previews have no timing/power labels.

All older figures, generators and their original hashes remain intact. The
following section documents **archived experiments**; its layouts and stage
charts must not be relabeled as the new frozen candidate.

## Archived figure sources

The README uses architecture/control diagrams, an accepted simulation comparison,
and measured physical artifacts. No layout, timing or benchmark result is
generated from an illustrative model.

| Figure | Source and scope |
|---|---|
| [Architecture](figures/architecture.svg) | V3 RTL data flow; depth 8 explicitly selected |
| [ROB reordering](figures/rob_reordering.svg) | RTL protocol schematic; illustrative two-beat bursts, not a recorded trace |
| [MAC-feed utilization](figures/mac_feed_utilization.svg) | Accepted [V2/V3 measurement table](../sim/README.md#utilization-measurement); 256 pairs / 3777 vs. 941 DMA-busy bus cycles |
| [OSU timing comparison](figures/timing_optimization.svg) | [Block synthesis metrics](../syn/README.md#first-synthesis-driven-optimization); global QoR paths, ideal clocks |
| [OSU power comparison](figures/power-gating-presentation.svg) | [SAIF-driven block estimates](../syn/README.md#targeted-buf_a-clock-gating-experiment); storage-entry terminology; no post-layout/Nangate45 power claim |
| [Final routed overview](../physical/figures/routed-overview-presentation.svg) | Accepted post-route GDS; selected M3–M7 layers; original geometry/axes/legend pixels preserved |
| [Routed-metal detail](../physical/figures/routing-detail-presentation.svg) | Actual M1–M5 metal and V1–V4 cuts in the accepted GDS; 30 × 30 µm window |
| [Placement and power grid](../physical/figures/placement-power-presentation.svg) / [detail](../physical/figures/power-grid-detail-presentation.svg) | Accepted final ODB cell footprints and power-only shapes; signal routes not displayed |
| [Implementation-stage timing](../physical/figures/timing-stages-presentation.svg) | Nangate45 [measured stage results](../physical/RESULTS.md#setup-and-hold); ideal/propagated clocks and estimated/extracted RC explicitly distinguished |
| [Area/cell-count evolution](../physical/figures/area-stages-presentation.svg) | Accepted OpenDB inventories; taps/fillers excluded |
| [Hold decomposition](../physical/figures/hold-path-contributions-presentation.svg) | [Matched-path analysis](../physical/HOLD_ANALYSIS.md); estimated/extracted RC, changed worst endpoint |

Original SVGs, accepted PNG/PDF charts and their physical provenance are
preserved byte-for-byte. The main README uses one sized layout overview and
measured charts; routed-metal, placement and power-grid detail stays in
[physical documentation](../physical/README.md).

## Presentation regeneration and provenance

```bash
python3 scripts/render_presentation_figures.py --output /tmp/vector-mac-presentation
```

Use a new output directory. The recorded renderer uses Matplotlib 3.9.2 and
NumPy 1.26.4. The generator reads the unchanged `physical/summary.json`,
`physical/hold-analysis.json` and accepted power table in `syn/README.md`;
setup is plotted in ns and hold in ps. Area/count, power and matched-path
contributions retain the exact accepted values before display rounding.
Vertically stacked chart panels keep the labels usable at narrow widths.

For layout figures, the generator composes a new SVG using the exact original
PNG body at native scale. Only the outer header/footer strips change. The body
includes all geometry, coordinate axes and legends; no GDS/ODB is re-read or
geometry inferred. The power chart uses the accepted SAIF table and correct
storage/gating terminology; it adds no power measurements. Common browser font
fallbacks keep the new chart text readable without an installed plotting font.

[Presentation manifest](../physical/figures/presentation-provenance.json)
records source/generator/output SHA-256s, plotted values, stage labels, retained
pixel bounds and accepted GDS/ODB hashes. The manifest supplements the immutable
[original chart provenance](../physical/figures/provenance.json) and
[original layout-image provenance](../physical/figures/p9_provenance.json).
[Archive identifiers](../physical/PROVENANCE.md) remain available for traceability.

Original presentation sources remain unchanged:
[power](figures/power_comparison.svg),
[timing](../physical/figures/timing-comparison.png),
[area](../physical/figures/area-comparison.png),
[hold](../physical/figures/hold-decomposition.png),
[overview](../physical/figures/overview_dark.png),
[routing detail](../physical/figures/routing_detail.png),
[placement](../physical/figures/placement_overview.png) and
[power-grid detail](../physical/figures/pdn_detail.png).
Their historical labels and hashes are deliberately preserved.

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
`python3 scripts/render_readme_figures.py`. Accepted physical chart and layout
regeneration requires the inputs and tools documented in
[REPRODUCE.md](../physical/REPRODUCE.md).
