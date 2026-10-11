# Engineering stages and archived identifiers

Reader-facing tables and figures use the engineering stage names below.
Historical identifiers remain in manifests, filenames, adapter commands and
archived review snapshots so that accepted evidence can still be located.

| Historical label | Engineering meaning | Timing model / evidence |
|---|---|---|
| `P1A` | Toolchain qualification with the official GCD reference flow | Completed GDS/supported checks; original 0.46 ns target did **not** close; [recorded result](RESULTS.md#resources-and-qualification-controls) |
| `P1B` | Standalone signed MAC pilot qualification | Separate 7.5 ns pilot contract; not the full-top result |
| `P2 synthesis` / `P2 Mapped` | **Post-Synthesis** | Pre-layout, ideal clocks, no extracted RC |
| `P3 placement` / `P3 Placed` | **Post-Placement** | Placement-estimated RC, ideal clocks |
| `P3 CTS` | **Post-CTS** | Placement-estimated RC, propagated clocks |
| `P4 routed` / `P4 Final` | **Post-Route** | Routed OpenRCX SPEF, propagated clocks |
| `P5` | Evidence/provenance audit and reproduction preparation validation | [Evidence chain](METHODOLOGY.md#evidence-chain) and [adapter preparation limits](REPRODUCE.md#reproduction-preparation-and-provenance-validation); fresh RTL-to-GDS replay untested |
| `P6R` | Acceptance-validator repair and validation | [CDC/antenna acceptance controls](REPRODUCE.md#experimental-adapter-acceptance-limits); read-only accepted-artifact validation, no new physical implementation |
| `P9` | Layout presentation and figure provenance | [Original image-copy manifest](figures/p9_provenance.json); accepted routed GDS and final ODB geometry |
| `P10` | Documentation clarity and commit-specific CI review | [Historical review snapshot](../docs/P10_REVIEW.md); no new design measurements |

`P3` covers floorplanning, placement and CTS; `P4` covers routing, extraction
and GDS validation. Resource-use rows therefore describe the whole activity
range, rather than suggesting that a checkpoint label names one command.

## Exact accepted checkpoints

The following are the real archive IDs in [summary.json](summary.json), not
new names for directories or machine-readable keys:

| Engineering scope | Preserved archive ID |
|---|---|
| Synthesis and mapped-design validation | `P2_v3_20261010T200353564218Z` |
| Floorplan, placement and CTS | `P3_v3_20261010T203048488005Z` |
| Routing, extraction and GDS validation | `P4_v3_20261010T210404869162Z` |

The adapter still accepts `P2`, `P3`, `P4` and prepares `p2`, `p3`, `p4`
directories. [Pinned templates](pins.json) retain their original names and
hashes. CSV/JSON stage keys and source/tool/artifact hashes are unchanged.

Within the routing history, `hold50` identifies the first global-route
hold-repair run; `antenna_split3` identifies the explicitly seeded three-buffer
clock-input repair run. Placement history retains `native_guards` and the
power-connectivity-repaired `pg_final` CTS run. These identifiers are used only
beside their engineering role in [reproduction](REPRODUCE.md) and
[matched-path analysis](HOLD_ANALYSIS.md).

## Presentation figures

[Presentation provenance](figures/presentation-provenance.json) binds new
stage-labelled charts to the unchanged public measurements and new layout
compositions to the immutable accepted images. Original figures, manifests
and generation/reproduction scripts remain available. Relabeling does not
create new timing, power, routing or signoff evidence.
