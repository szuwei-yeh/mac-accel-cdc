# Engineering stages and archived identifiers

Reader-facing tables and figures use the engineering stage names below.
Historical identifiers remain in manifests, filenames, adapter commands and
historical Git snapshots so that accepted evidence can still be located.

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
| `P10` | Documentation clarity and commit-specific CI review | [Archive cross-check records](#archive-cross-check-records); no new design measurements |

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

## Archive cross-check records

The documentation review against commit
[`5a456654b6847f50c89f4166355593a5f5427242`](https://github.com/szuwei-yeh/mac-accel-cdc/commit/5a456654b6847f50c89f4166355593a5f5427242)
recorded the following read-only archive cross-checks. Paths are relative to
that preserved implementation archive, not files supplied in this checkout.
These are historical provenance records, not a new implementation or replay.

| Archived record | SHA-256 / binding verified |
|---|---|
| `P4/audit/final-inputs/config.mk` | `1ec2dd873610d983a54bb23d64e763158b3f999ba64d7c7c205c4c5a81cb2ce0`; matches `pins.json` accepted P4 config hash |
| `P4/audit/summary-final.json` | `ef003577886c69f82c4de3c656dc2c090fa42fa759b0ae7756b8fe4f6f080266`; matches public `summary.json` P4 summary pin |
| `P4/exports/final_repair1.sdc` | `753f0ea4f6f07fdc864692cad92a5a9d192bb0fbc95babae4ece5bd9a1511e1e`; matches accepted SDC artifact hash; byte-identical to archived `antenna_split3/4_cts.sdc` |
| `P4/audit/antenna-clock-buffer-repair.json` | `ab6719e6d0fc58b5f6d64da10827a0c4eb0c69bf3c1cfeef7034d7aa6e5e4f29`; records three explicit `CLKBUF_X3` seeds and legalized coordinates |

The accepted summary records a 415.15 × 414.40 µm core, a 417.39 µm square die,
296 rows and 40.5516729544% standard-cell utilization (40.552% published).
Archived global-route logs record 135 + 271 = 406 added hold buffers across
`hold50` and `antenna_split3`, with `-hold_margin 0.05`.
[Measured results](RESULTS.md) and [matched-path analysis](HOLD_ANALYSIS.md)
retain the area, clock-latency and changed-endpoint interpretation.

At pinned ORFS revision `e5ce907f878490bf9947347f4cd4b73e97451c80`, the
[floorplan script](https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts/blob/e5ce907f878490bf9947347f4cd4b73e97451c80/flow/scripts/floorplan.tcl)
derives dimensions from utilization, aspect ratio and margins; the
[variable definitions](https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts/blob/e5ce907f878490bf9947347f4cd4b73e97451c80/flow/scripts/variables.yaml)
distinguish placement-density and hold-repair controls. The pinned
[Nangate45 platform configuration](https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts/blob/e5ce907f878490bf9947347f4cd4b73e97451c80/flow/platforms/nangate45/config.mk)
inherits the M1/M4/M7 PDN strategy. The historical review recorded that the
platform config and grid-script SHA-256s matched [platform pins](pins.json).
Accepted-script hashes and sanitized template hashes intentionally differ
where templates tokenize machine-specific paths.

Storage dimensions and gating scope remain documented in the
[OSU block experiment](../syn/README.md#targeted-buf_a-clock-gating-experiment):
256 16-bit `buf_a` entries / 4,096 register bits, with minimum gating
width/fanout 16 and `-no_sharing` in the unchanged
[power flow](../syn/dc_power_targeted.tcl). Those dimensions do not establish
independently banked memories; LATCH + AND gating remains a pre-layout estimate.
