# P10 Engineering Clarity Polish review

> Historical pre-publication review snapshot against `5a456654b6847f50c89f4166355593a5f5427242`.
> Status statements and milestone names below describe that review, not the
> current repository. For current engineering explanations and hosted results,
> see the [main README](../readme.md), [physical documentation](../physical/README.md)
> and [CI evidence](CI.md). The original review body is preserved.

Review baseline: published commit
[`5a456654b6847f50c89f4166355593a5f5427242`](https://github.com/szuwei-yeh/mac-accel-cdc/commit/5a456654b6847f50c89f4166355593a5f5427242).
Working branch: `polish/vector-mac-p10-clarity`. Changes remain uncommitted;
no remote settings are changed.

## Exact change scope

| File | Change |
|---|---|
| [Main README](../readme.md) | Correct buffer terminology, annotate the preserved graphic's historical “banks” label, update the hosted CI reference and link physical decisions |
| [Synthesis README](../syn/README.md) | Correct the same storage/banking terminology; retain all measured tables and gating configuration |
| [Physical README](../physical/README.md#physical-design-decisions-and-trade-offs) | Add configured-input/tool-optimization distinctions and grounded trade-offs |
| [CI documentation](CI.md) | Record the successful exact-commit hosted run; preserve the earlier baseline run |
| [Metadata proposals](GITHUB_METADATA.md) | Identify RID-indexed V3/Nangate45 research scope and explain material metadata changes; retain the existing topic set |
| This review | Record evidence provenance, executed checks and remaining limits |

Inspect tracked edits with:

```bash
git diff 5a456654b6847f50c89f4166355593a5f5427242 -- \
  readme.md syn/README.md physical/README.md docs/CI.md docs/GITHUB_METADATA.md

# New review document; exit 1 means a diff exists.
git diff --no-index /dev/null docs/P10_REVIEW.md
```

## Evidence and provenance

**Storage and gating.** [RTL](../rtl/mac_dma_rob.v) declares
`reg signed [DATA_WIDTH-1:0] buf_a [0:MAX_LEN-1]`, with defaults 16 and 256.
[The OSU power flow](../syn/dc_power_targeted.tcl) elaborates depth 8, includes
only `buf_a` registers, and configures minimum gating width/fanout 16 with
`-no_sharing`. Array dimensions establish 256 storage entries / 4,096 bits;
they do not establish independently banked memories. Synthesis-inserted
LATCH + AND gating and all existing power/timing numbers retain their OSU
pre-layout scope. No gating element report was regenerated.

**Configured physical inputs.** [P3](../physical/templates/p3/config.mk),
[P4](../physical/templates/p4/config.mk), the
[SDC](../physical/templates/p2/constraint.sdc), and
[repair guards](../physical/templates/p3/repair_guard_post.tcl) support the
utilization/density, clock/CDC, synchronizer and hold-repair descriptions.
[pins.json](../physical/pins.json) records both accepted-script and sanitized
template hashes; those are intentionally different for path-tokenized files.

Pinned ORFS revision: `e5ce907f878490bf9947347f4cd4b73e97451c80`.
Its [floorplan script](https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts/blob/e5ce907f878490bf9947347f4cd4b73e97451c80/flow/scripts/floorplan.tcl)
derives dimensions from utilization/aspect/margins; its
[variable definitions](https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts/blob/e5ce907f878490bf9947347f4cd4b73e97451c80/flow/scripts/variables.yaml)
distinguish density and hold-repair controls. The pinned Nangate45
[platform configuration](https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts/blob/e5ce907f878490bf9947347f4cd4b73e97451c80/flow/platforms/nangate45/config.mk)
inherits the M1/M4/M7 PDN strategy. Downloaded platform config and grid script
SHA-256s matched the existing platform pins. No custom PDN motivation,
optimal-density claim, or manually fixed cell/CTS topology is inferred.

**Archive cross-checks.** The following paths are relative to the preserved
archive, not public checkout files. All were read only; the large archive is
not added to Git.

| Archived record | SHA-256 / binding verified |
|---|---|
| `P4/audit/final-inputs/config.mk` | `1ec2dd873610d983a54bb23d64e763158b3f999ba64d7c7c205c4c5a81cb2ce0`; matches `pins.json` accepted P4 config hash |
| `P4/audit/summary-final.json` | `ef003577886c69f82c4de3c656dc2c090fa42fa759b0ae7756b8fe4f6f080266`; matches public `summary.json` P4 summary pin |
| `P4/exports/final_repair1.sdc` | `753f0ea4f6f07fdc864692cad92a5a9d192bb0fbc95babae4ece5bd9a1511e1e`; matches accepted SDC artifact hash; byte-identical to archived `antenna_split3/4_cts.sdc` |
| `P4/audit/antenna-clock-buffer-repair.json` | `ab6719e6d0fc58b5f6d64da10827a0c4eb0c69bf3c1cfeef7034d7aa6e5e4f29`; records three explicit `CLKBUF_X3` seeds and legalized coordinates |

The accepted summary confirms the 415.15 × 414.40 µm core, 417.39 µm square
die, 296 rows and final 40.5516729544% standard-cell utilization (40.552%
published). Accepted global-route logs report 135 + 271 = 406 added hold
buffers across `hold50` and `antenna_split3`, with `-hold_margin 0.05`.
[Published results](../physical/RESULTS.md) and
[matched-path analysis](../physical/HOLD_ANALYSIS.md) document area/clock
consequences and the changed worst endpoint. Final outcomes are tied to the
pinned accepted summary, not inferred from an earlier iteration's filename.

**Hosted CI.** GitHub's
[run record](https://api.github.com/repos/szuwei-yeh/mac-accel-cdc/actions/runs/38171676828)
binds run `38171676828` to the full baseline SHA above and reports
`completed` / `success`. The
[job records](https://api.github.com/repos/szuwei-yeh/mac-accel-cdc/actions/runs/38171676828/jobs)
report success for every step, including RTL, CDC, syntax, unit tests and
documentation. The job completed 2026-10-11 20:19:15 UTC.
The current uncommitted P10 revision has no hosted run.

**Metadata and hierarchy.** The public repository API still reports the earlier
Nexys A7 description and an empty topic list. Applying the updated V3 description
and existing topic proposals is material. The main README's four scoped
highlights and architecture-first progression already satisfy the scan/deep-read
goals; no broader hierarchy redesign is proposed.

## Executed validation

| Local check | Result |
|---|---|
| `python3.12 scripts/run_ci_rtl.py` | PASS: 38 cases (7 full top, 12 ROB, 11 DMA wrapper, 8 CPU-fed/CDC) |
| `python3.12 sim/run_cdc_regression.py --simulator iverilog` | PASS: six scenarios × six jobs, 36 jobs |
| `python3.12 -m unittest discover -s tests -v` | PASS: 13 self-contained unit tests |
| `python3.12 scripts/check_portfolio.py --python-syntax` | PASS: 38 Python files compiled without EDA execution |
| `python3.12 scripts/check_portfolio.py` | PASS: local Markdown links/fragments, fences, PNG headers, privacy patterns and pinned source/template/P9 hashes |
| Additional HTML/reference and render checks | All six local HTML image references resolve; nine README image instances load; desktop/mobile previews checked with no page overflow |
| `git diff --check` | PASS |
| Preservation against baseline | All 157 other published files byte-identical; original measurement tables in physical/synthesis READMEs unchanged |

Only five existing Markdown documents and this new review are changed. No
RTL, SDC, config/hook, proof, golden file, measurement output, accepted archive,
original figure or reproduction/CI script is modified. No synthesis, P&R,
extraction, STA, DRC, formal solver or mapped GLS was rerun for P10.

## Remaining limitations

LVS remains unsupported; extracted timing is single-corner with the original
I/O/reset/uncertainty contract. Native LEF antenna remains inconclusive;
supported GDS checks provide operative evidence. Fresh portable RTL-to-GDS
replay remains untested. No foundry signoff, multi-corner/OCV, SDF/LEC,
MTBF/RDC, dynamic IR/EM or Nangate45 signoff-power claim is added.

The immutable historical power graphic and its generator still contain the
“banks” label. Adjacent README text explains its limited clock-gating meaning;
those original files are preserved under the requested scope. No requested
documentation change required inventing measurements or design motivations.
