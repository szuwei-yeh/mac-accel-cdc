# Reproduction requirements and commands

## Pinned inputs

Use a Linux environment with GNU Make/time, Python 3, Tcl, OpenROAD/OpenSTA, KLayout, and OSS CAD Suite Yosys/ABC/Icarus/SymbiYosys formal solvers. Supply user-space installations; no installer, privileged action or system environment change is part of this package. Two CPU threads and one flow at a time are the qualified settings. Scripts bind CPU 0/1, so adjust that explicit affinity only when those CPUs are unavailable and record the change.

`pins.json` records RTL commit/source hashes, ORFS and actual tool commits, all Nangate45 platform hashes, installation provenance and archive checksums. OpenROAD 26Q2-1164-g08f67ee5ec differs from the OpenROAD submodule expected by the pinned ORFS revision; compatibility is based on the completed reference/pilot/full flows, not assumed commit identity. Nangate45 must include Liberty, LEF, cell GDS, RC rules, KLayout technology/mapping and its supported DRC deck. Obtain these separately under their applicable terms; no libraries are republished.

A tools prefix must provide `bin/openroad`, `bin/sta`, `bin/klayout`, and `bin/abc-batch`. The latter forwards to the OSS CAD Suite ABC batch executable with deterministic noninteractive operation. Existing native installation wrappers and relocated dependencies were reused; exact wrapper hashes are historical evidence, not portable paths. Versions and source/archive hashes are the portability pins. A portable batch launcher can use `exec "${OSS_SUITE_BIN:?}/yosys-abc" "$@"`; place it in the supplied tools prefix and record its hash. P2 generates qualified functional models from the supplied Liberty locally; those generated models are not checked in.

## Prepare without running the flow

Run from the repository root. Choose a **new, nonexistent** output directory; use paths without whitespace/shell metacharacters. Example environment variables point to your own tools, source and writable experiment storage:

```bash
python3 physical/reproduce.py \
  --source "$PWD" --orfs "$ORFS_ROOT" --tools "$MAC_TOOLS_ROOT" \
  --suite-bin "$OSS_SUITE_BIN" --output "$MAC_REPLAY_ROOT"

# Validate source/platform hashes and executable versions; materialize isolated inputs.
python3 physical/reproduce.py \
  --source "$PWD" --orfs "$ORFS_ROOT" --tools "$MAC_TOOLS_ROOT" \
  --suite-bin "$OSS_SUITE_BIN" --output "$MAC_REPLAY_ROOT" --prepare
```

The first command prints a plan only. `--prepare` never runs synthesis or P&R. It creates `p2`, `p3`, `p4` workspaces, redirects outputs/temp/cache/config/runtime there, hydrates path tokens in 46 reviewed scripts/configurations, and records source/tool/platform pins. The existing ORFS checkout and RTL remain read-only. Template hashes are verified before hydration; root README changes do not invalidate RTL/verification hashes.

## Explicit stage execution

After reviewing the prepared configuration and resource availability, the following experimental commands each run one stage. They stop on the production acceptance checks described below; inspect all measured outputs before making a closure claim:

```bash
python3 "$MAC_REPLAY_ROOT/stage.py" P2
python3 "$MAC_REPLAY_ROOT/stage.py" P3
python3 "$MAC_REPLAY_ROOT/stage.py" P4
```

P2: RTL/formal regression → CDC semantic tests → elaboration/parameter audit → Nangate45 synthesis and mapped-cell audit → GLS → qualified native readback and independent STA → exact constraint coverage. No OSU netlist is reused.

P3: copy only accepted qualified P2 netlist/ODB/SDC → `do-floorplan` → `do-place` → `do-cts` → mapped CDC/clock/register/PG/placement checks → coverage and GLS. Synchronizer hooks temporarily release cell protection only for supported physical repair, then verify all 26 cell identities/direct chains/clock assignments and restore protection. No retiming, clock gating or cloning is authorized.

P4: `do-grt`, `do-5_2_route`, `do-5_3_fillcell`, `do-5_route`, `do-5_route.sdc`, `do-finish`, `do-gds`. Run KLayout DRC with all supported rule groups and two workers in qualified flat mode. The adapter now **stops on every nonzero or incomplete GDS report**. It never automatically applies the archived three-buffer clock-input segmentation recipe: that historical repair required a separately inspected experiment and rerouting. The retained recipe is archival reference only. Final exported netlist/SDC/SPEF are independently loaded by OpenSTA; annotation/CDC/clock/coverage checks and seven zero-delay integration cases follow. Router DRC and GDS DRC/antenna are distinct evidence. Native LEF antenna results remain INCONCLUSIVE because the required model is absent; LVS remains UNSUPPORTED.

Each run keeps logs, reports, checkpoints, resource timing and status under its new root. `accepted.json` records stage completion, not foundry signoff. Do not overwrite or resume into sealed historical runs. Failed implemented timing/coverage/DRC gates require investigation; do not relax periods or exceptions to produce PASS.

## What P5 validated

The **original archived execution** completed P2–P4 with all stated gates. The new portable adapter was syntax checked, hydrated against the qualified tools/source/platform, and checked against the pinned Makefile variables/targets without running synthesis/P&R. A fresh end-to-end portability rerun has **not** been performed; new runs may differ in placement/routing and must be measured independently. Historical failed probes/iterations are preserved locally, but are not required deliberate steps of the clean reproduction recipe. The accepted P3 trajectory used placement checkpoints from `native_guards` and a separate power-connectivity repair before the `pg_final` CTS run; the adapter runs its consolidated hooks directly in `pg_final`. This clean recipe has not been demonstrated to reproduce that trajectory or its measured metrics. Summaries report the accepted evidence, not an unexecuted adapter result.

Final large artifacts are identified by filename/size/hash in `summary.json` and are intentionally absent from Git. The compact figure/metric regeneration scripts read these user-supplied artifacts or the public summary; no fake layout is synthesized for documentation.

## Experimental adapter acceptance limits

P6R corrected the two acceptance blockers and tested the **production code**. Every CDC row must match the mapped eight-group inventory: 602 bounded maxima must be finite and nonnegative; the three event crossings must have only the intended first-stage exceptions. All 13 direct two-stage chains, their 26 active setup/hold checks, clock roots, register identities, constraint coverage and five bound-effect probes are required. Positive synchronous-domain WNS cannot override negative CDC slack. Native Tcl audits also abort on negative required CDC/interstage slack.

GDS acceptance requires matching SHA-256/size records, checker exit/completion evidence, exact input/output/deck command paths, the qualified deck hash, all 133 rule categories and a valid XML item inventory. An empty item list alone is insufficient. Antenna markers require explicit net/pin context from the matching ODB and polygon coordinates matching the reported marker and the measured pin rectangle. Missing correlation rejects the result; every nonzero marker rejects it even when metadata is valid. The qualified deck is the stock rules in flat mode with `threads(2)`; the freshly checked generated deck differs from the archived deck only in a scheduling comment. Both hashes are retained. This is a research checker contract, not foundry signoff or a general antenna repair method.

The read-only **artifact acceptance path** was executed on the exact accepted P4 netlist, SDC, SPEF, ODB, DEF and GDS. Fresh native/standalone STA, mapped readback, coverage, annotation and complete supported KLayout DRC/antenna checks passed. Fault-injection controls call the same production validators. The nonzero antenna fixture is explicitly synthetic, at an actual final OpenDB pin rectangle with its measured net/pin identity; it is not a measured final violation. Its geometry is never written into the accepted database.

```bash
# A manifest describes your existing physical files and complete audit evidence.
python3 physical/acceptance.py --bundle "$MAC_ACCEPTANCE_BUNDLE" \
  --output "$MAC_ACCEPTANCE_RESULT"

# Fault-inject copies only; --work must not exist.
python3 physical/test_acceptance.py --bundle "$MAC_ACCEPTANCE_BUNDLE" \
  --work "$MAC_CONTROL_ROOT"
```

Bundle schema 1 requires `source_commit`, `source_root`, `platform_root`, six `artifacts` keyed `v/sdc/spef/odb/def/gds`, and `evidence` keyed `pins`, `summary`, `baseline`, `mapped`, `readback`, `cdc`, `sync`, `sta_log`, `coverage`, `drc_xml`, `drc_metadata`, `deck`, `drc_log`, `drc_execution`, `router_drc`, `router_log`. Each file record contains `path`, `bytes`, `sha256`; paths resolve relative to the bundle. `router_artifacts_sha256` binds ODB/DEF/GDS. `accepted_reference: true` additionally requires the exact published final artifact/router hashes. Checker metadata includes completed status/exit/version, six evidence/input hashes and marker counts; the execution record contains the actual command and exit code. For nonzero control fixtures, provide `antenna-fault-record.json` beside the bundle, with `item_xml`, marker `binding` and DB-derived `context` (ODB hash, net/pin membership and `pin_locations_um`). The validator and tests define the precise machine schema. Large physical files and local manifests are intentionally excluded from Git.

SHA-256 records detect accidental mismatch; they are not signed attestations. The checker and native database reader must run in a trusted environment. A manually rewritten manifest cannot prove that a claimed execution happened.

The complete **fresh RTL-to-GDS stage replay remains untested**. P6R did not rerun synthesis, placement, CTS or routing. `stage.py P2/P3/P4` remains experimental: consolidated P3 hooks and a new physical trajectory may differ from the archive, and a recurring antenna violation now stops for investigation. No claim is made that the adapter reproduces the archived repair trajectory or exact PPA. The archive's accepted seven-case GLS result is retained; GLS was not rerun for this acceptance-code repair.

Use normal, unoptimized Python for preparation/stage execution: legacy assertions remain and these entry points now explicitly reject optimized mode. The shared acceptance validator uses explicit exceptions, which remain active under `python -O`. Preparation verifies source/platform/template hashes and version strings; it does not rebuild tool binaries or authenticate all binaries against installation checksums. Record your supplied binary hashes separately. Qualified Python is 3.12; plotting additionally requires the recorded NumPy, Matplotlib and Pillow versions. Existing single-corner, reset/RDC, MTBF, SDF, LVS and signoff limitations still apply.

## Regenerate figures from measured data

Set temporary/cache/Matplotlib directories to your writable experiment root. No new tool installation is needed if the recorded plotting tools are present.

```bash
python3 physical/plot_results.py --output "$MAC_FIGURE_ROOT/charts"

# Native read-only OpenDB export; point to your own accepted ODB and Liberty.
MAC_LIB="$NANGATE_LIB" MAC_ODB="$ACCEPTED_CTS_ODB" \
  MAC_GEOMETRY="$MAC_FIGURE_ROOT/cts-geometry.json" \
  "$MAC_TOOLS_ROOT/bin/openroad" -no_init -exit -python physical/export_geometry.py
python3 physical/plot_layout.py --stage cts \
  --geometry "$MAC_FIGURE_ROOT/cts-geometry.json" --output "$MAC_FIGURE_ROOT/cts"

# KLayout offscreen GDS read/render: create the output directory first.
QT_QPA_PLATFORM=offscreen "$MAC_TOOLS_ROOT/bin/klayout" -zz \
  -rd "input_gds=$ACCEPTED_GDS" -rd "layer_properties=$NANGATE_LYP" \
  -rd "output_dir=$MAC_FIGURE_ROOT/gds" -r physical/render_gds.py
```

`render_gds.py` uses this measured 417.39 µm die and a specified 30 × 30 µm detail window. The CTS plot shows actual buffer coordinates and logical buffer-to-buffer connections as straight flylines, not routed geometry. `templates/p3/render.tcl` preserves the native original GUI renderer for the archived floorplan view. Rendered layer selection/captions and source SHA-256s are in `figures/provenance.json`. `measure_hold_paths.tcl` reproduces matched-path read-only STA using `MAC_LIB`, `MAC_DB`, `MAC_SDC`, `MAC_SPEF`, `MAC_STAGE`, and `MAC_RUN` (create `analysis/` there).
