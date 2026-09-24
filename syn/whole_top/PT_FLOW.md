# Whole-top standalone PrimeTime

Run from the UCSB project root with Python 3.6+:

```sh
python3 syn/whole_top/run_pt.py
# Optional explicit accepted DC bundle and new output directory:
python3 syn/whole_top/run_pt.py \
  --mapped-from /absolute/path/to/accepted/dc/bundle \
  --run-root /tmp/new-whole-top-pt-run
```

Create an accepted input bundle with `package_dc.py` as described in
[DC_FLOW.md](DC_FLOW.md#package-for-primetime), then pass it using `--mapped-from`.
The archived-run default input bundle is
`syn/runs/whole_top_mo8_bus10_mac7p5_20260924_dc`.
It must contain the Step-4 `ACCEPTANCE.json` with status
`MAPPED_PAIR_VERIFIED_BY_INDEPENDENT_DC_READBACK`. The mapped Verilog and resolved
SDC SHA-256 values must match that acceptance record. Configuration provenance
is `mac_accel_dma_rob_top`, `MAX_OUTSTANDING=8`; PrimeTime does not elaborate RTL.

The runner uses `/ece/synopsys/prime/R-2020.09-SP5-1/bin/pt_shell` and
`/fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db`.
It inherits `LM_LICENSE_FILE`, with a process-local UCSB default when absent.
There is no global environment change. The known problematic V-2023.12 tool
is not used.

Without `--run-root`, each invocation creates a unique
`/tmp/vectormac-whole-top-pt-*` directory. An explicit existing output directory
is rejected. Inputs, library bytes, Tcl helpers, runner, DC reference reports
and provenance are copied into `inputs/` and hashed in `manifest.json`.
Save the entire run directory to a new project artifact directory after review;
`/tmp` alone is not durable storage. Library-containing run bundles are private
EDA evidence and are not source deliverables.

`pt_flow.tcl` reads and links the mapped design, imports the exact resolved SDC,
updates timing, and generates clock, port, constraint, max/min timing, global
timing, coverage, disabled-arc and exception reports. It never applies a second
SDC profile or invokes synthesis. `effective_constraints.sdc` is a diagnostic
export; it does not replace the accepted input SDC.

`pt_checks.tcl` checks the actual clocks and all functional register clock pins,
classifies constant inactive preset warnings, validates the mapped CDC structure,
and checks both bounded cross-domain paths and synchronous synchronizer paths.
Existing DC helper files supply procedure definitions only. PT-specific query
adaptations use explicit functional `/D` pins and timing-path collections; they
do not change constraints or the netlist.

Useful outputs:

- `status.json`, `pt.log`, `manifest.json`: execution, diagnostics and provenance.
- `reports/{bus_clk,mac_clk}_{max,min}.rpt`: complete worst paths per domain.
- `reports/path_metrics.tsv`: worst slack, violation-only WNS, TNS and violating
  endpoint counts. TNS sums one worst negative slack per endpoint in each group.
- `reports/dc_{bus,mac}_max_max.rpt`: the fixed DC reference setup paths in PT.
- `reports/{check_timing,analysis_coverage,clock_pin_audit,pt_cdc}.rpt`:
  constraint coverage and structural checks.

The R-2020.09-SP5-1 startup prints `Error: ... (PT-063)` because the Library
Compiler executable path is not configured. The tested `.db` flow successfully
reads, links, updates and reports timing despite it. This exact diagnostic is
retained in `known_nonblocking_diagnostics`; other `Error:` diagnostics fail the
runner. A zero tool return code and the final flow-completion marker are also
required. `flow_complete` is execution status, not a timing-closure claim.

The analyzed portfolio profile uses asynchronous ideal clocks (bus 10 ns,
MAC 7.5 ns), functional bus I/O delays min 0/max 1 ns, and the existing typical
OSU 0.18 um library. It has no extracted RC, CTS, board I/O model, clock
uncertainty or multi-corner signoff qualification. Raw constant-pin and reset
coverage exclusions remain visible. Passing timing does not establish CDC/RDC,
reset-release, equivalence or physical signoff.

The observed results, input fingerprints and limitations are published in
[RESULTS.md](RESULTS.md). Generated bundles, raw logs, and technology files are
excluded from Git; a fresh clone requires licensed tools and the stated library
to regenerate them.
