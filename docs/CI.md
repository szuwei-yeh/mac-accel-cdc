# Lightweight CI

[rtl-ci.yml](../.github/workflows/rtl-ci.yml) runs on pushes, pull requests and
manual dispatch, using a GitHub-hosted Ubuntu runner, Icarus Verilog and Python.
See [workflow runs](https://github.com/szuwei-yeh/mac-accel-cdc/actions/workflows/rtl-ci.yml)
for ongoing CI status.
Checkout and Python setup actions are pinned to release commits. The job has
read-only repository permissions and a ten-minute timeout.

## Run locally

From the repository root, with `iverilog`, `vvp`, Git and Python 3 installed:

```bash
python3 scripts/run_ci_rtl.py
python3 sim/run_cdc_regression.py --simulator iverilog
python3 scripts/check_portfolio.py --python-syntax
python3 -m unittest discover -s tests -v
python3 scripts/check_publication.py
```

The current integrated inventory is 243 cases across 27 TBs; the table below
shows selected component suites. The separate frozen closure stress records
108 jobs. `check_publication.py` also checks the new routed revision binding
without EDA execution; optional `--archive` checks the external package bytes.

## Coverage

| Check | Required result / scope |
|---|---|
| V3 full top, depth 8 | 7 PASS / 0 FAIL; AXI-Lite → DMA/ROB → FIFO → MAC → result |
| OOO ROB engine, depth 8 | 12 PASS / 0 FAIL; response ordering and protocol checks |
| ROB DMA wrapper, depth 8 | 11 PASS / 0 FAIL; A/B pairing and stalls |
| CPU-fed MAC/CDC | 8 PASS / 0 FAIL; basic dual-clock behavior |
| Supplemental V3 CDC matrix | Six scenarios × six jobs; payload, events and Gray pointers |
| Python syntax | Compile published/new Python sources without executing EDA scripts |
| Acceptance unit tests | Synthetic temporary fixtures exercise production validation boundaries |
| Documentation integrity | Inline local Markdown links/fragments, PNG headers, paired fences, private-path/credential patterns, pinned source/template hashes and original layout-image hashes |

The RTL runner requires exact scoreboard summaries and rejects timeout/failure
diagnostics even when the testbench exits zero through `$finish`. Build products
and waveforms go into temporary workspaces. The existing CDC runner preserves
its unique temporary reports for inspection.

Acceptance unit fixtures are explicitly synthetic. They test fail-closed numeric,
file, table and GDS-report provenance checks without claiming a real DRC run.
The archived-bundle fault-injection suite in `physical/test_acceptance.py` is
excluded because it requires the full evidence archive.

## Exclusions

CI does not run Synopsys, synthesis, formal solvers, mapped GLS, Nangate45/PDK
installation, OpenROAD, extraction, LVS, or full physical acceptance. Historical
formal/physical results remain recorded evidence. A passing lightweight job
does not establish physical signoff or a fresh RTL-to-GDS replay.

## Verified hosted results

| Published commit | Hosted run | Verified result |
|---|---|---|
| [`37c926a`](https://github.com/szuwei-yeh/mac-accel-cdc/commit/37c926a2989aad01d118e07bb837a80373d16a53) | [38174881093](https://github.com/szuwei-yeh/mac-accel-cdc/actions/runs/38174881093) | Completed successfully; exact SHA and every job/check step verified through the Actions API |
| [`5a45665`](https://github.com/szuwei-yeh/mac-accel-cdc/commit/5a456654b6847f50c89f4166355593a5f5427242) | [38171676828](https://github.com/szuwei-yeh/mac-accel-cdc/actions/runs/38171676828) | Completed successfully; every job/check step passed; job completed 2026-10-11 20:19:15 UTC |
| [`3e81562`](https://github.com/szuwei-yeh/mac-accel-cdc/commit/3e815629718f3f1c9c8a1504cad0c7306ff95b91) | [38152191558](https://github.com/szuwei-yeh/mac-accel-cdc/actions/runs/38152191558) | Earlier published baseline; successful hosted run |

The run and job records were checked through GitHub's public Actions API.
These results belong only to the listed commits; they do not establish hosted
CI success for later revisions.
