# Integrated candidate: current evidence

The final functional RTL is frozen. The subsequent pre-layout requalification
and **new UCSB routed implementation have passed within the documented scope**.
Earlier closure reports and qualified6 record an earlier stage; their blocked
or pre-layout-only statements are historical, not the current readiness state.

| Evidence | Source-bound recorded result |
|---|---|
| Integrated RTL | 243/243 case IDs across 27 TBs |
| Multi-seed / multi-clock stress | 108/108 jobs; seeds 1,19,2345 and MAC periods 7.5,30,3.5 ns |
| V3 CDC | 36/36 jobs, six scenarios |
| Closure-era unit / structural checks | 31/31 units; 8/8 lint/structural checks |
| Reduced formal | Seven unchanged model/config-bound tasks reused; not full-top equivalence |
| New frozen pre-layout qualification | 28,885 mapped cells / 7,509 FFs, electrical violations=0, UNPLACED OpenDB readback |
| Final routed qualification | [Latest metrics, repairs and scope](ROUTED_RESULTS.md); native + independent extracted STA, supported DRC/antenna and final-netlist zero-delay GLS |

Results are reused by exact source/artifact binding; document preparation does
not rerun functional or physical experiments. Case/job categories overlap and
must not be summed as a unique-test total. Qualification-helper unit checks have
a separate later count in their own record; they do not change the closure-era
31/31 claim. Hosted CI has not run on this uncommitted candidate.

## Revision records

- [Closure freeze](../evidence/rtl-closure-freeze.json): nine RTL and 43 functional
  input hashes; the recorded P&R blocker at creation was resolved by later evidence.
- [Closure-era revision](../evidence/revisions.json) and [qualified6](../evidence/qualified6.json):
  retained; the pending-tail correction invalidated qualified6 for current V3.
- [Frozen requalification](../evidence/requalification.json) and
  [new checkpoint](../evidence/qualified-frozen-20261012.json): source manifest
  `706323b6…`, exact frozen RTL, separately qualified artifacts.
- [New routed binding](../evidence/routed-20261012/binding.json): current candidate
  physical evidence, compact checks and figures; full 794-file archive stays external.
- [Historical pins](../physical/pins.json): archived RTL revision `38d6353`, with
  changed old sources preserved under `evidence/historical/rtl`; no pin rewriting
  or broad source-hash exemption.

[Current interface contract](RTL_SUPPORTED_CONTRACT.md) fixes supported profiles,
byte enables, admission, error/status ownership, page/alignment and reset rules.
[Parameter scope](RTL_PARAMETER_SCOPE.md) lists unqualified combinations.
Historical Synopsys/OSU block timing/power and historical Nangate45 results are
not measurements of the new routed candidate.

## Reproduce

```bash
python3 scripts/run_ci_rtl.py
python3 scripts/run_rtl_closure_stress.py
python3 sim/run_cdc_regression.py --simulator iverilog
python3 scripts/check_publication.py
```

For new physical experiments use the [latest reproduction entry](ROUTED_RESULTS.md#reproduction),
not the archived qualified6 export example. The existing Gauss exporter remains
a pre-layout helper; it does not export or accept the new routed package.
