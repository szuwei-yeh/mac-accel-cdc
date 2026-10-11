# Portfolio polish review — 2026-10-11

> Historical pre-publication review snapshot against `726ac9a688a7d45e3a3d2b1ae435e22e203b9c6a`.
> Status statements and milestone names below describe that review, not the
> current repository. For current engineering explanations and hosted results,
> see the [main README](../readme.md), [physical documentation](../physical/README.md)
> and [CI evidence](CI.md). The original review body is preserved.

Reviewed locally on `polish/vector-mac-portfolio`, based on published commit
`726ac9a`. This report records preparation and the final pre-commit review.
No push, remote metadata update or LICENSE addition was performed.

## Changes

- Reorganized the main README into eight sections, with physical results in
  the opening table, quick navigation and preserved legacy heading anchors.
- Copied the four supplied P9 PNGs byte-for-byte into `physical/figures/`.
  Captions distinguish routed GDS metal/vias from final ODB cell footprints
  and power-only shapes. Original figures and provenance remain intact.
- Expanded the physical README while retaining its measurements, evidence
  links, reproduction requirements and verification limits.
- Prepared [GitHub description/topics](GITHUB_METADATA.md) and a
  [lightweight workflow](../.github/workflows/rtl-ci.yml), with
  [local commands and CI coverage](CI.md).
- Replaced one personal server working-directory path in the synthesis README
  with a repository-relative instruction. Reproduction scripts are unchanged.

## Executed validation

| Check | Actual result |
|---|---|
| Icarus Verilog 12.0 fast RTL suite | Full V3 top 7/7; OOO ROB 12/12; DMA wrapper 11/11; CPU-fed MAC/CDC 8/8 — 38 PASS, 0 FAIL |
| Icarus supplemental V3 CDC matrix | Six scenarios, six jobs each — 36/36 PASS |
| Acceptance/runner unit tests | 13/13 PASS using synthetic fixtures; no archive dependencies |
| Python syntax | All 37 published/new Python files compile; EDA scripts were not executed |
| Documentation integrity | All inline local links/fragments resolve; code fences and PNG headers valid |
| Render review | Main and physical READMEs rendered locally in Chrome with GitHub-style Markdown/CSS; all seven image references loaded; desktop and 390 px mobile layouts checked |
| Workflow | YAML parsed; `actionlint` 1.7.12 returned zero diagnostics (external ShellCheck disabled) |
| Preservation | All 9 functional RTL modules and all pinned source/template hashes unchanged; 142 of 145 previously published files remain byte-identical |
| Physical evidence | Public metrics, summaries, pins, original figures, acceptance/reproduction scripts and testbenches unchanged; archived final GDS/ODB hashes match published records |
| New figures | All four copied PNG hashes/sizes match supplied originals; recorded in [P9 provenance](../physical/figures/p9_provenance.json) |
| Privacy/diff | Basic personal-path and credential-pattern scan clean; `git diff --check` clean |

Only three previously published files changed: the main, physical and synthesis
READMEs. Logs, HTML previews and screenshots stay in local temporary storage.
The GitHub-hosted workflow has **not** executed; its first hosted run remains
pending publication. Action release pins were verified against the
official [checkout](https://github.com/actions/checkout) and
[setup-python](https://github.com/actions/setup-python) repositories.

## Numerical support and scope

| Claim retained / surfaced | Supporting published evidence |
|---|---|
| 4.0× simulated MAC-feed utilization | [256/3777 vs. 256/941 accepted pairs per DMA-busy cycle](../sim/README.md#utilization-measurement) |
| 26.36% synthesized critical-path reduction | [5.50 → 4.05 ns block QoR](../syn/README.md#first-synthesis-driven-optimization) |
| 53.59% SAIF-driven pre-layout dynamic power reduction | [76.1114 → 35.3225 mW active workload](../syn/README.md#targeted-buf_a-clock-gating-experiment) |
| 29,175 standard cells; 40.552% utilization; bus 10 ns / MAC 7.5 ns; positive extracted setup/hold slack; zero supported DRC violations | [Physical measured results](../physical/RESULTS.md), [machine-readable summary](../physical/summary.json) |

OSU pre-layout block benchmarks and Nangate45 full-top results remain separate
experiments. No matched-process comparison or new performance measurement is
claimed. All existing numerical claims and measurement tables were retained.

Qualified LVS remains **unsupported**; physical timing is **single-corner**;
native LEF antenna checks remain **inconclusive**; fresh portable RTL-to-GDS
replay remains **untested**. No synthesis, placement, CTS, routing, extraction,
or archived physical acceptance was rerun in this polish pass.

## Final independent pre-commit review

The complete proposed 16-file publication was reviewed against a refreshed
`origin/main` at `726ac9a688a7d45e3a3d2b1ae435e22e203b9c6a`. The branch began
at that exact commit, with no intervening commits; one polish commit can be
published as a fast-forward if remote main remains an ancestor.

All workflow check commands passed again in an isolated checkout containing
only publication files, using Python 3.12.7 with third-party site packages
disabled and Icarus 12.0: 38 RTL cases, 36 CDC jobs, 13 unit tests, 37 Python
syntax checks, 96 local links and workflow lint. This was a Mac execution,
not an Ubuntu or GitHub-hosted run. The desktop/mobile Markdown render was
reviewed again and all seven image references loaded.

The pinned actions use Node 24. The
[Ubuntu 24.04 runner image](https://github.com/actions/runner-images/blob/main/images/ubuntu/Ubuntu2404-Readme.md)
provides the corresponding runtime and Python 3.12 cache;
[Ubuntu's Icarus package](https://packages.ubuntu.com/en/noble/electronics/iverilog)
is available for the workflow's APT installation. Steps execute from the
checkout root, use only standard-library Python at runtime, and place simulator
outputs outside the checkout. No compatibility blocker was found; actual hosted
execution, APT/network availability and repository Actions permissions remain
unexercised.

All 21 historical numeric table rows in the main README were preserved verbatim.
Original evidence/reproduction link destinations remain present; the physical
overview image reference is the intended replacement. P9 copies and accepted
GDS/ODB hashes were independently rechecked against the supplied archive.
A publication-file credential/personal-path scan found no matches. Ignored
private notes, simulation/synthesis/formal outputs and Python caches are excluded.
Only the three README files differ among the 145 previously published files;
all RTL, accepted evidence, original figures and reproduction scripts remain
byte-identical. The staged diff has no whitespace errors or unintended files.
