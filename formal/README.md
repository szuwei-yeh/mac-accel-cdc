# Formal Verification

## Formal Verification Overview

SymbiYosys checks selected control-safety properties in the single-outstanding
DMA, asynchronous FIFO and ROB read engine. Assertions reside in each module's
`FORMAL` block; the configurations below select the engines, bounds and reduced
parameters. These checks complement [RTL simulation](../sim/README.md) and
[dual-clock CDC regression](../sim/CDC_REGRESSION.md).

The recorded implementation validation reports **seven selected formal tasks
PASS**, under the stated configurations and assumptions. This is scoped
property verification; full V3 depth-8 formal equivalence is not established.

## Verified Modules and Properties

| Module / configuration | Checked properties |
|---|---|
| [`mac_dma`](../rtl/mac_dma.v) / [`mac_dma_bp.sby`](mac_dma_bp.sby) | Legal FSM state; stream VALID and `{a,b,last}` stability during stalls; ARVALID state/address/burst invariants; ARVALID, ARADDR and ARLEN stability under AR backpressure |
| [`mac_fifo_async`](../rtl/mac_fifo_async.v) / [`mac_fifo_gray.sby`](mac_fifo_gray.sby) | Binary-to-Gray encoding in both domains; at most one bit changes between current and next Gray pointers; no write-pointer advance while full; no read-pointer advance while empty |
| [`axi_read_engine_rob`](../rtl/axi_read_engine_rob.v) / [`axi_read_engine_rob.sby`](axi_read_engine_rob.sby) | Occupancy/state bounds; independent command/AR address and burst-geometry reference model; issued-beat conservation; tag ownership; completion before retirement; receive/drain bounds; in-order output; output/AR stability; done counts; sticky response-error reporting |

The ROB's AR reference model observes command acceptance and AR handshakes,
without using the DUT's `issue_elem`, `next_beats` or `current_ar_addr_r`.
Its separate ghost element indices track entry order and expected response data.
Credits remain owned until the entry's final ordered output beat drains.

ROB cover statements target two issued requests, out-of-order filling, output
backpressure, completion after out-of-order filling, AR backpressure, the final
AR handshake and acceptance of a consecutive command on done. Reachability
covers demonstrate possible traces; they do not prove eventual completion.

## Assumptions and Environmental Constraints

`assume` restricts the environment explored by the solver; `assert` specifies
an obligation of the DUT. A PASS applies only within those restrictions.

| Configuration | Environment and parameter scope |
|---|---|
| DMA | Reset asserted before the first sampled clock; `length == 16'd2`; downstream `stream_ready` remains unconstrained, allowing arbitrary stall patterns. Other unassumed inputs remain free. |
| FIFO | `ADDR_WIDTH=2`, `WIDTH=4`, `multiclock on`; both resets held until the first write-clock edge. Independent clock inputs are explored in the digital multiclock model. |
| ROB | `MAX_OUTSTANDING=2`, `MAX_BURST_LEN=2`, `LEN_WIDTH=8`, `AXI_DATA_W=8`, `AXI_ADDR_W=12`, `AXI_ID_WIDTH=2`; reset initially asserted; command length constrained to 1–6. |

The ROB response assumptions permit beats only for allocated, incomplete
entries with remaining receive capacity; require per-ID order and RLAST on the
expected final beat; and constrain the modeled data's element-index field to
`g_base[f_rid] + e_recv[f_rid]`. Entry lookup uses the low RID bits through
`f_rid`. Different entries can fill out of order, but illegal response traces
excluded by these assumptions are outside the proof. This is an abstract
response environment, not exhaustive verification of an AXI slave or all AXI
protocol rules.

No fairness assumption requires READY to assert or memory responses to arrive.
Safety under stalls does not imply progress under an indefinitely stalled
consumer or memory. The small ROB length bound exceeds one complete ROB fill,
allowing entry/tag reuse; it does not cover arbitrary production lengths.

## BMC, Induction and IC3/PDR Methodology

**Bounded model checking (BMC)** searches traces from the assumed initial state
through the configured depth. PASS means no assertion violation within that
bound; it says nothing about later cycles.

**k-induction** uses a base case from the initial state and an induction step
from arbitrary states satisfying the induction hypothesis. Both obligations
must pass for an unbounded safety proof in the selected model. Unreachable
states can produce spurious induction counterexamples; legal-state and
AR-payload binding invariants strengthen the hypothesis. The configured DMA
and FIFO induction depth is 12.

**IC3/PDR** is selected for the ROB prove task using `abc pdr`. It discovers
inductive invariants rather than using a manually selected k-induction depth.
The BMC and cover tasks use `smtbmc boolector`; ROB arrays are mapped to logic
by `memory_map` before solving.

Temporal DMA and ROB assertions guard `$past` with `f_past_valid` and reset
conditions so the first sample and reset transitions do not use undefined
history. For example, the DMA's stream-stability assertion is:

```verilog
always @(posedge clk)
  if (f_past_valid && !rst && !$past(rst)
      && $past(stream_valid) && !$past(stream_ready)) begin
      a_valid_stable : assert (stream_valid);
      a_dataA_stable : assert (stream_a    == $past(stream_a));
      a_dataB_stable : assert (stream_b    == $past(stream_b));
      a_last_stable  : assert (stream_last == $past(stream_last));
  end
```

A stalled offered beat must remain valid with the same payload on the next
non-reset cycle. The DMA's skid register clears VALID on acceptance, while
`M_AXI_RREADY` prevents a new B beat from overwriting an unaccepted pair.
This property checks retention under backpressure, not complete MAC arithmetic
correctness or every stream-interface rule.

## Proof Results and Exact Scope

| Configuration | Recorded task results | Engine / depth |
|---|---|---|
| `mac_dma_bp.sby` | BMC PASS; prove PASS | `smtbmc boolector`; BMC 40, induction 12 |
| `mac_fifo_gray.sby` | BMC PASS; prove PASS | `smtbmc boolector`; BMC 30, induction 12; multiclock |
| `axi_read_engine_rob.sby` | BMC PASS; prove PASS; cover PASS | BMC `smtbmc boolector` depth 14; prove `abc pdr`; cover `smtbmc boolector` depth 30 |

The accepted [implementation methodology](../physical/METHODOLOGY.md#functional-and-physical-checks)
and [machine-readable summary](../physical/summary.json) record the seven-task
result. The unchanged [formal runner](../physical/templates/p2/verify_formal.py)
selects exactly these tasks and records status, exit code and configuration in
archived `audit/formal-results.json`. The
[accepted checkpoint identifiers](../physical/PROVENANCE.md#exact-accepted-checkpoints)
locate that archive; detailed solver logs are not included in the public checkout.
These are recorded results, not a new solver execution for this documentation.

## Reproduction Commands

Install SymbiYosys, Yosys, Boolector and ABC, for example through OSS CAD Suite.
The accepted flow's tool revisions are recorded in [physical/pins.json](../physical/pins.json).
With those executables on PATH, run from the repository root:

```bash
cd formal
sby -f mac_dma_bp.sby bmc
sby -f mac_dma_bp.sby prove
sby -f mac_fifo_gray.sby bmc
sby -f mac_fifo_gray.sby prove
sby -f axi_read_engine_rob.sby bmc
sby -f axi_read_engine_rob.sby prove
sby -f axi_read_engine_rob.sby cover
```

Omitting a task name runs all tasks in that configuration. Inspect each task's
`status` file and engine log; a failed assertion may produce a counterexample
VCD under `engine_0/` for waveform analysis. Generated task directories are
ignored by [formal/.gitignore](.gitignore). The archived implementation runner
also records these statuses separately from the depth-8 RTL regressions.

## Known Limitations

- Unbounded prove results concern only the selected assertions, reduced
  parameters and assumed environment. They do not establish correctness for
  every parameter setting, production vector length or full accelerator.
- The FIFO properties check digital pointer/control invariants. They do not
  prove payload conservation, analog metastability/MTBF, reset-domain-crossing
  signoff or physical synchronizer timing. Separate simulation and physical
  CDC audits provide their own documented evidence.
- Liveness, complete dot-product functional correctness, whole-top formal
  equivalence and complete AXI compliance are outside these proofs.
- [Lightweight CI](../docs/CI.md) does not execute formal solvers. A CI PASS is
  not a new proof result.

Historical source snapshots are retained byte-for-byte for implementation
provenance, with hashes recorded in [physical/pins.json](../physical/pins.json).
