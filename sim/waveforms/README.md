# Authentic ROB waveforms

## Primary depth-8 directed test

The main README uses a **Standalone ROB test, depth 8**: eight requests of
16 beats each, **128/128 correct output beats**. This directed case deliberately
adds 200 memory-latency cycles for Entry 0, while later entries use the existing
100-cycle base latency and OOO memory arbiter. It demonstrates read-burst
completion and ordered entry retirement, not MAC computation, a new performance
benchmark, or new full-DMA/CDC/system qualification.

Requires Python 3.9+ and Icarus Verilog (`iverilog` and `vvp`; recorded with
Icarus 12.0). From the repository root:

```bash
python3 sim/waveforms/capture_rob_depth8.py --output /tmp/vector-mac-rob-depth8
```

Use a **new directory outside the repository**, or omit `--output` for a new
temporary directory. No Downloads path or external bundle is required. The
runner generates the dedicated testbench, simulator binary, VCD, log, CSVs,
`trace_summary.json` and portable viewer files only in that output directory.
It requires **1 PASS / 0 FAIL** from the reused scoreboard and
**DEPTH8_WAVEFORM_PASS** from an independent VCD audit. This one supplemental
case does not replace or add to the frozen regression's published case counts.

### Test construction and checks

[The runner](capture_rob_depth8.py) reads the unchanged
[standalone regression](../tb_axi_read_engine_rob_ooo.v), checks its recorded
source hash, and reuses its checker/task definitions. In the generated copy only,
it changes the model latency connection to delay ARID 0 and substitutes
[one directed stimulus](depth8/stimulus.vh): `N=128`, `ooo_en=1`, no AR/output
backpressure, 10 ns clock, model reset LFSR `16'hACE1`. `-DENG_MAX_OUT=8` selects
eight slots in both DUT and memory model. The production RTL, original model,
original regression, and depth-4 wrapper/capture script remain unchanged.

[The depth-8 wrapper](depth8/tb_axi_read_engine_rob_waveform.v) is compiled in
isolation instead of the original depth-4 wrapper, using the same observation
top name to retain trace/viewer compatibility. Its packed `e_complete[7:0]` wire
concatenates the DUT's unpacked completion bits without feedback to the DUT.
The audit checks this alias against all eight DUT bits at every sampled rising
edge. In Surfer, its display name is `e_complete`, binary **[7:0]**; the
**rightmost bit is Entry 0**.

The audit uses **pre-edge handshake/payload values**, then checks settled
post-edge state. It verifies all accepted ARIDs, addresses and burst lengths,
RID-indexed data/storage, RRESP/RLAST and per-ID beat counts. On every output
handshake it checks data `1..128`, the entry and beat index, completion/valid
ownership, and command-level `out_last`. Each entry contributes exactly
16 consecutive beats, with no missing, duplicated or interleaved output beats.
It also checks the actual DUT `entry_drained` definition, each entry's `e_valid`
lifetime, head advancement, and occupancy/credit conservation. None of these
checks relies solely on a displayed pointer or annotation.

### Recorded events and definitions

All eight requests allocate in order, IDs **0–7 at 145–215 ns**, before the first
burst completes. Peak outstanding is **8**. The measured completion order is
**1 → 2 → 3 → 4 → 5 → 6 → 7 → 0**; retirement is
**0 → 1 → 2 → 3 → 4 → 5 → 6 → 7**. The runner compares its independently measured
events against the [recorded capture](depth8/provenance.json); that record does
not drive simulation signals or inject event times.

| Entry | Burst completion (ns) | First output handshake (ns) | Final output handshake / retirement (ns) |
|---|---:|---:|---:|
| 0 | 3305 | 3315 | 3465 |
| 1 | 1855 | 3475 | 3625 |
| 2 | 1925 | 3635 | 3785 |
| 3 | 2035 | 3795 | 3945 |
| 4 | 2115 | 3955 | 4105 |
| 5 | 2195 | 4115 | 4265 |
| 6 | 2235 | 4275 | 4425 |
| 7 | 2275 | 4435 | 4585 |

At **2276 ns**, `head_ptr=0`, `e_complete[7:0]=11111110`, `out_valid=0`,
`out_ready_tb=1`, and `occupancy=8`. Entries 1–7 have received complete read
bursts; Entry 0 is incomplete, so all eight entries remain allocated and no
output is available. AXI responses for the completed bursts are already done;
occupancy measures ROB ownership, not eight still-active AXI responses here.

- **Completion:** a rising edge accepts `RVALID && RREADY && RLAST`; the DUT's
  entry completion flag sets after that edge. This is not MAC completion.
- **First output:** the first beat of an entry successfully handshakes at a
  rising edge with `out_valid && out_ready`. Entry 0 completes at 3305 ns;
  its first accepted output beat is at 3315 ns.
- **Retirement:** the rising edge accepts the entry's **last** output beat.
  The actual RTL event is `entry_drained = out_acc &&
  (drain_idx == e_exp[head_ptr] - 1)`, where `out_acc = out_valid && out_ready`.
  Only then does `e_valid` clear and the credit return. At 4585 ns, Entry 7
  retires, `head_ptr` wraps **7 → 0**, and occupancy becomes **0**.

`out_last` marks only the entire 128-beat command's final beat. Completion flags
retain their old values after retirement until allocation/command reset; they
are not entry-ownership flags.

### Native Surfer view and provenance

[The depth-8 README image](../../docs/figures/rob_ooo_surfer_depth8.png) is a crop
of the verified run's **real Surfer 0.7.0 (git `1cdbaa9`) window**, not a plotted
waveform or a presentation slide. It retains signal names, cursor values, the ns
axis, all eight retirements and head wrap. The original 1800 × 1476 source PNG
is cropped at `[0, 106, 1800, 710]` to 1800 × 604 pixels. All retained pixels are
unchanged; clicking the README image opens that full-resolution crop.

To open a newly reproduced VCD with the matching settings, use Surfer 0.7.0:

```bash
wave_dir=/tmp/vector-mac-rob-depth8
cd "$wave_dir"
surfer rob_depth8.vcd --state-file rob_depth8_surfer.ron --command-file overview.sucl
# For the blocked-head detail instead:
surfer rob_depth8.vcd --state-file rob_depth8_surfer.ron --command-file blocked.sucl
```

The runner copies the [state](depth8/rob_depth8_surfer.ron),
[overview](depth8/overview.sucl), [detail](depth8/blocked.sucl), and
[local configuration](depth8/config.toml) into the output directory, including
`.surfer/config.toml`. The overview is **2180–4690 ns**, detail **2190–2340 ns**,
cursor **2276 ns**, with binary completion bits and decimal head/occupancy.
Use the full path to a compatible `surfer` executable if it is not on `PATH`.
For a new screenshot, capture this real viewer and preserve the labels/time axis;
do not redraw, splice, or change waveform values.

Original depth-8 screenshot VCD SHA-256:
`864027f294c5ffd1fa0420d07e5fb88f47f71991348dfbbbd08694ea5c7969a7`.
[The separate provenance record](depth8/provenance.json) binds this original
capture, source-PNG/raw-capture/binary hashes, crop, new image, source identities
and measured events. Fresh VCD hashes may differ because of date/source metadata;
the runner checks the measured events and all data/ownership properties instead
of relabeling the original hash. Generated VCDs, binaries, full logs, raw
screenshots and slide decks remain outside the repository.

## Supplemental depth-4 recording

The original image, source files and identity below are preserved. Its discussion
of the original regression's first depth-8 phase refers to the unmodified
64-word workload, not the directed 128-word case above.

Simulation requires Python 3.9+ and Icarus Verilog (`iverilog` and `vvp`);
the recorded run used Icarus 12.0. Inspection requires native Surfer 0.7.0.
In the viewer command below, replace `surfer` with the binary's full path if
it is not on `PATH`.

Run from the repository root:

```bash
python3 sim/waveforms/capture_rob_waveform.py --output /tmp/vector-mac-rob-waveform
```

Use a new output directory, or omit `--output` to create a temporary one. The
script prints the exact compile/run/viewer commands and requires **12 PASS / 0
FAIL** from the original scoreboard plus **WAVEFORM_PASS** from a VCD audit.
Generated VCD, simulator binary, log and `trace_summary.json` remain outside Git.
The JSON records source/VCD hashes, configuration and measured event times.

[The observability wrapper](tb_axi_read_engine_rob_waveform.v) instantiates the
unchanged [standalone regression](../tb_axi_read_engine_rob_ooo.v), original DUT and
original OOO model. Only `+ROB_VCD=<path>` enables dumping; the dump stops after
the first phase, while all 12 original cases still run. No stimulus, production
RTL or sealed qualification input is changed. Icarus's warnings about explicitly
dumped array words are expected; the audit requires those words to be present.

The captured phase selects **depth4**, 64 words, four 16-beat bursts, a 10ns
clock, 100-cycle model latency, `ooo_en=1`, and no output/AR backpressure. The
model's reset LFSR seed is `16'hACE1`; this phase contains no `$random` stimulus.
Depth4 gives a clear non-head completion trace; the first depth8 phase interleaves
responses but completes its four bursts in order. The qualified full accelerator
profile remains depth8.

Observed with Icarus 12.0; times identify rising-edge handshakes:

| Event | Actual order and time (ns) |
|---|---|
| Accepted ARID | 0 @ 145, 1 @ 155, 2 @ 165, 3 @ 175 |
| Accepted RLAST / RID | 2 @ 1655, 1 @ 1665, 3 @ 1725, 0 @ 1785 |
| Last output beat of each ROB entry | 0 @ 1945, 1 @ 2105, 2 @ 2265, 3 @ 2425 |

At 1655ns, `RID=2` sets `dut.e_complete[2]` while `dut.head_ptr=0`,
`dut.e_complete[0]=0`, and `out_valid=0`. Entries 1 and 3 also complete before
head0. Once head0 completes at 1785ns, accepted `out_data` runs from **1 through
64**, starting at 1795ns; `out_last` marks only word64. There are **46 RID-change
events between consecutive accepted R beats**. The audit samples pre-edge
handshake values and checks settled post-edge writes to `dut.rob_data`,
`dut.e_recv`, completion flags, head advancement and occupancy. It also checks
RRESP/RLAST, per-ID response data, output data/last and channel stability.

### Surfer screenshot and inspection

[The authentic PNG](../../docs/figures/rob_ooo_surfer_standalone.png) was captured
from the existing verified VCD in native **Surfer 0.7.0 (git `1cdbaa9`)** on macOS.
No simulation was rerun to make the PNG. The screenshot is a region of the
actual Surfer window; only its title/share indicator and unused space below the
traces were cropped. Waveform pixels, data and time axes were not edited.
The approved PNG remains at its original **2940 × 1034 pixels**; click the
README image to open that full-resolution file.

The four color-coded sections are **AXI Request**, **AXI Response**, **ROB State**,
and **Ordered Retirement**. The latter heading spans two native divider rows.
Bus IDs, pointers, occupancy and output data use unsigned decimal; completion
flags use binary. Signal labels omit bus-width suffixes for readability. Surfer
coalesces entries 3,2,1 into a three-bit `e_complete` bus, ordered **[3:1]**;
`e_complete[0]` remains separate. At the red **1656ns** cursor, the coalesced bus
is **010** (entry2 complete), head0's flag is **0**, `head_ptr=0`, and
`out_valid=0`. The column values describe this cursor, including for the earlier
left pane. The two panes show **120–195ns** (issue) and **1600–2440ns** (completion
and retirement) from the same trace; the vertical divider is a time-window break.

Open an existing verified dump without regenerating it:

```bash
# Run from the repository root; substitute your existing waveform directory.
wave_dir=/tmp/vector-mac-rob-waveform
surfer "$wave_dir/rob_ooo.vcd" \
  --state-file sim/waveforms/rob_ooo_surfer.ron \
  --command-file sim/waveforms/rob_ooo_surfer.sucl
```

[The portable state file](rob_ooo_surfer.ron) restores the actual signal
references, dividers, radix and cursor. [The startup commands](rob_ooo_surfer.sucl)
add the second pane with `viewport_add`, `viewport_set_active 1` and
`zoom_to 1600ns 2440ns`. The state contains one initial pane to avoid the
Surfer 0.7.0 startup cache issue encountered with a multi-pane state alone.
The original [GTKWave configuration](rob_ooo.gtkw) remains available.

To capture the view manually on macOS:

1. Open the verified VCD with the Surfer command above. Wait until both panes
   have loaded; confirm their nanosecond axes and the **010 / 0 / head0 / invalid
   output** cursor state described above.
2. Keep all signal labels, the time axes and `out_last` visible. The output bus
   is dense at this overview zoom; inspect individual beats by selecting the
   right pane, pressing Space and running `zoom_to 1780ns 1980ns`. Restore the
   overview with `zoom_to 1600ns 2440ns` before capturing.
3. Press **Shift–Command–4**, then drag a rectangle around the menu, time axes,
   signal labels and all traces. Exclude the title/share indicator and blank
   space below the traces. Keep the real viewer pixels; do not redraw or splice
   waveform segments.
4. Save the resulting PNG as `docs/figures/rob_ooo_surfer_standalone.png`, and
   compare the completion flags and head transitions with the measured event
   table above before replacing the existing capture.

Screenshot input VCD SHA-256:
`fa519e3a3c51406ab012d08af9e08e8c02f813665afc3ea50e45583cc51ca5a1`.
The full-window capture and its crop rectangle, input/config/PNG hashes and
read-only trace audit are retained outside the repository with the original
simulation log and `trace_summary.json`. These hashes identify this exact dump;
a fresh simulation may have a different VCD date header while preserving all
sampled events.

Viewer documentation: [Surfer commands](https://docs.surfer-project.org/book/commands/).
The supplemental waveform evidence does not extend the frozen implementation's
system, physical or signoff qualification.
