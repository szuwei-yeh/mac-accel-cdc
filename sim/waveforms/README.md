# Authentic ROB waveform

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
