# Whole-top synthesis and standalone timing analysis

Status: DC mapping and independent mapped readback completed; standalone
PrimeTime setup/hold analysis completed on the same netlist and resolved SDC.
See [results](RESULTS.md), [DC instructions](DC_FLOW.md), and
[PrimeTime instructions](PT_FLOW.md). This flow is separate from the canonical
`mac_dma_rob` timing/power benchmark. CDC/RDC signoff and physical timing closure
are not claimed.

## Profile and scope

The following portfolio analysis assumptions were used on 2026-09-24. They are
not board-level interface requirements or signoff specifications.

| Item | Assumption |
|---|---|
| Top | `mac_accel_dma_rob_top` |
| Parameters | DATA_WIDTH=16, AXI_DATA_WIDTH=32, AXI_ADDR_WIDTH=32, AXI_ID_WIDTH=4, S_AXI_ADDR_WIDTH=8, MAX_LEN=256, BURST_LEN=16, **MAX_OUTSTANDING=8** |
| Bus clock | `S_AXI_ACLK`, clock name `bus_clk`, period 10 ns |
| MAC clock | `mac_clk`, clock name `mac_clk`, period 7.5 ns |
| Relationship | Asynchronous; no valid phase relationship for setup/hold |
| Functional input/output delay | All AXI-Lite/AXI master data/control and `done_led`, bus referenced, min 0 ns / max 1 ns |
| Resets | `S_AXI_ARESETN` active-low; `mac_rst` active-high; excluded from functional data timing |
| Max fanout | 32, a design-rule analysis assumption carried from the block flow |
| Clock model | Ideal, default 50% waveform; no added uncertainty/latency or generated clocks |
| Drive / load | Unspecified; no board input slew or output capacitance model |
| Library | `/fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db` |
| Corner | `osu018_stdcells`, typical, process 1, 1.8 V, 25 C; same available library for min/max |

The DC and PT runs verified the library, corner and ns units above. The SDC
itself does not load a library; the tool launchers do. No extracted RC is
available. Passing this profile does not establish 100/133.33 MHz physical
operation; see the model limitations and coverage exclusions in [RESULTS.md](RESULTS.md).

## CDC inventory and exceptions

Names below are **RTL registers**, not promised mapped cell names. Line numbers
refer to the existing source at HEAD `17b4e36484482289e716c3761396514bdaca1f84`.

| ID | Source → destination | Domains | Policy | Evidence |
|---|---|---|---|---|
| start | `start_toggle_bus` → `start_sync_mac[0]` | bus→MAC | False path to first stage only | top:278–295, toggle + 2FF + XOR |
| done | `done_toggle_mac` → `done_sync_bus[0]` | MAC→bus | False path to first stage only | top:334–351, toggle + 2FF + XOR |
| busy | `u_pe/busy` → `busy_sync_bus[0]` | MAC→bus | False path to first stage only | top:323–329, 2FF level |
| write_gray | `u_fifo/wr_ptr_gray[4:0]` → `u_fifo/wr_ptr_gray_sync1[4:0]` | bus→MAC | Max 7.5 ns, cross-domain hold excluded | FIFO:35–48,107–118 |
| read_gray | `u_fifo/rd_ptr_gray[4:0]` → `u_fifo/rd_ptr_gray_sync1[4:0]` | MAC→bus | Max 7.5 ns, cross-domain hold excluded | FIFO:35–48,67–74 |
| payload | `u_fifo/mem[0:15][32:0]` → `u_pe/{a_in_r[15:0],b_in_r[15:0],last_in_r}` | bus→MAC | Max 7.5 ns, cross-domain hold excluded | FIFO:54–63,104; PE:53–74 |
| result | `u_pe/result[31:0]` → `res_bus_reg[31:0]` | MAC→bus | Max 10 ns, cross-domain hold excluded | PE:128–133; top:334–375 |
| latency | `u_pe/last_latency[31:0]` → `lat_bus_reg[31:0]` | MAC→bus | Max 10 ns, cross-domain hold excluded | Same completion/capture logic |

Budget rationale (engineering assumptions, not measured delays):

- Gray paths use the smaller clock period, 7.5 ns, no larger than either
  source update period. This limits per-bit flight time conservatively. The
  physical implementation would still need skew, synchronizer placement and
  metastability review; digital Gray assertions alone cannot establish them.
- FIFO payload has a one-MAC-period budget. A newly written location becomes
  readable after the write pointer traverses two MAC synchronizer stages;
  accepted reads capture held memory data. Full/empty ownership must prevent
  overwrite of an unread location. This is a protocol-dependent bound, not a
  synchronous edge relationship between the two clocks.
- Result/latency have a one-bus-period budget. PE stores data and asserts done;
  the source toggle updates on the following MAC edge, then crosses two bus
  synchronizer stages before bus capture. Data is held across this interval.
  Source result clears on a later start, so software must wait for completion
  and consume result before issuing the next job.

All bounded paths use `set_max_delay -ignore_clock_latency`, from **sequential
cells** to sequential cells. This includes source clock-to-Q, data delay and
destination setup, rather than being a pure net-delay or skew constraint.
The exact source/destination pairs receive `set_false_path -hold`; same-domain
hold checks remain. Do not replace this with unsupported `-datapath_only`.

`set_clock_groups -asynchronous -allow_paths` records the relationship while
leaving paths available for explicit checks. Removing `-allow_paths` would
suppress the bounded crossings. Unclassified crossings deliberately remain
visible, but their default edge-based slack is **not meaningful asynchronous
timing**: any such crossing blocks acceptance until classified.

Keep these paths normally timed:

- Every synchronizer stage 1→stage 2, and stage 2→pulse/control logic.
- MAC read pointer→FIFO read mux→PE input registers. Only paths launched from
  the bus-domain memory cells get the payload exception.
- Bus result capture→CSR output and all ordinary intra-domain logic.

No multicycle exception, broad hierarchy wildcard false path, generated clock,
case analysis or synchronizer `dont_touch` is introduced here. RTL contains no
`ASYNC_REG` attribute; preservation of the 2FF structures must be checked after
mapping. This SDC alone does not guarantee synthesis preserves them.

## Reset and protocol contract

This is functional steady-state timing. `set_false_path -from` each external
reset explicitly excludes both synchronous reset arrival and asynchronous
recovery/removal. Reset ports have no functional I/O delay. No reset case
analysis is applied, to avoid making reset a synthesis constant.

CSR/DMA/ROB include synchronous reset; FIFO/PE/CDC registers use asynchronous
reset. There is no reset-release synchronizer inside the top. Integration must
provide valid assertion duration, running clocks for synchronous reset, and
safe per-domain deassertion. These requirements are **not verified by this
profile**. Startup reset-order simulation is not RDC or recovery/removal proof.
Mid-job reset, single-domain reset and stopped clocks are outside the reviewed
operating contract; do not claim supported recovery.

Use one outstanding **job** (distinct from eight outstanding AXI bursts), keep
job configuration stable during execution, and wait for the new completion
before another start. Top CTRL writes generate a start without a busy guard
(top:174–176); the design does not enforce this software contract. Toggle CDC
is not an arbitrary-rate event queue. The supplemental regression covers six
sequential jobs per scenario under this contract, not unrestricted traffic.

## Binding contract

`portfolio.sdc` is a Tcl-based constraint entry point intended to be `source`d
after link. It intentionally errors before applying constraints if required
metadata/collections are missing. It requires live cell bindings and is **not a standalone mapped SDC**;
DC exports the resolved SDC for PrimeTime.

The DC runner populates:

- `WTOP_ELAB_PARAMS`: a dict recording actual elaboration parameters. Do not
  populate it merely by copying defaults; RTL defaults MAX_OUTSTANDING to 4.
- `WTOP_TIME_UNIT`: `ns`, only after inspecting the loaded library units.
- `WTOP_CDC_CELLS`: a dict with exactly 16 keys: `<ID>.from` and `<ID>.to` for
  each of the eight inventory IDs. Values are Synopsys sequential-cell
  collections resolved from the actual linked design. No pins/nets/hierarchy
  containers and no guessed mapping name fallback.

Expected source/destination cell counts are 1/1 for scalar groups, 5/5 for
each Gray group, 528/33 for payload, and 32/32 for result/latency. These reflect
the unoptimized register structure. Memory inference, register sharing,
replication or optimization can change counts. If so, stop and review the
actual representation and connectivity; do not loosen checks blindly.

The guards check counts/types, top name, parameter metadata, exact port bits
and port direction coverage. **They cannot prove that the caller's metadata or
selected cells are correct.** Before accepting a run:

1. Archive elaboration parameters, source/netlist hashes and exact collection
   names. Verify launch/capture clocks and actual connectivity for all groups.
   For Gray buses verify bit correspondence; for payload verify every memory
   bit's destination, not a nonexistent full Cartesian connectivity matrix.
2. Confirm no extra CDC paths, no unintended synchronizer stage-2 exceptions,
   and no missing/optimized-away synchronizer chains. Check bindings again
   after compile; do not assume precompile names persist.
3. Apply in a fresh design with no old constraints; abort on any tool error.
   Save clocks, I/O delays, exception coverage/ignored exceptions, check_timing
   and analysis coverage. Inspect both directions of cross-domain max/min
   paths and normally timed paths listed above.
4. Distinguish reset exclusions and intended first-stage false paths from
   unintended unconstrained endpoints; do not count exception totals as proof.
5. Export a resolved SDC with `write_sdc` for the matching mapped netlist and
   verify it on re-import before a later standalone PT run.

Use `run_dc.py` for whole-top elaboration/mapping and `run_pt.py` for the
accepted mapped pair. The block launcher remains dedicated to `mac_dma_rob`.
Both whole-top runners preserve input snapshots and create separate outputs.

## Version-specific command references

Syntax/semantics were checked in the installed primary documentation:

- DC R-2020.09-SP4: `/ece/synopsys/syn/R-2020.09-SP4/doc/syn/man/cat2/`
- PT R-2020.09-SP5-1: `/ece/synopsys/prime/R-2020.09-SP5-1/doc/pt/man/cat2/`

Relevant files: `set_clock_groups.2`, `set_max_delay.2`, `set_false_path.2`.
Both document `-allow_paths`, `-ignore_clock_latency` and hold-only false
paths. Manual review and Tcl checks are not a substitute for execution on the
real elaborated design. No vendor manual content is copied into this project.
