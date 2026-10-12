# Supported RTL interface contract

This closure qualifies V1 `mac_accel_axi` at DATA_WIDTH=16, VEC_LEN=4,
AXI data/address=32/8; V2 at DATA_WIDTH=16, AXI data/address/ID=32/32/4,
MAX_LEN=256, BURST_LEN=16; V3 uses the same DMA profile with outstanding8.
V3 default-depth4 structural/legacy checks are additional evidence, not a
new mapped checkpoint. Arbitrary parameter combinations are not certified.

## Software and byte enables

All AXI-Lite channels handshake on rising bus edges. AW and W are independent;
their saved address/data/strobes own one ordered write response. B and R payloads
remain stable while stalled. CSR accesses use aligned offsets 0x00 through 0x14;
the decode uses bits [5:2], so ignored address bits create aliases. Protection
bits are ignored. Unmapped reads return DEADBEEF/OKAY; unmapped writes are no-ops.

All AXI wrappers honor WSTRB. Zero strobes have no register, operand-index, mask,
or start side effects and still receive OKAY. V2/V3 address registers preserve
disabled bytes; LENGTH uses only bytes0/1; CTRL start requires byte0 and bit0.

V1 AIN/BIN target the next element. A write with either operand byte enabled
consumes exactly one index even if that element's update_mask bit is zero.
Selected bytes update only when the mask allows it; disabled bytes retain the
element's prior value. Upper-only/zero-strobe operand writes consume no index.
An enabled MASK byte updates its bits and resets both load indices; irrelevant
MASK bytes do nothing. At VEC_LEN=4 only byte0 is relevant. CTRL uses byte0.
The simplified `mac_accel` interface remains full-word with USE_WSTRB=0;
its new wstrb input is used only when USE_WSTRB=1 (selected by the AXI wrapper).
Legacy named-port users at the default remain valid. Positional instantiations
must account for the added port; external positional integrations were not run.

V1/V2 software allows one job at a time, holds operand/configuration registers
stable from start through observed sticky DONE, and consumes completion before
the next start. BUSY is a synchronized activity indication; its low level alone
does not acknowledge completion, especially around initial CDC latency. They
have no hardware busy-start/invalid-length rejection guarantee. This preserves
the existing one-job contract; overlapping starts were never qualified.
V2 LENGTH must be 1..256 before start. V3 rejects invalid lengths and busy starts
with SLVERR and preserves the active job/status; its BUSY includes admission
through result capture. V3 snapshots addresses/length for the admitted job.

## Read master, results and progress

V2/V3 issue ordinary 32-bit INCR reads with one signed16 operand in each beat's
low half. Source addresses must be four-byte aligned. Callers must ensure each
burst stays in its 4KB page and the vector is in valid, nonwrapping address space.
There is no automatic page split or alignment check. Closure exercises a full
burst ending exactly at a page boundary and the next legal burst on the next page.

The memory must obey accepted ID ownership, per-ID beat order, exact ARLEN/RLAST,
and stable R payloads under stalls. V3 accepts OOO/interleaved IDs and holds a
credit until the entry's last output beat retires. Malformed RID/RLAST, extra or
missing beats are outside the legal-slave contract; fault containment for them
is not established. Progress requires eventual ARREADY/responses and running
clocks; permanent external stalls do not constitute a promised completion.

Non-OKAY RRESP sets sticky error for the current job while all legal beats still
drain. CTRL DONE means the final FIFO item has been consumed/accumulated and result
captured in the bus domain, not merely DMA completion. Error clears on the next
start; software must discard error-marked results. The standalone DMA blocks'
dma_done denotes read-phase completion and can precede final skid acceptance;
the pending tail still owns its valid/payload until accepted, including across
a subsequent idle start. Standalone callers must retain the old job's error
status before starting another job and observe stream-last handshakes separately. V1 has no DMA-error status.
Signed products/sums return the low32 bits (two's-complement modulo 2^32);
overflow saturation and overflow status are not implemented. LATENCY is a
32-bit MAC-cycle counter, also modulo 2^32; counter-wrap stress was not run.

## CDC and reset

Both clocks run. Start/done toggles alternate in both directions; synchronized
done qualifies capture of a source result/latency held until the next source
start. Software/admission prevents a new start before that capture. FIFO data
uses Gray-pointer qualification, not independently synchronized payload bits.
Functional clock-ratio/phase tests do not provide analog MTBF or RDC signoff.

Reset requires both domains and any AXI peer to be initialized coherently:
assert both resets, hold through clock edges, release each on its own safe edge,
and keep transactions/start idle until both domains are released and settled.
Independent release order at startup is tested. Quiescent coordinated resets
are exercised by directed fixtures. Single-domain reset/recovery, active-job
abort with an external live memory, and stopped-clock recovery are not qualified;
single-domain reset can create false toggle events or corrupt FIFO ownership.
Use coordinated system reset/peer quiescence rather than asserting one side.

MAX_BURST_LEN=1 has an invalid zero-width ROB beat slice and is unsupported.
Non-power-of-two ROB depth/burst, insufficient ID bits, FIFO ADDR_WIDTH<2,
larger counters/bursts and arbitrary widths remain outside the qualified profile.
Reduced formal profiles certify only their stated control properties.

AXI handshake/strobe reference: [Arm IHI0022H](https://developer.arm.com/-/media/Arm%20Developer%20Community/PDF/IHI0022H_amba_axi_protocol_spec.pdf).
