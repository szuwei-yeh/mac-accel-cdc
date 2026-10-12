# Qualified RTL parameter scope

The full V3 development candidate is tested with DATA_WIDTH=16, AXI data/address=32, AXI ID=4, MAX_LEN=256, BURST_LEN=16, MAX_OUTSTANDING=8; generic default-depth4 checks also pass. The read master uses four bytes per beat. These are fixed qualified instances, not arbitrary-width certification.

ROB reduced formal uses outstanding2/burst2 and smaller declared widths under documented assumptions. The new deterministic completion simulation uses outstanding8/burst2, full32-bit payload. Neither extends the formal proof to a complete depth8 accelerator.

MAX_BURST_LEN=1 is unsupported: clog2(1)=0 produces invalid [-1:0] beat slices at axi_read_engine_rob.v lines156/164. Power-of-two burst descriptions must not be interpreted as support for every such value. Larger burst/count limits, non-power-of-two ROB depth, minimal ID-width tie-offs, FIFO ADDR_WIDTH below2 and arbitrary AXI/data widths are not qualified. No general parameterization or historical RTL/proof edits were made.

The caller must keep every burst within a4KB page and obey the explicit
[supported interface/reset contract](RTL_SUPPORTED_CONTRACT.md). Final RTL
closure fixes V1/V2 byte enables in addition to F02/F03; zero strobes have no
side effects, and disabled storage bytes are preserved. Arbitrary parameters
and single-domain reset recovery are not qualified.
