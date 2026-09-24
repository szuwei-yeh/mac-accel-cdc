# Whole-top CDC supplemental regression

```sh
python3 sim/run_cdc_regression.py --simulator iverilog
# Requires the configured UCSB VCS installation and a valid license:
python3 sim/run_cdc_regression.py --simulator vcs
```

The runner uses Python 3.6+, Icarus (`iverilog`/`vvp` on PATH), or
`/ece/synopsys/vcs/V-2023.12-SP2/bin/vcs`. Each invocation snapshots sources,
compiles once, and writes logs, command metadata, hashes, result JSON and event
CSV files in a new `/tmp/vectormac-cdc-supp-*` directory. Save desired reports
before temporary storage is cleared. The runner does not overwrite the project.

The DUT is `mac_accel_dma_rob_top` with `MAX_OUTSTANDING=8`. Each process runs
lengths **1, 17, 129, 256, 33, 1**, with one startup reset and **zero inter-job
resets**. Operand patterns vary by job/index and include signed values. Bus
period is 10 ns; MAC phase below is the delay before its clock generator starts.

| Scenario | MAC period (ns) | Phase (ns) | Startup reset release order |
|---|---:|---:|---|
| reference | 7.500 | 0.000 | MAC then bus |
| mac_slower | 17.000 | 1.250 | MAC then bus |
| equal_aligned | 10.000 | 0.000 | MAC then bus |
| equal_shifted | 10.000 | 2.500 | bus then MAC |
| near_edges | 7.502 | 1.250 | MAC then bus |
| mac_faster | 3.500 | 0.375 | bus then MAC |

The regression checks FIFO payload/order/last, MAC and CSR results, Gray-pointer
transitions, stream/AR stability under backpressure, AR/R beat conservation,
ID ownership, one start/done event per job, alternating toggles and absence of
stale completion. Lengths 129 and 256 must actually reach eight simultaneous
outstanding bursts. Transaction and process timeouts prevent silent hangs.

Recorded result: **36/36 jobs on Icarus 12.0 and 36/36 on VCS V-2023.12-SP2**.
Source/runner hashes, job statistics and start/done event CSVs agreed between
simulators. A deliberately corrupted expected-result control failed as expected
during testbench review. The six-scenario Icarus matrix was also rerun before
publication and passed.

This is digital functional verification, not a metastability model or CDC/RDC
signoff. Near-edge scheduling does not model analog behavior. Reset is asserted
at startup and released on each domain's negedge; mid-job reset, single-domain
reset recovery and stopped clocks are not covered. The software contract allows
one job at a time and requires consuming completion before issuing a new job.
The AXI-Lite driver uses full-word writes; complete AXI protocol coverage is not
claimed. Existing regressions and RTL remain unchanged.
