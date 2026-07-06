# Synopsys DC Synthesis Flow

This directory contains a small Design Compiler flow for sanity-checking the
ROB/outstanding-read RTL with the UCSB ECE OSU standard-cell libraries.

Default target library:

```text
/fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db
```

Run from the repository root on the ECE server:

```bash
cd /fs/student/szu-wei/work/mac-accel-cdc

bash syn/run_dc.sh
TOP=axi_read_engine_rob bash syn/run_dc.sh
```

Useful overrides:

```bash
TOP=mac_dma_rob CLK_PERIOD=5.0 bash syn/run_dc.sh
TARGET_LIB=/path/to/other.db bash syn/run_dc.sh
```

Reports are written to `syn/reports/`; mapped netlists and SDC files are written
to `syn/mapped/`. Treat the results as synthesis sanity data, not signoff timing.

Use `*_qor.rpt` and `*_timing_constraints.rpt` for timing/design-rule pass/fail.
The `*_all_constraints.rpt` file intentionally includes every DC constraint
class, including power targets that may be unset in this portable flow.
