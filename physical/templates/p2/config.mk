export DESIGN_NAME = mac_accel_dma_rob_top
export PLATFORM = nangate45
export VERILOG_FILES = @SOURCE@/rtl/mac_accel_dma_rob_top.v @SOURCE@/rtl/mac_dma_rob.v @SOURCE@/rtl/axi_read_engine_rob.v @SOURCE@/rtl/mac_fifo_async.v @SOURCE@/rtl/mac_pe.v
export VERILOG_TOP_PARAMS = DATA_WIDTH 16 AXI_DATA_WIDTH 32 AXI_ADDR_WIDTH 32 AXI_ID_WIDTH 4 S_AXI_ADDR_WIDTH 8 MAX_LEN 256 BURST_LEN 16 MAX_OUTSTANDING 8
export SDC_FILE = @P2_PHASE@/constraint.sdc
export SYNTH_REPEATABLE_BUILD = 1
export SYNTH_CANONICALIZE_TCL = @P2_PHASE@/canonicalize.tcl
export ABC_CLOCK_PERIOD_IN_PS = 7500
