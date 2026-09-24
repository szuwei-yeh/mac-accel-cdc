# User-approved portfolio analysis assumptions; units must be ns.
# Separate from the canonical mac_dma_rob benchmark.
namespace eval wtop {
    variable top mac_accel_dma_rob_top
    variable parameters [dict create DATA_WIDTH 16 AXI_DATA_WIDTH 32 \
        AXI_ADDR_WIDTH 32 AXI_ID_WIDTH 4 S_AXI_ADDR_WIDTH 8 \
        MAX_LEN 256 BURST_LEN 16 MAX_OUTSTANDING 8]
    variable bus_period 10.0
    variable mac_period 7.5
    variable io_min 0.0
    variable io_max 1.0
    variable max_fanout 32
    # Additional conservative CDC analysis budgets, not interface specs.
    variable gray_budget 7.5
    variable payload_budget 7.5
    variable result_budget 10.0
    variable bus_inputs {
        S_AXI_AWADDR 8 S_AXI_AWPROT 3 S_AXI_AWVALID 1
        S_AXI_WDATA 32 S_AXI_WSTRB 4 S_AXI_WVALID 1 S_AXI_BREADY 1
        S_AXI_ARADDR 8 S_AXI_ARPROT 3 S_AXI_ARVALID 1 S_AXI_RREADY 1
        M_AXI_ARREADY 1 M_AXI_RID 4 M_AXI_RDATA 32 M_AXI_RRESP 2
        M_AXI_RLAST 1 M_AXI_RVALID 1
    }
    variable bus_outputs {
        S_AXI_AWREADY 1 S_AXI_WREADY 1 S_AXI_BRESP 2 S_AXI_BVALID 1
        S_AXI_ARREADY 1 S_AXI_RDATA 32 S_AXI_RRESP 2 S_AXI_RVALID 1
        M_AXI_ARID 4 M_AXI_ARADDR 32 M_AXI_ARLEN 8 M_AXI_ARSIZE 3
        M_AXI_ARBURST 2 M_AXI_ARVALID 1 M_AXI_RREADY 1 done_led 1
    }
}
