`timescale 1ns/1ps

// axi_read_mem_model.v
// -----------------------------------------------------------------------------
// Behavioural AXI4 read-only slave (memory model) with a CONFIGURABLE access
// latency.  Test infrastructure only -- NOT synthesized.
//
// This is the Step-0 memory model for the outstanding-transaction / ROB project.
// It models the dominant cost of a real memory: a long access latency to the
// FIRST beat of a burst, after which beats stream back at full bandwidth (one
// per bus_clk cycle, gated by RREADY).  That "L cycles idle, then stream" shape
// is exactly what multiple-outstanding transactions are meant to hide, so it is
// the right stimulus for a baseline-vs-improvement utilization comparison.
//
// Scope for Step 0 (matches the current V2 DMA, which ties M_AXI_ARID=0 and is
// single-outstanding):
//   * Single outstanding transaction: AR is only accepted when no burst is in
//     flight.  Responses are therefore trivially in order.
//   * INCR bursts, 4-byte beats.  ARSIZE/ARBURST are ignored (assumed 4B/INCR).
//   * RRESP always OKAY.
//
// Step 2 will extend a sibling model to accept multiple outstanding ARs with
// unique IDs and return responses out of order; this single-outstanding model
// is left intact as the baseline stimulus.
//
// Latency convention (registered model):
//   ar_latency = L is sampled at the AR handshake.  The first R beat is asserted
//   max(L,1) bus_clk cycles after the AR handshake (L wait cycles in S_LAT for
//   L>=1; a registered model cannot present in zero cycles, so L=0 behaves as 1).
//   Subsequent beats stream one per cycle, gated by RREADY.
//
// Memory contents: internal `mem[]` word array, preloaded by the testbench via a
// hierarchical reference (e.g. u_mem.mem[idx] = ...).  Reset does NOT clear mem.
// -----------------------------------------------------------------------------

module axi_read_mem_model #(
    parameter AXI_ID_WIDTH   = 4,
    parameter AXI_ADDR_WIDTH = 32,
    parameter AXI_DATA_WIDTH = 32,
    parameter MEM_WORDS      = 1024     // 4 KB / 4 B; must be a power of two
)(
    input  wire                      clk,        // bus_clk
    input  wire                      resetn,     // active-low
    input  wire [15:0]               ar_latency, // access latency, sampled at AR accept

    //----------------------------------------------------
    // AXI4 read address channel
    //----------------------------------------------------
    input  wire [AXI_ID_WIDTH-1:0]   arid,
    input  wire [AXI_ADDR_WIDTH-1:0] araddr,
    input  wire [7:0]                arlen,
    input  wire [2:0]                arsize,     // ignored (assumed 4 bytes)
    input  wire [1:0]                arburst,    // ignored (assumed INCR)
    input  wire                      arvalid,
    output reg                       arready,

    //----------------------------------------------------
    // AXI4 read data channel
    //----------------------------------------------------
    output reg  [AXI_ID_WIDTH-1:0]   rid,
    output reg  [AXI_DATA_WIDTH-1:0] rdata,
    output reg  [1:0]                rresp,
    output reg                       rlast,
    output reg                       rvalid,
    input  wire                      rready
);

    localparam S_IDLE = 2'd0;   // ready for AR, no burst in flight
    localparam S_LAT  = 2'd1;   // access-latency wait before first beat
    localparam S_DATA = 2'd2;   // streaming burst beats

    reg [1:0]                state;
    reg [AXI_ADDR_WIDTH-1:0] addr_r;     // byte address of the beat currently/next presented
    reg [7:0]                len_r;      // ARLEN (beats-1) of the active burst
    reg [AXI_ID_WIDTH-1:0]   id_r;       // captured ARID, echoed on every R beat
    reg [7:0]                beat_r;     // index of the beat currently presented (0..len_r)
    reg [15:0]               lat_cnt;    // access-latency countdown

    // Preloaded by the testbench via hierarchical backdoor (sim-only); it has no
    // in-module driver by design, so waive Verilator's UNDRIVEN warning here.
    /* verilator lint_off UNDRIVEN */
    reg [AXI_DATA_WIDTH-1:0] mem [0:MEM_WORDS-1];
    /* verilator lint_on UNDRIVEN */

    // suppress unused-input lint (ARSIZE/ARBURST are part of the contract but
    // this model assumes 4-byte INCR)
    wire _unused_ok = &{1'b0, arsize, arburst};

    always @(posedge clk) begin
        if (!resetn) begin
            state   <= S_IDLE;
            arready <= 1'b1;
            rvalid  <= 1'b0;
            rlast   <= 1'b0;
            rresp   <= 2'b00;
            rid     <= {AXI_ID_WIDTH{1'b0}};
            rdata   <= {AXI_DATA_WIDTH{1'b0}};
            addr_r  <= {AXI_ADDR_WIDTH{1'b0}};
            len_r   <= 8'd0;
            id_r    <= {AXI_ID_WIDTH{1'b0}};
            beat_r  <= 8'd0;
            lat_cnt <= 16'd0;
        end else begin
            case (state)
                //--------------------------------------------------
                // Accept one AR (single outstanding)
                //--------------------------------------------------
                S_IDLE: begin
                    arready <= 1'b1;
                    rvalid  <= 1'b0;
                    rlast   <= 1'b0;
                    if (arvalid && arready) begin
                        arready <= 1'b0;
                        addr_r  <= araddr;
                        len_r   <= arlen;
                        id_r    <= arid;
                        beat_r  <= 8'd0;
                        // L wait cycles before the first beat (see header note)
                        lat_cnt <= (ar_latency == 16'd0) ? 16'd0 : (ar_latency - 16'd1);
                        state   <= S_LAT;
                    end
                end

                //--------------------------------------------------
                // Access-latency wait, then present the first beat
                //--------------------------------------------------
                S_LAT: begin
                    if (lat_cnt == 16'd0) begin
                        rvalid <= 1'b1;
                        rdata  <= mem[(addr_r >> 2) & (MEM_WORDS-1)];
                        rlast  <= (beat_r == len_r);    // single-beat burst -> last now
                        rid    <= id_r;
                        rresp  <= 2'b00;                // always OKAY
                        state  <= S_DATA;
                    end else begin
                        lat_cnt <= lat_cnt - 16'd1;
                    end
                end

                //--------------------------------------------------
                // Stream beats at full bandwidth (one per accepted cycle)
                //--------------------------------------------------
                S_DATA: begin
                    if (rvalid && rready) begin
                        if (beat_r == len_r) begin
                            // last beat consumed -> ready for the next AR
                            rvalid  <= 1'b0;
                            rlast   <= 1'b0;
                            state   <= S_IDLE;
                            arready <= 1'b1;
                        end else begin
                            beat_r <= beat_r + 8'd1;
                            addr_r <= addr_r + 32'd4;
                            rdata  <= mem[((addr_r + 32'd4) >> 2) & (MEM_WORDS-1)];
                            rlast  <= ((beat_r + 8'd1) == len_r);
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
