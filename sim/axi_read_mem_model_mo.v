`timescale 1ns/1ps

// axi_read_mem_model_mo.v
// -----------------------------------------------------------------------------
// MULTIPLE-OUTSTANDING, IN-ORDER behavioural AXI4 read slave with a configurable
// per-request access latency.  Test infrastructure only -- NOT synthesized.
//
// This is the Step-1 stimulus.  Unlike the Step-0 single-outstanding model, it
// accepts up to MAX_OUTSTANDING AR requests before any response returns, and runs
// an independent access-latency countdown for EACH in-flight request.  Because
// the timers run in parallel, the latencies overlap: once the first request's
// data is ready, later requests' latencies have already elapsed and their data
// streams back-to-back.  That is exactly the latency hiding the read engine's
// outstanding-credit issue logic is meant to exploit.
//
// Responses are returned strictly in allocation order (a circular FIFO of
// outstanding requests), all echoing the request's ARID.  This matches the
// Step-1 engine, which uses a single shared ARID, so in-order return is AXI-legal
// and needs no reorder buffer.  A sibling model that returns responses OUT OF
// ORDER (for the unique-ID ROB) is a Step-2 deliverable; this model is left as
// the in-order baseline stimulus.
//
// Latency convention matches axi_read_mem_model.v: first beat of a request is
// presented max(L,1) cycles after that request's AR handshake; subsequent beats
// stream one per cycle, gated by RREADY.
//
// ar_stall: test-only input.  While high, ARREADY is forced low to inject AR
// back-pressure (used to stress the engine's AR valid/payload stability).  Tie
// low for clean throughput measurement.
//
// MEM_WORDS and MAX_OUTSTANDING must be powers of two (free pointer wrap).
// -----------------------------------------------------------------------------

module axi_read_mem_model_mo #(
    parameter AXI_ID_WIDTH    = 4,
    parameter AXI_ADDR_WIDTH  = 32,
    parameter AXI_DATA_WIDTH  = 32,
    parameter MEM_WORDS       = 1024,
    parameter MAX_OUTSTANDING = 16     // model capacity; keep >= engine's count
)(
    input  wire                      clk,
    input  wire                      resetn,
    input  wire [15:0]               ar_latency, // access latency, sampled at AR accept
    input  wire                      ar_stall,   // test-only AR back-pressure inject

    //----------------------------------------------------
    // AXI4 read address channel
    //----------------------------------------------------
    input  wire [AXI_ID_WIDTH-1:0]   arid,
    input  wire [AXI_ADDR_WIDTH-1:0] araddr,
    input  wire [7:0]                arlen,
    input  wire [2:0]                arsize,     // ignored (assumed 4 bytes)
    input  wire [1:0]                arburst,    // ignored (assumed INCR)
    input  wire                      arvalid,
    output wire                      arready,

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

    localparam PTR_W = (MAX_OUTSTANDING <= 1) ? 1 : $clog2(MAX_OUTSTANDING);

    // Per-request entries (circular FIFO; entry index = allocation order)
    reg                       e_valid [0:MAX_OUTSTANDING-1];
    reg [AXI_ADDR_WIDTH-1:0]  e_addr  [0:MAX_OUTSTANDING-1];
    reg [7:0]                 e_len   [0:MAX_OUTSTANDING-1];
    reg [AXI_ID_WIDTH-1:0]    e_id    [0:MAX_OUTSTANDING-1];
    reg [15:0]                e_timer [0:MAX_OUTSTANDING-1];   // parallel latency countdowns

    reg [PTR_W-1:0]           alloc_ptr;   // next entry to allocate
    reg [PTR_W-1:0]           head_ptr;    // next entry to return (in order)
    reg [PTR_W:0]             count;       // occupancy 0..MAX_OUTSTANDING

    // Streaming snapshot for the head burst currently on the R channel
    reg                       streaming;
    reg [AXI_ADDR_WIDTH-1:0]  cur_addr;
    reg [7:0]                 cur_len;
    reg [7:0]                 cur_beat;

    // Preloaded by the testbench via hierarchical backdoor (sim-only); intentionally
    // undriven in-module, so waive Verilator's UNDRIVEN warning here.
    /* verilator lint_off UNDRIVEN */
    reg [AXI_DATA_WIDTH-1:0] mem [0:MEM_WORDS-1];
    /* verilator lint_on UNDRIVEN */

    wire _unused_ok = &{1'b0, arsize, arburst};

    assign arready = resetn && !ar_stall && (count < MAX_OUTSTANDING);

    wire ar_acc     = arvalid && arready;
    wire head_ready = (count != 0) && e_valid[head_ptr] && (e_timer[head_ptr] == 16'd0);
    wire burst_done = streaming && rvalid && rready && (cur_beat == cur_len);

    integer i;
    always @(posedge clk) begin
        if (!resetn) begin
            for (i = 0; i < MAX_OUTSTANDING; i = i + 1) begin
                e_valid[i] <= 1'b0;
                e_timer[i] <= 16'd0;
            end
            alloc_ptr <= {PTR_W{1'b0}};
            head_ptr  <= {PTR_W{1'b0}};
            count     <= {(PTR_W+1){1'b0}};
            streaming <= 1'b0;
            cur_addr  <= {AXI_ADDR_WIDTH{1'b0}};
            cur_len   <= 8'd0;
            cur_beat  <= 8'd0;
            rvalid    <= 1'b0;
            rlast     <= 1'b0;
            rresp     <= 2'b00;
            rid       <= {AXI_ID_WIDTH{1'b0}};
            rdata     <= {AXI_DATA_WIDTH{1'b0}};
        end else begin
            // (a) parallel access-latency countdown for every in-flight request
            for (i = 0; i < MAX_OUTSTANDING; i = i + 1)
                if (e_valid[i] && e_timer[i] != 16'd0)
                    e_timer[i] <= e_timer[i] - 16'd1;

            // (b) accept one AR (allocate an entry)
            if (ar_acc) begin
                e_valid[alloc_ptr] <= 1'b1;
                e_addr [alloc_ptr] <= araddr;
                e_len  [alloc_ptr] <= arlen;
                e_id   [alloc_ptr] <= arid;
                e_timer[alloc_ptr] <= (ar_latency == 16'd0) ? 16'd0 : (ar_latency - 16'd1);
                alloc_ptr          <= alloc_ptr + 1'b1;   // wraps (power-of-two depth)
            end

            // (c) in-order response return (one head burst at a time)
            if (!streaming) begin
                rvalid <= 1'b0;
                rlast  <= 1'b0;
                if (head_ready) begin
                    streaming <= 1'b1;
                    cur_addr  <= e_addr[head_ptr];
                    cur_len   <= e_len[head_ptr];
                    cur_beat  <= 8'd0;
                    rvalid    <= 1'b1;
                    rdata     <= mem[(e_addr[head_ptr] >> 2) & (MEM_WORDS-1)];
                    rlast     <= (e_len[head_ptr] == 8'd0);
                    rid       <= e_id[head_ptr];
                    rresp     <= 2'b00;
                end
            end else begin
                if (rvalid && rready) begin
                    if (cur_beat == cur_len) begin
                        // last beat of this burst consumed -> retire the head entry
                        rvalid            <= 1'b0;
                        rlast             <= 1'b0;
                        streaming         <= 1'b0;
                        e_valid[head_ptr] <= 1'b0;
                        head_ptr          <= head_ptr + 1'b1;
                    end else begin
                        cur_beat <= cur_beat + 8'd1;
                        cur_addr <= cur_addr + 32'd4;
                        rdata    <= mem[((cur_addr + 32'd4) >> 2) & (MEM_WORDS-1)];
                        rlast    <= ((cur_beat + 8'd1) == cur_len);
                    end
                end
            end

            // (d) occupancy update (alloc +1, retire -1; both may occur)
            case ({ar_acc, burst_done})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
