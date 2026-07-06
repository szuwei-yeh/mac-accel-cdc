`timescale 1ns/1ps

// axi_read_mem_model_ooo.v
// -----------------------------------------------------------------------------
// OUT-OF-ORDER, multi-ID behavioural AXI4 read slave with configurable latency.
// Test infrastructure only -- NOT synthesized.  This is the Step-2 stimulus that
// makes out-of-order completion real, so the engine's reorder buffer has to do
// actual work.
//
// Each in-flight request occupies a slot indexed by its ARID (the engine issues a
// unique ARID = ROB entry index, so IDs do not collide while in flight).  Per-slot
// access-latency countdowns run in parallel, with optional per-request jitter so
// requests mature in a scrambled order.  An R-channel arbiter then returns beats:
//   * ooo_en=1 : round-robin one beat at a time among ready slots -> beats from
//                different IDs INTERLEAVE, and bursts complete out of order.
//   * ooo_en=0 : finish the current ready slot's burst before moving on (still
//                served in readiness order, so completion order can differ from
//                allocation order).
// AXI legality is preserved: a given ID's beats are emitted in order (per-slot
// beat counter) with RLAST on that ID's last beat, and RRESP is always OKAY.
//
// ar_stall: test-only AR back-pressure inject (forces ARREADY low while high).
//
// MEM_WORDS and MAX_OUTSTANDING are powers of two.  MAX_OUTSTANDING == the
// engine's outstanding count (slots are indexed by ARID in [0,MAX_OUTSTANDING)).
// -----------------------------------------------------------------------------

module axi_read_mem_model_ooo #(
    parameter AXI_ID_WIDTH    = 4,
    parameter AXI_ADDR_WIDTH  = 32,
    parameter AXI_DATA_WIDTH  = 32,
    parameter MEM_WORDS       = 1024,
    parameter MAX_OUTSTANDING = 4
)(
    input  wire                      clk,
    input  wire                      resetn,
    input  wire [15:0]               ar_latency,
    input  wire                      ar_stall,
    input  wire                      ooo_en,      // 1 = beat-interleave, 0 = burst-granular

    //----------------------------------------------------
    // AXI4 read address channel
    //----------------------------------------------------
    input  wire [AXI_ID_WIDTH-1:0]   arid,
    input  wire [AXI_ADDR_WIDTH-1:0] araddr,
    input  wire [7:0]                arlen,
    input  wire [2:0]                arsize,
    input  wire [1:0]                arburst,
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

    localparam ID_W = (MAX_OUTSTANDING <= 1) ? 1 : $clog2(MAX_OUTSTANDING);

    // Per-slot request state (slot index = ARID)
    reg                       e_valid [0:MAX_OUTSTANDING-1];
    reg [AXI_ADDR_WIDTH-1:0]  e_addr  [0:MAX_OUTSTANDING-1];
    reg [7:0]                 e_len   [0:MAX_OUTSTANDING-1];   // ARLEN
    reg [7:0]                 e_beat  [0:MAX_OUTSTANDING-1];   // beats already accepted
    reg [15:0]                e_timer [0:MAX_OUTSTANDING-1];   // access-latency countdown

    reg [ID_W:0]              count;       // occupancy
    reg [ID_W-1:0]            rr_base;     // round-robin arbiter base
    reg [ID_W-1:0]            p_id;        // slot whose beat is presented on R
    reg [7:0]                 p_beat;      // beat index presented on R
    reg [15:0]                lfsr;        // jitter source

    /* verilator lint_off UNDRIVEN */
    reg [AXI_DATA_WIDTH-1:0] mem [0:MEM_WORDS-1];
    /* verilator lint_on UNDRIVEN */

    wire _unused_ok = &{1'b0, arsize, arburst, arid[AXI_ID_WIDTH-1:ID_W]};

    assign arready = resetn && !ar_stall && (count < MAX_OUTSTANDING);
    wire ar_acc = arvalid && arready;

    // combinational arbiter scratch
    wire [ID_W-1:0] arid_idx = arid[ID_W-1:0];
    wire            beat_acc = rvalid && rready;
    wire            free_now = beat_acc && (p_beat == e_len[p_id]);

    //----------------------------------------------------
    // Combinational R-channel arbiter: pick the next beat to present.
    // Scans round-robin from rr_base for the first slot that is valid, has its
    // data ready (timer==0), and still has beats to send -- with eff_sent
    // accounting for a beat being accepted THIS cycle (so a continuing burst
    // advances without a bubble).
    //----------------------------------------------------
    integer        k;
    reg            found;
    reg [ID_W-1:0] gsel;
    reg [7:0]      geff;
    reg [ID_W-1:0] idx;
    reg [7:0]      eff_sent;

    always @(*) begin
        found    = 1'b0;
        gsel     = {ID_W{1'b0}};
        geff     = 8'd0;
        idx      = {ID_W{1'b0}};
        eff_sent = 8'd0;
        for (k = 0; k < MAX_OUTSTANDING; k = k + 1) begin
            idx      = rr_base + k[ID_W-1:0];                       // round-robin scan, wraps
            eff_sent = e_beat[idx] + ((beat_acc && (idx == p_id)) ? 8'd1 : 8'd0);
            if (!found
                && e_valid[idx]
                && !(free_now && (idx == p_id))                    // skip the just-retired slot
                && (e_timer[idx] == 16'd0)                         // data ready
                && (eff_sent <= e_len[idx])) begin                 // beats remaining
                found = 1'b1;
                gsel  = idx;
                geff  = eff_sent;
            end
        end
    end

    // word index of the presented beat (width-clean: zero-extend the beat offset)
    wire [AXI_ADDR_WIDTH-1:0] beat_word =
        (e_addr[gsel] >> 2) + {{(AXI_ADDR_WIDTH-8){1'b0}}, geff};

    integer ki;
    always @(posedge clk) begin
        if (!resetn) begin
            for (ki = 0; ki < MAX_OUTSTANDING; ki = ki + 1) begin
                e_valid[ki] <= 1'b0;
                e_timer[ki] <= 16'd0;
                e_beat[ki]  <= 8'd0;
            end
            count   <= {(ID_W+1){1'b0}};
            rr_base <= {ID_W{1'b0}};
            p_id    <= {ID_W{1'b0}};
            p_beat  <= 8'd0;
            lfsr    <= 16'hACE1;
            rvalid  <= 1'b0;
            rlast   <= 1'b0;
            rresp   <= 2'b00;
            rid     <= {AXI_ID_WIDTH{1'b0}};
            rdata   <= {AXI_DATA_WIDTH{1'b0}};
        end else begin
            // jitter LFSR (16-bit maximal taps), used as a random arbiter base
            lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[3]};

            // (a) parallel access-latency countdown (uniform L per request)
            for (ki = 0; ki < MAX_OUTSTANDING; ki = ki + 1)
                if (e_valid[ki] && e_timer[ki] != 16'd0)
                    e_timer[ki] <= e_timer[ki] - 16'd1;

            // (b) accept one AR -> allocate slot[arid]
            if (ar_acc) begin
                e_valid[arid_idx] <= 1'b1;
                e_addr [arid_idx] <= araddr;
                e_len  [arid_idx] <= arlen;
                e_beat [arid_idx] <= 8'd0;
                e_timer[arid_idx] <= (ar_latency == 16'd0) ? 16'd0 : (ar_latency - 16'd1);
            end

            // (c) R-channel acceptance bookkeeping
            if (beat_acc) begin
                e_beat[p_id] <= e_beat[p_id] + 8'd1;
                if (free_now)
                    e_valid[p_id] <= 1'b0;
            end

            // (d) present the arbitrated beat when the channel is free
            if (!rvalid || beat_acc) begin
                if (found) begin
                    rvalid  <= 1'b1;
                    p_id    <= gsel;
                    p_beat  <= geff;
                    rid     <= {{(AXI_ID_WIDTH-ID_W){1'b0}}, gsel};
                    rdata   <= mem[beat_word & (MEM_WORDS-1)];
                    rlast   <= (geff == e_len[gsel]);
                    rresp   <= 2'b00;
                    // ooo_en=1: random base each beat -> interleave + scrambled
                    // completion.  ooo_en=0: finish this burst, then round-robin.
                    rr_base <= ooo_en ? lfsr[ID_W-1:0]
                                      : ((geff == e_len[gsel]) ? (gsel + 1'b1) : gsel);
                end else begin
                    rvalid <= 1'b0;
                    rlast  <= 1'b0;
                end
            end

            // (e) occupancy update (alloc +1, retire -1)
            case ({ar_acc, free_now})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
