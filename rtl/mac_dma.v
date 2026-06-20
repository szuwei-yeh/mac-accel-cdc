`timescale 1ns/1ps

// mac_dma.v
// AXI4 master DMA front-end for the MAC accelerator.
//
// Reads vector A then vector B from external memory via AXI4 INCR bursts and
// emits paired {last, a, b} samples on an internal stream interface intended
// for the async FIFO at the bus_clk -> mac_clk boundary.
//
// All logic runs in bus_clk.  Write channels (AW/W/B) are unused; this is a
// read-only master.  Each AXI beat carries one element in the low DATA_WIDTH
// bits (upper bits ignored, no sign-extension required because mac_pe treats
// the operands as signed [DATA_WIDTH-1:0]).
//
// AXI burst rules respected:
//   - ARSIZE  = 3'b010 (4 bytes)
//   - ARBURST = 2'b01  (INCR)
//   - Each burst <= BURST_LEN beats.  Multi-burst transactions issued if
//     length exceeds one burst.  Bursts never cross a 4 KB boundary as long
//     as the caller passes 4-byte aligned addresses (16-beat bursts span 64 B).

module mac_dma #(
    parameter DATA_WIDTH      = 16,
    parameter AXI_DATA_WIDTH  = 32,
    parameter AXI_ADDR_WIDTH  = 32,
    parameter AXI_ID_WIDTH    = 4,
    parameter MAX_LEN         = 256,   // local buffer depth for vector A
    parameter BURST_LEN       = 16     // max beats per AXI burst
)(
    input  wire                       clk,        // bus_clk
    input  wire                       rst,        // active-high

    //----------------------------------------------------
    // Control (bus_clk domain)
    //----------------------------------------------------
    input  wire                       start,      // 1-cycle pulse
    input  wire [AXI_ADDR_WIDTH-1:0]  src_a_addr,
    input  wire [AXI_ADDR_WIDTH-1:0]  src_b_addr,
    input  wire [15:0]                length,
    output reg                        dma_busy,
    output reg                        dma_done,   // 1-cycle pulse
    output reg                        dma_err,    // sticky: RRESP != OKAY

    //----------------------------------------------------
    // AXI4 master AR channel
    //----------------------------------------------------
    output wire [AXI_ID_WIDTH-1:0]    M_AXI_ARID,
    output reg  [AXI_ADDR_WIDTH-1:0]  M_AXI_ARADDR,
    output reg  [7:0]                 M_AXI_ARLEN,
    output wire [2:0]                 M_AXI_ARSIZE,
    output wire [1:0]                 M_AXI_ARBURST,
    output reg                        M_AXI_ARVALID,
    input  wire                       M_AXI_ARREADY,

    //----------------------------------------------------
    // AXI4 master R channel
    //----------------------------------------------------
    input  wire [AXI_ID_WIDTH-1:0]    M_AXI_RID,
    input  wire [AXI_DATA_WIDTH-1:0]  M_AXI_RDATA,
    input  wire [1:0]                 M_AXI_RRESP,
    input  wire                       M_AXI_RLAST,
    input  wire                       M_AXI_RVALID,
    output wire                       M_AXI_RREADY,

    //----------------------------------------------------
    // Outgoing stream -> async FIFO write side
    //----------------------------------------------------
    output reg                          stream_valid,
    output reg                          stream_last,
    output reg  signed [DATA_WIDTH-1:0] stream_a,
    output reg  signed [DATA_WIDTH-1:0] stream_b,
    input  wire                         stream_ready   // !fifo_full
);

    // Constant AXI tie-offs
    assign M_AXI_ARID    = {AXI_ID_WIDTH{1'b0}};
    assign M_AXI_ARSIZE  = 3'b010;   // 4 bytes per beat
    assign M_AXI_ARBURST = 2'b01;    // INCR

    //----------------------------------------------------
    // FSM
    //----------------------------------------------------
    localparam S_IDLE = 3'd0;
    localparam S_AR_A = 3'd1;
    localparam S_R_A  = 3'd2;
    localparam S_AR_B = 3'd3;
    localparam S_R_B  = 3'd4;
    localparam S_DONE = 3'd5;

    reg [2:0] state, next_state;

    // Local buffer for vector A (single-port, write-then-read)
    reg signed [DATA_WIDTH-1:0] buf_a [0:MAX_LEN-1];

    // Tracking
    reg [15:0]                length_r;
    reg [15:0]                elem_cnt;          // elements transferred in current vector
    reg [15:0]                write_idx_a;       // write pointer into buf_a
    reg [15:0]                read_idx_a;        // read pointer into buf_a
    reg [AXI_ADDR_WIDTH-1:0]  cur_addr;          // address of next burst
    reg [15:0]                cur_burst_beats;   // beats outstanding in this burst

    wire [15:0] remaining  = length_r - elem_cnt;
    wire [15:0] next_beats = (remaining > BURST_LEN[15:0]) ? BURST_LEN[15:0] : remaining;

    // RREADY: in S_R_A always accept (internal buffer); in S_R_B accept a new
    // beat only when the skid register has room -- empty, or being drained this
    // cycle.  (READY may depend on READY/full; only VALID may not depend on READY.)
    assign M_AXI_RREADY = (state == S_R_A) ? 1'b1 :
                          (state == S_R_B) ? (!stream_valid || stream_ready) : 1'b0;

    wire ar_handshake = M_AXI_ARVALID & M_AXI_ARREADY;
    wire r_handshake  = M_AXI_RVALID  & M_AXI_RREADY;

    //----------------------------------------------------
    // FSM next state
    //----------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:  if (start) next_state = S_AR_A;

            S_AR_A:  if (ar_handshake) next_state = S_R_A;

            S_R_A:   if (r_handshake && M_AXI_RLAST)
                         next_state = (elem_cnt + 16'd1 == length_r) ? S_AR_B : S_AR_A;

            S_AR_B:  if (ar_handshake) next_state = S_R_B;

            S_R_B:   if (r_handshake && M_AXI_RLAST)
                         next_state = (elem_cnt + 16'd1 == length_r) ? S_DONE : S_AR_B;

            S_DONE:  next_state = S_IDLE;

            default: next_state = S_IDLE;
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) state <= S_IDLE;
        else     state <= next_state;
    end

    //----------------------------------------------------
    // Datapath
    //----------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            length_r         <= 16'd0;
            elem_cnt         <= 16'd0;
            write_idx_a      <= 16'd0;
            read_idx_a       <= 16'd0;
            cur_addr         <= {AXI_ADDR_WIDTH{1'b0}};
            cur_burst_beats  <= 16'd0;

            dma_busy         <= 1'b0;
            dma_done         <= 1'b0;
            dma_err          <= 1'b0;

            M_AXI_ARADDR     <= {AXI_ADDR_WIDTH{1'b0}};
            M_AXI_ARLEN      <= 8'd0;
            M_AXI_ARVALID    <= 1'b0;

            stream_valid     <= 1'b0;
            stream_last      <= 1'b0;
            stream_a         <= {DATA_WIDTH{1'b0}};
            stream_b         <= {DATA_WIDTH{1'b0}};
        end else begin
            // single-cycle pulse default low
            dma_done     <= 1'b0;

            // sticky error
            if (r_handshake && (M_AXI_RRESP != 2'b00))
                dma_err <= 1'b1;

            // Skid drain: clear VALID only when the consumer accepts (READY).
            // VALID is NEVER a function of READY -> AXI-Stream compliant.  Runs
            // in every state so the final beat still drains after S_R_B exits.
            if (stream_valid && stream_ready)
                stream_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    dma_busy      <= 1'b0;
                    M_AXI_ARVALID <= 1'b0;
                    if (start) begin
                        dma_busy    <= 1'b1;
                        dma_err     <= 1'b0;
                        length_r    <= length;
                        elem_cnt    <= 16'd0;
                        write_idx_a <= 16'd0;
                        read_idx_a  <= 16'd0;
                        cur_addr    <= src_a_addr;
                    end
                end

                S_AR_A, S_AR_B: begin
                    // Drive AR until handshake
                    M_AXI_ARVALID   <= 1'b1;
                    M_AXI_ARADDR    <= cur_addr;
                    M_AXI_ARLEN     <= next_beats[7:0] - 8'd1;
                    cur_burst_beats <= next_beats;
                    if (ar_handshake)
                        M_AXI_ARVALID <= 1'b0;
                end

                S_R_A: begin
                    if (r_handshake) begin
                        buf_a[write_idx_a] <= M_AXI_RDATA[DATA_WIDTH-1:0];
                        write_idx_a        <= write_idx_a + 16'd1;
                        elem_cnt           <= elem_cnt + 16'd1;
                        if (M_AXI_RLAST) begin
                            // Default: advance pointer for next burst of A
                            cur_addr <= cur_addr + (cur_burst_beats << 2);
                            if (elem_cnt + 16'd1 == length_r) begin
                                // Vector A fully read -> switch to B
                                elem_cnt <= 16'd0;
                                cur_addr <= src_b_addr;
                            end
                        end
                    end
                end

                S_R_B: begin
                    // Accept a B beat only when the skid has room (gated by
                    // RREADY above).  Load it into the skid register and assert
                    // VALID; VALID is held (independent of READY) until the drain
                    // above clears it -> no element is ever dropped or advanced
                    // past before it is safely written to the FIFO.
                    if (r_handshake) begin
                        stream_a     <= buf_a[read_idx_a];
                        stream_b     <= M_AXI_RDATA[DATA_WIDTH-1:0];
                        stream_last  <= (elem_cnt + 16'd1 == length_r);
                        stream_valid <= 1'b1;
                        read_idx_a   <= read_idx_a + 16'd1;
                        elem_cnt     <= elem_cnt + 16'd1;
                        if (M_AXI_RLAST)
                            cur_addr <= cur_addr + (cur_burst_beats << 2);
                    end
                end

                S_DONE: begin
                    dma_done <= 1'b1;
                    dma_busy <= 1'b0;
                end

                default: ;
            endcase
        end
    end

`ifndef SYNTHESIS
    initial begin
        $display("[MAC_DMA] instantiated: BURST_LEN=%0d MAX_LEN=%0d", BURST_LEN, MAX_LEN);
    end
`endif

`ifdef FORMAL
    // -------------------------------------------------------------------
    // Formal: AXI-Stream VALID-stability / no-drop property (single bus_clk
    // domain).  Proves the producer never drops a back-pressured beat.
    //
    // stream_ready (= ~fifo_full) is a FREE input -> the solver drives it
    // adversarially, modelling any back-pressure pattern a real FIFO could
    // produce.  length is pinned small so BMC unrolls a short, fast bound.
    //
    // The property below asserts that once VALID is asserted and the consumer
    // has not yet accepted (READY low), VALID stays high and {a,b,last} are
    // held stable on the next cycle -- the spec-compliant handshake guarantee.
    // -------------------------------------------------------------------
    reg f_past_valid = 1'b0;
    always @(posedge clk) f_past_valid <= 1'b1;

    // force a clean reset in the very first step so registers start at their
    // reset values (otherwise BMC picks an arbitrary -> spurious -> initial state)
    always @(*) if (!f_past_valid) assume (rst);

    // bound the transfer so the counterexample is short
    always @(*) assume (length == 16'd2);

    // -------------------------------------------------------------------
    // Inductive invariant (needed for `mode prove` / k-induction):
    //   k-induction starts the inductive step from an ARBITRARY register
    //   state, not from reset.  Without help the solver may begin in an
    //   unreachable state where `state` holds one of the two illegal 3-bit
    //   encodings (3'd6 / 3'd7), then manufacture a spurious counterexample.
    //   Asserting the FSM is always in a legal encoding does double duty: it
    //   is a true property AND it strengthens the induction hypothesis so the
    //   step can close.  (BMC reaches this for free; only `prove` needs it.)
    // -------------------------------------------------------------------
    always @(posedge clk)
        if (f_past_valid && !rst)
            a_state_legal : assert (state <= S_DONE);

    // CORE PROPERTY (AXI-Stream VALID stability / no-drop under back-pressure):
    //   Once VALID is asserted it must remain asserted, with the payload held
    //   stable, until the consumer accepts it (READY).  VALID must NEVER depend
    //   on READY.  This is the spec-compliant guarantee that a back-pressured
    //   element is held -- never dropped -- until it is written to the FIFO.
    always @(posedge clk)
        if (f_past_valid && !rst && !$past(rst)
            && $past(stream_valid) && !$past(stream_ready)) begin
            a_valid_stable : assert (stream_valid);
            a_dataA_stable : assert (stream_a    == $past(stream_a));
            a_dataB_stable : assert (stream_b    == $past(stream_b));
            a_last_stable  : assert (stream_last == $past(stream_last));
        end
`endif

endmodule
