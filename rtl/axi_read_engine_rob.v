`timescale 1ns/1ps

// axi_read_engine_rob.v
// -----------------------------------------------------------------------------
// Standalone AXI4 read engine for the outstanding-transaction / ROB project.
// Owns ONLY AR-issue + R-collection for one read phase; knows nothing about MAC,
// A/B pairing, or CDC.  Output is an in-order beat stream.
//
// *** STEP 2 (Milestone B): UNIQUE ARID per burst + REORDER BUFFER ***
//   - Issues up to MAX_OUTSTANDING read bursts, each with a UNIQUE ARID equal to
//     its ROB entry index (alloc_ptr).  Unique IDs let the slave return responses
//     OUT OF ORDER / interleaved across IDs (AXI only requires same-ID beats to be
//     in order).  This is what makes out-of-order completion real.
//   - A reorder buffer (circular, depth = MAX_OUTSTANDING, entry index = ARID)
//     restores program order: responses fill entries out of order (indexed by
//     RID), but retirement is strictly in allocation order (head pointer).  The
//     output stream out_valid/out_data/out_last is therefore in order regardless
//     of response order.  Ordering here is in-order RETIREMENT, not data-hazard
//     detection: v1 is read-only.
//   - R is always accepted (RREADY=1 while running): the ROB IS the buffer, so R
//     reception is decoupled from out retirement.  Back-pressure propagates the
//     other way: if out stalls, entries fill, occupancy hits MAX_OUTSTANDING, and
//     AR issue stalls -- the producer is throttled without ever dropping a beat.
//
// AR valid/payload stability under back-pressure holds by construction (same as
// Step 1): ARVALID = want_issue, and want_issue / the AR payload change only on an
// AR accept (a freed credit can only make want_issue more true), so ARVALID never
// depends on ARREADY and the payload is held until the slave accepts.
//
// Requirements: MAX_OUTSTANDING and MAX_BURST_LEN are powers of two (the flat ROB
// index is {entry, beat} = alloc_ptr||recv_beat, and pointer wrap is free bit
// truncation).  MAX_OUTSTANDING >= 2 for the ROB to be meaningful.
// -----------------------------------------------------------------------------

module axi_read_engine_rob #(
    parameter AXI_DATA_W      = 32,
    parameter AXI_ADDR_W      = 32,
    parameter AXI_ID_WIDTH    = 4,
    parameter MAX_OUTSTANDING = 4,    // in-flight bursts / ROB depth (power of two, >=2)
    parameter MAX_BURST_LEN   = 16,   // max beats per burst (power of two)
    parameter LEN_WIDTH       = 16
)(
    input  wire                     clk,
    input  wire                     rst,        // active-high

    //----------------------------------------------------
    // Command: start one read phase
    //----------------------------------------------------
    input  wire                     cmd_valid,
    output wire                     cmd_ready,
    input  wire [AXI_ADDR_W-1:0]    cmd_addr,
    input  wire [LEN_WIDTH-1:0]     cmd_len,

    //----------------------------------------------------
    // AXI4 master AR channel
    //----------------------------------------------------
    output wire [AXI_ID_WIDTH-1:0]  M_AXI_ARID,
    output wire [AXI_ADDR_W-1:0]    M_AXI_ARADDR,
    output wire [7:0]               M_AXI_ARLEN,
    output wire [2:0]               M_AXI_ARSIZE,
    output wire [1:0]               M_AXI_ARBURST,
    output wire                     M_AXI_ARVALID,
    input  wire                     M_AXI_ARREADY,

    //----------------------------------------------------
    // AXI4 master R channel
    //----------------------------------------------------
    input  wire [AXI_ID_WIDTH-1:0]  M_AXI_RID,
    input  wire [AXI_DATA_W-1:0]    M_AXI_RDATA,
    input  wire [1:0]               M_AXI_RRESP,
    input  wire                     M_AXI_RLAST,
    input  wire                     M_AXI_RVALID,
    output wire                     M_AXI_RREADY,

    //----------------------------------------------------
    // Output stream: in-order retired beats for the active phase
    //----------------------------------------------------
    output wire                     out_valid,
    input  wire                     out_ready,
    output wire [AXI_DATA_W-1:0]    out_data,
    output wire                     out_last,     // final beat of the PHASE (not burst)

    //----------------------------------------------------
    // Status
    //----------------------------------------------------
    output wire                     engine_busy,
    output reg                      engine_done,
    output reg                      engine_err
);

    localparam S_IDLE = 2'd0;
    localparam S_RUN  = 2'd1;
    localparam S_DONE = 2'd2;

    localparam ID_W       = (MAX_OUTSTANDING <= 1) ? 1 : $clog2(MAX_OUTSTANDING);
    localparam LOG2_BURST = $clog2(MAX_BURST_LEN);
    localparam RIDX_W     = ID_W + LOG2_BURST;          // flat ROB data index width
    localparam OCC_W      = ID_W + 1;                   // occupancy 0..MAX_OUTSTANDING

    //----------------------------------------------------
    // ROB entry metadata (depth = MAX_OUTSTANDING, index = ARID)
    //----------------------------------------------------
    reg                  e_valid    [0:MAX_OUTSTANDING-1];   // allocated
    reg                  e_complete [0:MAX_OUTSTANDING-1];   // all beats + RLAST seen
    reg [7:0]            e_recv     [0:MAX_OUTSTANDING-1];   // beats received so far
    reg [7:0]            e_exp      [0:MAX_OUTSTANDING-1];   // expected beats (ARLEN+1)
    reg [AXI_DATA_W-1:0] rob_data   [0:(1<<RIDX_W)-1];       // {entry,beat} -> beat data

    reg [ID_W-1:0]       alloc_ptr;     // next entry to allocate (== ARID)
    reg [ID_W-1:0]       head_ptr;      // next entry to retire (in order)
    reg [OCC_W-1:0]      occupancy;     // full when == MAX_OUTSTANDING

    reg [7:0]            drain_idx;     // beat index within the head entry being drained

    reg [1:0]            state;
    reg [AXI_ADDR_W-1:0] cmd_addr_r;
    reg [LEN_WIDTH-1:0]  cmd_len_r;
    reg [LEN_WIDTH-1:0]  issue_elem;    // beats whose AR has been accepted
    reg [LEN_WIDTH-1:0]  retire_elem;   // beats retired to out (in order)

    integer i;

    //----------------------------------------------------
    // Next-burst geometry (combinational; depends only on issue_elem)
    //----------------------------------------------------
    wire [LEN_WIDTH-1:0] elems_left = cmd_len_r - issue_elem;
    wire [LEN_WIDTH-1:0] next_beats = (elems_left > MAX_BURST_LEN) ? MAX_BURST_LEN[LEN_WIDTH-1:0]
                                                                   : elems_left;
    wire [AXI_ADDR_W-1:0] issue_byte_off =
        {{(AXI_ADDR_W-LEN_WIDTH){1'b0}}, issue_elem} << 2;

    wire more_bursts = (issue_elem < cmd_len_r);
    wire have_credit = (occupancy < MAX_OUTSTANDING);
    wire want_issue  = (state == S_RUN) && more_bursts && have_credit;

    //----------------------------------------------------
    // AR channel (ARID = alloc_ptr, zero-extended to AXI_ID_WIDTH).
    // Requires AXI_ID_WIDTH > ID_W (true here: AXI_ID_WIDTH=4, MAX_OUTSTANDING<=8).
    //----------------------------------------------------
    assign M_AXI_ARID    = {{(AXI_ID_WIDTH-ID_W){1'b0}}, alloc_ptr};
    assign M_AXI_ARADDR  = cmd_addr_r + issue_byte_off;
    assign M_AXI_ARLEN   = next_beats[7:0] - 8'd1;
    assign M_AXI_ARSIZE  = 3'b010;                 // 4 bytes/beat
    assign M_AXI_ARBURST = 2'b01;                  // INCR
    assign M_AXI_ARVALID = want_issue;

    //----------------------------------------------------
    // R channel: always accept into the ROB while running
    //----------------------------------------------------
    wire [ID_W-1:0]     rid_idx  = M_AXI_RID[ID_W-1:0];
    wire [RIDX_W-1:0]   rob_widx = {rid_idx, e_recv[rid_idx][LOG2_BURST-1:0]};
    assign M_AXI_RREADY = (state == S_RUN);
    wire r_acc = M_AXI_RVALID && M_AXI_RREADY;

    //----------------------------------------------------
    // Retire: in allocation order from head, once the head entry is complete
    //----------------------------------------------------
    wire head_ready = e_valid[head_ptr] && e_complete[head_ptr];
    wire [RIDX_W-1:0] rob_ridx = {head_ptr, drain_idx[LOG2_BURST-1:0]};

    reg [AXI_DATA_W-1:0] out_data_r;
    always @(*) out_data_r = rob_data[rob_ridx];

    assign out_valid    = (state == S_RUN) && head_ready;
    assign out_data     = out_data_r;
    assign out_last     = (retire_elem == cmd_len_r - {{(LEN_WIDTH-1){1'b0}}, 1'b1});

    assign cmd_ready   = (state == S_IDLE);
    assign engine_busy = (state == S_RUN);

    //----------------------------------------------------
    // Handshake events
    //----------------------------------------------------
    wire ar_acc        = M_AXI_ARVALID && M_AXI_ARREADY;
    wire out_acc       = out_valid && out_ready;
    wire entry_drained = out_acc && (drain_idx == e_exp[head_ptr] - 8'd1);
    wire last_beat     = out_acc && (retire_elem == cmd_len_r - {{(LEN_WIDTH-1){1'b0}}, 1'b1});

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            cmd_addr_r  <= {AXI_ADDR_W{1'b0}};
            cmd_len_r   <= {LEN_WIDTH{1'b0}};
            issue_elem  <= {LEN_WIDTH{1'b0}};
            retire_elem <= {LEN_WIDTH{1'b0}};
            alloc_ptr   <= {ID_W{1'b0}};
            head_ptr    <= {ID_W{1'b0}};
            occupancy   <= {OCC_W{1'b0}};
            drain_idx   <= 8'd0;
            engine_done <= 1'b0;
            engine_err  <= 1'b0;
            for (i = 0; i < MAX_OUTSTANDING; i = i + 1) begin
                e_valid[i]    <= 1'b0;
                e_complete[i] <= 1'b0;
            end
        end else begin
            engine_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (cmd_valid) begin
                        cmd_addr_r  <= cmd_addr;
                        cmd_len_r   <= cmd_len;
                        issue_elem  <= {LEN_WIDTH{1'b0}};
                        retire_elem <= {LEN_WIDTH{1'b0}};
                        alloc_ptr   <= {ID_W{1'b0}};
                        head_ptr    <= {ID_W{1'b0}};
                        occupancy   <= {OCC_W{1'b0}};
                        drain_idx   <= 8'd0;
                        engine_err  <= 1'b0;
                        for (i = 0; i < MAX_OUTSTANDING; i = i + 1) begin
                            e_valid[i]    <= 1'b0;
                            e_complete[i] <= 1'b0;
                        end
                        state <= S_RUN;
                    end
                end

                S_RUN: begin
                    //------------------------------------------------
                    // Issue: allocate a ROB entry on each accepted AR
                    //------------------------------------------------
                    if (ar_acc) begin
                        e_valid[alloc_ptr]    <= 1'b1;
                        e_complete[alloc_ptr] <= 1'b0;
                        e_recv[alloc_ptr]     <= 8'd0;
                        e_exp[alloc_ptr]      <= next_beats[7:0];
                        issue_elem            <= issue_elem + next_beats;
                        alloc_ptr             <= alloc_ptr + 1'b1;   // wraps mod MAX_OUTSTANDING
                    end

                    //------------------------------------------------
                    // Receive: write each accepted R beat into entry[RID]
                    // (responses may arrive out of order / interleaved)
                    //------------------------------------------------
                    if (r_acc) begin
                        rob_data[rob_widx] <= M_AXI_RDATA;
                        e_recv[rid_idx]    <= e_recv[rid_idx] + 8'd1;
                        if (M_AXI_RLAST)
                            e_complete[rid_idx] <= 1'b1;
                        if (M_AXI_RRESP != 2'b00)
                            engine_err <= 1'b1;
                    end

                    //------------------------------------------------
                    // Retire: stream the head entry's beats in order
                    //------------------------------------------------
                    if (out_acc) begin
                        retire_elem <= retire_elem + 1'b1;
                        if (entry_drained) begin
                            e_valid[head_ptr] <= 1'b0;
                            head_ptr          <= head_ptr + 1'b1;   // wraps mod MAX_OUTSTANDING
                            drain_idx         <= 8'd0;
                        end else begin
                            drain_idx <= drain_idx + 8'd1;
                        end
                    end

                    //------------------------------------------------
                    // Occupancy: +1 on AR accept, -1 on entry fully drained
                    //------------------------------------------------
                    case ({ar_acc, entry_drained})
                        2'b10:   occupancy <= occupancy + 1'b1;
                        2'b01:   occupancy <= occupancy - 1'b1;
                        default: occupancy <= occupancy;
                    endcase

                    if (last_beat)
                        state <= S_DONE;
                end

                S_DONE: begin
                    engine_done <= 1'b1;
                    state       <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // only the low ID_W bits of RID index the ROB; tie off the unused upper bits
    wire _unused_ok = &{1'b0, M_AXI_RID[AXI_ID_WIDTH-1:ID_W]};

`ifdef FORMAL
    // =====================================================================
    // STEP 3 formal: ROB safety properties (SymbiYosys, BMC + IC3/PDR).
    // Proven on a small bounded config set via chparam in the .sby
    // (MAX_OUTSTANDING=2, MAX_BURST_LEN=2, LEN_WIDTH=8).  Environment
    // assumptions (assume) model a LEGAL AXI read slave; DUT assertions
    // (assert) check the engine.  Kept strictly separate.
    // =====================================================================

    // cmd_len is bounded so retire/issue counters stay small; F_MAXLEN exceeds
    // one full ROB fill so entry/tag REUSE (alloc_ptr wrap) is exercised.
    localparam F_MAXLEN = MAX_OUTSTANDING*MAX_BURST_LEN + MAX_BURST_LEN;

    reg f_past_valid = 1'b0;
    always @(posedge clk) f_past_valid <= 1'b1;
    always @(*) if (!f_past_valid) assume (rst);   // clean reset in step 0

    // ---- environment: command is legal (cmd_len in [1, F_MAXLEN]) ----
    always @(*) begin
        assume (cmd_len >= 1);
        assume (cmd_len <= F_MAXLEN);
    end

    // ---- ghost: element index of each entry's first beat (in-order check) ----
    reg [LEN_WIDTH-1:0] g_base [0:MAX_OUTSTANDING-1];
    always @(posedge clk)
        if (ar_acc) g_base[alloc_ptr] <= issue_elem;

    wire [ID_W-1:0] f_rid = M_AXI_RID[ID_W-1:0];

    // ---- environment: a legal AXI read slave ----
    //   * returns R beats only for valid, not-yet-complete entries (allocated IDs)
    //   * same-ID beats in order, RLAST exactly on that ID's expected final beat
    //   * memory[i] == i, so a beat carries element index (base + position)
    always @(*) begin
        if (M_AXI_RVALID) begin
            assume (e_valid[f_rid]);
            assume (!e_complete[f_rid]);
            assume (e_recv[f_rid] < e_exp[f_rid]);
            assume (M_AXI_RLAST == (e_recv[f_rid] == e_exp[f_rid] - 8'd1));
            assume (M_AXI_RDATA[LEN_WIDTH-1:0] == g_base[f_rid] + e_recv[f_rid]);
        end
    end

    // ---- DUT safety assertions (combinational / current-state) ----
    always @(posedge clk) if (f_past_valid && !rst) begin
        a_occ_bound   : assert (occupancy <= MAX_OUTSTANDING);   // no overflow
        a_state_legal : assert (state <= S_DONE);
        if (ar_acc)
            a_tag_no_reuse : assert (!e_valid[alloc_ptr]);       // ID not reused while valid
        if (out_valid) begin
            a_retire_complete : assert (e_valid[head_ptr] && e_complete[head_ptr]);
            a_drain_bound     : assert (drain_idx < e_exp[head_ptr]);
            a_inorder         : assert (out_data[LEN_WIDTH-1:0] ==
                                        retire_elem[LEN_WIDTH-1:0]);  // in-order data
        end
        if (engine_done)
            a_done_timing : assert (retire_elem == cmd_len_r);   // done after all retired
    end

    // ---- DUT assertions needing the previous cycle ($past) ----
    always @(posedge clk) if (f_past_valid && !rst && !$past(rst)) begin
        // output-stream stability while back-pressured
        if ($past(out_valid) && !$past(out_ready)) begin
            a_out_valid_stable : assert (out_valid);
            a_out_data_stable  : assert (out_data == $past(out_data));
            a_out_last_stable  : assert (out_last == $past(out_last));
        end
        // AR valid/payload stability while back-pressured
        if ($past(M_AXI_ARVALID) && !$past(M_AXI_ARREADY)) begin
            a_ar_valid_stable : assert (M_AXI_ARVALID);
            a_ar_addr_stable  : assert (M_AXI_ARADDR == $past(M_AXI_ARADDR));
            a_ar_len_stable   : assert (M_AXI_ARLEN  == $past(M_AXI_ARLEN));
            a_ar_id_stable    : assert (M_AXI_ARID   == $past(M_AXI_ARID));
        end
        // engine_err: sticky *within a command*; cleared only when a new command
        // is accepted (cmd_ready && cmd_valid in S_IDLE).  Also set on a bad RRESP.
        if ($past(engine_err) && !($past(cmd_ready) && $past(cmd_valid)))
            a_err_sticky : assert (engine_err);
        if ($past(r_acc) && ($past(M_AXI_RRESP) != 2'b00))
            a_err_set : assert (engine_err);
    end

    // ---- per-entry counter bounds (no recv overflow; complete => full) ----
    // (labels omitted inside the generate so Yosys auto-names per genvar scope)
    genvar gi;
    generate for (gi = 0; gi < MAX_OUTSTANDING; gi = gi + 1) begin: g_entry
        always @(posedge clk) if (f_past_valid && !rst) begin
            if (e_valid[gi]) begin
                assert (e_recv[gi] <= e_exp[gi]);                          // no recv overflow
                assert (e_exp[gi] >= 8'd1 && e_exp[gi] <= MAX_BURST_LEN);  // exp in range
            end
            if (e_complete[gi])
                assert (e_recv[gi] == e_exp[gi]);                          // complete => full
        end
    end endgenerate

    // ---- induction-strengthening invariants for in-order retirement ----
    // (1) the head entry's base + current drain position == total retired so far
    // (2) the two active entries' bases chain by exp (MAX_OUTSTANDING==2 config)
    always @(posedge clk) if (f_past_valid && !rst) begin
        if (e_valid[head_ptr])
            inv_base_head : assert (g_base[head_ptr] + drain_idx == retire_elem);
        if (occupancy == 2)
            inv_base_chain : assert (g_base[head_ptr + 1'b1] ==
                                     g_base[head_ptr] + e_exp[head_ptr]);
    end

    // ---- cover: useful traces ----
    reg [3:0] g_ar_count;
    reg       g_ooo_seen;
    always @(posedge clk) begin
        if (rst) begin
            g_ar_count <= 4'd0;
            g_ooo_seen <= 1'b0;
        end else begin
            if (ar_acc && g_ar_count != 4'hF) g_ar_count <= g_ar_count + 4'd1;
            // a beat for a non-head entry arrives before the head completes
            if (r_acc && !e_complete[head_ptr] && (rid_idx != head_ptr))
                g_ooo_seen <= 1'b1;
        end
    end
    always @(posedge clk) if (f_past_valid && !rst) begin
        c_two_ar   : cover (g_ar_count == 4'd2);                  // >= two ARs issued
        c_ooo_fill : cover (g_ooo_seen);                          // out-of-order fill
        c_backpres : cover (out_valid && !out_ready);             // output back-pressure
        c_done_ooo : cover (engine_done && g_ooo_seen);           // in-order done after OOO fill
    end
`endif

endmodule
