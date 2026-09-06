`timescale 1ns/1ps

// tb_axi_read_engine_rob_ooo.v
// -----------------------------------------------------------------------------
// STEP 2 (Milestone B) standalone testbench: proves the reorder buffer in
// axi_read_engine_rob restores a strictly in-order out_valid/out_data/out_last
// stream despite OUT-OF-ORDER / INTERLEAVED R responses from
// axi_read_mem_model_ooo.  Single clock domain; does NOT touch any V2 or Step-0/1
// regression file.
//
// Checks per run:
//   * Correctness: output beats == memory contents IN ORDER (golden), and
//     out_last asserted exactly on the final phase beat -- regardless of the
//     order/interleave in which responses arrived.
//   * Non-vacuity: counts response INTERLEAVE events (consecutive accepted beats
//     with different RID) and OUT-OF-ORDER burst COMPLETIONS (a burst's RLAST
//     arriving before an earlier-allocated burst's).  In ooo_en=1 these must be
//     > 0, otherwise the ROB would not actually be reordering anything.
//   * AR + output-stream stability under random AR + output back-pressure.
//   * Accepted AR address sequence, burst geometry, and command-beat
//     conservation from an external command/handshake reference model.
//   * Utilization, same definition as Step 0/1: U_bus = beats/T_active,
//     U_mac = U_bus/2.
//
// MAX_OUTSTANDING (engine == model) set by -DENG_MAX_OUT=<n> (default 4).
// -----------------------------------------------------------------------------

`ifndef ENG_MAX_OUT
`define ENG_MAX_OUT 4
`endif

module tb_axi_read_engine_rob_ooo;

    localparam AXI_DATA_W   = 32;
    localparam AXI_ADDR_W   = 32;
    localparam AXI_ID_WIDTH = 4;
    localparam MAX_BURST    = 16;
    localparam LEN_WIDTH    = 16;
    localparam MEM_WORDS    = 1024;
    localparam MAX_OUT      = `ENG_MAX_OUT;   // engine AND model outstanding count

    //----------------------------------------------------
    // Clock & reset
    //----------------------------------------------------
    reg clk = 0;
    always #5 clk = ~clk;
    reg resetn;

    reg [15:0] mem_latency;
    reg        ooo_mode;      // model ooo_en
    reg        ready_mode;    // 0 = clean, 1 = random out_ready + ar_stall

    //----------------------------------------------------
    // Engine <-> model AXI nets
    //----------------------------------------------------
    wire [AXI_ID_WIDTH-1:0] ARID;
    wire [AXI_ADDR_W-1:0]   ARADDR;
    wire [7:0]              ARLEN;
    wire [2:0]              ARSIZE;
    wire [1:0]              ARBURST;
    wire                    ARVALID, ARREADY;

    wire [AXI_ID_WIDTH-1:0] RID;
    wire [AXI_DATA_W-1:0]   RDATA;
    wire [1:0]              RRESP;
    wire                    RLAST, RVALID, RREADY;

    reg                   cmd_valid_tb;
    wire                  cmd_ready;
    reg  [AXI_ADDR_W-1:0] cmd_addr_tb;
    reg  [LEN_WIDTH-1:0]  cmd_len_tb;

    wire                  out_valid;
    wire [AXI_DATA_W-1:0] out_data;
    wire                  out_last;
    wire                  engine_busy, engine_done, engine_err;

    reg                   out_ready_tb;
    reg                   ar_stall_tb;

    //----------------------------------------------------
    // DUT (ROB engine, unique IDs)
    //----------------------------------------------------
    axi_read_engine_rob #(
        .AXI_DATA_W     (AXI_DATA_W),
        .AXI_ADDR_W     (AXI_ADDR_W),
        .AXI_ID_WIDTH   (AXI_ID_WIDTH),
        .MAX_OUTSTANDING(MAX_OUT),
        .MAX_BURST_LEN  (MAX_BURST),
        .LEN_WIDTH      (LEN_WIDTH)
    ) dut (
        .clk(clk), .rst(~resetn),
        .cmd_valid(cmd_valid_tb), .cmd_ready(cmd_ready),
        .cmd_addr(cmd_addr_tb), .cmd_len(cmd_len_tb),
        .M_AXI_ARID(ARID), .M_AXI_ARADDR(ARADDR), .M_AXI_ARLEN(ARLEN),
        .M_AXI_ARSIZE(ARSIZE), .M_AXI_ARBURST(ARBURST),
        .M_AXI_ARVALID(ARVALID), .M_AXI_ARREADY(ARREADY),
        .M_AXI_RID(RID), .M_AXI_RDATA(RDATA), .M_AXI_RRESP(RRESP),
        .M_AXI_RLAST(RLAST), .M_AXI_RVALID(RVALID), .M_AXI_RREADY(RREADY),
        .out_valid(out_valid), .out_ready(out_ready_tb),
        .out_data(out_data), .out_last(out_last),
        .engine_busy(engine_busy), .engine_done(engine_done), .engine_err(engine_err)
    );

    //----------------------------------------------------
    // Out-of-order memory model (slots indexed by ARID)
    //----------------------------------------------------
    axi_read_mem_model_ooo #(
        .AXI_ID_WIDTH   (AXI_ID_WIDTH),
        .AXI_ADDR_WIDTH (AXI_ADDR_W),
        .AXI_DATA_WIDTH (AXI_DATA_W),
        .MEM_WORDS      (MEM_WORDS),
        .MAX_OUTSTANDING(MAX_OUT)
    ) u_mem (
        .clk(clk), .resetn(resetn), .ar_latency(mem_latency),
        .ar_stall(ar_stall_tb), .ooo_en(ooo_mode),
        .arid(ARID), .araddr(ARADDR), .arlen(ARLEN), .arsize(ARSIZE),
        .arburst(ARBURST), .arvalid(ARVALID), .arready(ARREADY),
        .rid(RID), .rdata(RDATA), .rresp(RRESP),
        .rlast(RLAST), .rvalid(RVALID), .rready(RREADY)
    );

    //----------------------------------------------------
    // out_ready / ar_stall drivers
    //----------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            out_ready_tb <= 1'b1;
            ar_stall_tb  <= 1'b0;
        end else begin
            out_ready_tb <= ready_mode ? $random : 1'b1;
            ar_stall_tb  <= ready_mode ? $random : 1'b0;
        end
    end

    //----------------------------------------------------
    // Golden data + preload
    //----------------------------------------------------
    reg signed [15:0] golden [0:511];
    task automatic preload_phase(input [31:0] base, input integer len);
        integer kk; reg signed [15:0] v;
    begin
        for (kk = 0; kk < len; kk = kk + 1) begin
            v = kk + 1;
            u_mem.mem[((base >> 2) + kk) & (MEM_WORDS-1)] = {{16{v[15]}}, v};
            golden[kk] = v;
        end
    end
    endtask

    //----------------------------------------------------
    // Measurement + checkers (synchronous clear via `clr`)
    //----------------------------------------------------
    integer m_active, m_beats, m_arh;
    integer o_idx, chk_err, last_err, chk_len;
    reg     clr;

    // Independent AR request-sequence scoreboard.  This uses only the command
    // handshake and accepted AR payloads; it does not inspect DUT internals.
    wire [8:0] ar_seq_burst_beats = {1'b0, ARLEN} + 9'd1;
    wire [LEN_WIDTH:0] ar_seq_burst_beats_ext =
        {{(LEN_WIDTH-8){1'b0}}, ar_seq_burst_beats};
    wire [AXI_ADDR_W-1:0] ar_seq_stride =
        {{(AXI_ADDR_W-9){1'b0}}, ar_seq_burst_beats} << ARSIZE;
    reg [AXI_ADDR_W-1:0] ar_seq_expected_addr;
    reg [LEN_WIDTH:0] ar_seq_command_beats;
    reg [LEN_WIDTH:0] ar_seq_issued_beats;
    wire [LEN_WIDTH:0] ar_seq_remaining =
        ar_seq_command_beats - ar_seq_issued_beats;
    wire [LEN_WIDTH:0] ar_seq_expected_burst =
        (ar_seq_remaining > MAX_BURST) ? MAX_BURST : ar_seq_remaining;
    wire [LEN_WIDTH:0] ar_seq_issued_after =
        ar_seq_issued_beats + ar_seq_burst_beats_ext;
    integer ar_seq_err;
    reg     ar_seq_active;

    always @(posedge clk) begin
        if (!resetn || clr) begin
            ar_seq_expected_addr  <= {AXI_ADDR_W{1'b0}};
            ar_seq_command_beats  <= 0;
            ar_seq_issued_beats   <= 0;
            ar_seq_err            <= 0;
            ar_seq_active         <= 1'b0;
        end else begin
            if (cmd_valid_tb && cmd_ready) begin
                ar_seq_expected_addr <= cmd_addr_tb;
                ar_seq_command_beats <= {1'b0, cmd_len_tb};
                ar_seq_issued_beats  <= 0;
                ar_seq_active        <= 1'b1;
            end

            if (ARVALID && ARREADY) begin
                if (!ar_seq_active ||
                    ARADDR !== ar_seq_expected_addr ||
                    ARSIZE !== 3'b010 ||
                    ARBURST !== 2'b01 ||
                    ar_seq_burst_beats_ext < 1 ||
                    ar_seq_burst_beats_ext > MAX_BURST ||
                    ar_seq_burst_beats_ext != ar_seq_expected_burst ||
                    ar_seq_issued_after > ar_seq_command_beats) begin
                    ar_seq_err <= ar_seq_err + 1;
                    $display("  [AR SEQUENCE ERROR] got addr=%h beats=%0d size=%0d burst=%0d; exp addr=%h beats=%0d issued=%0d/%0d",
                             ARADDR, ar_seq_burst_beats, ARSIZE, ARBURST,
                             ar_seq_expected_addr, ar_seq_expected_burst,
                             ar_seq_issued_beats, ar_seq_command_beats);
                end
                ar_seq_expected_addr <= ar_seq_expected_addr + ar_seq_stride;
                ar_seq_issued_beats  <= ar_seq_issued_after;
            end

            if (engine_done) begin
                if (!ar_seq_active || ar_seq_issued_beats != ar_seq_command_beats) begin
                    ar_seq_err <= ar_seq_err + 1;
                    $display("  [AR CONSERVATION ERROR] issued=%0d command=%0d",
                             ar_seq_issued_beats, ar_seq_command_beats);
                end
                ar_seq_active <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (!resetn || clr) begin
            m_active <= 0; m_beats <= 0; m_arh <= 0;
        end else begin
            if (engine_busy)               m_active <= m_active + 1;
            if (ARVALID && ARREADY)        m_arh    <= m_arh + 1;
            if (out_valid && out_ready_tb) m_beats  <= m_beats + 1;
        end
    end

    // output correctness: in-order data + out_last alignment
    always @(posedge clk) begin
        if (!resetn || clr) begin
            o_idx <= 0; chk_err <= 0; last_err <= 0;
        end else if (out_valid && out_ready_tb) begin
            if (out_data[15:0] !== golden[o_idx][15:0]) begin
                chk_err <= chk_err + 1;
                $display("  [DATA MISMATCH] beat %0d: got %0d exp %0d",
                         o_idx, $signed(out_data[15:0]), golden[o_idx]);
            end
            if (o_idx == chk_len - 1) begin
                if (!out_last) last_err <= last_err + 1;
            end else if (out_last) begin
                last_err <= last_err + 1;
            end
            o_idx <= o_idx + 1;
        end
    end

    //----------------------------------------------------
    // OOO observability: interleave events + out-of-order completions
    //----------------------------------------------------
    integer interleave_cnt, ooo_complete_cnt;
    reg [AXI_ID_WIDTH-1:0] prev_rid;
    reg                    seen_beat;
    integer alloc_ctr, max_done_seq;
    reg                    seen_done;
    integer alloc_seq [0:15];   // alloc order per ID (MAX_OUT <= 16)

    always @(posedge clk) begin
        if (!resetn || clr) begin
            interleave_cnt   <= 0;
            ooo_complete_cnt <= 0;
            seen_beat        <= 1'b0;
            prev_rid         <= 0;
            alloc_ctr        <= 0;
            max_done_seq     <= 0;
            seen_done        <= 1'b0;
        end else begin
            // record allocation order on AR handshake
            if (ARVALID && ARREADY) begin
                alloc_seq[ARID] <= alloc_ctr;
                alloc_ctr       <= alloc_ctr + 1;
            end
            // observe R beats
            if (RVALID && RREADY) begin
                if (seen_beat && (RID !== prev_rid))
                    interleave_cnt <= interleave_cnt + 1;
                prev_rid  <= RID;
                seen_beat <= 1'b1;
                if (RLAST) begin
                    if (seen_done && (alloc_seq[RID] < max_done_seq))
                        ooo_complete_cnt <= ooo_complete_cnt + 1;
                    if (!seen_done || (alloc_seq[RID] > max_done_seq))
                        max_done_seq <= alloc_seq[RID];
                    seen_done <= 1'b1;
                end
            end
        end
    end

    //----------------------------------------------------
    // Stability monitors (AR + output stream)
    //----------------------------------------------------
    integer ar_stab_err, out_stab_err;
    reg     prev_ov, prev_or, prev_ol;
    reg [AXI_DATA_W-1:0] prev_od;
    reg     prev_arv, prev_arr;
    reg [AXI_ADDR_W-1:0] prev_araddr;
    reg [7:0] prev_arlen;
    reg [AXI_ID_WIDTH-1:0] prev_arid;
    reg     mon_init;

    always @(posedge clk) begin
        if (!resetn) begin
            ar_stab_err<=0; out_stab_err<=0; mon_init<=0;
            prev_ov<=0; prev_or<=0; prev_ol<=0; prev_od<=0;
            prev_arv<=0; prev_arr<=0; prev_araddr<=0; prev_arlen<=0; prev_arid<=0;
        end else begin
            if (mon_init) begin
                if (prev_ov && !prev_or) begin
                    if (!out_valid)                out_stab_err <= out_stab_err + 1;
                    else if (out_data !== prev_od) out_stab_err <= out_stab_err + 1;
                    else if (out_last !== prev_ol) out_stab_err <= out_stab_err + 1;
                end
                if (prev_arv && !prev_arr) begin
                    if (!ARVALID)                    ar_stab_err <= ar_stab_err + 1;
                    else if (ARADDR !== prev_araddr) ar_stab_err <= ar_stab_err + 1;
                    else if (ARLEN  !== prev_arlen)  ar_stab_err <= ar_stab_err + 1;
                    else if (ARID   !== prev_arid)   ar_stab_err <= ar_stab_err + 1;
                end
            end
            prev_ov<=out_valid; prev_or<=out_ready_tb; prev_ol<=out_last; prev_od<=out_data;
            prev_arv<=ARVALID; prev_arr<=ARREADY; prev_araddr<=ARADDR;
            prev_arlen<=ARLEN; prev_arid<=ARID;
            mon_init<=1;
        end
    end

    //----------------------------------------------------
    // Reset
    //----------------------------------------------------
    task do_reset;
    begin
        resetn=0; cmd_valid_tb=0; cmd_addr_tb=0; cmd_len_tb=0;
        ready_mode=0; ooo_mode=1; clr=0; mem_latency=16'd100;
        repeat (6) @(posedge clk);
        @(posedge clk); #1; resetn=1;
        repeat (4) @(posedge clk);
    end
    endtask

    integer pass_cnt = 0, fail_cnt = 0;
    integer head_ubus_h, head_umac_h, head_active;

    //----------------------------------------------------
    // One measured phase
    //----------------------------------------------------
    task run_phase(
        input [31:0]  base,
        input integer len,
        input [15:0]  latency,
        input         ooo_en_v,
        input         bp_mode,
        input         is_headline
    );
        integer ubus_h, umac_h, timeout;
    begin
        mem_latency = latency;
        chk_len     = len;
        ooo_mode    = ooo_en_v;
        preload_phase(base, len);

        @(posedge clk); clr <= 1'b1;
        @(posedge clk); clr <= 1'b0;
        ready_mode <= bp_mode;

        cmd_addr_tb <= base;
        cmd_len_tb  <= len[LEN_WIDTH-1:0];
        cmd_valid_tb<= 1'b1;
        do @(posedge clk); while (!cmd_ready);
        cmd_valid_tb<= 1'b0;

        timeout = 0;
        while (!engine_done && timeout < 200000) begin
            @(posedge clk); timeout = timeout + 1;
        end
        ready_mode <= 1'b0;
        @(posedge clk);

        ubus_h = (m_beats * 10000) / m_active;
        umac_h = ubus_h / 2;

        $display("---- MAX_OUT=%0d N=%0d L=%0d  ooo_en=%0d %s ----",
                 MAX_OUT, len, latency, ooo_en_v, bp_mode ? "(back-pressure)" : "(throughput)");
        $display("  OOO observed: interleave=%0d  out-of-order-completions=%0d",
                 interleave_cnt, ooo_complete_cnt);

        if (timeout >= 200000) begin
            $display("  FAIL: TIMEOUT");  fail_cnt = fail_cnt + 1;
        end else if (chk_err != 0 || last_err != 0 || engine_err) begin
            $display("  FAIL: data_err=%0d last_err=%0d engine_err=%0b",
                     chk_err, last_err, engine_err);
            fail_cnt = fail_cnt + 1;
        end else if (out_stab_err != 0 || ar_stab_err != 0 || ar_seq_err != 0) begin
            $display("  FAIL: out_stab_err=%0d ar_stab_err=%0d ar_seq_err=%0d",
                     out_stab_err, ar_stab_err, ar_seq_err);
            fail_cnt = fail_cnt + 1;
        end else if (m_beats != len ||
                     ar_seq_issued_beats != {1'b0, len[LEN_WIDTH-1:0]} ||
                     m_arh != ((len + MAX_BURST - 1) / MAX_BURST)) begin
            $display("  FAIL: beats=%0d issued=%0d AR=%0d expected beats=%0d AR=%0d",
                     m_beats, ar_seq_issued_beats, m_arh, len,
                     (len + MAX_BURST - 1) / MAX_BURST);
            fail_cnt = fail_cnt + 1;
        end else if (ooo_en_v && !bp_mode && len > MAX_BURST && interleave_cnt == 0) begin
            // a multi-burst ooo run must show interleaved responses, else the ROB
            // demux/reorder path was never exercised (out-of-order COMPLETION is a
            // stronger, probabilistic property reported separately above)
            $display("  FAIL: ooo run was vacuous (no interleaved responses)");
            fail_cnt = fail_cnt + 1;
        end else begin
            pass_cnt = pass_cnt + 1;
            $display("  PASS  in-order output; AR sequence/geometry/conservation/stability OK");
            $display("  T_active=%0d  AR bursts=%0d  beats=%0d", m_active, m_arh, m_beats);
            if (!bp_mode) begin
                $display("  U_bus = %0d.%02d %%   U_mac(=/2) = %0d.%02d %%",
                         ubus_h/100, ubus_h%100, umac_h/100, umac_h%100);
                if (is_headline) begin
                    head_ubus_h=ubus_h; head_umac_h=umac_h; head_active=m_active;
                end
            end
        end
        $display("");
    end
    endtask

    //----------------------------------------------------
    // Stimulus
    //----------------------------------------------------
    initial begin
        $display("==================================================");
        $display("  STEP 2 (Milestone B): unique ARID + reorder buffer");
        $display("  engine/model MAX_OUTSTANDING = %0d   burst = %0d beats", MAX_OUT, MAX_BURST);
        $display("  ROB restores in-order output from out-of-order/interleaved R");
        $display("  Step-0 baseline @L=100: U_bus=13.54%%  U_mac=6.77%%");
        $display("==================================================");
        $display("");

        do_reset;

        // Headline: out-of-order responses, N=64, L=100, clean throughput
        run_phase(32'h0000_0000, 64,  16'd100, 1'b1, 1'b0, 1'b1);

        // Steady-state, many bursts, out of order
        run_phase(32'h0000_0000, 256, 16'd100, 1'b1, 1'b0, 1'b0);

        // Multi-burst with a PARTIAL final burst (16 + 4), out of order
        run_phase(32'h0000_0000, 20,  16'd100, 1'b1, 1'b0, 1'b0);

        // Out-of-order + random AR/output back-pressure (stability + correctness)
        run_phase(32'h0000_0000, 64,  16'd100, 1'b1, 1'b1, 1'b0);

        // Burst-granular reorder variant (ooo_en=0)
        run_phase(32'h0000_0000, 64,  16'd100, 1'b0, 1'b0, 1'b0);

        // Directed AR geometry/address boundaries.  Commands are consecutive
        // without global reset.  High non-zero bases exercise address carries.
        run_phase(32'h1234_00FC, 1,  16'd3,   1'b1, 1'b0, 1'b0);
        run_phase(32'h1234_0180, 15, 16'd20,  1'b1, 1'b0, 1'b0);
        run_phase(32'h1234_0280, 16, 16'd20,  1'b1, 1'b1, 1'b0);
        run_phase(32'h1234_0FC0, 17, 16'd40,  1'b1, 1'b1, 1'b0);
        run_phase(32'h5678_02F0, 31, 16'd40,  1'b1, 1'b0, 1'b0);
        run_phase(32'h5678_037C, 32, 16'd40,  1'b1, 1'b1, 1'b0);
        run_phase(32'h9ABC_0F7C, 33, 16'd100, 1'b1, 1'b1, 1'b0);

        $display("==================================================");
        $display("  TOTAL: %0d PASS / %0d FAIL", pass_cnt, fail_cnt);
        $display("  MILESTONE B (MAX_OUT=%0d, N=64, L=100, out-of-order):", MAX_OUT);
        $display("    U_bus = %0d.%02d %%  (baseline 13.54%%)   U_mac = %0d.%02d %%  (baseline 6.77%%)",
                 head_ubus_h/100, head_ubus_h%100, head_umac_h/100, head_umac_h%100);
        $display("==================================================");

        #50; $finish;
    end

    initial begin
        #20000000;
        $display("[TB] global timeout");
        $finish;
    end

endmodule
