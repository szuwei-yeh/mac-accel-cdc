`timescale 1ns/1ps

// tb_axi_read_engine_rob.v
// -----------------------------------------------------------------------------
// STEP 1 (Milestone A) standalone testbench for axi_read_engine_rob in its
// multiple-outstanding / single-shared-ID / in-order configuration, driven
// against axi_read_mem_model_mo (multi-outstanding, in-order, latency L).
//
// Does NOT touch any V2 file or the Step-0 baseline files.  Single clock domain
// (the engine is a bus_clk block); no CDC here.
//
// What it checks / measures:
//   * Correctness: the output beat stream equals memory contents in order, and
//     out_last is asserted exactly on the final beat of the phase.
//   * AR valid/payload stability: when ARVALID is high and ARREADY is low,
//     ARVALID stays high with ARADDR/ARLEN stable (stressed via ar_stall inject).
//   * Output-stream stability: when out_valid is high and out_ready is low,
//     out_valid stays high with out_data/out_last stable (stressed via random
//     out_ready back-pressure).
//   * Utilization, using the SAME definition as Step 0:
//       U_bus = beats / T_active      (AXI read-data-bus utilization)
//       U_mac = U_bus / 2             (projected MAC-feed utilization for the
//                                      integrated A-then-B design: 2 beats/pair)
//     T_active = engine_busy cycles (the read-active window).  A single phase's
//     U_bus here is directly comparable to the Step-0 baseline U_bus, which is
//     identical per phase (Step 0: U_bus=13.54%, U_mac=6.77% at L=100).
//
// MAX_OUTSTANDING is a compile-time parameter (ENG_MAX_OUT, default 4).  Sweep it
// by rebuilding with -DENG_MAX_OUT=<n>.
// -----------------------------------------------------------------------------

`ifndef ENG_MAX_OUT
`define ENG_MAX_OUT 4
`endif

module tb_axi_read_engine_rob;

    localparam AXI_DATA_W   = 32;
    localparam AXI_ADDR_W   = 32;
    localparam AXI_ID_WIDTH = 4;
    localparam MAX_BURST    = 16;
    localparam LEN_WIDTH    = 16;
    localparam MEM_WORDS    = 1024;
    localparam MODEL_OUT    = 16;            // memory-model capacity (>= engine)
    localparam MAX_OUT      = `ENG_MAX_OUT;  // engine outstanding count under test

    // Step-0 baseline references at L=100 (for the comparison print)
    localparam BASE_UBUS_H  = 1354;   // 13.54 %  (hundredths)
    localparam BASE_UMAC_H  = 677;    //  6.77 %

    //----------------------------------------------------
    // Clock & reset (single domain)
    //----------------------------------------------------
    reg clk = 0;
    always #5 clk = ~clk;     // 100 MHz
    reg resetn;

    reg [15:0] mem_latency;

    //----------------------------------------------------
    // Engine <-> model AXI nets
    //----------------------------------------------------
    wire [AXI_ID_WIDTH-1:0] ARID;
    wire [AXI_ADDR_W-1:0]   ARADDR;
    wire [7:0]              ARLEN;
    wire [2:0]              ARSIZE;
    wire [1:0]              ARBURST;
    wire                    ARVALID;
    wire                    ARREADY;

    wire [AXI_ID_WIDTH-1:0] RID;
    wire [AXI_DATA_W-1:0]   RDATA;
    wire [1:0]              RRESP;
    wire                    RLAST;
    wire                    RVALID;
    wire                    RREADY;

    //----------------------------------------------------
    // Command + output-stream nets
    //----------------------------------------------------
    reg                   cmd_valid_tb;
    wire                  cmd_ready;
    reg  [AXI_ADDR_W-1:0] cmd_addr_tb;
    reg  [LEN_WIDTH-1:0]  cmd_len_tb;

    wire                  out_valid;
    wire [AXI_DATA_W-1:0] out_data;
    wire                  out_last;

    wire                  engine_busy;
    wire                  engine_done;
    wire                  engine_err;

    // test-controlled back-pressure
    reg                   ready_mode;   // 0 = out_ready always 1, 1 = random
    reg                   out_ready_tb;
    reg                   ar_stall_tb;

    //----------------------------------------------------
    // DUT
    //----------------------------------------------------
    axi_read_engine_rob #(
        .AXI_DATA_W     (AXI_DATA_W),
        .AXI_ADDR_W     (AXI_ADDR_W),
        .AXI_ID_WIDTH   (AXI_ID_WIDTH),
        .MAX_OUTSTANDING(MAX_OUT),
        .MAX_BURST_LEN  (MAX_BURST),
        .LEN_WIDTH      (LEN_WIDTH)
    ) dut (
        .clk          (clk),
        .rst          (~resetn),

        .cmd_valid    (cmd_valid_tb),
        .cmd_ready    (cmd_ready),
        .cmd_addr     (cmd_addr_tb),
        .cmd_len      (cmd_len_tb),

        .M_AXI_ARID   (ARID),
        .M_AXI_ARADDR (ARADDR),
        .M_AXI_ARLEN  (ARLEN),
        .M_AXI_ARSIZE (ARSIZE),
        .M_AXI_ARBURST(ARBURST),
        .M_AXI_ARVALID(ARVALID),
        .M_AXI_ARREADY(ARREADY),

        .M_AXI_RID    (RID),
        .M_AXI_RDATA  (RDATA),
        .M_AXI_RRESP  (RRESP),
        .M_AXI_RLAST  (RLAST),
        .M_AXI_RVALID (RVALID),
        .M_AXI_RREADY (RREADY),

        .out_valid    (out_valid),
        .out_ready    (out_ready_tb),
        .out_data     (out_data),
        .out_last     (out_last),

        .engine_busy  (engine_busy),
        .engine_done  (engine_done),
        .engine_err   (engine_err)
    );

    //----------------------------------------------------
    // Memory model (multi-outstanding, in-order)
    //----------------------------------------------------
    axi_read_mem_model_mo #(
        .AXI_ID_WIDTH   (AXI_ID_WIDTH),
        .AXI_ADDR_WIDTH (AXI_ADDR_W),
        .AXI_DATA_WIDTH (AXI_DATA_W),
        .MEM_WORDS      (MEM_WORDS),
        .MAX_OUTSTANDING(MODEL_OUT)
    ) u_mem (
        .clk        (clk),
        .resetn     (resetn),
        .ar_latency (mem_latency),
        .ar_stall   (ar_stall_tb),

        .arid       (ARID),
        .araddr     (ARADDR),
        .arlen      (ARLEN),
        .arsize     (ARSIZE),
        .arburst    (ARBURST),
        .arvalid    (ARVALID),
        .arready    (ARREADY),

        .rid        (RID),
        .rdata      (RDATA),
        .rresp      (RRESP),
        .rlast      (RLAST),
        .rvalid     (RVALID),
        .rready     (RREADY)
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
            ar_stall_tb  <= ready_mode ? $random : 1'b0;   // inject AR stalls only in bp mode
        end
    end

    //----------------------------------------------------
    // Golden data + preload helper
    //----------------------------------------------------
    reg signed [15:0] golden [0:511];

    task automatic preload_phase(input [31:0] base, input integer len);
        integer k;
        reg signed [15:0] v;
    begin
        for (k = 0; k < len; k = k + 1) begin
            v = k + 1;                       // distinguishable, order-sensitive pattern
            u_mem.mem[(base >> 2) + k] = {{16{v[15]}}, v};
            golden[k] = v;
        end
    end
    endtask

    //----------------------------------------------------
    // Measurement counters (synchronous clear via `clr`)
    //----------------------------------------------------
    integer m_active, m_beats, m_arh;
    reg     clr;

    always @(posedge clk) begin
        if (!resetn || clr) begin
            m_active <= 0;
            m_beats  <= 0;
            m_arh    <= 0;
        end else begin
            if (engine_busy)          m_active <= m_active + 1;
            if (ARVALID && ARREADY)   m_arh    <= m_arh + 1;
            if (out_valid && out_ready_tb) m_beats <= m_beats + 1;
        end
    end

    //----------------------------------------------------
    // Output-stream correctness checker (order + out_last alignment)
    //----------------------------------------------------
    integer o_idx;
    integer chk_err, last_err;
    integer chk_len;     // expected phase length

    always @(posedge clk) begin
        if (!resetn || clr) begin
            o_idx    <= 0;
            chk_err  <= 0;
            last_err <= 0;
        end else if (out_valid && out_ready_tb) begin
            if (out_data[15:0] !== golden[o_idx][15:0]) begin
                chk_err <= chk_err + 1;
                $display("  [DATA MISMATCH] beat %0d: got %0d exp %0d",
                         o_idx, $signed(out_data[15:0]), golden[o_idx]);
            end
            // out_last must be high exactly on the final beat
            if (o_idx == chk_len - 1) begin
                if (!out_last) last_err <= last_err + 1;
            end else begin
                if (out_last)  last_err <= last_err + 1;
            end
            o_idx <= o_idx + 1;
        end
    end

    //----------------------------------------------------
    // Handshake-stability monitors (AR channel + output stream)
    //----------------------------------------------------
    integer ar_stab_err, out_stab_err;
    reg     prev_ov, prev_or, prev_ol;
    reg [AXI_DATA_W-1:0] prev_od;
    reg     prev_arv, prev_arr;
    reg [AXI_ADDR_W-1:0] prev_araddr;
    reg [7:0] prev_arlen;
    reg     mon_init;

    always @(posedge clk) begin
        if (!resetn) begin
            ar_stab_err <= 0;
            out_stab_err<= 0;
            mon_init    <= 0;
            prev_ov<=0; prev_or<=0; prev_ol<=0; prev_od<=0;
            prev_arv<=0; prev_arr<=0; prev_araddr<=0; prev_arlen<=0;
        end else begin
            if (mon_init) begin
                // output stream: held stable while valid && !ready
                if (prev_ov && !prev_or) begin
                    if (!out_valid)                 out_stab_err <= out_stab_err + 1;
                    else if (out_data !== prev_od)  out_stab_err <= out_stab_err + 1;
                    else if (out_last !== prev_ol)  out_stab_err <= out_stab_err + 1;
                end
                // AR channel: held stable while valid && !ready
                if (prev_arv && !prev_arr) begin
                    if (!ARVALID)                    ar_stab_err <= ar_stab_err + 1;
                    else if (ARADDR !== prev_araddr) ar_stab_err <= ar_stab_err + 1;
                    else if (ARLEN  !== prev_arlen)  ar_stab_err <= ar_stab_err + 1;
                end
            end
            prev_ov<=out_valid; prev_or<=out_ready_tb; prev_ol<=out_last; prev_od<=out_data;
            prev_arv<=ARVALID; prev_arr<=ARREADY; prev_araddr<=ARADDR; prev_arlen<=ARLEN;
            mon_init<=1;
        end
    end

    //----------------------------------------------------
    // Reset
    //----------------------------------------------------
    task do_reset;
    begin
        resetn       = 1'b0;
        cmd_valid_tb = 1'b0;
        cmd_addr_tb  = 0;
        cmd_len_tb   = 0;
        ready_mode   = 1'b0;
        clr          = 1'b0;
        mem_latency  = 16'd100;
        repeat (6) @(posedge clk);
        @(posedge clk); #1; resetn = 1'b1;
        repeat (4) @(posedge clk);
    end
    endtask

    //----------------------------------------------------
    // Bookkeeping
    //----------------------------------------------------
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    integer head_ubus_h, head_umac_h, head_active;

    //----------------------------------------------------
    // One measured phase
    //----------------------------------------------------
    task run_phase(
        input [31:0]  base,
        input integer len,
        input [15:0]  latency,
        input         bp_mode,        // 0 = clean throughput, 1 = random back-pressure
        input         is_headline
    );
        integer ubus_h, umac_h;
        integer timeout;
    begin
        // configure + preload
        mem_latency = latency;
        chk_len     = len;
        preload_phase(base, len);

        // clear measurement state
        @(posedge clk); clr <= 1'b1;
        @(posedge clk); clr <= 1'b0;
        ready_mode <= bp_mode;

        // issue command
        cmd_addr_tb <= base;
        cmd_len_tb  <= len[LEN_WIDTH-1:0];
        cmd_valid_tb<= 1'b1;
        do @(posedge clk); while (!cmd_ready);   // accepted when cmd_ready seen high
        cmd_valid_tb<= 1'b0;

        // wait for completion (with timeout)
        timeout = 0;
        while (!engine_done && timeout < 200000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        ready_mode <= 1'b0;     // restore clean ready between phases
        @(posedge clk);

        ubus_h = (m_beats * 10000) / m_active;
        umac_h = ubus_h / 2;

        $display("---- MAX_OUT=%0d  N=%0d  L=%0d  %s ----",
                 MAX_OUT, len, latency, bp_mode ? "(random back-pressure)" : "(throughput)");

        if (timeout >= 200000) begin
            $display("  TIMEOUT");
            fail_cnt = fail_cnt + 1;
        end else if (chk_err != 0 || last_err != 0 || engine_err) begin
            $display("  FAIL: data_err=%0d last_err=%0d engine_err=%0b",
                     chk_err, last_err, engine_err);
            fail_cnt = fail_cnt + 1;
        end else if (out_stab_err != 0 || ar_stab_err != 0) begin
            $display("  FAIL: out_stab_err=%0d ar_stab_err=%0d", out_stab_err, ar_stab_err);
            fail_cnt = fail_cnt + 1;
        end else if (m_beats != len) begin
            $display("  FAIL: beats=%0d != N=%0d", m_beats, len);
            fail_cnt = fail_cnt + 1;
        end else begin
            pass_cnt = pass_cnt + 1;
            $display("  PASS  data+order OK, out_last OK, AR/stream stable");
            $display("  T_active=%0d  AR bursts=%0d  beats=%0d", m_active, m_arh, m_beats);
            if (!bp_mode) begin
                $display("  U_bus = %0d.%02d %%   U_mac(=/2) = %0d.%02d %%",
                         ubus_h/100, ubus_h%100, umac_h/100, umac_h%100);
                if (is_headline) begin
                    head_ubus_h = ubus_h;
                    head_umac_h = umac_h;
                    head_active = m_active;
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
        $display("  STEP 1 (Milestone A): multiple outstanding, shared ID, in-order");
        $display("  engine MAX_OUTSTANDING = %0d   burst = %0d beats", MAX_OUT, MAX_BURST);
        $display("  U_bus = beats/T_active   U_mac = U_bus/2 (A-then-B projection)");
        $display("  Step-0 baseline @L=100:  U_bus=13.54%%  U_mac=6.77%%");
        $display("==================================================");
        $display("");

        do_reset;

        // Headline: same N as the Step-0 headline (64), L=100, clean throughput
        run_phase(32'h0000_0000, 64,  16'd100, 1'b0, 1'b1);

        // Steady-state: many bursts (16) so larger MAX_OUT is fully exercised
        run_phase(32'h0000_0000, 256, 16'd100, 1'b0, 1'b0);

        // Small single-burst-per-... sanity
        run_phase(32'h0000_0000, 16,  16'd100, 1'b0, 1'b0);

        // Correctness + stability under random AR + output back-pressure
        run_phase(32'h0000_0000, 64,  16'd100, 1'b1, 1'b0);

        $display("==================================================");
        $display("  TOTAL: %0d PASS / %0d FAIL", pass_cnt, fail_cnt);
        $display("  MILESTONE A (MAX_OUT=%0d, N=64, L=100):", MAX_OUT);
        $display("    U_bus = %0d.%02d %%  (baseline 13.54%%)   U_mac = %0d.%02d %%  (baseline 6.77%%)",
                 head_ubus_h/100, head_ubus_h%100, head_umac_h/100, head_umac_h%100);
        $display("==================================================");

        #50;
        $finish;
    end

    initial begin
        #20000000;
        $display("[TB] global timeout");
        $finish;
    end

endmodule
