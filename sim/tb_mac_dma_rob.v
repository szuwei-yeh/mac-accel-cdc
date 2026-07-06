`timescale 1ns/1ps

// tb_mac_dma_rob.v
// -----------------------------------------------------------------------------
// Experimental integration test for mac_dma_rob.
//
// This is still standalone DMA-level verification, not top-level integration:
// it checks that the formally verified axi_read_engine_rob can be reused for the
// A phase and B phase, preserving the existing A-then-B pairing contract:
//   A phase -> fill buf_a
//   B phase -> pair B beats with buffered A and emit {last,a,b}
//
// The AXI memory model returns out-of-order/interleaved responses, so the ROB
// must restore in-order phase streams before pairing.
// -----------------------------------------------------------------------------

`ifndef DMA_ROB_MAX_OUT
`define DMA_ROB_MAX_OUT 4
`endif

module tb_mac_dma_rob;

    localparam DATA_WIDTH      = 16;
    localparam AXI_DATA_WIDTH  = 32;
    localparam AXI_ADDR_WIDTH  = 32;
    localparam AXI_ID_WIDTH    = 4;
    localparam MAX_LEN         = 256;
    localparam BURST_LEN       = 16;
    localparam MAX_OUT         = `DMA_ROB_MAX_OUT;
    localparam MEM_WORDS       = 2048;

    //----------------------------------------------------
    // Clock/reset
    //----------------------------------------------------
    reg clk = 0;
    always #5 clk = ~clk;

    reg rst;
    reg [15:0] mem_latency;
    reg        ooo_mode;
    reg        ready_mode;

    //----------------------------------------------------
    // DUT control
    //----------------------------------------------------
    reg                         start;
    reg [AXI_ADDR_WIDTH-1:0]    src_a_addr;
    reg [AXI_ADDR_WIDTH-1:0]    src_b_addr;
    reg [15:0]                  length;
    wire                        dma_busy;
    wire                        dma_done;
    wire                        dma_err;

    //----------------------------------------------------
    // AXI wires
    //----------------------------------------------------
    wire [AXI_ID_WIDTH-1:0]     M_AXI_ARID;
    wire [AXI_ADDR_WIDTH-1:0]   M_AXI_ARADDR;
    wire [7:0]                  M_AXI_ARLEN;
    wire [2:0]                  M_AXI_ARSIZE;
    wire [1:0]                  M_AXI_ARBURST;
    wire                        M_AXI_ARVALID;
    wire                        M_AXI_ARREADY;

    wire [AXI_ID_WIDTH-1:0]     M_AXI_RID;
    wire [AXI_DATA_WIDTH-1:0]   M_AXI_RDATA;
    wire [1:0]                  M_AXI_RRESP;
    wire                        M_AXI_RLAST;
    wire                        M_AXI_RVALID;
    wire                        M_AXI_RREADY;

    //----------------------------------------------------
    // Output stream
    //----------------------------------------------------
    wire                          stream_valid;
    wire                          stream_last;
    wire signed [DATA_WIDTH-1:0]  stream_a;
    wire signed [DATA_WIDTH-1:0]  stream_b;
    reg                           stream_ready;
    reg                           ar_stall;

    mac_dma_rob #(
        .DATA_WIDTH      (DATA_WIDTH),
        .AXI_DATA_WIDTH  (AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH  (AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH    (AXI_ID_WIDTH),
        .MAX_LEN         (MAX_LEN),
        .BURST_LEN       (BURST_LEN),
        .MAX_OUTSTANDING (MAX_OUT)
    ) dut (
        .clk             (clk),
        .rst             (rst),
        .start           (start),
        .src_a_addr      (src_a_addr),
        .src_b_addr      (src_b_addr),
        .length          (length),
        .dma_busy        (dma_busy),
        .dma_done        (dma_done),
        .dma_err         (dma_err),

        .M_AXI_ARID      (M_AXI_ARID),
        .M_AXI_ARADDR    (M_AXI_ARADDR),
        .M_AXI_ARLEN     (M_AXI_ARLEN),
        .M_AXI_ARSIZE    (M_AXI_ARSIZE),
        .M_AXI_ARBURST   (M_AXI_ARBURST),
        .M_AXI_ARVALID   (M_AXI_ARVALID),
        .M_AXI_ARREADY   (M_AXI_ARREADY),

        .M_AXI_RID       (M_AXI_RID),
        .M_AXI_RDATA     (M_AXI_RDATA),
        .M_AXI_RRESP     (M_AXI_RRESP),
        .M_AXI_RLAST     (M_AXI_RLAST),
        .M_AXI_RVALID    (M_AXI_RVALID),
        .M_AXI_RREADY    (M_AXI_RREADY),

        .stream_valid    (stream_valid),
        .stream_last     (stream_last),
        .stream_a        (stream_a),
        .stream_b        (stream_b),
        .stream_ready    (stream_ready)
    );

    axi_read_mem_model_ooo #(
        .AXI_ID_WIDTH    (AXI_ID_WIDTH),
        .AXI_ADDR_WIDTH  (AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH  (AXI_DATA_WIDTH),
        .MEM_WORDS       (MEM_WORDS),
        .MAX_OUTSTANDING (MAX_OUT)
    ) u_mem (
        .clk             (clk),
        .resetn          (~rst),
        .ar_latency      (mem_latency),
        .ar_stall        (ar_stall),
        .ooo_en          (ooo_mode),

        .arid            (M_AXI_ARID),
        .araddr          (M_AXI_ARADDR),
        .arlen           (M_AXI_ARLEN),
        .arsize          (M_AXI_ARSIZE),
        .arburst         (M_AXI_ARBURST),
        .arvalid         (M_AXI_ARVALID),
        .arready         (M_AXI_ARREADY),

        .rid             (M_AXI_RID),
        .rdata           (M_AXI_RDATA),
        .rresp           (M_AXI_RRESP),
        .rlast           (M_AXI_RLAST),
        .rvalid          (M_AXI_RVALID),
        .rready          (M_AXI_RREADY)
    );

    //----------------------------------------------------
    // Test-controlled backpressure
    //----------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            stream_ready <= 1'b1;
            ar_stall     <= 1'b0;
        end else begin
            stream_ready <= ready_mode ? $random : 1'b1;
            ar_stall     <= ready_mode ? $random : 1'b0;
        end
    end

    //----------------------------------------------------
    // Golden vectors + memory preload
    //----------------------------------------------------
    reg signed [15:0] golden_a [0:MAX_LEN-1];
    reg signed [15:0] golden_b [0:MAX_LEN-1];

    task automatic mem_write_h(input [31:0] byte_addr, input signed [15:0] val);
    begin
        u_mem.mem[(byte_addr >> 2) & (MEM_WORDS-1)] = {{16{val[15]}}, val};
    end
    endtask

    task automatic preload_vectors(input [31:0] a_base, input [31:0] b_base,
                                   input integer len, input integer pattern);
        integer k;
        reg signed [15:0] av;
        reg signed [15:0] bv;
    begin
        for (k = 0; k < len; k = k + 1) begin
            case (pattern)
                0: begin
                    av = k + 1;
                    bv = 16'sd2;
                end
                1: begin
                    av = -16'sd1 * (k + 1);
                    bv = k[0] ? -16'sd3 : 16'sd4;
                end
                default: begin
                    av = (k % 7) - 3;
                    bv = (k % 5) + 1;
                end
            endcase
            golden_a[k] = av;
            golden_b[k] = bv;
            mem_write_h(a_base + k*4, av);
            mem_write_h(b_base + k*4, bv);
        end
    end
    endtask

    function automatic integer expected_sum(input integer len);
        integer k;
    begin
        expected_sum = 0;
        for (k = 0; k < len; k = k + 1)
            expected_sum = expected_sum + golden_a[k] * golden_b[k];
    end
    endfunction

    //----------------------------------------------------
    // Counters/checkers
    //----------------------------------------------------
    integer m_active, m_arh, m_beats, m_pairs;
    integer pair_idx, data_err, last_err, got_sum;
    integer chk_len;

    always @(posedge clk) begin
        if (rst) begin
            m_active <= 0;
            m_arh    <= 0;
            m_beats  <= 0;
            m_pairs  <= 0;
        end else begin
            if (dma_busy)                         m_active <= m_active + 1;
            if (M_AXI_ARVALID && M_AXI_ARREADY)   m_arh    <= m_arh + 1;
            if (M_AXI_RVALID && M_AXI_RREADY)     m_beats  <= m_beats + 1;
            if (stream_valid && stream_ready)     m_pairs  <= m_pairs + 1;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            pair_idx <= 0;
            data_err <= 0;
            last_err <= 0;
            got_sum  <= 0;
        end else if (stream_valid && stream_ready) begin
            if (stream_a !== golden_a[pair_idx] || stream_b !== golden_b[pair_idx]) begin
                data_err <= data_err + 1;
                $display("  [PAIR MISMATCH] idx=%0d got a=%0d b=%0d exp a=%0d b=%0d",
                         pair_idx, stream_a, stream_b, golden_a[pair_idx], golden_b[pair_idx]);
            end
            if (pair_idx == chk_len - 1) begin
                if (!stream_last) last_err <= last_err + 1;
            end else if (stream_last) begin
                last_err <= last_err + 1;
            end
            got_sum  <= got_sum + stream_a * stream_b;
            pair_idx <= pair_idx + 1;
        end
    end

    //----------------------------------------------------
    // Stream and AR stability monitors
    //----------------------------------------------------
    integer stream_stab_err, ar_stab_err;
    reg prev_sv, prev_sr, prev_sl;
    reg signed [DATA_WIDTH-1:0] prev_sa, prev_sb;
    reg prev_arv, prev_arr;
    reg [AXI_ADDR_WIDTH-1:0] prev_araddr;
    reg [7:0] prev_arlen;
    reg [AXI_ID_WIDTH-1:0] prev_arid;
    reg mon_init;

    always @(posedge clk) begin
        if (rst) begin
            stream_stab_err <= 0;
            ar_stab_err     <= 0;
            mon_init        <= 0;
            prev_sv <= 0; prev_sr <= 0; prev_sl <= 0; prev_sa <= 0; prev_sb <= 0;
            prev_arv <= 0; prev_arr <= 0; prev_araddr <= 0; prev_arlen <= 0; prev_arid <= 0;
        end else begin
            if (mon_init) begin
                if (prev_sv && !prev_sr) begin
                    if (!stream_valid)             stream_stab_err <= stream_stab_err + 1;
                    else if (stream_a !== prev_sa) stream_stab_err <= stream_stab_err + 1;
                    else if (stream_b !== prev_sb) stream_stab_err <= stream_stab_err + 1;
                    else if (stream_last !== prev_sl) stream_stab_err <= stream_stab_err + 1;
                end
                if (prev_arv && !prev_arr) begin
                    if (!M_AXI_ARVALID)                       ar_stab_err <= ar_stab_err + 1;
                    else if (M_AXI_ARADDR !== prev_araddr)    ar_stab_err <= ar_stab_err + 1;
                    else if (M_AXI_ARLEN  !== prev_arlen)     ar_stab_err <= ar_stab_err + 1;
                    else if (M_AXI_ARID   !== prev_arid)      ar_stab_err <= ar_stab_err + 1;
                end
            end
            prev_sv <= stream_valid; prev_sr <= stream_ready; prev_sl <= stream_last;
            prev_sa <= stream_a; prev_sb <= stream_b;
            prev_arv <= M_AXI_ARVALID; prev_arr <= M_AXI_ARREADY;
            prev_araddr <= M_AXI_ARADDR; prev_arlen <= M_AXI_ARLEN; prev_arid <= M_AXI_ARID;
            mon_init <= 1'b1;
        end
    end

    //----------------------------------------------------
    // Reset and test runner
    //----------------------------------------------------
    task automatic do_reset;
    begin
        rst          = 1'b1;
        start        = 1'b0;
        src_a_addr   = 0;
        src_b_addr   = 0;
        length       = 0;
        mem_latency  = 16'd100;
        ooo_mode     = 1'b1;
        ready_mode   = 1'b0;
        repeat (8) @(posedge clk);
        @(posedge clk); #1; rst = 1'b0;
        repeat (4) @(posedge clk);
    end
    endtask

    integer pass_cnt = 0;
    integer fail_cnt = 0;
    integer head_umac_h, head_ubus_h, head_active;

    task automatic run_dma_rob_test(
        input integer len,
        input [31:0]  a_base,
        input [31:0]  b_base,
        input [15:0]  latency,
        input         bp_mode,
        input integer pattern,
        input         is_headline,
        input [255:0] name
    );
        integer timeout;
        integer umac_h;
        integer ubus_h;
        integer exp_sum;
    begin
        do_reset;
        chk_len = len;
        mem_latency = latency;
        preload_vectors(a_base, b_base, len, pattern);
        exp_sum = expected_sum(len);
        ready_mode = bp_mode;

        @(posedge clk);
        src_a_addr <= a_base;
        src_b_addr <= b_base;
        length     <= len[15:0];
        start      <= 1'b1;
        @(posedge clk);
        start      <= 1'b0;

        timeout = 0;
        while ((m_pairs < len || !dma_done) && timeout < 200000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        ready_mode = 1'b0;
        repeat (2) @(posedge clk);

        umac_h = (m_pairs * 10000) / m_active;
        ubus_h = (m_beats * 10000) / m_active;

        $display("---- %s ----", name);
        $display("  MAX_OUT=%0d len=%0d L=%0d bp=%0d expect_sum=%0d",
                 MAX_OUT, len, latency, bp_mode, exp_sum);

        if (timeout >= 200000) begin
            $display("  FAIL: TIMEOUT pairs=%0d done=%0b", m_pairs, dma_done);
            fail_cnt = fail_cnt + 1;
        end else if (data_err || last_err || dma_err || stream_stab_err || ar_stab_err) begin
            $display("  FAIL: data_err=%0d last_err=%0d dma_err=%0b stream_stab=%0d ar_stab=%0d",
                     data_err, last_err, dma_err, stream_stab_err, ar_stab_err);
            fail_cnt = fail_cnt + 1;
        end else if (m_pairs != len || m_beats != 2*len || got_sum != exp_sum) begin
            $display("  FAIL: pairs=%0d beats=%0d got_sum=%0d",
                     m_pairs, m_beats, got_sum);
            fail_cnt = fail_cnt + 1;
        end else begin
            pass_cnt = pass_cnt + 1;
            $display("  PASS pairs=%0d beats=%0d AR=%0d sum=%0d",
                     m_pairs, m_beats, m_arh, got_sum);
            $display("  T_active=%0d  U_mac=%0d.%02d %%  U_bus=%0d.%02d %%",
                     m_active, umac_h/100, umac_h%100, ubus_h/100, ubus_h%100);
            if (is_headline) begin
                head_umac_h = umac_h;
                head_ubus_h = ubus_h;
                head_active = m_active;
            end
        end
        $display("");
    end
    endtask

    initial begin
        $display("==================================================");
        $display("  mac_dma_rob integration test");
        $display("  A phase + B phase use axi_read_engine_rob, OOO memory responses");
        $display("  MAX_OUTSTANDING=%0d BURST_LEN=%0d", MAX_OUT, BURST_LEN);
        $display("==================================================");

        run_dma_rob_test(64,  32'h0000_0000, 32'h0000_0400, 16'd100, 1'b0, 0, 1'b1,
                         "headline len=64 L=100");
        run_dma_rob_test(20,  32'h0000_0800, 32'h0000_0C00, 16'd100, 1'b0, 1, 1'b0,
                         "partial final burst len=20");
        run_dma_rob_test(256, 32'h0000_1000, 32'h0000_1400, 16'd100, 1'b0, 2, 1'b0,
                         "steady len=256 L=100");
        run_dma_rob_test(64,  32'h0000_1800, 32'h0000_1C00, 16'd100, 1'b1, 0, 1'b0,
                         "random stream/AR backpressure");

        $display("==================================================");
        $display("  TOTAL: %0d PASS / %0d FAIL", pass_cnt, fail_cnt);
        $display("  HEADLINE MAX_OUT=%0d len=64 L=100: U_mac=%0d.%02d %% U_bus=%0d.%02d %% T_active=%0d",
                 MAX_OUT, head_umac_h/100, head_umac_h%100,
                 head_ubus_h/100, head_ubus_h%100, head_active);
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
