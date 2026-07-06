`timescale 1ns/1ps

// tb_mac_dma_rob_sweep.v
// -----------------------------------------------------------------------------
// Step 4 sweep harness for the experimental mac_dma_rob path.
//
// Prints machine-readable lines:
//   SWEEP,max_out,len,lat,t_active,pairs,beats,umac_h,ubus_h,PASS
//
// Compile with -DDMA_ROB_MAX_OUT=<2|4|8>.
// -----------------------------------------------------------------------------

`ifndef DMA_ROB_MAX_OUT
`define DMA_ROB_MAX_OUT 4
`endif

module tb_mac_dma_rob_sweep;

    localparam DATA_WIDTH      = 16;
    localparam AXI_DATA_WIDTH  = 32;
    localparam AXI_ADDR_WIDTH  = 32;
    localparam AXI_ID_WIDTH    = 4;
    localparam MAX_LEN         = 256;
    localparam BURST_LEN       = 16;
    localparam MAX_OUT         = `DMA_ROB_MAX_OUT;
    localparam MEM_WORDS       = 4096;

    reg clk;
    initial clk = 1'b0;
    always #5 clk = ~clk;

    reg rst;
    reg start;
    reg [AXI_ADDR_WIDTH-1:0] src_a_addr;
    reg [AXI_ADDR_WIDTH-1:0] src_b_addr;
    reg [15:0] length;

    wire dma_busy;
    wire dma_done;
    wire dma_err;

    wire [AXI_ID_WIDTH-1:0]   M_AXI_ARID;
    wire [AXI_ADDR_WIDTH-1:0] M_AXI_ARADDR;
    wire [7:0]                M_AXI_ARLEN;
    wire [2:0]                M_AXI_ARSIZE;
    wire [1:0]                M_AXI_ARBURST;
    wire                      M_AXI_ARVALID;
    wire                      M_AXI_ARREADY;
    wire [AXI_ID_WIDTH-1:0]   M_AXI_RID;
    wire [AXI_DATA_WIDTH-1:0] M_AXI_RDATA;
    wire [1:0]                M_AXI_RRESP;
    wire                      M_AXI_RLAST;
    wire                      M_AXI_RVALID;
    wire                      M_AXI_RREADY;

    wire                          stream_valid;
    wire                          stream_last;
    wire signed [DATA_WIDTH-1:0]  stream_a;
    wire signed [DATA_WIDTH-1:0]  stream_b;
    wire                          stream_ready = 1'b1;

    reg [15:0] mem_latency;

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
        .ar_stall        (1'b0),
        .ooo_en          (1'b1),
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

    reg signed [15:0] golden_a [0:MAX_LEN-1];
    reg signed [15:0] golden_b [0:MAX_LEN-1];

    task automatic mem_write_h(input [31:0] byte_addr, input signed [15:0] val);
    begin
        u_mem.mem[(byte_addr >> 2) & (MEM_WORDS-1)] = {{16{val[15]}}, val};
    end
    endtask

    task automatic preload_vectors(input [31:0] a_base, input [31:0] b_base, input integer len);
        integer k;
        reg [15:0] rem_a;
        reg [15:0] rem_b;
        reg signed [15:0] av;
        reg signed [15:0] bv;
    begin
        for (k = 0; k < len; k = k + 1) begin
            /* verilator lint_off WIDTHTRUNC */
            rem_a = k % 9;
            rem_b = k % 7;
            /* verilator lint_on WIDTHTRUNC */
            av = rem_a;
            bv = rem_b;
            av = av - 16'sd4;
            bv = bv - 16'sd3;
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

    integer m_active, m_pairs, m_beats;
    integer pair_idx, data_err, last_err, got_sum, chk_len;

    always @(posedge clk) begin
        if (rst) begin
            m_active <= 0;
            m_pairs  <= 0;
            m_beats  <= 0;
        end else begin
            if (dma_busy)                     m_active <= m_active + 1;
            if (stream_valid && stream_ready) m_pairs  <= m_pairs + 1;
            if (M_AXI_RVALID && M_AXI_RREADY) m_beats  <= m_beats + 1;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            pair_idx <= 0;
            data_err <= 0;
            last_err <= 0;
            got_sum  <= 0;
        end else if (stream_valid && stream_ready) begin
            if (stream_a !== golden_a[pair_idx] || stream_b !== golden_b[pair_idx])
                data_err <= data_err + 1;
            if (pair_idx == chk_len - 1) begin
                if (!stream_last) last_err <= last_err + 1;
            end else if (stream_last) begin
                last_err <= last_err + 1;
            end
            got_sum  <= got_sum + stream_a * stream_b;
            pair_idx <= pair_idx + 1;
        end
    end

    task automatic do_reset;
    begin
        rst         = 1'b1;
        start       = 1'b0;
        src_a_addr  = 0;
        src_b_addr  = 0;
        length      = 0;
        mem_latency = 0;
        repeat (8) @(posedge clk);
        @(posedge clk); #1; rst = 1'b0;
        repeat (4) @(posedge clk);
    end
    endtask

    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task automatic run_case(input integer len, input integer latency);
        integer timeout;
        integer umac_h;
        integer ubus_h;
        integer exp_sum;
        reg [31:0] a_base;
        reg [31:0] b_base;
    begin
        do_reset;
        a_base = 32'h0000_0000;
        b_base = 32'h0000_1000;
        chk_len = len;
        mem_latency = latency[15:0];
        preload_vectors(a_base, b_base, len);
        exp_sum = expected_sum(len);

        @(posedge clk);
        src_a_addr = a_base;
        src_b_addr = b_base;
        length     = len[15:0];
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        timeout = 0;
        while ((m_pairs < len || !dma_done) && timeout < 200000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        repeat (2) @(posedge clk);

        umac_h = (m_pairs * 10000) / m_active;
        ubus_h = (m_beats * 10000) / m_active;

        if (timeout < 200000 && !dma_err && data_err == 0 && last_err == 0 &&
            m_pairs == len && m_beats == 2*len && got_sum == exp_sum) begin
            pass_cnt = pass_cnt + 1;
            $display("SWEEP,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,PASS",
                     MAX_OUT, len, latency, m_active, m_pairs, m_beats, umac_h, ubus_h);
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("SWEEP,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,FAIL",
                     MAX_OUT, len, latency, m_active, m_pairs, m_beats, umac_h, ubus_h);
            $display("  fail detail: timeout=%0d dma_err=%0b data_err=%0d last_err=%0d got_sum=%0d exp_sum=%0d",
                     timeout, dma_err, data_err, last_err, got_sum, exp_sum);
        end
    end
    endtask

    initial begin
        $display("SWEEP_HEADER,max_out,len,lat,t_active,pairs,beats,umac_h,ubus_h,status");

        run_case(64, 0);
        run_case(64, 50);
        run_case(64, 100);
        run_case(64, 200);
        run_case(256, 0);
        run_case(256, 50);
        run_case(256, 100);
        run_case(256, 200);

        $display("SWEEP_TOTAL,%0d,%0d", pass_cnt, fail_cnt);
        #50;
        $finish;
    end

    initial begin
        #20000000;
        $display("[TB] global timeout");
        $finish;
    end

endmodule
