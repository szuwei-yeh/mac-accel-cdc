`timescale 1ns/1ps

// tb_mac_accel_dma_rob_top.v
// Full-top V3 regression:
//   AXI4-Lite CSR -> mac_dma_rob -> async FIFO CDC -> mac_pe -> result CSR.
//
// The memory model supports multiple outstanding reads and can return
// out-of-order/interleaved R-channel beats, so this verifies that the ROB DMA
// can replace the V2 single-outstanding DMA inside the complete accelerator
// shell.

module tb_mac_accel_dma_rob_top;

    localparam DATA_WIDTH      = 16;
    localparam AXI_DATA_WIDTH  = 32;
    localparam AXI_ADDR_WIDTH  = 32;
    localparam AXI_ID_WIDTH    = 4;
    localparam MAX_LEN         = 256;
    localparam BURST_LEN       = 16;
    localparam MAX_OUT         = 4;
    localparam MEM_WORDS       = 2048;

    localparam ADDR_CTRL    = 8'h00;
    localparam ADDR_SRC_A   = 8'h04;
    localparam ADDR_SRC_B   = 8'h08;
    localparam ADDR_LENGTH  = 8'h0C;
    localparam ADDR_RESULT  = 8'h10;
    localparam ADDR_LATENCY = 8'h14;

    //----------------------------------------------------
    // Clocks & reset
    //----------------------------------------------------
    reg bus_clk = 0;
    reg mac_clk = 0;
    always #5.0  bus_clk = ~bus_clk;   // 100 MHz
    always #3.75 mac_clk = ~mac_clk;   // ~133 MHz

    reg aresetn;
    reg mac_rst;

    //----------------------------------------------------
    // S_AXI (master driver) signals
    //----------------------------------------------------
    reg  [7:0]  S_AXI_AWADDR;
    reg         S_AXI_AWVALID;
    wire        S_AXI_AWREADY;
    reg  [31:0] S_AXI_WDATA;
    reg  [3:0]  S_AXI_WSTRB;
    reg         S_AXI_WVALID;
    wire        S_AXI_WREADY;
    wire [1:0]  S_AXI_BRESP;
    wire        S_AXI_BVALID;
    reg         S_AXI_BREADY;

    reg  [7:0]  S_AXI_ARADDR;
    reg         S_AXI_ARVALID;
    wire        S_AXI_ARREADY;
    wire [31:0] S_AXI_RDATA;
    wire [1:0]  S_AXI_RRESP;
    wire        S_AXI_RVALID;
    reg         S_AXI_RREADY;

    //----------------------------------------------------
    // M_AXI (DUT master -> out-of-order memory model)
    //----------------------------------------------------
    wire [AXI_ID_WIDTH-1:0]    M_AXI_ARID;
    wire [AXI_ADDR_WIDTH-1:0]  M_AXI_ARADDR;
    wire [7:0]                 M_AXI_ARLEN;
    wire [2:0]                 M_AXI_ARSIZE;
    wire [1:0]                 M_AXI_ARBURST;
    wire                       M_AXI_ARVALID;
    wire                       M_AXI_ARREADY;

    wire [AXI_ID_WIDTH-1:0]    M_AXI_RID;
    wire [AXI_DATA_WIDTH-1:0]  M_AXI_RDATA;
    wire [1:0]                 M_AXI_RRESP;
    wire                       M_AXI_RLAST;
    wire                       M_AXI_RVALID;
    wire                       M_AXI_RREADY;

    reg [15:0] mem_latency;
    reg        ooo_mode;
    reg        ar_bp_mode;
    reg        ar_stall;
    wire       done_led;

    //----------------------------------------------------
    // DUT
    //----------------------------------------------------
    mac_accel_dma_rob_top #(
        .DATA_WIDTH      (DATA_WIDTH),
        .AXI_DATA_WIDTH  (AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH  (AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH    (AXI_ID_WIDTH),
        .MAX_LEN         (MAX_LEN),
        .BURST_LEN       (BURST_LEN),
        .MAX_OUTSTANDING (MAX_OUT)
    ) dut (
        .S_AXI_ACLK     (bus_clk),
        .S_AXI_ARESETN  (aresetn),
        .S_AXI_AWADDR   (S_AXI_AWADDR),
        .S_AXI_AWPROT   (3'b000),
        .S_AXI_AWVALID  (S_AXI_AWVALID),
        .S_AXI_AWREADY  (S_AXI_AWREADY),
        .S_AXI_WDATA    (S_AXI_WDATA),
        .S_AXI_WSTRB    (S_AXI_WSTRB),
        .S_AXI_WVALID   (S_AXI_WVALID),
        .S_AXI_WREADY   (S_AXI_WREADY),
        .S_AXI_BRESP    (S_AXI_BRESP),
        .S_AXI_BVALID   (S_AXI_BVALID),
        .S_AXI_BREADY   (S_AXI_BREADY),
        .S_AXI_ARADDR   (S_AXI_ARADDR),
        .S_AXI_ARPROT   (3'b000),
        .S_AXI_ARVALID  (S_AXI_ARVALID),
        .S_AXI_ARREADY  (S_AXI_ARREADY),
        .S_AXI_RDATA    (S_AXI_RDATA),
        .S_AXI_RRESP    (S_AXI_RRESP),
        .S_AXI_RVALID   (S_AXI_RVALID),
        .S_AXI_RREADY   (S_AXI_RREADY),

        .M_AXI_ARID     (M_AXI_ARID),
        .M_AXI_ARADDR   (M_AXI_ARADDR),
        .M_AXI_ARLEN    (M_AXI_ARLEN),
        .M_AXI_ARSIZE   (M_AXI_ARSIZE),
        .M_AXI_ARBURST  (M_AXI_ARBURST),
        .M_AXI_ARVALID  (M_AXI_ARVALID),
        .M_AXI_ARREADY  (M_AXI_ARREADY),
        .M_AXI_RID      (M_AXI_RID),
        .M_AXI_RDATA    (M_AXI_RDATA),
        .M_AXI_RRESP    (M_AXI_RRESP),
        .M_AXI_RLAST    (M_AXI_RLAST),
        .M_AXI_RVALID   (M_AXI_RVALID),
        .M_AXI_RREADY   (M_AXI_RREADY),

        .mac_clk        (mac_clk),
        .mac_rst        (mac_rst),
        .done_led       (done_led)
    );

    axi_read_mem_model_ooo #(
        .AXI_ID_WIDTH    (AXI_ID_WIDTH),
        .AXI_ADDR_WIDTH  (AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH  (AXI_DATA_WIDTH),
        .MEM_WORDS       (MEM_WORDS),
        .MAX_OUTSTANDING (MAX_OUT)
    ) u_mem (
        .clk             (bus_clk),
        .resetn          (aresetn),
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

    always @(posedge bus_clk) begin
        if (!aresetn) ar_stall <= 1'b0;
        else          ar_stall <= ar_bp_mode ? $random : 1'b0;
    end

    //----------------------------------------------------
    // Memory preload helpers
    //----------------------------------------------------
    reg signed [15:0] golden_a [0:MAX_LEN-1];
    reg signed [15:0] golden_b [0:MAX_LEN-1];

    task automatic mem_init;
        integer i;
    begin
        for (i = 0; i < MEM_WORDS; i = i + 1)
            u_mem.mem[i] = 32'h0000_0000;
    end
    endtask

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
                    bv = (k + 1) * 10;
                end
                1: begin
                    av = -16'sd1 * (k + 1);
                    bv = k + 1;
                end
                2: begin
                    av = k + 1;
                    bv = 16'sd2;
                end
                3: begin
                    av = 16'sd1;
                    bv = 16'sd1;
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
    // AXI4-Lite master driver tasks
    //----------------------------------------------------
    task automatic axi_write(input [7:0] addr, input [31:0] data);
    begin
        @(posedge bus_clk);
        S_AXI_AWADDR  <= addr;
        S_AXI_AWVALID <= 1'b1;
        S_AXI_WDATA   <= data;
        S_AXI_WSTRB   <= 4'hF;
        S_AXI_WVALID  <= 1'b1;
        S_AXI_BREADY  <= 1'b1;

        do @(posedge bus_clk); while (!S_AXI_AWREADY);
        S_AXI_AWVALID <= 1'b0;

        while (!S_AXI_WREADY) @(posedge bus_clk);
        S_AXI_WVALID <= 1'b0;

        while (!S_AXI_BVALID) @(posedge bus_clk);
        @(posedge bus_clk);
        S_AXI_BREADY <= 1'b0;
    end
    endtask

    task automatic axi_read(input [7:0] addr, output [31:0] data);
    begin
        @(posedge bus_clk);
        S_AXI_ARADDR  <= addr;
        S_AXI_ARVALID <= 1'b1;
        S_AXI_RREADY  <= 1'b1;

        while (!S_AXI_ARREADY) @(posedge bus_clk);
        @(posedge bus_clk);
        S_AXI_ARVALID <= 1'b0;

        while (!S_AXI_RVALID) @(posedge bus_clk);
        data = S_AXI_RDATA;
        @(posedge bus_clk);
        S_AXI_RREADY <= 1'b0;
    end
    endtask

    //----------------------------------------------------
    // Reset, counters, and test runner
    //----------------------------------------------------
    integer m_active, m_arh, m_beats, fifo_full_cycles;

    always @(posedge bus_clk) begin
        if (!aresetn) begin
            m_active         <= 0;
            m_arh            <= 0;
            m_beats          <= 0;
            fifo_full_cycles <= 0;
        end else begin
            if (dut.busy_any_bus)                  m_active <= m_active + 1;
            if (M_AXI_ARVALID && M_AXI_ARREADY)    m_arh <= m_arh + 1;
            if (M_AXI_RVALID && M_AXI_RREADY)      m_beats <= m_beats + 1;
            if (dut.fifo_full_bus)                 fifo_full_cycles <= fifo_full_cycles + 1;
        end
    end

    task automatic do_reset;
    begin
        aresetn       = 1'b0;
        mac_rst       = 1'b1;
        ar_bp_mode    = 1'b0;
        ar_stall      = 1'b0;
        ooo_mode      = 1'b1;
        mem_latency   = 16'd20;
        S_AXI_AWADDR  = 0;
        S_AXI_AWVALID = 0;
        S_AXI_WDATA   = 0;
        S_AXI_WSTRB   = 0;
        S_AXI_WVALID  = 0;
        S_AXI_BREADY  = 0;
        S_AXI_ARADDR  = 0;
        S_AXI_ARVALID = 0;
        S_AXI_RREADY  = 0;

        repeat (10) @(posedge bus_clk);
        repeat (10) @(posedge mac_clk);

        @(posedge mac_clk); #1; mac_rst = 1'b0;
        repeat (3) @(posedge bus_clk);
        @(posedge bus_clk); #1; aresetn = 1'b1;
        repeat (8) @(posedge bus_clk);
    end
    endtask

    task wait_done(output reg timed_out);
        integer cnt;
    begin
        timed_out = 0;
        cnt = 0;
        while (done_led !== 1'b1 && cnt < 200000) begin
            @(posedge bus_clk);
            cnt = cnt + 1;
        end
        if (cnt >= 200000) timed_out = 1;
    end
    endtask

    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task run_dma_rob_top_test(
        input integer  len,
        input [31:0]   src_a_addr,
        input [31:0]   src_b_addr,
        input [15:0]   latency,
        input          ooo_en,
        input          ar_bp_en,
        input integer  pattern,
        input [255:0]  name
    );
        reg [31:0] res, lat, ctrl;
        reg        timed_out;
        integer    expected;
    begin
        do_reset;
        mem_init;
        preload_vectors(src_a_addr, src_b_addr, len, pattern);
        expected = expected_sum(len);
        mem_latency = latency;
        ooo_mode    = ooo_en;
        ar_bp_mode  = ar_bp_en;

        $display("---- %s ----", name);
        $display("  len=%0d src_a=0x%08h src_b=0x%08h L=%0d ooo=%0d ar_bp=%0d expect=%0d",
                  len, src_a_addr, src_b_addr, latency, ooo_en, ar_bp_en, expected);

        axi_write(ADDR_SRC_A,  src_a_addr);
        axi_write(ADDR_SRC_B,  src_b_addr);
        axi_write(ADDR_LENGTH, len);
        axi_write(ADDR_CTRL,   32'h0000_0001);

        wait_done(timed_out);

        if (timed_out) begin
            $display("  TIMEOUT (done_led never asserted)");
            fail_cnt = fail_cnt + 1;
        end else begin
            axi_read(ADDR_RESULT,  res);
            axi_read(ADDR_LATENCY, lat);
            axi_read(ADDR_CTRL,    ctrl);
            $display("  HW RESULT=%0d LATENCY=%0d mac_clk cycles CTRL=0x%08h",
                      $signed(res), lat, ctrl);
            $display("  AR=%0d Rbeats=%0d active=%0d fifo_full_cycles=%0d",
                      m_arh, m_beats, m_active, fifo_full_cycles);

            if ($signed(res) == expected && ctrl[2] && !ctrl[3]) begin
                $display("  PASS");
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL: got %0d expected %0d done=%0b dma_err=%0b",
                         $signed(res), expected, ctrl[2], ctrl[3]);
                fail_cnt = fail_cnt + 1;
            end
        end
        ar_bp_mode = 1'b0;
        $display("");
    end
    endtask

    initial begin
        $display("==================================================");
        $display("  mac_accel_dma_rob_top Testbench (V3 full top)");
        $display("  bus_clk=100MHz mac_clk=133MHz MAX_OUT=%0d", MAX_OUT);
        $display("==================================================");

        run_dma_rob_top_test(4,   32'h0000_0000, 32'h0000_0100, 16'd0,   1'b0, 1'b0, 0,
                             "Test 1: basic length=4");
        run_dma_rob_top_test(4,   32'h0000_0200, 32'h0000_0300, 16'd3,   1'b1, 1'b0, 1,
                             "Test 2: negative operands with OOO");
        run_dma_rob_top_test(16,  32'h0000_0400, 32'h0000_0500, 16'd25,  1'b1, 1'b0, 2,
                             "Test 3: exactly one full burst");
        run_dma_rob_top_test(20,  32'h0000_0600, 32'h0000_0700, 16'd100, 1'b1, 1'b0, 4,
                             "Test 4: multi-burst latency hiding");
        run_dma_rob_top_test(33,  32'h0000_0800, 32'h0000_0900, 16'd25,  1'b1, 1'b1, 3,
                             "Test 5: multi-burst with AR stalls");
        run_dma_rob_top_test(64,  32'h0000_0A00, 32'h0000_0C00, 16'd0,   1'b1, 1'b0, 4,
                             "Test 6: fast memory FIFO backpressure");
        run_dma_rob_top_test(1,   32'h0000_0E00, 32'h0000_0F00, 16'd5,   1'b1, 1'b0, 2,
                             "Test 7: length=1");

        $display("==================================================");
        $display("  TOTAL: %0d PASS / %0d FAIL", pass_cnt, fail_cnt);
        $display("==================================================");
        #100;
        $finish;
    end

    initial begin
        #20000000;
        $display("[TB] Global timeout reached -- aborting");
        $finish;
    end

endmodule
