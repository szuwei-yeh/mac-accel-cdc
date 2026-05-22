`timescale 1ns/1ps

// tb_mac_accel_dma.v
// Testbench for V2 (DMA) MAC accelerator.
//
// Verifies:
//   - AXI4-Lite slave control plane (CTRL / SRC_A / SRC_B / LENGTH / RESULT / LATENCY)
//   - AXI4 master DMA burst reads from a behavioural memory model
//   - True CDC: bus_clk = 100 MHz, mac_clk = 133 MHz (non-integer ratio)
//   - Single-burst (len<=16) and multi-burst (len>16) transactions
//
// AXI memory model: 4 KB, services INCR bursts.  Address taken modulo MEM_BYTES
// so tests can pick any base address.

module tb_mac_accel_dma;

    localparam DATA_WIDTH = 16;

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
    // M_AXI (DUT master -> our memory model)
    //----------------------------------------------------
    wire [3:0]  M_AXI_ARID;
    wire [31:0] M_AXI_ARADDR;
    wire [7:0]  M_AXI_ARLEN;
    wire [2:0]  M_AXI_ARSIZE;
    wire [1:0]  M_AXI_ARBURST;
    wire        M_AXI_ARVALID;
    reg         M_AXI_ARREADY;

    reg  [3:0]  M_AXI_RID;
    reg  [31:0] M_AXI_RDATA;
    reg  [1:0]  M_AXI_RRESP;
    reg         M_AXI_RLAST;
    reg         M_AXI_RVALID;
    wire        M_AXI_RREADY;

    wire        done_led;

    //----------------------------------------------------
    // DUT
    //----------------------------------------------------
    mac_accel_dma_top #(
        .DATA_WIDTH    (DATA_WIDTH),
        .MAX_LEN       (256),
        .BURST_LEN     (16)
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

    //----------------------------------------------------
    // Behavioural AXI memory model (4 KB, INCR bursts only)
    //----------------------------------------------------
    localparam MEM_WORDS = 1024;             // 4 KB / 4 B
    reg [31:0] mem [0:MEM_WORDS-1];

    // Latched AR + burst state
    reg [31:0] ar_addr_r;
    reg [7:0]  ar_len_r;
    reg [3:0]  ar_id_r;
    reg [7:0]  beat_cnt;
    reg        burst_active;

    task automatic mem_init;
        integer i;
    begin
        for (i = 0; i < MEM_WORDS; i = i + 1)
            mem[i] = 32'h0000_0000;
    end
    endtask

    // Write a 16-bit signed value as a 32-bit word at byte addr
    task automatic mem_write_h(input [31:0] byte_addr, input signed [15:0] val);
    begin
        mem[(byte_addr >> 2) & (MEM_WORDS-1)] = {{16{val[15]}}, val};
    end
    endtask

    // Memory model FSM
    always @(posedge bus_clk) begin
        if (!aresetn) begin
            M_AXI_ARREADY <= 1'b0;
            M_AXI_RVALID  <= 1'b0;
            M_AXI_RDATA   <= 32'd0;
            M_AXI_RLAST   <= 1'b0;
            M_AXI_RRESP   <= 2'b00;
            M_AXI_RID     <= 4'd0;
            burst_active  <= 1'b0;
            beat_cnt      <= 8'd0;
            ar_addr_r     <= 32'd0;
            ar_len_r      <= 8'd0;
            ar_id_r       <= 4'd0;
        end else begin
            // Accept AR when no burst in flight
            if (!burst_active && M_AXI_ARVALID && !M_AXI_ARREADY) begin
                M_AXI_ARREADY <= 1'b1;
                ar_addr_r     <= M_AXI_ARADDR;
                ar_len_r      <= M_AXI_ARLEN;
                ar_id_r       <= M_AXI_ARID;
            end else if (M_AXI_ARREADY) begin
                // Handshake taken last cycle
                M_AXI_ARREADY <= 1'b0;
                burst_active  <= 1'b1;
                beat_cnt      <= 8'd0;
            end

            // Drive R channel
            if (burst_active && !M_AXI_RVALID) begin
                M_AXI_RVALID <= 1'b1;
                M_AXI_RDATA  <= mem[(ar_addr_r >> 2) & (MEM_WORDS-1)];
                M_AXI_RLAST  <= (beat_cnt == ar_len_r);
                M_AXI_RID    <= ar_id_r;
            end else if (M_AXI_RVALID && M_AXI_RREADY) begin
                if (beat_cnt == ar_len_r) begin
                    // Last beat consumed
                    M_AXI_RVALID <= 1'b0;
                    M_AXI_RLAST  <= 1'b0;
                    burst_active <= 1'b0;
                end else begin
                    beat_cnt   <= beat_cnt + 8'd1;
                    ar_addr_r  <= ar_addr_r + 32'd4;
                    M_AXI_RDATA <= mem[((ar_addr_r + 32'd4) >> 2) & (MEM_WORDS-1)];
                    M_AXI_RLAST <= ((beat_cnt + 8'd1) == ar_len_r);
                end
            end
        end
    end

    //----------------------------------------------------
    // S_AXI master driver tasks
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

        // Wait for AW handshake
        do @(posedge bus_clk); while (!S_AXI_AWREADY);
        S_AXI_AWVALID <= 1'b0;

        // Wait for W handshake (may have already happened)
        while (!S_AXI_WREADY) @(posedge bus_clk);
        S_AXI_WVALID <= 1'b0;

        // Wait for B handshake
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
    // Reset
    //----------------------------------------------------
    task do_reset;
    begin
        aresetn       = 1'b0;
        mac_rst       = 1'b1;
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

    //----------------------------------------------------
    // Wait for done_led with timeout
    //----------------------------------------------------
    task wait_done(output reg timed_out);
        integer cnt;
    begin
        timed_out = 0;
        cnt = 0;
        while (done_led !== 1'b1 && cnt < 5000) begin
            @(posedge bus_clk);
            cnt = cnt + 1;
        end
        if (cnt >= 5000) timed_out = 1;
    end
    endtask

    //----------------------------------------------------
    // Test runner
    //----------------------------------------------------
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task run_dma_test(
        input integer  length,
        input [31:0]   src_a_addr,
        input [31:0]   src_b_addr,
        input signed [31:0] expected,
        input [255:0]  name
    );
        reg [31:0] res, lat;
        reg        timed_out;
    begin
        do_reset;

        $display("---- %s ----", name);
        $display("  length=%0d  src_a=0x%08h  src_b=0x%08h  expect=%0d",
                  length, src_a_addr, src_b_addr, expected);

        // Program registers
        axi_write(ADDR_SRC_A,  src_a_addr);
        axi_write(ADDR_SRC_B,  src_b_addr);
        axi_write(ADDR_LENGTH, length);

        // Kick off
        axi_write(ADDR_CTRL, 32'h0000_0001);

        // Wait
        wait_done(timed_out);

        if (timed_out) begin
            $display("  TIMEOUT (done_led never asserted)");
            fail_cnt = fail_cnt + 1;
        end else begin
            axi_read(ADDR_RESULT,  res);
            axi_read(ADDR_LATENCY, lat);
            $display("  HW RESULT=%0d  LATENCY=%0d mac_clk cycles",
                      $signed(res), lat);

            if ($signed(res) == expected) begin
                $display("  PASS");
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL: got %0d, expected %0d", $signed(res), expected);
                fail_cnt = fail_cnt + 1;
            end
        end
    end
    endtask

    //----------------------------------------------------
    // Stimulus
    //----------------------------------------------------
    integer i;
    integer sum;

    initial begin
        $display("========================================");
        $display("  mac_accel_dma_top Testbench (V2)");
        $display("  bus_clk=100MHz  mac_clk=133MHz");
        $display("========================================");

        mem_init;

        //----- Test 1: basic 4-element, single burst -----
        // A=[1,2,3,4] @ 0x000, B=[10,20,30,40] @ 0x100, expect=300
        mem_write_h(32'h0000_0000, 16'sd1);
        mem_write_h(32'h0000_0004, 16'sd2);
        mem_write_h(32'h0000_0008, 16'sd3);
        mem_write_h(32'h0000_000C, 16'sd4);
        mem_write_h(32'h0000_0100, 16'sd10);
        mem_write_h(32'h0000_0104, 16'sd20);
        mem_write_h(32'h0000_0108, 16'sd30);
        mem_write_h(32'h0000_010C, 16'sd40);
        run_dma_test(4, 32'h0000_0000, 32'h0000_0100, 300,
                     "Test 1: basic length=4");

        //----- Test 2: negative values -----
        // A=[-1,-2,-3,-4] @ 0x200, B=[1,2,3,4] @ 0x300, expect=-30
        mem_write_h(32'h0000_0200, -16'sd1);
        mem_write_h(32'h0000_0204, -16'sd2);
        mem_write_h(32'h0000_0208, -16'sd3);
        mem_write_h(32'h0000_020C, -16'sd4);
        mem_write_h(32'h0000_0300, 16'sd1);
        mem_write_h(32'h0000_0304, 16'sd2);
        mem_write_h(32'h0000_0308, 16'sd3);
        mem_write_h(32'h0000_030C, 16'sd4);
        run_dma_test(4, 32'h0000_0200, 32'h0000_0300, -30,
                     "Test 2: negative");

        //----- Test 3: length=16, exactly one burst -----
        // A[i]=i+1, B[i]=2, sum = 2 * (1+..+16) = 2*136 = 272
        sum = 0;
        for (i = 0; i < 16; i = i + 1) begin
            mem_write_h(32'h0000_0400 + i*4, i + 1);
            mem_write_h(32'h0000_0500 + i*4, 16'sd2);
            sum = sum + 2 * (i + 1);
        end
        run_dma_test(16, 32'h0000_0400, 32'h0000_0500, sum,
                     "Test 3: length=16 (single burst)");

        //----- Test 4: length=20, multi-burst (16+4) -----
        // A[i]=i+1, B[i]=1, sum=210
        sum = 0;
        for (i = 0; i < 20; i = i + 1) begin
            mem_write_h(32'h0000_0600 + i*4, i + 1);
            mem_write_h(32'h0000_0700 + i*4, 16'sd1);
            sum = sum + (i + 1);
        end
        run_dma_test(20, 32'h0000_0600, 32'h0000_0700, sum,
                     "Test 4: length=20 (multi-burst 16+4)");

        //----- Test 5: length=33, multi-burst (16+16+1) -----
        // A[i]=1, B[i]=1, sum=33
        for (i = 0; i < 33; i = i + 1) begin
            mem_write_h(32'h0000_0800 + i*4, 16'sd1);
            mem_write_h(32'h0000_0900 + i*4, 16'sd1);
        end
        run_dma_test(33, 32'h0000_0800, 32'h0000_0900, 33,
                     "Test 5: length=33 (3 bursts: 16+16+1)");

        //----- Test 6: length=1 (degenerate single beat) -----
        mem_write_h(32'h0000_0A00, 16'sd7);
        mem_write_h(32'h0000_0A40, 16'sd6);
        run_dma_test(1, 32'h0000_0A00, 32'h0000_0A40, 42,
                     "Test 6: length=1");

        //----- Test 7: back-to-back same memory contents -----
        // Re-use Test 1 vectors with a fresh reset
        run_dma_test(4, 32'h0000_0000, 32'h0000_0100, 300,
                     "Test 7: back-to-back (re-use Test 1 data)");

        $display("========================================");
        $display("  TOTAL: %0d PASS / %0d FAIL", pass_cnt, fail_cnt);
        $display("========================================");

        #100;
        $finish;
    end

    // Global timeout safety net
    initial begin
        #200000;
        $display("[TB] Global timeout reached -- aborting");
        $finish;
    end

endmodule
