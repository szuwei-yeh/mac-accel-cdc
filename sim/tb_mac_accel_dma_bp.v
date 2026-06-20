`timescale 1ns/1ps
// tb_mac_accel_dma_bp.v
// Adversarial BACK-PRESSURE test for V2 (DMA) MAC accelerator.
//
// Same DUT + behavioural AXI memory model as tb_mac_accel_dma.v, BUT:
//   - mac_clk is made MUCH SLOWER than bus_clk (20 MHz vs 100 MHz) so the
//     consumer (mac_pe) drains the async FIFO ~5x slower than the DMA fills
//     it -> the depth-16 FIFO saturates -> stream_ready (= ~fifo_full) drops
//     constantly -> the S_R_B RREADY back-pressure path is heavily exercised.
//   - long vectors (multi-burst) maximise the number of stall/resume edges.
//
// Trust model: this uses the REAL async FIFO + REAL mac_pe and checks the
// final signed dot-product against a known answer. If ANY element is dropped
// or duplicated under back-pressure, the result is wrong -> FAIL.

module tb_mac_accel_dma_bp;

    localparam DATA_WIDTH = 16;

    localparam ADDR_CTRL    = 8'h00;
    localparam ADDR_SRC_A   = 8'h04;
    localparam ADDR_SRC_B   = 8'h08;
    localparam ADDR_LENGTH  = 8'h0C;
    localparam ADDR_RESULT  = 8'h10;
    localparam ADDR_LATENCY = 8'h14;

    // ---- clocks: consumer (mac_clk) deliberately SLOW ----
    reg bus_clk = 0;
    reg mac_clk = 0;
    always #5.0  bus_clk = ~bus_clk;   // 100 MHz  (fast producer)
    always #25.0 mac_clk = ~mac_clk;   // 20  MHz  (slow consumer) -> FIFO saturates

    reg aresetn;
    reg mac_rst;

    reg  [7:0]  S_AXI_AWADDR;  reg S_AXI_AWVALID; wire S_AXI_AWREADY;
    reg  [31:0] S_AXI_WDATA;   reg [3:0] S_AXI_WSTRB; reg S_AXI_WVALID; wire S_AXI_WREADY;
    wire [1:0]  S_AXI_BRESP;   wire S_AXI_BVALID; reg S_AXI_BREADY;
    reg  [7:0]  S_AXI_ARADDR;  reg S_AXI_ARVALID; wire S_AXI_ARREADY;
    wire [31:0] S_AXI_RDATA;   wire [1:0] S_AXI_RRESP; wire S_AXI_RVALID; reg S_AXI_RREADY;

    wire [3:0]  M_AXI_ARID;  wire [31:0] M_AXI_ARADDR; wire [7:0] M_AXI_ARLEN;
    wire [2:0]  M_AXI_ARSIZE; wire [1:0] M_AXI_ARBURST; wire M_AXI_ARVALID; reg M_AXI_ARREADY;
    reg  [3:0]  M_AXI_RID;   reg [31:0] M_AXI_RDATA; reg [1:0] M_AXI_RRESP;
    reg         M_AXI_RLAST; reg M_AXI_RVALID; wire M_AXI_RREADY;
    wire        done_led;

    mac_accel_dma_top #(.DATA_WIDTH(DATA_WIDTH), .MAX_LEN(256), .BURST_LEN(16)) dut (
        .S_AXI_ACLK(bus_clk), .S_AXI_ARESETN(aresetn),
        .S_AXI_AWADDR(S_AXI_AWADDR), .S_AXI_AWPROT(3'b000), .S_AXI_AWVALID(S_AXI_AWVALID), .S_AXI_AWREADY(S_AXI_AWREADY),
        .S_AXI_WDATA(S_AXI_WDATA), .S_AXI_WSTRB(S_AXI_WSTRB), .S_AXI_WVALID(S_AXI_WVALID), .S_AXI_WREADY(S_AXI_WREADY),
        .S_AXI_BRESP(S_AXI_BRESP), .S_AXI_BVALID(S_AXI_BVALID), .S_AXI_BREADY(S_AXI_BREADY),
        .S_AXI_ARADDR(S_AXI_ARADDR), .S_AXI_ARPROT(3'b000), .S_AXI_ARVALID(S_AXI_ARVALID), .S_AXI_ARREADY(S_AXI_ARREADY),
        .S_AXI_RDATA(S_AXI_RDATA), .S_AXI_RRESP(S_AXI_RRESP), .S_AXI_RVALID(S_AXI_RVALID), .S_AXI_RREADY(S_AXI_RREADY),
        .M_AXI_ARID(M_AXI_ARID), .M_AXI_ARADDR(M_AXI_ARADDR), .M_AXI_ARLEN(M_AXI_ARLEN),
        .M_AXI_ARSIZE(M_AXI_ARSIZE), .M_AXI_ARBURST(M_AXI_ARBURST), .M_AXI_ARVALID(M_AXI_ARVALID), .M_AXI_ARREADY(M_AXI_ARREADY),
        .M_AXI_RID(M_AXI_RID), .M_AXI_RDATA(M_AXI_RDATA), .M_AXI_RRESP(M_AXI_RRESP),
        .M_AXI_RLAST(M_AXI_RLAST), .M_AXI_RVALID(M_AXI_RVALID), .M_AXI_RREADY(M_AXI_RREADY),
        .mac_clk(mac_clk), .mac_rst(mac_rst), .done_led(done_led)
    );

    // ---- behavioural AXI memory model (identical to tb_mac_accel_dma.v) ----
    localparam MEM_WORDS = 1024;
    reg [31:0] mem [0:MEM_WORDS-1];
    reg [31:0] ar_addr_r; reg [7:0] ar_len_r; reg [3:0] ar_id_r; reg [7:0] beat_cnt; reg burst_active;

    task automatic mem_init; integer i; begin
        for (i = 0; i < MEM_WORDS; i = i + 1) mem[i] = 32'h0;
    end endtask
    task automatic mem_write_h(input [31:0] byte_addr, input signed [15:0] val); begin
        mem[(byte_addr >> 2) & (MEM_WORDS-1)] = {{16{val[15]}}, val};
    end endtask

    always @(posedge bus_clk) begin
        if (!aresetn) begin
            M_AXI_ARREADY <= 0; M_AXI_RVALID <= 0; M_AXI_RDATA <= 0; M_AXI_RLAST <= 0;
            M_AXI_RRESP <= 0; M_AXI_RID <= 0; burst_active <= 0; beat_cnt <= 0;
            ar_addr_r <= 0; ar_len_r <= 0; ar_id_r <= 0;
        end else begin
            if (!burst_active && M_AXI_ARVALID && !M_AXI_ARREADY) begin
                M_AXI_ARREADY <= 1; ar_addr_r <= M_AXI_ARADDR; ar_len_r <= M_AXI_ARLEN; ar_id_r <= M_AXI_ARID;
            end else if (M_AXI_ARREADY) begin
                M_AXI_ARREADY <= 0; burst_active <= 1; beat_cnt <= 0;
            end
            if (burst_active && !M_AXI_RVALID) begin
                M_AXI_RVALID <= 1; M_AXI_RDATA <= mem[(ar_addr_r >> 2) & (MEM_WORDS-1)];
                M_AXI_RLAST <= (beat_cnt == ar_len_r); M_AXI_RID <= ar_id_r;
            end else if (M_AXI_RVALID && M_AXI_RREADY) begin
                if (beat_cnt == ar_len_r) begin
                    M_AXI_RVALID <= 0; M_AXI_RLAST <= 0; burst_active <= 0;
                end else begin
                    beat_cnt <= beat_cnt + 1; ar_addr_r <= ar_addr_r + 4;
                    M_AXI_RDATA <= mem[((ar_addr_r + 4) >> 2) & (MEM_WORDS-1)];
                    M_AXI_RLAST <= ((beat_cnt + 1) == ar_len_r);
                end
            end
        end
    end

    task automatic axi_write(input [7:0] addr, input [31:0] data); begin
        @(posedge bus_clk);
        S_AXI_AWADDR <= addr; S_AXI_AWVALID <= 1; S_AXI_WDATA <= data; S_AXI_WSTRB <= 4'hF;
        S_AXI_WVALID <= 1; S_AXI_BREADY <= 1;
        do @(posedge bus_clk); while (!S_AXI_AWREADY);
        S_AXI_AWVALID <= 0;
        while (!S_AXI_WREADY) @(posedge bus_clk);
        S_AXI_WVALID <= 0;
        while (!S_AXI_BVALID) @(posedge bus_clk);
        @(posedge bus_clk); S_AXI_BREADY <= 0;
    end endtask

    task automatic axi_read(input [7:0] addr, output [31:0] data); begin
        @(posedge bus_clk);
        S_AXI_ARADDR <= addr; S_AXI_ARVALID <= 1; S_AXI_RREADY <= 1;
        while (!S_AXI_ARREADY) @(posedge bus_clk);
        @(posedge bus_clk); S_AXI_ARVALID <= 0;
        while (!S_AXI_RVALID) @(posedge bus_clk);
        data = S_AXI_RDATA;
        @(posedge bus_clk); S_AXI_RREADY <= 0;
    end endtask

    task do_reset; begin
        aresetn = 0; mac_rst = 1;
        S_AXI_AWADDR=0; S_AXI_AWVALID=0; S_AXI_WDATA=0; S_AXI_WSTRB=0; S_AXI_WVALID=0; S_AXI_BREADY=0;
        S_AXI_ARADDR=0; S_AXI_ARVALID=0; S_AXI_RREADY=0;
        repeat (10) @(posedge bus_clk);
        repeat (10) @(posedge mac_clk);
        @(posedge mac_clk); #1; mac_rst = 0;
        repeat (3) @(posedge bus_clk);
        @(posedge bus_clk); #1; aresetn = 1;
        repeat (8) @(posedge bus_clk);
    end endtask

    task wait_done(output reg timed_out); integer cnt; begin
        timed_out = 0; cnt = 0;
        while (done_led !== 1'b1 && cnt < 20000) begin @(posedge bus_clk); cnt = cnt + 1; end
        if (cnt >= 20000) timed_out = 1;
    end endtask

    integer pass_cnt = 0, fail_cnt = 0;
    task run_dma_test(input integer length, input [31:0] src_a_addr, input [31:0] src_b_addr,
                      input signed [31:0] expected, input [255:0] name);
        reg [31:0] res, lat; reg timed_out;
    begin
        do_reset;
        $display("---- %s ----", name);
        $display("  length=%0d  src_a=0x%08h  src_b=0x%08h  expect=%0d", length, src_a_addr, src_b_addr, expected);
        axi_write(ADDR_SRC_A, src_a_addr);
        axi_write(ADDR_SRC_B, src_b_addr);
        axi_write(ADDR_LENGTH, length);
        axi_write(ADDR_CTRL, 32'h1);
        wait_done(timed_out);
        if (timed_out) begin
            $display("  TIMEOUT (done_led never asserted)  <-- possible deadlock/drop");
            fail_cnt = fail_cnt + 1;
        end else begin
            axi_read(ADDR_RESULT, res);
            axi_read(ADDR_LATENCY, lat);
            $display("  HW RESULT=%0d  LATENCY=%0d mac_clk cycles", $signed(res), lat);
            if ($signed(res) == expected) begin
                $display("  PASS"); pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL: got %0d, expected %0d  <-- BACK-PRESSURE DATA DROP", $signed(res), expected);
                fail_cnt = fail_cnt + 1;
            end
        end
    end endtask

    integer i; integer sum;
    initial begin
        $display("========================================");
        $display("  V2 BACK-PRESSURE stress (mac_clk=20MHz << bus_clk=100MHz)");
        $display("========================================");
        mem_init;

        // Test A: length=20, multi-burst, B[i]=1, sum=210
        sum = 0;
        for (i = 0; i < 20; i = i + 1) begin
            mem_write_h(32'h0000_0600 + i*4, i + 1);
            mem_write_h(32'h0000_0700 + i*4, 16'sd1);
            sum = sum + (i + 1);
        end
        run_dma_test(20, 32'h0000_0600, 32'h0000_0700, sum, "BP Test A: length=20");

        // Test B: length=64, lots of bursts + saturation, A[i]=i+1, B[i]=1, sum=2080
        sum = 0;
        for (i = 0; i < 64; i = i + 1) begin
            mem_write_h(32'h0000_0000 + i*4, i + 1);
            mem_write_h(32'h0000_0800 + i*4, 16'sd1);
            sum = sum + (i + 1);
        end
        run_dma_test(64, 32'h0000_0000, 32'h0000_0800, sum, "BP Test B: length=64 (saturated)");

        // Test C: length=37, mixed magnitudes, A[i]=i+1, B[i]=i+1, sum=sum((i+1)^2)
        sum = 0;
        for (i = 0; i < 37; i = i + 1) begin
            mem_write_h(32'h0000_0000 + i*4, i + 1);
            mem_write_h(32'h0000_0900 + i*4, i + 1);
            sum = sum + (i + 1) * (i + 1);
        end
        run_dma_test(37, 32'h0000_0000, 32'h0000_0900, sum, "BP Test C: length=37 (a[i]*b[i])");

        $display("========================================");
        $display("  TOTAL: %0d PASS / %0d FAIL", pass_cnt, fail_cnt);
        $display("========================================");
        #100; $finish;
    end

    initial begin
        #2000000;
        $display("[TB] Global timeout reached -- aborting");
        $finish;
    end

endmodule
