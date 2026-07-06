`timescale 1ns/1ps

// tb_mac_dma_baseline.v
// -----------------------------------------------------------------------------
// STEP 0 baseline + measurement harness for the outstanding-transaction / ROB
// project.  Drives the UNMODIFIED V2 DMA top (mac_accel_dma_top) against the new
// configurable-latency AXI read-slave (axi_read_mem_model) and measures MAC
// datapath utilization at a fixed memory latency L.
//
// This file does NOT modify any V2 RTL or the V2 regression testbench
// (sim/tb_mac_accel_dma.v); it is an additive, separate harness.
//
// -----------------------------------------------------------------------------
// HOW UTILIZATION IS MEASURED (single bus_clk domain, no CDC ambiguity)
// -----------------------------------------------------------------------------
// Measurement window  T_active = number of bus_clk cycles for which the DMA read
//   front-end is busy, i.e. dut.u_dma.dma_busy == 1.  dma_busy goes high one
//   cycle after `start` and low when the FSM reaches S_DONE, so it brackets the
//   ENTIRE read path (all AR issue + all R beats of both the A phase and the B
//   phase, including the per-burst latency stalls).  The fixed MAC pipeline
//   drain after the last operand pair runs in mac_clk and is latency-independent,
//   so it is deliberately excluded -- T_active isolates the part that memory
//   latency governs and that the ROB/outstanding work will improve.
//
// Useful work delivered into the datapath:
//   pairs  = count of {last,a,b} stream handshakes (stream_valid && stream_ready)
//            at the DMA output.  Exactly N over a length-N run: one operand pair
//            -> one multiply-accumulate in mac_pe.  The DMA-side stream can
//            deliver at most one pair per bus_clk cycle, so one-pair-per-cycle is
//            the datapath's peak feed rate.
//   beats  = count of AXI R-channel handshakes (M_AXI_RVALID && M_AXI_RREADY).
//            Exactly 2N: N beats for vector A, N beats for vector B.
//
// Reported metrics:
//   U_mac = pairs / T_active   (MAC datapath feed utilization; 100% = one MAC
//                               operand pair delivered every bus_clk cycle)
//   U_bus = beats / T_active   (AXI read-data-bus utilization; classic memory-
//                               bandwidth view).  Note U_bus == 2*U_mac exactly,
//                               because the single-outstanding engine reads A
//                               fully, THEN B (2 beats per pair, sequentially) --
//                               so even at zero latency the MAC feed is capped at
//                               ~50% of the read bus.  Hiding latency lifts both;
//                               concurrent A/B prefetch (optional Step 6) would
//                               attack the structural 2x.
// -----------------------------------------------------------------------------

module tb_mac_dma_baseline;

    localparam DATA_WIDTH = 16;
    localparam MEM_WORDS  = 1024;     // 4 KB, matches the memory model default

    // S_AXI control register byte offsets
    localparam ADDR_CTRL    = 8'h00;
    localparam ADDR_SRC_A   = 8'h04;
    localparam ADDR_SRC_B   = 8'h08;
    localparam ADDR_LENGTH  = 8'h0C;
    localparam ADDR_RESULT  = 8'h10;
    localparam ADDR_LATENCY = 8'h14;  // DUT's mac_clk latency counter (NOT mem latency)

    //----------------------------------------------------
    // Clocks & reset
    //----------------------------------------------------
    reg bus_clk = 0;
    reg mac_clk = 0;
    always #5.0  bus_clk = ~bus_clk;   // 100 MHz
    always #3.75 mac_clk = ~mac_clk;   // ~133 MHz

    reg aresetn;
    reg mac_rst;

    // memory access latency (cycles), sampled by the model at each AR accept
    reg [15:0] mem_latency;

    //----------------------------------------------------
    // S_AXI (master driver) signals
    //----------------------------------------------------
    reg  [7:0]  S_AXI_AWADDR;
    reg         S_AXI_AWVALID;
    wire        S_AXI_AWREADY;
    reg  [31:0] S_AXI_WDATA;
    reg  [3:0]  S_AXI_WSTRB;
    reg         S_AXI_WVALID;
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
    // M_AXI (DUT master <-> memory model).  All wires now -- the memory model
    // drives the slave-side signals.
    //----------------------------------------------------
    wire [3:0]  M_AXI_ARID;
    wire [31:0] M_AXI_ARADDR;
    wire [7:0]  M_AXI_ARLEN;
    wire [2:0]  M_AXI_ARSIZE;
    wire [1:0]  M_AXI_ARBURST;
    wire        M_AXI_ARVALID;
    wire        M_AXI_ARREADY;

    wire [3:0]  M_AXI_RID;
    wire [31:0] M_AXI_RDATA;
    wire [1:0]  M_AXI_RRESP;
    wire        M_AXI_RLAST;
    wire        M_AXI_RVALID;
    wire        M_AXI_RREADY;

    wire        done_led;

    //----------------------------------------------------
    // DUT (unmodified V2 top)
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
    // Configurable-latency AXI read-slave (the Step-0 memory model)
    //----------------------------------------------------
    axi_read_mem_model #(
        .AXI_ID_WIDTH   (4),
        .AXI_ADDR_WIDTH (32),
        .AXI_DATA_WIDTH (32),
        .MEM_WORDS      (MEM_WORDS)
    ) u_mem (
        .clk        (bus_clk),
        .resetn     (aresetn),
        .ar_latency (mem_latency),

        .arid       (M_AXI_ARID),
        .araddr     (M_AXI_ARADDR),
        .arlen      (M_AXI_ARLEN),
        .arsize     (M_AXI_ARSIZE),
        .arburst    (M_AXI_ARBURST),
        .arvalid    (M_AXI_ARVALID),
        .arready    (M_AXI_ARREADY),

        .rid        (M_AXI_RID),
        .rdata      (M_AXI_RDATA),
        .rresp      (M_AXI_RRESP),
        .rlast      (M_AXI_RLAST),
        .rvalid     (M_AXI_RVALID),
        .rready     (M_AXI_RREADY)
    );

    //----------------------------------------------------
    // Memory preload helper (signed 16-bit value -> sign-extended 32-bit word)
    //----------------------------------------------------
    task automatic mem_write_h(input [31:0] byte_addr, input signed [15:0] val);
    begin
        u_mem.mem[(byte_addr >> 2) & (MEM_WORDS-1)] = {{16{val[15]}}, val};
    end
    endtask

    //----------------------------------------------------
    // Utilization counters (all bus_clk domain).  Zeroed on reset; only
    // accumulate while the DMA read front-end is busy, so each measured run
    // starts from zero (do_reset precedes every run).
    //----------------------------------------------------
    integer m_active;   // bus_clk cycles with dma_busy high  (= T_active)
    integer m_arh;      // AR handshakes (bursts issued)
    integer m_beats;    // R-channel beats accepted           (= 2N)
    integer m_pairs;    // {last,a,b} pairs delivered to MAC  (= N)

    always @(posedge bus_clk) begin
        if (!aresetn) begin
            m_active <= 0;
            m_arh    <= 0;
            m_beats  <= 0;
            m_pairs  <= 0;
        end else begin
            if (dut.u_dma.dma_busy)
                m_active <= m_active + 1;
            if (M_AXI_ARVALID && M_AXI_ARREADY)
                m_arh <= m_arh + 1;
            if (M_AXI_RVALID && M_AXI_RREADY)
                m_beats <= m_beats + 1;
            if (dut.u_dma.stream_valid && dut.u_dma.stream_ready)
                m_pairs <= m_pairs + 1;
        end
    end

    //----------------------------------------------------
    // S_AXI master driver tasks (identical handshake to the V2 regression TB)
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
    // Wait for done_led with a generous timeout (big runs at high L)
    //----------------------------------------------------
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

    //----------------------------------------------------
    // Result bookkeeping
    //----------------------------------------------------
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    // headline number captured for the summary line
    integer head_umac_h;
    integer head_ubus_h;
    integer head_active;
    integer head_len;
    integer head_lat;

    //----------------------------------------------------
    // One measured run: program a length-N transfer at latency L, self-check the
    // dot-product, and report utilization.  Pattern A[i]=i+1, B[i]=2.
    //----------------------------------------------------
    task run_measured(
        input integer length,
        input [31:0]  src_a,
        input [31:0]  src_b,
        input [15:0]  latency,
        input         is_headline
    );
        reg [31:0] res, mlat;
        reg        timed_out;
        integer    i;
        integer    expected;
        integer    umac_h, ubus_h;   // utilization * 100  (hundredths of a percent)
    begin
        do_reset;
        mem_latency = latency;

        // preload operands and compute expected dot-product
        expected = 0;
        for (i = 0; i < length; i = i + 1) begin
            mem_write_h(src_a + i*4, i + 1);
            mem_write_h(src_b + i*4, 16'sd2);
            expected = expected + 2 * (i + 1);
        end

        // program control registers and kick off
        axi_write(ADDR_SRC_A,  src_a);
        axi_write(ADDR_SRC_B,  src_b);
        axi_write(ADDR_LENGTH, length);
        axi_write(ADDR_CTRL,   32'h0000_0001);

        wait_done(timed_out);

        $display("---- N=%0d  L=%0d cycles ----", length, latency);

        if (timed_out) begin
            $display("  TIMEOUT (done_led never asserted)");
            fail_cnt = fail_cnt + 1;
        end else begin
            axi_read(ADDR_RESULT,  res);
            axi_read(ADDR_LATENCY, mlat);

            // functional self-check
            if ($signed(res) !== expected) begin
                $display("  FUNCTIONAL FAIL: result=%0d expected=%0d",
                         $signed(res), expected);
                fail_cnt = fail_cnt + 1;
            end else begin
                pass_cnt = pass_cnt + 1;

                // sanity-check the counters against the analytic expectation
                if (m_pairs != length)
                    $display("  WARN: pairs=%0d != N=%0d", m_pairs, length);
                if (m_beats != 2*length)
                    $display("  WARN: beats=%0d != 2N=%0d", m_beats, 2*length);

                umac_h = (m_pairs * 10000) / m_active;   // pairs / active, *100%
                ubus_h = (m_beats * 10000) / m_active;   // beats / active, *100%

                $display("  result=%0d (OK)   mac_latency=%0d mac_clk cyc", $signed(res), mlat);
                $display("  T_active(dma_busy) = %0d bus_clk cycles", m_active);
                $display("  AR bursts issued   = %0d", m_arh);
                $display("  R beats accepted   = %0d   (2N=%0d)", m_beats, 2*length);
                $display("  MAC pairs delivered= %0d   (N =%0d)", m_pairs, length);
                $display("  cycles per element = %0d", m_active / length);
                $display("  U_mac (pairs/active) = %0d.%02d %%", umac_h/100, umac_h%100);
                $display("  U_bus (beats/active) = %0d.%02d %%", ubus_h/100, ubus_h%100);

                if (is_headline) begin
                    head_umac_h = umac_h;
                    head_ubus_h = ubus_h;
                    head_active = m_active;
                    head_len    = length;
                    head_lat    = latency;
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
        $display("  STEP 0 baseline: single-outstanding DMA");
        $display("  bus_clk=100MHz  mac_clk=133MHz  BURST_LEN=16");
        $display("  U_mac = MAC operand pairs / DMA-busy cycles");
        $display("  U_bus = AXI R beats        / DMA-busy cycles");
        $display("==================================================");
        $display("");

        // PRIMARY baseline: 64 elements (4 full 16-beat bursts/vector), L=100
        run_measured(64,  32'h0000_0000, 32'h0000_0800, 16'd100, 1'b1);

        // supporting context at L=100: small (1 burst/vec) and large (16 bursts/vec)
        run_measured(16,  32'h0000_0000, 32'h0000_0800, 16'd100, 1'b0);
        run_measured(256, 32'h0000_0000, 32'h0000_0800, 16'd100, 1'b0);

        // zero-latency structural ceiling for this single-outstanding A-then-B path
        run_measured(64,  32'h0000_0000, 32'h0000_0800, 16'd0,   1'b0);

        // higher latency -> lower utilization (latency-sensitivity trend)
        run_measured(64,  32'h0000_0000, 32'h0000_0800, 16'd200, 1'b0);

        $display("==================================================");
        $display("  TOTAL: %0d PASS / %0d FAIL", pass_cnt, fail_cnt);
        $display("  BASELINE (headline): N=%0d at L=%0d cycles", head_len, head_lat);
        $display("    U_mac = %0d.%02d %%   U_bus = %0d.%02d %%   (T_active=%0d)",
                 head_umac_h/100, head_umac_h%100,
                 head_ubus_h/100, head_ubus_h%100, head_active);
        $display("==================================================");

        #100;
        $finish;
    end

    // Global timeout safety net
    initial begin
        #5000000;
        $display("[TB] Global timeout reached -- aborting");
        $finish;
    end

endmodule
