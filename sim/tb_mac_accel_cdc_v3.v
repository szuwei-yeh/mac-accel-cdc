`timescale 1ns/1ps

// tb_mac_accel_cdc_v3.v
// True dual-clock CDC testbench
// bus_clk = 100 MHz (period = 10ns)
// mac_clk = 133 MHz (period = 7.5ns) — non-integer ratio to stress CDC paths

module tb_mac_accel_cdc_v3;

    localparam DATA_WIDTH = 16;
    localparam VEC_LEN    = 4;

    localparam ADDR_CTRL    = 4'h0;
    localparam ADDR_AIN     = 4'h1;
    localparam ADDR_BIN     = 4'h2;
    localparam ADDR_MASK    = 4'h3;
    localparam ADDR_RESULT  = 4'h4;
    localparam ADDR_LATENCY = 4'h5;

    // -------------------------------------------------------
    // Two INDEPENDENT clocks (non-integer ratio = real CDC)
    // -------------------------------------------------------
    reg bus_clk;
    reg mac_clk;

    initial bus_clk = 0;
    always #5.0  bus_clk = ~bus_clk;   // 100 MHz

    initial mac_clk = 0;
    always #3.75 mac_clk = ~mac_clk;   // ~133 MHz  (non-integer ratio w/ bus_clk)

    // -------------------------------------------------------
    // Reset (independent per domain)
    // -------------------------------------------------------
    reg bus_rst;
    reg mac_rst;

    // -------------------------------------------------------
    // Bus interface signals (driven in bus_clk domain)
    // -------------------------------------------------------
    reg         we;
    reg         re;
    reg  [3:0]  addr;
    reg  [31:0] wdata;
    wire [31:0] rdata;
    wire        done_led;

    integer pass_cnt;
    integer fail_cnt;

    // -------------------------------------------------------
    // DUT
    // -------------------------------------------------------
    mac_accel #(
        .DATA_WIDTH(DATA_WIDTH),
        .VEC_LEN   (VEC_LEN)
    ) dut (
        .bus_clk (bus_clk),
        .bus_rst (bus_rst),
        .mac_clk (mac_clk),
        .mac_rst (mac_rst),
        .we      (we),
        .re      (re),
        .addr    (addr),
        .wdata   (wdata),
        .rdata   (rdata),
        .done_led(done_led)
    );

    // -------------------------------------------------------
    // Tasks — all synchronised to bus_clk
    // -------------------------------------------------------
    task write_reg(input [3:0] a, input [31:0] d);
    begin
        @(posedge bus_clk); #1;
        addr  <= a;
        wdata <= d;
        we    <= 1'b1;
        re    <= 1'b0;
        @(posedge bus_clk); #1;
        we    <= 1'b0;
    end
    endtask

    task read_reg(input [3:0] a, output [31:0] d);
    begin
        @(posedge bus_clk); #1;
        addr <= a;
        re   <= 1'b1;
        we   <= 1'b0;
        @(posedge bus_clk); #1;
        d = rdata;
        re <= 1'b0;
    end
    endtask

    // Reset both domains independently (different de-assert timing
    // to stress the CDC paths further)
    task do_reset;
    begin
        we      = 0;
        re      = 0;
        addr    = 0;
        wdata   = 0;
        bus_rst = 1;
        mac_rst = 1;

        // Hold reset for several cycles of EACH clock
        repeat (8) @(posedge bus_clk);
        repeat (8) @(posedge mac_clk);

        // De-assert mac_rst first, then bus_rst a few cycles later
        // (worst-case ordering for CDC reset synchronisation)
        @(posedge mac_clk); #1; mac_rst = 0;
        repeat (3) @(posedge bus_clk);
        @(posedge bus_clk); #1; bus_rst = 0;

        // Let both domains settle
        repeat (5) @(posedge bus_clk);
    end
    endtask

    // -------------------------------------------------------
    // Timeout watchdog — avoids infinite hang on CDC failure
    // -------------------------------------------------------
    task wait_done_timeout;
        output timed_out;
        integer cnt;
    begin
        timed_out = 0;
        cnt = 0;
        while (done_led !== 1'b1 && cnt < 2000) begin
            @(posedge bus_clk);
            cnt = cnt + 1;
        end
        if (cnt >= 2000) begin
            timed_out = 1;
            $display("  [TIMEOUT] done_led never asserted — possible CDC failure!");
        end
    end
    endtask

    // -------------------------------------------------------
    // Main test task
    // -------------------------------------------------------
    task run_test(
        input signed [15:0] a0, a1, a2, a3,
        input signed [15:0] b0, b1, b2, b3,
        input signed [31:0] expected,
        input [255:0] test_name
    );
        reg [31:0] res, lat;
        reg timed_out;
    begin
        do_reset;

        $display("---- %s ----", test_name);
        $display("  A=[%0d,%0d,%0d,%0d] B=[%0d,%0d,%0d,%0d] expect=%0d",
                  a0, a1, a2, a3, b0, b1, b2, b3, expected);

        // Set mask → enables all 4 elements and resets load index
        write_reg(ADDR_MASK, 32'h0000_000F);

        // Load vector A (sign-extended to 32 bits)
        write_reg(ADDR_AIN, {{16{a0[15]}}, a0});
        write_reg(ADDR_AIN, {{16{a1[15]}}, a1});
        write_reg(ADDR_AIN, {{16{a2[15]}}, a2});
        write_reg(ADDR_AIN, {{16{a3[15]}}, a3});

        // Load vector B
        write_reg(ADDR_BIN, {{16{b0[15]}}, b0});
        write_reg(ADDR_BIN, {{16{b1[15]}}, b1});
        write_reg(ADDR_BIN, {{16{b2[15]}}, b2});
        write_reg(ADDR_BIN, {{16{b3[15]}}, b3});

        // Trigger — start pulse crosses bus→mac via toggle synchronizer
        write_reg(ADDR_CTRL, 32'h0000_0001);

        // Wait for done (synchronized back to bus_clk domain)
        wait_done_timeout(timed_out);

        if (!timed_out) begin
            @(posedge bus_clk);
            read_reg(ADDR_RESULT,  res);
            read_reg(ADDR_LATENCY, lat);

            $display("  HW RESULT=%0d  LATENCY=%0d cycles (mac_clk)",
                      $signed(res), lat);

            if ($signed(res) == expected) begin
                $display("  PASS");
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL: got %0d, expected %0d", $signed(res), expected);
                fail_cnt = fail_cnt + 1;
            end
        end else begin
            fail_cnt = fail_cnt + 1;
        end
    end
    endtask

    // -------------------------------------------------------
    // Stimulus
    // -------------------------------------------------------
    initial begin
        pass_cnt = 0;
        fail_cnt = 0;

        $display("========================================");
        $display("  mac_accel TRUE CDC Testbench");
        $display("  bus_clk=100MHz  mac_clk=133MHz");
        $display("========================================");

        // Test 1: Basic dot product
        run_test( 1,  2,  3,  4,
                 10, 20, 30, 40,
                 300, "Test 1: Basic");

        // Test 2: All zeros
        run_test(0, 0, 0, 0,
                 0, 0, 0, 0,
                 0, "Test 2: All zeros");

        // Test 3: Max positive
        run_test(32767, 32767, 32767, 32767,
                 1, 1, 1, 1,
                 131068, "Test 3: Max positive");

        // Test 4: Negative A  (-1*1 + -2*2 + -3*3 + -4*4 = -30)
        run_test(-1, -2, -3, -4,
                  1,  2,  3,  4,
                 -30, "Test 4: Negative A");

        // Test 5: Both negative (-1*-1 + -2*-2 + -3*-3 + -4*-4 = 30)
        run_test(-1, -2, -3, -4,
                 -1, -2, -3, -4,
                  30, "Test 5: Both negative");

        // Test 6: Orthogonal vectors
        run_test(1, 0, 1, 0,
                 0, 1, 0, 1,
                 0, "Test 6: Orthogonal");

        // Test 7: Back-to-back — stress CDC with no extra delay between runs
        // (resets between runs via do_reset inside run_test, but clocks keep running)
        run_test(3, 3, 3, 3,
                 3, 3, 3, 3,
                 36, "Test 7: Back-to-back stress");

        // Test 8: 32-bit overflow check
        // -32768 * -32768 = 1,073,741,824 per element
        // sum of 4 = 4,294,967,296 = 2^32 → wraps to 0 in 32-bit result
        run_test(-32768, -32768, -32768, -32768,
                 -32768, -32768, -32768, -32768,
                 32'h00000000, "Test 8: 32-bit overflow (expect wrap=0)");

        $display("========================================");
        $display("  TOTAL: %0d PASS / %0d FAIL", pass_cnt, fail_cnt);
        $display("========================================");

        #100;
        $finish;
    end

endmodule