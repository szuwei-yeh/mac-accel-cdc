`timescale 1ns/1ps

// V3 CSR byte enables only: no memory responses, and no DMA completion claim.
// Drive on negedges and count handshakes on posedges, with independent AW/W.
// Non-operational LENGTH bit patterns test storage only and are never started.
module tb_mac_accel_dma_rob_axil_wstrb;
    localparam LIMIT = 80;
    reg bus_clk = 0, mac_clk = 0;
    always #5 bus_clk = ~bus_clk;
    always #3.75 mac_clk = ~mac_clk;
    reg resetn = 0, mac_rst = 1;
    reg [7:0] awaddr = 0, araddr = 0;
    reg awvalid = 0, wvalid = 0, bready = 0;
    reg arvalid = 0, rready = 0;
    reg [31:0] wdata = 0;
    reg [3:0] wstrb = 0;
    wire awready, wready, bvalid, arready, rvalid;
    wire [1:0] bresp, rresp;
    wire [31:0] rdata;

    mac_accel_dma_rob_top #(.MAX_OUTSTANDING(8)) dut (
        .S_AXI_ACLK(bus_clk), .S_AXI_ARESETN(resetn),
        .S_AXI_AWADDR(awaddr), .S_AXI_AWPROT(3'b0),
        .S_AXI_AWVALID(awvalid), .S_AXI_AWREADY(awready),
        .S_AXI_WDATA(wdata), .S_AXI_WSTRB(wstrb),
        .S_AXI_WVALID(wvalid), .S_AXI_WREADY(wready),
        .S_AXI_BRESP(bresp), .S_AXI_BVALID(bvalid), .S_AXI_BREADY(bready),
        .S_AXI_ARADDR(araddr), .S_AXI_ARPROT(3'b0),
        .S_AXI_ARVALID(arvalid), .S_AXI_ARREADY(arready),
        .S_AXI_RDATA(rdata), .S_AXI_RRESP(rresp),
        .S_AXI_RVALID(rvalid), .S_AXI_RREADY(rready),
        .M_AXI_ARREADY(1'b0), .M_AXI_RID(4'b0), .M_AXI_RDATA(32'b0),
        .M_AXI_RRESP(2'b0), .M_AXI_RLAST(1'b0), .M_AXI_RVALID(1'b0),
        .mac_clk(mac_clk), .mac_rst(mac_rst)
    );

    integer pass_cnt = 0, fail_cnt = 0;
    integer starts = 0, aws = 0, ws = 0, bs = 0;
    reg stalled_b = 0, previous_start = 0;
    reg [1:0] held_bresp;
    always @(posedge bus_clk) begin
        if (!resetn) begin
            starts = 0; aws = 0; ws = 0; bs = 0;
            stalled_b = 0; previous_start = 0;
        end else begin
            if (stalled_b && (bvalid !== 1'b1 || bresp !== held_bresp))
                $fatal(1, "B response changed under backpressure");
            stalled_b = bvalid && !bready;
            held_bresp = bresp;
            if (dut.start_pulse_bus) starts = starts + 1;
            if (previous_start && dut.start_pulse_bus)
                $fatal(1, "Start pulse exceeded one cycle");
            previous_start = dut.start_pulse_bus;
            if (awvalid && awready) aws = aws + 1;
            if (wvalid && wready) ws = ws + 1;
            if (bvalid && bready) bs = bs + 1;
        end
    end

    task automatic send_aw(input [7:0] addr, input integer delay_cycles);
        integer n;
        reg accepted;
    begin
        repeat (delay_cycles) @(negedge bus_clk);
        @(negedge bus_clk); awaddr = addr; awvalid = 1;
        n = 0; accepted = 0;
        while (!accepted && n < LIMIT) begin
            @(posedge bus_clk); accepted = awready === 1'b1; n = n + 1;
        end
        if (!accepted) $fatal(1, "AW transaction timeout");
        @(negedge bus_clk); awvalid = 0; awaddr = 8'hfc;
    end
    endtask

    task automatic send_w(input [31:0] data, input [3:0] strb,
                          input integer delay_cycles);
        integer n;
        reg accepted;
    begin
        repeat (delay_cycles) @(negedge bus_clk);
        @(negedge bus_clk); wdata = data; wstrb = strb; wvalid = 1;
        n = 0; accepted = 0;
        while (!accepted && n < LIMIT) begin
            @(posedge bus_clk); accepted = wready === 1'b1; n = n + 1;
        end
        if (!accepted) $fatal(1, "W transaction timeout");
        // Poison live payload after the handshake to verify latched data/strobes.
        @(negedge bus_clk); wvalid = 0; wdata = ~data; wstrb = ~strb;
    end
    endtask

    task automatic issue(input [7:0] addr, input [31:0] data,
                         input [3:0] strb, input integer skew);
    begin
        fork
            send_aw(addr, skew < 0 ? -skew : 0);
            send_w(data, strb, skew > 0 ? skew : 0);
        join
    end
    endtask

    task automatic wait_b;
        integer n;
    begin
        n = 0;
        while (bvalid !== 1'b1 && n < LIMIT) begin
            @(negedge bus_clk); n = n + 1;
        end
        if (bvalid !== 1'b1) $fatal(1, "B transaction timeout");
        if (bresp !== 2'b00) $fatal(1, "Expected OKAY write response");
    end
    endtask

    task automatic drain_b;
    begin
        @(negedge bus_clk); bready = 1;
        @(posedge bus_clk);
        if (bvalid !== 1'b1 || bresp !== 2'b00)
            $fatal(1, "Missing response at B handshake");
        @(negedge bus_clk); bready = 0;
    end
    endtask

    task automatic write_csr(input [7:0] addr, input [31:0] data,
                             input [3:0] strb, input integer skew,
                             input integer hold_cycles);
    begin
        issue(addr, data, strb, skew);
        wait_b;
        repeat (hold_cycles) begin
            @(negedge bus_clk);
            if (bvalid !== 1'b1 || bresp !== 2'b00)
                $fatal(1, "Unstable stalled B response");
        end
        drain_b;
        repeat (4) @(negedge bus_clk);
        if (bvalid !== 1'b0) $fatal(1, "Duplicate B response");
    end
    endtask

    task automatic read_csr(input [7:0] addr, output [31:0] value);
        integer n;
        reg accepted;
    begin
        @(negedge bus_clk); araddr = addr; arvalid = 1;
        n = 0; accepted = 0;
        while (!accepted && n < LIMIT) begin
            @(posedge bus_clk); accepted = arready === 1'b1; n = n + 1;
        end
        if (!accepted) $fatal(1, "AR transaction timeout");
        @(negedge bus_clk); arvalid = 0;
        n = 0;
        while (rvalid !== 1'b1 && n < LIMIT) begin
            @(negedge bus_clk); n = n + 1;
        end
        if (rvalid !== 1'b1) $fatal(1, "R transaction timeout");
        if (rresp !== 2'b00) $fatal(1, "Expected OKAY read response");
        value = rdata; rready = 1;
        @(posedge bus_clk);
        @(negedge bus_clk); rready = 0;
    end
    endtask

    task automatic seed;
    begin
        @(negedge bus_clk); resetn = 0; mac_rst = 1;
        awvalid = 0; wvalid = 0; bready = 0; arvalid = 0; rready = 0;
        repeat (6) @(negedge bus_clk);
        resetn = 1; mac_rst = 0;
        repeat (4) @(negedge bus_clk);
        write_csr(8'h04, 32'h11223344, 4'hf, 0, 0);
        write_csr(8'h08, 32'h55667788, 4'hf, 0, 0);
        write_csr(8'h0c, 32'h00000040, 4'hf, 0, 0);
    end
    endtask

    task automatic check_state(input [255:0] name, input [31:0] exp_a,
                               input [31:0] exp_b, input [15:0] exp_len,
                               input integer exp_starts, input integer exp_writes);
        reg [31:0] got_a, got_b, got_len;
    begin
        read_csr(8'h04, got_a);
        read_csr(8'h08, got_b);
        read_csr(8'h0c, got_len);
        if (got_a !== exp_a || got_b !== exp_b || got_len !== {16'b0, exp_len}
            || dut.reg_src_a !== exp_a || dut.reg_src_b !== exp_b
            || dut.reg_length !== exp_len || starts != exp_starts
            || dut.start_toggle_bus !== (exp_starts != 0)
            || aws != exp_writes || ws != exp_writes || bs != exp_writes) begin
            fail_cnt = fail_cnt + 1;
            $display("FAIL: %0s A=%08h/%08h B=%08h/%08h LEN=%04h/%04h starts=%0d/%0d AW/W/B=%0d/%0d/%0d expected=%0d",
                     name, got_a, exp_a, got_b, exp_b, got_len, exp_len,
                     starts, exp_starts, aws, ws, bs, exp_writes);
        end else begin
            pass_cnt = pass_cnt + 1;
            $display("PASS: %0s", name);
        end
    end
    endtask

    task automatic run_case(input [255:0] name, input [7:0] addr,
                            input [31:0] data, input [3:0] strb,
                            input integer skew, input integer hold_cycles,
                            input [31:0] exp_a, input [31:0] exp_b,
                            input [15:0] exp_len, input integer exp_starts);
    begin
        seed;
        write_csr(addr, data, strb, skew, hold_cycles);
        check_state(name, exp_a, exp_b, exp_len, exp_starts, 4);
    end
    endtask

    initial begin
        run_case("full A", 8'h04, 32'haabbccdc, 4'hf, 0, 0, 32'haabbccdc, 32'h55667788, 16'h0040, 0);
        run_case("full B", 8'h08, 32'haabbccdc, 4'hf, 0, 0, 32'h11223344, 32'haabbccdc, 16'h0040, 0);
        run_case("full LENGTH 16-bit", 8'h0c, 32'hdead1234, 4'hf, 0, 0, 32'h11223344, 32'h55667788, 16'h1234, 0);
        run_case("zero A", 8'h04, 32'hdeadbeef, 4'h0, 0, 0, 32'h11223344, 32'h55667788, 16'h0040, 0);
        run_case("zero B", 8'h08, 32'hdeadbeef, 4'h0, 0, 0, 32'h11223344, 32'h55667788, 16'h0040, 0);
        run_case("zero LENGTH", 8'h0c, 32'hdeadbeef, 4'h0, 0, 0, 32'h11223344, 32'h55667788, 16'h0040, 0);
        run_case("zero CTRL no start", 8'h00, 32'h00000001, 4'h0, 0, 6, 32'h11223344, 32'h55667788, 16'h0040, 0);
        run_case("A lane 0", 8'h04, 32'haabbccdc, 4'h1, 0, 0, 32'h112233dc, 32'h55667788, 16'h0040, 0);
        run_case("A lane 3", 8'h04, 32'haabbccdc, 4'h8, 0, 0, 32'haa223344, 32'h55667788, 16'h0040, 0);
        run_case("A mixed lanes 0/2", 8'h04, 32'haabbccdc, 4'h5, 0, 0, 32'h11bb33dc, 32'h55667788, 16'h0040, 0);
        run_case("B lane 0", 8'h08, 32'haabbccdc, 4'h1, 0, 0, 32'h11223344, 32'h556677dc, 16'h0040, 0);
        run_case("B lane 3", 8'h08, 32'haabbccdc, 4'h8, 0, 0, 32'h11223344, 32'haa667788, 16'h0040, 0);
        run_case("B mixed lanes 1/3", 8'h08, 32'haabbccdc, 4'ha, 0, 0, 32'h11223344, 32'haa66cc88, 16'h0040, 0);
        run_case("LENGTH lane 0", 8'h0c, 32'hdeadbe20, 4'h1, 0, 0, 32'h11223344, 32'h55667788, 16'h0020, 0);
        run_case("LENGTH lane 1", 8'h0c, 32'hdead01ff, 4'h2, 0, 0, 32'h11223344, 32'h55667788, 16'h0140, 0);
        run_case("LENGTH upper lanes ignored", 8'h0c, 32'hdeadbeef, 4'hc, 0, 0, 32'h11223344, 32'h55667788, 16'h0040, 0);
        run_case("LENGTH mixed lanes 0/2", 8'h0c, 32'h12345621, 4'h5, 0, 0, 32'h11223344, 32'h55667788, 16'h0021, 0);
        run_case("CTRL low byte start once", 8'h00, 32'h00000001, 4'h1, 0, 6, 32'h11223344, 32'h55667788, 16'h0040, 1);
        run_case("CTRL full word start once", 8'h00, 32'h00000001, 4'hf, 0, 6, 32'h11223344, 32'h55667788, 16'h0040, 1);
        run_case("CTRL upper byte no start", 8'h00, 32'h00000001, 4'h8, 0, 0, 32'h11223344, 32'h55667788, 16'h0040, 0);
        run_case("CTRL mixed no low byte", 8'h00, 32'h00000001, 4'ha, 0, 0, 32'h11223344, 32'h55667788, 16'h0040, 0);
        run_case("CTRL enabled byte bit0 clear", 8'h00, 32'hfffffffe, 4'h1, 0, 0, 32'h11223344, 32'h55667788, 16'h0040, 0);
        run_case("AW before W partial A", 8'h04, 32'haabbccdc, 4'h5, 7, 0, 32'h11bb33dc, 32'h55667788, 16'h0040, 0);
        run_case("W before AW partial B", 8'h08, 32'haabbccdc, 4'ha, -7, 0, 32'h11223344, 32'haa66cc88, 16'h0040, 0);
        run_case("B stalled partial A", 8'h04, 32'haabbccdc, 4'h5, 0, 12, 32'h11bb33dc, 32'h55667788, 16'h0040, 0);

        // A second independent AW/W pair can be held behind a stalled response.
        seed;
        issue(8'h04, 32'haabbccdc, 4'hf, 0);
        wait_b;
        issue(8'h08, 32'h99aabbcc, 4'hf, -5);
        repeat (8) begin
            @(negedge bus_clk);
            if (bvalid !== 1'b1 || bresp !== 2'b00 || bs != 3
                || dut.reg_src_a !== 32'haabbccdc || dut.reg_src_b !== 32'h55667788)
                $fatal(1, "Queued write committed before stalled response drained");
        end
        drain_b;
        wait_b;
        drain_b;
        repeat (4) @(negedge bus_clk);
        if (bvalid !== 1'b0) $fatal(1, "Duplicate queued B response");
        check_state("queued write behind stalled B", 32'haabbccdc, 32'h99aabbcc, 16'h0040, 0, 5);

        $display("TOTAL: %0d PASS / %0d FAIL", pass_cnt, fail_cnt);
        if (fail_cnt != 0) $fatal(1, "WSTRB regression failed");
        $finish;
    end

    initial begin
        #2000000;
        $fatal(1, "Global transaction timeout");
    end
endmodule
