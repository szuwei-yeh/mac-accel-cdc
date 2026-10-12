`timescale 1ns/1ps

// V3 invalid LENGTH admission, legal jobs, recovery, WSTRB and B backpressure.
// Test-only hierarchy observes protocol events; stimulus uses AXI-Lite.
module tb_mac_accel_dma_rob_length;
    localparam LIMIT = 80;
    localparam MAX_LEN = 256;
    localparam JOB_LIMIT = 5000;
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

    wire [3:0] mid, rid;
    wire [31:0] maddr, mdata;
    wire [7:0] mlen;
    wire [2:0] msize;
    wire [1:0] mburst, mrresp;
    wire marvalid, marready, mrvalid, mrready, mlast;
    reg ar_stall = 0;
    mac_accel_dma_rob_top #(.MAX_LEN(MAX_LEN), .MAX_OUTSTANDING(8)) dut (
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
        .M_AXI_ARID(mid), .M_AXI_ARADDR(maddr), .M_AXI_ARLEN(mlen),
        .M_AXI_ARSIZE(msize), .M_AXI_ARBURST(mburst), .M_AXI_ARVALID(marvalid),
        .M_AXI_ARREADY(marready), .M_AXI_RID(rid), .M_AXI_RDATA(mdata),
        .M_AXI_RRESP(mrresp), .M_AXI_RLAST(mlast), .M_AXI_RVALID(mrvalid),
        .M_AXI_RREADY(mrready),
        .mac_clk(mac_clk), .mac_rst(mac_rst)
    );

    axi_read_mem_model_ooo #(.MEM_WORDS(2048), .MAX_OUTSTANDING(8)) mem (
        .clk(bus_clk), .resetn(resetn), .ar_latency(16'd12),
        .ar_stall(ar_stall), .ooo_en(1'b1),
        .arid(mid), .araddr(maddr), .arlen(mlen), .arsize(msize),
        .arburst(mburst), .arvalid(marvalid), .arready(marready),
        .rid(rid), .rdata(mdata), .rresp(mrresp), .rlast(mlast),
        .rvalid(mrvalid), .rready(mrready)
    );

    integer pass_cnt = 0, fail_cnt = 0;
    integer bus_starts = 0, mac_starts = 0, mac_dones = 0, bus_dones = 0;
    integer ar_count = 0, r_count = 0, fifo_writes = 0, retired = 0;
    integer aws = 0, ws = 0, bs = 0;
    reg failed = 0, watch_rejection = 0;
    reg stalled_b = 0;
    reg [1:0] held_bresp;
    reg [511:0] case_name;

    task automatic check(input condition, input [767:0] detail);
    begin
        if (condition !== 1'b1) begin
            failed = 1;
            $display("CHECK_FAIL: %0s: %0s", case_name, detail);
        end
    end
    endtask

    always @(posedge bus_clk) begin
        if (!resetn) begin
            bus_starts = 0; bus_dones = 0; ar_count = 0; r_count = 0;
            fifo_writes = 0; retired = 0; aws = 0; ws = 0; bs = 0;
            stalled_b = 0;
        end else begin
            if (stalled_b && (bvalid !== 1'b1 || bresp !== held_bresp))
                $fatal(1, "B response changed under backpressure");
            stalled_b = bvalid && !bready; held_bresp = bresp;
            if (dut.start_pulse_bus) bus_starts = bus_starts + 1;
            if (dut.done_pulse_bus) bus_dones = bus_dones + 1;
            if (marvalid && marready) ar_count = ar_count + 1;
            if (mrvalid && mrready) r_count = r_count + 1;
            if (dut.fifo_wr_en) fifo_writes = fifo_writes + 1;
            if (dut.u_dma.engine_out_acc) retired = retired + 1;
            if (awvalid && awready) aws = aws + 1;
            if (wvalid && wready) ws = ws + 1;
            if (bvalid && bready) bs = bs + 1;
            if (watch_rejection && (dut.start_pulse_bus !== 1'b0
                || marvalid !== 1'b0 || mrready !== 1'b0
                || dut.fifo_wr_en !== 1'b0 || dut.u_dma.dma_busy !== 1'b0
                || dut.u_dma.state !== 3'd0 || dut.u_dma.u_read_engine.state !== 2'd0
                || dut.u_dma.u_read_engine.occupancy !== 4'd0
                || dut.u_dma.engine_out_valid !== 1'b0 || dut.done_pulse_bus !== 1'b0))
                failed = 1;
        end
    end
    always @(posedge mac_clk) begin
        if (mac_rst) begin mac_starts = 0; mac_dones = 0; end
        else begin
            if (dut.start_pulse_mac) mac_starts = mac_starts + 1;
            if (dut.done_mac) mac_dones = mac_dones + 1;
            if (watch_rejection && (dut.start_pulse_mac !== 1'b0
                || dut.busy_mac !== 1'b0 || dut.done_mac !== 1'b0
                || dut.fifo_rd_en_mac !== 1'b0 || dut.u_pe.valid_mul !== 1'b0))
                failed = 1;
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
    end
    endtask

    task automatic drain_b;
    begin
        @(negedge bus_clk); bready = 1;
        @(posedge bus_clk);
        if (bvalid !== 1'b1) $fatal(1, "Missing response at B handshake");
        @(negedge bus_clk); bready = 0;
    end
    endtask

    task automatic write_csr(input [7:0] addr, input [31:0] data,
                             input [3:0] strb, input integer skew,
                             input integer hold_cycles, input [1:0] response);
        reg [1:0] observed;
    begin
        issue(addr, data, strb, skew);
        wait_b; observed = bresp;
        check(bresp === response, "write BRESP differs from contract");
        repeat (hold_cycles) begin
            @(negedge bus_clk);
            if (bvalid !== 1'b1 || bresp !== observed)
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

    task automatic begin_case(input [511:0] name);
        integer i;
    begin
        watch_rejection = 0;
        @(negedge bus_clk); resetn = 0; mac_rst = 1;
        awvalid = 0; wvalid = 0; bready = 0; arvalid = 0; rready = 0;
        ar_stall = 0; failed = 0; case_name = name;
        repeat (8) @(negedge bus_clk);
        for (i = 0; i < 2048; i = i + 1) mem.mem[i] = 0;
        for (i = 0; i < MAX_LEN; i = i + 1) begin
            mem.mem[i] = i + 1;
            mem.mem[1024+i] = 2;
        end
        @(negedge mac_clk); mac_rst = 0;
        @(negedge bus_clk); resetn = 1;
        repeat (5) @(negedge bus_clk);
        write_csr(8'h04, 32'h00000000, 4'hf, 0, 0, 2'b00);
        write_csr(8'h08, 32'h00001000, 4'hf, 0, 0, 2'b00);
    end
    endtask

    task automatic end_case;
    begin
        check(aws == ws && ws == bs, "AW/W/B conservation");
        if (failed) begin fail_cnt = fail_cnt + 1; $display("FAIL: %0s", case_name); end
        else begin pass_cnt = pass_cnt + 1; $display("PASS: %0s", case_name); end
    end
    endtask

    task automatic valid_job(input integer length, input integer skew, input set_length);
        integer n, a0, r0, f0, s0, ms0, md0, d0, t0;
        reg [31:0] ctrl, result, latency;
    begin
        a0 = ar_count; r0 = r_count; f0 = fifo_writes; t0 = retired;
        s0 = bus_starts; ms0 = mac_starts; md0 = mac_dones; d0 = bus_dones;
        if (set_length) write_csr(8'h0c, length, 4'hf, skew, 0, 2'b00);
        write_csr(8'h00, 32'h1, 4'hf, skew, 3, 2'b00);
        n = 0;
        // Require a fresh completion event, never accept a previous sticky done.
        while (bus_dones == d0 && n < JOB_LIMIT) begin
            @(negedge bus_clk); n = n + 1;
        end
        check(n < JOB_LIMIT, "legal job exceeded bounded completion wait");
        repeat (16) @(negedge bus_clk);
        read_csr(8'h00, ctrl); read_csr(8'h10, result); read_csr(8'h14, latency);
        check(ctrl === 32'h4, "legal job status must be done, idle, no DMA error");
        check(result === length*(length+1), "legal job dot product");
        check(latency > 0, "legal job latency");
        check(bus_starts == s0+1 && mac_starts == ms0+1, "one start per domain");
        check(bus_dones == d0+1 && mac_dones == md0+1, "one completion per domain");
        check(ar_count-a0 == 2*((length+15)/16), "legal AR burst count");
        check(r_count-r0 == 2*length && retired-t0 == 2*length, "legal R/retirement count");
        check(fifo_writes-f0 == length, "legal operand/FIFO count");
    end
    endtask

    task automatic rejected_start(input [3:0] strb, input [1:0] response,
                                   input integer skew);
        integer a0, r0, f0, s0, ms0, md0, d0, t0;
        reg [31:0] ctrl0, result0, latency0, ctrl, result, latency;
        reg start_toggle0, done_toggle0;
        reg [15:0] wi0, ri0, issue0, retire0;
        reg [2:0] alloc0, head0;
    begin
        read_csr(8'h00, ctrl0); read_csr(8'h10, result0); read_csr(8'h14, latency0);
        a0 = ar_count; r0 = r_count; f0 = fifo_writes; t0 = retired;
        s0 = bus_starts; ms0 = mac_starts; md0 = mac_dones; d0 = bus_dones;
        start_toggle0 = dut.start_toggle_bus; done_toggle0 = dut.done_toggle_mac;
        wi0 = dut.u_dma.write_idx_a; ri0 = dut.u_dma.read_idx_a;
        issue0 = dut.u_dma.u_read_engine.issue_elem; retire0 = dut.u_dma.u_read_engine.retire_elem;
        alloc0 = dut.u_dma.u_read_engine.alloc_ptr; head0 = dut.u_dma.u_read_engine.head_ptr;
        watch_rejection = 1;
        write_csr(8'h00, 32'h1, strb, skew, 12, response);
        repeat (100) @(negedge bus_clk);
        read_csr(8'h00, ctrl); read_csr(8'h10, result); read_csr(8'h14, latency);
        watch_rejection = 0;
        check(ctrl === ctrl0 && result === result0 && latency === latency0,
               "rejection preserves prior CSR status/result/latency");
        check(bus_starts == s0 && mac_starts == ms0, "rejection has no start");
        check(bus_dones == d0 && mac_dones == md0, "rejection has no fresh completion");
        check(ar_count == a0 && r_count == r0 && retired == t0, "rejection has no AR/R/ROB work");
        check(fifo_writes == f0, "rejection has no FIFO writes");
        check(dut.start_toggle_bus === start_toggle0 && dut.done_toggle_mac === done_toggle0,
               "rejection preserves event toggles");
        check(dut.fifo_empty_mac === 1'b1 && dut.u_dma.write_idx_a === wi0
              && dut.u_dma.read_idx_a === ri0 && dut.u_dma.u_read_engine.issue_elem === issue0
              && dut.u_dma.u_read_engine.retire_elem === retire0
              && dut.u_dma.u_read_engine.alloc_ptr === alloc0 && dut.u_dma.u_read_engine.head_ptr === head0,
              "rejection preserves buffer indices and ROB issue/retire ownership");
        $display("REJECTION_OBS len=%0d BRESP_expected=%b starts_delta=%0d AR_delta=%0d R_delta=%0d FIFO_delta=%0d DMA_state=%0d engine_state=%0d busy=%b done=%b",
                 dut.reg_length, response, bus_starts-s0, ar_count-a0, r_count-r0,
                 fifo_writes-f0, dut.u_dma.state, dut.u_dma.u_read_engine.state,
                 dut.busy_any_bus, dut.done_bus);
    end
    endtask

    task automatic invalid_then_valid(input [511:0] name, input integer length,
                                      input integer recovery_length);
    begin
        begin_case(name);
        write_csr(8'h0c, length, 4'hf, 0, 0, 2'b00);
        rejected_start(4'hf, 2'b10, 0);
        valid_job(recovery_length, 0, 1'b1);
        end_case;
    end
    endtask

    task automatic queued_slverr_case;
        integer s0, a0, f0, b0;
    begin
        begin_case("queued write behind SLVERR");
        write_csr(8'h0c, 0, 4'hf, 0, 0, 2'b00);
        s0 = bus_starts; a0 = ar_count; f0 = fifo_writes; b0 = bs;
        watch_rejection = 1;
        issue(8'h00, 32'h1, 4'hf, 0);
        wait_b;
        check(bresp === 2'b10, "queued case invalid start SLVERR");
        issue(8'h0c, 32'h1, 4'hf, -7);
        repeat (12) begin
            @(negedge bus_clk);
            check(bvalid === 1'b1 && bresp === 2'b10, "pending SLVERR must stay stable");
            check(dut.reg_length === 16'd0 && bs == b0, "queued LENGTH must wait for prior B");
        end
        drain_b;
        wait_b;
        check(bresp === 2'b00, "next write response returns to OKAY");
        drain_b;
        repeat (100) @(negedge bus_clk);
        check(dut.reg_length === 16'd1 && bvalid === 1'b0, "queued LENGTH committed exactly once");
        check(bus_starts == s0 && ar_count == a0 && fifo_writes == f0
              && mac_starts == 0 && mac_dones == 0 && bus_dones == 0,
              "queued rejected start must have no execution");
        watch_rejection = 0;
        valid_job(1, 0, 1'b0);
        end_case;
    end
    endtask

    reg [31:0] observed_length;
    initial begin
        invalid_then_valid("LENGTH=0 then recovery=1", 0, 1);
        invalid_then_valid("LENGTH=MAX_LEN+1 recovery=17", MAX_LEN+1, 17);
        invalid_then_valid("LENGTH=65535 recovery=16", 65535, 16);
        begin_case("LENGTH=1 full-word AW-first"); valid_job(1, 7, 1'b1); end_case;
        begin_case("LENGTH=16 full-word W-first"); valid_job(16, -7, 1'b1); end_case;
        begin_case("LENGTH=17"); valid_job(17, 0, 1'b1); end_case;
        begin_case("LENGTH=MAX_LEN"); valid_job(MAX_LEN, 0, 1'b1); end_case;
        begin_case("invalid-valid no intervening reset");
        write_csr(8'h0c, 0, 4'hf, 0, 0, 2'b00);
        rejected_start(4'hf, 2'b10, -7);
        valid_job(MAX_LEN, 0, 1'b1); end_case;
        begin_case("valid-invalid-valid sticky done");
        valid_job(17, 0, 1'b1);
        write_csr(8'h0c, 0, 4'hf, 0, 0, 2'b00);
        rejected_start(4'hf, 2'b10, 7);
        valid_job(1, 0, 1'b1); end_case;
        begin_case("partial LENGTH invalid then repair");
        write_csr(8'h0c, 16, 4'hf, 0, 0, 2'b00);
        write_csr(8'h0c, 32'hdead01aa, 4'h2, -7, 0, 2'b00);
        read_csr(8'h0c, observed_length);
        check(observed_length === 32'h00000110, "partial LENGTH merged to 272");
        rejected_start(4'h1, 2'b10, 0);
        write_csr(8'h0c, 32'hbeef00ff, 4'h2, 7, 0, 2'b00);
        read_csr(8'h0c, observed_length);
        check(observed_length === 32'h00000010, "partial LENGTH repaired to 16");
        // Start directly after byte repair; do not overwrite it with a full word.
        valid_job(16, 0, 1'b0);
        end_case;
        begin_case("invalid LENGTH zero CTRL strobe");
        write_csr(8'h0c, 0, 4'hf, 0, 0, 2'b00);
        rejected_start(4'h0, 2'b00, 0); valid_job(1, 0, 1'b1); end_case;
        begin_case("invalid LENGTH CTRL low disabled");
        write_csr(8'h0c, 0, 4'hf, 0, 0, 2'b00);
        rejected_start(4'ha, 2'b00, 0); valid_job(1, 0, 1'b1); end_case;
        queued_slverr_case;
        $display("TOTAL: %0d PASS / %0d FAIL", pass_cnt, fail_cnt);
        if (fail_cnt != 0) $fatal(1, "Invalid LENGTH regression failed");
        $finish;
    end
    initial begin
        #20000000;
        $fatal(1, "Global transaction timeout");
    end
endmodule
