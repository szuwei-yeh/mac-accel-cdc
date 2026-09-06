`timescale 1ns/1ps

// Deterministic activity/verification workload for the targeted buf_a clock-
// gating experiment.  The measured active and idle windows are printed in ps
// so the same RTL VCD can be sliced into identical SAIF files for both mapped
// implementations.

`ifndef POWER_MAX_OUT
`define POWER_MAX_OUT 8
`endif

module tb_mac_dma_rob_power;
    localparam DATA_WIDTH      = 16;
    localparam AXI_DATA_WIDTH  = 32;
    localparam AXI_ADDR_WIDTH  = 32;
    localparam AXI_ID_WIDTH    = 4;
    localparam MAX_LEN         = 256;
    localparam BURST_LEN       = 16;
    localparam MAX_OUT         = `POWER_MAX_OUT;
    localparam MEM_WORDS       = 8192;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst, start;
    reg [AXI_ADDR_WIDTH-1:0] src_a_addr, src_b_addr;
    reg [15:0] length;
    wire dma_busy, dma_done, dma_err;

    wire [AXI_ID_WIDTH-1:0] M_AXI_ARID;
    wire [AXI_ADDR_WIDTH-1:0] M_AXI_ARADDR;
    wire [7:0] M_AXI_ARLEN;
    wire [2:0] M_AXI_ARSIZE;
    wire [1:0] M_AXI_ARBURST;
    wire M_AXI_ARVALID, M_AXI_ARREADY;
    wire [AXI_ID_WIDTH-1:0] M_AXI_RID;
    wire [AXI_DATA_WIDTH-1:0] M_AXI_RDATA;
    wire [1:0] M_AXI_RRESP;
    wire M_AXI_RLAST, M_AXI_RVALID, M_AXI_RREADY;

    wire stream_valid, stream_last;
    wire signed [DATA_WIDTH-1:0] stream_a, stream_b;
    reg stream_ready, ar_stall;
    reg pattern_enable;
    integer pattern_cycle;

`ifdef POWER_GATE_LEVEL
    `POWER_DUT_MODULE dut (
`else
    mac_dma_rob #(
        .DATA_WIDTH(DATA_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .MAX_LEN(MAX_LEN),
        .BURST_LEN(BURST_LEN),
        .MAX_OUTSTANDING(MAX_OUT)
    ) dut (
`endif
        .clk(clk), .rst(rst), .start(start),
        .src_a_addr(src_a_addr), .src_b_addr(src_b_addr), .length(length),
        .dma_busy(dma_busy), .dma_done(dma_done), .dma_err(dma_err),
        .M_AXI_ARID(M_AXI_ARID), .M_AXI_ARADDR(M_AXI_ARADDR),
        .M_AXI_ARLEN(M_AXI_ARLEN), .M_AXI_ARSIZE(M_AXI_ARSIZE),
        .M_AXI_ARBURST(M_AXI_ARBURST), .M_AXI_ARVALID(M_AXI_ARVALID),
        .M_AXI_ARREADY(M_AXI_ARREADY), .M_AXI_RID(M_AXI_RID),
        .M_AXI_RDATA(M_AXI_RDATA), .M_AXI_RRESP(M_AXI_RRESP),
        .M_AXI_RLAST(M_AXI_RLAST), .M_AXI_RVALID(M_AXI_RVALID),
        .M_AXI_RREADY(M_AXI_RREADY), .stream_valid(stream_valid),
        .stream_last(stream_last), .stream_a(stream_a), .stream_b(stream_b),
        .stream_ready(stream_ready)
    );

    axi_read_mem_model_ooo #(
        .AXI_ID_WIDTH(AXI_ID_WIDTH), .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH), .MEM_WORDS(MEM_WORDS),
        .MAX_OUTSTANDING(MAX_OUT)
    ) u_mem (
        .clk(clk), .resetn(~rst), .ar_latency(16'd40),
        .ar_stall(ar_stall), .ooo_en(1'b1),
        .arid(M_AXI_ARID), .araddr(M_AXI_ARADDR), .arlen(M_AXI_ARLEN),
        .arsize(M_AXI_ARSIZE), .arburst(M_AXI_ARBURST),
        .arvalid(M_AXI_ARVALID), .arready(M_AXI_ARREADY),
        .rid(M_AXI_RID), .rdata(M_AXI_RDATA), .rresp(M_AXI_RRESP),
        .rlast(M_AXI_RLAST), .rvalid(M_AXI_RVALID), .rready(M_AXI_RREADY)
    );

    // Fixed-cycle pattern: deterministic AXI and stream backpressure with no
    // simulator-dependent random state.
    always @(posedge clk) begin
        if (rst) begin
            pattern_cycle <= 0;
            stream_ready  <= 1'b1;
            ar_stall      <= 1'b0;
        end else begin
            pattern_cycle <= pattern_cycle + 1;
            if (pattern_enable) begin
                stream_ready <= ((pattern_cycle % 7) != 0) &&
                                ((pattern_cycle % 11) != 0);
                ar_stall     <= ((pattern_cycle % 13) == 0) ||
                                ((pattern_cycle % 17) == 0);
            end else begin
                stream_ready <= 1'b1;
                ar_stall     <= 1'b0;
            end
        end
    end

    reg signed [15:0] golden_a [0:MAX_LEN-1];
    reg signed [15:0] golden_b [0:MAX_LEN-1];
    integer current_len, pair_idx, job_errors;
    integer rid_switches, accepted_r_beats;
    reg [AXI_ID_WIDTH-1:0] previous_rid;
    reg previous_rid_valid;

    task automatic mem_write(input [31:0] byte_addr, input signed [15:0] value);
    begin
        u_mem.mem[(byte_addr >> 2) & (MEM_WORDS-1)] = {{16{value[15]}}, value};
    end
    endtask

    task automatic preload(input [31:0] a_base, input [31:0] b_base,
                           input integer len, input integer salt);
        integer i;
        reg signed [15:0] av, bv;
    begin
        for (i = 0; i < len; i = i + 1) begin
            av = ((i * 13 + salt * 7) % 127) - 63;
            bv = ((i * 9  + salt * 5) % 61)  - 30;
            golden_a[i] = av;
            golden_b[i] = bv;
            mem_write(a_base + i*4, av);
            mem_write(b_base + i*4, bv);
        end
    end
    endtask

    always @(posedge clk) begin
        if (rst) begin
            pair_idx           <= 0;
            job_errors         <= 0;
            rid_switches       <= 0;
            accepted_r_beats   <= 0;
            previous_rid       <= 0;
            previous_rid_valid <= 0;
        end else begin
            if (M_AXI_RVALID && M_AXI_RREADY) begin
                accepted_r_beats <= accepted_r_beats + 1;
                if (previous_rid_valid && M_AXI_RID != previous_rid)
                    rid_switches <= rid_switches + 1;
                previous_rid       <= M_AXI_RID;
                previous_rid_valid <= 1'b1;
            end
            if (stream_valid && stream_ready) begin
                if (pair_idx >= current_len ||
                    stream_a !== golden_a[pair_idx] ||
                    stream_b !== golden_b[pair_idx]) begin
                    job_errors <= job_errors + 1;
                    $display("POWER_PAIR_ERROR idx=%0d got=(%0d,%0d)",
                             pair_idx, stream_a, stream_b);
                end
                if (stream_last !== (pair_idx == current_len-1)) begin
                    job_errors <= job_errors + 1;
                    $display("POWER_LAST_ERROR idx=%0d last=%0b", pair_idx, stream_last);
                end
                pair_idx <= pair_idx + 1;
            end
        end
    end

    integer total_pass = 0;
    integer total_fail = 0;

    task automatic run_job(input integer len, input [31:0] a_base,
                           input [31:0] b_base, input integer salt,
                           input integer enable_bp, input [127:0] job_name);
        integer timeout;
        integer start_errors, start_rid_switches, start_r_beats;
    begin
        while (dma_busy) @(posedge clk);
        repeat (3) @(posedge clk);
        preload(a_base, b_base, len, salt);
        current_len = len;
        pair_idx = 0;
        start_errors = job_errors;
        start_rid_switches = rid_switches;
        start_r_beats = accepted_r_beats;
        pattern_enable = enable_bp;
        @(negedge clk);
        src_a_addr = a_base;
        src_b_addr = b_base;
        length = len[15:0];
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        timeout = 0;
        while (!dma_done && timeout < 100000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        @(negedge clk);
        pattern_enable = 1'b0;
        if (timeout >= 100000 || dma_err || pair_idx != len ||
            accepted_r_beats-start_r_beats != 2*len ||
            job_errors != start_errors) begin
            total_fail = total_fail + 1;
            $display("POWER_JOB_FAIL name=%s len=%0d timeout=%0d pairs=%0d beats=%0d errors=%0d dma_err=%0b",
                     job_name, len, timeout, pair_idx,
                     accepted_r_beats-start_r_beats,
                     job_errors-start_errors, dma_err);
        end else begin
            total_pass = total_pass + 1;
            $display("POWER_JOB_PASS name=%s len=%0d cycles=%0d rid_switches=%0d",
                     job_name, len, timeout, rid_switches-start_rid_switches);
        end
    end
    endtask

`ifdef POWER_ACTIVITY_DUMP
    initial begin
        // VCS VCD dumping omits Verilog unpacked memories.  VPD memory tracing
        // followed by the vendor vpd2vcd converter retains their RTL names,
        // which is required for buf_a/ROB backward-SAIF annotation.
        $vcdplusfile("mac_dma_rob_power.vpd");
        $vcdpluson(0, dut);
        $vcdplusmemon(dut.buf_a);
        $vcdplusmemon(dut.u_read_engine.rob_data);
    end
`endif

    integer idle_i;
    time active_start_ps, active_end_ps, idle_start_ps, idle_end_ps;
`ifdef POWER_GATE_DEBUG
    always @(posedge clk) begin
        if ($time < 1000)
            $display("POWER_GATE_DEBUG t=%0t rst=%b start=%b busy=%b done=%b state=%b eng_state=%b issue=%h len=%h occ=%h cmd_vr=%b%b ar_vr=%b%b r_vr=%b%b stream_vr=%b%b",
                     $time, rst, start, dma_busy, dma_done,
                     dut.state, dut.u_read_engine.state,
                     dut.u_read_engine.issue_elem, dut.u_read_engine.cmd_len_r,
                     dut.u_read_engine.occupancy,
                     dut.engine_cmd_valid, dut.engine_cmd_ready,
                     M_AXI_ARVALID, M_AXI_ARREADY, M_AXI_RVALID, M_AXI_RREADY,
                     stream_valid, stream_ready);
    end
`endif
    initial begin
        rst = 1'b1;
        start = 1'b0;
        src_a_addr = 0;
        src_b_addr = 0;
        length = 0;
        pattern_enable = 1'b0;
        repeat (10) @(posedge clk);
        @(negedge clk); rst = 1'b0;
        repeat (5) @(posedge clk);

        // The len=64 job is the sole active power measurement window.  It
        // spans four bursts per operand and produces interleaved RIDs.
        active_start_ps = $time;
        $display("POWER_ACTIVE_START_PS=%0t", active_start_ps);
        run_job(64, 32'h0000_0000, 32'h0000_1000, 11, 1,
                "measured_len64");
        active_end_ps = $time;
        $display("POWER_ACTIVE_END_PS=%0t", active_end_ps);

        // Stable, job-free interval for idle average power.
        idle_start_ps = $time;
        $display("POWER_IDLE_START_PS=%0t", idle_start_ps);
        for (idle_i = 0; idle_i < 512; idle_i = idle_i + 1) @(posedge clk);
        @(negedge clk);
        idle_end_ps = $time;
        $display("POWER_IDLE_END_PS=%0t", idle_end_ps);

        // Consecutive no-reset jobs cover wake-up, one beat, exact burst, and
        // a multi-burst crossing while deterministic stalls remain active.
        run_job(1,  32'h0000_2000, 32'h0000_2400, 23, 1, "wake_len1");
        run_job(16, 32'h0000_2800, 32'h0000_2C00, 31, 1, "len16");
        run_job(33, 32'h0000_3000, 32'h0000_3400, 47, 1, "cross_burst_len33");

        $display("POWER_WINDOWS active_ps=%0d idle_ps=%0d",
                 active_end_ps-active_start_ps, idle_end_ps-idle_start_ps);
        $display("POWER_TOTAL %0d PASS / %0d FAIL rid_switches=%0d",
                 total_pass, total_fail, rid_switches);
        if (total_fail != 0 || total_pass != 4 || rid_switches == 0) begin
            $display("POWER_REGRESSION_FAIL");
            $fatal(1);
        end
        $display("POWER_REGRESSION_PASS");
        #50 $finish;
    end

    initial begin
        #5000000;
        $display("POWER_GLOBAL_TIMEOUT");
        $fatal(1);
    end
endmodule
