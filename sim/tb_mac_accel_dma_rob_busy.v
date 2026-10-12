`timescale 1ns/1ps

// V3 one-job admission: deterministic lifecycle targeting and signed scoreboards.
// Test-only hierarchy observes protocol events; stimulus uses AXI-Lite.
module tb_mac_accel_dma_rob_busy;
    localparam LIMIT = 80;
    localparam MAX_LEN = 256;
    localparam JOB_LIMIT = 10000;
    reg bus_clk = 0, mac_clk = 0;
    always #5 bus_clk = ~bus_clk;
    real base_mac_period = 17.0, mac_period = 17.0, mac_phase = 1.25;
    integer arg_status;
    initial begin
        arg_status = $value$plusargs("MAC_PERIOD=%f", base_mac_period);
        arg_status = $value$plusargs("MAC_PHASE=%f", mac_phase);
        mac_period = base_mac_period;
        #mac_phase;
        forever #(mac_period/2.0) mac_clk = ~mac_clk;
    end
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
    reg ar_stall = 0, inject_error = 0;
    reg [15:0] mem_latency = 80;
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
        .M_AXI_RRESP(inject_error ? 2'b10 : mrresp), .M_AXI_RLAST(mlast), .M_AXI_RVALID(mrvalid),
        .M_AXI_RREADY(mrready),
        .mac_clk(mac_clk), .mac_rst(mac_rst)
    );

    axi_read_mem_model_ooo #(.MEM_WORDS(2048), .MAX_OUTSTANDING(8)) mem (
        .clk(bus_clk), .resetn(resetn), .ar_latency(mem_latency),
        .ar_stall(ar_stall), .ooo_en(1'b1),
        .arid(mid), .araddr(maddr), .arlen(mlen), .arsize(msize),
        .arburst(mburst), .arvalid(marvalid), .arready(marready),
        .rid(rid), .rdata(mdata), .rresp(mrresp), .rlast(mlast),
        .rvalid(mrvalid), .rready(mrready)
    );

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

    integer pass_cnt = 0, fail_cnt = 0;
    integer accepted = 0, rejected = 0, noops = 0, planned = 0;
    integer bus_starts = 0, mac_starts = 0, mac_dones = 0, bus_dones = 0;
    integer toggles = 0, ar_count = 0, r_count = 0, fifo_writes = 0, fifo_reads = 0;
    integer aws = 0, ws = 0, bs = 0, ticks = 0, capture_tick = -1;
    integer ar_index = 0, write_index = 0, read_index = 0, accept_tick = -100;
    integer lengths [0:7], patterns [0:7], sums [0:7];
    reg [31:0] a_bases [0:7], b_bases [0:7];
    reg errors [0:7];
    reg failed = 0, reference_active = 0, pending_busy_probe = 0;
    reg stalled_b = 0, previous_toggle = 0;
    reg [1:0] held_bresp, expected_response = 0;
    integer target_mode = 0;
    reg [511:0] case_name;

    function automatic signed [15:0] operand(input integer pattern,
                                              input integer index, input is_b);
    begin
        if (pattern == 0) operand = is_b ? (index % 5)+1 : (index % 7)-3;
        else operand = is_b ? 3 : -2*(index+1);
    end
    endfunction

    task automatic check(input condition, input [767:0] detail);
    begin
        if (condition !== 1'b1) begin
            failed = 1;
            $display("CHECK_FAIL: %0s: %0s", case_name, detail);
        end
    end
    endtask

    function automatic phase_hit(input integer mode);
    begin
        case (mode)
            2: phase_hit = bus_starts == 1 && bus_dones == 0 && ticks == accept_tick+2;
            3: phase_hit = dut.u_dma.state == 3'd2;
            4: phase_hit = dut.u_dma.state == 3'd4;
            5: phase_hit = !dut.dma_busy && dut.busy_mac;
            6: phase_hit = !dut.dma_busy && dut.busy_mac && !dut.fifo_empty_mac;
            7: phase_hit = !dut.busy_bus && mac_starts == 0 && bus_dones == 0;
            8: phase_hit = !dut.busy_mac && !dut.done_pulse_bus
                          && dut.done_toggle_mac != dut.done_sync_bus_d;
            9: phase_hit = dut.done_pulse_bus;
            10: phase_hit = bus_dones == 1 && ticks == capture_tick+1;
            default: phase_hit = 1;
        endcase
    end
    endfunction

    integer monitor_job, burst, element;
    reg commit_ctrl, consume_done, consume_start, legitimate_start;
    reg [31:0] previous_result, previous_latency;
    reg previous_done, previous_error;
    always @(posedge bus_clk) begin
        if (!resetn) begin
            accepted = 0; rejected = 0; noops = 0;
            bus_starts = 0; bus_dones = 0; toggles = 0;
            ar_count = 0; r_count = 0; fifo_writes = 0;
            aws = 0; ws = 0; bs = 0; ticks = 0; capture_tick = -1;
            ar_index = 0; write_index = 0; read_index = 0; accept_tick = -100;
            reference_active = 0; stalled_b = 0; previous_toggle = 0;
        end else begin
            ticks = ticks + 1;
            if (stalled_b && (bvalid !== 1'b1 || bresp !== held_bresp))
                $fatal(1, "B response changed under backpressure");
            stalled_b = bvalid && !bready; held_bresp = bresp;
            if (awvalid && awready) aws = aws + 1;
            if (wvalid && wready) ws = ws + 1;
            if (bvalid && bready) bs = bs + 1;
            consume_start = dut.start_pulse_bus;
            legitimate_start = ticks == accept_tick+1;
            check(consume_start === legitimate_start,"start pulse must match accepted commit exactly");
            consume_done = dut.done_pulse_bus;
            previous_result = dut.res_bus_reg; previous_latency = dut.lat_bus_reg;
            previous_done = dut.done_bus; previous_error = dut.dma_err_bus;
            if (consume_start) begin
                bus_starts = bus_starts + 1;
                check(bus_starts <= accepted, "extra bus start from rejected command");
            end
            if (marvalid && marready) begin
                monitor_job = accepted-1;
                check(monitor_job >= 0 && monitor_job < planned, "AR without accepted software job");
                if (monitor_job >= 0 && monitor_job < planned) begin
                    burst = (lengths[monitor_job]+15)/16;
                    element = (ar_index % burst)*16;
                    check(ar_index < 2*burst, "extra AR attributable to rejected start");
                    check(maddr === ((ar_index < burst ? a_bases[monitor_job] : b_bases[monitor_job])+element*4),
                          "AR address sequence differs from accepted job");
                    check(mlen === (lengths[monitor_job]-element > 16 ? 8'd15 : lengths[monitor_job]-element-1),
                          "AR burst geometry differs from accepted job");
                    check(msize === 3'd2 && mburst === 2'b01, "AR size/type");
                end
                ar_index = ar_index + 1; ar_count = ar_count + 1;
            end
            if (mrvalid && mrready) r_count = r_count + 1;
            if (dut.fifo_wr_en) begin
                monitor_job = accepted-1;
                check(monitor_job >= 0 && monitor_job < planned, "FIFO write without accepted job");
                if (monitor_job >= 0 && monitor_job < planned) begin
                    check(write_index < lengths[monitor_job], "extra operand pair");
                    check(dut.stream_a === operand(patterns[monitor_job],write_index,0)
                          && dut.stream_b === operand(patterns[monitor_job],write_index,1), "active-job operands corrupted");
                    check(dut.stream_last === (write_index == lengths[monitor_job]-1), "operand last position");
                end
                write_index = write_index + 1; fifo_writes = fifo_writes + 1;
            end
            if (consume_done) begin
                check(reference_active, "completion without an expected active job");
                bus_dones = bus_dones + 1; capture_tick = ticks; reference_active = 0;
                check(bus_dones <= accepted && mac_dones == bus_dones, "unexpected bus completion");
            end
            commit_ctrl = dut.aw_latched && dut.w_latched && !bvalid && dut.aw_word_addr == 0;
            if (commit_ctrl) begin
                check(phase_hit(target_mode), "CTRL commit missed intended lifecycle boundary");
                if (target_mode == 8 || target_mode == 9)
                    check(dut.busy_any_bus === 1'b1,"BUSY released before safe completion capture");
                $display("COMMIT_OBS mode=%0d tick=%0d done=%b DMA=%0d busy_mac=%b busy_bus=%b busy_any=%b expected_BRESP=%b",
                         target_mode,ticks,dut.done_pulse_bus,dut.u_dma.state,
                         dut.busy_mac,dut.busy_bus,dut.busy_any_bus,expected_response);
                if (dut.w_strb_r[0] && dut.w_data_r[0]) begin
                    if (expected_response == 0) begin
                        check(!reference_active, "test attempted overlapping accepted jobs");
                        accepted = accepted + 1; reference_active = 1; accept_tick = ticks;
                        check(accepted <= planned, "accepted job has no independent plan");
                        ar_index = 0; write_index = 0; read_index = 0;
                    end else rejected = rejected + 1;
                end else noops = noops + 1;
            end
            #1;
            if (dut.start_toggle_bus !== previous_toggle) begin
                toggles = toggles + 1; previous_toggle = dut.start_toggle_bus;
                check(toggles <= accepted, "extra start-toggle transition");
            end
            if (commit_ctrl) begin
                check(bvalid === 1'b1 && bresp === expected_response, "commit BRESP");
                if (expected_response == 2'b10)
                    check(dut.start_pulse_bus === 1'b0, "rejected commit emitted start pulse");
                else if (dut.w_strb_r[0] && dut.w_data_r[0]) begin
                    check(dut.start_pulse_bus === 1'b1, "accepted commit missing start pulse");
                    if (pending_busy_probe)
                        check(dut.busy_any_bus === 1'b1, "BUSY missing in pending-start interval");
                end
            end
            if (consume_done && bus_dones <= planned) begin
                check(dut.res_bus_reg === sums[bus_dones-1] && dut.lat_bus_reg > 0,
                      "completed result/latency not safely captured");
                check(dut.done_bus === 1'b1, "completion lost sticky done");
            end else begin
                check(dut.res_bus_reg === previous_result && dut.lat_bus_reg === previous_latency,
                      "result/latency replaced without completion");
                if (!legitimate_start)
                    check(dut.done_bus === previous_done, "sticky done changed without start/completion");
            end
            if (previous_error && !legitimate_start)
                check(dut.dma_err_bus === 1'b1, "sticky DMA error lost");
        end
    end

    integer mac_job;
    always @(posedge mac_clk or posedge mac_rst) begin
        if (mac_rst) begin mac_starts = 0; mac_dones = 0; fifo_reads = 0; end
        else begin
            if (dut.start_pulse_mac) begin
                mac_starts = mac_starts + 1;
                check(mac_starts <= accepted, "extra MAC start event");
            end
            if (dut.fifo_rd_en_mac && !dut.fifo_empty_mac) begin
                mac_job = accepted-1;
                if (mac_job >= 0 && mac_job < planned) begin
                    check(dut.fifo_dout_mac === {(read_index == lengths[mac_job]-1),
                          operand(patterns[mac_job],read_index,0), operand(patterns[mac_job],read_index,1)},
                          "FIFO drain payload/order/last corrupted");
                end else check(0,"FIFO read without accepted job");
                read_index = read_index + 1; fifo_reads = fifo_reads + 1;
            end
            if (dut.done_mac) begin
                mac_dones = mac_dones + 1;
                check(mac_dones <= accepted, "unexpected MAC completion");
                if (mac_dones <= planned)
                    check(dut.result_mac === sums[mac_dones-1], "signed MAC result");
            end
        end
    end

    task automatic wait_b;
        integer n;
    begin
        n = 0;
        while (bvalid !== 1'b1 && n < LIMIT) begin @(negedge bus_clk); n = n+1; end
        if (bvalid !== 1'b1) $fatal(1,"B transaction timeout");
        check(bresp === expected_response,"write response differs from admission contract");
    end
    endtask
    task automatic drain_b;
    begin
        @(negedge bus_clk); bready = 1;
        @(posedge bus_clk);
        if (bvalid !== 1'b1) $fatal(1,"missing B handshake");
        @(negedge bus_clk); bready = 0;
    end
    endtask
    task automatic finish_write(input integer hold_cycles);
        reg [1:0] response;
    begin
        wait_b; response = bresp;
        repeat (hold_cycles) begin
            @(negedge bus_clk);
            if (bvalid !== 1'b1 || bresp !== response) $fatal(1,"stalled response changed");
        end
        drain_b;
    end
    endtask
    task automatic write_csr(input [7:0] addr,input [31:0] data,input [3:0] strb,
                             input [1:0] response,input integer mode,input integer hold_cycles);
    begin
        expected_response = response; target_mode = mode;
        issue(addr,data,strb,0); finish_write(hold_cycles);
    end
    endtask
    // Called at an observed source event or a known bus negedge. Inputs remain
    // stable through acceptance; no DUT signal is forced or deposited.
    task automatic ctrl_now(input [1:0] response,input integer mode);
        integer n;
        reg aw_done,w_done;
    begin
        expected_response = response; target_mode = mode;
        awaddr = 0; wdata = 1; wstrb = 4'hf; awvalid = 1; wvalid = 1;
        aw_done = 0; w_done = 0; n = 0;
        while (!(aw_done && w_done) && n < LIMIT) begin
            @(posedge bus_clk);
            if (awvalid && awready) aw_done = 1;
            if (wvalid && wready) w_done = 1;
            n = n+1;
            @(negedge bus_clk);
            if (aw_done) awvalid = 0;
            if (w_done) wvalid = 0;
        end
        if (!(aw_done && w_done)) $fatal(1,"boundary AW/W transaction timeout");
        awaddr = 8'hfc; wdata = 32'hfffffffe; wstrb = 0; bready = 0;
        finish_write(0);
    end
    endtask

    task automatic begin_case(input [511:0] name,input real period);
        integer i;
    begin
        @(negedge bus_clk); resetn = 0; mac_rst = 1;
        awvalid = 0; wvalid = 0; bready = 0; arvalid = 0; rready = 0;
        inject_error = 0; ar_stall = 0; mem_latency = 80;
        planned = 0; failed = 0; case_name = name; pending_busy_probe = 0;
        mac_period = period > 0 ? period : base_mac_period;
        repeat (8) @(negedge bus_clk);
        for (i=0;i<2048;i=i+1) mem.mem[i]=0;
        @(negedge mac_clk); mac_rst = 0;
        @(negedge bus_clk); resetn = 1;
        repeat (5) @(negedge bus_clk);
    end
    endtask
    task automatic plan_job(input integer length,input integer pattern,input error);
        integer k,slot,total;
        reg signed [15:0] a,b;
    begin
        slot = planned;
        lengths[slot] = length; patterns[slot] = pattern; errors[slot] = error;
        a_bases[slot] = slot*32'h400; b_bases[slot] = 32'h1000+slot*32'h400;
        total = 0;
        for (k=0;k<length;k=k+1) begin
            a = operand(pattern,k,0); b = operand(pattern,k,1);
            mem.mem[(a_bases[slot]>>2)+k] = {{16{a[15]}},a};
            mem.mem[(b_bases[slot]>>2)+k] = {{16{b[15]}},b};
            total = total+a*b;
        end
        sums[slot] = total; planned = planned+1;
        write_csr(8'h04,a_bases[slot],4'hf,0,0,0);
        write_csr(8'h08,b_bases[slot],4'hf,0,0,0);
        write_csr(8'h0c,length,4'hf,0,0,0);
    end
    endtask
    task automatic start_job;
    begin write_csr(8'h00,1,4'hf,0,0,0); end
    endtask
    task automatic wait_phase(input integer mode);
        integer n;
    begin
        n = 0;
        @(negedge bus_clk);
        while (!phase_hit(mode) && n<JOB_LIMIT) begin @(negedge bus_clk); n=n+1; end
        check(n<JOB_LIMIT,"bounded wait could not target lifecycle phase");
    end
    endtask
    task automatic finish_jobs(input integer expected_jobs);
        integer n,k,total_len,total_ar;
        reg [31:0] ctrl,result,latency;
    begin
        n=0;
        while (bus_dones<expected_jobs && n<JOB_LIMIT) begin @(negedge bus_clk); n=n+1; end
        check(n<JOB_LIMIT,"accepted job exceeded bounded completion wait");
        repeat(24) @(negedge bus_clk);
        read_csr(0,ctrl);read_csr(8'h10,result);read_csr(8'h14,latency);
        check(ctrl === (errors[expected_jobs-1] ? 32'hc : 32'h4),"final CSR done/error/busy");
        check(result === sums[expected_jobs-1] && latency>0,"final signed result/latency");
        check(accepted==expected_jobs && bus_starts==expected_jobs && toggles==expected_jobs
              && mac_starts==expected_jobs && mac_dones==expected_jobs && bus_dones==expected_jobs,
              "accepted/start/toggle/done event conservation");
        total_len=0;total_ar=0;
        for(k=0;k<expected_jobs;k=k+1) begin
            total_len=total_len+lengths[k];total_ar=total_ar+2*((lengths[k]+15)/16);
        end
        check(ar_count==total_ar && r_count==2*total_len,"no extra AR/R work from rejection");
        check(fifo_writes==total_len && fifo_reads==total_len,"operand conservation");
        check(aws==ws && ws==bs,"AW/W/B response conservation");
    end
    endtask
    task automatic end_case(input integer expected_rejects,input integer expected_noops);
    begin
        check(rejected==expected_rejects && noops==expected_noops,"reject/no-op independent counts");
        $display("CASE_COUNTS accepted=%0d rejected=%0d noop=%0d starts=%0d toggles=%0d MAC_start=%0d MAC_done=%0d BUS_done=%0d AR=%0d R=%0d FIFO=%0d/%0d",
                 accepted,rejected,noops,bus_starts,toggles,mac_starts,mac_dones,bus_dones,
                 ar_count,r_count,fifo_writes,fifo_reads);
        if(failed)begin fail_cnt=fail_cnt+1;$display("FAIL: %0s",case_name);end
        else begin pass_cnt=pass_cnt+1;$display("PASS: %0s",case_name);end
    end
    endtask

    integer j;
    initial begin
        $display("BUSY_CONFIG base_MAC_PERIOD=%0.3f MAC_PHASE=%0.3f",base_mac_period,mac_phase);
        begin_case("01 start after reset and pending BUSY",0);
        pending_busy_probe=1;plan_job(1,1,0);start_job;finish_jobs(1);end_case(0,0);

        begin_case("02 earliest queued second start",200);
        plan_job(16,1,0);
        expected_response=0;target_mode=0;issue(0,1,4'hf,0);wait_b;
        // Present the next AW/W immediately after the prior handshake. Drain
        // the first B on the next edge; the second commit is one edge later.
        bready=1;ctrl_now(2'b10,2);finish_jobs(1);end_case(1,0);

        begin_case("03 start during A DMA",0);
        plan_job(64,0,0);start_job;wait_phase(3);ctrl_now(2'b10,3);
        finish_jobs(1);end_case(1,0);

        begin_case("04 start during B DMA",100);
        plan_job(64,0,0);start_job;wait_phase(4);ctrl_now(2'b10,4);
        finish_jobs(1);end_case(1,0);

        begin_case("05 DMA finished MAC active",100);
        plan_job(16,1,0);start_job;wait_phase(5);ctrl_now(2'b10,5);
        finish_jobs(1);end_case(1,0);

        begin_case("06 start during FIFO draining",100);
        plan_job(16,0,0);start_job;wait_phase(6);ctrl_now(2'b10,6);
        finish_jobs(1);end_case(1,0);

        begin_case("07 start before synchronized MAC busy",200);
        plan_job(16,1,0);start_job;wait_phase(7);ctrl_now(2'b10,7);
        finish_jobs(1);end_case(1,0);

        begin_case("08 done toggle synchronization window",0);
        plan_job(17,0,0);start_job;
        // Bound the source-event wait and drive before the first bus sample.
        fork : wait_toggle
            begin @(dut.done_toggle_mac); #0.001; ctrl_now(2'b10,8); end
            begin #(JOB_LIMIT*10); $fatal(1,"done-toggle boundary timeout"); end
        join_any
        disable wait_toggle;
        finish_jobs(1);end_case(1,0);

        begin_case("09 CTRL commits on completion edge",0);
        plan_job(17,1,0);start_job;
        j=0;@(negedge bus_clk);
        while(dut.done_sync_bus[0]===dut.done_sync_bus[1] && j<JOB_LIMIT)begin @(negedge bus_clk);j=j+1;end
        check(j<JOB_LIMIT,"completion-boundary preparation");
        ctrl_now(2'b10,9);finish_jobs(1);end_case(1,0);

        begin_case("10 first edge after safe capture",0);
        plan_job(17,0,0);start_job;plan_job(16,1,0);
        j=0;@(negedge bus_clk);
        while(dut.done_pulse_bus!==1'b1 && j<JOB_LIMIT)begin @(negedge bus_clk);j=j+1;end
        check(j<JOB_LIMIT,"post-completion preparation");
        ctrl_now(0,10);finish_jobs(2);end_case(0,0);

        begin_case("11 repeated SLVERR stalls sticky DMA status",100);
        plan_job(64,1,1);inject_error=1;start_job;
        j=0;@(negedge bus_clk);
        while(dut.dma_err_bus!==1'b1 && j<JOB_LIMIT)begin @(negedge bus_clk);j=j+1;end
        check(j<JOB_LIMIT,"DMA error fixture reached sticky state");
        for(j=0;j<3;j=j+1)write_csr(0,1,4'hf,2'b10,0,12);
        finish_jobs(1);end_case(3,0);

        begin_case("12 rejected busy then new valid job",0);
        plan_job(33,0,0);start_job;wait_phase(3);ctrl_now(2'b10,3);finish_jobs(1);
        plan_job(17,1,0);start_job;finish_jobs(2);end_case(1,0);

        begin_case("13 invalid LENGTH while active",0);
        plan_job(64,0,0);start_job;write_csr(8'h0c,0,4'hf,0,0,0);
        write_csr(0,1,4'hf,2'b10,0,12);finish_jobs(1);end_case(1,0);

        begin_case("14 CTRL strobes and noops while busy",100);
        plan_job(64,0,0);start_job;
        write_csr(0,1,4'h0,0,0,4);write_csr(0,1,4'ha,0,0,4);
        write_csr(0,32'hfffffffe,4'h1,0,0,4);
        write_csr(0,1,4'h1,2'b10,0,4);finish_jobs(1);end_case(1,3);

        begin_case("15 three valid jobs without reset",0);
        plan_job(1,1,0);start_job;finish_jobs(1);
        plan_job(16,0,0);start_job;finish_jobs(2);
        plan_job(33,1,0);start_job;finish_jobs(3);end_case(0,0);

        $display("TOTAL: %0d PASS / %0d FAIL",pass_cnt,fail_cnt);
        if(fail_cnt!=0)$fatal(1,"Busy-start regression failed");
        $finish;
    end
    initial begin #30000000;$fatal(1,"Global transaction timeout");end
endmodule
