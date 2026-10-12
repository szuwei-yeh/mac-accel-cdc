`timescale 1ns/1ps

// V3 error ownership: no reset between jobs in each scenario.
// AXI response faults are selected from accepted AR addresses and beat indices,
// not DUT state. A one-beat response buffer holds every R payload through stalls.
// Signed arithmetic, byte/beat counts and expected faults use independent plans.
module tb_mac_accel_dma_rob_error;
    localparam LIMIT = 80, JOB_LIMIT = 30000;
    reg bus_clk = 0, mac_clk = 0;
    always #5 bus_clk = ~bus_clk;
    real mac_period = 17.0;
    initial begin #1.25; forever #(mac_period/2.0) mac_clk = ~mac_clk; end
    reg resetn = 0, mac_rst = 1;
    reg [7:0] awaddr = 0, araddr = 0;
    reg awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
    reg [31:0] wdata = 0;
    wire awready, wready, bvalid, arready, rvalid;
    wire [31:0] rdata;
    wire [1:0] bresp, rresp;
    wire [3:0] mid, raw_rid;
    wire [31:0] maddr, raw_data;
    wire [7:0] mlen;
    wire [2:0] msize;
    wire [1:0] mburst, raw_resp;
    wire marvalid, marready, raw_valid, raw_ready, raw_last, mrready;
    reg rx_valid = 0, rx_last = 0, rx_b = 0;
    reg [3:0] rx_id = 0;
    reg [31:0] rx_data = 0;
    reg [1:0] rx_resp = 0;
    integer rx_job = 0, rx_element = 0;
    reg response_gaps = 0;
    integer ticks = 0;
    assign raw_ready = !rx_valid && (!response_gaps || (ticks % 7 >= 3));

    mac_accel_dma_rob_top #(.MAX_OUTSTANDING(8)) dut (
        .S_AXI_ACLK(bus_clk), .S_AXI_ARESETN(resetn),
        .S_AXI_AWADDR(awaddr), .S_AXI_AWPROT(3'b0), .S_AXI_AWVALID(awvalid), .S_AXI_AWREADY(awready),
        .S_AXI_WDATA(wdata), .S_AXI_WSTRB(4'hf), .S_AXI_WVALID(wvalid), .S_AXI_WREADY(wready),
        .S_AXI_BRESP(bresp), .S_AXI_BVALID(bvalid), .S_AXI_BREADY(bready),
        .S_AXI_ARADDR(araddr), .S_AXI_ARPROT(3'b0), .S_AXI_ARVALID(arvalid), .S_AXI_ARREADY(arready),
        .S_AXI_RDATA(rdata), .S_AXI_RRESP(rresp), .S_AXI_RVALID(rvalid), .S_AXI_RREADY(rready),
        .M_AXI_ARID(mid), .M_AXI_ARADDR(maddr), .M_AXI_ARLEN(mlen), .M_AXI_ARSIZE(msize),
        .M_AXI_ARBURST(mburst), .M_AXI_ARVALID(marvalid), .M_AXI_ARREADY(marready),
        .M_AXI_RID(rx_id), .M_AXI_RDATA(rx_data), .M_AXI_RRESP(rx_resp), .M_AXI_RLAST(rx_last),
        .M_AXI_RVALID(rx_valid), .M_AXI_RREADY(mrready), .mac_clk(mac_clk), .mac_rst(mac_rst)
    );
    axi_read_mem_model_ooo #(.MEM_WORDS(2048), .MAX_OUTSTANDING(8)) mem (
        .clk(bus_clk), .resetn(resetn), .ar_latency(16'd5), .ar_stall(1'b0), .ooo_en(1'b1),
        .arid(mid), .araddr(maddr), .arlen(mlen), .arsize(msize), .arburst(mburst),
        .arvalid(marvalid), .arready(marready), .rid(raw_rid), .rdata(raw_data), .rresp(raw_resp),
        .rlast(raw_last), .rvalid(raw_valid), .rready(raw_ready)
    );

    integer lengths[0:3], patterns[0:3], sums[0:3], masks[0:3];
    reg final_only[0:3];
    reg [31:0] abase[0:3], bbase[0:3];
    integer ar_jobs[0:3], r_jobs[0:3], a_faults[0:3], b_faults[0:3];
    integer final_b_faults[0:3], final_a_faults[0:3];
    integer tag_job[0:15], tag_element[0:15], tag_count[0:15], tag_beats[0:15];
    reg tag_valid[0:15], tag_b[0:15];
    integer planned = 0, requested_starts = 0, bus_starts = 0, mac_starts = 0;
    integer bus_dones = 0, mac_dones = 0, toggles = 0, rejected = 0;
    integer ar_count = 0, r_count = 0, fw_count = 0, fr_count = 0;
    integer ar_index = 0, write_index = 0, read_index = 0;
    integer aws = 0, ws = 0, bs = 0, gap_cycles = 0, output_stalls = 0;
    integer capture_tick = -1, immediate_commits = 0;
    reg expect_immediate = 0, previous_toggle = 0;
    integer pass_cnt = 0, fail_cnt = 0;
    reg failed = 0;
    reg [511:0] case_name;

    function automatic signed [15:0] operand(input integer pattern, input integer index, input is_b);
        operand = pattern == 0 ? (is_b ? index%5+1 : index%7-3) : (is_b ? 3 : -2*(index+1));
    endfunction
    function automatic [1:0] planned_resp(input integer job, input is_b, input integer index);
        planned_resp = ((is_b ? (masks[job]&2) : (masks[job]&1)) != 0 &&
                        (!final_only[job] || index == lengths[job]-1)) ? 2'b10 : 2'b00;
    endfunction
    task automatic check(input condition, input [767:0] detail);
        if (condition !== 1'b1) begin failed = 1; $display("FAIL: %0s: %0s",case_name,detail); end
    endtask

    integer j, k, bursts, elem, n;
    reg is_b, consume_done, commit_start, had_b_stall = 0, had_r_stall = 0, had_raw_stall = 0;
    reg [1:0] held_b;
    reg [38:0] held_r, held_raw;
    reg latch_due = 0, final_b_now, consume_start, command_accept;
    reg old_dma_fault, old_bus_fault;
    always @(posedge bus_clk) begin
        if (!resetn) begin
            ticks = 0; rx_valid <= 0; bus_starts = 0; bus_dones = 0; toggles = 0;
            ar_count = 0; r_count = 0; fw_count = 0; ar_index = 0; write_index = 0; read_index = 0;
            aws = 0; ws = 0; bs = 0; gap_cycles = 0; output_stalls = 0;
            capture_tick = -1; immediate_commits = 0; previous_toggle = 0;
            had_b_stall = 0; had_r_stall = 0; had_raw_stall = 0; latch_due = 0;
            for (j=0;j<16;j=j+1) tag_valid[j] = 0;
            for (j=0;j<4;j=j+1) begin
                ar_jobs[j]=0; r_jobs[j]=0; a_faults[j]=0; b_faults[j]=0;
                final_b_faults[j]=0; final_a_faults[j]=0;
            end
        end else begin
            ticks = ticks+1;
            consume_start=dut.start_pulse_bus;
            command_accept=dut.u_dma.engine_cmd_valid && dut.u_dma.engine_cmd_ready;
            old_dma_fault=dut.u_dma.dma_err; old_bus_fault=dut.dma_err_bus;
            if (had_b_stall) check(bvalid && bresp===held_b,"B response stability");
            had_b_stall = bvalid && !bready; held_b = bresp;
            if (had_r_stall) check(rx_valid && {rx_id,rx_data,rx_resp,rx_last}===held_r,"DUT R stability");
            had_r_stall = rx_valid && !mrready; held_r = {rx_id,rx_data,rx_resp,rx_last};
            if (had_raw_stall) check(raw_valid && {raw_rid,raw_data,raw_resp,raw_last}===held_raw,"memory R stability");
            had_raw_stall = raw_valid && !raw_ready; held_raw = {raw_rid,raw_data,raw_resp,raw_last};
            if (awvalid && awready) aws=aws+1;
            if (wvalid && wready) ws=ws+1;
            if (bvalid && bready) bs=bs+1;
            if (dut.start_pulse_bus) begin
                bus_starts=bus_starts+1;
                check(bus_starts<=requested_starts,"unexpected bus start");
                ar_index=0; write_index=0; read_index=0;
            end
            if (marvalid && marready) begin
                j=bus_starts-1;
                check(j>=0 && j<planned,"AR without planned start");
                if (j>=0 && j<planned) begin
                    bursts=(lengths[j]+15)/16; is_b=(ar_index>=bursts); elem=(ar_index%bursts)*16;
                    check(ar_index<2*bursts,"extra AR");
                    check(maddr===(is_b ? bbase[j] : abase[j])+elem*4,"AR address sequence");
                    check(mlen===(lengths[j]-elem>16 ? 8'd15 : lengths[j]-elem-1),"AR burst geometry");
                    check(msize===2 && mburst===1,"AR size/type");
                    check(!tag_valid[mid],"tag reused before response completed");
                    tag_job[mid]=j; tag_b[mid]=is_b; tag_element[mid]=elem;
                    tag_count[mid]=0; tag_beats[mid]=mlen+1; tag_valid[mid]=1;
                    ar_jobs[j]=ar_jobs[j]+1;
                end
                ar_index=ar_index+1; ar_count=ar_count+1;
            end
            // Legal source buffering: raw_ready is low while an output beat is held.
            if (rx_valid && mrready) rx_valid<=0;
            if (raw_valid && raw_ready) begin
                check(tag_valid[raw_rid],"response for unowned ID");
                j=tag_job[raw_rid]; elem=tag_element[raw_rid]+tag_count[raw_rid];
                check(raw_last===(tag_count[raw_rid]==tag_beats[raw_rid]-1),"per-ID RLAST geometry");
                check($signed(raw_data)===$signed(operand(patterns[j],elem,tag_b[raw_rid])),"memory beat payload");
                rx_valid<=1; rx_id<=raw_rid; rx_data<=raw_data; rx_last<=raw_last;
                rx_job<=j; rx_b<=tag_b[raw_rid]; rx_element<=elem;
                rx_resp<=planned_resp(j,tag_b[raw_rid],elem);
                tag_count[raw_rid]=tag_count[raw_rid]+1;
                if (raw_last) tag_valid[raw_rid]=0;
            end
            if (response_gaps && raw_valid && !rx_valid && !raw_ready) gap_cycles=gap_cycles+1;
            if (dut.stream_valid && !dut.stream_ready) output_stalls=output_stalls+1;
            final_b_now=0;
            if (rx_valid && mrready) begin
                j=rx_job;
                check(j==bus_starts-1,"R belongs to wrong job");
                check(rx_resp===planned_resp(j,rx_b,rx_element),"current-job RRESP plan");
                r_count=r_count+1; r_jobs[j]=r_jobs[j]+1;
                if (rx_resp!=0) begin
                    if (rx_b) b_faults[j]=b_faults[j]+1; else a_faults[j]=a_faults[j]+1;
                    if (rx_element==lengths[j]-1) begin
                        check(rx_last,"last element must have RLAST");
                        if (rx_b) begin final_b_faults[j]=final_b_faults[j]+1; final_b_now=1; end
                        else final_a_faults[j]=final_a_faults[j]+1;
                    end
                end
            end
            if (dut.fifo_wr_en) begin
                j=bus_starts-1;
                check(write_index<lengths[j],"extra FIFO write");
                check(dut.stream_a===operand(patterns[j],write_index,0) &&
                      dut.stream_b===operand(patterns[j],write_index,1),"operand pairing/order");
                check(dut.stream_last===(write_index==lengths[j]-1),"stream last position");
                write_index=write_index+1; fw_count=fw_count+1;
            end
            consume_done=dut.done_pulse_bus;
            if (consume_done) begin
                check(bus_dones<bus_starts && mac_dones==bus_dones+1,"completion event ownership");
                bus_dones=bus_dones+1; capture_tick=ticks;
            end
            commit_start=dut.aw_latched && dut.w_latched && !bvalid && dut.aw_word_addr==0;
            if (commit_start && expect_immediate) begin
                check(ticks==capture_tick+1,"next start did not commit first safe edge");
                immediate_commits=immediate_commits+1; expect_immediate=0;
            end
            if (final_b_now)
                $display("FINAL_B_R job=%0d tick=%0d state=%0d engine_err_before=%b dma_err_before=%b",
                         rx_job,ticks,dut.u_dma.state,dut.u_dma.engine_err,dut.u_dma.dma_err);
            if (dut.u_dma.engine_done)
                $display("PHASE_DONE tick=%0d state=%0d engine_err=%b dma_err=%b",
                         ticks,dut.u_dma.state,dut.u_dma.engine_err,dut.u_dma.dma_err);
            if (consume_start || command_accept)
                $display("OWNERSHIP_BEFORE tick=%0d start=%b cmd=%b state=%0d engine_err=%b dma_err=%b bus_err=%b",
                         ticks,consume_start,command_accept,dut.u_dma.state,
                         dut.u_dma.engine_err,dut.u_dma.dma_err,dut.dma_err_bus);
            #1;
            if (consume_start) check(!dut.u_dma.dma_err && !dut.dma_err_bus,"new start must clear both sticky faults");
            else begin
                if (old_dma_fault) check(dut.u_dma.dma_err,"wrapper lost sticky fault within job");
                if (old_bus_fault) check(dut.dma_err_bus,"top lost sticky fault within job");
            end
            if (command_accept) check(!dut.u_dma.engine_err,"new command must clear engine sticky fault");
            if (bus_starts>bus_dones && masks[bus_starts-1]==0)
                check(!dut.u_dma.dma_err && !dut.dma_err_bus,"clean active job inherited stale fault");
            if (consume_start || command_accept)
                $display("OWNERSHIP_AFTER tick=%0d state=%0d engine_err=%b dma_err=%b bus_err=%b",
                         ticks,dut.u_dma.state,dut.u_dma.engine_err,dut.u_dma.dma_err,dut.dma_err_bus);
            if (latch_due) $display("FINAL_B_LATCH tick=%0d state=%0d engine_err=%b dma_err=%b",
                                   ticks,dut.u_dma.state,dut.u_dma.engine_err,dut.u_dma.dma_err);
            if (latch_due) check(dut.u_dma.dma_err===1'b1,"final B fault not captured by wrapper next edge");
            latch_due=final_b_now;
            if (final_b_now) check(dut.u_dma.engine_err===1'b1,"engine lost final B R fault");
            if (dut.start_toggle_bus!==previous_toggle) begin toggles=toggles+1; previous_toggle=dut.start_toggle_bus; end
            if (consume_done) begin
                j=bus_dones-1;
                check(dut.res_bus_reg===sums[j] && dut.lat_bus_reg>0,"completion result/latency");
                check(dut.done_bus && dut.dma_err_bus===(masks[j]!=0),"completed job sticky flags");
            end
        end
    end

    integer mac_job;
    always @(posedge mac_clk or posedge mac_rst) begin
        if (mac_rst) begin mac_starts=0; mac_dones=0; fr_count=0; end
        else begin
            if (dut.start_pulse_mac) begin mac_starts=mac_starts+1; check(mac_starts<=requested_starts,"extra MAC start"); end
            if (dut.fifo_rd_en_mac && !dut.fifo_empty_mac) begin
                mac_job=bus_starts-1;
                check(read_index<lengths[mac_job],"extra FIFO read");
                check(dut.fifo_dout_mac==={(read_index==lengths[mac_job]-1),
                      operand(patterns[mac_job],read_index,0),operand(patterns[mac_job],read_index,1)},"FIFO payload/order/last");
                read_index=read_index+1; fr_count=fr_count+1;
            end
            if (dut.done_mac) begin
                mac_job=mac_dones; mac_dones=mac_dones+1;
                check(mac_dones<=requested_starts && dut.result_mac===sums[mac_job],"MAC result/completion");
            end
        end
    end

    // Reuse the original F01 reproducer's independent, bounded AXI-Lite drivers.
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
        @(negedge bus_clk); wdata = data;  wvalid = 1;
        n = 0; accepted = 0;
        while (!accepted && n < LIMIT) begin
            @(posedge bus_clk); accepted = wready === 1'b1; n = n + 1;
        end
        if (!accepted) $fatal(1, "W transaction timeout");
        // Poison live payload after the handshake to verify latched data/strobes.
        @(negedge bus_clk); wvalid = 0; wdata = ~data;
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

    task automatic write_csr(input [7:0] addr,input [31:0] data,input [1:0] response,input integer hold_cycles);
        integer t;
    begin
        issue(addr,data,4'hf,0); t=0;
        while (!bvalid && t<LIMIT) begin @(negedge bus_clk);t=t+1;end
        if (!bvalid) $fatal(1,"B transaction timeout");
        check(bresp===response,"AXI-Lite BRESP");
        repeat(hold_cycles) @(negedge bus_clk);
        @(negedge bus_clk);bready=1;@(posedge bus_clk);
        if (!bvalid) $fatal(1,"missing B handshake");
        @(negedge bus_clk);bready=0;
    end
    endtask
    task automatic begin_case(input [511:0] name,input gaps,input real period);
        integer i;
    begin
        @(negedge bus_clk);resetn=0;mac_rst=1;
        awvalid=0;wvalid=0;bready=0;arvalid=0;rready=0;
        planned=0;requested_starts=0;rejected=0;failed=0;expect_immediate=0;
        case_name=name;response_gaps=gaps;mac_period=period;
        repeat(8) @(negedge bus_clk);
        for(i=0;i<2048;i=i+1) mem.mem[i]=0;
        @(negedge mac_clk);mac_rst=0;
        @(negedge bus_clk);resetn=1;repeat(5) @(negedge bus_clk);
    end
    endtask
    task automatic plan_job(input integer length,input integer pattern,input integer fault_mask,input only_final);
        integer slot,i,total;
        reg signed [15:0] a,b;
    begin
        slot=planned;lengths[slot]=length;patterns[slot]=pattern;masks[slot]=fault_mask;final_only[slot]=only_final;
        abase[slot]=slot*32'h400;bbase[slot]=32'h1000+slot*32'h400;total=0;
        for(i=0;i<length;i=i+1) begin
            a=operand(pattern,i,0);b=operand(pattern,i,1);
            mem.mem[(abase[slot]>>2)+i]={{16{a[15]}},a};
            mem.mem[(bbase[slot]>>2)+i]={{16{b[15]}},b};total=total+a*b;
        end
        sums[slot]=total;planned=planned+1;
        write_csr(8'h04,abase[slot],0,0);write_csr(8'h08,bbase[slot],0,0);write_csr(8'h0c,length,0,0);
    end
    endtask
    task automatic start_job;
    begin requested_starts=requested_starts+1;write_csr(0,1,0,0);end
    endtask
    task automatic wait_done(input integer count);
        integer t;
    begin
        t=0;while(bus_dones<count && t<JOB_LIMIT)begin @(negedge bus_clk);t=t+1;end
        if(bus_dones!=count) $fatal(1,"job completion timeout");
    end
    endtask
    reg [31:0] csr;
    task automatic finish_job(input integer count);
        reg [31:0] ctrl,value;
    begin
        wait_done(count);
        read_csr(0,ctrl);read_csr(8'h10,value);
        check(ctrl===(masks[count-1]!=0 ? 32'hc : 32'h4),"CTRL done/fault/busy for completed job");
        check(value===sums[count-1],"signed result CSR");
        $display("JOB_CTRL job=%0d len=%0d mask=%0d final_only=%b actual=%h expected=%h result=%0d",
                 count,lengths[count-1],masks[count-1],final_only[count-1],ctrl,
                 masks[count-1]!=0 ? 32'hc : 32'h4,$signed(value));
    end
    endtask
    task automatic end_case(input integer expect_rejects,input need_gaps,input need_backpressure,input need_immediate);
        integer i,total_len,total_ar,expected_a,expected_b;
    begin
        repeat(12) @(negedge bus_clk);total_len=0;total_ar=0;
        for(i=0;i<planned;i=i+1) begin
            total_len=total_len+lengths[i];total_ar=total_ar+2*((lengths[i]+15)/16);
            expected_a=(masks[i]&1) ? (final_only[i] ? 1 : lengths[i]) : 0;
            expected_b=(masks[i]&2) ? (final_only[i] ? 1 : lengths[i]) : 0;
            check(ar_jobs[i]==2*((lengths[i]+15)/16) && r_jobs[i]==2*lengths[i],"per-job AR/R conservation");
            check(a_faults[i]==expected_a && b_faults[i]==expected_b,"non-vacuous phase fault coverage");
            check(final_a_faults[i]==((masks[i]&1)!=0) && final_b_faults[i]==((masks[i]&2)!=0),"final phase beat fault coverage");
        end
        check(bus_starts==planned && requested_starts==planned && toggles==planned &&
              mac_starts==planned && mac_dones==planned && bus_dones==planned,"exact start/toggle/completion counts");
        check(ar_count==total_ar && r_count==2*total_len,"total AR/R conservation");
        check(fw_count==total_len && fr_count==total_len && dut.fifo_empty_mac && !rx_valid,"no extra FIFO/response data");
        check(aws==ws && ws==bs && rejected==expect_rejects,"AW/W/B and rejection counts");
        check(!need_gaps || gap_cycles>0,"response gaps not exercised");
        check(!need_backpressure || output_stalls>0,"output backpressure not exercised");
        check(immediate_commits==(need_immediate ? 1 : 0),"immediate acceptance coverage");
        $display("CASE_COUNTS jobs=%0d starts=%0d toggles=%0d MAC_start=%0d MAC_done=%0d BUS_done=%0d AR=%0d R=%0d FIFO=%0d/%0d gaps=%0d output_stalls=%0d rejected=%0d immediate=%0d",
                 planned,bus_starts,toggles,mac_starts,mac_dones,bus_dones,ar_count,r_count,fw_count,fr_count,gap_cycles,output_stalls,rejected,immediate_commits);
        if(failed)begin fail_cnt=fail_cnt+1;$display("FAIL: %0s",case_name);end
        else begin pass_cnt=pass_cnt+1;$display("PASS: %0s",case_name);end
    end
    endtask

    initial begin
        begin_case("01 minimal final B fault -> clean len1",0,17);
        plan_job(1,1,2,1);start_job;finish_job(1);
        plan_job(1,0,0,1);start_job;finish_job(2);end_case(0,0,0,0);

        begin_case("02 clean -> clean len16/17",0,17);
        plan_job(16,0,0,0);start_job;finish_job(1);
        plan_job(17,1,0,0);start_job;finish_job(2);end_case(0,0,0,0);

        begin_case("03 A fault -> clean len1/17",0,17);
        plan_job(1,1,1,0);start_job;finish_job(1);
        plan_job(17,0,0,0);start_job;finish_job(2);end_case(0,0,0,0);

        begin_case("04 B faults -> clean len16",0,17);
        plan_job(16,0,2,0);start_job;finish_job(1);
        plan_job(16,1,0,0);start_job;finish_job(2);end_case(0,0,0,0);

        begin_case("05 A+B faults -> clean len17",0,17);
        plan_job(17,0,3,0);start_job;finish_job(1);
        plan_job(17,1,0,0);start_job;finish_job(2);end_case(0,0,0,0);

        begin_case("06 fault -> clean -> fault",1,17);
        plan_job(17,1,2,1);start_job;finish_job(1);
        plan_job(1,0,0,0);start_job;finish_job(2);
        plan_job(16,0,1,1);start_job;finish_job(3);end_case(0,1,0,0);

        begin_case("07 fault -> fault -> clean",1,17);
        plan_job(64,0,2,0);start_job;finish_job(1);
        plan_job(17,1,3,0);start_job;finish_job(2);
        plan_job(1,0,0,0);start_job;finish_job(3);end_case(0,1,0,0);

        begin_case("08 long partial burst with gaps/full FIFO",1,100);
        plan_job(65,1,3,1);start_job;finish_job(1);
        plan_job(65,0,0,0);start_job;finish_job(2);end_case(0,1,1,0);

        begin_case("09 final B only len16 -> clean17 with gaps",1,17);
        plan_job(16,1,2,1);start_job;finish_job(1);
        plan_job(17,0,0,0);start_job;finish_job(2);end_case(0,1,0,0);

        begin_case("10 busy rejection preserves active fault",1,100);
        plan_job(64,0,3,0);start_job;
        n=0;while(!dut.dma_err_bus && n<JOB_LIMIT)begin @(negedge bus_clk);n=n+1;end
        if(n==JOB_LIMIT)$fatal(1,"fault fixture timeout");
        read_csr(0,csr);check(csr===32'ha,"active CTRL busy/fault fixture");
        write_csr(0,1,2'b10,8);rejected=rejected+1;
        check(dut.dma_err_bus && dut.job_in_flight,"busy rejection cleared active state");
        finish_job(1);plan_job(17,1,0,0);start_job;finish_job(2);end_case(1,1,1,0);

        begin_case("11 accept next job first safe completion edge",1,17);
        plan_job(1,1,2,1);start_job;plan_job(1,0,0,0);
        n=0;while(!dut.done_pulse_bus && n<JOB_LIMIT)begin @(negedge bus_clk);n=n+1;end
        if(n==JOB_LIMIT)$fatal(1,"completion boundary timeout");
        // done is consumed on the next edge; queued AW/W commit one edge later.
        expect_immediate=1;requested_starts=requested_starts+1;
        awaddr=0;wdata=1;awvalid=1;wvalid=1;
        @(posedge bus_clk);@(negedge bus_clk);
        check(awready && wready,"boundary channel readiness");
        @(posedge bus_clk);@(negedge bus_clk);awvalid=0;wvalid=0;
        check(bvalid && bresp===0,"first safe edge start rejected");
        bready=1;@(posedge bus_clk);@(negedge bus_clk);bready=0;
        finish_job(2);end_case(0,1,0,1);

        begin_case("12 final A only len17 -> clean16 with gaps",1,17);
        plan_job(17,0,1,1);start_job;finish_job(1);
        plan_job(16,1,0,0);start_job;finish_job(2);end_case(0,1,0,0);

        $display("TOTAL: %0d PASS / %0d FAIL",pass_cnt,fail_cnt);
        if(fail_cnt!=0)$fatal(1,"V3 fault ownership regression failed");
        $finish;
    end
    initial begin #30000000;$fatal(1,"Global transaction timeout");end
endmodule
