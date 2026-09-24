`timescale 1ns/1ps
// Supplemental V3 CDC regression. Existing RTL and regressions are unchanged.
// One startup reset per process; six jobs share the live FIFO/toggle state.
// Interconnect and memory-model instance wiring follows tb_mac_accel_dma_rob_top.v.
module tb_mac_accel_dma_rob_cdc;

    localparam DATA_WIDTH      = 16;
    localparam AXI_DATA_WIDTH  = 32;
    localparam AXI_ADDR_WIDTH  = 32;
    localparam AXI_ID_WIDTH    = 4;
    localparam MAX_LEN         = 256;
    localparam BURST_LEN       = 16;
    localparam MAX_OUT         = 8;
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
    real mac_period = 7.5;
    real mac_phase = 0.0;
    integer reset_bus_first = 0;
    integer arg_status;
    always #5.0 bus_clk = ~bus_clk;
    initial begin
        arg_status = $value$plusargs("MAC_PERIOD=%f", mac_period);
        arg_status = $value$plusargs("MAC_PHASE=%f", mac_phase);
        arg_status = $value$plusargs("RESET_BUS_FIRST=%d", reset_bus_first);
        if (mac_period <= 0.0 || mac_phase < 0.0) $fatal(1, "Invalid clock setting");
        #(mac_phase);
        forever #(mac_period/2.0) mac_clk = ~mac_clk;
    end

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

    integer ticks=0, job=0, passes=0;
    integer n=0, writes=0, reads=0, starts_bus=0, starts_mac=0;
    integer dones_mac=0, dones_bus=0, ar_count=0, r_count=0;
    integer inflight=0, peak_inflight=0, full_cycles=0, stall_cycles=0;
    integer wr_wraps=0, rd_wraps=0;
    reg live[0:15];
    integer remaining[0:15];
    reg signed [15:0] golden_a[0:MAX_LEN-1], golden_b[0:MAX_LEN-1];
    reg [32:0] fifo_history[0:MAX_LEN-1];
    reg signed [31:0] golden_sum=0;
    reg active=0, stalled=0, ar_stalled=0;
    reg [32:0] held_payload;
    reg [48:0] held_ar_payload;
    reg [4:0] old_wgray, old_rgray, wdiff, rdiff;
    integer trace_file;

    task automatic require(input reg condition, input string message);
        if (condition !== 1'b1) $fatal(1,"CDC_FAIL job=%0d t=%0t %s",job,$time,message);
    endtask

    // Deterministic AR stalls; no dependency on simulator-specific random seeds.
    always @(posedge bus_clk) begin
        if (!aresetn) begin ticks<=0; ar_stall<=0; end
        else begin
            ticks<=ticks+1;
            ar_stall<=ar_bp_mode && ((ticks%7==2)||(ticks%7==3));
        end
    end

    // AW and W acceptance are tracked independently; drive away from posedge.
    task automatic axi_write(input [7:0] address, input [31:0] value);
        reg aw_done,w_done;
        integer timeout;
        begin
            @(negedge bus_clk);
            S_AXI_AWADDR=address; S_AXI_AWVALID=1;
            S_AXI_WDATA=value; S_AXI_WSTRB=15; S_AXI_WVALID=1;
            S_AXI_BREADY=1; aw_done=0; w_done=0; timeout=0;
            while (!(aw_done && w_done)) begin
                @(posedge bus_clk);
                if (S_AXI_AWVALID && S_AXI_AWREADY===1'b1) aw_done=1;
                if (S_AXI_WVALID && S_AXI_WREADY===1'b1) w_done=1;
                timeout=timeout+1;
                require(timeout<1000,"AXI-Lite AW/W timeout");
                @(negedge bus_clk);
                if(aw_done) S_AXI_AWVALID=0;
                if(w_done) S_AXI_WVALID=0;
            end
            @(posedge bus_clk);
            while(S_AXI_BVALID!==1'b1) begin
                timeout=timeout+1; require(timeout<1000,"AXI-Lite B timeout");
                @(posedge bus_clk);
            end
            require(S_AXI_BRESP===2'b00,"AXI-Lite BRESP error");
            @(negedge bus_clk); S_AXI_BREADY=0;
        end
    endtask

    task automatic axi_read(input [7:0] address, output [31:0] value);
        integer timeout;
        begin
            @(negedge bus_clk); S_AXI_ARADDR=address; S_AXI_ARVALID=1; S_AXI_RREADY=1;
            timeout=0;
            @(posedge bus_clk);
            while(S_AXI_ARREADY!==1'b1) begin
                timeout=timeout+1; require(timeout<1000,"AXI-Lite AR timeout");
                @(posedge bus_clk);
            end
            @(negedge bus_clk); S_AXI_ARVALID=0;
            @(posedge bus_clk);
            while(S_AXI_RVALID!==1'b1) begin
                timeout=timeout+1; require(timeout<1000,"AXI-Lite R timeout");
                @(posedge bus_clk);
            end
            require(S_AXI_RRESP===2'b00,"AXI-Lite RRESP error");
            value=S_AXI_RDATA;
            @(negedge bus_clk); S_AXI_RREADY=0;
        end
    endtask

    // Monitors sample pre-NBA handshake data. The FIFO synchronizers ensure
    // a read never depends on a new write at a coincident clock edge.
    always @(posedge bus_clk) begin
        if(aresetn && active) begin
            if(dut.start_pulse_bus) begin
                starts_bus=starts_bus+1; require(starts_bus==1,"duplicate bus start");
                $fdisplay(trace_file,"%0.3f,%0d,START_BUS,%0d",$realtime,job,dut.start_toggle_bus);
            end
            if(stalled) require({dut.stream_valid,dut.fifo_din_bus}==={1'b1,held_payload},"stalled FIFO input changed");
            stalled=dut.stream_valid && !dut.stream_ready;
            held_payload=dut.fifo_din_bus;
            if(stalled) stall_cycles=stall_cycles+1;
            if(dut.fifo_full_bus) full_cycles=full_cycles+1;
            if(ar_stalled) require({M_AXI_ARVALID,M_AXI_ARID,M_AXI_ARADDR,M_AXI_ARLEN,M_AXI_ARSIZE,M_AXI_ARBURST}==={1'b1,held_ar_payload},"AR payload changed under stall");
            ar_stalled=M_AXI_ARVALID && !M_AXI_ARREADY;
            held_ar_payload={M_AXI_ARID,M_AXI_ARADDR,M_AXI_ARLEN,M_AXI_ARSIZE,M_AXI_ARBURST};
            // Process response completion before allocation if both happen together.
            if(M_AXI_RVALID && M_AXI_RREADY) begin
                require((^M_AXI_RID)!==1'bx,"unknown RID");
                require(live[M_AXI_RID],"response for unallocated ID");
                require(M_AXI_RRESP===2'b00,"AXI RRESP error");
                require(M_AXI_RLAST===(remaining[M_AXI_RID]==1),"RLAST/beat geometry mismatch");
                remaining[M_AXI_RID]=remaining[M_AXI_RID]-1;
                r_count=r_count+1;
                if(M_AXI_RLAST) begin live[M_AXI_RID]=0; inflight=inflight-1; end
            end
            if(M_AXI_ARVALID && M_AXI_ARREADY) begin
                require((^M_AXI_ARID)!==1'bx && M_AXI_ARID<8,"invalid ARID");
                require(!live[M_AXI_ARID],"ID reused before completion");
                live[M_AXI_ARID]=1; remaining[M_AXI_ARID]=M_AXI_ARLEN+1;
                inflight=inflight+1; if(inflight>peak_inflight)peak_inflight=inflight;
                require(inflight<=8,"too many outstanding reads");
                ar_count=ar_count+1;
            end
            if(dut.fifo_wr_en && !dut.fifo_full_bus) begin
                require(writes<n,"extra FIFO write");
                require(dut.fifo_din_bus==={(writes==n-1),golden_a[writes],golden_b[writes]},"wrong FIFO input data/last/order");
                fifo_history[writes]=dut.fifo_din_bus;
                if(dut.u_fifo.wr_ptr_bin[3:0]==15)wr_wraps=wr_wraps+1;
                writes=writes+1;
            end
            if(dut.done_pulse_bus) begin
                dones_bus=dones_bus+1;
                require(dones_bus==1 && dones_mac==1,"duplicate/early bus done");
                require(writes==n && reads==n,"done with missing FIFO data");
                require(inflight==0 && r_count==2*n,"missing AXI responses");
                $fdisplay(trace_file,"%0.3f,%0d,DONE_BUS,%0d",$realtime,job,dut.result_mac);
                #0.001;
                require(dut.res_bus_reg===golden_sum,"result capture mismatch");
                require(dut.lat_bus_reg===dut.latency_mac,"latency capture mismatch");
            end
        end
    end
    always @(posedge mac_clk) begin
        if(!mac_rst && active) begin
            if(dut.start_pulse_mac) begin
                starts_mac=starts_mac+1; require(starts_mac==1,"duplicate MAC start");
                $fdisplay(trace_file,"%0.3f,%0d,START_MAC,%0d",$realtime,job,dut.start_sync_mac[1]);
            end
            if(dut.fifo_rd_en_mac && !dut.fifo_empty_mac) begin
                require(reads<writes && reads<n,"extra FIFO read");
                require(dut.fifo_dout_mac===fifo_history[reads],"FIFO read data/order/last mismatch");
                require({dut.u_pe.last_in_r,dut.u_pe.a_in_r,dut.u_pe.b_in_r}===fifo_history[reads],"PE captured payload mismatch");
                require(dut.u_pe.valid_in_r===1'b1,"FIFO pop without PE valid");
                if(dut.u_fifo.rd_ptr_bin[3:0]==15)rd_wraps=rd_wraps+1;
                reads=reads+1;
            end
            if(dut.done_mac) begin
                dones_mac=dones_mac+1;
                require(dones_mac==1 && starts_mac==1 && reads==n,"duplicate/early MAC done");
                require(dut.result_mac===golden_sum,"MAC reference result mismatch");
                $fdisplay(trace_file,"%0.3f,%0d,DONE_MAC,%0d",$realtime,job,dut.result_mac);
            end
        end
    end
    always @(posedge bus_clk) begin
        old_wgray=dut.u_fifo.wr_ptr_gray;
        if(aresetn) begin
            #0.001; wdiff=old_wgray^dut.u_fifo.wr_ptr_gray;
            if(aresetn)require((^wdiff!==1'bx)&&((wdiff&(wdiff-1))==0),"write Gray transition invalid");
        end
    end
    always @(posedge mac_clk) begin
        old_rgray=dut.u_fifo.rd_ptr_gray;
        if(!mac_rst) begin
            #0.001; rdiff=old_rgray^dut.u_fifo.rd_ptr_gray;
            if(!mac_rst)require((^rdiff!==1'bx)&&((rdiff&(rdiff-1))==0),"read Gray transition invalid");
        end
    end

    task automatic run_job(input integer len,input integer latency,input reg use_stalls);
        integer k,timeout;
        reg [31:0] res,ctrl,lat;
        begin
            @(negedge bus_clk);
            require(dut.busy_any_bus===1'b0,"new job while busy");
            require(dut.fifo_empty_mac===1'b1,"FIFO not drained before next job");
            job=job+1; n=len; golden_sum=0;
            writes=0;reads=0;starts_bus=0;starts_mac=0;dones_mac=0;dones_bus=0;
            ar_count=0;r_count=0;inflight=0;peak_inflight=0;full_cycles=0;stall_cycles=0;
            wr_wraps=0;rd_wraps=0;stalled=0;ar_stalled=0;
            for(k=0;k<16;k=k+1)begin live[k]=0; remaining[k]=0; end
            for(k=0;k<n;k=k+1)begin
                golden_a[k]=((k*13+job*7)%257)-128;
                golden_b[k]=((k*17+job*11)%251)-125;
                golden_sum=golden_sum+golden_a[k]*golden_b[k];
                u_mem.mem[k]={{16{golden_a[k][15]}},golden_a[k]};
                u_mem.mem[1024+k]={{16{golden_b[k][15]}},golden_b[k]};
            end
            active=1; mem_latency=latency;ar_bp_mode=use_stalls;ooo_mode=1;
            axi_write(ADDR_SRC_A,0);axi_write(ADDR_SRC_B,32'h1000);axi_write(ADDR_LENGTH,n);
            axi_write(ADDR_CTRL,1);
            // Explicitly observe stale done clearing before accepting completion.
            timeout=0;
            while(done_led!==1'b0) begin
                @(negedge bus_clk);timeout=timeout+1;require(timeout<1000,"stale done did not clear");
            end
            timeout=0;
            while(dones_bus!=1)begin
                @(negedge bus_clk);timeout=timeout+1;require(timeout<200000,"job completion timeout");
            end
            axi_read(ADDR_RESULT,res);axi_read(ADDR_LATENCY,lat);axi_read(ADDR_CTRL,ctrl);
            require(res===golden_sum,"CSR result mismatch/stale result");
            require(ctrl[3:1]===3'b010,"CSR error/done/busy state wrong");
            require(lat===dut.lat_bus_reg && lat>0,"CSR latency mismatch");
            require(starts_bus==1 && starts_mac==1 && dones_mac==1 && dones_bus==1,"event conservation failure");
            require(ar_count==2*((n+15)/16) && r_count==2*n,"AR/R conservation failure");
            require(writes==n && reads==n,"FIFO conservation failure");
            require(dut.start_toggle_bus===(job%2!=0),"start toggle did not alternate across jobs");
            require(dut.done_toggle_mac===(job%2!=0),"done toggle did not alternate across jobs");
            if(n>=129) require(peak_inflight==8,"depth-8 scenario did not reach 8 outstanding");
            $display("CDC_JOB_PASS job=%0d len=%0d result=%0d writes=%0d reads=%0d ar=%0d r=%0d peak=%0d full=%0d stalls=%0d wr_wraps=%0d rd_wraps=%0d toggle=%0d",job,n,$signed(res),writes,reads,ar_count,r_count,peak_inflight,full_cycles,stall_cycles,wr_wraps,rd_wraps,dut.start_toggle_bus);
            passes=passes+1;
        end
    endtask

    initial begin
        trace_file=$fopen("cdc_events.csv","w");
        $fdisplay(trace_file,"time_ns,job,event,value");
        aresetn=0;mac_rst=1;ar_bp_mode=0;ar_stall=0;ooo_mode=1;mem_latency=100;
        S_AXI_AWADDR=0;S_AXI_AWVALID=0;S_AXI_WDATA=0;S_AXI_WSTRB=0;S_AXI_WVALID=0;S_AXI_BREADY=0;
        S_AXI_ARADDR=0;S_AXI_ARVALID=0;S_AXI_RREADY=0;
        repeat(12)@(posedge bus_clk);repeat(12)@(posedge mac_clk);
        if(reset_bus_first)begin
            @(negedge bus_clk);aresetn=1;
            repeat(4)@(posedge mac_clk);@(negedge mac_clk);mac_rst=0;
        end else begin
            @(negedge mac_clk);mac_rst=0;
            repeat(4)@(posedge bus_clk);@(negedge bus_clk);aresetn=1;
        end
        repeat(8)@(posedge bus_clk);
        $display("CDC_CONFIG bus_period=10 mac_period=%0.3f mac_phase=%0.3f bus_reset_first=%0d MAX_OUT=8",mac_period,mac_phase,reset_bus_first);
        run_job(1,0,0);
        run_job(17,40,1);
        run_job(129,100,0);
        run_job(256,100,1);
        run_job(33,0,1);
        run_job(1,0,0);
        // Keep the last job's monitors active to detect delayed extra events.
        repeat(20)@(posedge bus_clk);repeat(20)@(posedge mac_clk);
        require(starts_bus==1 && starts_mac==1 && dones_mac==1 && dones_bus==1,"late duplicate event");
        require(passes==6,"missing jobs");
        $display("CDC_TOTAL: 6 PASS / 0 FAIL; startup_resets=1; interjob_resets=0");
        $fclose(trace_file);$finish;
    end
    initial begin
        #20000000;$fatal(1,"CDC_FAIL global timeout");
    end
endmodule
