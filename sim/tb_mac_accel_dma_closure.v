`timescale 1ns/1ps
// RTL closure: signed extremes, multi-seed consecutive jobs and final-beat errors.
// One coordinated startup reset; twelve jobs, no inter-job reset.
// Interconnect and memory-model instance wiring follows tb_mac_accel_dma_top.v.
module tb_mac_accel_dma_closure;

    localparam DATA_WIDTH      = 16;
    localparam AXI_DATA_WIDTH  = 32;
    localparam AXI_ADDR_WIDTH  = 32;
    localparam AXI_ID_WIDTH    = 4;
    localparam MAX_LEN         = 256;
    localparam BURST_LEN       = 16;
    localparam MAX_OUT         = 8;
    localparam MEM_WORDS       = 4096;

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
    mac_accel_dma_top #(
        .DATA_WIDTH      (DATA_WIDTH),
        .AXI_DATA_WIDTH  (AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH  (AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH    (AXI_ID_WIDTH),
        .MAX_LEN         (MAX_LEN),
        .BURST_LEN       (BURST_LEN)
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
        .M_AXI_RRESP    (injected_rresp),
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

    integer seed=1, arg_seed, rng;
    function [31:0] step(input [31:0] x);
        reg[31:0] t; begin t=x^(x<<13);t=t^(t>>17);step=t^(t<<5);end
    endfunction
    integer ticks=0, job=0, passes=0;
    integer total_full=0,total_stalls=0,total_ar_stalls=0,total_errors=0;
    integer total_nonhead=0,total_reuses=0,total_aw_first=0,total_w_first=0;
    integer total_b_stalls=0,total_r_stalls=0,total_partial=0;
    integer used[0:15], response_index[0:15];
    reg fault_job=0,holding_result=0;
    integer fault_mask=0;
    reg[31:0]stable_result,stable_latency;
    integer partial_bursts=0, slave_r_stalls=0;
    reg slave_r_held=0;
    reg[38:0]saved_slave_r;

    wire [1:0] injected_rresp = M_AXI_RLAST &&
        (((fault_mask&1)!=0 && response_index[M_AXI_RID]==n-1) ||
         ((fault_mask&2)!=0 && response_index[M_AXI_RID]==2*n-1)) ? 2'b10 : 2'b00;
    reg b_held=0,r_held=0;
    reg[1:0] held_br,held_rr;
    reg[31:0] held_rd;
    integer aws=0,ws=0,bs=0,csr_ars=0,csr_rs=0;
    reg signed[63:0] wide_sum, wide_product;

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
            rng=step(rng); ar_stall<=ar_bp_mode && (rng[3:0]<7);
        end
    end

    localparam LIMIT=200;
    task automatic send_aw(input [7:0] addr, input integer delay_cycles);
        integer n;
        reg accepted;
    begin
        repeat (delay_cycles) @(negedge bus_clk);
        @(negedge bus_clk); S_AXI_AWADDR = addr; S_AXI_AWVALID = 1;
        n = 0; accepted = 0;
        while (!accepted && n < LIMIT) begin
            @(posedge bus_clk); accepted = S_AXI_AWREADY === 1'b1; n = n + 1;
        end
        if (!accepted) $fatal(1, "AW transaction timeout");
        @(negedge bus_clk); S_AXI_AWVALID = 0; S_AXI_AWADDR = 8'hfc;
    end
    endtask

    task automatic send_w(input [31:0] data, input [3:0] strb,
                          input integer delay_cycles);
        integer n;
        reg accepted;
    begin
        repeat (delay_cycles) @(negedge bus_clk);
        @(negedge bus_clk); S_AXI_WDATA = data; S_AXI_WSTRB = strb; S_AXI_WVALID = 1;
        n = 0; accepted = 0;
        while (!accepted && n < LIMIT) begin
            @(posedge bus_clk); accepted = S_AXI_WREADY === 1'b1; n = n + 1;
        end
        if (!accepted) $fatal(1, "W transaction timeout");
        // Poison live payload after the handshake to verify latched data/strobes.
        @(negedge bus_clk); S_AXI_WVALID = 0; S_AXI_WDATA = ~data; S_AXI_WSTRB = ~strb;
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
        while (S_AXI_BVALID !== 1'b1 && n < LIMIT) begin
            @(negedge bus_clk); n = n + 1;
        end
        if (S_AXI_BVALID !== 1'b1) $fatal(1, "B transaction timeout");
        if (S_AXI_BRESP !== 2'b00) $fatal(1, "Expected OKAY write response");
    end
    endtask

    task automatic drain_b;
    begin
        @(negedge bus_clk); S_AXI_BREADY = 1;
        @(posedge bus_clk);
        if (S_AXI_BVALID !== 1'b1 || S_AXI_BRESP !== 2'b00)
            $fatal(1, "Missing response at B handshake");
        @(negedge bus_clk); S_AXI_BREADY = 0;
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
            if (S_AXI_BVALID !== 1'b1 || S_AXI_BRESP !== 2'b00)
                $fatal(1, "Unstable stalled B response");
        end
        drain_b;
        repeat (4) @(negedge bus_clk);
        if (S_AXI_BVALID !== 1'b0) $fatal(1, "Duplicate B response");
    end
    endtask

    task automatic read_csr(input [7:0] addr, output [31:0] value);
        integer n;
        reg accepted;
    begin
        @(negedge bus_clk); S_AXI_ARADDR = addr; S_AXI_ARVALID = 1;
        n = 0; accepted = 0;
        while (!accepted && n < LIMIT) begin
            @(posedge bus_clk); accepted = S_AXI_ARREADY === 1'b1; n = n + 1;
        end
        if (!accepted) $fatal(1, "AR transaction timeout");
        @(negedge bus_clk); S_AXI_ARVALID = 0;
        n = 0;
        while (S_AXI_RVALID !== 1'b1 && n < LIMIT) begin
            @(negedge bus_clk); n = n + 1;
        end
        if (S_AXI_RVALID !== 1'b1) $fatal(1, "R transaction timeout");
        if (S_AXI_RRESP !== 2'b00) $fatal(1, "Expected OKAY read response");
        value = S_AXI_RDATA; repeat ((rng & 7)+2) @(negedge bus_clk); S_AXI_RREADY = 1;
        @(posedge bus_clk);
        @(negedge bus_clk); S_AXI_RREADY = 0;
    end
    endtask

    task automatic axi_write(input [7:0] address,input [31:0] value);
        integer skew;
        begin
            skew=(rng & 7)-3;
            if(skew>0)total_aw_first=total_aw_first+1;
            if(skew<0)total_w_first=total_w_first+1;
            write_csr(address,value,4'hf,skew,2+(rng&7));
        end
    endtask
    task automatic axi_read(input [7:0] address,output [31:0] value);
        begin read_csr(address,value);end
    endtask
    always @(posedge bus_clk) if(aresetn) begin
        if(b_held)require(S_AXI_BVALID===1 && S_AXI_BRESP===held_br,"B stability");
        if(r_held)require(S_AXI_RVALID===1 && S_AXI_RDATA===held_rd && S_AXI_RRESP===held_rr,"R stability");
        b_held=S_AXI_BVALID&&!S_AXI_BREADY;held_br=S_AXI_BRESP;
        r_held=S_AXI_RVALID&&!S_AXI_RREADY;held_rr=S_AXI_RRESP;held_rd=S_AXI_RDATA;
        if(b_held)total_b_stalls=total_b_stalls+1;
        if(r_held)total_r_stalls=total_r_stalls+1;
        if(S_AXI_AWVALID&&S_AXI_AWREADY)aws=aws+1;
        if(S_AXI_WVALID&&S_AXI_WREADY)ws=ws+1;
        if(S_AXI_BVALID&&S_AXI_BREADY)bs=bs+1;
        if(S_AXI_ARVALID&&S_AXI_ARREADY)csr_ars=csr_ars+1;
        if(S_AXI_RVALID&&S_AXI_RREADY)csr_rs=csr_rs+1;
    end
    // Monitors sample pre-NBA handshake data. The FIFO synchronizers ensure
    // a read never depends on a new write at a coincident clock edge.
    always @(posedge bus_clk) begin
        if(aresetn && active) begin
            if(slave_r_held)require({M_AXI_RVALID,M_AXI_RID,M_AXI_RDATA,injected_rresp,M_AXI_RLAST}==={1'b1,saved_slave_r},"memory R stability");
            slave_r_held=M_AXI_RVALID&&!M_AXI_RREADY;
            saved_slave_r={M_AXI_RID,M_AXI_RDATA,injected_rresp,M_AXI_RLAST};
            if(slave_r_held)slave_r_stalls=slave_r_stalls+1;

            if(dut.start_pulse_bus) begin
                starts_bus=starts_bus+1; require(starts_bus==1,"duplicate bus start");
                $fdisplay(trace_file,"%0.3f,%0d,START_BUS,%0d",$realtime,job,dut.start_toggle_bus);
            end
            if(stalled) require({dut.stream_valid,dut.fifo_din_bus}==={1'b1,held_payload},"stalled FIFO input changed");
            stalled=dut.stream_valid && !dut.stream_ready;
            held_payload=dut.fifo_din_bus;
            if(stalled) stall_cycles=stall_cycles+1;
            if(dut.fifo_full_bus) full_cycles=full_cycles+1;
            if(ar_stalled)total_ar_stalls=total_ar_stalls+1;
            if(ar_stalled) require({M_AXI_ARVALID,M_AXI_ARID,M_AXI_ARADDR,M_AXI_ARLEN,M_AXI_ARSIZE,M_AXI_ARBURST}==={1'b1,held_ar_payload},"AR payload changed under stall");
            ar_stalled=M_AXI_ARVALID && !M_AXI_ARREADY;
            held_ar_payload={M_AXI_ARID,M_AXI_ARADDR,M_AXI_ARLEN,M_AXI_ARSIZE,M_AXI_ARBURST};
            // Process response completion before allocation if both happen together.
            if(M_AXI_RVALID && M_AXI_RREADY) begin
                require((^M_AXI_RID)!==1'bx,"unknown RID");
                require(live[M_AXI_RID],"response for unallocated ID");
                require(injected_rresp===(M_AXI_RLAST && (((fault_mask&1)!=0 && response_index[M_AXI_RID]==n-1) || ((fault_mask&2)!=0 && response_index[M_AXI_RID]==2*n-1)) ? 2'b10 : 2'b00),"RRESP injection");
                if(injected_rresp!=0)total_errors=total_errors+1;
                response_index[M_AXI_RID]=response_index[M_AXI_RID]+1;
                require(M_AXI_RLAST===(remaining[M_AXI_RID]==1),"RLAST/beat geometry mismatch");
                remaining[M_AXI_RID]=remaining[M_AXI_RID]-1;
                r_count=r_count+1;
                if(M_AXI_RLAST) begin  live[M_AXI_RID]=0; inflight=inflight-1; end
            end
            if(M_AXI_ARVALID && M_AXI_ARREADY) begin
                require((^M_AXI_ARID)!==1'bx && M_AXI_ARID<8,"invalid ARID");
                require(!live[M_AXI_ARID],"ID reused before completion");
                if(used[M_AXI_ARID]>0)total_reuses=total_reuses+1;
                used[M_AXI_ARID]=used[M_AXI_ARID]+1;
                response_index[M_AXI_ARID]=(ar_count/((n+15)/16))*n + (ar_count%((n+15)/16))*16;
                live[M_AXI_ARID]=1; remaining[M_AXI_ARID]=M_AXI_ARLEN+1;
                require(M_AXI_ARADDR===(((ar_count/((n+15)/16))==0 ? 32'hfc0 : 32'h2000) + 4*((ar_count%((n+15)/16))*16)),"independent AR address");
                require(M_AXI_ARSIZE===2 && M_AXI_ARBURST===1 && M_AXI_ARLEN===((n-(ar_count%((n+15)/16))*16)>16 ? 8'd15 : (n-(ar_count%((n+15)/16))*16)-1),"AR geometry");
                require((M_AXI_ARADDR&4095)+4*(M_AXI_ARLEN+1)<=4096,"4KB legal edge");
                inflight=inflight+1; if(inflight>peak_inflight)peak_inflight=inflight;
                require(inflight<=8,"too many outstanding reads");
                if(M_AXI_ARLEN<15)partial_bursts=partial_bursts+1;
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
            if(holding_result && !dut.start_pulse_mac)require(dut.result_mac===stable_result && dut.latency_mac===stable_latency,"bundled result hold stability");
            if(dut.start_pulse_mac) begin
                holding_result=0;
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
                holding_result=1;stable_result=dut.result_mac;stable_latency=dut.latency_mac;
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
            require((dut.busy_bus || dut.dma_busy)===1'b0,"new job while busy");
            require(dut.fifo_empty_mac===1'b1,"FIFO not drained before next job");
            job=job+1; n=len; golden_sum=0; wide_sum=0; fault_mask=(job%6==1 ? 2 : job%6==3 ? 1 : job%6==5 ? 3 : 0);fault_job=(fault_mask!=0);
            writes=0;reads=0;starts_bus=0;starts_mac=0;dones_mac=0;dones_bus=0;
            ar_count=0;r_count=0;inflight=0;peak_inflight=0;full_cycles=0;stall_cycles=0;
            wr_wraps=0;rd_wraps=0;stalled=0;ar_stalled=0;
            for(k=0;k<16;k=k+1)begin live[k]=0; remaining[k]=0;used[k]=0;response_index[k]=0;end
            for(k=0;k<n;k=k+1)begin
                rng=step(rng);golden_a[k]=rng[15:0];
                rng=step(rng);golden_b[k]=rng[15:0];
                if(job%3==0)begin golden_a[k]=-32768;golden_b[k]=-32768;end
                if(job%3==1 && k<4)begin golden_a[k]=-32768;golden_b[k]=32767;end
                wide_product=$signed(golden_a[k]);wide_product=wide_product*$signed(golden_b[k]);
                wide_sum=wide_sum+wide_product;
                u_mem.mem[1008+k]={16'hbeef,golden_a[k]};
                u_mem.mem[2048+k]={16'ha55a,golden_b[k]};
            end
            golden_sum=wide_sum[31:0];
            active=1; mem_latency=latency;ar_bp_mode=use_stalls;ooo_mode=1;
            axi_write(ADDR_SRC_A,32'hfc0);axi_write(ADDR_SRC_B,32'h2000);axi_write(ADDR_LENGTH,n);
            // Normal relevant partial CSR writes, unchanged value; reads overlap writes and B stalls.
            fork
                begin write_csr(ADDR_SRC_A,32'hffff00c0,1,4,5);total_partial=total_partial+1;end
                begin axi_read(ADDR_SRC_B,res);require(res===32'h2000,"concurrent CSR address ownership");end
            join
            write_csr(ADDR_CTRL,1,0,-4,5);total_partial=total_partial+1;
            require(starts_bus==0 && starts_mac==0,"zero CTRL start side effect");
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
            require(ctrl[3:1]===(fault_job ? 3'b110 : 3'b010),"CSR error/done/busy state wrong");
            require(lat===dut.lat_bus_reg && lat>0,"CSR latency mismatch");
            require(starts_bus==1 && starts_mac==1 && dones_mac==1 && dones_bus==1,"event conservation failure");
            require(ar_count==2*((n+15)/16) && r_count==2*n,"AR/R conservation failure");
            require(writes==n && reads==n,"FIFO conservation failure");
            require(dut.start_toggle_bus===(job%2!=0),"start toggle did not alternate across jobs");
            require(dut.done_toggle_mac===(job%2!=0),"done toggle did not alternate across jobs");
            require(peak_inflight==1,"V2 single outstanding");
            $display("CDC_JOB_PASS job=%0d len=%0d result=%0d writes=%0d reads=%0d ar=%0d r=%0d peak=%0d full=%0d stalls=%0d wr_wraps=%0d rd_wraps=%0d toggle=%0d",job,n,$signed(res),writes,reads,ar_count,r_count,peak_inflight,full_cycles,stall_cycles,wr_wraps,rd_wraps,dut.start_toggle_bus);
            total_full=total_full+full_cycles;total_stalls=total_stalls+stall_cycles;
            passes=passes+1;
        end
    endtask

    initial begin
        arg_seed=$value$plusargs("SEED=%d",seed);rng=seed;require(seed>0,"nonzero seed");
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
        $display("CLOSURE_CONFIG version=2 bus_period=10 mac_period=%0.3f mac_phase=%0.3f bus_reset_first=%0d MAX_OUT=1",mac_period,mac_phase,reset_bus_first);
        run_job(1,0,1);run_job(15,30,1);run_job(16,30,1);run_job(17,30,1);
        run_job(31,40,1);run_job(32,40,1);run_job(33,40,1);run_job(127,100,1);
        run_job(129,100,1);run_job(255,100,1);run_job(256,100,1);run_job(1,0,1);
        // Keep the last job's monitors active to detect delayed extra events.
        repeat(20)@(posedge bus_clk);repeat(20)@(posedge mac_clk);
        require(starts_bus==1 && starts_mac==1 && dones_mac==1 && dones_bus==1,"late duplicate event");
        require(passes==12,"missing jobs");
        require(partial_bursts>0 && total_reuses>0,"partial burst / ID reuse not exercised");
        require(total_errors==8,"final A/B beat error injections not exercised");
        require(aws==ws && ws==bs && csr_ars==csr_rs,"CSR transaction conservation");
        require(total_aw_first>0 && total_w_first>0 && total_b_stalls>0 && total_r_stalls>0 && total_ar_stalls>0,"protocol stress vacuous");
        if(mac_period>=17)require(total_full>0 && total_stalls>0,"FIFO full/backpressure never triggered");
        $display("CLOSURE_EDGE_COUNTERS partial_bursts=%0d slave_r_stalls=%0d",partial_bursts,slave_r_stalls);
        $display("CLOSURE_COUNTERS seed=%0d jobs=%0d errors=%0d full=%0d fifo_stalls=%0d ar_stalls=%0d reuse=%0d nonhead=%0d aw_first=%0d w_first=%0d b_stalls=%0d r_stalls=%0d partial=%0d AW=%0d W=%0d B=%0d AR=%0d R=%0d",seed,passes,total_errors,total_full,total_stalls,total_ar_stalls,total_reuses,total_nonhead,total_aw_first,total_w_first,total_b_stalls,total_r_stalls,total_partial,aws,ws,bs,csr_ars,csr_rs);
        $display("TOTAL: 12 PASS / 0 FAIL; startup_resets=1; interjob_resets=0");
        $fclose(trace_file);$finish;
    end
    initial begin
        #20000000;$fatal(1,"CDC_FAIL global timeout");
    end
endmodule
