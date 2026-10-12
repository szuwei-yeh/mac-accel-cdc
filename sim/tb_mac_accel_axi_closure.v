`timescale 1ns/1ps

// V1 read/write port arbitration and AXI-Lite response stability.
// Drive on negedges and count handshakes on posedges, with independent AW/W.
// Independent external handshake scoreboards; bounded transactions and process timeout.
module tb_mac_accel_axi_closure;
    localparam LIMIT = 80;
    reg bus_clk = 0, mac_clk = 0;
    always #5 bus_clk = ~bus_clk;
    real mac_period=7.5;
    integer clock_arg;
    initial begin clock_arg=$value$plusargs("MAC_PERIOD=%f",mac_period);if(mac_period<=0)$fatal(1,"clock period");forever #(mac_period/2)mac_clk=~mac_clk;end
    reg resetn = 0, mac_rst = 1;
    reg [7:0] awaddr = 0, araddr = 0;
    reg awvalid = 0, wvalid = 0, bready = 0;
    reg arvalid = 0, rready = 0;
    reg [31:0] wdata = 0;
    reg [3:0] wstrb = 0;
    wire awready, wready, bvalid, arready, rvalid;
    wire [1:0] bresp, rresp;
    wire [31:0] rdata;

    mac_accel_axi #(.VEC_LEN(4)) dut (
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
        .mac_clk(mac_clk), .mac_rst(mac_rst)
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

    integer seed=1,rng,arg_seed,j,i,lane,n;
    function[31:0]step(input[31:0]x);reg[31:0]t;begin t=x^(x<<13);t=t^(t>>17);step=t^(t<<5);end endfunction
    reg[15:0]a[0:3],b[0:3];reg[31:0]data,value;reg[3:0]strb,mask;
    reg signed[63:0]sum,product,aa,bb;
    integer starts=0,dones=0,aws=0,ws=0,bs=0,ars=0,rs=0,pops=0;
    integer aw_first=0,w_first=0,partial=0,zero=0,br_stalls=0,rr_stalls=0;
    reg stalled_b=0,stalled_r=0,holding_result=0;
    reg[1:0]saved_b,saved_rr;reg[31:0]saved_r,held_result,held_latency;
    always @(posedge bus_clk) if(resetn)begin
        if(stalled_b && (!bvalid || bresp!==saved_b))$fatal(1,"B stability");
        if(stalled_r && (!rvalid || rdata!==saved_r || rresp!==saved_rr))$fatal(1,"R stability");
        stalled_b=bvalid&&!bready;saved_b=bresp;stalled_r=rvalid&&!rready;saved_r=rdata;saved_rr=rresp;
        if(stalled_b)br_stalls=br_stalls+1;if(stalled_r)rr_stalls=rr_stalls+1;
        if(awvalid&&awready)aws=aws+1;if(wvalid&&wready)ws=ws+1;if(bvalid&&bready)bs=bs+1;
        if(arvalid&&arready)ars=ars+1;if(rvalid&&rready)rs=rs+1;
    end
    always @(posedge mac_clk) if(!mac_rst)begin
        if(dut.u_mac_accel.start_pulse_mac)begin starts=starts+1;holding_result=0;end
        if(dut.u_mac_accel.fifo_rd_en_mac && !dut.u_mac_accel.fifo_empty_mac)begin
            if(pops>=4 || dut.u_mac_accel.fifo_dout_mac!=={(pops==3),a[pops],b[pops]})$fatal(1,"V1 FIFO payload order/last");
            pops=pops+1;
        end
        if(holding_result && (dut.u_mac_accel.result_mac!==held_result || dut.u_mac_accel.latency_mac!==held_latency))$fatal(1,"bundled result changed before next source start");
        if(dut.u_mac_accel.done_mac)begin
            if(pops!=4 || dut.u_mac_accel.result_mac!==sum[31:0])$fatal(1,"early completion/result");
            dones=dones+1;holding_result=1;held_result=dut.u_mac_accel.result_mac;held_latency=dut.u_mac_accel.latency_mac;
        end
    end
    task put(input[7:0]addr,input[31:0]payload,input[3:0]en);
        integer skew;begin skew=(rng&7)-3;if(skew>0)aw_first=aw_first+1;if(skew<0)w_first=w_first+1;
        write_csr(addr,payload,en,skew,(rng>>4)&7);end
    endtask
    initial begin
        arg_seed=$value$plusargs("SEED=%d",seed);if(seed<=0)$fatal(1,"seed must be positive");rng=seed;
        for(i=0;i<4;i=i+1)begin a[i]=0;b[i]=0;end
        repeat(8)@(negedge bus_clk);resetn=1;mac_rst=0;
        for(j=0;j<12;j=j+1)begin
            pops=0;mask=j%4==0 ? 15 : j%4==1 ? 5 : j%4==2 ? 10 : 0;
            put(12,mask,1);
            for(i=0;i<4;i=i+1)begin
                rng=step(rng);data=rng;strb=i%3==0 ? 1 : i%3==1 ? 2 : 3;
                put(4,0,0);zero=zero+1;
                if(dut.u_mac_accel.load_idx_a!==i)$fatal(1,"zero A advanced index");
                put(4,data,strb);partial=partial+1;
                for(lane=0;lane<2;lane=lane+1)if(mask[i]&&strb[lane])a[i][lane*8+:8]=data[lane*8+:8];
                rng=step(rng);data=rng;strb=i%3==0 ? 2 : i%3==1 ? 3 : 1;
                put(8,data,strb);partial=partial+1;
                for(lane=0;lane<2;lane=lane+1)if(mask[i]&&strb[lane])b[i][lane*8+:8]=data[lane*8+:8];
            end
            sum=0;for(i=0;i<4;i=i+1)begin aa=$signed(a[i]);bb=$signed(b[i]);product=aa*bb;sum=sum+product;end
            // R and B can stall independently while a no-op write completes.
            fork put(0,1,0);begin
                @(negedge bus_clk);araddr=16;arvalid=1;n=0;
                do begin @(posedge bus_clk);n=n+1;if(n>80)$fatal(1,"AR timeout");end while(!arready);
                @(negedge bus_clk);arvalid=0;n=0;
                while(!rvalid)begin @(negedge bus_clk);n=n+1;if(n>80)$fatal(1,"R timeout");end
                repeat(9)@(negedge bus_clk);rready=1;@(posedge bus_clk);@(negedge bus_clk);rready=0;
            end join
            put(0,1,1);n=0;
            while(!dut.done_led && n<2000)begin @(negedge bus_clk);n=n+1;end
            if(!dut.done_led)$fatal(1,"completion timeout");
            read_csr(16,value);if(value!==sum[31:0])$fatal(1,"CSR signed result");
            read_csr(0,value);if(value!==4 || starts!=j+1 || dones!=j+1)$fatal(1,"status/event ownership");
            if(dut.u_mac_accel.start_toggle_bus!==(j%2==0) || dut.u_mac_accel.done_toggle_mac!==(j%2==0))$fatal(1,"toggle polarity");
            $display("V1_CLOSURE_JOB seed=%0d job=%0d mask=%0h result=%0d pops=%0d",seed,j+1,mask,$signed(sum[31:0]),pops);
        end
        repeat(20)@(negedge bus_clk);
        if(aws!=ws || ws!=bs || ars!=rs || starts!=12 || dones!=12 || aw_first==0 || w_first==0 || br_stalls==0 || rr_stalls==0)$fatal(1,"conservation/coverage");
        $display("V1_CLOSURE_COUNTERS seed=%0d jobs=12 starts=%0d dones=%0d AW=%0d W=%0d B=%0d AR=%0d R=%0d partial=%0d zero=%0d aw_first=%0d w_first=%0d b_stalls=%0d r_stalls=%0d",seed,starts,dones,aws,ws,bs,ars,rs,partial,zero,aw_first,w_first,br_stalls,rr_stalls);
        $display("TOTAL: 12 PASS / 0 FAIL");$finish;
    end
    initial begin #2000000;$fatal(1,"V1 closure timeout");end
endmodule
