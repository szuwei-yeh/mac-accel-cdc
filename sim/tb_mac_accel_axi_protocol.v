`timescale 1ns/1ps

// V1 read/write port arbitration and AXI-Lite response stability.
// Drive on negedges and count handshakes on posedges, with independent AW/W.
// Non-operational LENGTH bit patterns test storage only and are never started.
module tb_mac_accel_axi_protocol;
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

    integer aws=0,ws=0,bs=0,ars=0,rs=0,starts=0,dones=0,pass_cnt=0,read_write_waits=0;
    reg r_stalled=0,b_stalled=0;
    reg [31:0] held_data;
    reg [1:0] held_rr,held_br;
    always @(posedge bus_clk)if(resetn)begin
        if(r_stalled && (!rvalid || rdata!==held_data || rresp!==held_rr))$fatal(1,"R stalled response changed");
        if(b_stalled && (!bvalid || bresp!==held_br))$fatal(1,"B stalled response changed");
        r_stalled=rvalid&&!rready;held_data=rdata;held_rr=rresp;
        b_stalled=bvalid&&!bready;held_br=bresp;
        if(dut.rd_pending && dut.core_we)read_write_waits=read_write_waits+1;
        if(awvalid&&awready)aws=aws+1;
        if(wvalid&&wready)ws=ws+1;
        if(bvalid&&bready)bs=bs+1;
        if(arvalid&&arready)ars=ars+1;
        if(rvalid&&rready)rs=rs+1;
    end
    always @(posedge mac_clk)if(!mac_rst)begin
        if(dut.u_mac_accel.start_pulse_mac)starts=starts+1;
        if(dut.u_mac_accel.done_mac)dones=dones+1;
    end
    task read_check(input[7:0]addr,input[31:0]expected);
        reg[31:0]value;
        begin read_csr(addr,value);if(value!==expected)$fatal(1,"read address=%h got=%h expected=%h",addr,value,expected);end
    endtask
    task pass(input[511:0]name);
        begin pass_cnt=pass_cnt+1;$display("PASS: %0s",name);end
    endtask
    task job(input integer pattern,input integer expected);
        integer i,n;
    begin
        write_csr(8'h0c,15,4'hf,0,0);
        for(i=0;i<4;i=i+1)write_csr(4,pattern==0 ? i+1 : (pattern==1 ? -i-1 : 3),4'hf,0,0);
        for(i=0;i<4;i=i+1)write_csr(8,pattern==0 ? (i+1)*10 : (pattern==1 ? i+1 : 3),4'hf,0,0);
        write_csr(0,1,4'hf,0,0);
        n=0;@(negedge bus_clk);
        while(!dut.done_led && n<2000)begin @(negedge bus_clk);n=n+1;end
        if(!dut.done_led)$fatal(1,"wrapper job timeout");
        read_check(8'h10,expected);read_check(0,4);
    end
    endtask
    // Held R response, with writes accepted and completed while RREADY remains low.
    task blocked_read_with_write;
        integer n;
        reg accepted;
    begin
        @(negedge bus_clk);araddr=8'h10;arvalid=1;accepted=0;n=0;
        while(!accepted && n<LIMIT)begin @(posedge bus_clk);accepted=arready;n=n+1;end
        if(!accepted)$fatal(1,"blocked AR timeout");
        @(negedge bus_clk);arvalid=0;n=0;
        while(!rvalid && n<LIMIT)begin @(negedge bus_clk);n=n+1;end
        if(!rvalid||rdata!==300)$fatal(1,"blocked result read");
        write_csr(8'h0c,15,4'hf,0,4);write_csr(4,7,4'hf,0,3);
        repeat(8)@(negedge bus_clk);
        if(!rvalid||rdata!==300)$fatal(1,"writes corrupted held result");
        rready=1;@(posedge bus_clk);@(negedge bus_clk);rready=0;
    end
    endtask
    reg[31:0]value,latency;
    integer i;
    initial begin
        repeat(8)@(negedge bus_clk);resetn=1;mac_rst=0;
        write_csr(8'h0c,15,4'hf,0,0);read_check(0,0);pass("01 read after write uses AR address");
        read_check(8'h0c,32'hdeadbeef);read_check(8'h3c,32'hdeadbeef);pass("02 unmapped reads");
        for(i=0;i<4;i=i+1)begin read_check(0,0);read_check(8'h10,0);read_check(8'h14,0);read_check(8'h0c,32'hdeadbeef);end
        pass("03 alternating idle CSR addresses");
        job(0,300);read_csr(8'h14,latency);if(latency==0)$fatal(1,"missing latency");
        for(i=0;i<4;i=i+1)begin read_check(0,4);read_check(8'h10,300);read_check(8'h14,latency);read_check(8'h0c,32'hdeadbeef);end
        pass("04 alternating completed CSR addresses");
        fork write_csr(8'h0c,15,4'hf,0,6);begin @(negedge bus_clk);read_check(0,4);end join
        fork write_csr(4,7,4'hf,2,6);read_check(8'h10,300);join
        if(dut.u_mac_accel.vecA[0]!==16'd7 || dut.u_mac_accel.load_idx_a!==1)$fatal(1,"read interfered with core write");
        if(read_write_waits==0)$fatal(1,"core write/read arbitration not exercised");
        pass("05 simultaneous read/write and B stalls");
        blocked_read_with_write;pass("06 held R while writes progress");
        job(1,-30);pass("07 signed job after concurrent traffic");
        job(2,36);repeat(12)@(negedge bus_clk);
        if(starts!=3||dones!=3||aws!=ws||ws!=bs||ars!=rs)$fatal(1,"AXI/event conservation");
        pass("08 three wrapper jobs without reset");
        $display("COUNTS AW=%0d W=%0d B=%0d AR=%0d R=%0d start=%0d done=%0d",aws,ws,bs,ars,rs,starts,dones);
        $display("TOTAL: %0d PASS / 0 FAIL",pass_cnt);$finish;
    end
    initial begin #300000;$fatal(1,"global V1 AXI timeout");end
endmodule
