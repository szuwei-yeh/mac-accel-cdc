`timescale 1ns/1ps

// V1 read/write port arbitration and AXI-Lite response stability.
// Drive on negedges and count handshakes on posedges, with independent AW/W.
// Non-operational LENGTH bit patterns test storage only and are never started.
module tb_mac_accel_axi_wstrb;
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

    integer cases=0, fails=0, starts=0, dones=0, aws=0, ws=0, bs=0;
    always @(posedge bus_clk) if(resetn) begin
        if(awvalid&&awready)aws=aws+1;
        if(wvalid&&wready)ws=ws+1;
        if(bvalid&&bready)bs=bs+1;
    end
    always @(posedge mac_clk) if(!mac_rst) begin
        if(dut.u_mac_accel.start_pulse_mac)starts=starts+1;
        if(dut.u_mac_accel.done_mac)dones=dones+1;
    end
    task check(input condition, input [511:0] text);
        begin cases=cases+1;
        if(condition!==1'b1)begin fails=fails+1;$display("FAIL: %0s",text);end
        else $display("PASS: %0s",text);end
    endtask
    reg signed[15:0] a[0:3],b[0:3];
    reg signed[63:0] sum, product;
    reg[31:0] value;
    integer i,j,n;
    initial begin
        repeat(8)@(negedge bus_clk); resetn=1; mac_rst=0;
        write_csr(12,15,15,0,0);
        for(i=0;i<4;i=i+1) begin
            a[i]=16'h8123+i; b[i]=16'h4567+i;
            write_csr(4,a[i],15,0,0);write_csr(8,b[i],15,0,0);
        end
        write_csr(12,0,0,3,5);
        check(dut.u_mac_accel.update_mask===4'hf && dut.u_mac_accel.load_idx_a===4 && dut.u_mac_accel.load_idx_b===4,"zero MASK preserves state and indices");
        write_csr(12,0,4'hc,-3,4);
        check(dut.u_mac_accel.update_mask===4'hf && dut.u_mac_accel.load_idx_a===4,"irrelevant MASK bytes no effects");
        write_csr(12,15,1,0,0);
        write_csr(4,0,0,3,3);write_csr(8,0,0,-3,3);
        check(dut.u_mac_accel.load_idx_a===0 && dut.u_mac_accel.load_idx_b===0 && dut.u_mac_accel.vecA[0]===a[0] && dut.u_mac_accel.vecB[0]===b[0],"zero operands preserve payload and indices");
        write_csr(4,0,4'hc,0,0);write_csr(8,0,4'h8,0,0);
        check(dut.u_mac_accel.load_idx_a===0 && dut.u_mac_accel.load_idx_b===0,"irrelevant operand bytes no effects");
        // Each relevant partial write consumes one element, preserving its old disabled byte.
        for(i=0;i<4;i=i+1)begin
            write_csr(4,32'hfefe00a0+i,1,i%2==0 ? 4 : -4,i);
            a[i]={a[i][15:8],8'ha0+i[7:0]};
            write_csr(8,32'habcdde00,2,i%2==0 ? -4 : 4,i);
            b[i]={8'hde,b[i][7:0]};
            check(dut.u_mac_accel.vecA[i]===a[i] && dut.u_mac_accel.vecB[i]===b[i],"partial operand disabled bytes preserved");
        end
        write_csr(0,1,0,0,3);write_csr(0,1,8,0,3);
        repeat(10)@(negedge bus_clk);
        check(starts==0 && dones==0,"CTRL requires low byte");
        for(j=0;j<4;j=j+1)begin
            if(j!=0)begin
                write_csr(12,15,1,0,0);
                for(i=0;i<4;i=i+1)begin
                    case(j)
                      1:begin a[i]=-32768;b[i]=-32768;end
                      2:begin a[i]=32767;b[i]=-32768;end
                      3:begin a[i]=i-2;b[i]=3-i;end
                    endcase
                    write_csr(4,a[i],3,0,0);write_csr(8,b[i],3,0,0);
                end
            end
            sum=0;
            for(i=0;i<4;i=i+1)begin product=$signed(a[i]);product=product*$signed(b[i]);sum=sum+product;end
            write_csr(0,1,1,0,4);
            n=0;while(!dut.done_led && n<2000)begin @(negedge bus_clk);n=n+1;end
            if(!dut.done_led)$fatal(1,"job timeout");
            read_csr(16,value);
            check(value===sum[31:0] && starts==j+1 && dones==j+1,"independent signed modulo-32 result and events");
            write_csr(0,1,0,0,2);read_csr(0,value);
            check(value===4,"zero CTRL after completion preserves DONE");
        end
        check(aws==ws && ws==bs,"AW/W/B conservation");
        $display("V1_WSTRB_COUNTS jobs=%0d starts=%0d dones=%0d AW=%0d W=%0d B=%0d",4,starts,dones,aws,ws,bs);
        $display("TOTAL: %0d PASS / %0d FAIL",cases-fails,fails);
        if(fails)$fatal(1,"V1 byte enables failed");$finish;
    end
    initial begin #2000000;$fatal(1,"global V1 WSTRB timeout");end
endmodule
