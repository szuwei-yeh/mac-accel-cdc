`timescale 1ns/1ps

// V3 AXI-Lite stress: independent AW/W, concurrent reads/writes, held VALID and B/R stalls.
// Drive on negedges and count handshakes on posedges, with independent AW/W.
// Independent external handshake scoreboards; bounded transactions and process timeout.
module tb_mac_accel_dma_rob_axil_stream;
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
        .M_AXI_ARREADY(1'b0),.M_AXI_RVALID(1'b0),.M_AXI_RID(4'b0),.M_AXI_RDATA(32'b0),.M_AXI_RLAST(1'b0),.M_AXI_RRESP(2'b0),
        .mac_clk(mac_clk), .mac_rst(mac_rst)
    );

    localparam N=256;
    integer ac=0,wc=0,bc=0,arc=0,rc=0,ticks=0,phase=0;
    reg [7:0] aq[0:N-1],rq[0:N-1];
    reg [31:0] dq[0:N-1], model[0:3];reg[3:0] sq[0:N-1];
    reg stalled_b=0,stalled_r=0;reg[1:0]saved_b;reg[31:0]saved_r;
    integer lane,word_index,commits=0;reg prev_b=0,prev_r=0;integer rcreated=0;reg[31:0]read_expected[0:N-1];
    always@(posedge bus_clk)begin
        if(!resetn)begin ac=0;wc=0;bc=0;arc=0;rc=0;ticks=0;stalled_b=0;stalled_r=0;commits=0;prev_b=0;prev_r=0;rcreated=0;end
        else begin
            ticks=ticks+1;
            if(stalled_b&&(bvalid!==1||bresp!==saved_b))$fatal(1,"B stability");
            if(stalled_r&&(rvalid!==1||rdata!==saved_r||rresp!==0))$fatal(1,"R stability");
            stalled_b=bvalid&&!bready;saved_b=bresp;
            stalled_r=rvalid&&!rready;saved_r=rdata;
            if(awvalid&&awready)begin if(ac>=N)$fatal(1,"extra AW");aq[ac]=awaddr;ac=ac+1;end
            if(wvalid&&wready)begin if(wc>=N)$fatal(1,"extra W");dq[wc]=wdata;sq[wc]=wstrb;wc=wc+1;end
            if(bvalid&&bready)begin
                if(bc>=ac||bc>=wc||bresp!==0)$fatal(1,"B response before actual AW/W or wrong ordering");
                bc=bc+1;
            end
            if(arvalid&&arready)begin if(arc>=N)$fatal(1,"extra AR");rq[arc]=araddr;arc=arc+1;end
            if(rvalid&&rready)begin
                if(rc>=arc||rresp!==0||rc>=rcreated||rdata!==read_expected[rc])$fatal(1,"R data/order/external acceptance mismatch");
                rc=rc+1;
            end
            #0.001;
            if(rvalid&&!prev_r)begin
                if(rcreated>=arc)$fatal(1,"R response lacks accepted AR ownership");
                word_index=rq[rcreated][5:2];
                read_expected[rcreated]=(word_index>=1 && word_index<=3) ? model[word_index] : 32'hdeadbeef;
                if(rdata!==read_expected[rcreated])$fatal(1,"read uses wrong saved AR/state snapshot");
                rcreated=rcreated+1;
            end
            prev_r=rvalid;
            // Model writes only from external AW/W queues and each new B response.
            if(bvalid&&!prev_b)begin
                if(commits>=ac||commits>=wc)$fatal(1,"write response lacks accepted ownership");
                word_index=aq[commits][5:2];
                if(word_index>=1&&word_index<=3)
                    for(lane=0;lane<4;lane=lane+1)
                        if(sq[commits][lane] && (word_index!=3 || lane<2))
                            model[word_index][lane*8+:8]=dq[commits][lane*8+:8];
                commits=commits+1;
            end
            prev_b=bvalid;
            if(dut.reg_src_a!==model[1] || dut.reg_src_b!==model[2] || {16'b0,dut.reg_length}!==model[3])$fatal(1,"external write sequence/byte model differs from CSR state");
            if(dut.start_pulse_bus || dut.start_toggle_bus)$fatal(1,"non-CTRL traffic started job");
        end
    end
    task drive_aw(input integer mode);
        integer i,gap;
    begin
        i=0;@(negedge bus_clk);awvalid=1;awaddr=8'h04;
        while(i<N)begin
            do @(posedge bus_clk);while(awready!==1);
            i=i+1;@(negedge bus_clk);
            gap=(mode==1||mode==3)?(i%4):0;
            if(gap>0)begin awvalid=0;repeat(gap)@(negedge bus_clk);end
            if(i<N)begin awaddr=(i%4==0 ? 8'h04 : i%4==1 ? 8'h08 : i%4==2 ? 8'h0c : 8'h3c);awvalid=1;end else awvalid=0;
        end
    end endtask
    task drive_w(input integer mode);
        integer i,gap;
    begin
        i=0;@(negedge bus_clk);wvalid=1;wdata=32'ha55a3344;wstrb=4'hf;
        while(i<N)begin
            do @(posedge bus_clk);while(wready!==1);
            i=i+1;@(negedge bus_clk);
            gap=(mode==2||mode==3)?((i+1)%5):0;
            if(gap>0)begin wvalid=0;repeat(gap)@(negedge bus_clk);end
            if(i<N)begin wdata=32'ha55a3344 ^ (32'h01020305*i);wstrb=i%16;wvalid=1;end else wvalid=0;
        end
    end endtask
    task consume_b;
    begin
        while(bc<N)begin @(negedge bus_clk);bready=(ticks%17>=9);end
        @(negedge bus_clk);bready=0;
    end endtask
    task drive_ar;
        integer i;
    begin
        i=0;@(negedge bus_clk);arvalid=1;araddr=4;
        while(i<N)begin
            do @(posedge bus_clk);while(arready!==1);
            i=i+1;@(negedge bus_clk);
            if(i<N)begin araddr=(i%4==0 ? 8'h04 : i%4==1 ? 8'h08 : i%4==2 ? 8'h0c : 8'h3c);arvalid=1;end else arvalid=0;
        end
    end endtask
    task consume_r;
    begin
        while(rc<N)begin @(negedge bus_clk);rready=(ticks%13>=7);end
        @(negedge bus_clk);rready=0;
    end endtask
    integer k;
    initial begin
        for(phase=0;phase<4;phase=phase+1)begin
            @(negedge bus_clk);resetn=0;mac_rst=1;awvalid=0;wvalid=0;bready=0;arvalid=0;rready=0;
            for(k=0;k<4;k=k+1)model[k]=0;
            repeat(8)@(negedge bus_clk);resetn=1;mac_rst=0;
            fork drive_aw(phase);drive_w(phase);consume_b;drive_ar;consume_r;join
            repeat(4)@(negedge bus_clk);
            if(ac!=N||wc!=N||bc!=N||arc!=N||rc!=N||commits!=N||rcreated!=N||bvalid||rvalid)$fatal(1,"transaction conservation");
            $display("AXIL_STREAM_PASS mode=%0d AW=%0d W=%0d B=%0d AR=%0d R=%0d",phase,ac,wc,bc,arc,rc);
        end
        $display("TOTAL: 4 PASS / 0 FAIL; writes=1024 reads=1024");$finish;
    end
    initial begin #2000000;$fatal(1,"probe timeout");end
endmodule
