`timescale 1ns/1ps
// A completed DMA read phase may still own a stalled output skid beat.
// A subsequent idle start must not destroy that beat: V2 already retains it.
module tb_mac_dma_pending_tail;
    reg clk=0,rst=1,start=0,ready=0;
    always #5 clk=~clk;
    wire busy,done,err,arvalid,rready,valid,last;
    wire[3:0] arid;
    wire[31:0] araddr;
    wire[7:0] arlen;
    wire[2:0] arsize;
    wire[1:0] arburst;
    reg[3:0] rid=0;
    reg[31:0] rdata=0;
    reg rvalid=0;
    reg[1:0] rresp=0;
    wire signed[15:0] a,b;
    integer generation=0,ars=0,rs=0,outs=0,dones=0,holds=0;
    reg stalled=0;
    reg[32:0] saved;
    mac_dma dut(
        .clk(clk),.rst(rst),.start(start),.src_a_addr(32'h100),.src_b_addr(32'h200),.length(16'd1),
        .dma_busy(busy),.dma_done(done),.dma_err(err),
        .M_AXI_ARID(arid),.M_AXI_ARADDR(araddr),.M_AXI_ARLEN(arlen),.M_AXI_ARSIZE(arsize),
        .M_AXI_ARBURST(arburst),.M_AXI_ARVALID(arvalid),.M_AXI_ARREADY(1'b1),
        .M_AXI_RID(rid),.M_AXI_RDATA(rdata),.M_AXI_RRESP(rresp),.M_AXI_RLAST(1'b1),
        .M_AXI_RVALID(rvalid),.M_AXI_RREADY(rready),
        .stream_valid(valid),.stream_ready(ready),.stream_last(last),.stream_a(a),.stream_b(b));
    always @(posedge clk) begin
        if(rst)rvalid<=0;
        else begin
            if(rvalid&&rready)begin rvalid<=0;rs=rs+1;end
            if(arvalid)begin
                if(arlen!==0 || arsize!==2 || arburst!==1 ||
                   araddr!==(ars%2==0 ? 32'h100 : 32'h200))$fatal(1,"AR geometry/order");
                ars=ars+1;rid<=arid;
                rdata<=araddr==32'h100 ? generation*2 : generation*2+1;
                rresp<=generation==1 && araddr==32'h200 ? 2'b10 : 2'b00;
                rvalid<=1;
            end
            if(stalled && (!valid || {last,a,b}!==saved))$fatal(1,"pending final beat discarded by idle start");
            stalled=valid&&!ready;saved={last,a,b};
            if(stalled)holds=holds+1;
            if(valid&&ready)begin
                if(!last || a!==(outs+1)*2 || b!==(outs+1)*2+1)$fatal(1,"output job/order");
                outs=outs+1;
            end
            if(done)begin dones=dones+1;$display("DMA_TAIL_DONE generation=%0d outputs=%0d pending=%b dma_err=%b",generation,outs,valid,err);end
        end
    end
    task launch;
        begin @(negedge clk);generation=generation+1;start=1;@(negedge clk);start=0;end
    endtask
    task wait_idle;
        integer n;
        begin n=0;while(busy!==0 || dones<generation)begin @(negedge clk);n=n+1;if(n>200)$fatal(1,"DMA completion timeout");end end
    endtask
    integer n;
    initial begin
        repeat(8)@(negedge clk);rst=0;launch;wait_idle;
        if(!valid || {last,a,b}!=={1'b1,16'd2,16'd3} || err!==1)$fatal(1,"stalled final/error fixture");
        launch;
        // Keep the old tail stalled while the next job's A phase progresses.
        repeat(30)@(negedge clk);
        if(!valid || outs!=0 || holds<20)$fatal(1,"tail hold coverage");
        ready=1;wait_idle;
        n=0;while(outs!=2)begin @(negedge clk);n=n+1;if(n>200)$fatal(1,"tail drain timeout");end
        repeat(8)@(negedge clk);
        if(ars!=4 || rs!=4 || dones!=2 || err!==0 || valid)$fatal(1,"job/error/transaction conservation");
        $display("DMA_PENDING_TAIL_PASS jobs=2 AR=%0d R=%0d OUT=%0d dones=%0d held_cycles=%0d",ars,rs,outs,dones,holds);
        $display("TOTAL: 2 PASS / 0 FAIL");$finish;
    end
    initial begin #100000;$fatal(1,"pending tail global timeout");end
endmodule
