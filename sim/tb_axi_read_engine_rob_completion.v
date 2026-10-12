`timescale 1ns/1ps
// Deterministic completion order 7..0 with output blocked, then ID reuse 1,0.
// Burst-completion coverage is counted independently from per-ID AR sequence.
// Designated case requires a positive reorder count; interleaving is irrelevant.
module tb_axi_read_engine_rob_completion;
reg clk=0,rst=1,cmd=0,ready=0;always #5 clk=~clk;
reg[3:0]rid=0;reg[31:0]rd=0;reg rv=0,rl=0;
wire cr,arv,rr,ov,ol,busy,done,err;wire[3:0]aid;wire[31:0]addr,od;wire[7:0]len;wire[2:0]size;wire[1:0]burst;
axi_read_engine_rob #(.MAX_OUTSTANDING(8),.MAX_BURST_LEN(2)) dut(.clk(clk),.rst(rst),.cmd_valid(cmd),.cmd_ready(cr),.cmd_addr(32'd0),.cmd_len(16'd20),.M_AXI_ARID(aid),.M_AXI_ARADDR(addr),.M_AXI_ARLEN(len),.M_AXI_ARSIZE(size),.M_AXI_ARBURST(burst),.M_AXI_ARVALID(arv),.M_AXI_ARREADY(1'b1),.M_AXI_RID(rid),.M_AXI_RDATA(rd),.M_AXI_RRESP(2'b0),.M_AXI_RLAST(rl),.M_AXI_RVALID(rv),.M_AXI_RREADY(rr),.out_valid(ov),.out_ready(ready),.out_data(od),.out_last(ol),.engine_busy(busy),.engine_done(done),.engine_err(err));
integer ars=0,rs=0,outs=0,n,i;
integer completed=0,ooo_complete_cnt=0,oldest_incomplete=0,seq;
integer tag_sequence[0:15];reg completion_seen[0:9];
always@(posedge clk)if(!rst)begin
 if(arv)begin if(addr!==ars*8||aid!==ars%8||len!==1)$fatal(1,"AR geometry/tag");tag_sequence[aid]=ars;ars=ars+1;end
 if(rv&&rr)begin
  rs=rs+1;
  if(rl)begin
   seq=tag_sequence[rid];
   if(completion_seen[seq])$fatal(1,"duplicate burst completion");
   if(seq!=oldest_incomplete)ooo_complete_cnt=ooo_complete_cnt+1;
   completion_seen[seq]=1;completed=completed+1;
   while(oldest_incomplete<10 && completion_seen[oldest_incomplete])oldest_incomplete=oldest_incomplete+1;
  end
 end
 if(ov&&ready)begin if(od!==(32'ha5000000|outs)||ol!==(outs==19))$fatal(1,"full-word ordered retirement");outs=outs+1;end
end
task response(input integer id,input integer element);
 integer k,t;
 begin
 for(k=0;k<2;k=k+1)begin
  @(negedge clk);rid=id;rd=32'ha5000000|(element+k);rl=k==1;rv=1;
  t=0;do begin @(posedge clk);t=t+1;if(t>1000)$fatal(1,"R timeout");end while(!rr);
 end
 @(negedge clk);rv=0;rl=0;
 end
endtask
initial begin
 for(i=0;i<10;i=i+1)completion_seen[i]=0;
 repeat(8)@(negedge clk);rst=0;cmd=1;@(negedge clk);cmd=0;
 n=0;while(ars<8&&n<1000)begin @(negedge clk);n=n+1;end
 if(ars!=8)$fatal(1,"eight-AR setup");
 for(i=7;i>=0;i=i-1)response(i,2*i);
 repeat(10)@(negedge clk);
 if(ars!=8||rs!=16||outs!=0||!ov||od!==32'ha5000000)$fatal(1,"receive completion released credit early");
 ready=1;@(negedge clk);ready=0;repeat(10)@(negedge clk);
 if(ars!=8||outs!=1||od!==32'ha5000001)$fatal(1,"partial entry drain released credit early");
 ready=1;n=0;while(ars<10&&n<1000)begin @(negedge clk);n=n+1;end
 if(ars!=10)$fatal(1,"ordered final drains did not release credit");
 response(1,18);response(0,16);
 n=0;while(!done&&n<1000)begin @(negedge clk);n=n+1;end
 if(!done||err||rs!=20||outs!=20)$fatal(1,"completion conservation");
 if(ooo_complete_cnt==0||completed!=10)$fatal(1,"designated completion reorder case was vacuous");
 $display("OOO_COMPLETIONS=%0d completed=%0d",ooo_complete_cnt,completed);
 $display("ROB_COMPLETION_PASS AR=%0d R=%0d OUT=%0d first_completion_ID=7 last_initial_completion_ID=0 high_payload=a5000000",ars,rs,outs);
 $display("TOTAL: 1 PASS / 0 FAIL");$finish;
end
initial begin #100000;$fatal(1,"probe timeout");end
endmodule
