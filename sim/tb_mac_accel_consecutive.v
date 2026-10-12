`timescale 1ns/1ps
// Three serialized CPU-fed jobs, one startup reset, both start toggle edges.
module tb_mac_accel_consecutive;
    reg bus_clk=0,mac_clk=0,bus_rst=1,mac_rst=1,we=0;
    reg [3:0] addr=0;
    reg [31:0] wdata=0;
    wire [31:0] rdata;
    wire done;
    real period=7.5,phase=0.0;
    integer arg_status;
    always #5 bus_clk=~bus_clk;
    initial begin
        arg_status=$value$plusargs("MAC_PERIOD=%f",period);
        arg_status=$value$plusargs("MAC_PHASE=%f",phase);
        #phase;forever #(period/2)mac_clk=~mac_clk;
    end
    mac_accel dut(.bus_clk(bus_clk),.mac_clk(mac_clk),.bus_rst(bus_rst),.mac_rst(mac_rst),
        .we(we),.re(1'b1),.addr(addr),.wdata(wdata),.rdata(rdata),.done_led(done));
    integer starts=0,dones=0,bus_dones=0,writes=0,reads=0,rises=0,falls=0;
    reg old_toggle=0;
    always @(posedge bus_clk) if(!bus_rst)begin
        if(dut.fifo_wr_en_bus && !dut.fifo_full_bus)writes=writes+1;
        if(dut.done_pulse_bus)bus_dones=bus_dones+1;
        #1;
        if(dut.start_toggle_bus!=old_toggle)begin
            if(dut.start_toggle_bus)rises=rises+1;else falls=falls+1;
            old_toggle=dut.start_toggle_bus;
        end
    end
    always @(posedge mac_clk)if(!mac_rst)begin
        if(dut.start_pulse_mac)starts=starts+1;
        if(dut.done_mac)dones=dones+1;
        if(dut.fifo_rd_en_mac&&!dut.fifo_empty_mac)reads=reads+1;
    end
    task write(input[3:0]a,input[31:0]d);
        begin @(negedge bus_clk);addr=a;wdata=d;we=1;@(negedge bus_clk);we=0;end
    endtask
    integer job,i,n,base,expected;
    initial begin
        repeat(8)@(negedge bus_clk);@(negedge mac_clk);mac_rst=0;@(negedge bus_clk);bus_rst=0;
        for(job=0;job<3;job=job+1)begin
            base=job==0 ? -5 : (job==1 ? 2 : 7);expected=4*base+6;
            write(3,15);for(i=0;i<4;i=i+1)write(1,base+i);for(i=0;i<4;i=i+1)write(2,1);
            write(0,1);n=0;while(!done&&n<2000)begin @(negedge bus_clk);n=n+1;end
            if(!done)$fatal(1,"serialized job completion timeout job=%0d",job+1);
            addr=4;#1;
            if(rdata!==expected||starts!=job+1||dones!=job+1||bus_dones!=job+1||writes!=4*(job+1)||reads!=writes)
                $fatal(1,"consecutive result/event/payload mismatch job=%0d result=%h expected=%h starts=%0d dones=%0d",job+1,rdata,expected,starts,dones);
            $display("PASS job=%0d sum=%0d starts=%0d dones=%0d FIFO=%0d/%0d",job+1,expected,starts,dones,writes,reads);
        end
        if(rises!=2||falls!=1)$fatal(1,"both toggle directions not covered");
        $display("TOTAL: 3 PASS / 0 FAIL");$finish;
    end
    initial begin #200000;$fatal(1,"global consecutive-job timeout");end
endmodule
