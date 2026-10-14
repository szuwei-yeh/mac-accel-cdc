`timescale 1ns/1ps

`ifndef ENG_MAX_OUT
`define ENG_MAX_OUT 4
`endif

// Observability wrapper only: original sealed testbench, DUT and model unchanged.
// +ROB_VCD=<path> opts into a dump of the first (64-beat) regression phase.
module tb_axi_read_engine_rob_waveform;
    tb_axi_read_engine_rob_ooo tb();
    reg [4095:0] dump_path;
    reg capture_enabled = 0;
    initial begin
        if ($value$plusargs("ROB_VCD=%s", dump_path)) begin
            capture_enabled = 1;
            $dumpfile(dump_path);
            $dumpvars(0, tb.clk, tb.resetn, tb.ARID, tb.ARADDR, tb.ARLEN,
                         tb.ARVALID, tb.ARREADY, tb.RID, tb.RDATA, tb.RRESP,
                         tb.RLAST, tb.RVALID, tb.RREADY, tb.out_valid,
                         tb.out_ready_tb, tb.out_data, tb.out_last,
                         tb.engine_done, tb.engine_err);
            $dumpvars(0, tb.dut);
            wait (tb.engine_done === 1'b1);
            #1;
            $dumpoff;
        end
    end
    // Icarus does not include unpacked arrays in a scope dump automatically.
    genvar entry, word_index;
    generate
        for (entry = 0; entry < `ENG_MAX_OUT; entry = entry + 1) begin: entries
            initial begin
                #0;
                if (capture_enabled) begin
                    $dumpvars(0, tb.dut.e_complete[entry]);
                    $dumpvars(0, tb.dut.e_recv[entry]);
                end
            end
        end
        for (word_index = 0; word_index < `ENG_MAX_OUT * 16;
             word_index = word_index + 1) begin: words
            initial begin
                #0;
                if (capture_enabled) $dumpvars(0, tb.dut.rob_data[word_index]);
            end
        end
    endgenerate
endmodule
