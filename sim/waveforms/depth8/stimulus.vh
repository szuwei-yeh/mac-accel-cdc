    // Supplemental directed case; existing scoreboard/tasks above are reused.
    // Only the generated testbench's model latency connection delays ARID 0.
    initial begin
        $display("Standalone ROB test, depth 8; 128 beats; delay entry 0 by 200 cycles");
        do_reset;
        run_phase(32'h0000_0000, 128, 16'd100, 1'b1, 1'b0, 1'b0);
        $display("DEPTH8_SCOREBOARD: %0d PASS / %0d FAIL", pass_cnt, fail_cnt);
        if (pass_cnt != 1 || fail_cnt != 0)
            $fatal(1, "Depth-8 demonstration failed");
        #150; $finish;
    end

    initial begin
        #2000000;
        $fatal(1, "Depth-8 timeout");
    end
