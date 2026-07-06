# Basic single-clock constraints for ROB/DMA synthesis sanity checks.
# This is not a signoff SDC; it is intentionally conservative and portable.

create_clock -name clk -period $CLK_PERIOD [get_ports clk]

set_input_delay  [expr {$CLK_PERIOD * 0.10}] -clock clk [remove_from_collection [all_inputs] [get_ports clk]]
set_output_delay [expr {$CLK_PERIOD * 0.10}] -clock clk [all_outputs]

set_max_fanout 32 [current_design]
