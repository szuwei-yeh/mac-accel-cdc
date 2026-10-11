# Nangate45 units ns/fF. Exact baseline clock, I/O and CDC budgets.
create_clock -name bus_clk -period 10.0 [get_ports S_AXI_ACLK]
create_clock -name mac_clk -period 7.5 [get_ports mac_clk]
# Qualified adapter contract: no external clock source insertion.
set_clock_latency -source 0.0 [all_clocks]
set_clock_groups -name bus_mac_async_allow_paths -asynchronous -allow_paths -group [get_clocks bus_clk] -group [get_clocks mac_clk]
set ins [get_ports {S_AXI_AWADDR* S_AXI_AWPROT* S_AXI_AWVALID S_AXI_WDATA* S_AXI_WSTRB* S_AXI_WVALID S_AXI_BREADY S_AXI_ARADDR* S_AXI_ARPROT* S_AXI_ARVALID S_AXI_RREADY M_AXI_ARREADY M_AXI_RID* M_AXI_RDATA* M_AXI_RRESP* M_AXI_RLAST M_AXI_RVALID}]
set outs [all_outputs]
set_input_delay -clock bus_clk -min 0.0 $ins
set_input_delay -clock bus_clk -max 1.0 $ins
set_output_delay -clock bus_clk -min 0.0 $outs
set_output_delay -clock bus_clk -max 1.0 $outs
set_input_transition 0.10 $ins
set_load 5.0 $outs
set_max_fanout 32 [current_design]
set_false_path -from [get_ports {S_AXI_ARESETN mac_rst}]
source $::env(P2_RUN)/audit/cdc-bindings.tcl
foreach id {start done busy write_gray read_gray payload result latency} {
 set src [get_cells $p2_bindings(${id}.from)]
 set dst [get_cells $p2_bindings(${id}.to)]
 if {[llength $src] != $p2_counts(${id}.from) || [llength $dst] != $p2_counts(${id}.to)} {error "CDC resolved collection mismatch: $id"}
 if {$id in {start done busy}} {
  set_false_path -from $src -to $dst
 } else {
  set budget [expr {$id in {result latency} ? 10.0 : 7.5}]
  set_max_delay $budget -ignore_clock_latency -from $src -to $dst
  set_false_path -hold -from $src -to $dst
 }
}
