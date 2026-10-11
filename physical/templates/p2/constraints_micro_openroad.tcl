read_liberty @FLOW@/platforms/nangate45/lib/NangateOpenCellLibrary_typical.lib
read_lef @FLOW@/platforms/nangate45/lef/NangateOpenCellLibrary.tech.lef
read_lef @FLOW@/platforms/nangate45/lef/NangateOpenCellLibrary.macro.mod.lef
read_verilog $::env(P2_PHASE)/constraints_micro.v
link_design constraints_micro
create_clock -name bus_clk -period 10.0 [get_ports bus_clk]
create_clock -name mac_clk -period 7.5 [get_ports mac_clk]
set_clock_groups -name async_allow_paths -asynchronous -allow_paths -group [get_clocks bus_clk] -group [get_clocks mac_clk]
set_input_delay -clock bus_clk -max 1.0 [get_ports data]
set_input_delay -clock bus_clk -min 0.0 [get_ports data]
set_output_delay -clock mac_clk -max 1.0 [all_outputs]
set_output_delay -clock mac_clk -min 0.0 [all_outputs]
set_input_transition 0.10 [get_ports data]
set_load 5.0 [all_outputs]
set_false_path -from [get_ports rn]
set_max_delay 7.5 -ignore_clock_latency -from [get_cells src] -to [get_cells gray1]
set_false_path -hold -from [get_cells src] -to [get_cells gray1]
set_false_path -from [get_cells event_src] -to [get_cells event1]
proc paths {from to mode} {
 return [find_timing_paths -from [get_cells $from] -to [get_cells $to] -path_delay $mode]
}
proc require_path {from to mode} {
 set p [paths $from $to $mode]
 if {[llength $p]!=1} {error "Expected one timed $mode path $from -> $to"}
 return [get_property [lindex $p 0] slack]
}
set base [require_path src gray1 max]
if {[llength [paths src gray1 min]]} {error "Bounded CDC hold not excluded"}
foreach pair {{gray1 gray2} {event1 event2}} {
 foreach mode {max min} {require_path {*}$pair $mode}
}
foreach mode {max min} {
 if {[llength [paths event_src event1 $mode]]} {error "Event source cut ineffective"}
}
set_clock_latency -source 0.0 [get_clocks bus_clk]
set_clock_latency -source 0.0 [get_clocks mac_clk]
set_clock_latency 0.8 [get_clocks bus_clk]
set_clock_latency 0.2 [get_clocks mac_clk]
set changed [require_path src gray1 max]
puts "IGNORE_CLOCK_LATENCY_TEST base=$base shifted=$changed"
if {abs($base-$changed)>0.000001} {error "Clock latency leaked into bounded CDC"}
report_checks -from [get_cells src] -to [get_cells gray1] -path_delay max -format full_clock_expanded -digits 6
set_max_delay 1.0 -ignore_clock_latency -reset_path -from [get_cells src] -to [get_cells gray1]
set tight [require_path src gray1 max]
puts "BOUND_EFFECT_TEST original=$changed tightened=$tight delta=[expr {$changed-$tight}]"
if {abs(($changed-$tight)-6.5)>0.000001} {error "Bound not effective"}
# Negative control: the broad asynchronous cut must suppress the bounded path.
set_clock_groups -name broad_negative_control -asynchronous -group [get_clocks bus_clk] -group [get_clocks mac_clk]
if {[llength [paths src gray1 max]]} {error "Negative control did not mask bound"}
puts "CDC_SEMANTICS_PASS max_bound_effective=1 ignored_clock_latencies=1 targeted_hold_cut=1 event_stage1_cut=1 interstage_setup_hold=1 broad_cut_negative_control=1"
