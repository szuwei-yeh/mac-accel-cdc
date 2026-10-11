set res $::env(P2_RUN)/results/nangate45/mac_accel_dma_rob_top/base
read_liberty @FLOW@/platforms/nangate45/lib/NangateOpenCellLibrary_typical.lib
read_verilog $res/1_2_yosys.v
link_design mac_accel_dma_rob_top
read_sdc $res/1_synth_qualified.sdc
source $::env(P2_RUN)/audit/cdc-bindings.tcl
source $::env(P2_RUN)/audit/sta-pairs.tcl
report_units
report_clock_properties [all_clocks]
if {[llength [all_clocks]]!=2 || [get_property [get_clocks bus_clk] period]!=10.0 || [get_property [get_clocks mac_clk] period]!=7.5} {error "Wrong clocks"}
puts "REGISTER_COVERAGE total=[llength [all_registers -cells]] bus=[llength [all_registers -cells -clock bus_clk]] mac=[llength [all_registers -cells -clock mac_clk]]"
if {[llength [all_registers -cells]]!=7503 || [llength [all_registers -cells -clock bus_clk]]!=7281 || [llength [all_registers -cells -clock mac_clk]]!=222} {error "Sequential clock coverage failed"}
puts "CONSTRAINT_COVERAGE_BEGIN"
check_setup -verbose
puts "CONSTRAINT_COVERAGE_END"
proc paths {src dst mode} {return [find_timing_paths -from [get_cells $src] -to [get_cells $dst] -path_delay $mode]}
set f [open $::env(P2_RUN)/audit/sta-paths.tsv w]
puts $f "group\tsource\tdestination\tmax_slack_ns\tmin_status"
array set minima {}
array set count {}
foreach pair $p2_pairs {
 lassign $pair id src dst
 set max [paths $src $dst max]
 if {[llength $max]==1} {set slack [get_property [lindex $max 0] slack]}
 # A new path search invalidates prior PathEnd handles; save scalar first.
 set min [paths $src $dst min]
 if {[llength $min]} {error "Cross-domain hold exclusion missing: $pair"}
 if {$id in {start done busy}} {
  if {[llength $max]} {error "Event stage1 exclusion missing: $pair"}
  puts $f "$id\t$src\t$dst\tFALSE_PATH\tFALSE_PATH"
 } else {
  if {[llength $max]!=1} {error "Bounded crossing is untimed: $pair"}
  if {$slack < 0} {error "Required bounded CDC path has negative slack: $pair"}
  puts $f "$id\t$src\t$dst\t$slack\tTARGETED_HOLD_FALSE"
  if {![info exists minima($id)] || $slack < $minima($id)} {set minima($id) $slack}
  incr count($id)
 }
}
foreach id {write_gray read_gray payload result latency} {
 puts "CDC_BOUND $id timed_pairs=$count($id) worst_slack_ns=$minima($id)"
 puts "CDC_CRITICAL_PATH $id"
 report_checks -from [get_cells $p2_bindings(${id}.from)] -to [get_cells $p2_bindings(${id}.to)] -path_delay max -format full_clock_expanded -fields {slew cap fanout} -digits 6
}
foreach chain $p2_chains {
 lassign $chain name src dst
 foreach mode {max min} {
  set p [paths $src $dst $mode]
  if {[llength $p]!=1} {error "Synchronizer interstage path masked: $chain $mode"}
  if {[get_property [lindex $p 0] slack] < 0} {error "Negative synchronizer interstage slack: $chain $mode"}
  puts "SYNC_INTERSTAGE $name $mode slack_ns=[get_property [lindex $p 0] slack]"
 }
}
close $f
foreach clock {bus_clk mac_clk} {
 foreach mode {max min} {
  puts "DOMAIN_PATHS $clock $mode"
  report_checks -path_group $clock -path_delay $mode -group_path_count 5 -format full_clock_expanded -fields {slew cap fanout} -digits 6
 }
}
foreach mode {max min} {
 puts "OVERALL_$mode"
 report_worst_slack -$mode -digits 6
 report_tns -$mode -digits 6
}
puts "ELECTRICAL_BEGIN"
report_check_types -max_slew -max_capacitance -max_fanout -violators -digits 6
puts "ELECTRICAL_END"
report_check_types -max_delay -min_delay -violators -digits 6
# Tighten each bound in this disposable audit session; every group must respond.
foreach id {write_gray read_gray payload result latency} {
 set budget [expr {$id in {result latency} ? 10.0 : 7.5}]
 set_max_delay [expr {$budget-2.0}] -ignore_clock_latency -from [get_cells $p2_bindings(${id}.from)] -to [get_cells $p2_bindings(${id}.to)]
 set p [find_timing_paths -from [get_cells $p2_bindings(${id}.from)] -to [get_cells $p2_bindings(${id}.to)] -path_delay max]
 set changed [get_property [lindex $p 0] slack]
 if {abs(($minima($id)-$changed)-2.0)>0.00001} {error "Bound effect mismatch: $id $changed $minima($id)"}
 puts "BOUND_EFFECT_PASS $id slack_delta_ns=[expr {$minima($id)-$changed}]"
}
puts "P2_STA_AUDIT_PASS bounded_pairs=602 event_pairs=3 interstage_checks=26"
