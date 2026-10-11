set stage $::env(P4_AUDIT_STAGE)
set label [expr {[info exists ::env(P4_AUDIT_LABEL)]?$::env(P4_AUDIT_LABEL):$stage}]
set finalprefix [expr {[info exists ::env(P4_FINAL_PREFIX)]?$::env(P4_FINAL_PREFIX):"final"}]
set standalone [expr {[info exists ::env(P4_STANDALONE)] && $::env(P4_STANDALONE) eq "1"}]
set res $::env(P4_RUN)/results/nangate45/mac_accel_dma_rob_top/$::env(P4_VARIANT)
read_liberty @FLOW@/platforms/nangate45/lib/NangateOpenCellLibrary_typical.lib
if {$standalone} {
 read_verilog $::env(P4_RUN)/exports/${finalprefix}.v
 link_design mac_accel_dma_rob_top
 read_sdc $::env(P4_RUN)/exports/${finalprefix}.sdc
 read_spef $::env(P4_RUN)/exports/${finalprefix}.spef
} else {
 if {$stage eq "globalroute"} {set db 5_1_grt.odb;set sdc 5_1_grt.sdc} elseif {$stage eq "final"} {set db 6_final.odb;set sdc 6_final.sdc} else {error "Unexpected audit stage"}
 read_db $res/$db
 read_sdc $res/$sdc
 if {$stage eq "globalroute"} {
  source @FLOW@/platforms/nangate45/setRC.tcl
  set_routing_layers -clock metal4-metal10 -signal metal2-metal10
  estimate_parasitics -global_routing -spef_file $::env(P4_RUN)/exports/${label}-estimated.spef
 } else {
  read_spef $res/6_final.spef
 }
}
set_propagated_clock [all_clocks]
puts "PARASITIC_ANNOTATION_BEGIN"
report_parasitic_annotation -report_unannotated
puts "PARASITIC_ANNOTATION_END"
source $::env(P4_RUN)/audit/p2-cdc-bindings.tcl
source $::env(P4_RUN)/audit/p2-sta-pairs.tcl
if {!$standalone} {source $::env(P4_PHASE)/repair_guard_post.tcl}
puts "REGISTER_COVERAGE total=[llength [all_registers -cells]] bus=[llength [all_registers -cells -clock bus_clk]] mac=[llength [all_registers -cells -clock mac_clk]]"
if {[llength [all_registers -cells]]!=7503 || [llength [all_registers -cells -clock bus_clk]]!=7281 || [llength [all_registers -cells -clock mac_clk]]!=222} {error "Register coverage changed"}
if {[get_property [get_clocks bus_clk] period]!=10.0 || [get_property [get_clocks mac_clk] period]!=7.5 || [llength [all_clocks]]!=2} {error "Clock contract changed"}
foreach clk {bus_clk mac_clk} {
 puts "CLOCK_PROPAGATED $clk [get_property [get_clocks $clk] is_propagated]"
 if {![get_property [get_clocks $clk] is_propagated]} {error "Clock not propagated"}
}
if {!$standalone} {
puts "PLACEMENT_CHECK_BEGIN"
check_placement -verbose
puts "PLACEMENT_CHECK_END"
check_power_grid -net VDD -error_file $::env(P4_RUN)/audit/${label}-vdd-errors.rpt
check_power_grid -net VSS -error_file $::env(P4_RUN)/audit/${label}-vss-errors.rpt
set block [ord::get_db_block]
set inst_index [dict create]
foreach inst [$block getInsts] {
 set key [string map [list {\[} {[} {\]} {]}] [$inst getName]]
 if {[dict exists $inst_index $key]} {error "Ambiguous physical instance name"}
 dict set inst_index $key $inst
}
foreach name {VDD VSS} {
 set missing {}
 foreach inst [$block getInsts] {
  foreach term [$inst getITerms] {
   if {[[$term getMTerm] getName] eq $name && [$term getNet] eq "NULL"} {lappend missing [$inst getName]}
  }
 }
 puts "PG_PIN_COVERAGE $name missing=[llength $missing]"
 if {[llength $missing]} {error "Unconnected supply pins"}
}
report_design_area

 if {$stage eq "final"} {
  puts "ROUTING_COMPLETE [design_is_routed -verbose]"
  report_wire_length -detailed_route -summary -file $::env(P4_RUN)/audit/${label}-wire-length.rpt
  if {![design_is_routed]} {error "Design not fully routed"}
  check_antennas -report_file $::env(P4_RUN)/audit/${label}-antennas.rpt
 }
}
puts "CONSTRAINT_COVERAGE_BEGIN"
check_setup -verbose
puts "CONSTRAINT_COVERAGE_END"
proc paths {src dst mode} {return [find_timing_paths -from [get_cells $src] -to [get_cells $dst] -path_delay $mode]}
proc data_delay {p src} {
 # PathEnd.points starts at data Q; subtract late rising arrival at launch CK.
 set points [get_property $p points]
 set endpoint_arrival [get_property [lindex $points end] arrival]
 set clock_arrival [get_property [get_pins ${src}/CK] arrival_max_rise]
 return [expr {$endpoint_arrival-$clock_arrival}]
}
set f [open $::env(P4_RUN)/audit/${label}-cdc-paths.tsv w]
puts $f "group\tsource\tdestination\tmax_slack_ns\tclock_q_and_datapath_ns\tmin_status"
array set minima {};array set count {};array set graydelay {}
foreach pair $p2_pairs {
 lassign $pair id src dst
 set max [paths $src $dst max]
 if {[llength $max]==1} {
  set p [lindex $max 0];set slack [get_property $p slack]
  set delay [data_delay $p $src]
 }
 set min [paths $src $dst min]
 if {[llength $min]} {error "Cross-domain hold exclusion missing: $pair"}
 if {$id in {start done busy}} {
  if {[llength $max]} {error "Event exclusion missing: $pair"}
  puts $f "$id\t$src\t$dst\tFALSE_PATH\tFALSE_PATH\tFALSE_PATH"
 } else {
  if {[llength $max]!=1} {error "Untimed bounded CDC: $pair"}
  if {$slack < 0} {error "Required bounded CDC path has negative slack: $pair"}
  puts $f "$id\t$src\t$dst\t$slack\t$delay\tTARGETED_HOLD_FALSE"
  if {![info exists minima($id)] || $slack<$minima($id)} {set minima($id) $slack}
  incr count($id)
  if {$id in {write_gray read_gray}} {lappend graydelay($id) $delay;puts "GRAY_BIT $id $src -> $dst data_delay_ns=$delay slack_ns=$slack"}
 }
}
close $f
foreach id {write_gray read_gray payload result latency} {
 puts "CDC_BOUND $id timed_pairs=$count($id) worst_slack_ns=$minima($id)"
 report_checks -from [get_cells $p2_bindings(${id}.from)] -to [get_cells $p2_bindings(${id}.to)] -path_delay max -format full_clock_expanded -digits 6 -fields {slew cap fanout}
}
foreach id {write_gray read_gray} {
 set delays [lsort -real $graydelay($id)]
 puts "GRAY_SPREAD $id min_ns=[lindex $delays 0] max_ns=[lindex $delays end] spread_ns=[expr {[lindex $delays end]-[lindex $delays 0]}]"
}
set f [open $::env(P4_RUN)/audit/${label}-synchronizers.tsv w]
puts $f "chain\tstage1\tstage2\tmanhattan_um\tmax_slack_ns\tmin_slack_ns"
foreach chain $p2_chains {
 lassign $chain name src dst
 set max [paths $src $dst max];if {[llength $max]!=1} {error "Interstage setup absent"};set setup [get_property [lindex $max 0] slack]
 set min [paths $src $dst min];if {[llength $min]!=1} {error "Interstage hold absent"};set hold [get_property [lindex $min 0] slack]
 if {$setup < 0 || $hold < 0} {error "Negative synchronizer interstage timing: $name"}
 if {!$standalone} {
 set a [dict get $inst_index $src];set b [dict get $inst_index $dst]
 if {$a eq "NULL" || $b eq "NULL"} {error "Physical stage identity missing"}
 set ra [$a getBBox];set rb [$b getBBox];set u [$block getDbUnitsPerMicron]
 set dist [expr {(abs(([$ra xMin]+[$ra xMax])/2.0-([$rb xMin]+[$rb xMax])/2.0)+abs(([$ra yMin]+[$ra yMax])/2.0-([$rb yMin]+[$rb yMax])/2.0))/$u}]
 } else {set dist "NA"}
 puts $f "$name\t$src\t$dst\t$dist\t$setup\t$hold"
 puts "SYNC_INTERSTAGE $name distance_um=$dist setup_ns=$setup hold_ns=$hold"
}
close $f
foreach clk {bus_clk mac_clk} {
 report_clock_properties [get_clocks $clk]
 if {1} {
  report_clock_latency -clocks [get_clocks $clk] -digits 6
 }
 foreach mode {max min} {
  puts "DOMAIN_PATHS $clk $mode"
  report_checks -path_group $clk -path_delay $mode -group_path_count 5 -format full_clock_expanded -fields {slew cap fanout} -digits 6
  set worst [find_timing_paths -path_group $clk -path_delay $mode]
  puts "DOMAIN_WNS $clk $mode [get_property [lindex $worst 0] slack]"
  set ps [find_timing_paths -path_group $clk -path_delay $mode -group_path_count 20000 -slack_max 0.0 -unique_paths_to_endpoint]
  set tns 0.0;set violated 0
  foreach p $ps {set sl [get_property $p slack];if {$sl<0} {incr violated;set tns [expr {$tns+$sl}]}}
  puts "DOMAIN_VIOLATIONS $clk $mode endpoints=$violated tns_ns=$tns"
 }
}
if {1} {report_clock_skew -setup -digits 6;report_clock_skew -hold -digits 6}
foreach mode {max min} {report_worst_slack -$mode -digits 6;report_tns -$mode -digits 6}
puts "ELECTRICAL_BEGIN"
report_check_types -max_slew -max_capacitance -max_fanout -violators -digits 6
puts "ELECTRICAL_END"
report_check_types -max_delay -min_delay -violators -digits 6
if {!$standalone} {report_floating_nets -verbose}
if {!$standalone} {
 write_def $::env(P4_RUN)/exports/${label}.def
 set excluded {}
 foreach lib [[ord::get_db] getLibs] {
  foreach master [$lib getMasters] {
   set name [$master getName]
   if {[string match TAPCELL* $name] || [string match FILLCELL* $name]} {lappend excluded $name}
  }
 }
 write_verilog -remove_cells $excluded $::env(P4_RUN)/exports/${label}.v
 write_sdc -no_timestamp $::env(P4_RUN)/exports/${label}.sdc
}
# Test constraint effects only after persisted checkpoint exports.
foreach id {write_gray read_gray payload result latency} {
 set budget [expr {$id in {result latency}?10.0:7.5}]
 set_max_delay [expr {$budget-2}] -ignore_clock_latency -from [get_cells $p2_bindings(${id}.from)] -to [get_cells $p2_bindings(${id}.to)]
 set p [find_timing_paths -from [get_cells $p2_bindings(${id}.from)] -to [get_cells $p2_bindings(${id}.to)] -path_delay max]
 set shifted [get_property [lindex $p 0] slack]
 if {abs(($minima($id)-$shifted)-2)>0.00001} {error "CDC bound ineffective: $id"}
 puts "BOUND_EFFECT_PASS $id delta_ns=[expr {$minima($id)-$shifted}]"
}
puts "P4_PHYSICAL_AUDIT_COMPLETE $label bounded_pairs=602 events=3 interstage_checks=26"
