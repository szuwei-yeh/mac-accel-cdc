read_liberty $::env(MAC_LIB)
read_db $::env(MAC_DB)
read_sdc $::env(MAC_SDC)
read_spef $::env(MAC_SPEF)
set_propagated_clock [all_clocks]
set stage $::env(MAC_STAGE)
set f [open $::env(MAC_RUN)/analysis/${stage}-hold-paths.tsv w]
set points_file [open $::env(MAC_RUN)/analysis/${stage}-hold-points.tsv w]
puts $f "id\tstartpoint\tendpoint\tslack_ns\tdata_arrival_ns\trequired_ns\tcapture_clock_early_ns\tclock_to_required_delta_ns"
puts $points_file "id\tindex\tpin\tcell_type\tarrival_ns"
proc capture {id path f pf} {
 set p [lindex $path 0]
 set slack [get_property $p slack]
 set src [get_full_name [get_property $p startpoint]]
 set dst [get_full_name [get_property $p endpoint]]
 set capture_pin [get_property $p endpoint_clock_pin]
 set ps [get_property $p points]
 set arrival [get_property [lindex $ps end] arrival]
 set required [expr {$arrival-$slack}]
 set index 0
 foreach q $ps {
  set pin [get_property $q pin]
  set cell [get_cells -of_objects $pin]
  set type [expr {[llength $cell] ? [get_property $cell ref_name] : "PORT"}]
  puts $pf "$id\t$index\t[get_full_name $pin]\t$type\t[get_property $q arrival]"
  incr index
 }
 # Pin-arrival queries can invalidate PathEnd/Path objects; save all scalars first.
 set clock_arrival [get_property $capture_pin arrival_min_rise]
 puts $f "$id\t$src\t$dst\t$slack\t$arrival\t$required\t$clock_arrival\t[expr {$required-$clock_arrival}]"
 puts "MAC_HOLD_PATH $id slack=$slack data_arrival=$arrival required=$required capture_clock_early=$clock_arrival"
}
capture domain_worst [find_timing_paths -path_group bus_clk -path_delay min] $f $points_file
set paths {
 {p3_worst {M_AXI_RDATA[6]} {u_dma.u_read_engine.rob_data[52][6]$_DFFE_PP_}}
 {first_grt_worst {M_AXI_RDATA[0]} {u_dma.u_read_engine.rob_data[56][0]$_DFFE_PP_}}
 {first_route_worst {M_AXI_RDATA[0]} {u_dma.u_read_engine.rob_data[59][0]$_DFFE_PP_}}
 {second_grt_worst {M_AXI_RDATA[8]} {u_dma.u_read_engine.rob_data[30][8]$_DFFE_PP_}}
 {final_worst {M_AXI_RDATA[1]} {u_dma.u_read_engine.rob_data[37][1]$_DFFE_PP_}}
}
foreach spec $paths {
 lassign $spec id src dst
 set a [get_ports $src];set b [get_pins ${dst}/D]
 if {[llength $a]!=1 || [llength $b]!=1} {error "Missing matched hold endpoint $id"}
 set ps [find_timing_paths -from $a -to $b -path_delay min]
 if {[llength $ps]!=1} {error "Hold path missing $id"}
 capture $id $ps $f $points_file
 puts "MAC_MATCHED_FULL_PATH $id"
 report_checks -from $a -to $b -path_delay min -format full_clock_expanded -fields {slew cap fanout} -digits 6
}
close $f;close $points_file
puts "MAC_READ_ONLY_HOLD_ANALYSIS_COMPLETE $stage"
