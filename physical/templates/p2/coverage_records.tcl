# Evaluate exported SDC with side-effect-free query/constraint mocks.
# Use native Tcl tokenization rather than a Python approximation.
proc get_cells {objects} {return $objects}
proc get_ports {objects} {return $objects}
proc get_clocks {objects} {return $objects}
proc current_design {args} {return mac_accel_dma_rob_top}
proc record {kind args} {
 set fields [list $kind]
 foreach arg $args {
  set tokens {}
  foreach token $arg {lappend tokens [binary encode hex [encoding convertto utf-8 $token]]}
  lappend fields "[binary encode hex [encoding convertto utf-8 $arg]]:[join $tokens ,]"
 }
 puts [join $fields \t]
}
foreach kind {create_clock set_clock_latency set_clock_groups set_input_delay set_output_delay set_input_transition set_load set_max_delay set_false_path set_max_fanout} {
 interp alias {} $kind {} record $kind
}
source [lindex $argv 0]
