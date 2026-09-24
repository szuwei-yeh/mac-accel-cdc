# Whole-top functional-mode portfolio profile. SOURCE after link in a fresh
# design with no existing clocks/exceptions. This file performs no synthesis.
# Required caller variables (see README):
#   WTOP_ELAB_PARAMS, WTOP_TIME_UNIT, WTOP_CDC_CELLS
# It deliberately refuses incomplete CDC bindings, rather than silently cutting
# every bus/MAC path. Caller must abort on any Tcl/tool error; no catch-and-continue.
set wtop_dir [file dirname [file normalize [info script]]]
source [file join $wtop_dir profile.tcl]
source [file join $wtop_dir cdc_constraints.tcl]

if {[get_object_name [current_design]] ne $wtop::top} {
    error "WTOP: wrong current design; expected $wtop::top"
}
if {![info exists WTOP_ELAB_PARAMS] || ![info exists WTOP_TIME_UNIT] ||
    ![info exists WTOP_CDC_CELLS]} {
    error "WTOP: missing verified configuration, unit, or CDC bindings; see README"
}
if {$WTOP_TIME_UNIT ne "ns"} {error "WTOP: this profile requires ns library units"}
dict for {name value} $wtop::parameters {
    if {![dict exists $WTOP_ELAB_PARAMS $name] ||
        [dict get $WTOP_ELAB_PARAMS $name] != $value} {
        error "WTOP: elaboration must use $name=$value"
    }
}
if {[sizeof_collection [get_clocks -quiet *]] != 0} {
    error "WTOP: use a fresh design; do not overlay canonical block constraints"
}
wtop::validate_cdc $WTOP_CDC_CELLS

# Exact names are checked after the broad port lookup, including every bus bit.
proc wtop::ports {name width} {
    set ports [get_ports -quiet ${name}*]
    set expected [list $name]
    if {$width > 1} {
        set expected {}
        for {set i 0} {$i < $width} {incr i} {
            lappend expected [format {%s[%d]} $name $i]
        }
    }
    if {[lsort [get_object_name $ports]] ne [lsort $expected]} {
        error "WTOP: missing/extra port bits for $name (expected width $width)"
    }
    return $ports
}
set wtop_bus_port [wtop::ports S_AXI_ACLK 1]
set wtop_mac_port [wtop::ports mac_clk 1]
set wtop_bus_reset [wtop::ports S_AXI_ARESETN 1]
set wtop_mac_reset [wtop::ports mac_rst 1]
set wtop_inputs {}
foreach {name width} $wtop::bus_inputs {
    set wtop_inputs [add_to_collection $wtop_inputs [wtop::ports $name $width]]
}
set wtop_outputs {}
foreach {name width} $wtop::bus_outputs {
    set wtop_outputs [add_to_collection $wtop_outputs [wtop::ports $name $width]]
}
set wtop_special [get_ports {S_AXI_ACLK mac_clk S_AXI_ARESETN mac_rst}]
set wtop_all_inputs [add_to_collection $wtop_inputs $wtop_special]
foreach {actual expected} [list [all_inputs] $wtop_all_inputs [all_outputs] $wtop_outputs] {
    if {[lsort [get_object_name $actual]] ne [lsort [get_object_name $expected]]} {
        error "WTOP: port direction/coverage mismatch; review interface"
    }
}

create_clock -name bus_clk -period $wtop::bus_period $wtop_bus_port
create_clock -name mac_clk -period $wtop::mac_period $wtop_mac_port
# Express asynchronous relationship WITHOUT suppressing every interclock path.
# Both DC R-2020.09-SP4 and PT R-2020.09-SP5-1 document -allow_paths.
set_clock_groups -name wtop_async -asynchronous -allow_paths \
    -group [get_clocks bus_clk] -group [get_clocks mac_clk]
set_input_delay -clock bus_clk -min $wtop::io_min $wtop_inputs
set_input_delay -clock bus_clk -max $wtop::io_max $wtop_inputs
set_output_delay -clock bus_clk -min $wtop::io_min $wtop_outputs
set_output_delay -clock bus_clk -max $wtop::io_max $wtop_outputs
set_max_fanout $wtop::max_fanout [current_design]

# Explicit reset-timing exclusion for this profile. No case analysis that could
# turn reset into a constant during synthesis; reset hardware must remain.
# Recovery/removal, synchronous reset arrival and RDC are NOT covered.
set_false_path -from $wtop_bus_reset
set_false_path -from $wtop_mac_reset
wtop::apply_cdc $WTOP_CDC_CELLS
puts "WTOP_PORTFOLIO_CONSTRAINTS_APPLIED: requires post-application coverage review"
