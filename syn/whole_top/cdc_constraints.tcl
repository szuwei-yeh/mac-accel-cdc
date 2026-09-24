# Collection contract for a linked whole-top design. No guessed cell patterns.
# A later elaboration/mapping step must bind and audit actual sequential cells.
namespace eval wtop {
    # ID, source-register bits, destination-register bits, policy, budget variable
    variable crossings {
        start       1   1 false  none
        done        1   1 false  none
        busy        1   1 false  none
        write_gray  5   5 bound  gray_budget
        read_gray   5   5 bound  gray_budget
        payload   528  33 bound  payload_budget
        result     32  32 bound  result_budget
        latency    32  32 bound  result_budget
    }
}

proc wtop::validate_cdc {bindings} {
    variable crossings
    if {[dict size $bindings] != 16} {
        error "WTOP: require exactly 16 CDC collections; see README binding contract"
    }
    foreach {id nfrom nto policy budget} $crossings {
        foreach {side count} [list from $nfrom to $nto] {
            set key ${id}.${side}
            if {![dict exists $bindings $key]} {error "WTOP: missing $key"}
            set cells [dict get $bindings $key]
            if {[sizeof_collection $cells] != $count} {
                error "WTOP: $key must contain $count distinct sequential cells"
            }
            if {[sizeof_collection [filter_collection $cells {is_sequential == true}]] != $count} {
                error "WTOP: $key contains non-sequential objects"
            }
            if {[llength [lsort -unique [get_object_name $cells]]] != $count} {
                error "WTOP: duplicate objects in $key"
            }
        }
    }
}

proc wtop::apply_cdc {bindings} {
    variable crossings
    foreach {id nfrom nto policy budget} $crossings {
        set src [dict get $bindings ${id}.from]
        set dst [dict get $bindings ${id}.to]
        if {$policy eq "false"} {
            # Only the asynchronous source -> first synchronizer stage.
            set_false_path -from $src -to $dst
        } else {
            variable $budget
            # These Synopsys versions support -ignore_clock_latency,
            # not -datapath_only. Includes clock-to-Q and endpoint setup.
            set_max_delay [set $budget] -ignore_clock_latency -from $src -to $dst
            # No phase-related hold check between unrelated clocks.
            # The max-delay bound remains active; same-domain hold remains active.
            set_false_path -hold -from $src -to $dst
        }
    }
}
