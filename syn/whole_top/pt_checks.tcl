# Read-only checks of the imported constraint interpretation.
# In PT a fanin query on CLK itself includes that register as a startpoint.
# Functional CDC cones must therefore use the active data pin, not every input.
# This override is local to PT; the existing DC audit script stays unchanged.
proc wtop_path_pins {cells kind} {
    if {$kind eq "from"} {set pattern */CLK} else {set pattern */D}
    return [get_pins -of_objects $cells -filter "full_name =~ $pattern"]
}
proc wtop_report_path {run label from to delay expect_timed} {
    set paths [get_timing_paths -delay_type $delay -from $from -to $to \
        -max_paths 1 -slack_lesser_than 1e9]
    redirect -file [file join $run reports ${label}_${delay}.rpt] {
        report_timing -delay_type $delay -from $from -to $to -max_paths 1 \
            -path_type full_clock_expanded -significant_digits 6 -nosplit
    }
    if {[expr {[sizeof_collection $paths] > 0}] != $expect_timed} {
        error "WTOP_PT unexpected constrained-path visibility: $label $delay"
    }
}
proc wtop_cell_fanin {cell} {
    # OSU DFFSR async R/S checks can also mark controls as data pins. Select
    # the functional D pin explicitly; CLK and preset controls are audited below.
    set pins [get_pins -of_objects $cell -filter {full_name =~ */D}]
    if {[sizeof_collection $pins] != 1 || ![string match */D [get_object_name $pins]]} {
        error "WTOP_PT unexpected functional data pin on [get_object_name $cell]"
    }
    return [all_fanin -to $pins -flat -startpoints_only -only_cells -trace_arcs all]
}
proc wtop_pt_clock_pins {run} {
    set f [open [file join $run reports clock_pin_audit.rpt] w]
    set count 0
    set inactive_sets {}
    foreach_in_collection cell [get_cells -hierarchical -filter {is_sequential == true && is_hierarchical == false}] {
        set clk_count 0
        foreach_in_collection pin [get_pins -of_objects $cell] {
            set name [get_object_name $pin]
            if {[string match */CLK $name]} {
                set clocks [get_attribute -quiet $pin clocks]
                if {[sizeof_collection $clocks] != 1 || [get_object_name $clocks] ni {bus_clk mac_clk}} {
                    error "WTOP_PT missing/ambiguous functional clock: $name"
                }
                incr clk_count
            }
            if {[get_attribute $cell ref_name] eq "DFFSR" && [string match */S $name]} {
                set value [get_attribute -quiet $pin constant_value]
                if {$value ne "1"} {error "WTOP_PT unexpected preset connection: $name=$value"}
                lappend inactive_sets $name
                puts $f "INACTIVE_PRESET $name CONSTANT=$value"
            }
        }
        if {$clk_count != 1} {error "WTOP_PT expected one CLK pin per register"}
        incr count
    }
    set input [open [file join $run reports check_timing.rpt]]
    set text [read $input]
    close $input
    set no_clock [regexp -all -inline -line {^[^ \t\r\n]+/[^ \t\r\n]+$} $text]
    if {[lsort $no_clock] ne [lsort $inactive_sets]} {
        error "WTOP_PT no_clock list differs from verified inactive preset pins"
    }
    puts $f "FUNCTIONAL_CLK_PINS_CHECKED=$count MISSING_FUNCTIONAL_CLOCK=0"
    puts $f "NO_CLOCK_PRESET_PINS=[llength $inactive_sets] ALL_CONSTANT_ONE"
    close $f
}
proc wtop_pt_checks {run} {
    if {[sizeof_collection [get_clocks *]] != 2} {error "WTOP_PT expected two clocks"}
    foreach {name period} {bus_clk 10.0 mac_clk 7.5} {
        set clk [get_clocks $name]
        if {[get_attribute $clk period] != $period} {error "WTOP_PT clock period mismatch"}
        set propagated [get_attribute -quiet $clk propagated_clock]
        if {$propagated ne "" && $propagated} {error "WTOP_PT unexpected propagated clock"}
    }
    wtop_pt_clock_pins $run
    set bindings [wtop_bind]
    wtop::validate_cdc $bindings
    wtop_audit_cdc $run pt $bindings
    wtop_timing_visibility $run pt $bindings
    set f [open [file join $run reports path_metrics.tsv] w]
    puts $f "group\tdelay\tworst_slack\tviolation_wns\ttns\tviolating_endpoints\tstartpoint\tendpoint\tarrival"
    foreach name {bus_clk mac_clk} {
        foreach delay {max min} {
            redirect -file [file join $run reports ${name}_${delay}.rpt] {
                report_timing -group $name -delay_type $delay -max_paths 1 -nworst 1 \
                    -path_type full_clock_expanded -input_pins -nets -transition_time \
                    -capacitance -significant_digits 6 -nosplit
            }
            set worst [get_timing_paths -group $name -delay_type $delay -max_paths 1 -nworst 1 -slack_lesser_than 1e9]
            if {[sizeof_collection $worst] != 1} {error "WTOP_PT missing worst path for $name $delay"}
            set violations [get_timing_paths -group $name -delay_type $delay -max_paths 100000 -nworst 1 -slack_lesser_than 0]
            set tns 0.0
            foreach_in_collection p $violations {set tns [expr {$tns + [get_attribute $p slack]}]}
            set slack [get_attribute $worst slack]
            puts $f [join [list $name $delay $slack [expr {min(0.0,$slack)}] $tns \
                [sizeof_collection $violations] [get_object_name [get_attribute $worst startpoint]] \
                [get_object_name [get_attribute $worst endpoint]] [get_attribute $worst arrival]] "\t"]
        }
    }
    close $f
    # Pin-identical DC reference paths, separately from each tool's own worst path.
    set index [wtop_index_cells]
    foreach {label src dst} {
        dc_bus_max u_dma/u_read_engine/head_ptr_reg[1] u_dma/buf_a_reg[127][12]
        dc_mac_max u_pe/a_in_r_reg[1] u_pe/mul_reg_reg[31]
    } {
        wtop_report_path $run $label [wtop_path_pins [dict get $index $src] from] \
            [wtop_path_pins [dict get $index $dst] to] max 1
    }
    puts "WTOP_PT_SANITY_COMPLETE"
}
