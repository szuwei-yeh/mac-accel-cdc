proc wtop_mapped_reports {run prefix} {
    redirect -file [file join $run reports ${prefix}_qor.rpt] {report_qor}
    redirect -file [file join $run reports ${prefix}_area.rpt] {report_area -hierarchy}
    redirect -file [file join $run reports ${prefix}_design.rpt] {report_design; check_design}
    redirect -file [file join $run reports ${prefix}_hierarchy.rpt] {report_reference -hierarchy}
    redirect -file [file join $run reports ${prefix}_check_timing.rpt] {check_timing}
    redirect -file [file join $run reports ${prefix}_clocks.rpt] {report_clock; report_clock -groups}
    redirect -file [file join $run reports ${prefix}_ports.rpt] {report_port -verbose}
    redirect -file [file join $run reports ${prefix}_constraints.rpt] {report_constraint -all_violators}
    redirect -file [file join $run reports ${prefix}_exceptions.rpt] {report_timing_requirements -nosplit}
    redirect -file [file join $run reports ${prefix}_ignored.rpt] {report_timing_requirements -ignored -nosplit}
    redirect -file [file join $run reports ${prefix}_sequential.rpt] {
        foreach_in_collection c [get_cells -hierarchical -filter {is_sequential == true && is_hierarchical == false}] {
            puts "[get_object_name $c] [get_attribute $c ref_name]"
        }
    }
    foreach delay {max min} {
        redirect -file [file join $run reports ${prefix}_timing_${delay}.rpt] {
            report_timing -delay_type $delay -path_type full_clock_expanded \
                -max_paths 10 -nworst 1 -input_pins -nets -significant_digits 4
        }
    }
}
proc wtop_path_pins {cells kind} {
    if {$kind eq "from"} {set pins [all_registers -clock_pins]
    } else {set pins [all_registers -data_pins]}
    return [wtop_intersect [get_pins -of_objects $cells] $pins]
}
proc wtop_report_path {run label from to delay expect_timed} {
    redirect -variable text {
        report_timing -delay_type $delay -from $from -to $to \
            -max_paths 1 -path_type full_clock_expanded -significant_digits 4
    }
    set f [open [file join $run reports ${label}_${delay}.rpt] w]
    puts $f $text
    close $f
    # DC explicit -from/-to reports can display a false path as unconstrained.
    # Presence of a path alone does not mean that it is timed.
    set timed [regexp {slack[ \t]+\((MET|VIOLATED)\)} $text]
    if {$timed != $expect_timed} {error "WTOP unexpected path visibility: $label $delay"}
    if {!$expect_timed && ![regexp {Path is unconstrained|No paths} $text]} {
        error "WTOP missing evidence of intended exclusion: $label $delay"
    }
}
proc wtop_timing_visibility {run prefix bindings} {
    foreach {id ns nd policy budget} $wtop::crossings {
        set from [wtop_path_pins [dict get $bindings ${id}.from] from]
        set to [wtop_path_pins [dict get $bindings ${id}.to] to]
        foreach delay {max min} {
            set expected [expr {$policy eq "bound" && $delay eq "max"}]
            wtop_report_path $run ${prefix}_cdc_${id} $from $to $delay $expected
        }
    }
    set index [wtop_index_cells]
    set seconds [concat {start_sync_mac_reg[1] done_sync_bus_reg[1] busy_sync_bus_reg[1]} \
        [wtop_bits u_fifo/wr_ptr_gray_sync2_reg 5] [wtop_bits u_fifo/rd_ptr_gray_sync2_reg 5]]
    set number 0
    foreach dst $seconds {
        set src [string map {_sync2_reg _sync1_reg} $dst]
        if {$src eq $dst} {set src [string map {[1] [0]} $dst]}
        foreach delay {max min} {
            wtop_report_path $run ${prefix}_sync_[incr number] \
                [wtop_path_pins [dict get $index $src] from] \
                [wtop_path_pins [dict get $index $dst] to] $delay 1
        }
    }
    set readptr [wtop_cells $index [wtop_bits u_fifo/rd_ptr_bin_reg 4]]
    foreach delay {max min} {
        wtop_report_path $run ${prefix}_readptr_to_payload \
            [wtop_path_pins $readptr from] \
            [wtop_path_pins [dict get $bindings payload.to] to] $delay 1
    }
    puts "WTOP_TIMING_VISIBILITY_PASS=$prefix"
}
proc wtop_readback {run netlist sdc} {
    read_verilog $netlist
    current_design mac_accel_dma_rob_top
    if {![link]} {error "WTOP mapped readback link failed"}
    read_sdc $sdc
    wtop_mapped_reports $run readback
    set bindings [wtop_bind]
    wtop::validate_cdc $bindings
    wtop_audit_cdc $run readback $bindings
    wtop_timing_visibility $run readback $bindings
    puts "WTOP_MAPPED_READBACK_COMPLETE"
}
proc wtop_dc_synthesis {run scripts} {
    # Preserve hierarchy for an auditable CDC mapping; no retiming/gating/RTL edits.
    puts "WTOP_COMPILE_COMMAND=compile_ultra -no_autoungroup"
    if {![compile_ultra -no_autoungroup]} {error "WTOP compile failed"}
    set netlist [file join $run mapped mac_accel_dma_rob_top_mapped.v]
    set sdc [file join $run mapped mac_accel_dma_rob_top.sdc]
    write -format ddc -hierarchy -output [file join $run mapped mac_accel_dma_rob_top.ddc]
    write -format verilog -hierarchy -output $netlist
    write_sdc $sdc
    wtop_mapped_reports $run mapped
    set bindings [wtop_bind]
    wtop::validate_cdc $bindings
    wtop_audit_cdc $run mapped $bindings
    wtop_timing_visibility $run mapped $bindings
    # Independent readback into a fresh design in the same DC process.
    remove_design -all
    wtop_readback $run $netlist $sdc
}
