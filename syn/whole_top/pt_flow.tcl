# Consume the exact resolved SDC; do not source/apply portfolio.sdc again.
set_app_var sh_continue_on_error false
proc wtop_pt_step {name command} {
    set result [uplevel 1 $command]
    if {$result eq "0"} {error "WTOP_PT $name returned 0"}
    puts "WTOP_PT_STEP_PASS=$name"
}
proc wtop_pt_main {} {
    set run $::env(WTOP_PT_RUN)
    set input [file join $run inputs]
    set reports [file join $run reports]
    file mkdir $reports
    set scripts [file join $input scripts]
    set_app_var search_path [list [file join $input mapped] $input]
    set_app_var link_path [list * [file join $input library.db]]
    set_app_var link_create_black_boxes false
    set_app_var report_default_significant_digits 6
    wtop_pt_step read_verilog {read_verilog [file join $input mapped mac_accel_dma_rob_top_mapped.v]}
    wtop_pt_step link_design {link_design mac_accel_dma_rob_top}
    current_design mac_accel_dma_rob_top
    wtop_pt_step read_sdc {read_sdc [file join $input mapped mac_accel_dma_rob_top.sdc]}
    wtop_pt_step update_timing {update_timing}
    redirect -file [file join $reports check_timing.rpt] {check_timing -verbose}
    redirect -file [file join $reports analysis_coverage.rpt] {
        report_analysis_coverage -status_details {untested violated} -significant_digits 6 -nosplit
    }
    redirect -file [file join $reports clocks.rpt] {report_clock -attributes -nosplit; report_clock -groups -nosplit}
    redirect -file [file join $reports clock_skew.rpt] {report_clock -skew -nosplit}
    redirect -file [file join $reports ports.rpt] {report_port -verbose -nosplit}
    redirect -file [file join $reports design.rpt] {report_design}
    redirect -file [file join $reports units.rpt] {report_units}
    redirect -file [file join $reports constraints.rpt] {report_constraint -all_violators -nosplit}
    redirect -file [file join $reports disabled_timing.rpt] {report_disable_timing -nosplit}
    redirect -file [file join $reports case_analysis.rpt] {report_case_analysis -all}
    redirect -file [file join $reports exceptions.rpt] {report_exceptions -nosplit}
    redirect -file [file join $reports ignored_exceptions.rpt] {report_exceptions -ignored -nosplit}
    foreach delay {max min} {
        redirect -file [file join $reports timing_${delay}.rpt] {
            report_timing -delay_type $delay -path_type full_clock_expanded \
                -max_paths 20 -nworst 1 -slack_lesser_than 1e9 -input_pins -nets \
                -transition_time -capacitance -significant_digits 6 -nosplit
        }
        redirect -file [file join $reports global_${delay}.rpt] {
            report_global_timing -delay_type $delay -separate_all_groups -significant_digits 6
        }
    }
    redirect -file [file join $reports configuration.rpt] {
        puts "TOP=[get_object_name [current_design]]"
        puts "LIBRARIES=[get_object_name [get_libs *]]"
        puts "LINK_PATH=[get_app_var link_path]"
        puts "MAX_FANOUT=[get_attribute [current_design] max_fanout]"
        puts "NO_RTL_ELABORATION: MAX_OUTSTANDING=8 established by accepted mapped input provenance"
        list_licenses
    }
    # Load helper definitions only. No DC command or synthesis is called here.
    source [file join $scripts profile.tcl]
    source [file join $scripts cdc_constraints.tcl]
    source [file join $scripts dc_constraints.tcl]
    source [file join $scripts dc_synthesis.tcl]
    source [file join $scripts pt_checks.tcl]
    wtop_pt_checks $run
    write_sdc [file join $reports effective_constraints.sdc]
    puts "WTOP_PT_FLOW_COMPLETE"
}
if {[catch {wtop_pt_main} reason]} {
    puts stderr "WTOP_PT_FATAL: $reason"
    puts stderr $::errorInfo
    exit 1
}
exit 0
