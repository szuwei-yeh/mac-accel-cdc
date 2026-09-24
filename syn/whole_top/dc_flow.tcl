# Dedicated whole-top flow; all work, cache and outputs stay under RUN_ROOT.
set_app_var sh_continue_on_error false
proc wtop_main {} {
    global env WTOP_ELAB_PARAMS WTOP_TIME_UNIT WTOP_CDC_CELLS
    set run $env(RUN_ROOT)
    set input $env(INPUT_ROOT)
    set scripts [file join $input syn whole_top]
    source [file join $scripts profile.tcl]
    set lib /fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db
    file mkdir [file join $run reports] [file join $run mapped] [file join $run work]
    set_app_var search_path [list [file join $input rtl] [file dirname $lib]]
    set_app_var target_library [list $lib]
    set_app_var link_library [list * $lib]
    set_app_var alib_library_analysis_path [file join $run alib]
    define_design_lib WORK -path [file join $run work]
    if {$env(WTOP_STAGE) eq "readback"} {
        source [file join $scripts cdc_constraints.tcl]
        source [file join $scripts dc_constraints.tcl]
        source [file join $scripts dc_synthesis.tcl]
        wtop_readback $run [file join $input mapped mac_accel_dma_rob_top_mapped.v] \
            [file join $input mapped mac_accel_dma_rob_top.sdc]
        puts "WTOP_STAGE_COMPLETE=readback"
        return
    }
    set rtl {}
    foreach name {mac_accel_dma_rob_top.v mac_dma_rob.v axi_read_engine_rob.v mac_fifo_async.v mac_pe.v} {
        lappend rtl [file join $input rtl $name]
    }
    if {![analyze -format sverilog $rtl]} {error "analyze failed"}
    set params {}
    dict for {k v} $wtop::parameters {lappend params ${k}=${v}}
    set WTOP_ELAB_PARAMS $wtop::parameters
    puts "WTOP_ELABORATION_PARAMETERS=[join $params ,]"
    if {![elaborate $wtop::top -parameters [join $params ,]]} {error "elaborate failed"}
    # DC appends explicit parameter values to the elaborated design name.
    puts "WTOP_ELABORATED_DESIGN=[get_object_name [current_design]]"
    rename_design [current_design] $wtop::top
    current_design $wtop::top
    if {![link]} {error "link failed"}
    uniquify
    redirect -file [file join $run reports design.rpt] {check_design}
    redirect -file [file join $run reports hierarchy.rpt] {report_reference -hierarchy}
    redirect -file [file join $run reports library.rpt] {
        foreach attr {time_unit_name time_scale nom_process nom_voltage nom_temperature default_operating_conditions} {
            puts "$attr=[get_attribute [get_libs osu018_stdcells] $attr]"
        }
        report_design
    }
    redirect -file [file join $run reports lib_attributes.rpt] {list_attributes -application -class lib}
    redirect -file [file join $run reports sequential.rpt] {
        foreach_in_collection c [get_cells -hierarchical -filter "is_sequential == true"] {
            puts "[get_object_name $c] [get_attribute $c ref_name]"
        }
    }
    write -format ddc -hierarchy -output [file join $run mapped elaborated.ddc]
    write -format verilog -hierarchy -output [file join $run mapped elaborated.v]
    if {$env(WTOP_STAGE) eq "inspect"} {
        puts "WTOP_STAGE_COMPLETE=inspect"
        return
    }
    # Added only after real elaboration has established the binding names.
    source [file join $scripts dc_constraints.tcl]
    wtop_dc_constraints $run $scripts
    if {$env(WTOP_STAGE) eq "synthesis"} {
        source [file join $scripts dc_synthesis.tcl]
        wtop_dc_synthesis $run $scripts
    }
    puts "WTOP_STAGE_COMPLETE=$env(WTOP_STAGE)"
}
if {[catch {wtop_main} reason]} {
    puts stderr "WTOP_FATAL: $reason"
    puts stderr $::errorInfo
    exit 1
}
exit 0
