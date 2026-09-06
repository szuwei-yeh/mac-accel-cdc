# Reproducible synthesis baseline for the ROB/outstanding-read subsystem.
#
# Required environment:
#   RUN_ROOT=<absolute output directory>
#
# Optional environment:
#   TOP=mac_dma_rob|axi_read_engine_rob
#   MAX_OUTSTANDING=2|4|8
#   CLK_PERIOD=<ns>
#   TARGET_LIB=<path-to-standard-cell.db>

set script_dir [file normalize [file dirname [info script]]]
set repo_root  [file normalize [file join $script_dir ".."]]

set TOP mac_dma_rob
set MAX_OUTSTANDING 8
set CLK_PERIOD 10.0
set TARGET_LIB /fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db

if {[info exists ::env(TOP)]}             { set TOP $::env(TOP) }
if {[info exists ::env(MAX_OUTSTANDING)]} { set MAX_OUTSTANDING $::env(MAX_OUTSTANDING) }
if {[info exists ::env(CLK_PERIOD)]}      { set CLK_PERIOD $::env(CLK_PERIOD) }
if {[info exists ::env(TARGET_LIB)]}      { set TARGET_LIB $::env(TARGET_LIB) }

if {![info exists ::env(RUN_ROOT)]} {
    echo "ERROR: RUN_ROOT must name the output directory"
    exit 1
}
set run_root   [file normalize $::env(RUN_ROOT)]
set report_dir [file join $run_root reports]
set mapped_dir [file join $run_root mapped]
set work_dir   [file join $run_root work]

if {$TOP ni {mac_dma_rob axi_read_engine_rob}} {
    echo "ERROR: unsupported TOP: $TOP"
    exit 1
}
if {$MAX_OUTSTANDING ni {2 4 8}} {
    echo "ERROR: MAX_OUTSTANDING must be 2, 4, or 8"
    exit 1
}
if {![file exists $TARGET_LIB]} {
    echo "ERROR: TARGET_LIB does not exist: $TARGET_LIB"
    exit 1
}

file mkdir $report_dir
file mkdir $mapped_dir
file mkdir $work_dir

set_app_var search_path [list $repo_root [file join $repo_root rtl] [file dirname $TARGET_LIB]]
set_app_var target_library [list $TARGET_LIB]
set_app_var link_library [list * $TARGET_LIB]

define_design_lib WORK -path $work_dir

set RTL_FILES [list \
    [file join $repo_root rtl axi_read_engine_rob.v] \
    [file join $repo_root rtl mac_dma_rob.v] \
]

analyze -format sverilog $RTL_FILES
elaborate $TOP -parameters "MAX_OUTSTANDING=$MAX_OUTSTANDING"
link
uniquify

source [file join $script_dir constraints_rob.sdc]

redirect -file [file join $report_dir config_precompile.rpt] {
    echo "TOP=$TOP"
    echo "MAX_OUTSTANDING=$MAX_OUTSTANDING"
    echo "CLK_PERIOD=$CLK_PERIOD"
    echo "TARGET_LIB=$TARGET_LIB"
    echo "RTL_FILES=$RTL_FILES"
    report_clock
    report_reference -hierarchy
}

check_design > [file join $report_dir check_design.rpt]

compile_ultra

report_qor > [file join $report_dir qor.rpt]
report_area -hierarchy > [file join $report_dir area.rpt]
report_timing -delay_type max -path_type full_clock_expanded \
    -max_paths 10 -nworst 10 -input_pins -nets -transition_time \
    -capacitance -significant_digits 4 \
    > [file join $report_dir timing_top10_max.rpt]

set araddr_ports [get_ports -quiet M_AXI_ARADDR*]
if {[sizeof_collection $araddr_ports] > 0} {
    report_timing -delay_type max -path_type full_clock_expanded \
        -to $araddr_ports -max_paths 10 -nworst 10 -input_pins -nets \
        -transition_time -capacitance -significant_digits 4 \
        > [file join $report_dir timing_araddr_top10_max.rpt]
}

report_constraint -all_violators \
    -max_delay -min_delay -max_transition -max_capacitance -max_fanout \
    > [file join $report_dir timing_constraints.rpt]
report_constraint -all_violators > [file join $report_dir all_constraints.rpt]
report_cell > [file join $report_dir cells.rpt]
report_reference -hierarchy > [file join $report_dir hierarchy.rpt]

write -format ddc -hierarchy -output [file join $mapped_dir "${TOP}.ddc"]
write -format verilog -hierarchy -output [file join $mapped_dir "${TOP}_mapped.v"]
write_sdc [file join $mapped_dir "${TOP}.sdc"]

echo "DC baseline synthesis complete"
echo "TOP=$TOP MAX_OUTSTANDING=$MAX_OUTSTANDING CLK_PERIOD=$CLK_PERIOD"
echo "Reports: $report_dir"
exit
