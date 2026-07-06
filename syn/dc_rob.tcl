# Synopsys Design Compiler flow for the ROB/outstanding-read RTL.
#
# Environment overrides:
#   TOP=mac_dma_rob|axi_read_engine_rob
#   CLK_PERIOD=<ns>
#   TARGET_LIB=<path-to-standard-cell.db>

set script_dir [file normalize [file dirname [info script]]]
set repo_root  [file normalize [file join $script_dir ".."]]

if {[info exists ::env(TOP)]} {
    set TOP $::env(TOP)
} else {
    set TOP mac_dma_rob
}

if {[info exists ::env(CLK_PERIOD)]} {
    set CLK_PERIOD $::env(CLK_PERIOD)
} else {
    set CLK_PERIOD 10.0
}

if {[info exists ::env(TARGET_LIB)]} {
    set TARGET_LIB $::env(TARGET_LIB)
} else {
    set TARGET_LIB /fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db
}

set report_dir [file join $script_dir reports]
set mapped_dir [file join $script_dir mapped]
set work_dir   [file join $script_dir work]

file mkdir $report_dir
file mkdir $mapped_dir
file mkdir $work_dir

if {![file exists $TARGET_LIB]} {
    echo "ERROR: TARGET_LIB does not exist: $TARGET_LIB"
    exit 1
}

set_app_var search_path [list $repo_root [file join $repo_root rtl] [file dirname $TARGET_LIB]]
set_app_var target_library [list $TARGET_LIB]
set_app_var link_library [list * $TARGET_LIB]

define_design_lib WORK -path $work_dir

set RTL_FILES [list \
    [file join $repo_root rtl axi_read_engine_rob.v] \
    [file join $repo_root rtl mac_dma_rob.v] \
]

analyze -format sverilog $RTL_FILES
elaborate $TOP
current_design $TOP
link
uniquify

source [file join $script_dir constraints_rob.sdc]

check_design > [file join $report_dir "${TOP}_check_design.rpt"]

compile_ultra

report_qor > [file join $report_dir "${TOP}_qor.rpt"]
report_area -hierarchy > [file join $report_dir "${TOP}_area.rpt"]
report_timing -max_paths 20 -delay_type max > [file join $report_dir "${TOP}_timing.rpt"]
report_constraint -all_violators \
    -max_delay -min_delay -max_transition -max_capacitance -max_fanout \
    > [file join $report_dir "${TOP}_timing_constraints.rpt"]
report_constraint -all_violators > [file join $report_dir "${TOP}_all_constraints.rpt"]
report_cell > [file join $report_dir "${TOP}_cells.rpt"]

write -format ddc -hierarchy -output [file join $mapped_dir "${TOP}.ddc"]
write -format verilog -hierarchy -output [file join $mapped_dir "${TOP}_mapped.v"]
write_sdc [file join $mapped_dir "${TOP}.sdc"]

echo "DC synthesis complete for $TOP"
echo "Reports: $report_dir"
echo "Mapped outputs: $mapped_dir"
exit
