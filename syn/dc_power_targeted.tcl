# Symmetric baseline/targeted clock-gated DC + Power Compiler experiment.
# Required: RUN_ROOT, MODE={baseline|gated}, ACTIVE_SAIF, IDLE_SAIF.

foreach required {RUN_ROOT MODE ACTIVE_SAIF IDLE_SAIF} {
    if {![info exists ::env($required)]} {
        echo "ERROR: missing environment variable $required"
        exit 2
    }
}

set run_root [file normalize $::env(RUN_ROOT)]
set mode $::env(MODE)
set active_saif [file normalize $::env(ACTIVE_SAIF)]
set idle_saif [file normalize $::env(IDLE_SAIF)]
set CLK_PERIOD 10.0
set target_lib /fs/ece/tech/osu_soc_2.7/synopsys/lib/tsmc018/osu018_stdcells.db
set report_dir [file join $run_root reports]
set mapped_dir [file join $run_root mapped]
set work_dir [file join $run_root work]
set input_dir [file join $run_root inputs]
file mkdir $report_dir
file mkdir $mapped_dir
file mkdir $work_dir

if {$mode ni {baseline gated}} { echo "ERROR: MODE must be baseline or gated"; exit 2 }
foreach input [list \
    $target_lib $active_saif $idle_saif \
    [file join $input_dir axi_read_engine_rob.v] \
    [file join $input_dir mac_dma_rob.v] \
    [file join $input_dir constraints_rob.sdc]] {
    if {![file exists $input]} { echo "ERROR: missing input $input"; exit 2 }
}

set_app_var search_path [list $input_dir [file dirname $target_lib]]
set_app_var target_library [list $target_lib]
set_app_var link_library [list * $target_lib]
define_design_lib WORK -path $work_dir

# Start before HDL read so RTL names survive optimization for backward SAIF.
saif_map -start
analyze -format sverilog [list \
    [file join $input_dir axi_read_engine_rob.v] \
    [file join $input_dir mac_dma_rob.v]]
elaborate mac_dma_rob -parameters "MAX_OUTSTANDING=8"
link
uniquify
source [file join $input_dir constraints_rob.sdc]
if {[sizeof_collection [get_clocks -quiet clk]] != 1} {
    echo "ERROR: canonical clk constraint was not created"
    exit 2
}

set pre_buf_regs [get_cells -hierarchical -quiet "*buf_a_reg*"]
set pre_all_regs [all_registers]
set pre_non_buf_regs [remove_from_collection $pre_all_regs $pre_buf_regs]
redirect -file [file join $report_dir config_precompile.rpt] {
    echo "TOP=mac_dma_rob"
    echo "MAX_OUTSTANDING=8"
    echo "CLK_PERIOD=10.0"
    echo "TARGET_LIB=$target_lib"
    echo "MODE=$mode"
    echo "ACTIVE_SAIF=$active_saif"
    echo "IDLE_SAIF=$idle_saif"
    echo "PRECOMPILE_BUF_A_OBJECTS=[sizeof_collection $pre_buf_regs]"
    echo "PRECOMPILE_ALL_REG_OBJECTS=[sizeof_collection $pre_all_regs]"
    echo "PRECOMPILE_EXCLUDED_REG_OBJECTS=[sizeof_collection $pre_non_buf_regs]"
    report_clock
    report_reference -hierarchy
}
check_design > [file join $report_dir check_design_precompile.rpt]

if {$mode eq "gated"} {
    if {[sizeof_collection $pre_buf_regs] == 0} {
        echo "ERROR: no buf_a register objects found before compile"
        exit 2
    }
    # Explicitly exclude every register outside buf_a, then allow ordinary
    # clock-gating eligibility only on the 4,096 buf_a bits.
    set_clock_gating_objects -exclude $pre_non_buf_regs
    set_clock_gating_objects -include $pre_buf_regs
    set_clock_gating_style \
        -sequential_cell latch:osu018_stdcells/LATCH \
        -positive_edge_logic {and:osu018_stdcells/AND2X1} \
        -minimum_bitwidth 16 \
        -max_fanout 16 \
        -no_sharing \
        -control_point none
    redirect -file [file join $report_dir clock_gating_scope_precompile.rpt] {
        echo "EXPLICIT_NON_BUF_REGISTERS_EXCLUDED=[sizeof_collection $pre_non_buf_regs]"
        echo "ONLY_INCLUDED_PATTERN=buf_a_reg[*]"
        echo "INCLUDED_OBJECT_COUNT=[sizeof_collection $pre_buf_regs]"
        report_clock_gating -style
    }
    compile_ultra -gate_clock
} else {
    compile_ultra
}

report_qor > [file join $report_dir qor.rpt]
report_area -hierarchy > [file join $report_dir area.rpt]
report_cell > [file join $report_dir cells.rpt]
report_reference -hierarchy > [file join $report_dir hierarchy.rpt]
report_timing -delay_type max -path_type full_clock_expanded \
    -max_paths 20 -nworst 20 -input_pins -nets -transition_time \
    -capacitance -significant_digits 4 \
    > [file join $report_dir timing_top20_setup.rpt]
report_timing -delay_type min -path_type full_clock_expanded \
    -max_paths 20 -nworst 20 -input_pins -nets -transition_time \
    -capacitance -significant_digits 4 \
    > [file join $report_dir timing_top20_hold.rpt]
report_constraint -all_violators > [file join $report_dir all_constraint_violators.rpt]
report_constraint -all_violators -max_delay -min_delay -max_transition \
    -max_capacitance -max_fanout \
    > [file join $report_dir timing_constraint_violators.rpt]

report_clock_gating > [file join $report_dir clock_gating_summary.rpt]
report_clock_gating -verbose -gating_elements -nosplit \
    > [file join $report_dir clock_gating_elements.rpt]
report_clock_gating -gated -nosplit > [file join $report_dir clock_gating_gated_regs.rpt]
report_clock_gating -ungated -nosplit > [file join $report_dir clock_gating_ungated_regs.rpt]
report_clock_gating_check -nosplit -significant_digits 4 \
    > [file join $report_dir clock_gating_checks.rpt]

saif_map -write_map [file join $mapped_dir rtl_to_mapped.saifmap]
write -format ddc -hierarchy -output [file join $mapped_dir mac_dma_rob.ddc]
write -format verilog -hierarchy -output [file join $mapped_dir mac_dma_rob_mapped.v]
write_sdc [file join $mapped_dir mac_dma_rob.sdc]
write_sdf [file join $mapped_dir mac_dma_rob.sdf]

# Active job power.
set active_status [read_saif -input $active_saif \
    -instance_name tb_mac_dma_rob_power/dut -auto_map_names -verbose]
redirect -file [file join $report_dir saif_active_status.rpt] {
    echo "READ_SAIF_STATUS=$active_status"
    report_saif -rtl_saif
    report_saif -hierarchy
}
report_saif -hierarchy -missing > [file join $report_dir saif_active_missing.rpt]
report_saif -rtl_saif -only [get_cells -hierarchical -quiet "*buf_a_reg*"] \
    > [file join $report_dir saif_active_buf_a.rpt]
report_power -analysis_effort high -verbose > [file join $report_dir power_active.rpt]
report_power -analysis_effort high -hierarchy -levels 3 \
    > [file join $report_dir power_active_hierarchy.rpt]
report_power -analysis_effort high -cell -nworst 500 -sort_mode dynamic_power \
    > [file join $report_dir power_active_cells.rpt]

# Remove every prior annotation before applying the independent idle window.
reset_switching_activity
set idle_status [read_saif -input $idle_saif \
    -instance_name tb_mac_dma_rob_power/dut -auto_map_names -verbose]
redirect -file [file join $report_dir saif_idle_status.rpt] {
    echo "READ_SAIF_STATUS=$idle_status"
    report_saif -rtl_saif
    report_saif -hierarchy
}
report_saif -hierarchy -missing > [file join $report_dir saif_idle_missing.rpt]
report_saif -rtl_saif -only [get_cells -hierarchical -quiet "*buf_a_reg*"] \
    > [file join $report_dir saif_idle_buf_a.rpt]
report_power -analysis_effort high -verbose > [file join $report_dir power_idle.rpt]
report_power -analysis_effort high -hierarchy -levels 3 \
    > [file join $report_dir power_idle_hierarchy.rpt]
report_power -analysis_effort high -cell -nworst 500 -sort_mode dynamic_power \
    > [file join $report_dir power_idle_cells.rpt]

echo "POWER_EXPERIMENT_COMPLETE mode=$mode run_root=$run_root"
exit
