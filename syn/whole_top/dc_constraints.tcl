# Exact register names confirmed by DC elaboration, not wildcard exceptions.
proc wtop_index_cells {} {
    set index [dict create]
    foreach_in_collection c [get_cells -hierarchical -filter {is_sequential == true && is_hierarchical == false}] {
        dict set index [get_object_name $c] $c
    }
    return $index
}
proc wtop_cells {index names} {
    set cells {}
    foreach name $names {
        if {![dict exists $index $name]} {error "WTOP binding missing: $name"}
        set cells [add_to_collection $cells [dict get $index $name]]
    }
    return $cells
}
proc wtop_bits {base width} {
    set result {}
    for {set i 0} {$i < $width} {incr i} {lappend result [format {%s[%d]} $base $i]}
    return $result
}
proc wtop_bind {} {
    set index [wtop_index_cells]
    set names [dict create \
        start.from {start_toggle_bus_reg} start.to {start_sync_mac_reg[0]} \
        done.from {done_toggle_mac_reg} done.to {done_sync_bus_reg[0]} \
        busy.from {u_pe/busy_reg} busy.to {busy_sync_bus_reg[0]} \
        write_gray.from [wtop_bits u_fifo/wr_ptr_gray_reg 5] \
        write_gray.to [wtop_bits u_fifo/wr_ptr_gray_sync1_reg 5] \
        read_gray.from [wtop_bits u_fifo/rd_ptr_gray_reg 5] \
        read_gray.to [wtop_bits u_fifo/rd_ptr_gray_sync1_reg 5] \
        result.from [wtop_bits u_pe/result_reg 32] \
        result.to [wtop_bits res_bus_reg_reg 32] \
        latency.from [wtop_bits u_pe/last_latency_reg 32] \
        latency.to [wtop_bits lat_bus_reg_reg 32]]
    set mem {}
    for {set word 0} {$word < 16} {incr word} {
        for {set bit 0} {$bit < 33} {incr bit} {
            lappend mem [format {u_fifo/mem_reg[%d][%d]} $word $bit]
        }
    }
    dict set names payload.from $mem
    dict set names payload.to [concat [wtop_bits u_pe/a_in_r_reg 16] \
        [wtop_bits u_pe/b_in_r_reg 16] {u_pe/last_in_r_reg}]
    set bindings [dict create]
    dict for {key list} $names {dict set bindings $key [wtop_cells $index $list]}
    return $bindings
}
proc wtop_intersect {a b} {return [remove_from_collection $a [remove_from_collection $a $b]]}
proc wtop_cell_fanin {cell} {
    set result {}
    foreach_in_collection pin [get_pins -of_objects $cell -filter {pin_direction == in}] {
        set result [add_to_collection $result [all_fanin -to $pin -flat -startpoints_only -only_cells -trace_arcs all]]
    }
    return $result
}

proc wtop_audit_cdc {run prefix bindings} {
    set bus [all_registers -clock bus_clk -cells]
    set mac [all_registers -clock mac_clk -cells]
    set seq [get_cells -hierarchical -filter {is_sequential == true && is_hierarchical == false}]
    set unclocked [remove_from_collection $seq [add_to_collection $bus $mac]]
    if {[sizeof_collection $unclocked]} {error "WTOP unclocked sequential cells: [get_object_name $unclocked]"}
    if {[sizeof_collection [wtop_intersect $bus $mac]]} {error "WTOP multiply-clocked registers"}
    set f [open [file join $run reports ${prefix}_cdc.rpt] w]
    puts $f "BUS_REGISTERS=[sizeof_collection $bus] MAC_REGISTERS=[sizeof_collection $mac] UNCLOCKED=0"
    dict for {key cells} $bindings {puts $f "BIND $key [lsort [get_object_name $cells]]"}
    set probe [index_collection [dict get $bindings payload.to] 0]
    puts $f "PAYLOAD_FANIN_PROBE [get_object_name $probe] [get_object_name [wtop_cell_fanin $probe]]"
    set seen [dict create]
    foreach {src_domain dst_domain src_regs dst_regs} [list bus mac $bus $mac mac bus $mac $bus] {
        set src_clock ${src_domain}_clk
        set output_pins [all_registers -clock $src_clock -output_pins]
        set endpoints [all_fanout -from $output_pins -flat -endpoints_only -only_cells -trace_arcs all]
        set dests [wtop_intersect $endpoints $dst_regs]
        puts $f "DIRECTION $src_domain->$dst_domain ENDPOINTS=[sizeof_collection $dests]"
        foreach_in_collection dst $dests {
            set name [get_object_name $dst]
            set fanin [wtop_cell_fanin $dst]
            set cross [wtop_intersect $fanin $src_regs]
            set allowed {}
            dict for {key cells} $bindings {
                if {![string match *.to $key]} {continue}
                if {[sizeof_collection [wtop_intersect $dst $cells]]} {
                    set id [string range $key 0 end-3]
                    set source [dict get $bindings ${id}.from]
                    if {[sizeof_collection [remove_from_collection $source $src_regs]]} {
                        error "WTOP wrong launch domain for $id"
                    }
                    set allowed [add_to_collection $allowed $source]
                    dict set seen $id 1
                }
            }
            set extra [remove_from_collection $cross $allowed]
            puts $f "PATH $name FROM [lsort [get_object_name $cross]]"
            if {[sizeof_collection $extra]} {
                close $f
                error "WTOP unclassified crossing into $name from [get_object_name $extra]"
            }
            if {![sizeof_collection $cross]} {error "WTOP empty crossing cone: $name"}
        }
    }
    foreach {id nfrom nto policy budget} $wtop::crossings {
        if {$prefix eq "precompile" && $id eq "payload"} {
            # Real elaboration evidence: the FIFO read mux is a MUX_OP and
            # DC's timing graph cone stops there. Do not call this a CDC PASS.
            set mux [get_cells -quiet u_fifo/C2269]
            if {[sizeof_collection $mux] != 1} {error "WTOP missing inspected FIFO mux"}
            set mux_ref [get_attribute $mux ref_name]
            puts "WTOP_FIFO_MUX_REFERENCE=$mux_ref"
            if {![string match *MUX_OP* $mux_ref]} {
                error "WTOP unexpected precompile FIFO mux representation: $mux_ref"
            }
            foreach_in_collection d [dict get $bindings payload.to] {
                if {![sizeof_collection [wtop_intersect [wtop_cell_fanin $d] $mux]]} {
                    error "WTOP payload destination does not trace to inspected MUX_OP"
                }
            }
            if {[sizeof_collection [remove_from_collection [dict get $bindings payload.from] $bus]] ||
                [sizeof_collection [remove_from_collection [dict get $bindings payload.to] $mac]]} {
                error "WTOP payload clock-domain mismatch"
            }
            puts $f "PAYLOAD_CONNECTIVITY_DEFERRED_UNMAPPED_MUX_OP; 528 bus sources, 33 MAC captures; post-map check mandatory"
            continue
        }
        if {![dict exists $seen $id]} {error "WTOP no actual crossing found for $id"}
        set src [dict get $bindings ${id}.from]
        set dst [dict get $bindings ${id}.to]
        set source_pins [get_pins -of_objects $src -filter {pin_direction == out}]
        set reachable [all_fanout -from $source_pins -flat -endpoints_only -only_cells -trace_arcs all]
        if {[sizeof_collection [remove_from_collection $dst $reachable]]} {
            error "WTOP unreachable destinations for $id"
        }
        foreach_in_collection d $dst {
            set name [get_object_name $d]
            set actual [lsort [get_object_name [wtop_intersect [wtop_cell_fanin $d] $src]]]
            switch $id {
                start {set expected {start_toggle_bus_reg}}
                done {set expected {done_toggle_mac_reg}}
                busy {set expected {u_pe/busy_reg}}
                write_gray - read_gray {set expected [list [string map {_sync1_reg _reg} $name]]}
                result {set expected [list [string map {res_bus_reg_reg u_pe/result_reg} $name]]}
                latency {set expected [list [string map {lat_bus_reg_reg u_pe/last_latency_reg} $name]]}
                payload {
                    if {[regexp {u_pe/a_in_r_reg\[([0-9]+)\]} $name unused bit]} {
                        incr bit 16
                    } elseif {[regexp {u_pe/b_in_r_reg\[([0-9]+)\]} $name unused bit]} {
                        # b occupies payload bits 15:0.
                    } elseif {$name eq "u_pe/last_in_r_reg"} {set bit 32
                    } else {error "WTOP unexpected payload destination $name"}
                    set expected {}
                    for {set word 0} {$word < 16} {incr word} {
                        lappend expected [format {u_fifo/mem_reg[%d][%d]} $word $bit]
                    }
                }
            }
            if {$actual ne [lsort $expected]} {error "WTOP CDC bit connectivity mismatch at $name: $actual"}
        }
    }
    set index [wtop_index_cells]
    set second_stages {start_sync_mac_reg[1] done_sync_bus_reg[1] busy_sync_bus_reg[1]}
    set second_stages [concat $second_stages [wtop_bits u_fifo/wr_ptr_gray_sync2_reg 5] [wtop_bits u_fifo/rd_ptr_gray_sync2_reg 5]]
    foreach name $second_stages {
        set expected [string map {_sync2_reg _sync1_reg} $name]
        if {$expected eq $name} {set expected [string map {[1] [0]} $name]}
        set actual [get_object_name [wtop_intersect [wtop_cell_fanin [dict get $index $name]] $seq]]
        if {[lsort $actual] ne [list $expected]} {error "WTOP synchronizer chain changed: $name <- $actual"}
        puts $f "SYNC_STAGE2 $name FROM $expected"
    }
    if {$prefix eq "precompile"} {
        puts $f "CDC_CLASSIFICATION_PARTIAL_UNMAPPED_PAYLOAD"
    } else {
        puts $f "CDC_CLASSIFICATION_PASS"
    }
    close $f
}

proc wtop_dc_constraints {run scripts} {
    global WTOP_ELAB_PARAMS WTOP_TIME_UNIT WTOP_CDC_CELLS
    set WTOP_TIME_UNIT [get_attribute [get_libs osu018_stdcells] time_unit_name]
    if {[get_attribute [get_libs osu018_stdcells] time_scale] != 1} {error "WTOP requires 1 ns unit"}
    set_operating_conditions -library osu018_stdcells typical
    set WTOP_CDC_CELLS [wtop_bind]
    source [file join $scripts portfolio.sdc]
    redirect -file [file join $run reports precompile_clocks.rpt] {report_clock; report_clock -groups}
    redirect -file [file join $run reports precompile_ports.rpt] {report_port -verbose}
    redirect -file [file join $run reports precompile_check_timing.rpt] {check_timing}
    redirect -file [file join $run reports precompile_exceptions.rpt] {report_timing_requirements -nosplit}
    redirect -file [file join $run reports precompile_ignored.rpt] {report_timing_requirements -ignored -nosplit}
    wtop_audit_cdc $run precompile $WTOP_CDC_CELLS
    write_sdc [file join $run mapped precompile.sdc]
    write -format ddc -hierarchy -output [file join $run mapped constrained.ddc]
}
