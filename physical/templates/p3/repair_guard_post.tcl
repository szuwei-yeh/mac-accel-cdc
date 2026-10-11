# Exact synchronizer structures and register-clock coverage before checkpointing.
set a [get_cells {start_sync_mac[0]$_DFF_PP0_}]
set b [get_cells {start_sync_mac[1]$_DFF_PP0_}]
set d [get_pins {start_sync_mac[1]$_DFF_PP0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: start_sync_mac"}
set a [get_cells {done_sync_bus[0]$_DFF_PN0_}]
set b [get_cells {done_sync_bus[1]$_DFF_PN0_}]
set d [get_pins {done_sync_bus[1]$_DFF_PN0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: done_sync_bus"}
set a [get_cells {busy_sync_bus[0]$_DFF_PN0_}]
set b [get_cells {busy_sync_bus[1]$_DFF_PN0_}]
set d [get_pins {busy_sync_bus[1]$_DFF_PN0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: busy_sync_bus"}
set a [get_cells {u_fifo.wr_ptr_gray_sync1[0]$_DFF_PP0_}]
set b [get_cells {u_fifo.wr_ptr_gray_sync2[0]$_DFF_PP0_}]
set d [get_pins {u_fifo.wr_ptr_gray_sync2[0]$_DFF_PP0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: wr_gray[0]"}
set a [get_cells {u_fifo.wr_ptr_gray_sync1[1]$_DFF_PP0_}]
set b [get_cells {u_fifo.wr_ptr_gray_sync2[1]$_DFF_PP0_}]
set d [get_pins {u_fifo.wr_ptr_gray_sync2[1]$_DFF_PP0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: wr_gray[1]"}
set a [get_cells {u_fifo.wr_ptr_gray_sync1[2]$_DFF_PP0_}]
set b [get_cells {u_fifo.wr_ptr_gray_sync2[2]$_DFF_PP0_}]
set d [get_pins {u_fifo.wr_ptr_gray_sync2[2]$_DFF_PP0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: wr_gray[2]"}
set a [get_cells {u_fifo.wr_ptr_gray_sync1[3]$_DFF_PP0_}]
set b [get_cells {u_fifo.wr_ptr_gray_sync2[3]$_DFF_PP0_}]
set d [get_pins {u_fifo.wr_ptr_gray_sync2[3]$_DFF_PP0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: wr_gray[3]"}
set a [get_cells {u_fifo.wr_ptr_gray_sync1[4]$_DFF_PP0_}]
set b [get_cells {u_fifo.wr_ptr_gray_sync2[4]$_DFF_PP0_}]
set d [get_pins {u_fifo.wr_ptr_gray_sync2[4]$_DFF_PP0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: wr_gray[4]"}
set a [get_cells {u_fifo.rd_ptr_gray_sync1[0]$_DFF_PN0_}]
set b [get_cells {u_fifo.rd_ptr_gray_sync2[0]$_DFF_PN0_}]
set d [get_pins {u_fifo.rd_ptr_gray_sync2[0]$_DFF_PN0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: rd_gray[0]"}
set a [get_cells {u_fifo.rd_ptr_gray_sync1[1]$_DFF_PN0_}]
set b [get_cells {u_fifo.rd_ptr_gray_sync2[1]$_DFF_PN0_}]
set d [get_pins {u_fifo.rd_ptr_gray_sync2[1]$_DFF_PN0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: rd_gray[1]"}
set a [get_cells {u_fifo.rd_ptr_gray_sync1[2]$_DFF_PN0_}]
set b [get_cells {u_fifo.rd_ptr_gray_sync2[2]$_DFF_PN0_}]
set d [get_pins {u_fifo.rd_ptr_gray_sync2[2]$_DFF_PN0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: rd_gray[2]"}
set a [get_cells {u_fifo.rd_ptr_gray_sync1[3]$_DFF_PN0_}]
set b [get_cells {u_fifo.rd_ptr_gray_sync2[3]$_DFF_PN0_}]
set d [get_pins {u_fifo.rd_ptr_gray_sync2[3]$_DFF_PN0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: rd_gray[3]"}
set a [get_cells {u_fifo.rd_ptr_gray_sync1[4]$_DFF_PN0_}]
set b [get_cells {u_fifo.rd_ptr_gray_sync2[4]$_DFF_PN0_}]
set d [get_pins {u_fifo.rd_ptr_gray_sync2[4]$_DFF_PN0_/D}]
set fanin [lsort -unique [get_fanin -to $d -flat -startpoints_only -only_cells]]
if {$fanin ne [lsort $a]} {error "Synchronizer chain changed: rd_gray[4]"}
if {[llength [all_registers -cells -clock bus_clk]]!=7281 || [llength [all_registers -cells -clock mac_clk]]!=222 || [llength [all_registers -cells]]!=7503} {error "Physical register/clock coverage changed"}
global_connect
source $::env(P3_PHASE)/protect.tcl
puts "P3_PHYSICAL_REPAIR_GUARD_PASS chains=13 registers=7503"
