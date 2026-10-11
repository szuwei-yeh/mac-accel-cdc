# Protection is restored at checkpoints; repair may reconnect reset/clock loads.
source $::env(P4_PHASE)/protect.tcl
unset_dont_touch [get_cells $p3_sync_cells]
puts "P4_SYNC_UNLOCKED_DURING_PHYSICAL_REPAIR; no retiming/gating/cloning"
