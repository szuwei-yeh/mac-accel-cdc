source $::env(P3_PHASE)/protect.tcl
unset_dont_touch [get_cells $p3_sync_cells]
puts "P3_SYNC_TEMPORARILY_UNLOCKED_FOR_PG_CONNECTIONS_ONLY"
