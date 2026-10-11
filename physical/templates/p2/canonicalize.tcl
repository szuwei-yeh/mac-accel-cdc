write_rtlil $::env(P2_RUN)/audit/elaborated-hierarchy.rtlil
flatten
# External metadata overlay; no functional RTL changes.
foreach wire {start_sync_mac done_sync_bus busy_sync_bus u_fifo.wr_ptr_gray_sync1 u_fifo.wr_ptr_gray_sync2 u_fifo.rd_ptr_gray_sync1 u_fifo.rd_ptr_gray_sync2} {
 select -assert-count 1 w:$wire
 setattr -set keep 1 -set async_reg 1 w:$wire
}
select -clear
yosys proc
write_json $::env(P2_RUN)/audit/elaborated.json
