set stage $::env(P3_RENDER_STAGE)
set res $::env(P3_RUN)/results/nangate45/mac_accel_dma_rob_top/pg_final
switch $stage {floorplan {set db 3_2_place_iop.odb} placement {set db 3_place.odb} cts {set db 4_cts.odb}}
read_liberty @FLOW@/platforms/nangate45/lib/NangateOpenCellLibrary_typical.lib
read_db $res/$db
proc p3_save_image {} {
 gui::fit
 gui::set_display_controls "*" visible false
 gui::set_display_controls "Layers/*" visible true
 gui::set_display_controls "Nets/*" visible true
 gui::set_display_controls "Shape Types/*" visible true
 gui::set_display_controls "Misc/Scale bar" visible true
 if {$::env(P3_RENDER_STAGE) ne "floorplan"} {gui::set_display_controls "Instances/*" visible true}
 set resolution [expr {double([[[ord::get_db_block] getDieArea] dy])/[ [ord::get_db_block] getDbUnitsPerMicron]/1600.0}]
 save_image -resolution $resolution $::env(P3_RUN)/images/$::env(P3_RENDER_STAGE).png
}
gui::show {p3_save_image} false
