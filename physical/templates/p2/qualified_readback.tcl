read_liberty @FLOW@/platforms/nangate45/lib/NangateOpenCellLibrary_typical.lib
read_db @P2_RUN@/results/nangate45/mac_accel_dma_rob_top/base/1_synth.odb
read_sdc @P2_PHASE@/constraint.sdc
write_sdc -no_timestamp @P2_RUN@/results/nangate45/mac_accel_dma_rob_top/base/1_synth_qualified.sdc
write_db @P2_RUN@/results/nangate45/mac_accel_dma_rob_top/base/1_synth_qualified.odb
