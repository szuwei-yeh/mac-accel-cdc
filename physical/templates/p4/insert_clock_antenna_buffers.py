import os,json,shutil
import openroad,odb
from pathlib import Path
r=Path(os.environ['P4_RUN']);old=r/'results/nangate45/mac_accel_dma_rob_top/hold50';out=r/'results/nangate45/mac_accel_dma_rob_top/antenna_split3';out.mkdir()
tech=openroad.Tech();tech.readLiberty('@FLOW@/platforms/nangate45/lib/NangateOpenCellLibrary_typical.lib');d=openroad.Design(tech);d.readDb(str(old/'5_1_grt.odb'));b=d.getBlock();root=b.findInst('clkbuf_0_S_AXI_ACLK');master=root.getMaster();assert master.getName()=='CLKBUF_X3';pin=root.findITerm('A');original=b.findNet('S_AXI_ACLK');assert pin.getNet()==original
prior=original;records=[];u=b.getDbUnitsPerMicron()
# Three identity buffers split the diagonal input route into four short segments.
# Coordinates are proposal locations; legalizer may refine them within the same floorplan.
for index,(x,y) in enumerate([(52.44,361.2),(104.69,310.8),(156.94,259.0)]):
 name=f'antenna_busclk_{index}';assert b.findInst(name) is None
 i=odb.dbInst_create(b,master,name);i.setOrient('R0');i.setLocation(round(x*u),round(y*u));i.setPlacementStatus('PLACED')
 net=odb.dbNet_create(b,f'antenna_busclk_net_{index}');net.setSigType('CLOCK');i.findITerm('A').connect(prior);i.findITerm('Z').connect(net)
 records.append(dict(instance=name,cell=master.getName(),proposal_um=[x,y],input_net=prior.getName(),output_net=net.getName()));prior=net
pin.disconnect();pin.connect(prior)
shutil.copyfile(old/'5_1_grt.sdc',out/'4_cts.sdc');shutil.copyfile(old/'1_2_yosys.v',out/'1_2_yosys.v')
d.evalTclString(f'read_sdc {out}/4_cts.sdc')
d.evalTclString('set_propagated_clock [all_clocks]')
d.evalTclString('global_connect')
d.evalTclString('detailed_placement')
d.evalTclString('check_placement -verbose')
d.evalTclString('global_connect')
d.evalTclString('write_db '+str(out/'4_cts.odb'))
d.evalTclString('write_verilog '+str(r/'exports/antenna-seed.v'))
for v in records:
 q=b.findInst(v['instance']).getBBox();v['legal_bbox_um']=[q.xMin()/u,q.yMin()/u,q.xMax()/u,q.yMax()/u]
(r/'audit/antenna-clock-buffer-repair.json').write_text(json.dumps(dict(method='Physical-only insertion of three qualified CLKBUF_X3 cells on S_AXI_ACLK input before the existing root; no clock/SDC/RTL change',baseline=str(old/'5_1_grt.odb'),buffers=records,status='SEEDED_LEGAL'),indent=2)+'\n')
print('P4_CLOCK_INPUT_BUFFER_SEED_PASS',records)
