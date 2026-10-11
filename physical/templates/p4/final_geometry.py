import os,json,collections
import openroad,odb
prefix=os.environ.get('P4_FINAL_PREFIX','final');r=os.environ['P4_RUN'];variant=os.environ['P4_VARIANT'];tech=openroad.Tech();tech.readLiberty('@FLOW@/platforms/nangate45/lib/NangateOpenCellLibrary_typical.lib');d=openroad.Design(tech);d.readDb(r+'/results/nangate45/mac_accel_dma_rob_top/'+variant+'/6_final.odb');b=d.getBlock();u=b.getDbUnitsPerMicron();core=b.getCoreArea();die=b.getDieArea()
def box(q):return [q.xMin()/u,q.yMin()/u,q.xMax()/u,q.yMax()/u]
area=(core.xMax()-core.xMin())*(core.yMax()-core.yMin())/u**2;counts=collections.Counter();cats=collections.Counter();areas=collections.Counter();protected=[];clock=collections.Counter();holds=[];signal_nets=0;route_nets=0;single_terminal=[]
for i in b.getInsts():
 m=i.getMaster();n=i.getName();t=m.getName();counts[t]+=1;a=m.getWidth()*m.getHeight()/u**2
 cat='filler' if t.startswith('FILLCELL') else 'tap' if t.startswith('TAPCELL') else 'standard'
 cats[cat]+=1;areas[cat]+=a
 if i.isDoNotTouch():protected.append(n)
 if n.startswith('clkbuf_'):clock['bus' if n.endswith('S_AXI_ACLK') else 'mac' if n.endswith('mac_clk') else 'unknown']+=1
 if n.startswith('hold'):holds.append(n)
for net in b.getNets():
 if str(net.getSigType()) in ['POWER','GROUND']:continue
 signal_nets+=1
 if net.getWire() is not None:route_nets+=1
 if len(net.getITerms())+len(net.getBTerms())<2:single_terminal.append(net.getName())
info=dict(core_um=box(core),die_um=box(die),core_area_um2=area,rows=len(b.getRows()),instances=len(b.getInsts()),master_counts=dict(counts),instance_categories=dict(cats),area_um2=dict(areas),standard_cell_utilization_percent=100*areas['standard']/area,all_instance_utilization_percent=100*sum(areas.values())/area,protected_cells=protected,clock_tree_buffers_by_root_name=dict(clock),hold_named_cells=len(holds),hold_cells=holds,signal_nets=signal_nets,signal_nets_with_wire=route_nets,single_terminal_signal_nets=single_terminal)
open(r+'/audit/'+prefix+'-geometry.json','w').write(json.dumps(info,indent=2)+'\n');print('P4_FINAL_GEOMETRY',cats,dict(areas),dict(clock),len(holds))
