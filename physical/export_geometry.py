import os,json,collections
import openroad
tech=openroad.Tech();tech.readLiberty(os.environ['MAC_LIB']);design=openroad.Design(tech);design.readDb(os.environ['MAC_ODB']);b=design.getBlock();u=b.getDbUnitsPerMicron()
def box(q):return [q.xMin()/u,q.yMin()/u,q.xMax()/u,q.yMax()/u]
instances={};nets=[]
for i in b.getInsts():
 m=i.getMaster();n=i.getName().replace('\\[','[').replace('\\]',']');t=m.getName()
 instances[n]=dict(master=t,box_um=box(i.getBBox()),category='filler' if t.startswith('FILLCELL') else 'tap' if t.startswith('TAPCELL') else 'standard',area_um2=m.getWidth()*m.getHeight()/u**2)
for n in b.getNets():
 if str(n.getSigType())!='CLOCK':continue
 pts=[]
 for it in n.getITerms():
  i=it.getInst();q=i.getBBox();pts.append(dict(instance=i.getName().replace('\\[','[').replace('\\]',']'),pin=it.getMTerm().getName(),direction=str(it.getIoType()),xy_um=[(q.xMin()+q.xMax())/2/u,(q.yMin()+q.yMax())/2/u]))
 nets.append(dict(name=n.getName(),terminals=pts))
json.dump(dict(source_odb=os.environ['MAC_ODB'],core_um=box(b.getCoreArea()),die_um=box(b.getDieArea()),instances=instances,clock_nets=nets),open(os.environ['MAC_GEOMETRY'],'w'))
print('INSTANCES',len(instances),'CLOCK_NETS',len(nets))
