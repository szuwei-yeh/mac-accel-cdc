#!/usr/bin/env python3
"""Plot actual OpenDB geometry exported by export_geometry.py; no illustrative layout."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection,LineCollection
p=argparse.ArgumentParser();p.add_argument('--geometry',type=Path,required=True);p.add_argument('--stage',choices=['placement','cts'],required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
out=a.output;out.mkdir(parents=True,exist_ok=False)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'savefig.dpi':220,'pdf.fonttype':42})
def save(fig,name,caption,source):
 fig.text(.01,.015,caption,fontsize=8,color='#475569');fig.savefig(out/(name+'.png'),bbox_inches='tight');fig.savefig(out/(name+'.pdf'),bbox_inches='tight');plt.close(fig)
for stage in [a.stage]:
 d=json.loads((a.geometry).read_text());fig,ax=plt.subplots(figsize=(7.5,8));fig.subplots_adjust(bottom=.13);patches=[];cs=[]
 for n,i in d['instances'].items():
  if i['category']!='standard':continue
  box=i['box_um'];patches.append(Rectangle((box[0],box[1]),box[2]-box[0],box[3]-box[1]));cs.append('#247da8' if i['master'].startswith('DFF') else '#d7e0e6')
 ax.add_collection(PatchCollection(patches,facecolor=cs,edgecolor='none',rasterized=True))
 if stage=='cts':
  segments=[]
  for net in d['clock_nets']:
   pts=net['terminals'];drivers=[p for p in pts if p['direction']=='OUTPUT'];sinks=[p for p in pts if p['direction']=='INPUT']
   for a in drivers:
    for b in sinks:
     if b['pin']=='A' and (b['instance'].startswith('clkbuf_') or b['instance'].startswith('antenna_')):segments.append([a['xy_um'],b['xy_um']])
  ax.add_collection(LineCollection(segments,colors='#b67732',linewidths=.45,alpha=.85));
  q=[i for n,i in d['instances'].items() if n.startswith('clkbuf_')];ax.scatter([(i['box_um'][0]+i['box_um'][2])/2 for i in q],[(i['box_um'][1]+i['box_um'][3])/2 for i in q],s=1,color='#b67732',zorder=4)
 ax.set_xlim(0,417.39);ax.set_ylim(0,417.39);ax.set_aspect('equal');ax.set_xlabel('x (µm)');ax.set_ylabel('y (µm)');ax.set_title('V3 '+('placed standard cells' if stage=='placement' else 'clock-tree buffer placement'),loc='left',fontweight='bold')
 save(fig,stage,'Actual saved '+stage+' ODB. Blue: registers; gray: combinational cells.'+(' Gold: CTS buffers and buffer-to-buffer connectivity (not routed wires).' if stage=='cts' else ''),f'P3 checkpoints/{stage}/{stage}.odb; geometry from native OpenDB')
