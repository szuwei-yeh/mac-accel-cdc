#!/usr/bin/env python3
"""Regenerate measured PPA/timing charts; requires matplotlib 3.10.8 and Pillow 11.1.0."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
HERE=Path(__file__).resolve().parent
s=json.loads((HERE/'summary.json').read_text());h=json.loads((HERE/'hold-analysis.json').read_text())
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'savefig.dpi':220,'pdf.fonttype':42,'svg.fonttype':'none'})
provenance={}
def save(fig,name,caption,source):
 fig.text(.01,.015,caption,fontsize=8,color='#475569');fig.savefig(out/(name+'.png'),bbox_inches='tight');fig.savefig(out/(name+'.pdf'),bbox_inches='tight');plt.close(fig);provenance[name]=dict(caption=caption,source=source)
colors=['#247da8','#8c5ba5'];x=np.arange(4);labels=['P2\nMapped','P3\nPlaced','P3\nCTS','P4\nRouted']
fig,axes=plt.subplots(1,2,figsize=(10,4));fig.subplots_adjust(bottom=.23,wspace=.3)
for c,col in zip(['bus_clk','mac_clk'],colors):
 for ax,mode,scale in zip(axes,['max','min'],[1,1000]):
  y=[q['timing'][c][mode]['wns_ns']*scale for q in s['stages']];ax.plot(x,y,'o-',color=col,label=c.replace('_',' '));
  for a,b in zip(x,y):
   offset=8 if c=='bus_clk' else -16
   if mode=='min' and a in [1,3]:offset=-18 if c=='bus_clk' else 8
   ax.annotate(f'{b:.3f}',(a,b),xytext=(0,offset),textcoords='offset points',ha='center',fontsize=8,color=col)
for ax,title,unit in zip(axes,['Setup margin','Hold margin'],['Worst slack (ns)','Worst slack (ps)']):ax.set_xticks(x,labels);ax.set_title(title,loc='left',fontweight='bold');ax.set_ylabel(unit);ax.grid(axis='y',alpha=.18);ax.legend(fontsize=8)
axes[0].set_ylim(5.75,8.5)
axes[1].set_ylim(-3,92)
axes[1].axhline(0,color='#555',lw=.6)
save(fig,'timing-comparison','Bus 10 ns / MAC 7.5 ns. P2: pre-layout; P3: estimated RC; P4: extracted RC. All TNS = 0 ns.','summary.json; preserved P2–P4 timing reports')
fig,axes=plt.subplots(1,2,figsize=(10,4));fig.subplots_adjust(bottom=.23,wspace=.3)
for ax,key,title,unit in [(axes[0],'standard_area_um2','Standard-cell area','Area (µm²)'),(axes[1],'standard_cells','Standard-cell count','Cells')]:
 vals=[q[key] for q in s['stages']];ax.bar(x,vals,color=['#879eac','#a2b4c0','#5588a5','#247da8'],width=.6)
 for a,b in zip(x,vals):ax.text(a,b+max(vals)*.015,f'{b:,.1f}' if 'area' in key else f'{b:,}',ha='center',fontsize=8)
 ax.set_ylim(0,max(vals)*1.14);ax.set_xticks(x,labels);ax.set_title(title,loc='left',fontweight='bold');ax.set_ylabel(unit);ax.grid(axis='y',alpha=.15)
save(fig,'area-comparison','Taps and fillers excluded. Standard cells include clock buffers, dummy CTS loads and hold buffers. Core = 172,038.160 µm².','summary.json; actual OpenDB inventories')
fig,axes=plt.subplots(1,2,figsize=(10,4));fig.subplots_adjust(bottom=.26,wspace=.32)
for j,ident in enumerate(['p3_worst','final_worst']):
 a=next(q for q in h['comparisons'] if q['id']==ident);d=a['deltas_ns'];vals=[1000*d['cell_delay_ns'],1000*d['wire_delay_ns'],-1000*d['capture_clock_early_ns'],-1000*d['clock_to_required_delta_ns'],1000*d['slack_ns']]
 axes[j].bar(np.arange(5),vals,color=['#247da8','#7eafc7','#b77a48','#b7a88d','#57916e']);axes[j].axhline(0,color='#555',lw=.6);axes[j].set_xticks(np.arange(5),['Cell\ndelay','Wire\ndelay','Capture\nclock','Hold\narc','Net slack\ngain']);axes[j].set_ylabel('Contribution (ps)');axes[j].set_title(('P3 worst endpoint: ROB[52][6]' if j==0 else 'P4 worst endpoint: ROB[37][1]'),loc='left',fontsize=10,fontweight='bold')
 for xx,v in enumerate(vals):axes[j].annotate(f'{v:+.3f}',(xx,v),xytext=(0,5 if v>=0 else 8 if v < -10 else -13),textcoords='offset points',ha='center',fontsize=8)
save(fig,'hold-decomposition','Matched input → D-pin paths; library hold contribution = −Δ(required − capture clock). Models change from estimated to extracted RC.','hold-analysis.json; read-only STA, saved P3/P4 ODB/SDC/SPEF')
