import json,functools,collections,sys
from pathlib import Path
from run import *
stage=sys.argv[1]
allmod=json.loads((RUN/'audit'/f'{stage}-mapped.json').read_text())['modules'];t=allmod['mac_accel_dma_rob_top']
# OpenDB escapes brackets in identifiers; reconcile them with accepted Yosys names.
def norm(n):return n.replace('\\[','[').replace('\\]',']')
cs={norm(n):c for n,c in t['cells'].items()};assert len(cs)==len(t['cells'])
seq={n:c for n,c in cs.items() if c['type'].startswith('DFF')}
oldrun=Path(json.loads((RUN/'audit/baseline.json').read_text())['p2_run']);old=json.loads((oldrun/'audit/readback.json').read_text())['modules']['mac_accel_dma_rob_top']
oldclock={old['ports']['S_AXI_ACLK']['bits'][0]:'bus',old['ports']['mac_clk']['bits'][0]:'mac'}
expected={n:oldclock[c['connections']['CK'][0]] for n,c in old['cells'].items() if c['type'].startswith('DFF')}
assert set(seq)==set(expected),(set(seq)-set(expected),set(expected)-set(seq))
assert not [n for n,c in cs.items() if c['type'].startswith('$') or c['type'] not in allmod]
assert not [n for n,c in cs.items() if c['type'].startswith(('DLH','DLL')) or 'LATCH' in c['type']]
drivers={};sinks=collections.Counter()
for n,c in cs.items():
 for p,bs in c['connections'].items():
  for b in bs:
   if c['port_directions'][p]=='output':
    assert b not in drivers,(b,n,drivers.get(b));drivers[b]=(n,p)
   else:sinks[b]+=1
@functools.lru_cache(None)
def srcs(b):
 if b not in drivers:return frozenset()
 n,p=drivers[b]
 if n in seq:return frozenset([n])
 return frozenset().union(*(srcs(v) for po,vs in cs[n]['connections'].items() if cs[n]['port_directions'][po]=='input' for v in vs))
primary={b:(n,i) for n,p in t['ports'].items() if p['direction']=='input' for i,b in enumerate(p['bits'])}
@functools.lru_cache(None)
def clockroots(b):
 if b in primary:return frozenset([primary[b][0]])
 assert b in drivers,('Disconnected clock',b)
 n,p=drivers[b];assert n not in seq,('Sequential clock source',n)
 return frozenset().union(*(clockroots(v) for po,vs in cs[n]['connections'].items() if cs[n]['port_directions'][po]=='input' for v in vs))
for n,c in seq.items():assert clockroots(c['connections']['CK'][0])=={'S_AXI_ACLK' if expected[n]=='bus' else 'mac_clk'},n
fanin={n:srcs(c['connections']['D'][0]) for n,c in seq.items()}
g=json.loads((RUN/'audit/p2-mapping-gate.json').read_text());bind=g['bindings'];unclassified=[];actual=[];counts=collections.Counter()
for dst,ss in fanin.items():
 for source in ss:
  if expected[source]==expected[dst]:continue
  ids=[id for id in ['start','done','busy','write_gray','read_gray','payload','result','latency'] if source in bind[id+'.from'] and dst in bind[id+'.to']]
  if len(ids)!=1:unclassified.append([source,dst,ids])
  else:counts[ids[0]]+=1;actual.append((ids[0],source,dst))
assert not unclassified,unclassified
baseline={(x['group'],x['source'],x['destination']) for x in g['crossings']};assert set(actual)==baseline,(set(actual)-baseline,baseline-set(actual))
for c in g['synchronizer_chains']:
 a,b=c['stage1'],c['stage2'];assert fanin[b]=={a},(c,fanin[b]);assert [n for n,ss in fanin.items() if a in ss]==[b],c
unloaded_seq=[n for n,c in seq.items() if not any(sinks[b] or any(b in p['bits'] for p in t['ports'].values() if p['direction']=='output') for port in ['Q','QN'] for b in c['connections'].get(port,[]))];assert not unloaded_seq,unloaded_seq
# Physical-only cells and CTS dummy unloaded outputs are explicitly distinguished.
physical={n:c for n,c in cs.items() if c['type']=='TAPCELL_X1'}
result=dict(status='PASS',stage=stage,total_exported_cells=len(cs),functional_and_clock_cells=len(cs)-len(physical),physical_taps=len(physical),sequential_cells=len(seq),clock_domains=dict(collections.Counter(expected.values())),cell_types=dict(collections.Counter(c['type'] for c in cs.values())),synchronizer_chains=13,classified_pairs=dict(counts),unclassified=unclassified,unmapped=0,latches=0,unloaded_sequential=unloaded_seq,clock_roots='Exact baseline assignment for every register; combinational CTS buffers traced to top clock',gray_bit_aliases=g['gray_msb_aliases'])
(RUN/'audit'/f'{stage}-mapping-gate.json').write_text(json.dumps(result,indent=2)+'\n');print('P4_MAPPED_GRAPH_PASS',result)
