import json
from run import RUN
j=json.loads((RUN/'audit/elaborated.json').read_text())['modules']['mac_accel_dma_rob_top']
manifest=json.loads((RUN/'audit/source-manifest.json').read_text())
p={k:int(v,2) for k,v in j['parameter_default_values'].items()};assert p==manifest['parameters'],(p,manifest['parameters'])
m=j['memories'];assert (m['u_dma.buf_a']['size'],m['u_dma.buf_a']['width'])==(256,16)
assert (m['u_dma.u_read_engine.rob_data']['size'],m['u_dma.u_read_engine.rob_data']['width'])==(128,32)
assert all(m['u_dma.u_read_engine.'+n]['size']==8 for n in ['e_valid','e_complete','e_recv','e_exp'])
nets=j['netnames'];cells=j['cells'];drivers={}
for n,c in cells.items():
 for port,bits in c['connections'].items():
  if c['port_directions'][port]=='output':
   for i,b in enumerate(bits):drivers[b]=(n,c,i)
def seqsrc(b,visited=None):
 if not isinstance(b,int):return set()
 visited=set() if visited is None else visited
 if b in visited:return set()
 visited.add(b)
 n,c,i=drivers[b]
 if c['type'] in ('$dff','$adff'):return {(n,i)}
 return set().union(*(seqsrc(v,visited) for po,vs in c['connections'].items() if c['port_directions'][po]=='input' for v in vs))
chains=[]
for base in ['start_sync_mac','done_sync_bus','busy_sync_bus']:
 bs=nets[base]['bits'];chains.append((base,bs[0],bs[1]))
for direction in ['wr','rd']:
 a=nets[f'u_fifo.{direction}_ptr_gray_sync1']['bits'];b=nets[f'u_fifo.{direction}_ptr_gray_sync2']['bits']
 chains.extend((f'{direction}_gray[{i}]',x,y) for i,(x,y) in enumerate(zip(a,b)))
for name,a,b in chains:
 na,ca,ia=drivers[a];nb,cb,ib=drivers[b]
 assert ca['type'] in ('$dff','$adff') and cb['type'] in ('$dff','$adff')
 assert ca['connections']['CLK']==cb['connections']['CLK']
 assert seqsrc(cb['connections']['D'][ib])=={(na,ia)},name
for word in range(16):assert len(nets[f'u_fifo.mem[{word}]']['bits'])==33
out=dict(status='PASS',parameters=p,memories=m,fifo=dict(depth=16,width=33,bits=528),synchronizer_chains=[n for n,a,b in chains],chains=13,clock_ports=['S_AXI_ACLK','mac_clk'],hierarchy=['top','u_dma','u_dma.u_read_engine','u_fifo','u_pe'])
(RUN/'audit/elaboration-gate.json').write_text(json.dumps(out,indent=2)+'\n');print('ELABORATION_PASS',p,'13 chains; A buffer4096 ROB4096 FIFO528 bits')
