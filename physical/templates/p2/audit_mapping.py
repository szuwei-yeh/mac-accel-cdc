import json,collections,functools
from run import RUN
j=json.loads((RUN/'audit/mapped.json').read_text());top=j['modules']['mac_accel_dma_rob_top'];cs=top['cells'];nets=top['netnames']
seq={n:c for n,c in cs.items() if c['type'].startswith('DFF')}
unknown=[n for n,c in cs.items() if c['type'].startswith('$') or c['type'] not in j['modules']]
latches=[n for n,c in cs.items() if c['type'].startswith(('DLH','DLL')) or 'LATCH' in c['type']]
assert not unknown and not latches,(unknown,latches)
clkbits={top['ports']['S_AXI_ACLK']['bits'][0]:'bus',top['ports']['mac_clk']['bits'][0]:'mac'}
domain={}
for n,c in seq.items():
 assert len(c['connections']['CK'])==1 and c['connections']['CK'][0] in clkbits,(n,c['connections']['CK'])
 domain[n]=clkbits[c['connections']['CK'][0]]
drivers={}
for n,c in cs.items():
 for p,vs in c['connections'].items():
  if c['port_directions'][p]=='output':
   for b in vs:
    assert b not in drivers,(b,n,drivers.get(b))
    drivers[b]=(n,p)
@functools.lru_cache(None)
def srcs(b):
 if b not in drivers:return frozenset()
 n,p=drivers[b]
 if n in seq:return frozenset([n])
 return frozenset().union(*(srcs(v) for po,vs in cs[n]['connections'].items() if cs[n]['port_directions'][po]=='input' for v in vs))
def bits(base,width=1):
 if base in nets:
  bs=nets[base]['bits'];assert len(bs)==width,(base,len(bs),width);return bs
 out=[]
 for i in range(width):
  key=f'{base}[{i}]'
  if key not in nets and key in ['u_fifo.wr_ptr_gray[4]','u_fifo.rd_ptr_gray[4]']:
   # Gray = bin XOR (bin >> 1): its MSB is exactly bin[4].
   # Yosys merged these same-clock/reset registers; inspect actual alias.
   key=key.replace('_gray','_bin')
  out.append(nets[key]['bits'][0])
 return out
def regs(base,width=1):
 out=[]
 for b in bits(base,width):
  ss=srcs(b);assert len(ss)==1,(base,b,ss)
  n=next(iter(ss));assert drivers[b][0]==n,(base,b,'Q not direct',drivers[b],n)
  out.append(n)
 assert len(set(out))==width,(base,'merged registers',out)
 return out
bindings={}
def group(id,a,b):bindings[id+'.from']=a;bindings[id+'.to']=b
for id,source,dest in [('start','start_toggle_bus','start_sync_mac'),('done','done_toggle_mac','done_sync_bus'),('busy','busy_mac','busy_sync_bus')]:group(id,regs(source),regs(dest,2)[:1])
for id,d in [('write_gray','wr'),('read_gray','rd')]:group(id,regs(f'u_fifo.{d}_ptr_gray',5),regs(f'u_fifo.{d}_ptr_gray_sync1',5))
mem=[r for w in range(16) for r in regs(f'u_fifo.mem[{w}]',33)]
group('payload',mem,regs('u_pe.a_in_r',16)+regs('u_pe.b_in_r',16)+regs('u_pe.last_in_r'))
group('result',regs('result_mac',32),regs('res_bus_reg',32))
group('latency',regs('latency_mac',32),regs('lat_bus_reg',32))
fanin={n:srcs(c['connections']['D'][0]) for n,c in seq.items()}
chains=[]
for base in ['start_sync_mac','done_sync_bus','busy_sync_bus']:
 a,b=regs(base,2);chains.append(dict(name=base,stage1=a,stage2=b))
for d in ['wr','rd']:
 a=regs(f'u_fifo.{d}_ptr_gray_sync1',5);b=regs(f'u_fifo.{d}_ptr_gray_sync2',5)
 chains.extend(dict(name=f'{d}_gray[{i}]',stage1=x,stage2=y) for i,(x,y) in enumerate(zip(a,b)))
for c in chains:
 assert fanin[c['stage2']]=={c['stage1']},(c,fanin[c['stage2']])
 assert domain[c['stage1']]==domain[c['stage2']]
 # Stage1 has exactly one sequential data fanout, the associated stage2.
 assert [n for n,ss in fanin.items() if c['stage1'] in ss]==[c['stage2']],c
for id in ['start','done','busy','write_gray','read_gray','result','latency']:
 for a,b in zip(bindings[id+'.from'],bindings[id+'.to']):assert fanin[b] & set(bindings[id+'.from'])=={a},(id,a,b)
for i,b in enumerate(bindings['payload.to']):
 bit=i+16 if i<16 else i-16 if i<32 else 32
 expected={mem[w*33+bit] for w in range(16)}
 assert fanin[b]&set(mem)==expected,('payload',i,b,fanin[b]&set(mem),expected)
crossings=[];classified=collections.Counter();unclassified=[]
for dst,ss in fanin.items():
 for source in ss:
  if domain[source]==domain[dst]:continue
  ids=[id for id in ['start','done','busy','write_gray','read_gray','payload','result','latency'] if source in bindings[id+'.from'] and dst in bindings[id+'.to']]
  if len(ids)!=1:unclassified.append([source,dst,ids])
  else:classified[ids[0]]+=1;crossings.append(dict(source=source,destination=dst,group=ids[0]))
assert not unclassified,unclassified
assert dict(classified)==dict(start=1,done=1,busy=1,write_gray=5,read_gray=5,payload=528,result=32,latency=32),classified
for id in ['start','done','busy','write_gray','read_gray','payload','result','latency']:
 expected=('bus','mac') if id in ['start','write_gray','payload'] else ('mac','bus')
 assert set(domain[n] for n in bindings[id+'.from'])=={expected[0]}
 assert set(domain[n] for n in bindings[id+'.to'])=={expected[1]}
counts=collections.Counter(c['type'] for c in cs.values());area=0
# Cell area is parsed independently from liberty by the readback/stat stage.
result=dict(gray_msb_aliases={'u_fifo.wr_ptr_gray[4]':'u_fifo.wr_ptr_bin[4]','u_fifo.rd_ptr_gray[4]':'u_fifo.rd_ptr_bin[4]'},status='PASS',cell_count=len(cs),sequential_count=len(seq),cell_types=dict(counts),clock_domains=dict(collections.Counter(domain.values())),unmapped=unknown,latches=latches,unclocked=0,bindings=bindings,synchronizer_chains=chains,classified_pairs=dict(classified),unclassified=unclassified,crossings=crossings)
(RUN/'audit/mapping-gate.json').write_text(json.dumps(result,indent=2)+'\n')
lines=['# Exact mapped cells obtained from Q/D connectivity, not name-pattern guesses.']
for k,ns in bindings.items():lines.extend([f'set p2_bindings({k}) {{'+ ' '.join('{'+n+'}' for n in ns)+'}',f'set p2_counts({k}) {len(ns)}'])
(RUN/'audit/cdc-bindings.tcl').write_text('\n'.join(lines)+'\n')
print('MAPPING_CDC_PASS',len(cs),'cells',len(seq),'regs',dict(classified),'13 chains')

# Independent STA enumerates every real crossing and every interstage check.
lines=[]
for c in crossings:lines.append('lappend p2_pairs {'+' '.join('{'+c[k]+'}' for k in ['group','source','destination'])+'}')
for c in chains:lines.append('lappend p2_chains {'+' '.join('{'+c[k]+'}' for k in ['name','stage1','stage2'])+'}')
(RUN/'audit/sta-pairs.tcl').write_text('\n'.join(lines)+'\n')
