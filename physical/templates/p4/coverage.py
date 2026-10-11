import subprocess,json,collections,re,sys
from run import RUN,PHASE
stage=sys.argv[1]
sdc=RUN/'exports'/f'{stage}.sdc'
raw=subprocess.check_output(['/usr/bin/tclsh',str(PHASE/'coverage_records.tcl'),str(sdc)],text=True)
records=[];tokens={}
for line in raw.splitlines():
 fields=line.split('\t');args=[]
 for field in fields[1:]:
  value,parts=field.split(':',1);value=bytes.fromhex(value).decode();args.append(value)
  tokens[value]=[bytes.fromhex(x).decode() for x in parts.split(',')] if parts else []
 records.append((fields[0],args))
M=json.loads((RUN/'audit/p2-mapping-gate.json').read_text());T=json.loads((RUN/'audit'/f'{stage}-mapped.json').read_text())['modules']['mac_accel_dma_rob_top']
def ns(v):return {x.replace('\\[','[').replace('\\]',']') for x in tokens[v]}
def opt(a,key):return a[a.index(key)+1]
bind={k:set(v) for k,v in M['bindings'].items()}
inports={n if len(p['bits'])==1 else f'{n}[{i}]' for n,p in T['ports'].items() if p['direction']=='input' for i in range(len(p['bits']))};special={'S_AXI_ACLK','mac_clk','S_AXI_ARESETN','mac_rst'}
outports={n if len(p['bits'])==1 else f'{n}[{i}]' for n,p in T['ports'].items() if p['direction']=='output' for i in range(len(p['bits']))}
for kind,ports in [('set_input_delay',inports-special),('set_output_delay',outports)]:
 for mode,value in [('-min',0),('-max',1)]:
  matches=[a for k,a in records if k==kind and mode in a];coverage=set()
  for a in matches:
   assert float(a[0])==value and ns(opt(a,'-clock'))=={'bus_clk'}
   coverage.update(ns(a[-1]))
  assert coverage==ports,(kind,mode,ports-coverage,coverage-ports)
clocks=[a for k,a in records if k=='create_clock'];assert len(clocks)==2
for a in clocks:assert float(opt(a,'-period'))==({'bus_clk':10,'mac_clk':7.5}[opt(a,'-name')])
groups=[a for k,a in records if k=='set_clock_groups'];assert len(groups)==1 and '-allow_paths' in groups[0] and '-asynchronous' in groups[0]
maxima=[a for k,a in records if k=='set_max_delay'];assert len(maxima)==5
for id in ['write_gray','read_gray','payload','result','latency']:
 matches=[a for a in maxima if ns(opt(a,'-from'))==bind[id+'.from'] and ns(opt(a,'-to'))==bind[id+'.to']];assert len(matches)==1
 a=matches[0];assert '-ignore_clock_latency' in a and float(a[-1])== (10 if id in ['result','latency'] else 7.5)
false=[a for k,a in records if k=='set_false_path'];used=set()
for id in ['start','done','busy','write_gray','read_gray','payload','result','latency']:
 matches=[(i,a) for i,a in enumerate(false) if '-to' in a and ns(opt(a,'-from'))==bind[id+'.from'] and ns(opt(a,'-to'))==bind[id+'.to']];assert len(matches)==1
 i,a=matches[0];used.add(i);assert ('-hold' in a)==(id not in ['start','done','busy']) and '-setup' not in a
resets=set()
for i,a in enumerate(false):
 if i not in used:
  assert '-to' not in a and '-hold' not in a and '-setup' not in a
  resets.update(ns(opt(a,'-from')))
assert resets=={'S_AXI_ARESETN','mac_rst'}
log=(RUN/'console'/f'{stage}-timing-cdc-audit.log').read_text();coverage=log.split('CONSTRAINT_COVERAGE_BEGIN\n')[1].split('CONSTRAINT_COVERAGE_END')[0]
assert '2 input ports missing set_input_delay' in coverage and 'S_AXI_ARESETN' in coverage and 'mac_rst' in coverage
assert len(re.findall('Warning:',coverage))==1 and 'P4_PHYSICAL_AUDIT_COMPLETE' in log
fanout=[a for k,a in records if k=='set_max_fanout'];assert len(fanout)==1 and float(fanout[0][0])==32
lat=[a for k,a in records if k=='set_clock_latency'];assert len(lat)==2 and all('-source' in a and float(a[1])==0 for a in lat)
prop=[a for k,a in records if k=='set_propagated_clock'];assert prop and set().union(*(ns(a[0]) for a in prop))=={'bus_clk','mac_clk'}
summary=dict(status='PASS',input_bits_with_min_and_max=104,output_bits_with_min_and_max=93,excluded_reset_ports=sorted(resets),clock_ports=['S_AXI_ACLK','mac_clk'],clock_count=2,external_source_latency_ns=0,max_delay_groups=5,event_stage1_false_path_groups=3,bounded_hold_false_path_groups=5,broad_clock_cut=False,unclassified_exceptions=0,unconstrained_functional_endpoints=0,propagated_clocks=True,check_setup_warning=coverage.strip(),reset_coverage='reset arrival/recovery/removal/RDC intentionally excluded; safe external release assumed')
assert len(inports-special)==104 and len(outports)==93
(RUN/'audit'/f'{stage}-coverage-gate.json').write_text(json.dumps(summary,indent=2)+'\n');print('COVERAGE_PASS',summary)
