import json,re,collections,sys,csv
from run import *
prefix=sys.argv[1] if len(sys.argv)>1 else 'final'
independent=sys.argv[2] if len(sys.argv)>2 else 'independentsta'
modules=json.loads((RUN/'audit'/f'{prefix}-mapped.json').read_text())['modules'];model=modules['mac_accel_dma_rob_top']
def norm(s):return s.replace('\\[','[').replace('\\]',']')
cells={norm(n):c for n,c in model['cells'].items()};ports=model['ports'];sinks=collections.Counter()
for c in cells.values():
 for p,bs in c['connections'].items():
  if c['port_directions'][p]=='input':sinks.update(bs)
for p in ports.values():
 if p['direction']=='output':sinks.update(p['bits'])
primary={n if len(p['bits'])==1 else f'{n}[{i}]':b for n,p in ports.items() if p['direction']=='input' for i,b in enumerate(p['bits'])}
logs=[f'{prefix}-timing-cdc-audit.log',f'{independent}-timing-cdc-audit.log'];checks=[]
for filename in logs:
 text=(RUN/'console'/filename).read_text();section=text.split('PARASITIC_ANNOTATION_BEGIN\n')[1].split('PARASITIC_ANNOTATION_END')[0]
 match=re.search(r'Found (\d+) unannotated drivers\.\n(.*?)Found (\d+) partially unannotated drivers\.',section,re.S);assert match,section[:200]
 assert int(match.group(3))==0,'Partial SPEF annotation'
 names=[norm(n.strip()) for n in match.group(2).splitlines() if n.strip()];assert len(names)==int(match.group(1))
 categories=collections.Counter();unexpected=[]
 for name in names:
  if name in primary:
   if sinks[primary[name]]:unexpected.append(name)
   else:categories['unused_primary_input']+=1
  else:
   inst,pin=name.rsplit('/',1)
   if inst not in cells or pin not in modules[cells[inst]['type']]['ports'] or modules[cells[inst]['type']]['ports'][pin]['direction']!='output':unexpected.append(name);continue
   bits=cells[inst]['connections'].get(pin,[])
   if any(sinks[b] for b in bits):unexpected.append(name)
   else:categories['unloaded_library_output']+=1
 assert not unexpected,unexpected[:30]
 checks.append(dict(log=filename,unannotated_drivers=len(names),partially_unannotated=0,unannotated_active_drivers=0,expected_categories=dict(categories),expected_driver_names=names))
# Independent STA uses the saved final Verilog/SDC/SPEF, not the physical database.
a=(RUN/'console'/logs[0]).read_text();b=(RUN/'console'/logs[1]).read_text()
def timing(s):
 w={(c,m):float(v) for c,m,v in re.findall(r'DOMAIN_WNS (\w+) (max|min) ([-\d.]+)',s)}
 t={(c,m):(int(n),float(v)) for c,m,n,v in re.findall(r'DOMAIN_VIOLATIONS (\w+) (max|min) endpoints=(\d+) tns_ns=([-\d.eE]+)',s)}
 return dict(wns={f'{c}.{m}':v for (c,m),v in w.items()},violations={f'{c}.{m}':v for (c,m),v in t.items()})
assert timing(a)==timing(b),(timing(a),timing(b))
pa=list(csv.DictReader((RUN/'audit'/f'{prefix}-cdc-paths.tsv').open(),delimiter='\t'));pb=list(csv.DictReader((RUN/'audit'/f'{independent}-cdc-paths.tsv').open(),delimiter='\t'));assert pa==pb,'CDC timing readback mismatch'
sa=list(csv.DictReader((RUN/'audit'/f'{prefix}-synchronizers.tsv').open(),delimiter='\t'));sb=list(csv.DictReader((RUN/'audit'/f'{independent}-synchronizers.tsv').open(),delimiter='\t'));assert len(sa)==len(sb)==13
for x,y in zip(sa,sb):assert {k:v for k,v in x.items() if k!='manhattan_um'}=={k:v for k,v in y.items() if k!='manhattan_um'}
for s in [a,b]:assert s.split('ELECTRICAL_BEGIN\n')[1].split('ELECTRICAL_END')[0].strip()=='' and 'P4_PHYSICAL_AUDIT_COMPLETE' in s
assert set(checks[0]['expected_driver_names'])==set(checks[1]['expected_driver_names'])
result=dict(status='PASS',annotation=checks,independent_timing_identical=True,timing=timing(a),all_605_cdc_rows_identical=True,all_26_interstage_slacks_identical=True,electrical_zero_both=True,spef='Extracted OpenRCX final routed SPEF; no placement/global-route fallback',readback='Standalone OpenSTA on saved final netlist/SDC/SPEF')
(RUN/'audit'/f'{prefix}-annotation-consistency-gate.json').write_text(json.dumps(result,indent=2)+'\n');print('P4_ANNOTATION_INDEPENDENT_STA_PASS',timing(a),[(x['unannotated_drivers'],x['expected_categories']) for x in checks])
