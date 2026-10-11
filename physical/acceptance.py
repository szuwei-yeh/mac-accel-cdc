#!/usr/bin/env python3
"""Fail-closed acceptance of research CDC and physical evidence; no P&R execution."""
import argparse,collections,csv,functools,hashlib,json,math,re,sys
from pathlib import Path
import xml.etree.ElementTree as ET
GROUPS=dict(start=1,done=1,busy=1,write_gray=5,read_gray=5,payload=528,result=32,latency=32)
EVENTS={'start','done','busy'}
CHAINS={'start_sync_mac','done_sync_bus','busy_sync_bus'}|{f'{d}_gray[{i}]' for d in ['wr','rd'] for i in range(5)}
class Rejected(ValueError):pass
def require(ok,reason):
 if not ok:raise Rejected(reason)
def load(p):
 try:return json.loads(Path(p).read_text())
 except (OSError,ValueError) as e:raise Rejected('Missing/invalid JSON: '+str(p)) from e
def sha(p):
 try:return hashlib.sha256(Path(p).read_bytes()).hexdigest()
 except OSError as e:raise Rejected('Missing/unreadable evidence: '+str(p)) from e
def finite(v,where,nonnegative=True):
 try:q=float(v)
 except (ValueError,TypeError) as e:raise Rejected('Invalid number '+where) from e
 require(math.isfinite(q) and (q>=0 or not nonnegative),'Nonfinite/negative value '+where)
 return q
def norm(n):return n.replace(chr(92)+'[','[').replace(chr(92)+']',']')
def table(p,required):
 try:
  with Path(p).open() as f:
   r=csv.DictReader(f,delimiter='\t');require(r.fieldnames is not None and set(required)<=set(r.fieldnames),'Missing table columns');rows=list(r)
 except OSError as e:raise Rejected('Missing/unreadable CDC table') from e
 require(all(None not in q and all(v is not None for v in q.values()) for q in rows),'Malformed table row')
 return rows

def graph_check(mapped,baseline,readback):
 modules=load(mapped)['modules'];t=modules['mac_accel_dma_rob_top'];cells={norm(n):c for n,c in t['cells'].items()}
 require(len(cells)==len(t['cells']),'Ambiguous escaped instance names')
 require(all(not c['type'].startswith('$') and c['type'] in modules and not c['type'].startswith(('DLH','DLL')) and 'LATCH' not in c['type'] for c in cells.values()),'Unmapped/black-box/latch cell')
 seq={n:c for n,c in cells.items() if c['type'].startswith('DFF')};old=load(readback)['modules']['mac_accel_dma_rob_top'];oldclk={old['ports']['S_AXI_ACLK']['bits'][0]:'bus',old['ports']['mac_clk']['bits'][0]:'mac'}
 expected={norm(n):oldclk[c['connections']['CK'][0]] for n,c in old['cells'].items() if c['type'].startswith('DFF')}
 require(set(seq)==set(expected) and len(seq)==7503,'Sequential identity/coverage changed')
 require(dict(collections.Counter(expected.values()))=={'bus':7281,'mac':222},'Domain register inventory')
 drivers={};sinks=collections.Counter()
 for n,c in cells.items():
  for pin,bits in c['connections'].items():
   for b in bits:
    if c['port_directions'][pin]=='output':require(b not in drivers,'Multiple drivers');drivers[b]=(n,pin)
    else:sinks[b]+=1
 primary={b:n for n,p in t['ports'].items() if p['direction']=='input' for b in p['bits']}
 @functools.lru_cache(None)
 def sources(b,clock=False):
  if clock and b in primary:return frozenset([primary[b]])
  if b not in drivers:
   require(not clock,'Unresolved clock endpoint');return frozenset()
  n,pin=drivers[b]
  if n in seq:
   require(not clock,'Sequential clock source');return frozenset([n])
  return frozenset().union(*(sources(v,clock) for po,vs in cells[n]['connections'].items() if cells[n]['port_directions'][po]=='input' for v in vs))
 for n,c in seq.items():require(sources(c['connections']['CK'][0],True)=={'S_AXI_ACLK' if expected[n]=='bus' else 'mac_clk'},'Clock root changed '+n)
 fanin={n:sources(c['connections']['D'][0]) for n,c in seq.items()};actual=set()
 for dst,ss in fanin.items():
  for src in ss:
   if expected[src]!=expected[dst]:actual.add((src,dst))
 crossed={(q['source'],q['destination']) for q in baseline['crossings']}
 require(actual==crossed,'Mapped crossing inventory changed')
 for q in baseline['synchronizer_chains']:
  a,b=q['stage1'],q['stage2'];require(a in seq and b in seq and expected[a]==expected[b],'Unresolved synchronizer')
  require(fanin[b]=={a} and {n for n,ss in fanin.items() if a in ss}=={b},'Synchronizer structure/fanout changed '+q['name'])
 return dict(registers=len(seq),chains=13,crossings=len(actual))

def validate_cdc(*,baseline,mapped,readback,paths,log,coverage,synchronizers=None,phase='P4'):
 b=load(baseline);chains=b['synchronizer_chains'];cross=b['crossings']
 require(len(chains)==13 and {q['name'] for q in chains}==CHAINS,'Missing/duplicate synchronizer chains')
 require(len({q[k] for q in chains for k in ['stage1','stage2']})==26,'Merged synchronizer stages')
 require(len(cross)==605 and dict(collections.Counter(q['group'] for q in cross))==GROUPS,'Baseline eight-group inventory incomplete')
 expected={(q['group'],q['source'],q['destination']) for q in cross};require(len(expected)==605,'Duplicate baseline crossing')
 graph=graph_check(mapped,b,readback)
 rows=table(paths,['group','source','destination','max_slack_ns','min_status']);keys=[(q['group'],q['source'],q['destination']) for q in rows]
 require(len(rows)==605 and len(set(keys))==605 and set(keys)==expected,'Missing/duplicate/unresolved CDC path')
 minima={};counts=collections.Counter()
 for q in rows:
  g=q['group']
  if g in EVENTS:require(q['max_slack_ns']==q['min_status']=='FALSE_PATH','Event exception must be first-stage only')
  else:
   value=finite(q['max_slack_ns'],'bounded '+repr((g,q['source'],q['destination'])))
   require(q['min_status']=='TARGETED_HOLD_FALSE','Incorrect bounded hold semantics')
   minima[g]=min(value,minima.get(g,value));counts[g]+=1
 require(sum(counts.values())==602,'602 bounded paths required')
 c=load(coverage)
 for k,v in dict(status='PASS',input_bits_with_min_and_max=104,output_bits_with_min_and_max=93,clock_count=2,external_source_latency_ns=0,max_delay_groups=5,event_stage1_false_path_groups=3,bounded_hold_false_path_groups=5,broad_clock_cut=False,unclassified_exceptions=0,unconstrained_functional_endpoints=0).items():require(c.get(k)==v,'Constraint coverage '+k)
 require(set(c.get('excluded_reset_ports',[]))=={'S_AXI_ARESETN','mac_rst'} and set(c.get('clock_ports',[]))=={'S_AXI_ACLK','mac_clk'},'Reset/clock contract')
 if phase!='P2':require(c.get('propagated_clocks') is True,'Unpropagated clocks')
 try:text=Path(log).read_text()
 except OSError as e:raise Rejected('Missing/unreadable STA report') from e
 require('Error:' not in text,'STA execution error')
 marker='P2_STA_AUDIT_PASS bounded_pairs=602 event_pairs=3 interstage_checks=26' if phase=='P2' else ('P3_PHYSICAL_AUDIT_PASS' if phase=='P3' else 'P4_PHYSICAL_AUDIT_COMPLETE')
 require(marker in text,'Missing completed STA/CDC execution')
 bounds=re.findall(r'CDC_BOUND (\w+) timed_pairs=(\d+) worst_slack_ns=(\S+)',text)
 require(len(bounds)==5 and {q[0] for q in bounds}==set(counts),'Incomplete bound reports')
 for g,n,v in bounds:require(int(n)==counts[g] and abs(finite(v,'bound summary')-minima[g])<1e-5,'Bound summary/row mismatch')
 effects=re.findall(r'BOUND_EFFECT_PASS (\w+) (?:slack_)?delta_ns=(\S+)',text)
 require(len(effects)==5 and {q[0] for q in effects}==set(counts),'Missing constraint-effect probes')
 for g,v in effects:require(abs(finite(v,'bound effect')-2)<1e-5,'Ignored/ineffective CDC bound')
 sync=[]
 if synchronizers:
  rr=table(synchronizers,['chain','stage1','stage2','max_slack_ns','min_slack_ns']);require(len(rr)==13,'Incomplete interstage timing')
  wanted={q['name']:(q['stage1'],q['stage2']) for q in chains};require({q['chain'] for q in rr}==CHAINS,'Interstage chain identity')
  for q in rr:
   require((q['stage1'],q['stage2'])==wanted[q['chain']],'Interstage endpoints changed')
   sync.append((q['chain'],'max',finite(q['max_slack_ns'],'interstage setup')));sync.append((q['chain'],'min',finite(q['min_slack_ns'],'interstage hold')))
 else:
  sync=[(n,m,finite(v,'interstage')) for n,m,v in re.findall(r'SYNC_INTERSTAGE (\S+) (max|min) slack_ns=(\S+)',text)]
 require(len(sync)==26 and {(n,m) for n,m,v in sync}=={(n,m) for n in CHAINS for m in ['max','min']},'Missing/duplicate active interstage timing')
 if synchronizers:
  logged=re.findall(r'SYNC_INTERSTAGE (\S+) distance_um=\S+ setup_ns=(\S+) hold_ns=(\S+)',text)
  require(len(logged)==13 and {q[0] for q in logged}==CHAINS,'Incomplete logged synchronizer checks')
  actual={(n,m):v for n,m,v in sync}
  for n,a,z in logged:
   require(abs(finite(a,'logged sync setup')-actual[n,'max'])<1e-5 and abs(finite(z,'logged sync hold')-actual[n,'min'])<1e-5,'Synchronizer table/log mismatch')
 domains=re.findall(r'DOMAIN_WNS (\w+) (max|min) (\S+)',text)
 if phase!='P2':
  require(len(domains)==4 and {(c,m) for c,m,v in domains}=={(c,m) for c in ['bus_clk','mac_clk'] for m in ['max','min']},'Missing domain WNS')
  for c,m,v in domains:finite(v,'domain WNS')
  violations=re.findall(r'DOMAIN_VIOLATIONS (\w+) (max|min) endpoints=(\d+) tns_ns=(\S+)',text)
  require(len(violations)==4 and {(c,m) for c,m,n,v in violations}=={(c,m) for c in ['bus_clk','mac_clk'] for m in ['max','min']},'Missing domain TNS/endpoints')
  for c,m,n,v in violations:require(int(n)==0 and finite(v,'domain TNS')==0,'Domain timing violations')
 else:
  ww=re.findall(r'worst slack (max|min) (\S+)',text);require(len(ww)==2 and {m for m,v in ww}=={'max','min'},'Missing P2 overall timing')
  for m,v in ww:finite(v,'P2 overall WNS')
 return dict(status='PASS',groups=GROUPS,bounded_paths=602,event_first_stage_exceptions=3,active_interstage_checks=26,minima_ns=minima,graph=graph,domain_wns={c+'.'+m:float(v) for c,m,v in domains})

def checked_file(record,root):
 require(isinstance(record,dict) and set(['path','sha256','bytes'])<=set(record),'Missing provenance record')
 p=(root/record['path']).resolve();require(p.is_file(),'Missing provenance file')
 require(type(record['bytes']) is int and p.stat().st_size==record['bytes'] and re.fullmatch('[a-f0-9]{64}',record['sha256']) and sha(p)==record['sha256'],'File hash/size provenance mismatch')
 return p

def validate_gds(*,report,metadata,gds,odb,deck,execution_log,execution_record,contract,net_context=None):
 m=load(metadata);record=load(execution_record)
 require(m.get('status')=='COMPLETE' and m.get('exit_code')==0 and m.get('checker')=='KLayout','Missing completed GDS checker execution')
 for key,p in [('report_sha256',report),('gds_sha256',gds),('odb_sha256',odb),('deck_sha256',deck),('log_sha256',execution_log),('execution_record_sha256',execution_record)]:require(m.get(key)==sha(p),'GDS provenance mismatch '+key)
 require(record.get('exit_code')==0 and m.get('checker_version')==contract['klayout_version'],'GDS checker version/exit')
 command=record.get('command',record.get('args',[]));require(isinstance(command,list) and all(isinstance(x,str) for x in command),'Invalid checker command')
 require('in_gds='+str(Path(gds).resolve()) in command and 'report_file='+str(Path(report).resolve()) in command and str(Path(deck).resolve()) in command,'Checker command/artifact mismatch')
 text=Path(execution_log).read_text();require('Total elapsed:' in text and 'Writing report database: '+str(Path(report).resolve()) in text,'Incomplete checker log')
 require(not re.search(r'\b(?:ERROR|Error|FATAL|Exception):',text),'Checker error in log')
 require(sha(deck)==contract['flat_deck_sha256'],'Unsupported/modified GDS rule deck')
 root=ET.parse(report).getroot();require(root.tag=='report-database','Malformed report root')
 require(root.findtext('top-cell')=='mac_accel_dma_rob_top' and root.find('items') is not None and root.find('categories') is not None and root.find('cells') is not None,'Incomplete GDS report')
 require(root.find('./cells/cell/name') is not None and root.findtext('./cells/cell/name')=='mac_accel_dma_rob_top','Missing report cell inventory')
 generator=root.findtext('generator','');require(str(Path(deck).resolve()) in generator,'Report generator/deck mismatch')
 cats=[(q.findtext('name',''),q.findtext('description','')) for q in root.findall('./categories/category')]
 signature=hashlib.sha256(json.dumps(cats,separators=(',',':')).encode()).hexdigest()
 require(len(cats)==contract['category_count'] and signature==contract['category_sha256'],'Missing/altered checker rule categories')
 require(all(q.tag=='item' for q in root.find('items')),'Malformed marker inventory')
 require(all(q.tag=='category' for q in root.find('categories')),'Malformed rule inventory')
 names={n for n,d in cats};items=root.findall('./items/item');violations=[]
 for index,q in enumerate(items):
  category=q.findtext('category');require(category in names,'Unknown marker category')
  require(q.findtext('cell')=='mac_accel_dma_rob_top' and q.findtext('multiplicity')=='1','Invalid marker identity/multiplicity')
  polygons=[v.text for v in q.findall('./values/value') if v.text and v.text.startswith('polygon: ')]
  require(polygons,'Missing violation location')
  coords=[]
  for polygon in polygons:
   require(re.fullmatch(r'polygon: \([-+0-9.eE,;]+\)',polygon) is not None,'Malformed polygon location')
   pairs=polygon[10:-1].split(';');require(len(pairs)>=3,'Incomplete polygon')
   for pair in pairs:
    xy=pair.split(',');require(len(xy)==2,'Malformed coordinate');coords.append([finite(v,'marker coordinate',False) for v in xy])
  lo,hi=contract['coordinate_limits_um'];require(all(lo<=v<=hi for xy in coords for v in xy),'Out-of-layout violation location')
  item=dict(index=index,category=category,coordinates_um=coords)
  if 'ANTENNA' in category:
   require(net_context is not None,'Antenna violation lacks verified net/pin correlation; automatic repair disabled')
   context=load(net_context);require(context.get('odb_sha256')==sha(odb),'Net context belongs to different ODB')
   witness=m.get('violations',[]);require(len(witness)==len(items),'Missing per-marker net/pin metadata')
   w=witness[index];require(w.get('index')==index and w.get('category')==category and w.get('coordinates_um')==coords,'Marker/net location mismatch')
   net=w.get('net');pin=w.get('pin');require(net in context['nets'] and pin in context['nets'][net]['pins'],'Unresolved violating net/pin')
   rectangle=context['nets'][net].get('pin_locations_um',{}).get(pin)
   require(isinstance(rectangle,list) and len(rectangle)==4,'Missing DB-derived pin location')
   x1,y1,x2,y2=[finite(v,'pin location',False) for v in rectangle]
   require(x1<x2 and y1<y2 and all(x1<=x<=x2 and y1<=y<=y2 for x,y in coords),'Violation location does not match verified pin geometry')
   item.update(net=net,pin=pin)
  violations.append(item)
 require(m.get('marker_count')==len(items) and m.get('antenna_count')==sum('ANTENNA' in q['category'] for q in violations),'Marker-count metadata mismatch')
 require(not violations,'Supported GDS DRC/antenna violations: '+json.dumps(violations))
 return dict(status='PASS',supported_drc_markers=0,gds_antenna_markers=0,rule_categories=len(cats),native_lef_antenna='INCONCLUSIVE',net_correlation='Not required for zero markers; violations require explicit context and always reject')

def validate_bundle(path):
 path=Path(path).resolve();root=path.parent;b=load(path);require(b.get('schema')==1,'Unsupported acceptance bundle')
 required={'v','sdc','spef','odb','def','gds'};require(set(b.get('artifacts',{}))==required,'Incomplete physical artifact inventory')
 artifacts={k:checked_file(v,root) for k,v in b['artifacts'].items()};require(all(p.stat().st_size>0 for p in artifacts.values()),'Empty physical artifact')
 files={k:checked_file(v,root) for k,v in b['evidence'].items()}
 pins=load(files['pins']);summary=load(files['summary']);require(summary['parameters']['MAX_OUTSTANDING']==8 and summary['parameters']['DATA_WIDTH']==16,'Wrong V3 parameters')
 require(b.get('source_commit')==pins['rtl_commit']==summary['rtl_commit'],'Wrong source revision')
 for rel,h in pins['source_sha256'].items():require(sha(root/b['source_root']/rel)==h,'Source hash changed '+rel)
 for rel,h in pins['platform_sha256'].items():require(sha(root/b['platform_root']/rel)==h,'Platform hash changed '+rel)
 if b.get('accepted_reference'):
  for k,p in artifacts.items():require(sha(p)==summary['artifacts'][k]['sha256'],'Accepted-reference artifact mismatch')
 if b.get('accepted_reference'):
  for key,h in pins['accepted_router_evidence_sha256'].items():require(sha(files[key])==h,'Accepted router report provenance mismatch')
 result=validate_cdc(baseline=files['baseline'],mapped=files['mapped'],readback=files['readback'],paths=files['cdc'],log=files['sta_log'],coverage=files['coverage'],synchronizers=files['sync'])
 sta_text=files['sta_log'].read_text()
 electrical=re.findall(r'ELECTRICAL_BEGIN\n(.*?)ELECTRICAL_END',sta_text,re.S)
 require(len(electrical)==1 and not electrical[0].strip(),'Missing/nonclean electrical evidence')
 result['gds']=validate_gds(report=files['drc_xml'],metadata=files['drc_metadata'],gds=artifacts['gds'],odb=artifacts['odb'],deck=files['deck'],execution_log=files['drc_log'],execution_record=files['drc_execution'],contract=pins['gds_check_contract'],net_context=files.get('net_context'))
 require(files['router_drc'].read_text().strip()=='','Router DRC violations')
 router=files['router_log'].read_text();require('Complete detail routing.' in router and 'Total wire length =' in router and 'Total number of vias =' in router and re.search(r'Number of violations\s*=\s*0',router),'Missing completed clean router evidence')
 require(b['router_artifacts_sha256']=={k:sha(artifacts[k]) for k in ['odb','def','gds']},'Router provenance/artifacts mismatch')
 result.update(status='PASS',scope='Typical research artifact acceptance; no fresh synthesis/P&R replay or foundry signoff',router_drc=0,lvs='UNSUPPORTED',source_commit=b['source_commit'],artifact_sha256={k:sha(v) for k,v in artifacts.items()})
 return result

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 try:
  require(not a.output.exists(),'Output already exists');result=validate_bundle(a.bundle)
 except (Rejected,OSError,ValueError,KeyError,TypeError,ET.ParseError,RecursionError) as e:
  print('REJECT: '+str(e),file=sys.stderr);return 1
 a.output.parent.mkdir(parents=True,exist_ok=True)
 with a.output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print('ACCEPTANCE_PASS 602 bounded paths; 26 active interstage checks; supported GDS/router checks complete');return 0
if __name__=='__main__':sys.exit(main())
