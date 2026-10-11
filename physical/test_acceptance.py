#!/usr/bin/env python3
"""Fault-inject copies of a supplied complete artifact bundle; production gates only."""
import argparse,copy,csv,hashlib,json,re,sys,xml.etree.ElementTree as ET
from pathlib import Path
from acceptance import validate_bundle,Rejected,sha

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',type=Path,required=True);p.add_argument('--work',type=Path,required=True);a=p.parse_args()
 if a.work.exists():p.error('Work directory exists')
 a.work.mkdir(parents=True);base=json.loads(a.bundle.read_text());original=a.bundle.resolve().parent
 for key in ['source_root','platform_root']:base[key]=str((original/base[key]).resolve())
 for q in list(base['artifacts'].values())+list(base['evidence'].values()):q['path']=str((original/q['path']).resolve())
 cases=[]
 def control(name,expected,change=None):
  d=a.work/name;d.mkdir();b=copy.deepcopy(base)
  def write(key,text):
   old=b['evidence'][key]['path'];f=d/(key+Path(old).suffix);f.write_text(text);b['evidence'][key]=dict(path=str(f.resolve()),bytes=f.stat().st_size,sha256=sha(f))
   if key=='drc_xml':
    execution=json.loads(read('drc_execution'));execution['command']=[('report_file='+str(f.resolve())) if x=='report_file='+old else x for x in execution['command']];ex=write('drc_execution',json.dumps(execution));lg=write('drc_log',read('drc_log').replace(old,str(f.resolve())));meta=json.loads(read('drc_metadata'));meta.update(report_sha256=sha(f),execution_record_sha256=sha(ex),log_sha256=sha(lg));write('drc_metadata',json.dumps(meta))
   return f
  def read(key):return Path(b['evidence'][key]['path']).read_text()
  if change:change(b,write,read)
  bundle=d/'bundle.json';bundle.write_text(json.dumps(b,indent=2)+'\n')
  try:result=validate_bundle(bundle);observed='PASS';reason=result['status']
  except (Rejected,OSError,ValueError,KeyError,TypeError,ET.ParseError,RecursionError) as e:observed='REJECT';reason=str(e)
  row=dict(name=name,expected=expected,observed=observed,reason=reason);cases.append(row);print(json.dumps(row),flush=True)
  if observed!=expected:raise SystemExit('Incorrect acceptance control '+name)
  if name=='antenna-valid-net-location' and not reason.startswith('Supported GDS DRC/antenna violations:'):raise SystemExit('Nonzero antenna fixture did not reach final violation rejection')
 def negative(b,write,read):
  rows=list(csv.DictReader(read('cdc').splitlines(),delimiter='\t'))
  row=next(q for q in rows if q['group']=='write_gray');row['max_slack_ns']='-0.125'
  import io
  f=io.StringIO();w=csv.DictWriter(f,fieldnames=list(rows[0]),delimiter='\t');w.writeheader();w.writerows(rows);write('cdc',f.getvalue())
 def remove_path(b,write,read):write('cdc','\n'.join(read('cdc').splitlines()[:-1])+'\n')
 def nan_path(b,write,read):write('cdc',re.sub(r'(write_gray[^\n]*?\t)([0-9]+\.[0-9]+)(\t)',r'\g<1>nan\3',read('cdc'),count=1))
 def missing(b,write,read):b['evidence']['cdc']['path']=str((a.work/'does-not-exist.tsv').resolve())
 def malformed(b,write,read):write('cdc','not a CDC report\n')
 def effect(b,write,read):write('sta_log',read('sta_log').replace('BOUND_EFFECT_PASS write_gray','MISSING_EFFECT write_gray',1))
 def masked(b,write,read):write('cdc',read('cdc').replace('TARGETED_HOLD_FALSE','FALSE_PATH',1))
 def sync_missing(b,write,read):write('sync','\n'.join(read('sync').splitlines()[:-1])+'\n')
 def sync_negative(b,write,read):
  rows=list(csv.DictReader(read('sync').splitlines(),delimiter='\t'));rows[0]['min_slack_ns']='-0.01'
  import io
  f=io.StringIO();w=csv.DictWriter(f,fieldnames=list(rows[0]),delimiter='\t');w.writeheader();w.writerows(rows);write('sync',f.getvalue())
 def broad(b,write,read):q=json.loads(read('coverage'));q['broad_clock_cut']=True;write('coverage',json.dumps(q))
 def graph_fault(b,write,read):
  q=json.loads(read('mapped'));t=q['modules']['mac_accel_dma_rob_top'];baseline=json.loads(read('baseline'));name=baseline['synchronizer_chains'][0]['stage2'];cell=next(c for n,c in t['cells'].items() if n.replace(chr(92)+'[','[').replace(chr(92)+']',']')==name);cell['connections']['D']=['0'];write('mapped',json.dumps(q))
 def invalid_meta(b,write,read):q=json.loads(read('drc_metadata'));q.pop('status');write('drc_metadata',json.dumps(q))
 def wrong_gds(b,write,read):q=json.loads(read('drc_metadata'));q['gds_sha256']='0'*64;write('drc_metadata',json.dumps(q))
 def incomplete_zero(b,write,read):
  q=ET.fromstring(read('drc_xml'));q.remove(q.find('items'));f=write('drc_xml',ET.tostring(q,encoding='unicode'));m=json.loads(read('drc_metadata'));m['report_sha256']=sha(f);write('drc_metadata',json.dumps(m))
 def malformed_items(b,write,read):
  q=ET.fromstring(read('drc_xml'));ET.SubElement(q.find('items'),'unrecognized-marker');write('drc_xml',ET.tostring(q,encoding='unicode'))
 def incomplete_categories(b,write,read):
  q=ET.fromstring(read('drc_xml'));q.find('categories').remove(q.find('./categories/category'));f=write('drc_xml',ET.tostring(q,encoding='unicode'));m=json.loads(read('drc_metadata'));m['report_sha256']=sha(f);write('drc_metadata',json.dumps(m))
 def unexecuted(b,write,read):
  f=write('drc_log','Checker started but did not finish\n');m=json.loads(read('drc_metadata'));m['log_sha256']=sha(f);write('drc_metadata',json.dumps(m))
 def wrong_exit(b,write,read):q=json.loads(read('drc_metadata'));q['exit_code']=1;write('drc_metadata',json.dumps(q))
 def artifact_fault(b,write,read):b['artifacts']['spef']['sha256']='0'*64
 # Nonzero fixtures inject a synthetic marker at a REAL DB-derived pin rectangle;
 # prepare records next to the bundle. This is not a measured final antenna violation.
 def violation(b,write,read,missing_context=False):
  historical=original/'antenna-fault-record.json'
  if not historical.exists():raise SystemExit('Need real antenna-fault-record.json for nonzero controls')
  fault=json.loads(historical.read_text());root=ET.fromstring(read('drc_xml'));items=root.find('items');item=ET.fromstring(fault['item_xml']);items.append(item);f=write('drc_xml',ET.tostring(root,encoding='unicode'));m=json.loads(read('drc_metadata'));m.update(report_sha256=sha(f),marker_count=1,antenna_count=1,violations=[fault['binding']]);write('drc_metadata',json.dumps(m))
  context=fault['context']
  if context['odb_sha256']!=b['artifacts']['odb']['sha256']:raise SystemExit('Fault context belongs to another ODB')
  g=a.work/(('context-'+str(len(cases)))+'.json');g.write_text(json.dumps(context));b['evidence']['net_context']=dict(path=str(g.resolve()),bytes=g.stat().st_size,sha256=sha(g))
  if missing_context:b['evidence'].pop('net_context')
 def wrong_pin_location(b,write,read):
  violation(b,write,read);q=json.loads(read('net_context'));net=next(iter(q['nets']));q['nets'][net]['pin_locations_um']['clkbuf_0_S_AXI_ACLK/A']=[0,0,1,1];write('net_context',json.dumps(q))
 def wrong_pin(b,write,read):
  violation(b,write,read);m=json.loads(read('drc_metadata'));m['violations'][0]['pin']='unresolved/A';write('drc_metadata',json.dumps(m))
 def bad_location(b,write,read):
  violation(b,write,read);q=ET.fromstring(read('drc_xml'));q.find('./items/item/values/value').text='polygon: malformed';f=write('drc_xml',ET.tostring(q,encoding='unicode'));m=json.loads(read('drc_metadata'));m['report_sha256']=sha(f);write('drc_metadata',json.dumps(m))
 def missing_net(b,write,read):violation(b,write,read,True)
 controls=[('accepted-p4-final','PASS',None),('negative-bounded-slack','REJECT',negative),('positive-domain-negative-cdc','REJECT',negative),('missing-crossing','REJECT',remove_path),('missing-cdc-report','REJECT',missing),('malformed-cdc-report','REJECT',malformed),('nonfinite-cdc-slack','REJECT',nan_path),('ignored-bound-effect','REJECT',effect),('masked-bounded-path','REJECT',masked),('missing-interstage-check','REJECT',sync_missing),('negative-interstage-check','REJECT',sync_negative),('broad-clock-cut','REJECT',broad),('broken-synchronizer-structure','REJECT',graph_fault),('antenna-valid-net-location','REJECT',violation),('antenna-missing-net-context','REJECT',missing_net),('antenna-invalid-location','REJECT',bad_location),('antenna-location-pin-mismatch','REJECT',wrong_pin_location),('antenna-unresolved-pin','REJECT',wrong_pin),('missing-antenna-metadata','REJECT',invalid_meta),('wrong-gds-provenance','REJECT',wrong_gds),('missing-items-zero-report','REJECT',incomplete_zero),('incomplete-rule-categories','REJECT',incomplete_categories),('malformed-marker-inventory','REJECT',malformed_items),('checker-not-completed','REJECT',unexecuted),('checker-nonzero-exit','REJECT',wrong_exit),('artifact-hash-mismatch','REJECT',artifact_fault),('clean-complete-antenna','PASS',None)]
 for name,expected,fn in controls:control(name,expected,fn)
 (a.work/'results.json').write_text(json.dumps(dict(status='PASS',production_code_sha256=sha(Path(__file__).with_name('acceptance.py')),controls=cases),indent=2)+'\n')
 return 0
if __name__=='__main__':sys.exit(main())
