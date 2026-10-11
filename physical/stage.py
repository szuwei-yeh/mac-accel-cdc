#!/usr/bin/env python3
"""Explicit sequential replay of qualified stages; never invoked during P5."""
import os,sys,json,shutil,re,xml.etree.ElementTree as ET
from pathlib import Path
from acceptance import validate_cdc,validate_gds,require,sha
if sys.flags.optimize or os.environ.get('PYTHONOPTIMIZE'):raise SystemExit('Unoptimized Python is required')
ROOT=Path(__file__).resolve().parent
cfg=json.loads((ROOT/'configuration.json').read_text());ph=sys.argv[1].lower();assert ph in ['p2','p3','p4']
phase=ROOT/ph/'inputs';os.environ[ph.upper()+'_RUN']=str(ROOT/ph);sys.path.insert(0,str(phase))
from run import *
if (RUN/'audit/accepted.json').exists():raise SystemExit('Stage already accepted; use a new run')
def python(name,*args):return execute(name[:-3],['/usr/bin/python3','-B',PHASE/name,*args])
def make_cmd(target,variant):return ['make','-j1','-f',FLOW/'Makefile',f'FLOW_HOME={FLOW}',f'WORK_HOME={RUN}',f'DESIGN_CONFIG={PHASE}/config.mk',f'FLOW_VARIANT={variant}','NUM_CORES=2',f'OPENROAD_EXE={TOOLS}/bin/openroad',f'YOSYS_EXE={SUITE}/yosys',f'KLAYOUT_CMD={TOOLS}/bin/klayout','PYTHON_EXE=/usr/bin/python3',f'SYNTH_SCRIPT={ROOT}/p2/inputs/synth.tcl',target]
def make_run(name,target,variant):execute(name,make_cmd(target,variant),timeout=3600)
def readback(label,v):
 execute(label+'-readback',[SUITE/'yosys','-Q','-T','-p',f'read_liberty -lib {FLOW}/platforms/nangate45/lib/NangateOpenCellLibrary_typical.lib; read_verilog {v}; hierarchy -check -top mac_accel_dma_rob_top; check -assert -mapped; write_json {RUN}/audit/{label}.json'])
res=RUN/'results/nangate45/mac_accel_dma_rob_top'
def baseline():
 p2=ROOT/'p2';assert (p2/'audit/accepted.json').exists()
 (RUN/'audit/baseline.json').write_text(json.dumps({'p2_run':str(p2)}))
 for a,b in [('mapping-gate.json','p2-mapping-gate.json'),('cdc-bindings.tcl','p2-cdc-bindings.tcl'),('sta-pairs.tcl','p2-sta-pairs.tcl')]:shutil.copyfile(p2/'audit'/a,RUN/'audit'/b)
if ph=='p2':
 python('verify_rtl.py');python('verify_formal.py')
 assert all(q['status'].startswith('PASS') for q in json.loads((RUN/'audit/formal-results.json').read_text()))
 for exe,name in [(TOOLS/'bin/sta','constraints_micro_network.tcl'),(TOOLS/'bin/openroad','constraints_micro_openroad.tcl')]:execute(name,[exe,'-exit',PHASE/name])
 make_run('elaboration',str(res/'base/1_1_yosys_canonicalize.rtlil'),'base');python('elaboration_audit.py')
 make_run('full-synthesis','synth','base');python('verify_gls.py')
 execute('qualified-readback',[TOOLS/'bin/openroad','-exit','-no_init',PHASE/'qualified_readback.tcl'])
 readback('readback',res/'base/1_2_yosys.v')
 execute('independent-sta-qualified',[TOOLS/'bin/sta','-exit',PHASE/'sta_audit.tcl'])
 execute('openroad-independent-sta-qualified',[TOOLS/'bin/openroad','-exit','-no_init',PHASE/'sta_audit_openroad.tcl']);python('coverage.py')
elif ph=='p3':
 baseline();p2=ROOT/'p2/results/nangate45/mac_accel_dma_rob_top/base'
 for var in ['base','pg_final']:
  (res/var).mkdir(parents=True)
  for a,b in [('1_synth_qualified.odb','1_synth.odb'),('1_synth_qualified.sdc','1_synth.sdc'),('1_2_yosys.v','1_2_yosys.v')]:shutil.copyfile(p2/a,res/var/b)
 for name,target in [('floorplan','do-floorplan'),('placement','do-place'),('cts','do-cts')]:make_run(name,target,'pg_final')
 os.environ['P3_AUDIT_STAGE']='cts';execute('cts-qualified-timing-cdc-audit',[TOOLS/'bin/openroad','-exit',PHASE/'physical_audit.tcl'])
 readback('cts-mapped',RUN/'exports/cts.v');python('mapped_audit.py','cts');python('coverage.py');python('verify_gls.py')
else:
 baseline();assert (ROOT/'p3/audit/accepted.json').exists();(res/'hold50').mkdir(parents=True)
 for a,b in [('4_cts.odb','4_cts.odb'),('4_cts.sdc','4_cts.sdc'),('1_2_yosys.v','1_2_yosys.v')]:shutil.copyfile(ROOT/'p3/results/nangate45/mac_accel_dma_rob_top/pg_final'/a,res/'hold50'/b)
 def route(var):
  make_run(var+'-global-route','do-grt',var);python('audit_stage.py','globalroute',var,'grt_'+var)
  for name,target in [('detailed-route','do-5_2_route'),('filler','do-5_3_fillcell'),('route-checkpoint','do-5_route'),('route-sdc','do-5_route.sdc'),('extraction','do-finish'),('gds','do-gds')]:make_run(var+'-'+name,target,var)
  report=RUN/'audit'/('drc-'+var+'.lyrdb')
  command=[TOOLS/'bin/klayout','-zz','-rd',f'in_gds={res}/{var}/6_final.gds','-rd',f'report_file={report}','-r',PHASE/'FreePDK45_flat_threads2.lydrc']
  execute('klayout-drc-'+var,command,timeout=1800)
  import time
  execution=RUN/'audit'/('drc-execution-'+var+'.json')
  execution.write_text(json.dumps(dict(exit_code=0,command=list(map(str,command)))))
  metadata=RUN/'audit'/('drc-metadata-'+var+'.json')
  tree=ET.parse(report).getroot();require(tree.find('items') is not None,'Missing GDS item inventory')
  items=tree.findall('./items/item')
  metadata.write_text(json.dumps(dict(status='COMPLETE',exit_code=0,checker='KLayout',checker_version=cfg['pins']['klayout_version'],report_sha256=sha(report),gds_sha256=sha(res/var/'6_final.gds'),odb_sha256=sha(res/var/'6_final.odb'),deck_sha256=sha(PHASE/'FreePDK45_flat_threads2.lydrc'),log_sha256=sha(RUN/'console'/('klayout-drc-'+var+'.log')),execution_record_sha256=sha(execution),marker_count=len(items),antenna_count=sum('ANTENNA' in q.findtext('category','') for q in items))))
  # Any violation stops for investigation. The historic segmentation recipe is never automatic.
  return validate_gds(report=report,metadata=metadata,gds=res/var/'6_final.gds',odb=res/var/'6_final.odb',deck=PHASE/'FreePDK45_flat_threads2.lydrc',execution_log=RUN/'console'/('klayout-drc-'+var+'.log'),execution_record=execution,contract=cfg['pins']['gds_check_contract'])
 route('hold50');variant='hold50'
 prefix='final';shutil.copyfile(res/variant/'6_final.spef',RUN/'exports/final.spef')
 python('audit_stage.py','final',variant,prefix)
 os.environ.update(P4_STANDALONE='1',P4_AUDIT_STAGE='independentsta',P4_AUDIT_LABEL='independentsta',P4_VARIANT=variant,P4_FINAL_PREFIX=prefix)
 execute('independentsta-timing-cdc-audit',[TOOLS/'bin/sta','-exit',PHASE/'physical_audit.tcl']);python('annotation_consistency.py',prefix,'independentsta');os.environ.pop('P4_STANDALONE')
 python('verify_gls.py',prefix)
 drc=RUN/'reports/nangate45/mac_accel_dma_rob_top'/variant/'5_route_drc.rpt'
 require(drc.exists() and not drc.read_text().strip(),'Router DRC')
 router=(RUN/'logs/nangate45/mac_accel_dma_rob_top'/variant/'5_2_route.log').read_text()
 require('Complete detail routing.' in router and re.search(r'Number of violations\s*=\s*0',router),'Incomplete/violating router report')
 for ext in ['gds','def','odb','sdc','spef','v']:assert (res/variant/('6_final.'+ext)).stat().st_size>0
# A completed replay is a separate experiment, never a substitute for the archived measured run.
if ph in ['p2','p3']:
 logname='openroad-independent-sta-qualified.log' if ph=='p2' else 'cts-qualified-timing-cdc-audit.log'
 log=(RUN/'console'/logname).read_text()
 if ph=='p2':
  wns=re.findall(r'worst slack (max|min) ([-\d.]+)',log);assert len(wns)==2 and all(float(v)>=0 for _,v in wns),'Setup/hold timing failed'
 else:
  wns=re.findall(r'DOMAIN_WNS (\w+) (max|min) ([-\d.]+)',log);assert len(wns)==4 and all(float(v)>=0 for _,_,v in wns),'Domain setup/hold timing failed'
 if ph=='p3':assert not log.split('ELECTRICAL_BEGIN\n')[1].split('ELECTRICAL_END')[0].strip(),'Electrical violations'
if ph=='p2':
 validate_cdc(baseline=RUN/'audit/mapping-gate.json',mapped=RUN/'audit/readback.json',readback=RUN/'audit/readback.json',paths=RUN/'audit/sta-paths-openroad.tsv',log=RUN/'console/openroad-independent-sta-qualified.log',coverage=RUN/'audit/coverage-gate.json',phase='P2')
elif ph=='p3':
 validate_cdc(baseline=RUN/'audit/p2-mapping-gate.json',mapped=RUN/'audit/cts-mapped.json',readback=ROOT/'p2/audit/readback.json',paths=RUN/'audit/cts-cdc-paths.tsv',log=RUN/'console/cts-qualified-timing-cdc-audit.log',coverage=RUN/'audit/cts-coverage-gate.json',synchronizers=RUN/'audit/cts-synchronizers.tsv',phase='P3')
(RUN/'audit/accepted.json').write_text(json.dumps(dict(stage=ph,status='COMPLETED',lvs='UNSUPPORTED',scope='Typical research flow; see reports and physical/METHODOLOGY.md'))+'\n')
print('Completed',ph,'Read timing/DRC/coverage reports before claiming closure.')
