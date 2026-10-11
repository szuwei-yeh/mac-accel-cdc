#!/usr/bin/env python3
"""Prepare pinned portable inputs. This command never runs synthesis or P&R."""
import argparse,ast,hashlib,json,os,re,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--source',type=Path,default=HERE.parent)
p.add_argument('--orfs',type=Path,required=True)
p.add_argument('--tools',type=Path,required=True,help='qualified installation with bin/openroad, sta, klayout, abc-batch')
p.add_argument('--suite-bin',type=Path,required=True)
p.add_argument('--output',type=Path,required=True,help='new, nonexistent output directory; whitespace-free paths')
p.add_argument('--prepare',action='store_true',help='validate hashes/versions and create inputs; otherwise print plan only')
a=p.parse_args();pins=json.loads((HERE/'pins.json').read_text());paths={k:v.resolve() for k,v in vars(a).items() if isinstance(v,Path)}
if any(re.search(r"[^A-Za-z0-9_/.:+\-]",str(v)) for v in paths.values()):p.error('Paths must contain only letters, digits, /_.:+-')
root=paths['output'];flow=paths['orfs']/'flow'
if root.exists():p.error('Output already exists; choose a new experiment directory')
print('Plan: P2 verification/synthesis → P3 floorplan/place/CTS → P4 route/extract/STA/DRC. Two threads; one job at a time.')
print('Future explicit execution: python3 '+str(root/'stage.py')+' P2; then P3; then P4')
if sys.flags.optimize or os.environ.get('PYTHONOPTIMIZE'):p.error('Unoptimized Python is required for legacy validation guards')
if not a.prepare:sys.exit(0)
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
assert subprocess.check_output(['git','-C',str(paths['orfs']),'rev-parse','HEAD'],text=True).strip()==pins['orfs_commit'],'ORFS revision'
for rel,h in pins['source_sha256'].items():assert sha(paths['source']/rel)==h,('RTL/verification source changed',rel)
for rel,h in pins['platform_sha256'].items():assert sha(flow/'platforms/nangate45'/rel)==h,('Platform changed',rel)
root.mkdir(parents=True)
for phase in ['p2','p3','p4']:
 for d in ['audit','console','timing','tmp','cache','config','runtime','verification','constraints-tests','exports','images','inputs']:(root/phase/d).mkdir(parents=True)
 (root/phase/'runtime').chmod(0o700)
subs={'@SOURCE@':str(paths['source']),'@FLOW@':str(flow),'@TOOLS@':str(paths['tools']),'@SUITE@':str(paths['suite_bin'])}
for ph in ['p2','p3','p4']:subs['@'+ph.upper()+'_RUN@']=str(root/ph);subs['@'+ph.upper()+'_PHASE@']=str(root/ph/'inputs')
for rel,provenance in pins['template_provenance'].items():
 src=HERE/'templates'/rel;assert sha(src)==provenance['template_sha256'],('Template changed',rel)
 text=src.read_text()
 for k,v in subs.items():text=text.replace(k,v)
 assert not re.search(r'@[A-Z0-9_]+@',text),rel
 dst=root/rel.split('/')[0]/'inputs'/Path(rel).name
 if dst.suffix=='.py':ast.parse(text,filename=rel)
 dst.write_text(text)
manifest=dict(commit=pins['rtl_commit'],source=str(paths['source']),sha256=pins['source_sha256'],parameters=json.loads((HERE/'summary.json').read_text())['parameters'])
(root/'p2/audit/source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(root/'configuration.json').write_text(json.dumps(dict(paths={k:str(v) for k,v in paths.items()},pins=pins,adapter_status='Prepared/validated; end-to-end portability rerun not performed in P5'),indent=2)+'\n')
(root/'stage.py').write_bytes((HERE/'stage.py').read_bytes())
(root/'acceptance.py').write_bytes((HERE/'acceptance.py').read_bytes())
for ph in ['p2','p3','p4']:(root/ph/'inputs/acceptance.py').write_bytes((HERE/'acceptance.py').read_bytes())
# Runtime-derived deck: identical enabled rules, flat geometry mode, two workers.
deck=(flow/'platforms/nangate45/drc/FreePDK45.lydrc').read_text()
assert 'threads(4)' in deck and 'deep' in deck
(root/'p4/inputs/FreePDK45_flat_threads2.lydrc').write_text(deck.replace('threads(4)','threads(2)').replace('deep','flat'))
e=os.environ.copy();r=root/'p2';e.update(TMPDIR=str(r/'tmp'),XDG_CACHE_HOME=str(r/'cache'),XDG_CONFIG_HOME=str(r/'config'),XDG_RUNTIME_DIR=str(r/'runtime'),KLAYOUT_HOME=str(r/'config'),QT_QPA_PLATFORM='offscreen')
versions={}
for exe,arg,expected in [(paths['tools']/'bin/openroad','-version',pins['openroad_version']),(paths['tools']/'bin/sta','-version',pins['opensta_version']),(paths['tools']/'bin/klayout','-v',pins['klayout_version']),(paths['suite_bin']/'yosys','-V',pins['yosys_version'])]:
 text=subprocess.check_output([str(exe),arg],env=e,text=True,stderr=subprocess.STDOUT);assert expected in text,(exe,text);versions[exe.name]=text.strip()
assert (paths['tools']/'bin/abc-batch').is_file()
(root/'tool-versions.json').write_text(json.dumps(versions,indent=2)+'\n')
print('Prepared:',root,'No synthesis or physical design was executed.')
