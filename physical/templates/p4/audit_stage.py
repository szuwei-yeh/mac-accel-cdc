import sys,os,json,re
if sys.flags.optimize or os.environ.get('PYTHONOPTIMIZE'):raise SystemExit('Unoptimized Python is required')
from run import *
stage=sys.argv[1];variant=sys.argv[2] if len(sys.argv)>2 else 'hold50'
label=sys.argv[3] if len(sys.argv)>3 else stage
os.environ.update(P4_AUDIT_STAGE=stage,P4_VARIANT=variant,P4_AUDIT_LABEL=label)
execute(label+'-timing-cdc-audit',[TOOLS/'bin/openroad','-exit',PHASE/'physical_audit.tcl'],timeout=600)
execute(label+'-mapped-readback',[SUITE/'yosys','-Q','-T','-p',f'read_liberty -lib {FLOW}/platforms/nangate45/lib/NangateOpenCellLibrary_typical.lib; read_verilog {RUN}/exports/{label}.v; hierarchy -check -top mac_accel_dma_rob_top; check -assert -mapped; stat -liberty {FLOW}/platforms/nangate45/lib/NangateOpenCellLibrary_typical.lib; write_json {RUN}/audit/{label}-mapped.json'])
execute(label+'-mapped-graph',['/usr/bin/python3','-B',PHASE/'mapped_audit.py',label])
execute(label+'-constraint-coverage',['/usr/bin/python3','-B',PHASE/'coverage.py',label])
s=(RUN/'console'/f'{label}-timing-cdc-audit.log').read_text()
wns={(c,m):float(v) for c,m,v in re.findall(r'DOMAIN_WNS (\w+) (max|min) ([-\d.]+)',s)}
elec=s.split('ELECTRICAL_BEGIN\n')[1].split('ELECTRICAL_END')[0].strip()
print('PHYSICAL_STAGE_METRICS',stage,wns,'electrical_empty',not elec)
if min(wns.values())<0 or elec:raise SystemExit('Physical timing/electrical violations require an isolated repair iteration')

from acceptance import validate_cdc
validate_cdc(baseline=RUN/'audit/p2-mapping-gate.json', mapped=RUN/'audit'/f'{label}-mapped.json', readback=__import__('pathlib').Path(json.loads((RUN/'audit/baseline.json').read_text())['p2_run'])/'audit/readback.json', paths=RUN/'audit'/f'{label}-cdc-paths.tsv', log=RUN/'console'/f'{label}-timing-cdc-audit.log', coverage=RUN/'audit'/f'{label}-coverage-gate.json', synchronizers=RUN/'audit'/f'{label}-synchronizers.tsv')
