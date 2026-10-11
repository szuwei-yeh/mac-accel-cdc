import json,re
from pathlib import Path
from run import *
res=RUN/'results/nangate45/mac_accel_dma_rob_top/base'
models=RUN/'verification/nangate45-functional.v'
models.parent.mkdir(exist_ok=True)
old=Path(json.loads((RUN/'audit/baseline.json').read_text())['p2_run'])
import shutil
shutil.copyfile(old/'verification/nangate45-functional.v',models)
s=(SRC/'sim/tb_mac_accel_dma_rob_top.v').read_text()
s,n=re.subn(r'mac_accel_dma_rob_top #\(.*?\) dut \(', 'mac_accel_dma_rob_top dut (',s,flags=re.S);assert n==1
s=s.replace('dut.busy_any_bus',r'(dut.dma_busy | dut.\busy_sync_bus[1] )')
tb=RUN/'verification/tb_v3_mapped_names.v';assert not tb.exists();tb.write_text(s)
loaded=set(re.findall(r'^module (\w+)',models.read_text(),re.M));used=set(json.loads((RUN/'audit/cts-mapping-gate.json').read_text())['cell_types']);assert not used-loaded
binary=RUN/'verification/mapped-qualified.vvp'
execute('cts-gls-compile-qualified',[SUITE/'iverilog','-g2012','-DV3_MAX_OUT=8','-s','tb_mac_accel_dma_rob_top','-o',binary,tb,RUN/'exports/cts.v',models,SRC/'sim/axi_read_mem_model_ooo.v'])
rc,log=execute('cts-gls-simulation',[SUITE/'vvp',binary],timeout=300,accept_failure=True)
(RUN/'audit/cts-gls-result.json').write_text(json.dumps(dict(status='PASS' if rc==0 and '7 PASS / 0 FAIL' in log else 'FAIL',cases=7,exit_code=rc,all_mapped_cell_models_present=True,delays='zero',formal_equivalence='not established'),indent=2)+'\n')
if rc or '7 PASS / 0 FAIL' not in log:raise SystemExit('Mapped functional failure requires investigation')
