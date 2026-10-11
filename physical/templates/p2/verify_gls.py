import json,re
from run import *
res=RUN/'results/nangate45/mac_accel_dma_rob_top/base'
models=RUN/'verification/nangate45-functional.v'
execute('library-functional-models-qualified',[SUITE/'yosys','-Q','-T','-p',f'read_liberty -ignore_miss_func {FLOW}/platforms/nangate45/lib/NangateOpenCellLibrary_typical.lib; write_verilog -noattr {models}'])
s=(SRC/'sim/tb_mac_accel_dma_rob_top.v').read_text()
s,n=re.subn(r'mac_accel_dma_rob_top #\(.*?\) dut \(', 'mac_accel_dma_rob_top dut (',s,flags=re.S);assert n==1
s=s.replace('dut.busy_any_bus',r'(dut.dma_busy | dut.\busy_sync_bus[1] )')
tb=RUN/'verification/tb_v3_mapped_names.v';assert not tb.exists();tb.write_text(s)
loaded=set(re.findall(r'^module (\w+)',models.read_text(),re.M));used=set(json.loads((RUN/'audit/mapping-gate.json').read_text())['cell_types']);assert not used-loaded
binary=RUN/'verification/mapped-qualified.vvp'
execute('mapped-gls-compile-qualified',[SUITE/'iverilog','-g2012','-DV3_MAX_OUT=8','-s','tb_mac_accel_dma_rob_top','-o',binary,tb,res/'1_2_yosys.v',models,SRC/'sim/axi_read_mem_model_ooo.v'])
rc,log=execute('mapped-gls-simulation',[SUITE/'vvp',binary],timeout=300,accept_failure=True)
(RUN/'audit/gls-result.json').write_text(json.dumps(dict(status='PASS' if rc==0 and '7 PASS / 0 FAIL' in log else 'FAIL',cases=7,exit_code=rc,all_mapped_cell_models_present=True,delays='zero',formal_equivalence='not established'),indent=2)+'\n')
if rc or '7 PASS / 0 FAIL' not in log:raise SystemExit('Mapped functional failure requires investigation')
