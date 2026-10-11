#!/usr/bin/python3
import json,re,shutil
from run import *
rtl=['rtl/mac_accel_dma_rob_top.v','rtl/mac_dma_rob.v','rtl/axi_read_engine_rob.v','rtl/mac_fifo_async.v','rtl/mac_pe.v']
tests=[
 ('v3-top','tb_mac_accel_dma_rob_top',['sim/tb_mac_accel_dma_rob_top.v',*rtl,'sim/axi_read_mem_model_ooo.v'],['-DV3_MAX_OUT=8'],7),
 ('rob-ooo','tb_axi_read_engine_rob_ooo',['sim/tb_axi_read_engine_rob_ooo.v','rtl/axi_read_engine_rob.v','sim/axi_read_mem_model_ooo.v'],['-DENG_MAX_OUT=8'],12),
 ('dma-rob','tb_mac_dma_rob',['sim/tb_mac_dma_rob.v','rtl/mac_dma_rob.v','rtl/axi_read_engine_rob.v','sim/axi_read_mem_model_ooo.v'],['-DDMA_ROB_MAX_OUT=8'],11),
]
results=[]
for label,top,files,defs,count in tests:
 work=RUN/'verification'/label;work.mkdir();binary=work/'simv'
 execute(label+'-compile',[SUITE/'iverilog','-g2012','-Wall',*defs,'-s',top,'-o',binary,*[SRC/f for f in files]],cwd=work)
 rc,log=execute(label+'-simulation',[SUITE/'vvp',binary],cwd=work)
 if f'{count} PASS / 0 FAIL' not in log or re.search(r'FATAL|\bFAIL\b(?!\s*[\n;/])',log.replace('0 FAIL','')):
  raise SystemExit('Functional failure: '+label)
 results.append(dict(test=label,status='PASS',cases=count))
# Isolated adaptation changes only the generated workspace root/default source;
# original runner and RTL remain read-only, and all six original scenarios run.
runner=(SRC/'sim/run_cdc_regression.py').read_text()
assert runner.count("dir='/tmp'")==1
runner=runner.replace("dir='/tmp'",f"dir={str(RUN/'verification')!r}")
adapted=RUN/'verification/run_cdc_regression_data.py';adapted.write_text(runner)
rc,log=execute('cdc-matrix',['/usr/bin/python3',adapted,'--project',SRC,'--simulator','iverilog'])
if 'CDC_MATRIX_PASS' not in log:raise SystemExit('CDC functional failure')
results.append(dict(test='cdc-matrix',status='PASS',scenarios=6,cases=36))
(RUN/'audit/rtl-results.json').write_text(json.dumps({'status':'PASS','fresh':True,'results':results},indent=2)+'\n')
