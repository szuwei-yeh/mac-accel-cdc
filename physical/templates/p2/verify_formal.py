#!/usr/bin/python3
import json
from run import *
results=[]
# Preserve published small formal instances and assumptions; depth-8 behavioral
# configuration is checked separately by RTL regression, not inferred from proof.
for name,tasks in [('mac_fifo_gray',['bmc','prove']),('mac_dma_bp',['bmc','prove']),('axi_read_engine_rob',['bmc','prove','cover'])]:
 directory=RUN/'verification'/('formal-'+name);directory.mkdir()
 source=SRC/'formal'/f'{name}.sby'
 config=source.read_text()
 for p in ['mac_fifo_async','mac_dma','axi_read_engine_rob']:
  config=config.replace('../rtl/'+p+'.v',str(SRC/'rtl'/f'{p}.v'))
 dest=directory/(name+'.sby');dest.write_text(config)
 for task in tasks:
  rc,log=execute('formal-'+name+'-'+task,[SUITE/'sby','-j','1','-d',directory/task,dest,task],cwd=directory,timeout=600,accept_failure=True)
  statusfile=directory/task/'status';status=statusfile.read_text().strip() if statusfile.exists() else 'NO_STATUS'
  results.append(dict(suite=name,task=task,status=status,exit_code=rc,config=str(dest)))
  (RUN/'audit/formal-results.json').write_text(json.dumps(results,indent=2)+'\n')
  if status.startswith('FAIL'):raise SystemExit('Formal counterexample requires investigation: '+name+'/'+task)
print(json.dumps(results,indent=2),flush=True)
