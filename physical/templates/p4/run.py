#!/usr/bin/python3
import os,json,subprocess,time,signal
from pathlib import Path
PHASE=Path(__file__).resolve().parent
RUN=Path(os.environ['P4_RUN']) if 'P4_RUN' in os.environ else Path((PHASE/'run-path.txt').read_text().strip())
SRC=Path('@SOURCE@')
FLOW=Path('@FLOW@')
TOOLS=Path('@TOOLS@')
SUITE=Path('@SUITE@')
def env():
 e=os.environ.copy();e.update(TMPDIR=str(RUN/'tmp'),XDG_CACHE_HOME=str(RUN/'cache'),XDG_CONFIG_HOME=str(RUN/'config'),XDG_RUNTIME_DIR=str(RUN/'runtime'),QT_QPA_PLATFORM='offscreen',KLAYOUT_HOME=str(RUN/'config/klayout'),PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2',MKL_NUM_THREADS='2',P4_RUN=str(RUN),P4_PHASE=str(PHASE));e['PATH']=str(SUITE)+':'+e['PATH'];return e
def execute(name,command,cwd=None,timeout=3600,accept_failure=False):
 log=RUN/'console'/f'{name}.log';rss=RUN/'timing'/f'{name}.txt'
 if log.exists() or rss.exists():raise RuntimeError('Existing output '+name)
 t=time.monotonic();rc=None
 with log.open('x') as out:
  p=subprocess.Popen(['taskset','-c','0,1','/usr/bin/time','-v','-o',str(rss)]+list(map(str,command)),cwd=cwd or RUN,env=env(),stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
  try:rc=p.wait(timeout=timeout)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
   rc=124
 record=dict(name=name,command=list(map(str,command)),cwd=str(cwd or RUN),exit_code=rc,elapsed_seconds=time.monotonic()-t,disk_bytes=int(subprocess.check_output(['du','-sb',str(RUN)]).split()[0]))
 with (RUN/'audit/stages.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
 print(f'{name}: exit={rc}, wall={record["elapsed_seconds"]:.2f}s; {log}',flush=True)
 if rc and not accept_failure:raise SystemExit(rc)
 return rc,log.read_text(errors='replace')

def make(target,variant='hold50'):
 return ['make','-j1','-f',FLOW/'Makefile',f'FLOW_HOME={FLOW}',f'WORK_HOME={RUN}',f'DESIGN_CONFIG={PHASE}/config.mk',f'FLOW_VARIANT={variant}','NUM_CORES=2',f'OPENROAD_EXE={TOOLS}/bin/openroad',f'YOSYS_EXE={SUITE}/yosys',f'KLAYOUT_CMD={TOOLS}/bin/klayout','PYTHON_EXE=/usr/bin/python3',target]
