#!/usr/bin/env python3
"""V3 supplemental CDC regression; generated artifacts stay in unique /tmp runs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time

SOURCES = ['sim/tb_mac_accel_dma_rob_cdc.sv', 'rtl/mac_accel_dma_rob_top.v',
           'rtl/mac_dma_rob.v', 'rtl/axi_read_engine_rob.v', 'rtl/mac_fifo_async.v',
           'rtl/mac_pe.v', 'sim/axi_read_mem_model_ooo.v']
# bus period is 10 ns. Values are MAC period, initial phase, bus-reset-first.
SCENARIOS = [('reference', '7.500', '0.000', 0),
             ('mac_slower', '17.000', '1.250', 0),
             ('equal_aligned', '10.000', '0.000', 0),
             ('equal_shifted', '10.000', '2.500', 1),
             ('near_edges', '7.502', '1.250', 0),
             ('mac_faster', '3.500', '0.375', 1)]
VCS = '/ece/synopsys/vcs/V-2023.12-SP2/bin/vcs'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--simulator', choices=('iverilog', 'vcs'), default='iverilog')
    args = parser.parse_args()
    project = args.project.resolve()
    work = Path(tempfile.mkdtemp(prefix='vectormac-cdc-supp-', dir='/tmp'))
    print('WORKSPACE=' + str(work), flush=True)
    for name in ('inputs', 'build', 'reports', 'tmp'):
        (work/name).mkdir()
    hashes = {}
    for name in SOURCES:
        dest = work/'inputs'/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(str(project/name), str(dest))
        hashes[name] = sha(dest)
        if hashes[name] != sha(project/name):
            raise RuntimeError('Source changed during snapshot: ' + name)
    env = dict(os.environ, TMPDIR=str(work/'tmp'))
    if args.simulator == 'vcs':
        env['VCS_HOME'] = str(Path(VCS).parent.parent)
    commands = []

    def run(label, argv, cwd):
        argv = [str(a) for a in argv]
        start = time.monotonic()
        logpath = work/'reports'/(label + '.log')
        with logpath.open('w') as log:
            try:
                proc = subprocess.run(argv, cwd=str(cwd), env=env, stdout=log,
                                      stderr=subprocess.STDOUT, timeout=180)
                code = proc.returncode
            except subprocess.TimeoutExpired:
                code = 124
                log.write('\nRUNNER_TIMEOUT\n')
        commands.append(dict(label=label, argv=argv, cwd=str(cwd), exit_code=code,
                             seconds=time.monotonic()-start))
        (work/'reports/commands.json').write_text(json.dumps(commands, indent=2)+'\n')
        return code, logpath.read_text(errors='replace')

    meta = dict(project=str(project), workspace=str(work), simulator=args.simulator,
                head=subprocess.check_output(['git','-C',str(project),'rev-parse','HEAD'], universal_newlines=True).strip(),
                source_sha256=hashes, runner_sha256=sha(Path(__file__).resolve()),
                scenarios=SCENARIOS, bus_period_ns=10, max_outstanding=8,
                lengths=[1,17,129,256,33,1], reset_policy='one startup reset; zero inter-job resets')
    (work/'reports/metadata.json').write_text(json.dumps(meta, indent=2)+'\n')
    run('version', ['iverilog','-V'] if args.simulator=='iverilog' else [VCS,'-ID'], work/'build')
    files = [work/'inputs'/n for n in SOURCES]
    if args.simulator == 'iverilog':
        compile_cmd = ['iverilog','-g2012','-Wall','-s','tb_mac_accel_dma_rob_cdc','-o','simv']+files
    else:
        compile_cmd = [VCS,'-full64','-sverilog']+files+['-top','tb_mac_accel_dma_rob_cdc','-o','simv']
    code, log = run('compile',compile_cmd,work/'build')
    if code:
        raise RuntimeError('Compilation failed: '+str(work/'reports/compile.log'))
    results = []
    for name, period, phase, reset_first in SCENARIOS:
        cwd = work/name
        cwd.mkdir()
        sim = ['vvp',str(work/'build/simv')] if args.simulator=='iverilog' else [str(work/'build/simv')]
        sim += ['+MAC_PERIOD='+period, '+MAC_PHASE='+phase, '+RESET_BUS_FIRST='+str(reset_first)]
        code, log = run(name,sim,cwd)
        jobs = [dict((k,int(v)) for k,v in re.findall(r'(\w+)=(-?\d+)',line))
                for line in log.splitlines() if line.startswith('CDC_JOB_PASS ')]
        passed = (code == 0 and 'CDC_TOTAL: 6 PASS / 0 FAIL; startup_resets=1; interjob_resets=0' in log
                  and [j.get('len') for j in jobs] == meta['lengths']
                  and [j.get('toggle') for j in jobs] == [1,0,1,0,1,0]
                  and all(j.get('peak')==8 for j in jobs if j.get('len',0)>=129)
                  and not re.search(r'CDC_FAIL|\bFATAL\b|RUNNER_TIMEOUT|Error-',log))
        results.append(dict(scenario=name, status='PASS' if passed else 'FAIL', jobs=jobs))
        if (cwd/'cdc_events.csv').is_file():
            shutil.copyfile(str(cwd/'cdc_events.csv'), str(work/'reports'/(name+'_events.csv')))
        print(name+': '+results[-1]['status'],flush=True)
    changed = [n for n in hashes if hashes[n]!=sha(project/n)]
    result = dict(status='PASS' if all(r['status']=='PASS' for r in results) and not changed else 'FAIL',
                  results=results, changed_sources=changed)
    (work/'reports/results.json').write_text(json.dumps(result,indent=2)+'\n')
    print('CDC_MATRIX_'+result['status']+'\nReports: '+str(work/'reports'))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':
    sys.exit(main())
