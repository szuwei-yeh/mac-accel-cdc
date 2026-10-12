#!/usr/bin/env python3
"""Export a verified, UNPLACED pre-layout V3 checkpoint; never run P&R."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys
from revision_evidence import validate_evidence

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def verify_qualification(run, readback, revision, artifact_dir=None, pending_tail=None):
    expected = revision['v3_synthesis']
    status = json.loads((run / 'audit/status.json').read_text())
    require(status['status'] == 'PASS' and status['physical_stages_executed'] is False,
            'Requires completed pre-layout qualification, not a physical checkpoint')
    require(status['source_manifest_sha256'] == expected['frozen_manifest_sha256'],
            'Wrong synthesis source snapshot')
    require(status['artifact_sha256'] == expected['artifact_sha256'], 'Wrong qualified artifact identity')
    res = artifact_dir if artifact_dir is not None else run / 'results/nangate45/mac_accel_dma_rob_top/base'
    for name, digest in expected['artifact_sha256'].items():
        require(sha(res / name) == digest, 'Checkpoint byte mismatch: ' + name)
    mapping = json.loads((run / 'audit/mapping-gate.json').read_text())
    require(mapping['status'] == 'PASS' and mapping['sequential_count'] == 7509
            and mapping['clock_domains'] == {'bus': 7287, 'mac': 222}
            and not mapping['unmapped'] and not mapping['latches'], 'Mapped structure mismatch')
    require(mapping['classified_pairs'] == dict(start=1, done=1, busy=1, write_gray=5,
                                               read_gray=5, payload=528, result=32, latency=32),
            'CDC crossing classification mismatch')
    require(len(mapping['synchronizer_chains']) == 13, 'Synchronizer count mismatch')
    identity = json.loads((run / 'audit/sequential-identity-proof.json').read_text())
    mapped = json.loads((run / 'audit/mapped.json').read_text())['modules']['mac_accel_dma_rob_top']
    cells = {n for n, c in mapped['cells'].items() if c['type'].startswith('DFF')}
    require(len(identity['bijection']) == 7509 and set(identity['bijection'].values()) == cells,
            'Incomplete mapped FF identity bijection')
    require(all(row['same_ff_clock_state_reset_and_output_functions']
                for row in identity['sized_ff_logic_equivalence']), 'Changed sized FF function')
    coverage = json.loads((run / 'audit/coverage-final-gate.json').read_text())
    require(coverage['status'] == 'PASS' and coverage['standalone_and_openroad_all_pair_slacks_identical']
            and coverage['unconstrained_functional_endpoints'] == 0
            and coverage['input_bits_with_min_and_max'] == 104
            and coverage['output_bits_with_min_and_max'] == 93 and not coverage['broad_clock_cut'],
            'Constraint coverage qualification incomplete')
    require((run / 'audit/sta-paths.tsv').read_bytes() == (run / 'audit/sta-paths-openroad.tsv').read_bytes(),
            'STA engines disagree')
    for engine in ['independent-sta-qualified', 'openroad-sta-qualified']:
        log = (run / 'console' / (engine + '.log')).read_text()
        wns = dict(re.findall(r'worst slack (max|min)\s+(-?\d+\.\d+)', log))
        tns = dict(re.findall(r'tns (max|min)\s+(-?\d+\.\d+)', log))
        require(set(wns) == {'max', 'min'} and all(float(v) >= 0 for v in wns.values()),
                'Missing/negative required STA slack')
        require(set(tns) == {'max', 'min'} and all(float(v) == 0 for v in tns.values()), 'Nonzero/missing TNS')
        require('P2_STA_AUDIT_PASS' in log and len(re.findall(r'^SYNC_INTERSTAGE ', log, re.M)) == 26,
                'Incomplete CDC/interstage timing audit')
        electrical = log.split('ELECTRICAL_BEGIN')[1].split('ELECTRICAL_END')[0]
        require('(VIOLATED)' not in electrical, 'Electrical qualification failed')
    gls = json.loads((run / 'audit/gls-result.json').read_text())
    require(gls['status'] == 'PASS' and gls['cases'] == 7, 'Mapped GLS incomplete')
    fault = (run / 'console/mapped-fault-gls.log').read_text()
    require(re.findall(r'TOTAL:\s*(\d+) PASS / (\d+) FAIL', fault) == [('2', '0')]
            and not re.search(r'\b(?:FATAL|ERROR)\b|\bFAIL\s*:|timeout|abort', fault, re.I),
            'Mapped error-recovery GLS failed')
    rb = json.loads((readback / 'status.json').read_text())
    require(rb['status'] == 'PASS' and rb['readback_Verilog_sha256'] == expected['artifact_sha256']['1_2_yosys.v']
            and sha(readback / 'readback.v') == rb['readback_Verilog_sha256'], 'OpenDB readback mismatch')
    if 'rtl_closure_binding' in revision:
        require(pending_tail is not None, 'Requalified closure requires mapped pending-tail GLS')
        tail = json.loads((pending_tail / 'status.json').read_text())
        require(tail['status'] == 'PASS' and tail['jobs'] == 2,
                'Mapped pending-tail verification incomplete')
        require(tail['full_top_qualified_netlist_sha256'] == expected['artifact_sha256']['1_2_yosys.v'],
                'Pending-tail evidence associated with another top checkpoint')
        sources = revision['current_source_sha256']
        require(tail['source_sha256'] == {n: sources[n] for n in
                                        ['rtl/mac_dma_rob.v', 'rtl/axi_read_engine_rob.v']},
                'Pending-tail mapped source mismatch')
        require(tail['original_TB_sha256'] == sources['sim/tb_mac_dma_rob_pending_tail.v'],
                'Pending-tail TB does not belong to frozen closure')
        require(sha(pending_tail / 'mac_dma_rob_mapped.v') == tail['netlist_sha256']
                and sha(pending_tail / 'tb_pending_tail_mapped.v') == tail['mapped_TB_sha256'],
                'Pending-tail mapped artifact mismatch')
        require(sha(run / 'verification/nangate45-functional.v') == tail['cell_model_sha256'],
                'Mapped cell models mismatch')
        log = (pending_tail / 'console/mapped-tail-gls.log').read_text()
        require(re.findall(r'TOTAL:\s*(\d+) PASS / (\d+) FAIL', log) == [('2', '0')]
                and not re.search(r'\b(?:FATAL|ERROR)\b|\bFAIL\s*:|timeout|abort', log, re.I),
                'Pending-tail GLS failed')
    return res, status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--qualified-run', type=Path, required=True)
    parser.add_argument('--readback-run', type=Path, required=True)
    parser.add_argument('--platform', type=Path, required=True)
    parser.add_argument('--pending-tail', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    pins = json.loads((ROOT / 'physical/pins.json').read_text())
    revision = validate_evidence(ROOT, pins)
    res, status = verify_qualification(args.qualified_run, args.readback_run, revision,
                                       pending_tail=args.pending_tail)
    for name, digest in pins['platform_sha256'].items():
        require(sha(args.platform / name) == digest, 'Platform mismatch: ' + name)
    out = args.output.resolve()
    require(not out.exists(), 'Use a new package output directory')
    out.mkdir(parents=True)
    source_files = set(revision['current_source_sha256']) | set(revision['historical_sources'].values()) | set(revision['support_source_sha256'])
    source_files |= {'evidence/revisions.json', 'physical/pins.json', 'scripts/revision_evidence.py',
                     revision['v3_synthesis']['checkpoint_record'],
                     revision['v3_synthesis']['frozen_manifest_file']}
    if 'rtl_closure_binding' in revision:
        source_files |= {'evidence/requalification.json', 'evidence/rtl-closure-freeze.json',
                         'RTL_CLOSURE.md'}
    for name in sorted(source_files):
        dst = out / 'source' / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, dst)
    for name in status['artifact_sha256']:
        dst = out / 'checkpoint' / name
        dst.parent.mkdir(exist_ok=True)
        shutil.copyfile(res / name, dst)
    for name in pins['platform_sha256']:
        dst = out / 'platform/nangate45' / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(args.platform / name, dst)
    # Raw qualification evidence stays in this external package, not the publication tree.
    for folder in ['audit', 'console', 'timing']:
        src = args.qualified_run / folder
        if src.exists():
            shutil.copytree(src, out / 'evidence/synthesis' / folder,
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '*.vvp'))
    shutil.copytree(args.readback_run, out / 'evidence/opendb-readback')
    if args.pending_tail:
        shutil.copytree(args.pending_tail, out / 'evidence/mapped-pending-tail',
                        ignore=shutil.ignore_patterns('tmp', 'simv', '*.pyc', '__pycache__'))
    shutil.copytree(args.qualified_run / 'verification', out / 'evidence/synthesis/verification',
                    ignore=shutil.ignore_patterns('*.vvp', '*.pyc', '__pycache__'))
    (out / 'profile.json').write_text(json.dumps(dict(top='mac_accel_dma_rob_top',
        parameters=revision['v3_synthesis']['parameters'], clock_period_ns={'bus': 10.0, 'mac': 7.5},
        model='Nangate45 typical 1.1 V / 25 C; ideal clocks, zero wire RC; all instances UNPLACED',
        tools=dict(OpenROAD='26Q2-1164-g08f67ee5ec', OpenSTA='3.1.0', Yosys='0.69+274 7513f8df8',
                   ORFS_commit=pins['orfs_commit']), scope='pre-layout synthesis only; no routed PPA/signoff'), indent=2) + '\n')
    (out / 'V3_SOURCES.f').write_text('\n'.join('source/' + n for n in revision['v3_synthesis']['source_list']) + '\n')
    manifest = dict(schema=1, publication_revision=revision['candidate'], git_baseline=revision['git_baseline'],
        publication_manifest_sha256=sha(ROOT / 'evidence/revisions.json'),
        frozen_V3_manifest_sha256=revision['v3_synthesis']['frozen_manifest_sha256'],
        artifact_sha256=status['artifact_sha256'], excluded='all floorplan/placement/CTS/routing/extraction/GDS outputs',
        files_sha256={str(p.relative_to(out)): sha(p) for p in sorted(out.rglob('*')) if p.is_file()})
    (out / 'PACKAGE_MANIFEST.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('GAUSS_PRELAYOUT_EXPORT_PASS ' + str(out))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
