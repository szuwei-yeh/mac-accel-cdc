"""Validate explicit current/historical source identities without changing old pins."""
import ast
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def relative_file(root, name):
    path = Path(name)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('Unsafe evidence path: ' + name)
    result = (root / path).resolve()
    if not result.is_relative_to(root.resolve()) or not result.is_file():
        raise ValueError('Missing/outside evidence file: ' + name)
    return result


def validate_sources(root, pins, historical, current):
    """Every old pin is checked; each current file has an independent exact hash."""
    if set(historical) != set(pins['source_sha256']):
        raise ValueError('Historical source inventory differs from immutable pins')
    if not set(historical).issubset(current):
        raise ValueError('Current inventory omits a historically pinned path')
    for name, expected in pins['source_sha256'].items():
        stored = historical[name]
        if stored not in (name, 'evidence/historical/' + name):
            raise ValueError('Unexpected historical archive binding: ' + name)
        if sha(relative_file(root, stored)) != expected:
            raise ValueError('Historical source hash mismatch: ' + name)
    for name, expected in current.items():
        if sha(relative_file(root, name)) != expected:
            raise ValueError('Current source hash mismatch: ' + name)


def validate_synthesis_binding(root, manifest):
    binding = manifest['v3_synthesis']
    expected_profile = dict(DATA_WIDTH=16, AXI_DATA_WIDTH=32, AXI_ADDR_WIDTH=32,
                            AXI_ID_WIDTH=4, S_AXI_ADDR_WIDTH=8, MAX_LEN=256,
                            BURST_LEN=16, MAX_OUTSTANDING=8)
    if binding['parameters'] != expected_profile or binding['clock_period_ns'] != {'bus': 10.0, 'mac': 7.5}:
        raise ValueError('Qualified V3 profile changed')
    sources = ['rtl/mac_accel_dma_rob_top.v', 'rtl/mac_dma_rob.v',
               'rtl/axi_read_engine_rob.v', 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v']
    if binding['source_list'] != sources:
        raise ValueError('V3 reachable-source inventory changed')
    expected = binding['qualified_input_sha256']
    required = set(sources) | {'physical/templates/p2/' + n for n in
                            ('config.mk', 'constraint.sdc', 'canonicalize.tcl',
                             'synth.tcl', 'audit_mapping.py', 'elaboration_audit.py')}
    if set(expected) != required:
        raise ValueError('Qualified flow/input inventory incomplete')
    frozen_path = relative_file(root, binding['frozen_manifest_file'])
    if sha(frozen_path) != binding['frozen_manifest_sha256']:
        raise ValueError('Frozen input manifest seal mismatch')
    frozen = json.loads(frozen_path.read_text())
    if any(frozen['source_sha256'].get(n) != h for n, h in expected.items()):
        raise ValueError('Claimed synthesis inputs do not belong to frozen checkpoint')
    if frozen['parameters'] != expected_profile or frozen['clock_period_ns'] != binding['clock_period_ns']:
        raise ValueError('Frozen synthesis profile mismatch')
    for name, digest in expected.items():
        if sha(relative_file(root, name)) != digest:
            raise ValueError('Publication does not implement qualified V3 source: ' + name)
    checkpoint = json.loads(relative_file(root, binding['checkpoint_record']).read_text())
    if checkpoint['source_manifest_sha256'] != binding['frozen_manifest_sha256']:
        raise ValueError('Checkpoint belongs to another frozen source manifest')
    if checkpoint['status'] != 'PASS' or checkpoint['artifact_sha256'] != binding['artifact_sha256']:
        raise ValueError('Qualified artifact identity/status mismatch')
    return binding


def validate_evidence(root, pins):
    # A new qualification is a separate record. The closure-era manifest and
    # old checkpoint remain immutable; presence of a new record is fail-closed.
    requalified = (root / 'evidence/requalification.json').exists()
    name = 'evidence/requalification.json' if requalified else 'evidence/revisions.json'
    manifest = json.loads(relative_file(root, name).read_text())
    if requalified:
        validate_closure_binding(root, manifest)
    if manifest['schema'] != 1 or manifest['historical_rtl_commit'] != pins['rtl_commit']:
        raise ValueError('Historical revision identity mismatch')
    validate_sources(root, pins, manifest['historical_sources'], manifest['current_source_sha256'])
    validate_synthesis_binding(root, manifest)
    for name, expected in manifest['support_source_sha256'].items():
        if sha(relative_file(root, name)) != expected:
            raise ValueError('Evidence/test support source hash mismatch: ' + name)
    if manifest['functional']['status'] != 'PASS':
        raise ValueError('Integrated functional gate is incomplete')
    required = {str(p.relative_to(root)) for directory in ['rtl', 'sim']
                for p in (root / directory).iterdir() if p.suffix in {'.v', '.sv'}}
    required |= {'scripts/run_ci_rtl.py', 'sim/run_cdc_regression.py'}
    if not required.issubset(manifest['current_source_sha256']):
        raise ValueError('Functional inventory missing executable inputs')
    module = ast.parse(relative_file(root, 'scripts/run_ci_rtl.py').read_text())
    node = next(n for n in module.body if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'TESTS' for t in n.targets))
    tests = ast.literal_eval(node.value)
    if len(tests) != len({t[0] for t in tests}) or sum(t[2] for t in tests) != manifest['functional']['unique_ci_cases']:
        raise ValueError('CI inventory/count mismatch')
    if {t[0] for t in tests} != set(manifest['functional']['ci_testbenches']):
        raise ValueError('CI testbench binding mismatch')
    for top in manifest['functional']['new_permanent_testbenches']:
        if top not in {t[0] for t in tests}:
            raise ValueError('New permanent TB is not executed by CI: ' + top)
    return manifest


def validate_closure_binding(root, manifest):
    """Requalification must implement exactly the completed RTL closure."""
    binding = manifest['rtl_closure_binding']
    freeze_path = relative_file(root, 'evidence/rtl-closure-freeze.json')
    if sha(freeze_path) != binding['freeze_sha256']:
        raise ValueError('RTL closure freeze seal mismatch')
    frozen = json.loads(freeze_path.read_text())
    prior = relative_file(root, 'evidence/revisions.json')
    if sha(prior) != frozen['evidence_manifest_sha256']:
        raise ValueError('Closure-era revision manifest changed')
    if sha(relative_file(root, 'docs/RTL_SUPPORTED_CONTRACT.md')) != frozen['contract_sha256']:
        raise ValueError('Frozen interface contract changed')
    if manifest['git_baseline'] != frozen['baseline']:
        raise ValueError('Closure baseline mismatch')
    current = manifest['current_source_sha256']
    for name, digest in frozen['functional_input_sha256'].items():
        if current.get(name) != digest or sha(relative_file(root, name)) != digest:
            raise ValueError('Requalification differs from frozen functional input: ' + name)
    if binding['functional_results'] != frozen['results']:
        raise ValueError('Closure functional-result binding mismatch')
