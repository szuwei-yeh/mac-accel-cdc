"""Fail-closed source binding controls; fixtures are synthetic, not EDA evidence."""
import hashlib
import json
import tempfile
from pathlib import Path
import unittest
from scripts.revision_evidence import relative_file, validate_sources, validate_synthesis_binding


class RevisionBindingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.path = 'rtl/example.v'
        (self.root / 'rtl').mkdir()
        (self.root / self.path).write_text('new source')
        archived = self.root / 'evidence/historical' / self.path
        archived.parent.mkdir(parents=True)
        archived.write_text('old source')
        self.historical = {self.path: 'evidence/historical/' + self.path}
        self.pins = {'source_sha256': {self.path: self.hash('old source')}}
        self.current = {self.path: self.hash('new source')}

    @staticmethod
    def hash(s):
        return hashlib.sha256(s.encode()).hexdigest()

    def check(self):
        validate_sources(self.root, self.pins, self.historical, self.current)

    def test_explicit_two_revisions(self):
        self.check()

    def test_current_drift_fails(self):
        (self.root / self.path).write_text('unverified source')
        with self.assertRaisesRegex(ValueError, 'Current source hash'):
            self.check()

    def test_historical_drift_fails(self):
        (self.root / self.historical[self.path]).write_text('changed history')
        with self.assertRaisesRegex(ValueError, 'Historical source hash'):
            self.check()

    def test_missing_historical_pin_fails(self):
        self.historical.clear()
        with self.assertRaisesRegex(ValueError, 'inventory'):
            self.check()

    def test_missing_current_pin_fails(self):
        self.current.clear()
        with self.assertRaisesRegex(ValueError, 'omits'):
            self.check()

    def test_unrestricted_archive_alias_fails(self):
        self.historical[self.path] = self.path + '.anything'
        with self.assertRaisesRegex(ValueError, 'archive binding'):
            self.check()

    def test_unsafe_paths_and_missing_files_fail(self):
        for name in ['/absolute', '../outside', 'missing']:
            with self.subTest(name=name), self.assertRaises(ValueError):
                relative_file(self.root, name)

    def test_wrong_profile_fails(self):
        with self.assertRaisesRegex(ValueError, 'profile'):
            validate_synthesis_binding(self.root, {'v3_synthesis': {'parameters': {}, 'clock_period_ns': {}}})

    def synthesis_fixture(self):
        paths = ['rtl/mac_accel_dma_rob_top.v', 'rtl/mac_dma_rob.v', 'rtl/axi_read_engine_rob.v',
                 'rtl/mac_fifo_async.v', 'rtl/mac_pe.v']
        profile = dict(DATA_WIDTH=16, AXI_DATA_WIDTH=32, AXI_ADDR_WIDTH=32, AXI_ID_WIDTH=4,
                       S_AXI_ADDR_WIDTH=8, MAX_LEN=256, BURST_LEN=16, MAX_OUTSTANDING=8)
        inputs = paths + ['physical/templates/p2/' + n for n in
                         ('config.mk', 'constraint.sdc', 'canonicalize.tcl', 'synth.tcl',
                          'audit_mapping.py', 'elaboration_audit.py')]
        for name in inputs:
            p = self.root / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(name)
        hashes = {n: self.hash(n) for n in inputs}
        frozen = dict(source_sha256=hashes, parameters=profile, clock_period_ns={'bus': 10.0, 'mac': 7.5})
        (self.root / 'frozen.json').write_text(json.dumps(frozen))
        seal = self.hash(json.dumps(frozen))
        artifact = {'mapped.v': self.hash('synthetic mapped netlist')}
        (self.root / 'checkpoint.json').write_text(json.dumps(dict(status='PASS', source_manifest_sha256=seal, artifact_sha256=artifact)))
        return {'v3_synthesis': dict(parameters=profile, clock_period_ns=frozen['clock_period_ns'],
                                    source_list=paths, qualified_input_sha256=hashes,
                                    frozen_manifest_file='frozen.json', frozen_manifest_sha256=seal,
                                    checkpoint_record='checkpoint.json', artifact_sha256=artifact)}

    def test_frozen_checkpoint_binding(self):
        validate_synthesis_binding(self.root, self.synthesis_fixture())

    def test_changed_reachable_source_cannot_reuse_checkpoint(self):
        m = self.synthesis_fixture()
        name = m['v3_synthesis']['source_list'][0]
        (self.root / name).write_text('changed RTL')
        m['v3_synthesis']['qualified_input_sha256'][name] = self.hash('changed RTL')
        with self.assertRaisesRegex(ValueError, 'do not belong'):
            validate_synthesis_binding(self.root, m)

    def test_wrong_checkpoint_source_manifest_fails(self):
        m = self.synthesis_fixture()
        p = self.root / 'checkpoint.json'
        record = json.loads(p.read_text())
        record['source_manifest_sha256'] = 'wrong source'
        p.write_text(json.dumps(record))
        with self.assertRaisesRegex(ValueError, 'another frozen'):
            validate_synthesis_binding(self.root, m)

    def test_wrong_artifact_identity_fails(self):
        m = self.synthesis_fixture()
        m['v3_synthesis']['artifact_sha256'] = {}
        with self.assertRaisesRegex(ValueError, 'artifact identity'):
            validate_synthesis_binding(self.root, m)
