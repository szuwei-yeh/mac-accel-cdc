"""Requalification cannot replace the frozen RTL/contract or rewrite old evidence."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from scripts.revision_evidence import validate_closure_binding


class ClosureBindingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        for name, text in {'rtl/x.v': 'frozen RTL',
                           'evidence/revisions.json': '{"original": true}',
                           'docs/RTL_SUPPORTED_CONTRACT.md': 'frozen contract'}.items():
            p = self.root / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(text)
        self.freeze = dict(baseline='baseline',
                           evidence_manifest_sha256=self.sha('evidence/revisions.json'),
                           contract_sha256=self.sha('docs/RTL_SUPPORTED_CONTRACT.md'),
                           functional_input_sha256={'rtl/x.v': self.sha('rtl/x.v')},
                           results={'CI_cases': 243, 'stress_jobs': 108, 'cdc_jobs': 36})
        p = self.root / 'evidence/rtl-closure-freeze.json'
        p.write_text(json.dumps(self.freeze))
        self.manifest = dict(git_baseline='baseline',
                             current_source_sha256=dict(self.freeze['functional_input_sha256']),
                             rtl_closure_binding=dict(freeze_sha256=self.sha(str(p.relative_to(self.root))),
                                                      functional_results=dict(self.freeze['results'])))

    def sha(self, name):
        return hashlib.sha256((self.root / name).read_bytes()).hexdigest()

    def check(self):
        validate_closure_binding(self.root, self.manifest)

    def test_exact_frozen_source_binding(self):
        self.check()

    def test_freeze_seal_drift(self):
        (self.root / 'evidence/rtl-closure-freeze.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'freeze seal'):
            self.check()

    def test_old_evidence_is_immutable(self):
        (self.root / 'evidence/revisions.json').write_text('rewritten')
        with self.assertRaisesRegex(ValueError, 'Closure-era'):
            self.check()

    def test_contract_drift(self):
        (self.root / 'docs/RTL_SUPPORTED_CONTRACT.md').write_text('new contract')
        with self.assertRaisesRegex(ValueError, 'contract'):
            self.check()

    def test_current_source_drift_or_omission(self):
        with self.assertRaisesRegex(ValueError, 'functional input'):
            self.manifest['current_source_sha256'].clear()
            self.check()

    def test_false_functional_results(self):
        self.manifest['rtl_closure_binding']['functional_results']['CI_cases'] = 999
        with self.assertRaisesRegex(ValueError, 'functional-result'):
            self.check()
