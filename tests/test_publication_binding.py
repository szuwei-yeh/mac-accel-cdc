"""Publication must reject misbound artifacts and overstated qualification scope."""
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from scripts.check_publication import BINDING, RECORD, ROOT, validate_routed


class PublicationBindingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        binding = json.loads((ROOT / BINDING).read_text())
        freeze = json.loads((ROOT / 'evidence/rtl-closure-freeze.json').read_text())
        paths = set(binding['records']) | set(binding['figures']) | set(binding.get('support_sha256', {}))
        paths |= set(freeze['functional_input_sha256'])
        paths |= {BINDING, RECORD, 'evidence/rtl-closure-freeze.json', 'evidence/requalification.json',
                  'evidence/frozen-v3-requalification-inputs.json', 'evidence/qualified-frozen-20261012.json'}
        for name in paths:
            dst = self.root / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, dst)

    def reseal_record(self, name, change):
        path = self.root / name
        record = json.loads(path.read_text())
        change(record)
        path.write_text(json.dumps(record))
        binding = json.loads((self.root / BINDING).read_text())
        binding['records'][name]['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        (self.root / BINDING).write_text(json.dumps(binding))

    def test_valid_exact_binding(self):
        validate_routed(self.root)

    def test_frozen_functional_drift(self):
        (self.root / 'rtl/mac_dma_rob.v').write_text('different RTL')
        with self.assertRaisesRegex(ValueError, 'functional input drift'):
            validate_routed(self.root)

    def test_wrong_seed_despite_new_record_seal(self):
        self.reseal_record(RECORD, lambda x: x['input_checkpoint_binding']['artifact_sha256'].update(
            {'1_synth_qualified.odb': '0' * 64}))
        with self.assertRaisesRegex(ValueError, 'checkpoint differs'):
            validate_routed(self.root)

    def test_final_netlist_must_be_tested(self):
        self.reseal_record(RECORD, lambda x: x['artifacts']['v'].update({'sha256': '0' * 64}))
        with self.assertRaisesRegex(ValueError, 'GLS does not bind'):
            validate_routed(self.root)

    def test_layout_bytes_drift(self):
        image = self.root / 'docs/figures/routed-20261012-overview.png'
        image.write_bytes(image.read_bytes() + b'corruption')
        with self.assertRaisesRegex(ValueError, 'Figure drift'):
            validate_routed(self.root)

    def test_missing_check_is_not_pass(self):
        (self.root / 'evidence/routed-20261012/gds-check.json').unlink()
        with self.assertRaisesRegex(ValueError, 'Missing publication member'):
            validate_routed(self.root)

    def test_unconstrained_endpoint_is_not_pass(self):
        self.reseal_record('evidence/routed-20261012/coverage.json',
                           lambda x: x.update({'unconstrained_functional_endpoints': 1}))
        with self.assertRaisesRegex(ValueError, 'coverage failure'):
            validate_routed(self.root)

    def test_cannot_relabel_lvs_as_pass(self):
        self.reseal_record(RECORD, lambda x: x.update({'qualified_LVS': 'PASS'}))
        with self.assertRaisesRegex(ValueError, 'Unsupported signoff'):
            validate_routed(self.root)
