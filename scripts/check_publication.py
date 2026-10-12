#!/usr/bin/env python3
"""Read-only publication and routed-revision checks; no EDA tools are invoked."""
import argparse
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import subprocess
import sys
import tarfile
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
RECORD = 'evidence/routed-20261012/results.json'
BINDING = 'evidence/routed-20261012/binding.json'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read(root, name):
    path = Path(name)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('Unsafe publication member: ' + name)
    target = (root / path).resolve()
    if not target.is_relative_to(root.resolve()) or not target.is_file():
        raise ValueError('Missing publication member: ' + name)
    return target


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate_routed(root):
    binding = json.loads(read(root, BINDING).read_text())
    for name, entry in binding['records'].items():
        require(sha(read(root, name)) == entry['sha256'], 'Routed record drift: ' + name)
    for name, entry in binding['figures'].items():
        path = read(root, name)
        require(sha(path) == entry['sha256'] and path.stat().st_size == entry['bytes'],
                'Figure drift: ' + name)
    for name, expected in binding.get('support_sha256', {}).items():
        require(sha(read(root, name)) == expected, 'Publication helper drift: ' + name)
    record = json.loads(read(root, RECORD).read_text())
    freeze = json.loads(read(root, 'evidence/rtl-closure-freeze.json').read_text())
    requalification = json.loads(read(root, 'evidence/requalification.json').read_text())
    source = read(root, record['source_manifest_file'])
    require(sha(source) == record['source_manifest_sha256'] == binding['source_manifest_sha256'],
            'Routed source manifest mismatch')
    require(record['closure_freeze_sha256'] == sha(read(root, 'evidence/rtl-closure-freeze.json')),
            'Routed closure seal mismatch')
    require(record['rtl_sha256'] == freeze['rtl_sha256'], 'Routed RTL inventory mismatch')
    for name, expected in freeze['functional_input_sha256'].items():
        require(sha(read(root, name)) == expected, 'Frozen functional input drift: ' + name)
    seed = json.loads(read(root, requalification['v3_synthesis']['checkpoint_record']).read_text())
    require(all(seed.get(k) == v for k, v in record['input_checkpoint_binding'].items()), 'Routed checkpoint differs from requalification')
    require(record['source_manifest_sha256'] == seed['source_manifest_sha256'], 'Wrong seed source')
    require(record['parameters'] == requalification['v3_synthesis']['parameters'] and
            record['clock_period_ns'] == requalification['v3_synthesis']['clock_period_ns'],
            'Routed profile/clock mismatch')
    require(record['status'] == 'PASS_WITHIN_DOCUMENTED_RESEARCH_SCOPE' and
            all(x == 'PASS' for x in record['P_and_R_stage_status'].values()), 'Incomplete routed stages')
    require(record['GLS']['status'] == 'PASS' and record['GLS']['original_cases'] == 7 and
            record['GLS']['error_recovery_jobs'] == 2 and not record['GLS']['full_top_equivalence'],
            'Wrong GLS scope/count')
    require(record['GLS']['netlist_sha256'] == record['artifacts']['v']['sha256'],
            'GLS does not bind final netlist')
    require(set(record['artifacts']) == {'v', 'sdc', 'odb', 'spef', 'gds', 'def'}, 'Incomplete final artifacts')
    require(record['qualified_LVS'] == 'UNSUPPORTED' and record['native_LEF_antenna'] == 'INCONCLUSIVE',
            'Unsupported signoff claim')
    prefix = 'evidence/routed-20261012/'
    mapping = json.loads(read(root, prefix + 'mapping.json').read_text())
    cdc = json.loads(read(root, prefix + 'cdc.json').read_text())
    coverage = json.loads(read(root, prefix + 'coverage.json').read_text())
    gds = json.loads(read(root, prefix + 'gds-check.json').read_text())
    gate = json.loads(read(root, prefix + 'gds-gate.json').read_text())
    require(mapping['status'] == cdc['status'] == coverage['status'] == gate['status'] == 'PASS',
            'Incomplete compact qualification')
    require(mapping['sequential_cells'] == record['FFs'] == seed['sequential'] and
            mapping['clock_domains'] == record['clock_domain_FFs'] == seed['clock_domains'],
            'FF/clock-domain mismatch')
    require(mapping['total_exported_cells'] == record['geometry']['instance_categories']['standard'],
            'Cell inventory mismatch')
    require(cdc['domain_wns'] == record['final_timing']['wns'], 'Timing summary mismatch')
    require(cdc['bounded_paths'] == 602 and cdc['event_first_stage_exceptions'] == 3 and
            cdc['active_interstage_checks'] == 26 and mapping['synchronizer_chains'] == 13,
            'CDC scope mismatch')
    require(coverage['unconstrained_functional_endpoints'] == 0 and coverage['propagated_clocks'] and
            not coverage['broad_clock_cut'] and coverage['unclassified_exceptions'] == 0,
            'Constraint coverage failure')
    require(all(value == [0, 0.0] for value in record['final_timing']['violations'].values()) and
            all(x >= 0 for x in record['final_timing']['wns'].values()) and
            all(x == 0 for x in record['electrical_violations'].values()), 'Timing/electrical failure')
    require(gds['status'] == 'COMPLETE' and gds['exit_code'] == 0 and
            gds['marker_count'] == gds['antenna_count'] == 0 and gate == record['GDS_gate'] and
            gate['rule_categories'] == 133, 'Incomplete/nonzero GDS rule checks')
    require(gds['gds_sha256'] == record['artifacts']['gds']['sha256'] and
            gds['odb_sha256'] == record['artifacts']['odb']['sha256'], 'GDS checks bind wrong artifacts')
    for entry in binding['figures'].values():
        if 'gds_sha256' in entry:
            require(entry['gds_sha256'] == record['artifacts']['gds']['sha256'], 'Wrong layout GDS binding')
    return binding, record


def validate_archive(path, binding, record):
    require(path.stat().st_size == record['external_archive']['bytes'] and
            sha(path) == record['external_archive']['sha256'], 'External archive identity mismatch')
    expected = {x['archive_path']: x['sha256'] for x in binding['records'].values() if 'archive_path' in x}
    expected.update({x['archive_path']: x['sha256'] for x in binding['figures'].values() if 'archive_path' in x})
    expected.update({'final-delivery/' + x['path']: x['sha256'] for x in record['artifacts'].values()})
    manifest_name = record['final_manifest_archive_path']
    expected[manifest_name] = record['final_manifest_sha256']
    manifest = None
    seen = set()
    with tarfile.open(path, 'r|gz') as archive:
        for member in archive:
            if member.name not in expected:
                continue
            require(member.isfile() and member.name not in seen, 'Invalid/duplicate archive member')
            stream = archive.extractfile(member)
            if member.name == manifest_name:
                data = stream.read()
                manifest = json.loads(data)
                digest = hashlib.sha256(data).hexdigest()
            else:
                h = hashlib.sha256()
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    h.update(chunk)
                digest = h.hexdigest()
            require(digest == expected[member.name], 'Archived evidence mismatch: ' + member.name)
            seen.add(member.name)
    require(seen == set(expected), 'Missing archived evidence member')
    for key in ('status', 'parameters', 'clock_period_ns', 'source_manifest_sha256', 'final_timing',
                'geometry', 'GLS', 'model', 'GDS_gate', 'FFs', 'clock_domain_FFs', 'input_checkpoint_binding'):
        require(manifest[key] == record[key], 'Public projection differs from archive: ' + key)
    for name, item in record['artifacts'].items():
        require(all(manifest['artifacts'][name][k] == item[k] for k in ('path', 'bytes', 'sha256')),
                'Final artifact projection mismatch: ' + name)
    print('ROUTED_ARCHIVE_PASS', len(seen), 'bound manifest/artifact/report/figure members; no extraction/EDA')


def validate_html_links(root):
    sys.path.insert(0, str(ROOT))
    from scripts.check_portfolio import anchors, prose

    class Links(HTMLParser):
        def __init__(self):
            super().__init__()
            self.links = []

        def handle_starttag(self, tag, attrs):
            for key, value in attrs:
                if key in ('href', 'src') and value:
                    self.links.append(value)

    count = 0
    for path in root.rglob('*.md'):
        if '.git' in path.relative_to(root).parts:
            continue
        parser = Links()
        parser.feed(prose(path.read_text()))
        for link in parser.links:
            parsed = urlsplit(link)
            if parsed.scheme or parsed.netloc:
                continue
            name = path.parent / unquote(parsed.path) if parsed.path else path
            target = name.resolve()
            require(target.is_relative_to(root.resolve()) and target.exists(),
                    'Missing HTML link: ' + str(path.relative_to(root)) + ': ' + link)
            if parsed.fragment and target.suffix == '.md':
                require(unquote(parsed.fragment) in anchors(target.read_text()),
                        'Missing HTML anchor: ' + link)
            count += 1
    print('HTML_LINKS_PASS', count, 'relative href/src targets')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, help='Optional sealed external tar.gz; read-only')
    args = parser.parse_args()
    # Existing freeze, historical pins/templates/figures and Markdown checks remain unchanged.
    subprocess.run([sys.executable, str(ROOT / 'scripts/check_portfolio.py')], check=True)
    validate_html_links(ROOT)
    binding, record = validate_routed(ROOT)
    if args.archive:
        validate_archive(args.archive, binding, record)
    print('PUBLICATION_PASS frozen inputs; independent routed revision; figures; no EDA execution')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, OSError, subprocess.CalledProcessError) as error:
        print('PUBLICATION_FAIL:', error, file=sys.stderr)
        raise SystemExit(1)
