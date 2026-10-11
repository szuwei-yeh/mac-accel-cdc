"""Synthetic validation fixtures only; no archived artifacts or EDA execution."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from physical.acceptance import (
    Rejected, checked_file, finite, sha, table, validate_bundle, validate_gds,
)


class AcceptanceBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="vector-mac-unit-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()

    def test_finite_nonnegative_slack(self):
        self.assertEqual(finite("0.125", "CDC slack"), 0.125)
        self.assertEqual(finite(0, "CDC slack"), 0)
        for value in (-0.001, "nan", "inf", "-inf", None, "invalid"):
            with self.subTest(value=value), self.assertRaises(Rejected):
                finite(value, "CDC slack")

    def test_signed_coordinates(self):
        self.assertEqual(finite(-2, "coordinate", nonnegative=False), -2)

    def test_file_hash_and_size_provenance(self):
        path = self.root / "evidence.json"
        path.write_text('{"synthetic": true}\n')
        record = dict(path=path.name, bytes=path.stat().st_size, sha256=sha(path))
        self.assertEqual(checked_file(record, self.root), path)
        for field, value in (("sha256", "0" * 64), ("bytes", 0), ("path", "missing")):
            with self.subTest(field=field), self.assertRaises(Rejected):
                checked_file(dict(record, **{field: value}), self.root)

    def test_cdc_table_rejects_missing_columns_and_truncated_rows(self):
        path = self.root / "cdc.tsv"
        path.write_text("source\tdestination\na\tb\n")
        self.assertEqual(table(path, ["source", "destination"]),
                         [dict(source="a", destination="b")])
        with self.assertRaises(Rejected):
            table(path, ["max_slack_ns"])
        path.write_text("source\tdestination\na\n")
        with self.assertRaises(Rejected):
            table(path, ["source", "destination"])

    def test_bundle_requires_complete_artifact_inventory(self):
        path = self.root / "bundle.json"
        for bundle in ({"schema": 99}, {"schema": 1, "artifacts": {}}):
            path.write_text(json.dumps(bundle))
            with self.subTest(bundle=bundle), self.assertRaises(Rejected):
                validate_bundle(path)


class SyntheticGdsReportTests(unittest.TestCase):
    """Exercise production GDS-report parsing with tiny fake files and a fake deck."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="vector-mac-gds-unit-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.files = {key: self.root / (key + ".fixture") for key in
                      ("report", "metadata", "gds", "odb", "deck", "execution_log",
                       "execution_record")}
        for key in ("gds", "odb", "deck"):
            self.files[key].write_text("SYNTHETIC UNIT FIXTURE " + key)
        self.xml = (
            '<report-database><top-cell>mac_accel_dma_rob_top</top-cell>'
            f'<generator>{self.files["deck"]}</generator>'
            '<cells><cell><name>mac_accel_dma_rob_top</name></cell></cells>'
            '<categories><category><name>ANTENNA</name>'
            '<description>Synthetic test rule</description></category></categories>'
            '<items></items></report-database>')
        self.files["report"].write_text(self.xml)
        self.files["execution_log"].write_text(
            "Total elapsed: 0\nWriting report database: " + str(self.files["report"]) + "\n")
        self.execution = dict(exit_code=0, command=[str(self.files["deck"]),
                              "in_gds=" + str(self.files["gds"]),
                              "report_file=" + str(self.files["report"])])
        categories = [("ANTENNA", "Synthetic test rule")]
        self.contract = dict(klayout_version="synthetic-unit-version",
                             flat_deck_sha256=sha(self.files["deck"]), category_count=1,
                             category_sha256=hashlib.sha256(json.dumps(
                                 categories, separators=(",", ":")).encode()).hexdigest(),
                             coordinate_limits_um=[0, 500])
        self.metadata = dict(status="COMPLETE", exit_code=0, checker="KLayout",
                             checker_version="synthetic-unit-version",
                             marker_count=0, antenna_count=0)
        self.refresh()

    def refresh(self):
        self.files["execution_record"].write_text(json.dumps(self.execution))
        for key in ("report", "gds", "odb", "deck", "execution_record"):
            self.metadata[key + "_sha256"] = sha(self.files[key])
        self.metadata["log_sha256"] = sha(self.files["execution_log"])
        self.files["metadata"].write_text(json.dumps(self.metadata))

    def validate(self):
        return validate_gds(**self.files, contract=self.contract)

    def test_complete_synthetic_zero_marker_report(self):
        self.assertEqual(self.validate()["status"], "PASS")

    def test_rejects_wrong_input_provenance(self):
        self.files["gds"].write_text("mutated fixture")
        with self.assertRaisesRegex(Rejected, "provenance mismatch"):
            self.validate()

    def test_rejects_incomplete_execution(self):
        for change in ({"status": "STARTED"}, {"exit_code": 1}):
            with self.subTest(change=change):
                original = copy.deepcopy(self.metadata)
                self.metadata.update(change)
                self.refresh()
                with self.assertRaises(Rejected):
                    self.validate()
                self.metadata = original
        self.files["execution_log"].write_text("Checker started\n")
        self.refresh()
        with self.assertRaisesRegex(Rejected, "Incomplete checker log"):
            self.validate()

    def test_rejects_wrong_checker_command(self):
        self.execution["command"][1] = "in_gds=unrelated.fixture"
        self.refresh()
        with self.assertRaisesRegex(Rejected, "command/artifact mismatch"):
            self.validate()

    def test_rejects_incomplete_zero_marker_report(self):
        for xml in (self.xml.replace("<items></items>", ""),
                    self.xml.replace("<items></items>", "<items><unexpected/></items>"),
                    self.xml.replace("<name>ANTENNA</name>", "<name>OTHER</name>")):
            with self.subTest(xml=xml):
                self.files["report"].write_text(xml)
                self.refresh()
                with self.assertRaises(Rejected):
                    self.validate()

    def test_rejects_antenna_marker_without_net_context(self):
        item = ('<item><category>ANTENNA</category><cell>mac_accel_dma_rob_top</cell>'
                '<multiplicity>1</multiplicity><values>'
                '<value>polygon: (1,1;2,1;2,2)</value></values></item>')
        self.files["report"].write_text(self.xml.replace("<items></items>",
                                                        "<items>" + item + "</items>"))
        self.metadata.update(marker_count=1, antenna_count=1)
        self.refresh()
        with self.assertRaisesRegex(Rejected, "lacks verified net/pin correlation"):
            self.validate()
