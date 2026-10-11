"""Ensure legacy zero-exit simulations cannot silently pass CI."""
import unittest
from scripts.run_ci_rtl import check_scoreboard


class ScoreboardTests(unittest.TestCase):
    def test_complete_scoreboard(self):
        check_scoreboard("  PASS\n  TOTAL: 7 PASS / 0 FAIL\n", 7)

    def test_rejects_incomplete_failed_or_timed_out_run(self):
        for log in ("PASS", "TOTAL: 6 PASS / 0 FAIL", "TOTAL: 6 PASS / 1 FAIL",
                    "TOTAL: 7 PASS / 0 FAIL\n[TB] Global timeout",
                    "FAIL: payload\nTOTAL: 7 PASS / 0 FAIL",
                    "FATAL: check\nTOTAL: 7 PASS / 0 FAIL",
                    "TOTAL: 7 PASS / 0 FAIL\nTOTAL: 7 PASS / 0 FAIL"):
            with self.subTest(log=log), self.assertRaises(RuntimeError):
                check_scoreboard(log, 7)
