"""Ensure legacy zero-exit simulations cannot silently pass CI."""
import unittest
import subprocess
from unittest.mock import patch
from scripts.run_ci_rtl import check_scoreboard, check_power_scoreboard, check_sweep_scoreboard, run


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


class PowerScoreboardTests(unittest.TestCase):
    def test_exact_legacy_power_format(self):
        log = "POWER_JOB_PASS first\nPOWER_JOB_PASS second\nPOWER_TOTAL 2 PASS / 0 FAIL\nPOWER_REGRESSION_PASS\n"
        check_power_scoreboard(log, 2)

    def test_power_failure_markers_and_incomplete_run(self):
        good = "POWER_JOB_PASS first\nPOWER_JOB_PASS second\nPOWER_TOTAL 2 PASS / 0 FAIL\nPOWER_REGRESSION_PASS\n"
        for bad in [good.replace('2 PASS / 0 FAIL', '1 PASS / 1 FAIL'),
                    good.replace('POWER_REGRESSION_PASS', ''),
                    good.replace('POWER_JOB_PASS second', ''),
                    good + 'TOTAL: 2 PASS / 0 FAIL\n',
                    good + 'FAIL: payload\n', good + 'global timeout\n']:
            with self.subTest(log=bad), self.assertRaises(RuntimeError):
                check_power_scoreboard(bad, 2)


class ProcessFailureTests(unittest.TestCase):
    def test_process_timeout_is_not_accepted(self):
        with patch('scripts.run_ci_rtl.subprocess.run', side_effect=subprocess.TimeoutExpired(['vvp'], 120)):
            with self.assertRaises(subprocess.TimeoutExpired):
                run(['vvp'], '.')

    def test_nonzero_exit_with_passing_text_is_not_accepted(self):
        result = subprocess.CompletedProcess(['vvp'], 1, 'TOTAL: 7 PASS / 0 FAIL\n')
        with patch('scripts.run_ci_rtl.subprocess.run', return_value=result):
            with self.assertRaises(subprocess.CalledProcessError):
                run(['vvp'], '.')


class SweepScoreboardTests(unittest.TestCase):
    def test_exact_csv_format(self):
        row = 'SWEEP,8,16,100,123,16,32,1,2,PASS\n'
        check_sweep_scoreboard(row * 2 + 'SWEEP_TOTAL,2,0\n', 2)

    def test_incomplete_failed_and_timeout_sweeps(self):
        row = 'SWEEP,8,16,100,123,16,32,1,2,PASS\n'
        good = row * 2 + 'SWEEP_TOTAL,2,0\n'
        for bad in [row + 'SWEEP_TOTAL,2,0\n', row * 2,
                    good.replace('2,0', '1,1'), good.replace(',PASS', ',FAIL'),
                    good + 'SWEEP_TOTAL,2,0\n', good + 'TOTAL: 2 PASS / 0 FAIL\n',
                    good + 'FATAL: stalled\n', good + 'timeout\n']:
            with self.subTest(log=bad), self.assertRaises(RuntimeError):
                check_sweep_scoreboard(bad, 2)
