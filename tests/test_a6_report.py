"""Offline pilot denominators, retained activations and missing-evidence rules."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/summarize_a6_pilot.py'
ABSENT = object()


class ReportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = None
        if SCRIPT.exists():
            spec = importlib.util.spec_from_file_location('a6_report_under_test', SCRIPT)
            cls.report = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.report)

    def setUp(self):
        self.assertIsNotNone(self.report, 'offline A6 pilot summarizer must exist')
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.results = self.root / 'results'
        self.results.mkdir()
        self.config = json.loads((ROOT / 'configs/a6_pilot.json').read_text())
        self.config_path = self.root / 'config.json'
        self.config_path.write_text(json.dumps(self.config))

    def activation(self, slot, success=ABSENT, status='VALID_TRIAL', suffix='01',
                   metrics=ABSENT, physical=ABSENT, completed=True, **extra):
        spec = next(s for s in self.config['slots'] if s['slot'] == slot)
        scene = next(s for s in self.config['scenes'] if s['id'] == spec['scene'])
        directory = self.results / ('slot-%02d-%s-%s' % (slot, spec['method'], suffix))
        directory.mkdir()
        record = dict(spec, seed=scene['seed'], kind='PILOT_ATTEMPT', status=status,
                      activation_wall=10., task_started=status == 'VALID_TRIAL')
        if completed:
            record['finish_wall'] = 20.
        if success is not ABSENT:
            record['retrieval_success'] = success
        record.update(extra)
        (directory / 'attempt.json').write_text(json.dumps(record))
        if metrics is not ABSENT:
            (directory / 'data').mkdir()
            (directory / 'data/metrics.json').write_text(json.dumps(metrics))
        if physical is not ABSENT:
            (directory / 'physical_summary.json').write_text(json.dumps(physical))
        return directory

    def summarize(self):
        return self.report.summarize_pilot(self.config, self.results)

    @staticmethod
    def slot(report, number):
        return next(row for row in report['slots'] if row['slot'] == number)

    def test_three_seed_primary_pairs_include_both_directions_of_discordance(self):
        for slot, success in [(2, True), (4, True), (5, True), (7, False),
                              (12, False), (10, True)]:
            self.activation(slot, success)
        report = self.summarize()
        primary = report['primary_comparison']
        self.assertEqual(primary['methods'], ['ours', 'generic'])
        self.assertEqual(primary['n_complete_pairs'], 3)
        self.assertEqual(primary['both_success'], 1)
        self.assertEqual(primary['neither_success'], 0)
        self.assertEqual(primary['ours_only_b'], 1)
        self.assertEqual(primary['generic_only_c'], 1)
        self.assertEqual(primary['paired_risk_difference'], 0.)
        self.assertEqual([p['seed'] for p in primary['pairs']],
                         [s['seed'] for s in self.config['scenes']])
        self.assertEqual(primary['incomplete_pairs'], [])
        self.assertTrue(report['descriptive_only'])
        self.assertNotIn('p_value', primary)

    def test_neither_success_and_nonzero_paired_risk_difference(self):
        for slot, success in [(2, False), (4, False), (5, True), (7, False)]:
            self.activation(slot, success)
        primary = self.summarize()['primary_comparison']
        self.assertEqual(primary['neither_success'], 1)
        self.assertEqual(primary['paired_risk_difference'], .5)
        self.assertEqual(len(primary['incomplete_pairs']), 1)

    def test_invalid_then_valid_keeps_both_records_and_directory_order_links(self):
        valid = self.activation(2, True, suffix='02', activation_wall=1.)
        invalid = self.activation(2, None, status='INVALID_TRIAL', suffix='01',
                                  reason='independent startup timeout', activation_wall=99.)
        self.activation(4, True)
        report = self.summarize()
        row = self.slot(report, 2)
        self.assertEqual(row['selected_attempt'], valid.name)
        self.assertTrue(row['retrieval_success'])
        self.assertEqual([a['attempt_dir'] for a in row['attempts']], [invalid.name, valid.name])
        self.assertIsNone(row['attempts'][0]['rerun_of'])
        self.assertEqual(row['attempts'][1]['rerun_of'], invalid.name)
        self.assertEqual(row['attempts'][0]['raw_attempt']['reason'], 'independent startup timeout')
        self.assertEqual(report['counts']['total_activations'], 3)
        self.assertEqual(report['counts']['invalid_activations'], 1)
        self.assertEqual(report['counts']['valid_completed_slots'], 2)
        sensitivity = report['first_activation_invalid_as_failure']
        self.assertEqual(sensitivity['generic_only_c'], 1)
        self.assertEqual(report['primary_comparison']['both_success'], 1)

    def test_first_invalid_remains_in_sensitivity_while_its_rerun_is_running(self):
        first = self.activation(2, None, status='INVALID_TRIAL', suffix='01',
                                reason='independent startup timeout')
        self.activation(2, None, status='INVALID_TRIAL', suffix='02', completed=False)
        self.activation(4, True)
        report = self.summarize()
        self.assertEqual(self.slot(report, 2)['status'], 'RUNNING')
        self.assertEqual(report['primary_comparison']['n_complete_pairs'], 0)
        sensitivity = report['first_activation_invalid_as_failure']
        self.assertEqual(sensitivity['n_complete_pairs'], 1)
        self.assertEqual(sensitivity['generic_only_c'], 1)
        self.assertEqual(sensitivity['paired_risk_difference'], -1.)
        self.assertEqual(sensitivity['pairs'][0]['ours']['attempt_dir'], first.name)
        self.assertEqual(sensitivity['pairs'][0]['ours']['status'], 'INVALID_TRIAL')

    def test_established_valid_failure_survives_missing_physical_and_lift_status(self):
        self.activation(2, False, reason='task wall guard', classification_reason='no handoff',
                        metrics=dict(terminal_status='LIFT', terminal_failure_stage='active',
                                     terminal_failure_reason='no confirmed candidate'))
        row = self.slot(self.summarize(), 2)
        self.assertEqual(row['status'], 'VALID_TRIAL')
        self.assertFalse(row['retrieval_success'])
        self.assertEqual(row['failure_stage'], 'active')
        self.assertEqual(row['failure_reason'], 'no confirmed candidate')
        self.assertEqual(row['attempts'][0]['raw_attempt']['reason'], 'task wall guard')

    def test_not_run_and_unfinished_initial_invalid_are_not_failures(self):
        self.activation(2, None, status='INVALID_TRIAL', completed=False)
        self.activation(4, True, completed=False)
        report = self.summarize()
        self.assertEqual([r['slot'] for r in report['slots']], list(range(1, 15)))
        self.assertEqual(self.slot(report, 1)['status'], 'NOT_RUN')
        for number in (2, 4):
            self.assertEqual(self.slot(report, number)['status'], 'RUNNING')
            self.assertIsNone(self.slot(report, number)['retrieval_success'])
        self.assertEqual(report['counts']['running_activations'], 2)
        self.assertEqual(report['counts']['invalid_activations'], 0)
        self.assertEqual(report['counts']['not_run_slots'], 12)
        self.assertEqual(report['primary_comparison']['n_complete_pairs'], 0)
        self.assertIsNone(report['primary_comparison']['paired_risk_difference'])
        self.assertEqual(len(report['primary_comparison']['incomplete_pairs']), 3)
        self.assertEqual(report['first_activation_invalid_as_failure']['n_complete_pairs'], 0)

    def test_duplicate_valid_is_ambiguous_without_best_or_latest_winner(self):
        first = self.activation(2, False, suffix='01')
        second = self.activation(2, True, suffix='02')
        self.activation(4, True)
        report = self.summarize()
        row = self.slot(report, 2)
        self.assertEqual(row['status'], 'AMBIGUOUS_DUPLICATE_VALID')
        self.assertEqual(row['duplicate_valid_attempts'], [first.name, second.name])
        self.assertIsNone(row['selected_attempt'])
        self.assertIsNone(row['retrieval_success'])
        self.assertEqual(report['counts']['valid_completed_activations'], 3)
        self.assertEqual(report['counts']['valid_completed_slots'], 1)
        self.assertEqual(report['primary_comparison']['n_complete_pairs'], 0)
        self.assertEqual(report['first_activation_invalid_as_failure']['n_complete_pairs'], 0)

    def test_missing_metrics_does_not_remove_known_primary_and_unknowns_stay_null(self):
        self.activation(2, True, physical=dict(status='PASS', target_lift_m=.15))
        self.activation(4, False, classification_reason='navigation failed')
        report = self.summarize()
        self.assertEqual(report['primary_comparison']['ours_only_b'], 1)
        row = self.slot(report, 2)
        for name in ('D_exec', 'D_env', 'C_env_count_final', 'C_env_count_max',
                     'counts', 'paths', 'T_first_env_sim', 'T_task_sim'):
            self.assertIsNone(row[name], name)
        self.assertEqual(row['physical_status'], 'PASS')
        self.assertEqual(row['attempts'][0]['physical_summary']['target_lift_m'], .15)
        self.assertIn('data/metrics.json', row['attempts'][0]['missing_files'])
        self.assertEqual(self.slot(report, 4)['failure_reason'], 'navigation failed')

    def test_recorded_counters_times_stage_and_incomplete_distance_are_preserved(self):
        metrics = dict(D_env=True, D_exec=False, C_env_count_final=2, C_env_count_max=4,
                       first_env_sim=18., T_first_env_sim=8., T_task_sim=37., T_active_sim=12.,
                       T_exec_ready_sim=None, clock_reset_detected=False,
                       counts=dict(capture_calls=3, discarded_windows=1, nbv_moves=0),
                       paths=dict(uav_total=dict(complete=False, distance_m=None,
                                                observed_distance_lower_bound_m=2.5, gap_count=1)),
                       stages=dict(ground_refine=dict(status='NOT_REACHED')))
        self.activation(2, False, metrics=metrics, physical=dict(status='FAIL', error='short lift'))
        row = self.slot(self.summarize(), 2)
        for name, value in metrics.items():
            self.assertEqual(row[name], value, name)
        self.assertEqual(row['failure_reason'], 'short lift')
        self.assertEqual(row['attempts'][0]['metrics'], metrics)

    def test_rm4d_environment_not_applicable_and_hard_ablations_are_secondary(self):
        self.activation(3, False, metrics=dict(D_env=False, D_exec=False))
        for number in (13, 14):
            self.activation(number, True)
        report = self.summarize()
        rm4d = self.slot(report, 3)
        self.assertIsNone(rm4d['D_env'])
        self.assertFalse(rm4d['D_env_applicable'])
        self.assertFalse(rm4d['attempts'][0]['metrics']['D_env'])
        self.assertEqual(self.slot(report, 13)['comparison_role'], 'secondary_ablation')
        self.assertEqual(self.slot(report, 14)['comparison_role'], 'secondary_ablation')
        self.assertEqual(report['primary_comparison']['n_complete_pairs'], 0)

    def test_setup_kind_and_unscheduled_or_mismatched_records_are_excluded(self):
        self.activation(2, True, kind='METHOD_INDEPENDENT_SETUP')
        self.activation(4, True, method='ours')
        self.activation(5, True, seed=-1)
        report = self.summarize()
        self.assertEqual(report['counts']['total_activations'], 0)
        self.assertEqual(report['counts']['not_run_slots'], 14)
        self.assertEqual(len(report['excluded_attempts']), 3)
        self.assertEqual(report['primary_comparison']['n_complete_pairs'], 0)

    def test_missing_or_nonbinary_primary_never_inferred_from_checker_or_lift(self):
        self.activation(2, metrics=dict(terminal_status='LIFT'), physical=dict(status='PASS'))
        self.activation(4, 1)
        report = self.summarize()
        for number in (2, 4):
            self.assertEqual(self.slot(report, number)['status'], 'VALID_OUTCOME_MISSING')
            self.assertIsNone(self.slot(report, number)['retrieval_success'])
        self.assertEqual(report['primary_comparison']['n_complete_pairs'], 0)

    def test_malformed_metrics_preserves_binary_and_records_read_error(self):
        directory = self.activation(2, False, metrics={})
        (directory / 'data/metrics.json').write_text('{"D_exec":')
        report = self.summarize()
        row = self.slot(report, 2)
        self.assertFalse(row['retrieval_success'])
        self.assertIsNone(row['D_exec'])
        self.assertIn('data/metrics.json', row['attempts'][0]['read_errors'][0])

    def test_cli_outputs_deterministic_json_and_missing_results_is_all_not_run(self):
        output = self.root / 'report.json'
        command = [sys.executable, str(SCRIPT), '--config', str(self.config_path),
                   '--results-dir', str(self.root / 'not-yet-created'), '--output', str(output)]
        subprocess.run(command, check=True, capture_output=True, text=True)
        first = output.read_text()
        subprocess.run(command, check=True, capture_output=True, text=True)
        self.assertEqual(output.read_text(), first)
        report = json.loads(first)
        self.assertEqual(report['counts']['scheduled_slots'], 14)
        self.assertEqual(report['counts']['total_activations'], 0)


if __name__ == '__main__':
    unittest.main()
