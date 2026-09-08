"""Literal offline projection of collected slots, including missing evidence."""

import copy
import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/a6_result_tables.py'


class ResultTablesTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SCRIPT.is_file(), 'offline result projection must exist')
        spec = importlib.util.spec_from_file_location('a6_result_tables', SCRIPT)
        self.module = importlib.util.module_from_spec(spec)
        with patch.object(sys, 'path', [str(ROOT / 'scripts'), *sys.path]):
            spec.loader.exec_module(self.module)
        self.config = dict(scenes=[dict(id='easy-001', tier='easy', seed=11),
                                   dict(id='easy-002', tier='easy', seed=12),
                                   dict(id='hard', seed=13)],
                           slots=[dict(slot=1, scene='easy-001', method='ours'),
                                  dict(slot=2, scene='easy-002', method='ours'),
                                  dict(slot=3, scene='hard', method='generic'),
                                  dict(slot=4, scene='easy-001', method='rm4d_only')])
        rows = []
        for slot, seed in zip(self.config['slots'], (11, 12, 13, 11)):
            rows.append(dict(slot, seed=seed, status='NOT_RUN', retrieval_success=None,
                             selected_attempt=None, attempts=[]))
        decisions = [dict(round=1, confirmed_candidate_count=0),
                     dict(round=2, confirmed_candidate_count=3),
                     dict(round=3, confirmed_candidate_count=2)]
        rows[0].update(status='VALID_TRIAL', retrieval_success=False, D_env=True, D_exec=True,
                       C_env_count_final=2, C_env_count_max=3, selected_attempt='slot1-02',
                       T_first_env_sim=6., T_active_sim=15., T_task_sim=200., T_exec_ready_sim=20.,
                       counts=dict(completed_windows=3, voted_windows=3, nbv_moves=2, sensing_rescans=0),
                       paths=dict(uav_active=dict(distance_m=None, complete=False, missing_samples=2,
                                                  gap_count=1, observed_distance_lower_bound_m=4.25),
                                  uav_total=dict(distance_m=5.2, complete=True, missing_samples=0,
                                                 gap_count=0, observed_distance_lower_bound_m=5.2)),
                       stages=dict(active=dict(status='SUCCEEDED', duration_sim_s=15.),
                                   ground_navigation=dict(status='FAILED', duration_sim_s=120.),
                                   ground_refine=dict(status='NOT_REACHED', duration_sim_s=None)),
                       metrics_terminal_status='FAILED', failure_stage='ground_navigation',
                       failure_reason='navigation timeout', attempts=[
                           dict(attempt_dir='slot1-01', effective_status='INVALID_TRIAL',
                                metrics=dict(environment_decisions=[dict(round=1, confirmed_candidate_count=99)])),
                           dict(attempt_dir='slot1-02', effective_status='VALID_TRIAL',
                                metrics=dict(environment_decisions=decisions))])
        rows[1].update(status='VALID_TRIAL', retrieval_success=True, D_env=True, D_exec=True,
                       selected_attempt='slot2-01', T_task_sim=40., T_active_sim=5.,
                       paths=dict(uav_active=dict(distance_m=2., complete=True, missing_samples=0,
                                                  gap_count=0, observed_distance_lower_bound_m=2.)),
                       stages=dict(lift=dict(status='SUCCEEDED', duration_sim_s=1.),
                                   retention=dict(status='NOT_REACHED', duration_sim_s=None)),
                       attempts=[dict(attempt_dir='slot2-01', effective_status='VALID_TRIAL', metrics=None)])
        rows[3].update(status='VALID_TRIAL', retrieval_success=False, D_env=None, D_exec=True,
                       selected_attempt='slot4-01', counts=dict(completed_windows=0, nbv_moves=0),
                       T_active_sim=0., attempts=[dict(attempt_dir='slot4-01', effective_status='VALID_TRIAL',
                                                       metrics=dict(environment_decisions=[]))])
        self.summary = dict(slots=rows)

    def describe(self):
        return self.module.describe_results(self.config, self.summary)

    def test_fixed_slot_order_literal_failure_and_inputs_are_unchanged(self):
        self.summary['slots'].reverse()
        before = copy.deepcopy((self.config, self.summary))
        report = self.describe()
        self.assertEqual([r['slot'] for r in report['rows']], [1, 2, 3, 4])
        row = report['rows'][0]
        self.assertEqual(row['status'], 'VALID_TRIAL')
        self.assertIs(row['retrieval_success'], False)
        self.assertEqual(row['selected_attempt'], 'slot1-02')
        self.assertEqual(row['invalid_activation_count'], 1)
        self.assertEqual(row['failure_stage'], 'ground_navigation')
        self.assertEqual(row['failure_reason'], 'navigation timeout')
        self.assertEqual((self.config, self.summary), before)
        self.assertTrue(report['descriptive_only'])
        self.assertFalse(any(isinstance(value, (dict, list)) for value in row.values()))
        self.assertNotIn('p_value', report)

    def test_discovery_uses_only_selected_attempt_and_recorded_decision_order(self):
        row = self.describe()['rows'][0]
        self.assertEqual(row['first_discovery_window'], 2)
        self.assertEqual(row['C_env_count_final'], 2)
        self.assertEqual(row['C_env_count_max'], 3)
        self.summary['slots'][0]['selected_attempt'] = None
        self.summary['slots'][0]['status'] = 'AMBIGUOUS_DUPLICATE_VALID'
        row = self.describe()['rows'][0]
        self.assertIsNone(row['first_discovery_window'])
        self.assertEqual(row['status'], 'AMBIGUOUS_DUPLICATE_VALID')

    def test_missing_paths_remain_null_and_observed_bounds_are_separate(self):
        row = self.describe()['rows'][0]
        self.assertIsNone(row['uav_active_distance_m'])
        self.assertIs(row['uav_active_complete'], False)
        self.assertEqual(row['uav_active_missing_samples'], 2)
        self.assertEqual(row['uav_active_gap_count'], 1)
        self.assertEqual(row['uav_active_observed_distance_lower_bound_m'], 4.25)
        self.assertEqual(row['uav_total_distance_m'], 5.2)
        for suffix in ('distance_m', 'complete', 'missing_samples', 'observed_distance_lower_bound_m'):
            self.assertIsNone(row['ground_total_' + suffix])

    def test_unreached_failed_and_missing_stages_remain_distinct(self):
        rows = self.describe()['rows']
        self.assertEqual(rows[0]['stage_ground_navigation_status'], 'FAILED')
        self.assertEqual(rows[0]['stage_ground_navigation_duration_sim_s'], 120.)
        self.assertEqual(rows[0]['stage_ground_refine_status'], 'NOT_REACHED')
        self.assertIsNone(rows[0]['stage_ground_refine_duration_sim_s'])
        self.assertIsNone(rows[2]['stage_ground_navigation_status'])
        # Physical success does not rewrite the reducer's retention status.
        self.assertIs(rows[1]['retrieval_success'], True)
        self.assertEqual(rows[1]['stage_retention_status'], 'NOT_REACHED')

    def test_rm4d_na_and_recorded_zero_are_not_missing_or_false_discovery(self):
        rows = self.describe()['rows']
        rm4d = rows[3]
        self.assertIs(rm4d['D_env_applicable'], False)
        self.assertIsNone(rm4d['D_env'])
        self.assertIsNone(rm4d['first_discovery_window'])
        self.assertEqual(rm4d['completed_windows'], 0)
        self.assertEqual(rm4d['T_active_sim'], 0.)
        self.assertIsNone(rows[2]['completed_windows'])
        self.assertIsNone(rows[2]['retrieval_success'])
        self.assertEqual(rows[2]['tier'], 'hard')

    def test_group_denominators_include_valid_failures_without_success_filtering(self):
        groups = {(g['tier'], g['method']): g for g in self.describe()['groups']}
        ours = groups['easy', 'ours']
        self.assertEqual(ours['scheduled_slots'], 2)
        self.assertEqual(ours['endpoints']['retrieval_success'], dict(numerator=1, denominator=2,
                                                                    not_applicable=0))
        self.assertEqual(ours['available_counts']['T_task_sim'], 2)
        self.assertEqual(ours['available_counts']['uav_active_distance_m'], 1)
        self.assertEqual(ours['available_counts']['uav_active_observed_distance_lower_bound_m'], 2)
        self.assertEqual(groups['hard', 'generic']['endpoints']['retrieval_success']['denominator'], 0)
        self.assertEqual(groups['easy', 'rm4d_only']['endpoints']['D_env'],
                         dict(numerator=0, denominator=0, not_applicable=1))

    def test_absent_summary_slot_and_attempt_list_do_not_become_zero_counts(self):
        self.summary['slots'].pop(2)
        self.summary['slots'][0].pop('attempts')
        rows = self.describe()['rows']
        self.assertIsNone(rows[0]['invalid_activation_count'])
        self.assertIsNone(rows[2]['invalid_activation_count'])
        self.assertIsNone(rows[2]['status'])
        self.assertIsNone(rows[2]['T_task_sim'])

    def test_wrong_config_join_is_rejected_without_relabeling(self):
        self.summary['slots'][0]['seed'] = 999
        with self.assertRaises(ValueError):
            self.describe()

    def test_unknown_activation_status_is_not_counted_as_known_noninvalid(self):
        self.summary['slots'][0]['attempts'][0].pop('effective_status')
        self.assertIsNone(self.describe()['rows'][0]['invalid_activation_count'])

    def test_existing_collector_fixture_projects_selected_attempt_after_invalid(self):
        from test_a6_report import ReportTests
        ReportTests.setUpClass()
        fixture = ReportTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        fixture.activation(2, None, status='INVALID_TRIAL', suffix='01')
        fixture.activation(2, False, suffix='02', metrics=dict(
            D_env=True, D_exec=False, terminal_status='FAILED', terminal_failure_stage='active',
            terminal_failure_reason='no executable selection', C_env_count_final=2,
            C_env_count_max=2, environment_decisions=[dict(round=3, confirmed_candidate_count=2)]))
        report = self.module.describe_results(fixture.config, fixture.summarize())
        row = report['rows'][1]
        self.assertEqual(row['invalid_activation_count'], 1)
        self.assertEqual(row['first_discovery_window'], 3)
        self.assertIs(row['retrieval_success'], False)
        self.assertIs(row['D_exec'], False)
        self.assertEqual(len(report['rows']), 14)

    def test_cli_writes_explicit_json_csv_and_never_changes_inputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config, summary = root / 'config.json', root / 'summary.json'
            output_json, output_csv = root / 'table.json', root / 'table.csv'
            config.write_text(json.dumps(self.config))
            summary.write_text(json.dumps(self.summary))
            before = (config.read_bytes(), summary.read_bytes())
            result = subprocess.run([sys.executable, str(SCRIPT), '--config', str(config),
                                     '--summary', str(summary), '--output-json', str(output_json),
                                     '--output-csv', str(output_csv)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(output_json.read_text()), self.describe())
            with output_csv.open(newline='') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 4)
            self.assertEqual(rows[0]['retrieval_success'], 'False')
            self.assertEqual(rows[0]['uav_active_distance_m'], '')
            self.assertEqual(rows[0]['uav_active_observed_distance_lower_bound_m'], '4.25')
            self.assertEqual((config.read_bytes(), summary.read_bytes()), before)


if __name__ == '__main__':
    unittest.main()
