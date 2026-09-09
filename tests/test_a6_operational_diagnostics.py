import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import zipfile

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/a6_operational_diagnostics.py'
module = None
if SCRIPT.is_file():
    spec = importlib.util.spec_from_file_location('operational_diagnostics', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)


def inputs():
    candidates = []
    for name in ('alias', 'collision', 'ambiguous', 'representative', 'environment'):
        gate = dict(blocked=False, target_collision=False, target_cells=1,
                    environment_cells=0, ambiguous_cells=0,
                    ground_supported=True, ground_supported_cells=8, ground_missing_cells=0)
        candidates.append(dict(candidate_id=name, x=1., y=2., yaw=.3, source_id=5,
                               occupied_cells=1, representative_blocked=False,
                               footprint_clipped=False, confirmed=False, operational=gate))
    candidates[0]['confirmed'] = True
    candidates[1]['operational'].update(blocked=True, target_collision=True)
    candidates[2]['operational'].update(blocked=True, ambiguous_cells=1)
    candidates[3].update(representative_blocked=True, footprint_clipped=True)
    candidates[3]['operational'].update(ground_supported=False, ground_missing_cells=1)
    candidates[4]['operational'].update(blocked=True, environment_cells=1)
    decision = dict(round=1, policy_method='ours', candidate_count=5,
                    confirmed_candidate_count=1, assessments=candidates,
                    operational_semantics='object-aware-v1.1')
    summary = dict(association_status='AVAILABLE', operational_semantics='object-aware-v1.1',
                   observation_windows=1, target=dict(geometry_allowance_m=.04))
    arrays = dict(target_occupied_votes=np.array([[2, 0], [1, 0]]),
                  environment_occupied_votes=np.array([[3, 0], [0, 0]]),
                  ambiguous_occupied_votes=np.array([[1, 4], [0, 0]]),
                  ground_votes=np.array([[3, 2], [1, 0]]))
    return decision, summary, arrays


class OperationalDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(module, 'read-only operational diagnostic reporter is not implemented')

    def test_alias_collision_ambiguous_and_representative_are_distinct(self):
        result = module.describe_round(*inputs())
        self.assertEqual(result['raw_grid_blocked_exact']['count'], 5)
        self.assertEqual(result['objectaware_blocked_exact']['ids'],
                         ['collision', 'ambiguous', 'environment'])
        self.assertEqual(result['blocked_causes']['target_collision']['ids'], ['collision'])
        self.assertEqual(result['blocked_causes']['ambiguous']['ids'], ['ambiguous'])
        self.assertEqual(result['blocked_causes']['environment']['ids'], ['environment'])
        self.assertEqual(result['target_alias_retained_exact']['ids'], ['alias', 'representative'])
        self.assertEqual(result['representative_only_blocked']['ids'], ['representative'])
        self.assertEqual(result['combined_blocked']['count'], 4)
        self.assertEqual(result['confirmed']['ids'], ['alias'])
        retained = result['candidates'][3]
        self.assertEqual(retained['pose_xyyaw'], [1., 2., .3])
        self.assertTrue(retained['target_alias_retained_exact'])
        self.assertTrue(retained['footprint_clipped'])
        self.assertFalse(retained['ground_supported'])
        self.assertEqual(retained['ground_missing_cells'], 1)
        self.assertFalse(retained['confirmed'])
        self.assertIsNone(result['representative_raw_grid_blocked'])

    def test_mixed_class_votes_and_cells_are_not_exclusive(self):
        result = module.describe_round(*inputs())
        self.assertEqual(result['occupied_classes'], dict(
            TARGET=dict(vote_sum=3, occupied_cell_count=2),
            ENVIRONMENT=dict(vote_sum=3, occupied_cell_count=1),
            AMBIGUOUS=dict(vote_sum=5, occupied_cell_count=2)))
        self.assertEqual(result['ground_votes_sum'], 6)
        self.assertFalse(result['occupied_class_cell_counts_exclusive'])

    def test_subcell_blockers_use_saved_intersections_and_fallback_not_coarse_cells(self):
        for coarse, intersections, fallback, blocked in (
                (3, [], [], False), (0, [2], [], True), (0, [], [17], True)):
            with self.subTest(coarse=coarse, intersections=intersections, fallback=fallback):
                decision, summary, arrays = inputs()
                candidate = decision['assessments'][2]
                candidate['operational']['ambiguous_cells'] = coarse
                candidate['ambiguous_subcell'] = dict(
                    intersecting_endpoint_indices=intersections,
                    legacy_fallback_cell_ids=fallback)
                report = module.describe_round(decision, summary, arrays)
                self.assertIs(report['candidates'][2]['ambiguous_blocked'], blocked)
                self.assertEqual(report['blocked_causes']['ambiguous']['ids'],
                                 ['ambiguous'] if blocked else [])
                self.assertEqual(report['candidates'][2]['ambiguous_cells'], coarse)

    def test_v13_missing_sidecar_is_unknown_even_with_zero_coarse_cells(self):
        for source in ('decision', 'summary'):
            for coarse in (0, 1):
                with self.subTest(source=source, coarse=coarse):
                    decision, summary, arrays = inputs()
                    decision.pop('operational_semantics')
                    summary.pop('operational_semantics')
                    (decision if source == 'decision' else summary)[
                        'operational_semantics'] = 'object-aware-v1.3'
                    decision['assessments'][2]['operational']['ambiguous_cells'] = coarse
                    report = module.describe_round(decision, summary, arrays)
                    self.assertIsNone(report['candidates'][2]['ambiguous_blocked'])
                    self.assertIsNone(report['blocked_causes']['ambiguous']['count'])
                    self.assertIn('ambiguous', report['blocked_causes']['ambiguous']['unavailable_ids'])
                    self.assertIn('candidate:ambiguous:ambiguous_blocked', report['unavailable'])

    def test_incomplete_subcell_evidence_cannot_establish_clearance(self):
        for sidecar in (None, {}, dict(intersecting_endpoint_indices=[]),
                        dict(legacy_fallback_cell_ids=[]),
                        dict(intersecting_endpoint_indices=None, legacy_fallback_cell_ids=[])):
            with self.subTest(sidecar=sidecar):
                decision, summary, arrays = inputs()
                decision['assessments'][2]['ambiguous_subcell'] = sidecar
                report = module.describe_round(decision, summary, arrays)
                self.assertIsNone(report['candidates'][2]['ambiguous_blocked'])
                self.assertEqual(report['blocked_causes']['ambiguous']['unavailable_ids'], ['ambiguous'])

    def test_empty_summary_semantics_defer_to_recorded_v13_decision(self):
        for summary_semantics in (None, ''):
            with self.subTest(summary_semantics=summary_semantics):
                decision, summary, arrays = inputs()
                decision['operational_semantics'] = 'object-aware-v1.3'
                summary['operational_semantics'] = summary_semantics
                report = module.describe_round(decision, summary, arrays)
                self.assertIsNone(report['candidates'][0]['ambiguous_blocked'])
                self.assertEqual(report['operational_semantics'], 'object-aware-v1.3')
                self.assertIsNone(report['blocked_causes']['ambiguous']['count'])

    def test_v14_missing_ambiguity_sidecar_remains_unknown(self):
        decision, summary, arrays = inputs()
        decision['operational_semantics'] = summary['operational_semantics'] = 'object-aware-v1.4'
        report = module.describe_round(decision, summary, arrays)
        self.assertIsNone(report['candidates'][0]['ambiguous_blocked'])
        self.assertIsNone(report['blocked_causes']['ambiguous']['count'])

    def test_recorded_subcell_hit_establishes_blocking_with_other_list_missing(self):
        for sidecar in (dict(intersecting_endpoint_indices=[0]),
                        dict(legacy_fallback_cell_ids=[17])):
            with self.subTest(sidecar=sidecar):
                decision, summary, arrays = inputs()
                candidate = decision['assessments'][0]
                candidate['ambiguous_subcell'] = sidecar
                report = module.describe_round(decision, summary, arrays)
                self.assertTrue(report['candidates'][0]['ambiguous_blocked'])

    def test_missing_reference_is_explicit_and_does_not_infer_target_alias(self):
        decision, summary, arrays = inputs()
        summary['association_status'] = 'UNAVAILABLE'
        report = module.describe_round(decision, summary, arrays)
        self.assertEqual(report['association_status'], 'UNAVAILABLE')
        self.assertIn('target_reference', report['unavailable'])
        self.assertIsNone(report['target_alias_retained_exact']['count'])

    def test_missing_ground_or_confirmation_is_not_recreated_from_free_cells(self):
        decision, summary, arrays = inputs()
        candidate = decision['assessments'][0]
        candidate.pop('confirmed')
        candidate['free_cells'] = 8
        for name in ('ground_supported', 'ground_supported_cells', 'ground_missing_cells'):
            candidate['operational'].pop(name)
        report = module.describe_round(decision, summary, arrays)
        self.assertIsNone(report['candidates'][0]['ground_supported'])
        self.assertIsNone(report['candidates'][0]['confirmed'])
        self.assertIsNone(report['confirmed']['count'])
        self.assertEqual(report['confirmed']['unavailable_ids'], ['alias'])

    def test_missing_records_are_unavailable_not_zero(self):
        result = module.describe_round(None, None, None)
        self.assertEqual(result['status'], 'UNAVAILABLE')
        for name in ('raw_grid_blocked_exact', 'objectaware_blocked_exact',
                     'target_alias_retained_exact', 'confirmed'):
            self.assertIsNone(result[name]['count'])
        self.assertIsNone(result['occupied_classes']['TARGET']['vote_sum'])
        self.assertIsNone(result['ground_votes_sum'])

    def test_missing_assessments_or_ground_evidence_cannot_be_available(self):
        decision, summary, arrays = inputs()
        del decision['assessments']
        del arrays['ground_votes']
        report = module.describe_round(decision, summary, arrays)
        self.assertEqual(report['status'], 'PARTIAL')
        self.assertIn('assessments', report['unavailable'])
        self.assertIn('ground_votes', report['unavailable'])
        self.assertIsNone(report['confirmed']['count'])
        self.assertIsNone(report['ground_votes_sum'])

    def test_missing_candidate_gate_remains_unknown_and_marks_partial_evidence(self):
        decision, summary, arrays = inputs()
        del decision['assessments'][0]['operational']
        report = module.describe_round(decision, summary, arrays)
        self.assertEqual(report['status'], 'PARTIAL')
        self.assertIsNone(report['objectaware_blocked_exact']['count'])
        self.assertEqual(report['objectaware_blocked_exact']['unavailable_ids'], ['alias'])
        self.assertIsNone(report['candidates'][0]['target_alias_retained_exact'])

    def test_alias_requires_both_raw_occupied_and_target_overlap(self):
        for field, value in (('occupied_cells', 0), ('target_cells', 0)):
            decision, summary, arrays = inputs()
            candidate = decision['assessments'][0]
            (candidate if field == 'occupied_cells' else candidate['operational'])[field] = value
            report = module.describe_round(decision, summary, arrays)
            self.assertFalse(report['candidates'][0]['target_alias_retained_exact'])

    def test_absent_vote_array_can_use_recorded_sum_but_cannot_infer_cells(self):
        decision, summary, arrays = inputs()
        del arrays['target_occupied_votes']
        summary['votes'] = dict(target_occupied_votes=19)
        report = module.describe_round(decision, summary, arrays)
        self.assertEqual(report['occupied_classes']['TARGET']['vote_sum'], 19)
        self.assertIsNone(report['occupied_classes']['TARGET']['occupied_cell_count'])

    def test_pure_rm4d_round_reports_not_applicable_even_with_zero_counts_recorded(self):
        report = module.describe_round(dict(policy_method='rm4d_only', assessments=[],
                                            confirmed_candidate_count=0), None, None)
        self.assertEqual(report['status'], 'NOT_APPLICABLE')
        self.assertFalse(report['mechanism_applicable'])
        self.assertIsNone(report['confirmed']['count'])
        self.assertIsNone(report['confirmed_candidate_count_recorded'])

    def test_inputs_are_preserved(self):
        decision, summary, arrays = inputs()
        before = copy.deepcopy((decision, summary, arrays))
        module.describe_round(decision, summary, arrays)
        self.assertEqual((decision, summary), before[:2])
        for key in arrays:
            np.testing.assert_array_equal(arrays[key], before[2][key])

    def test_saved_natural_final_has_two_confirmed_and_no_alias_retained(self):
        directory = ROOT / 'outputs/a6/operational-v11/natural-a5-attempt-01'
        if not (directory / 'decision.json').is_file():
            self.skipTest('local natural-v1.1 regression artifacts are not present')
        decision = json.loads((directory / 'decision.json').read_text())
        summary = json.loads((directory / 'operational_summary.json').read_text())
        with np.load(directory / 'operational_evidence.npz', allow_pickle=False) as arrays:
            result = module.describe_round(decision, summary, arrays)
        self.assertEqual(result['confirmed']['ids'], ['candidate-000008', 'candidate-000009'])
        self.assertEqual(result['confirmed']['count'], 2)
        self.assertEqual(result['target_alias_retained_exact']['count'], 0)
        self.assertEqual(result['raw_grid_blocked_exact']['count'], 34)
        self.assertEqual(result['objectaware_blocked_exact']['count'], 34)
        self.assertEqual(result['representative_only_blocked']['count'], 1)

    def test_saved_v13_natural_coarse_aliases_are_not_ambiguous_blockers(self):
        directory = ROOT / 'outputs/a6/v13-development/natural-attempt-04'
        if not (directory / 'decision.json').is_file():
            self.skipTest('local natural-v1.3 regression artifacts are not present')
        decision = json.loads((directory / 'decision.json').read_text())
        summary = json.loads((directory / 'operational_summary.json').read_text())
        with np.load(directory / 'operational_evidence.npz', allow_pickle=False) as arrays:
            result = module.describe_round(decision, summary, arrays)
        self.assertEqual(result['blocked_causes']['ambiguous']['count'], 27)
        by_id = {row['candidate_id']: row for row in result['candidates']}
        for candidate_id in ('candidate-000000', 'candidate-000012', 'candidate-000033',
                             'candidate-000052', 'candidate-000085'):
            self.assertGreater(by_id[candidate_id]['ambiguous_cells'], 0)
            self.assertFalse(by_id[candidate_id]['ambiguous_blocked'])
        self.assertTrue(by_id['candidate-000033']['operational_blocked'])
        self.assertTrue(by_id['candidate-000033']['target_collision'])
        self.assertEqual(result['confirmed']['ids'], ['candidate-000008'])

    def write_attempt(self, directory, method='ours', rounds=True):
        (directory / 'data').mkdir(parents=True)
        (directory / 'attempt.json').write_text(json.dumps(dict(
            method=method, status='VALID_TRIAL', slot=1, retrieval_success=False)))
        (directory / 'data/metrics.json').write_text(json.dumps(dict(
            method=method, D_env=True, D_exec=False, first_env_sim=42.,
            C_env_count_final=1, environment_decisions=[dict(round=1, confirmed_candidate_count=1)])))
        if rounds:
            path = directory / 'data/rounds/round-01'
            path.mkdir(parents=True)
            decision, summary, arrays = inputs()
            (path / 'decision.json').write_text(json.dumps(decision))
            (path / 'operational_summary.json').write_text(json.dumps(summary))
            np.savez(path / 'operational_evidence.npz', **arrays)

    def test_attempt_join_preserves_metrics_and_retrieval_outcome(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write_attempt(root / 'attempt-01')
            report = module.describe_results(root)
        attempt = report['attempts'][0]
        self.assertEqual(attempt['first_confirmed_window'], 1)
        self.assertEqual(attempt['first_confirmed_window_source'], 'metrics.environment_decisions')
        self.assertEqual(attempt['confirmed_final']['ids'], ['alias'])
        self.assertTrue(attempt['outcomes']['D_env'])
        self.assertFalse(attempt['outcomes']['D_exec'])
        self.assertFalse(attempt['outcomes']['retrieval_success'])
        self.assertEqual(attempt['outcomes']['first_env_sim'], 42.)

    def test_truncated_compressed_npz_is_unavailable_without_aborting_other_attempts(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write_attempt(root / 'attempt-01')
            self.write_attempt(root / 'attempt-02')
            path = root / 'attempt-01/data/rounds/round-01/operational_evidence.npz'
            np.savez_compressed(path, **inputs()[2])
            path.write_bytes(path.read_bytes()[:-22])
            with path.open('rb') as source, self.assertRaises(zipfile.BadZipFile):
                np.load(source, allow_pickle=False)
            try:
                report = module.describe_results(root)
            except zipfile.BadZipFile as error:
                self.fail('a damaged round aborted the report: ' + str(error))
        self.assertEqual(report['attempt_count'], 2)
        damaged = report['attempts'][0]['rounds'][0]
        self.assertEqual(damaged['status'], 'PARTIAL')
        self.assertIn('operational_evidence.npz', damaged['unavailable'])
        self.assertTrue(any('operational_evidence.npz' in error for error in damaged['read_errors']))
        self.assertIsNone(damaged['occupied_classes']['TARGET']['occupied_cell_count'])
        self.assertEqual(report['attempts'][1]['rounds'][0]['status'], 'AVAILABLE')

    def test_missing_expected_round_and_attempt_record_remain_visible(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory = root / 'attempt-01'
            self.write_attempt(directory, rounds=False)
            (directory / 'attempt.json').unlink()
            report = module.describe_results(root)['attempts'][0]
        self.assertIn('attempt.json', report['unavailable'])
        self.assertIsNone(report['outcomes']['retrieval_success'])
        self.assertIsNone(report['confirmed_final']['count'])
        self.assertEqual(report['rounds'][0]['round'], 1)
        self.assertEqual(report['rounds'][0]['status'], 'UNAVAILABLE')

    def test_setup_records_are_excluded_but_invalid_method_attempts_are_retained(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write_attempt(root / 'attempt-01')
            self.write_attempt(root / 'attempt-invalid', rounds=False)
            invalid = root / 'attempt-invalid/attempt.json'
            record = json.loads(invalid.read_text())
            record.update(kind='PILOT_ATTEMPT', status='INVALID_TRIAL', retrieval_success=None)
            invalid.write_text(json.dumps(record))
            for index, metadata in enumerate((
                    dict(kind='METHOD_INDEPENDENT_SETUP'), dict(method='SETUP_CHECK'),
                    dict(kind='METHOD_INDEPENDENT_SETUP', method='SETUP_CHECK'))):
                directory = root / 'setup' / ('setup-%d' % index)
                directory.mkdir(parents=True)
                (directory / 'attempt.json').write_text(json.dumps(metadata))
            report = module.describe_results(root)
        self.assertEqual(report['attempt_count'], 2)
        self.assertEqual([attempt['attempt_status'] for attempt in report['attempts']],
                         ['VALID_TRIAL', 'INVALID_TRIAL'])
        self.assertTrue(all(attempt['method'] == 'ours' for attempt in report['attempts']))

    def test_rm4d_only_mechanism_is_not_applicable_and_physical_metrics_survive(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write_attempt(root / 'attempt-01', method='rm4d_only', rounds=False)
            attempt = module.describe_results(root)['attempts'][0]
        self.assertEqual(attempt['mechanism_status'], 'NOT_APPLICABLE')
        self.assertIsNone(attempt['outcomes']['D_env'])
        self.assertIsNone(attempt['confirmed_final']['count'])
        self.assertFalse(attempt['outcomes']['D_exec'])
        self.assertFalse(attempt['outcomes']['retrieval_success'])
        self.assertEqual(attempt['rounds'], [])

    def test_cli_prints_json_and_preserves_saved_inputs(self):
        import sys
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write_attempt(root / 'attempt-01')
            before = {str(p): p.read_bytes() for p in root.rglob('*') if p.is_file()}
            run = subprocess.run([sys.executable, str(SCRIPT), '--results-dir', str(root)],
                                 capture_output=True, text=True, check=True)
            self.assertEqual(json.loads(run.stdout)['attempt_count'], 1)
            self.assertEqual(before, {str(p): p.read_bytes() for p in root.rglob('*') if p.is_file()})


if __name__ == '__main__':
    unittest.main()
