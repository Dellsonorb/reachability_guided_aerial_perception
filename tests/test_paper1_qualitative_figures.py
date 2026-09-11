"""Offline figure checks: selection, coordinate frames, and frozen evidence."""
import importlib.util
import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np


class QualitativeFigureTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('paper1_qualitative_figures'),
                             'offline qualitative generator is not implemented')
        import paper1_qualitative_figures
        self.figures = paper1_qualitative_figures

    def frozen_results_or_skip(self):
        results = Path(__file__).resolve().parents[1]/'outputs/paper1-final-eval-v1'
        required = [results/'slots.csv']
        for slot, method in [(1, 'generic'), (2, 'ours')]:
            data = results/f'slot-{slot:03d}-eval-hard-015-{method}/attempt/data'
            required.append(data/'events.jsonl')
            for n in range(1, 4):
                required.append(data/f'observation_{n:02d}.npz')
                required.extend(data/'rounds'/f'round-{n:02d}'/name for name in
                                ('decision.json', 'ranking.json', 'fields.npz',
                                 'operational_evidence.npz', 'operational_summary.json'))
        missing = next((p for p in required if not p.is_file()), None)
        if missing is not None:
            self.skipTest(f'selected raw illustration evidence is unavailable: {missing}')
        return results

    def test_selects_first_scheduled_discordant_primary_pair_not_largest_gap(self):
        rows = []
        for scene, first, role, status in [('later', 20, 'primary', 'VALID_TRIAL'),
                                          ('first', 4, 'primary', 'VALID_TRIAL'),
                                          ('invalid', 1, 'primary', 'INVALID'),
                                          ('ablation', 2, 'ablation', 'VALID_TRIAL')]:
            for method, offset in [('generic', 0), ('ours', 1)]:
                rows.append(dict(scene=scene, slot=str(first+offset), tier='hard',
                                 comparison_role=role, status=status, method=method,
                                 retrieval_success=str(method == 'ours')))
        pair = self.figures.select_pair(rows)
        self.assertEqual([r['slot'] for r in pair], ['4', '5'])
        self.assertEqual(self.figures.select_pair(rows[::-1]), pair)
        with self.assertRaisesRegex(ValueError, 'discordant'):
            self.figures.select_pair(rows, tier='easy')

    def test_sensor_pose_is_converted_to_uav_pose_with_rotated_mount(self):
        mount = np.eye(4)
        mount[:3, 3] = [.2, 0, .3]
        uav = np.array([[0, -1, 0, 2], [1, 0, 0, 3], [0, 0, 1, 1], [0, 0, 0, 1.]])
        np.testing.assert_allclose(self.figures.uav_pose(uav @ mount, mount),
                                   [2, 3, 1, np.pi/2], atol=1e-12)

    def test_extent_preserves_per_run_origins_and_row_column_order(self):
        np.testing.assert_allclose(self.figures.grid_extent([.07, -.9], .1, (2, 4)),
                                   (.07, .47, -.9, -.7), atol=1e-12)

    def test_frozen_first_pair_has_runtime_sources_and_exact_anchor_deficits(self):
        results = self.frozen_results_or_skip()
        cases = self.figures.load_examples(results)
        self.assertEqual([c['slot'] for c in cases], [1, 2])
        self.assertEqual([c['confirmed_counts'] for c in cases], [[0, 0, 0], [0, 0, 3]])
        self.assertEqual([c['anchor']['operational']['ground_missing_cells'] for c in cases], [19, 0])
        for case in cases:
            self.assertEqual(case['frame_id'], 'map')
            self.assertEqual(len(case['windows']), 3)
            self.assertTrue(all(w['packet_start_stamp_s'] <= w['packet_end_stamp_s']
                                for w in case['windows']))
            for source in case['sources']:
                self.assertTrue((results/source['path']).is_file())
            np.testing.assert_allclose(case['poses'], case['csv_poses'], atol=1e-10)

    def test_runtime_ready_transition_is_distinct_from_worker_budget_reason(self):
        results = self.frozen_results_or_skip()
        generic, ours = self.figures.load_examples(results)
        self.assertEqual(ours.get('worker_stop_reason'), 'VIEW_BUDGET_REACHED')
        self.assertEqual(ours.get('runtime_stop_reason'), 'SCREENED_CANDIDATE_READY')
        self.assertEqual(generic.get('runtime_stop_reason'), 'VIEW_BUDGET_REACHED')
        self.assertEqual(ours['runtime_selected_event']['candidate_id'], ours['anchor']['candidate_id'])
        self.assertTrue(ours['runtime_decision_source'].endswith('events.jsonl:43'))

    def test_selected_anchor_rejects_runtime_candidate_or_pose_mismatch(self):
        self.assertTrue(hasattr(self.figures, 'validate_selected_anchor'))
        anchor = dict(candidate_id='candidate-000000', source_id=584, x=2., y=-.4, yaw=0.)
        self.figures.validate_selected_anchor(anchor, dict(anchor))
        for changed in (dict(candidate_id='candidate-000002'), dict(x=2.1), dict(source_id=585)):
            with self.assertRaisesRegex(ValueError, 'A5_SELECTED'):
                self.figures.validate_selected_anchor(anchor, dict(anchor, **changed))

    def test_missing_frozen_illustration_fails_instead_of_reselecting(self):
        rows = [dict(slot=str(slot), method=method, scene='eval-hard-015', seed='1395226981',
                     tier='hard', comparison_role='primary', status='VALID_TRIAL', windows='3',
                     retrieval_success=str(method == 'ours'),
                     selected_attempt=f'slot-{slot:03d}-eval-hard-015-{method}/attempt')
                for slot, method in [(1, 'generic'), (2, 'ours')]]
        with tempfile.TemporaryDirectory() as temporary:
            results = Path(temporary)
            with (results/'slots.csv').open('w', newline='') as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            with self.assertRaises(Exception) as caught:
                self.figures.load_examples(results)
            self.assertIsInstance(caught.exception, FileNotFoundError)
            self.assertRegex(str(caught.exception), 'Frozen Figure 5.*slot-001-eval-hard-015-generic')


if __name__ == '__main__':
    unittest.main()
