"""Arithmetic/replay checks only; no proposed operational gate is implemented."""

import importlib.util
from pathlib import Path
import shutil
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/diagnose_v13_ambiguous_geometry.py'


class GeometryDiagnosticTest(unittest.TestCase):
    def module(self):
        self.assertTrue(SCRIPT.exists(), 'read-only geometry diagnostic is not present')
        spec = importlib.util.spec_from_file_location('v13_geometry_diagnostic', SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_distance_is_closed_oriented_rectangle_distance(self):
        module = self.module()
        points = np.array([[0., 0.], [.52, .39], [.55, .43], [0., .49]])
        np.testing.assert_allclose(module.distances(points, [0., 0.], 0.), [0., 0., .05, .10], atol=1e-14)
        rotation = np.array([[0., -1.], [1., 0.]])
        np.testing.assert_allclose(module.distances(points @ rotation.T + [2., -3.],
                                                    [2., -3.], np.pi / 2),
                                   [0., 0., .05, .10], atol=1e-14)

    def test_recorded_replay_preserves_labels_and_reports_all_points(self):
        module = self.module()
        data = ROOT / 'outputs/a6/v12-fresh-validation/slot-003-moderate-ours-01/data'
        report = module.diagnose(data)
        self.assertTrue(report['frozen_vote_replay_equal'])
        self.assertTrue(report['diagnostic_only'])
        self.assertFalse(report['selection_changed'])
        self.assertEqual(report['occupied_endpoint_counts'], {'ENVIRONMENT': 8127, 'AMBIGUOUS': 109, 'TARGET': 62})
        self.assertEqual(len(report['ambiguous_endpoints']), 109)
        self.assertEqual(len(report['continuous_target_clear_winners']), 5)
        endpoint = next(p for p in report['ambiguous_endpoints'] if p['observation_row'] == 58439)
        self.assertEqual(endpoint['cell_id'], 781)
        self.assertTrue(endpoint['association']['raw_mask'])
        self.assertFalse(endpoint['association']['interior_mask'])
        np.testing.assert_allclose(endpoint['map_xyz'], [2.153377439257136, -.01053885251230817,
                                                       .11410059833230446], atol=1e-12)
        row = report['continuous_target_clear_winners'][0]
        self.assertAlmostEqual(row['minimum_legacy_blocking_endpoint_distance_m'], .118215, places=6)
        self.assertIsNone(report['total_physical_uncertainty_radius_m'])

    def test_mismatched_saved_votes_raise_explicit_input_error(self):
        module = self.module()
        source = ROOT / 'outputs/a6/v12-fresh-validation/slot-003-moderate-ours-01/data'
        with tempfile.TemporaryDirectory(prefix='v13-diagnostic-input-') as temporary:
            data = Path(temporary)
            round_dir = data / 'rounds/round-01'
            round_dir.mkdir(parents=True)
            for name in ('initial.json', 'target_reference.npz', 'observation_01.npz'):
                shutil.copyfile(source / name, data / name)
            for name in ('ranking.json', 'decision.json'):
                shutil.copyfile(source / 'rounds/round-01' / name, round_dir / name)
            with np.load(source / 'rounds/round-01/operational_evidence.npz') as saved:
                arrays = {name: saved[name].copy() for name in saved.files}
            arrays['ground_votes'][0, 0] += 1
            np.savez_compressed(round_dir / 'operational_evidence.npz', **arrays)
            with self.assertRaisesRegex(ValueError, 'vote.*replay'):
                module.diagnose(data)


if __name__ == '__main__':
    unittest.main()
