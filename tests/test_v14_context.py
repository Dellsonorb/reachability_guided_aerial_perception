"""Explicit v1.4 context and saved ground presence preserve legacy artifacts."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec
from operational_gating.io import build_operational_context, record_initial_context, save_operational
from tests.test_a5_core import scan


ROOT = Path(__file__).resolve().parents[1]


class V14ContextTests(unittest.TestCase):
    def setUp(self):
        self.grid = EnvironmentGridSpec((-.5, -.5), 10, 10)
        self.initial = dict(operational_gating='v1.4', perceived_target=dict(
            center_xyz=[.04, .04, .06], yaw_rad=0., size_xyz=[.10, .10, .12]))
        self.observations = [scan([[.01, .01, 0.], [.02, .02, .08], [.21, .21, 0.]], stamp)
                             for stamp in (1., 2.)]

    def test_cli_requires_explicit_v14_opt_in_and_keeps_budget(self):
        spec = importlib.util.spec_from_file_location('a5_v14_parser', ROOT / 'scripts/run_a5_sim.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        args = sum(([name, 'unused'] for name in ('--output-dir', '--core-python', '--rm4d-root',
                                                 '--rm4d-config', '--rm4d-map')), [])
        parser = module.build_parser()
        self.assertEqual(parser.parse_args(args).operational_gating, 'v1')
        revised = parser.parse_args(args + ['--operational-gating', 'v1.4'])
        self.assertEqual(revised.operational_gating, 'v1.4')
        self.assertEqual(revised.max_viewpoints, 3)

    def test_initial_annotation_forwards_v14_without_changing_grasp(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'initial.json'
            path.write_text(json.dumps(dict(grasp=dict(frame_id='map'))))
            record_initial_context(path, self.initial)
            saved = json.loads(path.read_text())
        self.assertEqual(saved['operational_gating'], 'v1.4')
        self.assertEqual(saved['grasp'], dict(frame_id='map'))
        self.assertEqual(saved['target_reference_status'], 'UNAVAILABLE')

    def test_context_persists_separate_measured_presence_with_legacy_votes_unchanged(self):
        revised, metadata = build_operational_context(
            self.initial, self.grid, self.observations, BeliefConfig())
        legacy, _ = build_operational_context(
            dict(self.initial, operational_gating='v1.3'), self.grid, self.observations, BeliefConfig())
        self.assertEqual(revised.operational_semantics, 'object-aware-v1.4')
        self.assertEqual(metadata['operational_semantics'], 'object-aware-v1.4')
        self.assertEqual(metadata['ground_semantics'],
                         'actual_ground_presence_before_occupied_suppression_with_independent_blocking')
        self.assertEqual(metadata['association_status'], 'UNAVAILABLE')
        self.assertIsNotNone(revised.ambiguous_endpoints)
        self.assertEqual(revised.ground_presence_votes.sum(), 4)
        self.assertEqual(revised.ground_votes.sum(), 2)
        self.assertEqual(revised.ambiguous_occupied_votes.sum(), 2)
        names = ('environment_occupied_votes', 'ambiguous_occupied_votes',
                 'target_occupied_votes', 'ground_votes')
        with tempfile.TemporaryDirectory() as directory:
            save_operational(directory, revised, metadata)
            with np.load(Path(directory) / 'operational_evidence.npz', allow_pickle=False) as saved:
                np.testing.assert_array_equal(saved['ground_presence_votes'], revised.ground_presence_votes)
                for name in names:
                    np.testing.assert_array_equal(getattr(revised, name), getattr(legacy, name))
                    np.testing.assert_array_equal(saved[name], getattr(legacy, name))
            summary = json.loads((Path(directory) / 'operational_summary.json').read_text())
        self.assertEqual(summary['votes']['ground_presence_votes'], 4)
        self.assertEqual(summary['votes']['ground_votes'], 2)
        self.assertEqual(summary['ground_semantics'], metadata['ground_semantics'])

    def test_legacy_context_and_saved_files_do_not_invent_presence(self):
        for revision in ('v1.1', 'v1.3'):
            with self.subTest(revision=revision), tempfile.TemporaryDirectory() as directory:
                view, metadata = build_operational_context(
                    dict(self.initial, operational_gating=revision),
                    self.grid, self.observations, BeliefConfig())
                self.assertIsNone(view.ground_presence_votes)
                self.assertEqual(view.ambiguous_endpoints is not None, revision == 'v1.3')
                self.assertEqual(metadata['ground_semantics'],
                                 'actual_ground_votes_with_environment_or_ambiguous_priority')
                save_operational(directory, view, metadata)
                with np.load(Path(directory) / 'operational_evidence.npz', allow_pickle=False) as saved:
                    self.assertNotIn('ground_presence_votes', saved.files)
                summary = json.loads((Path(directory) / 'operational_summary.json').read_text())
                self.assertNotIn('ground_presence_votes', summary['votes'])


if __name__ == '__main__':
    unittest.main()
