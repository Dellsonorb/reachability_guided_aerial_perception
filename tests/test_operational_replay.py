"""Recorded replay cannot invent association or rewrite a Pilot-1 outcome."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec
from environment_belief.outputs import save_belief
from reachability_guided_aerial_perception import GraspTCP
from sim_active_perception.core import A5Config, assess_candidates, candidate_catalog, replay_observations
from sim_active_perception.worker import make_field, save_initial
from tests.test_a5_core import ground_points, scan
from tests.test_field import candidate, result


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/replay_operational_gating.py'
PILOT_ROOT = Path(__file__).resolve().parents[1] / 'outputs/a6/pilot-20260908'


def load_script():
    spec = importlib.util.spec_from_file_location('operational_replay', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class OperationalReplayTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SCRIPT.is_file(), 'the recorded operational replay is not implemented')
        self.replay = load_script()

    def fixture(self, root):
        slot = root / 'slot-12-hard-ours-attempt-02'
        data = slot / 'data'
        grasp = GraspTCP('replay-test', 'map', (.68, 0., .35), (0., 0., 0., 1.))
        raw = result([candidate(candidate_id='one', x=.001, y=.001, margin=.3)])
        raw['grasp_id'] = grasp.grasp_id
        config = A5Config()
        save_initial(grasp, raw, config, data)
        field = make_field(grasp, raw, config)
        grid = EnvironmentGridSpec(field.grid.origin_xy, field.grid.width_cells, field.grid.height_cells)
        points = np.vstack((ground_points(grid), [[.57, .01, .06]]))
        observations = [scan(points, stamp) for stamp in (1., 2.)]
        paths = []
        for index, observation in enumerate(observations, 1):
            path = data / f'observation_{index:02d}.npz'
            np.savez_compressed(path, points_xyz=observation.points_xyz, frame_id=observation.frame_id,
                                stamp_s=observation.stamp_s, T_map_sensor=observation.T_map_sensor,
                                valid_return=observation.valid_return)
            paths.append(str(path))
            directory = data / 'rounds' / f'round-{index:02d}'
            belief = replay_observations(grid, observations[:index], BeliefConfig())
            save_belief(belief, directory / 'a2')
            assessments = assess_candidates(field, belief, candidate_catalog(field, raw))
            (directory / 'decision.json').write_text(json.dumps(dict(assessments=assessments)))
            request_dir = directory / 'request-copy'
            request_dir.mkdir()
            (request_dir / 'request.json').write_text(json.dumps(dict(op='observe', observations=list(paths))))
        (data / 'events.jsonl').write_text(json.dumps(dict(
            state='AIR_HANDOFF', target_map=[.68, 0., .0575, 0.], observation_stamp=.5,
        )) + '\n')
        (slot / 'attempt.json').write_text(json.dumps(dict(
            status='VALID_TRIAL', scene='hard', method='ours',
            scene_spec=dict(target_xy=[99., 99.]), retrieval_success=False,
        )))
        return slot

    def test_replay_uses_handoff_and_no_reference_preserves_raw_belief_and_failures(self):
        with tempfile.TemporaryDirectory() as directory:
            slot = self.fixture(Path(directory))
            before = {str(p): p.read_bytes() for p in slot.rglob('*') if p.is_file()}
            report = self.replay.replay_slot(slot)
            self.assertEqual(report['association_status'], 'UNAVAILABLE')
            self.assertIn('RGB-D', report['limitation'])
            self.assertEqual(report['target']['center_xyz'], (.68, 0., .0575))
            self.assertEqual(len(report['rounds']), 2)
            for round_report in report['rounds']:
                self.assertTrue(all(round_report['raw_a2_exact_checks'].values()))
                self.assertEqual(len(round_report['raw_a2_exact_checks']), 10)
                self.assertTrue(round_report['saved_v1_assessments_exact'])
                self.assertTrue(round_report['nominal_task_relevance_unchanged'])
                self.assertEqual(round_report['target_occupied_votes'], 0)
                self.assertEqual(round_report['totals']['v1_confirmed'], 0)
                self.assertEqual(round_report['totals']['v11_confirmed'], 0)
                candidate_report = round_report['candidates'][0]
                self.assertTrue(candidate_report['v1']['exact_blocked'])
                self.assertTrue(candidate_report['v11']['exact']['blocked'])
                self.assertEqual(candidate_report['v11']['exact']['target_cells'], 0)
            after = {str(p): p.read_bytes() for p in slot.rglob('*') if p.is_file()}
            self.assertEqual(before, after)

    def test_raw_a2_mismatch_aborts_instead_of_reporting_a_corrected_trial(self):
        with tempfile.TemporaryDirectory() as directory:
            slot = self.fixture(Path(directory))
            path = slot / 'data/rounds/round-02/a2/belief.npz'
            with np.load(path, allow_pickle=False) as saved:
                arrays = {name: saved[name].copy() for name in saved.files}
            arrays['free_evidence'][0, 0] += 1
            np.savez_compressed(path, **arrays)
            with self.assertRaisesRegex(ValueError, 'raw A2.*free_evidence'):
                self.replay.replay_slot(slot)

    def test_saved_legacy_candidate_mismatch_is_reported_as_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            slot = self.fixture(Path(directory))
            path = slot / 'data/rounds/round-02/decision.json'
            value = json.loads(path.read_text())
            value['assessments'][0]['confirmed'] = True
            path.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError, 'saved v1 assessments'):
                self.replay.replay_slot(slot)

    def test_loader_retains_explicit_invalid_return_mask(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'cloud.npz'
            np.savez_compressed(path, points_xyz=[[0., 0., -2.], [0., 0., -1.]],
                                frame_id='lidar', stamp_s=1., T_map_sensor=np.eye(4),
                                valid_return=[True, False])
            observation = self.replay.load_observation(path)
            np.testing.assert_array_equal(observation.valid_return, [True, False])

    def test_handoff_missing_cannot_fall_back_to_scene_ground_truth(self):
        with tempfile.TemporaryDirectory() as directory:
            slot = self.fixture(Path(directory))
            (slot / 'data/events.jsonl').write_text('')
            with self.assertRaisesRegex(ValueError, 'AIR_HANDOFF'):
                self.replay.replay_slot(slot)

    def test_rectangle_gap_handles_overlap_contact_and_rotated_separation(self):
        square = np.array([[0., 0.], [1., 0.], [1., 1.], [0., 1.]])
        self.assertEqual(self.replay.rectangle_separation_m(square, square + [.2, .2]), 0.)
        self.assertEqual(self.replay.rectangle_separation_m(square, square + [1., 0.]), 0.)
        self.assertAlmostEqual(self.replay.rectangle_separation_m(square, square + [2., 2.]), np.sqrt(2))

    def test_known_recorded_ours_geometry_remains_geometry_only(self):
        example = self.replay.geometry_example(PILOT_ROOT / 'slot-12-hard-ours-attempt-02')
        self.assertEqual(example['candidate_id'], 'candidate-000008')
        self.assertEqual(example['source_id'], 585)
        self.assertAlmostEqual(example['continuous_separation_m'], .117820, places=6)
        self.assertTrue(example['exact_overlaps_occupied_cell_781'])
        self.assertFalse(example['representative_overlaps_occupied_cell_781'])
        self.assertEqual(example['interpretation'], 'GEOMETRY_REGRESSION_ONLY_NOT_A_CORRECTED_TRIAL')

    def test_output_refuses_pilot_tree_and_existing_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'pilot'
            root.mkdir()
            with self.assertRaisesRegex(ValueError, 'Pilot'):
                self.replay.write_report(root, root / 'derived', {})
            output = Path(directory) / 'existing'
            output.mkdir()
            marker = output / 'keep'
            marker.write_text('original')
            with self.assertRaises(FileExistsError):
                self.replay.write_report(root, output, {})
            self.assertEqual(marker.read_text(), 'original')


if __name__ == '__main__':
    unittest.main()
