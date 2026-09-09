"""Opt-in v1.3 integration; historical files are read, never overwritten."""

from dataclasses import asdict, replace
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec, PointCloudObservation
from operational_gating.io import build_operational_context, record_initial_context, save_operational
from reachability_guided_aerial_perception import GraspTCP
from reachability_guided_nbv import Viewpoint
from sim_active_perception.core import A5Config, build_support_task, candidate_catalog, assess_candidates, replay_observations
from sim_active_perception.worker import make_field
from task_relevant_uncertainty.outputs import field_summary


DATA = Path(__file__).resolve().parents[1] / 'outputs/a6/v12-fresh-validation/slot-003-moderate-ours-01/data'


def moderate():
    initial = json.loads((DATA / 'initial.json').read_text())
    if initial.get('target_reference_file'):
        initial['target_reference_file'] = str(DATA / 'target_reference.npz')
    ranking = json.loads((DATA / 'rounds/round-01/ranking.json').read_text())
    config = A5Config(**initial['config'])
    field = make_field(GraspTCP(**initial['grasp']), initial['result'], config)
    grid = EnvironmentGridSpec(**ranking['a3_summary']['grid'])
    with np.load(DATA / 'observation_01.npz') as saved:
        observation = PointCloudObservation(saved['points_xyz'], str(saved['frame_id'].item()),
                                            float(saved['stamp_s']), saved['T_map_sensor'])
    belief = replay_observations(grid, [observation], BeliefConfig(**ranking['a2_config']))
    return initial, ranking, config, field, grid, observation, belief


class V13RuntimeTests(unittest.TestCase):
    def test_explicit_cli_opt_in_and_initial_annotation(self):
        spec = importlib.util.spec_from_file_location('a5_v13_parser', 'scripts/run_a5_sim.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        args = sum(([name, 'unused'] for name in ('--output-dir', '--core-python', '--rm4d-root',
                                                 '--rm4d-config', '--rm4d-map')), [])
        self.assertEqual(module.build_parser().parse_args(args + ['--operational-gating', 'v1.3'])
                         .operational_gating, 'v1.3')
        initial, *_ = moderate()
        initial['operational_gating'] = 'v1.3'
        with tempfile.TemporaryDirectory(prefix='v13-initial-') as directory:
            path = Path(directory) / 'initial.json'
            path.write_text(json.dumps({'grasp': initial['grasp']}))
            record_initial_context(path, initial)
            saved = json.loads(path.read_text())
            self.assertEqual(saved['operational_gating'], 'v1.3')
            self.assertEqual(saved['grasp'], initial['grasp'])

    def test_real_endpoints_persist_without_changing_any_votes(self):
        initial, _, _, _, grid, observation, belief = moderate()
        legacy, _ = build_operational_context(initial, grid, [observation], belief.config)
        revised, metadata = build_operational_context(dict(initial, operational_gating='v1.3'),
                                                       grid, [observation], belief.config)
        self.assertEqual(metadata['operational_semantics'], 'object-aware-v1.3')
        names = ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes', 'ground_votes')
        for name in names:
            np.testing.assert_array_equal(getattr(revised, name), getattr(legacy, name))
        self.assertEqual(len(revised.ambiguous_endpoints.points_xy), 109)
        self.assertEqual(revised.target, legacy.target)
        with tempfile.TemporaryDirectory(prefix='v13-evidence-') as directory:
            save_operational(directory, revised, metadata)
            with np.load(Path(directory) / 'operational_evidence.npz') as saved:
                np.testing.assert_array_equal(saved['ambiguous_endpoint_xy'], revised.ambiguous_endpoints.points_xy)
                self.assertEqual(float(saved['ambiguous_endpoint_radius_m']), .033)
                for name in names:
                    np.testing.assert_array_equal(saved[name], getattr(legacy, name))

    def test_missing_profile_sidecar_stays_non_pickle_readable_and_blocking(self):
        initial, _, _, _, grid, observation, belief = moderate()
        op, metadata = build_operational_context(dict(initial, operational_gating='v1.3'), grid,
                                                 [observation], belief.config)
        fallback = replace(op, ambiguous_endpoints=replace(op.ambiguous_endpoints, profile=None))
        with tempfile.TemporaryDirectory(prefix='v13-missing-profile-') as directory:
            save_operational(directory, fallback, metadata)
            with np.load(Path(directory) / 'operational_evidence.npz', allow_pickle=False) as saved:
                self.assertEqual(str(saved['ambiguous_endpoint_profile'].item()), '')
                self.assertFalse(bool(saved['ambiguous_endpoint_profile_available']))

    def test_shared_exact_gate_removes_alias_not_target_collision_or_ground_deficit(self):
        initial, _, config, field, grid, observation, belief = moderate()
        op, _ = build_operational_context(dict(initial, operational_gating='v1.3'), grid, [observation], belief.config)
        task = build_support_task(field, initial['result'], belief, config, operational=op)
        rows = assess_candidates(field, belief, candidate_catalog(field, initial['result']), task=task, operational=op)
        self.assertEqual(task.operational_semantics, 'object-aware-v1.3')
        self.assertEqual(field_summary(task)['operational_semantics'], task.operational_semantics)
        self.assertEqual(sum(r['operational']['target_collision'] for r in rows), 29)
        clear = [r for r in rows if not r['operational']['blocked']]
        self.assertEqual({r['source_id'] for r in clear}, {585, 705, 584, 623, 622})
        self.assertFalse(any(r['confirmed'] for r in rows))
        poses = {p.source_id: p for p in task.poses}
        for row in rows:
            self.assertEqual(row['operational']['blocked'], poses[row['source_id']].blocked)
            self.assertIn('ambiguous_subcell', row)
        self.assertGreater(float(np.nansum(task.task_relevant_uncertainty)), 0.)

    def test_generic_ours_use_the_same_state_candidates_visibility_and_cost(self):
        from a6_pilot.policy import decide_policy
        initial, ranking, config, field, grid, observation, belief = moderate()
        op, _ = build_operational_context(dict(initial, operational_gating='v1.3'), grid, [observation], belief.config)
        current = Viewpoint(**ranking['current'])
        results = [decide_policy(field, initial['result'], belief, current, method=method, round_count=1,
                                config=config, operational=op) for method in ('ours', 'generic')]
        first, second = results[0][1], results[1][1]
        self.assertEqual([asdict(c) for c in first.candidates], [asdict(c) for c in second.candidates])
        np.testing.assert_array_equal(first.visibility, second.visibility)
        self.assertEqual(results[0][0]['assessments'], results[1][0]['assessments'])


if __name__ == '__main__':
    unittest.main()
