import json
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec
from operational_gating.io import build_operational_context, record_initial_context, save_operational
from tests.test_a5_core import scan
from tests.test_target_association import reference


class OperationalWorkerTests(unittest.TestCase):
    def setUp(self):
        self.grid = EnvironmentGridSpec((-.5, -.5), 10, 10)
        self.initial = dict(operational_gating='v1.1',
                            perceived_target=dict(center_xyz=[0, 0, 1.], yaw_rad=0,
                                                  size_xyz=[.24, .053, .115]))

    def test_default_v1_has_no_derived_view_or_initial_file_changes(self):
        self.assertEqual(build_operational_context({}, self.grid, [], BeliefConfig()), (None, None))
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'initial.json'
            p.write_text('{"grasp": "unchanged"}\n')
            before = p.read_bytes()
            record_initial_context(p, {})
            self.assertEqual(before, p.read_bytes())

    def test_runtime_cli_requires_explicit_v11_opt_in(self):
        spec = importlib.util.spec_from_file_location('a5_v11_parser', 'scripts/run_a5_sim.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        args = sum(([name, 'unused'] for name in ('--output-dir', '--core-python', '--rm4d-root',
                                                 '--rm4d-config', '--rm4d-map')), [])
        self.assertEqual(getattr(module.build_parser().parse_args(args), 'operational_gating', None), 'v1')
        self.assertEqual(module.build_parser().parse_args(args + ['--operational-gating', 'v1.1'])
                         .operational_gating, 'v1.1')

    def test_missing_recorded_reference_cannot_unlock_target(self):
        observation = scan([[0, 0, 1.05]], 2)
        view, metadata = build_operational_context(self.initial, self.grid, [observation], BeliefConfig())
        self.assertEqual(metadata['association_status'], 'UNAVAILABLE')
        self.assertEqual(view.target_occupied_votes.sum(), 0)
        self.assertEqual(view.ambiguous_occupied_votes.sum(), 1)

    def test_available_reference_labels_target_and_records_geometry_budget(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'reference.npz'
            np.savez_compressed(p, **reference(), target_pose_map_xyzyaw=[0, 0, 1, 0],
                                target_size_xyz=[.24, .053, .115])
            self.initial['target_reference_file'] = str(p)
            view, metadata = build_operational_context(self.initial, self.grid,
                                                       [scan([[0, 0, 1.05]], 2)], BeliefConfig())
            self.assertEqual(view.target_occupied_votes.sum(), 1)
            self.assertEqual(metadata['association_status'], 'AVAILABLE')
            self.assertGreater(view.target.geometry_allowance_m, .033)
            save_operational(Path(d) / 'derived', view, metadata)
            data = np.load(Path(d) / 'derived/operational_evidence.npz')
            np.testing.assert_array_equal(data['target_occupied_votes'], view.target_occupied_votes)
            self.assertEqual(json.loads((Path(d) / 'derived/operational_summary.json').read_text())
                             ['operational_semantics'], 'object-aware-v1.1')

    def test_reference_must_match_the_initial_perceived_object(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'reference.npz'
            np.savez_compressed(p, **reference(), target_pose_map_xyzyaw=[.2, 0, 1, 0],
                                target_size_xyz=[.24, .053, .115])
            self.initial['target_reference_file'] = str(p)
            with self.assertRaisesRegex(ValueError, 'handoff'):
                build_operational_context(self.initial, self.grid, [], BeliefConfig())

    def test_initial_annotation_records_explicit_revision_without_grasp_change(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'initial.json'
            p.write_text('{"grasp": {"frame_id": "map", "position_xyz": [1, 2, 3]}}')
            record_initial_context(p, self.initial)
            result = json.loads(p.read_text())
            self.assertEqual(result['grasp']['position_xyz'], [1, 2, 3])
            self.assertEqual(result['operational_gating'], 'v1.1')

    def test_a5_and_a6_workers_route_explicit_revision_and_save_separate_evidence(self):
        from a6_pilot import worker as a6_worker
        from sim_active_perception import worker as a5_worker
        from sim_active_perception.core import A5Config
        from reachability_guided_aerial_perception import GraspTCP
        from tests.test_a5_core import inputs, ground_points
        from tests.test_field import candidate
        field, raw = inputs([candidate(candidate_id='one', x=.01, y=.02, margin=.3)])
        grasp = GraspTCP('a5-test', 'map', (0, 0, .4), (0, 0, 0, 1))
        for worker in (a5_worker, a6_worker):
            with self.subTest(worker=worker.__name__), tempfile.TemporaryDirectory() as d:
                response = a5_worker.save_initial(grasp, raw, A5Config(grid_width_m=3, grid_height_m=3), d)
                record_initial_context(response['initial_file'], self.initial)
                grid = EnvironmentGridSpec(field.grid.origin_xy, 30, 30)
                paths = []
                for stamp in (2., 3.):
                    o = scan(ground_points(grid), stamp)
                    p = Path(d) / ('observation-%s.npz' % stamp)
                    np.savez_compressed(p, points_xyz=o.points_xyz, frame_id=o.frame_id,
                                        stamp_s=o.stamp_s, T_map_sensor=o.T_map_sensor)
                    paths.append(str(p))
                request = dict(initial_file=response['initial_file'], observations=paths,
                               uav_pose=[-1, 0, 1.5, 0], output_dir=str(Path(d) / 'round'), method='ours')
                decision = worker.observe(request)
                self.assertEqual(decision.get('operational_semantics'), 'object-aware-v1.1')
                self.assertTrue((Path(d) / 'round/operational_evidence.npz').is_file())
                raw_a2 = np.load(Path(d) / 'round/a2/belief.npz')
                self.assertEqual(raw_a2['occupied_evidence'].sum(), 0)


if __name__ == '__main__':
    unittest.main()
