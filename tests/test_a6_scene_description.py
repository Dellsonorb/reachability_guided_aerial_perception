"""Post-freeze descriptions use true geometry and first-window N=0 evidence."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/a6_scene_description.py'


class SceneDescriptionTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SCRIPT.is_file(), 'offline scene description is not implemented')
        spec = importlib.util.spec_from_file_location('a6_scene_description', SCRIPT)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.scene = dict(id='easy', seed=1, boxes=[])
        self.initial = dict(grasp=dict(position_xyz=[2., 0., .08]),
                            config=dict(grid_width_m=4., grid_height_m=2.))
        self.ranking = dict(current=dict(position_xyz=[0., 0., 1.2], yaw_rad=0., frame_id='map'),
                            sensor=dict(T_uav_lidar=np.eye(4).tolist(), min_elevation_deg=-90.,
                                        max_elevation_deg=90., horizontal_fov_deg=360),
                            a2_config=dict(ground_z_m=0., min_range_m=.2, max_range_m=40.),
                            a3_summary=dict(grid=dict(origin_xy=[0., -1.], width_cells=40,
                                                      height_cells=20, resolution_m=.1, frame_id='map')))
        self.decision = dict(round=1, assessments=[dict(candidate_id='exact', x=3., y=0., yaw=0.,
                                  relevance=1., footprint_clipped=False, free_cells=0,
                                  occupied_cells=96, representative_blocked=True, confirmed=False)])
        self.arrays = dict(a3_nominal_task_relevance=np.ones((20, 40)),
                           a2_observation_count=np.zeros((20, 40), dtype=int),
                           a2_state=np.full((20, 40), -1))

    def describe(self):
        return self.module.describe_snapshot(self.scene, self.initial, self.ranking,
                                             self.decision, self.arrays)

    def wall(self, x=1., y=0., height=1.):
        return dict(name='true_wall', center_xy=[x, y], size_xyz=[.1, 2., height], yaw=0.)

    def test_clear_scene_uses_all_exact_catalog_footprints_despite_belief_blocking(self):
        report = self.describe()
        self.assertEqual(report['high_support_patch_cells'], 96)
        self.assertEqual(report['eligible_high_support_cells'], 96)
        self.assertEqual(report['nominal_clear_high_relevance_footprints'], 1)
        self.assertEqual(report['o_task'], 0.)
        self.assertTrue(report['targets']['o_task']['met'])

    def test_true_wall_blocks_rays_without_any_a2_occupied_cells(self):
        self.scene.update(id='moderate', boxes=[self.wall()])
        report = self.describe()
        self.assertEqual(report['true_box_occluded_high_support_cells'], 96)
        self.assertEqual(report['o_task'], 1.)
        self.assertFalse(report['targets']['o_task']['met'])
        self.assertEqual(report['proposal_mismatches'], ['o_task'])

    def test_rays_above_actual_one_meter_box_are_not_occluded(self):
        self.scene['boxes'] = [self.wall()]
        self.ranking['current']['position_xyz'][2] = 2.
        self.assertEqual(self.describe()['o_task'], 0.)

    def test_actual_box_height_changes_occlusion_but_belief_state_does_not(self):
        self.scene['boxes'] = [self.wall(height=.1)]
        self.arrays['a2_state'][:] = 100
        self.assertEqual(self.describe()['o_task'], 0.)
        self.scene['boxes'] = [self.wall(height=1.)]
        self.assertEqual(self.describe()['o_task'], 1.)

    def test_true_footprint_collision_and_clipping_exclude_high_patch(self):
        for change in ('collision', 'clipping', 'low_relevance'):
            with self.subTest(change=change):
                self.setUp()
                if change == 'collision':
                    self.scene['boxes'] = [self.wall(x=3.)]
                elif change == 'clipping':
                    self.decision['assessments'][0]['x'] = 3.8
                else:
                    self.decision['assessments'][0]['relevance'] = .69
                report = self.describe()
                self.assertEqual(report['high_support_patch_cells'], 0)
                self.assertIsNone(report['o_task'])
                self.assertIsNone(report['targets']['o_task']['met'])

    def test_closed_true_box_contact_counts_as_footprint_collision(self):
        self.scene['boxes'] = [self.wall(x=3.57)]
        self.assertEqual(self.describe()['nominal_clear_high_relevance_footprints'], 0)
        self.scene['boxes'][0]['center_xy'][0] += .0001
        self.assertEqual(self.describe()['nominal_clear_high_relevance_footprints'], 1)

    def test_rotated_footprint_sat_does_not_use_its_axis_aligned_envelope(self):
        self.decision['assessments'][0].update(x=2., yaw=np.pi / 4.)
        self.scene['boxes'] = [dict(name='corner', center_xy=[2.6, -.6], size_xyz=[.1, .1, 1.], yaw=0.)]
        self.assertEqual(self.describe()['nominal_clear_high_relevance_footprints'], 1)
        self.scene['boxes'][0]['center_xy'] = [2.3, .3]
        self.assertEqual(self.describe()['nominal_clear_high_relevance_footprints'], 0)

    def test_clipped_footprints_still_contribute_to_candidate_relative_union(self):
        self.decision['assessments'][0]['x'] = 3.8
        report = self.describe()
        self.assertEqual(report['high_support_patch_cells'], 0)
        self.assertEqual(report['catalog_union_cells'], 64)
        self.assertEqual(report['candidate_relative_unsupported_never_observed_cells'], 736)

    def test_n_zero_areas_do_not_count_observed_but_still_unknown_cells(self):
        self.arrays['a2_observation_count'][:] = 1
        self.arrays['a2_observation_count'][6, 24] = 0
        self.arrays['a2_observation_count'][0, 0] = 0
        report = self.describe()
        self.assertEqual(report['state_unknown_cells'], 800)
        self.assertEqual(report['never_observed_cells'], 2)
        self.assertAlmostEqual(report['a_task_unknown_m2'], .01)
        self.assertAlmostEqual(report['a_irrel_m2'], .01)
        self.assertAlmostEqual(report['irrelevant_to_task_unknown_ratio'], .1)

    def test_low_nominal_and_candidate_relative_unsupported_are_union_not_sum(self):
        self.arrays['a3_nominal_task_relevance'][:] = np.nan
        self.arrays['a3_nominal_task_relevance'][6:14, 24:36] = 1.
        self.arrays['a3_nominal_task_relevance'][6, 24] = .1
        report = self.describe()
        self.assertEqual(report['high_support_patch_cells'], 95)
        self.assertEqual(report['candidate_relative_unsupported_never_observed_cells'], 704)
        self.assertEqual(report['irrelevant_never_observed_cells'], 705)

    def test_no_range_fov_eligible_cells_is_undefined_not_zero(self):
        self.ranking['a2_config']['max_range_m'] = 1.
        report = self.describe()
        self.assertEqual(report['eligible_high_support_cells'], 0)
        self.assertIsNone(report['o_task'])
        self.assertIn('o_task', report['undefined_targets'])
        self.assertNotIn('o_task', report['proposal_mismatches'])
        json.dumps(report, allow_nan=False)

    def test_occlusion_denominator_excludes_high_patch_outside_range(self):
        self.scene['boxes'] = [self.wall()]
        self.ranking['a2_config']['max_range_m'] = 3.
        report = self.describe()
        self.assertGreater(report['eligible_high_support_cells'], 0)
        self.assertLess(report['eligible_high_support_cells'], report['high_support_patch_cells'])
        self.assertEqual(report['o_task'], 1.)

    def test_sensor_mount_rotation_and_saved_yaw_determine_elevation_fov(self):
        c, s = np.cos(.35), np.sin(.35)
        mount = np.eye(4)
        mount[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
        mount[:3, 3] = [.13, 0., .28]
        self.ranking['sensor'].update(T_uav_lidar=mount.tolist(), min_elevation_deg=-7., max_elevation_deg=52.)
        forward = self.describe()
        self.assertGreater(forward['eligible_high_support_cells'], 0)
        self.ranking['current']['yaw_rad'] = np.pi
        backward = self.describe()
        self.assertEqual(backward['eligible_high_support_cells'], 0)
        np.testing.assert_allclose(backward['sensor_origin_map_m'], [-.13, 0., 1.48], atol=1e-12)

    def test_snapshot_grid_must_match_initial_grasp_centered_grid(self):
        self.ranking['a3_summary']['grid']['origin_xy'][0] += .01
        with self.assertRaisesRegex(ValueError, 'grid'):
            self.describe()

    def test_cli_reads_only_available_first_windows_and_does_not_fail_proposal_mismatch(self):
        self.scene.update(id='hard', boxes=[self.wall()])
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config_path, output = root / 'config.json', root / 'description.json'
            config_path.write_text(json.dumps(dict(scenes=[self.scene])))
            attempt = root / 'results' / 'slot-01-hard-ours-attempt-01'
            window = attempt / 'data' / 'rounds' / 'round-01'
            window.mkdir(parents=True)
            (attempt / 'attempt.json').write_text(json.dumps(dict(slot=1, scene='hard', seed=1,
                                         method='ours', scene_spec=self.scene, status='VALID_TRIAL')))
            (attempt / 'data' / 'initial.json').write_text(json.dumps(self.initial))
            (window / 'ranking.json').write_text(json.dumps(self.ranking))
            (window / 'decision.json').write_text(json.dumps(self.decision))
            np.savez(window / 'fields.npz', **self.arrays)
            later = window.with_name('round-02')
            later.mkdir()
            (later / 'ranking.json').write_text('not a first window')
            partial = attempt.with_name('slot-02-hard-fixed-attempt-01')
            partial.mkdir()
            (partial / 'attempt.json').write_text(json.dumps(dict(slot=2, scene='hard', seed=1,
                                         method='fixed', scene_spec=self.scene, status='RUNNING')))
            with contextlib.redirect_stdout(io.StringIO()):
                code = self.module.main(['--config', str(config_path), '--results-dir', str(root / 'results'),
                                         '--output', str(output)])
            report = json.loads(output.read_text())
            self.assertEqual(code, 0)
            self.assertTrue(report['descriptive_only'])
            self.assertEqual(report['snapshot_count'], 1)
            self.assertEqual(len(report['unavailable_snapshots']), 1)
            self.assertIn('o_task', report['snapshots'][0]['proposal_mismatches'])
            self.assertEqual(report['snapshots'][0]['actual_method'], 'ours')


if __name__ == '__main__':
    unittest.main()
