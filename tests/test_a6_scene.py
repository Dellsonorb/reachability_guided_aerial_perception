"""Method-blind necessary scene-admission checks, never a policy oracle."""

import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('a6_scene', ROOT / 'scripts/a6_scene.py')
scene_tools = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene_tools)


class SceneAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.scene = dict(target_xy=[2., 0.], target_z=.0575, target_yaw=0.)
        self.contract = dict(optical_xyz=[.13614773689465415, 0., .11272472554168546],
                             optical_rpy=[-1.9207963267948966, 0., -np.pi/2],
                             min_depth=.25, max_depth=4., target_size=[.240, .053, .115])

    def test_approved_view_has_no_brick_point_inside_frozen_depth_limit(self):
        report = scene_tools.depth_admission(self.scene, [-2.5, 0, 1.5, 0], self.contract)
        self.assertFalse(report['depth_overlap'])
        self.assertEqual(report['status'], 'SCENE_SETUP_INADMISSIBLE')
        np.testing.assert_allclose(report['optical_depth_bounds_m'], [4.500125537876818, 4.765008236817566])

    def test_existing_a5_view_has_depth_opportunity_not_promised_bootstrap_success(self):
        report = scene_tools.depth_admission(self.scene, [-.5, 0, 1.5, 0], self.contract)
        self.assertTrue(report['depth_overlap'])
        self.assertEqual(report['status'], 'DEPTH_OPPORTUNITY_ONLY')
        self.assertNotIn('success', report)

    def test_rotating_entire_scene_preserves_optical_depth(self):
        view = [-2.5, 0, 1.5, 0]
        original = scene_tools.depth_admission(self.scene, view, self.contract)
        rotated = dict(self.scene, target_xy=[0, 2.], target_yaw=np.pi/2)
        report = scene_tools.depth_admission(rotated, [0, -2.5, 1.5, np.pi/2], self.contract)
        np.testing.assert_allclose(report['optical_depth_bounds_m'], original['optical_depth_bounds_m'])

    def test_fixed_schedule_has_four_methods_per_scene_and_only_two_extra_hard_slots(self):
        config = json.loads((ROOT / 'configs/a6_pilot.json').read_text())
        self.assertEqual(len(config['slots']), config['attempt_limit'])
        self.assertEqual(config['attempt_limit'], 14)
        self.assertEqual([s['slot'] for s in config['slots']], list(range(1, 15)))
        for scene in ('easy', 'moderate', 'hard'):
            self.assertEqual({s['method'] for s in config['slots'][:12] if s['scene'] == scene},
                             {'fixed', 'generic', 'ours', 'rm4d_only'})
        self.assertEqual({(s['scene'], s['method']) for s in config['slots'][12:]},
                         {('hard', 'no_cost'), ('hard', 'no_occlusion')})
        self.assertEqual(config['primary_comparison'], ['ours', 'generic'])
        self.assertEqual(config['efficiency_clock'], 'simulation')

    def test_prelisted_seeds_fail_at_the_original_inadmissible_view(self):
        config = json.loads((ROOT / 'configs/a6_pilot.json').read_text())
        for scene in config['scenes']:
            with self.subTest(scene=scene['id']):
                self.assertFalse(scene_tools.depth_admission(scene, [-2.5, 0, 1.5, 0], self.contract)['depth_overlap'])

    def test_nonfinite_pose_is_rejected(self):
        with self.assertRaises(ValueError):
            scene_tools.depth_admission(self.scene, [float('nan'), 0, 1.5, 0], self.contract)


if __name__ == '__main__':
    unittest.main()
