"""Prospective v1.2 validation is fresh, paired and explicitly exact-anchored."""

import copy
import importlib.util
import json
import math
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('a6_attempt_v12', ROOT/'scripts/run_a6_attempt.py')
attempt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(attempt)


class V12ValidationTests(unittest.TestCase):
    def setUp(self):
        self.old = json.loads((ROOT/'configs/a6_pilot2.json').read_text())

    def config(self):
        path = ROOT/'configs/a6_v12_validation.json'
        self.assertTrue(path.is_file(), 'the complete fresh six-slot configuration must exist')
        return json.loads(path.read_text())

    def args(self, config):
        return attempt.adapter_args(config, Path('/tmp/out'), Path('/tmp/sim'), Path('/tmp/rm'))

    def test_explicit_anchor_is_forwarded_without_changing_legacy_arguments(self):
        original = self.args(self.old)
        self.assertNotIn('--support-anchor', original)
        for anchor in ('exact_winner', 'cell_center'):
            with self.subTest(anchor=anchor):
                configured = dict(self.old, support_anchor=anchor)
                before = copy.deepcopy(configured)
                self.assertEqual(self.args(configured), original+['--support-anchor', anchor])
                self.assertEqual(configured, before)

    def test_frozen_common_settings_and_tier_geometry_are_unchanged(self):
        new = self.config()
        for key in ('initial_view', 'uav_launch_pose', 'shared_a5_settings',
                    'observation_windows', 'window_sim_s', 'capture_wall_guard_s',
                    'task_wall_guard_s', 'efficiency_clock', 'primary_comparison'):
            self.assertEqual(new[key], self.old[key], key)
        self.assertEqual(len(new['scenes']), 3)
        for old_scene, scene in zip(self.old['scenes'], new['scenes']):
            for key in ('id', 'boxes', 'target_z', 'bunker_z'):
                self.assertEqual(scene[key], old_scene[key], key)
        self.assertEqual(new['operational_gating'], 'v1.1')
        self.assertEqual(new['support_anchor'], 'exact_winner')
        self.assertEqual(new['status'], 'FROZEN_FOR_PILOT')
        self.assertEqual(new['experiment'], 'V12_FRESH_PAIRED_VALIDATION')
        self.assertEqual(new['protocol'], 'docs/V12_FRESH_VALIDATION_PROTOCOL.md')
        self.assertTrue(new['record_diagnostics'])
        self.assertEqual(new['diagnostic_image_scope'], 'ground_handoff_to_end')

    def test_seeds_are_first_eligible_draws_excluding_all_old_reservations(self):
        new = self.config()
        excluded = set()
        for filename in ('a6_pilot.json', 'a6_pilot2.json', 'a6_formal.json'):
            old = json.loads((ROOT/'configs'/filename).read_text())
            excluded.update(scene['seed'] for scene in old['scenes'])
        self.assertEqual(len(excluded), 126)
        self.assertEqual(new['seed_draw_seed'], 2026090901)
        rng = random.Random(new['seed_draw_seed'])
        expected = []
        draws = 0
        while len(expected) < 3:
            seed = rng.randrange(1, 2**31)
            draws += 1
            if seed not in excluded and seed not in expected:
                expected.append(seed)
        self.assertEqual(draws, 3, 'no discarded initial draw')
        self.assertEqual(expected, [1249652395, 405111274, 1450983937])
        actual = [scene['seed'] for scene in new['scenes']]
        self.assertEqual(actual, expected)
        self.assertTrue(set(actual).isdisjoint(excluded))

    def test_serialized_scene_values_follow_original_six_draw_rule(self):
        for scene in self.config()['scenes']:
            rng = random.Random(scene['seed'])
            expected = dict(
                target_xy=[2+rng.uniform(-.15, .15), rng.uniform(-.15, .15)],
                target_yaw=rng.uniform(-math.pi/6, math.pi/6),
                bunker_xy=[3+rng.uniform(-.1, .1), -2.5+rng.uniform(-.1, .1)],
                bunker_yaw=math.pi+rng.uniform(-math.pi/36, math.pi/36))
            for key, value in expected.items():
                self.assertEqual(scene[key], value, (scene['id'], key))

    def test_only_six_paired_slots_follow_prospective_balanced_order(self):
        new = self.config()
        self.assertEqual(new['attempt_limit'], 6)
        self.assertEqual(new['order_seed'], 2026090902)
        order = ['generic', 'ours']
        random.Random(new['order_seed']).shuffle(order)
        expected = []
        for index, scene in enumerate(new['scenes']):
            for method in (order if index % 2 == 0 else order[::-1]):
                expected.append(dict(slot=len(expected)+1, scene=scene['id'], method=method))
        self.assertEqual(new['slots'], expected)
        self.assertEqual([(slot['scene'], slot['method']) for slot in expected],
                         [('easy', 'generic'), ('easy', 'ours'),
                          ('moderate', 'ours'), ('moderate', 'generic'),
                          ('hard', 'generic'), ('hard', 'ours')])
        for slot in expected:
            selected, scene = attempt.slot_spec(new, slot['slot'])
            self.assertEqual(selected, slot)
            self.assertEqual(scene['id'], slot['scene'])
        for invalid in (0, 7, 17):
            with self.assertRaises(ValueError):
                attempt.slot_spec(new, invalid)

    def test_validation_common_arguments_only_add_explicit_exact_anchor(self):
        new = self.config()
        self.assertEqual(self.args(new), self.args(self.old)+['--support-anchor', 'exact_winner'])


if __name__ == '__main__':
    unittest.main()
