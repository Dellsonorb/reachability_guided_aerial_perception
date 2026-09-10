import copy
import importlib.util
import json
import math
from pathlib import Path
import random
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class EvaluationDesignTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('paper1_eval_design'),
                             'pure evaluation generator is not implemented')
        import paper1_eval_design
        self.design = paper1_eval_design
        self.profile = json.loads((ROOT/'configs/current_sim_task.json').read_text())

    def test_cohort_size_balancing_and_subsets_are_fixed_before_outcomes(self):
        original = copy.deepcopy(self.profile)
        c = self.design.build_config(self.profile, {2026090801, 1339075492})
        self.assertEqual(self.profile, original)
        self.assertEqual(c, self.design.build_config(self.profile, {1339075492, 2026090801}))
        self.assertEqual(c['status'], 'FROZEN_FOR_EVALUATION')
        self.assertEqual(c['cohort'], 'paper1-eval-finite-scan-v1-final')
        self.assertEqual(len(c['scenes']), 96)
        self.assertEqual(len(c['slots']), 212)
        self.assertEqual(len({s['seed'] for s in c['scenes']}), 96)
        self.assertEqual(c['limits']['total_starts'], 224)
        self.assertEqual(c['limits']['reserve_starts'], 12)
        self.assertEqual(c['limits']['max_replacements_per_slot'], 1)
        self.assertEqual(c['limits']['wall_clock_hard_hours'], 120)
        self.assertEqual(c['limits']['new_data_gib'], 500)
        self.assertEqual(c['limits']['minimum_free_gib'], 100)
        self.assertEqual([r['slot'] for r in c['slots']], list(range(1, 213)))
        for tier, n in [('easy', 38), ('moderate', 28), ('hard', 30)]:
            scenes = [s for s in c['scenes'] if s['tier'] == tier]
            self.assertEqual(len(scenes), n)
            first = []
            for index, scene in enumerate(scenes):
                slots = [r for r in c['slots'] if r['scene'] == scene['id']]
                methods = [r['method'] for r in slots]
                self.assertEqual(set(methods[:2]), {'ours', 'generic'})
                self.assertEqual(slots[1]['slot'], slots[0]['slot']+1)
                first.append(methods[0])
                self.assertEqual({'rm4d_only', 'fixed'}.issubset(methods), index < 2)
                self.assertEqual({'no_occlusion', 'no_cost'}.issubset(methods),
                                 tier == 'hard' and index < 4)
            self.assertEqual(first.count('ours'), n//2)

    def test_seed_skip_only_and_geometry_draw_order(self):
        rng = random.Random(2026091022)
        first, second = rng.randrange(1, 2**31), rng.randrange(1, 2**31)
        c = self.design.build_config(self.profile, {first})
        self.assertEqual(c['scenes'][0]['seed'], second)
        for scene in c['scenes']:
            rng = random.Random(scene['seed'])
            self.assertEqual(scene['target_xy'], [2+rng.uniform(-.15,.15), rng.uniform(-.15,.15)])
            self.assertEqual(scene['target_yaw'], rng.uniform(-math.pi/6,math.pi/6))
            self.assertEqual(scene['bunker_xy'], [3+rng.uniform(-.1,.1),-2.5+rng.uniform(-.1,.1)])
            self.assertEqual(scene['bunker_yaw'], math.pi+rng.uniform(-math.pi/36,math.pi/36))
            centers = {'easy': [], 'moderate': [[.4,.65]], 'hard': [[.3,.5],[.5,-.4]]}[scene['tier']]
            self.assertEqual(len(scene['boxes']), len(centers))
            for box, center in zip(scene['boxes'], centers):
                self.assertEqual(box['center_xy'], [center[0]+rng.uniform(-.15,.15),center[1]+rng.uniform(-.15,.15)])
                self.assertEqual(box['yaw'], rng.uniform(-math.pi/12,math.pi/12))

    def test_commands_reference_only_explicit_slots_and_common_flags(self):
        c = self.design.build_config(self.profile, set())
        for slot in c['slots']:
            command = self.design.slot_command(c, Path('/config.json'), slot['slot'],
                                               Path('/data/final'))
            self.assertIn('run_retrieval.py', ' '.join(command))
            self.assertIn('evaluation', command)
            self.assertEqual(command[command.index('--slot')+1], str(slot['slot']))
            import run_retrieval
            actual = run_retrieval.task_command(Path('/sim'),Path('/rm'),Path('/config'),
                                               Path('/out'),slot=slot['slot'])
            for flag in ('--integrated-joint-velocity','--full-robot-manipulation',
                         '--execution-clearance','--ground-dynamics'):
                self.assertIn(flag, actual)
            self.assertNotIn('--ground-replay-from', command)
            self.assertNotIn('--setup-view', command)
        with self.assertRaises(ValueError):
            self.design.slot_command(c, Path('/config.json'), 213, Path('/data/final'))

    def test_generator_refuses_overwrite_and_preview_never_creates_output(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)/'manifest.json'
            out.write_text('historical')
            with self.assertRaises(FileExistsError):
                self.design.write_config(out, self.design.build_config(self.profile, set()))
            self.assertEqual(out.read_text(), 'historical')
            nonexistent = Path(directory)/'final'
            self.design.slot_command(self.design.build_config(self.profile,set()),
                                     Path('/config.json'), 1, nonexistent)
            self.assertFalse(nonexistent.exists())


if __name__ == '__main__':
    unittest.main()
