"""Prospective formal draws/order and metadata-only launcher compatibility."""

from collections import Counter
import contextlib
import copy
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import random
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
TIERS = ('easy', 'moderate', 'hard')
CORE = ['rm4d_only', 'fixed', 'generic', 'ours']


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / (name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


class FormalDesignTests(unittest.TestCase):
    def setUp(self):
        self.p1 = json.loads((ROOT / 'configs/a6_pilot.json').read_text())
        self.p2 = json.loads((ROOT / 'configs/a6_pilot2.json').read_text())

    def build(self, p1=None, p2=None):
        self.assertTrue((ROOT / 'scripts/a6_formal_design.py').is_file(),
                        'pure formal configuration generator must exist')
        return module('a6_formal_design').build_config(p1 or self.p1, p2 or self.p2)

    def test_serialized_config_reproduces_pure_builder_without_mutating_inputs(self):
        before = copy.deepcopy((self.p1, self.p2))
        config = self.build()
        path = ROOT / 'configs/a6_formal.json'
        self.assertTrue(path.is_file(), 'prospective formal configuration must be serialized')
        self.assertEqual(json.loads(path.read_text()), config)
        self.assertEqual(config, self.build())
        self.assertEqual((self.p1, self.p2), before)
        self.assertEqual(config['status'], 'FROZEN_FOR_FORMAL')
        self.assertEqual(config['protocol'], 'docs/A6_FORMAL_PROTOCOL.md')
        self.assertEqual(config['attempt_limit'], 560)

    def test_first_eligible_unique_seeds_use_replicate_then_tier_order(self):
        config = self.build()
        excluded = {s['seed'] for p in (self.p1, self.p2) for s in p['scenes']}
        rng = random.Random(202609083)
        expected = []
        while len(expected) < 120:
            seed = rng.randrange(1, 2**31)
            if seed not in excluded and seed not in expected:
                expected.append(seed)
        self.assertEqual([s['seed'] for s in config['scenes']], expected)
        self.assertEqual([(s['id'], s['tier'], s['replicate']) for s in config['scenes']],
                         [(f'{tier}-{rep:03d}', tier, rep)
                          for rep in range(1, 41) for tier in TIERS])
        self.assertEqual(config['seed_draw_seed'], 202609083)
        self.assertEqual(config['order_seed'], 202609084)
        # A pilot seed equal to the first draw is skipped, without replacing
        # or filtering any scene based on its geometry or observed outcome.
        p1 = copy.deepcopy(self.p1)
        p1['scenes'][0]['seed'] = expected[0]
        self.assertEqual(self.build(p1=p1)['scenes'][0]['seed'], expected[1])

    def test_all_six_draws_boxes_and_runtime_settings_remain_frozen(self):
        config = self.build()
        for key in ('initial_view', 'uav_launch_pose', 'observation_windows', 'window_sim_s',
                    'capture_wall_guard_s', 'task_wall_guard_s', 'efficiency_clock',
                    'primary_comparison', 'shared_a5_settings', 'operational_gating',
                    'record_diagnostics'):
            self.assertEqual(config[key], self.p2[key], key)
        templates = {s['id']: s for s in self.p2['scenes']}
        for scene in config['scenes']:
            for key in ('boxes', 'target_z', 'bunker_z'):
                self.assertEqual(scene[key], templates[scene['tier']][key])
            rng = random.Random(scene['seed'])
            expected = dict(target_xy=[2 + rng.uniform(-.15, .15), rng.uniform(-.15, .15)],
                            target_yaw=rng.uniform(-math.pi / 6, math.pi / 6),
                            bunker_xy=[3 + rng.uniform(-.1, .1), -2.5 + rng.uniform(-.1, .1)],
                            bunker_yaw=math.pi + rng.uniform(-math.pi / 36, math.pi / 36))
            for key, value in expected.items():
                self.assertEqual(scene[key], value)
        launch = module('run_a6_attempt')
        paths = [Path('/tmp/out'), Path('/tmp/sim'), Path('/tmp/rm')]
        self.assertEqual(launch.adapter_args(config, *paths), launch.adapter_args(self.p2, *paths))
        default_recording = {key: value for key, value in config.items()
                             if key != 'diagnostic_image_scope'}
        # After formal replicate 1, passive gripper diagnostics are appended
        # uniformly; the original pilot command and all robot arguments stay
        # unchanged. These topics carry no new input to a research policy.
        extra_topics = ['/ground/gripper_controller/follow_joint_trajectory/' + suffix
                        for suffix in ('goal', 'status', 'result', 'cancel')]
        extra_topics.append('/ground/gripper_controller/state')
        self.assertEqual(launch.diagnostic_command(default_recording, paths[0]),
                         launch.diagnostic_command(self.p2, paths[0]) + extra_topics)

    def test_formal_only_image_scope_is_identical_for_every_method(self):
        config = self.build()
        self.assertEqual(config.get('diagnostic_image_scope'), 'ground_handoff_to_end')
        self.assertNotIn('diagnostic_image_scope', self.p1)
        self.assertNotIn('diagnostic_image_scope', self.p2)
        self.assertTrue(config['record_diagnostics'])
        for slot in config['slots']:
            self.assertEqual(set(slot), {'slot', 'scene', 'method'})

    def test_order_matches_documented_rng_loops_and_exact_position_balance(self):
        config = self.build()
        rng, orders = random.Random(202609084), {}
        for tier in TIERS:
            for group in range(10):
                base, rotations = CORE[:], list(range(4))
                rng.shuffle(base)
                rng.shuffle(rotations)
                for offset, rotation in enumerate(rotations):
                    orders[tier, 4 * group + offset + 1] = base[rotation:] + base[:rotation]
        expected = []
        for rep in range(1, 41):
            tiers = list(TIERS)
            rng.shuffle(tiers)
            for tier in tiers:
                methods = orders[tier, rep][:]
                if tier == 'hard':
                    methods += ['no_occlusion', 'no_cost'] if rep % 2 else ['no_cost', 'no_occlusion']
                first = len(expected) + 1
                expected.extend(dict(slot=first + index,
                                     scene=f'{tier}-{rep:03d}', method=method)
                                for index, method in enumerate(methods))
        self.assertEqual(config['slots'], expected)
        self.assertEqual([s['slot'] for s in config['slots']], list(range(1, 561)))
        for tier in TIERS:
            for position in range(4):
                self.assertEqual(Counter(orders[tier, rep][position] for rep in range(1, 41)),
                                 Counter(dict.fromkeys(CORE, 10)))
        self.assertEqual(Counter(s['method'] for s in config['slots']),
                         Counter(dict.fromkeys(CORE, 120), no_occlusion=40, no_cost=40))

    def launch_metadata(self, config, options):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config_path, output = root / 'config.json', root / 'attempt'
            config_path.write_text(json.dumps(config))
            # The source launch file deliberately does not exist. Exercise the
            # real parser/record path, but no ROS import or subprocess can run.
            with patch.dict(os.environ), contextlib.redirect_stdout(io.StringIO()):
                try:
                    module('run_a6_attempt').main([
                        '--config', str(config_path), '--output-dir', str(output),
                        '--sim-root', str(root / 'absent-sim'), *options])
                except (ValueError, SystemExit) as error:
                    self.fail('supported formal metadata was rejected: ' + str(error))
            record = json.loads((output / 'attempt.json').read_text())
            self.assertIn('FileNotFoundError', record['reason'])
            self.assertFalse(record['task_started'])
            return record

    def test_launcher_records_formal_kind_without_changing_pilot_kind(self):
        formal = dict(self.p2, status='FROZEN_FOR_FORMAL')
        self.assertEqual(self.launch_metadata(formal, ['--slot', '1'])['kind'], 'FORMAL_ATTEMPT')
        self.assertEqual(self.launch_metadata(self.p2, ['--slot', '1'])['kind'], 'PILOT_ATTEMPT')

    def test_explicit_setup_scene_id_is_not_limited_to_tier_names(self):
        formal = copy.deepcopy(self.p2)
        formal.update(status='FROZEN_FOR_FORMAL')
        formal['scenes'][0].update(id='easy-001', tier='easy')
        record = self.launch_metadata(formal, ['--setup-scene', 'easy-001'])
        self.assertEqual(record['kind'], 'METHOD_INDEPENDENT_SETUP')
        self.assertEqual(record['scene'], 'easy-001')
        self.assertEqual(record['method'], 'SETUP_CHECK')

    def test_unlisted_formal_slots_and_method_pose_overrides_are_rejected(self):
        config = self.build()
        launch = module('run_a6_attempt')
        for number in (0, 561):
            with self.assertRaises(ValueError):
                launch.slot_spec(config, number)
        self.assertEqual(launch.slot_spec(config, 560)[0], config['slots'][-1])
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / 'config.json'
            path.write_text(json.dumps(config))
            for extra in (['--slot', '561'], ['--slot', '1', '--setup-view', '0', '0', '1', '0']):
                with self.assertRaises(ValueError):
                    launch.main(['--config', str(path), '--output-dir', str(root / 'out'), *extra])
                self.assertFalse((root / 'out').exists())

    def test_collector_selects_kind_from_config_and_never_pools_pilots(self):
        collector = module('summarize_a6_pilot')
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            slot, scene = self.p2['slots'][0], self.p2['scenes'][0]
            for index, kind in enumerate(('PILOT_ATTEMPT', 'FORMAL_ATTEMPT', 'METHOD_INDEPENDENT_SETUP')):
                directory = root / str(index)
                directory.mkdir()
                record = dict(slot, seed=scene['seed'], kind=kind, status='VALID_TRIAL',
                              finish_wall=20., retrieval_success=kind == 'FORMAL_ATTEMPT')
                (directory / 'attempt.json').write_text(json.dumps(record))
            for status, expected in [('FROZEN_FOR_FORMAL', 'FORMAL_ATTEMPT'),
                                     ('FROZEN_FOR_PILOT', 'PILOT_ATTEMPT')]:
                report = collector.summarize_pilot(dict(self.p2, status=status), root)
                self.assertEqual(report['counts']['total_activations'], 1)
                self.assertEqual(report['counts']['excluded_records'], 2)
                row = report['slots'][0]
                self.assertEqual(row['attempts'][0]['raw_attempt']['kind'], expected)
                self.assertEqual(row['retrieval_success'], expected == 'FORMAL_ATTEMPT')

    def test_scene_description_uses_tier_but_preserves_explicit_scene_id(self):
        from test_a6_scene_description import SceneDescriptionTests
        fixture = SceneDescriptionTests()
        fixture.setUp()
        for tier in TIERS:
            fixture.scene['id'] = tier
            baseline = fixture.describe()
            fixture.scene.update(id=tier + '-001', tier=tier)
            try:
                formal = fixture.describe()
            except KeyError as error:
                self.fail('description must look up target ranges by tier: ' + str(error))
            self.assertEqual(formal['scene_id'], tier + '-001')
            self.assertEqual(formal['targets'], baseline['targets'])
            fixture.scene.pop('tier')


if __name__ == '__main__':
    unittest.main()
