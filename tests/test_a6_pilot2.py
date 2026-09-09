"""Pilot-2 setup is fixed independently of method outcomes."""

import copy
import importlib.util
import json
import math
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('a6_attempt_p2', ROOT/'scripts/run_a6_attempt.py')
attempt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(attempt)


class Pilot2Tests(unittest.TestCase):
    def setUp(self):
        self.old = json.loads((ROOT/'configs/a6_pilot.json').read_text())

    def config(self):
        path = ROOT/'configs/a6_pilot2.json'
        self.assertTrue(path.is_file(), 'independent Pilot-2 configuration must exist')
        return json.loads(path.read_text())

    def test_launcher_forwards_existing_v11_switch_without_changing_other_arguments(self):
        old = attempt.adapter_args(self.old, Path('/tmp/out'), Path('/tmp/sim'), Path('/tmp/rm'))
        new = attempt.adapter_args(dict(self.old, operational_gating='v1.1'),
                                   Path('/tmp/out'), Path('/tmp/sim'), Path('/tmp/rm'))
        self.assertIn('--operational-gating', new)
        index = new.index('--operational-gating')
        self.assertEqual(new[index+1], 'v1.1')
        self.assertEqual(new[:index]+new[index+2:], old)

    def test_frozen_settings_and_boxes_are_not_retuned(self):
        new = self.config()
        for key in ('initial_view', 'uav_launch_pose', 'shared_a5_settings',
                    'observation_windows', 'window_sim_s', 'capture_wall_guard_s',
                    'task_wall_guard_s', 'efficiency_clock', 'primary_comparison'):
            self.assertEqual(new[key], self.old[key], key)
        for old_scene, scene in zip(self.old['scenes'], new['scenes']):
            for key in ('id','boxes','target_z','bunker_z'):
                self.assertEqual(scene[key], old_scene[key])
        self.assertEqual(new['operational_gating'], 'v1.1')
        self.assertEqual(new['status'], 'FROZEN_FOR_PILOT')

    def test_seeds_are_first_draw_and_independent_of_pilot1(self):
        new = self.config()
        seeds = [s['seed'] for s in new['scenes']]
        self.assertEqual(seeds, random.Random(202609082).sample(range(1,2**31),3))
        self.assertEqual(seeds, [1015873452,1240085801,656391333])
        self.assertTrue(set(seeds).isdisjoint(s['seed'] for s in self.old['scenes']))
        for scene in new['scenes']:
            r = random.Random(scene['seed'])
            expected = dict(target_xy=[2+r.uniform(-.15,.15),r.uniform(-.15,.15)],
                            target_yaw=r.uniform(-math.pi/6,math.pi/6),
                            bunker_xy=[3+r.uniform(-.1,.1),-2.5+r.uniform(-.1,.1)],
                            bunker_yaw=math.pi+r.uniform(-math.pi/36,math.pi/36))
            for key, value in expected.items():
                self.assertEqual(scene[key], value, key)

    def test_four_methods_paired_and_order_balanced_without_extra_task_ablation(self):
        new = self.config()
        order = ['rm4d_only','fixed','generic','ours']
        random.Random(new['order_seed']).shuffle(order)
        self.assertEqual(len(new['slots']),14)
        self.assertEqual([s['slot'] for s in new['slots']],list(range(1,15)))
        for tier, scene in enumerate(new['scenes']):
            slots = new['slots'][4*tier:4*tier+4]
            self.assertEqual([s['scene'] for s in slots],[scene['id']]*4)
            self.assertEqual([s['method'] for s in slots],order[tier:]+order[:tier])
        self.assertEqual([(s['scene'],s['method']) for s in new['slots'][12:]],
                         [('hard','no_occlusion'),('hard','no_cost')])

    def test_adding_option_does_not_mutate_existing_config(self):
        before = copy.deepcopy(self.old)
        attempt.adapter_args(dict(self.old,operational_gating='v1.1'),
                             Path('/tmp/out'),Path('/tmp/sim'),Path('/tmp/rm'))
        self.assertEqual(self.old,before)

    def test_passive_recording_is_opt_in_and_cannot_satisfy_checker_connection(self):
        self.assertTrue(hasattr(attempt, 'diagnostic_command'))
        self.assertIsNone(attempt.diagnostic_command(self.old, Path('/tmp/out')))
        command = attempt.diagnostic_command(dict(self.old, record_diagnostics=True), Path('/experiment/out'))
        self.assertEqual(command[:2], ['rosbag', 'record'])
        self.assertIn('--lz4', command)
        self.assertEqual(command[command.index('-O')+1], '/experiment/out/diagnostics.bag')
        self.assertNotIn('/air_ground_pick_demo/status', command)
        for topic in ('/clock', '/tf', '/tf_static', '/ground/nav_cmd_vel', '/ground/cmd_vel',
                      '/ground/d435/color/image_raw', '/ground/d435/depth/image_raw',
                      '/ground/arm_controller/state', '/ground/arm_controller/follow_joint_trajectory/result'):
            self.assertIn(topic, command)
        self.assertFalse(any('gazebo' in word or 'model_states' in word for word in command))

    def test_retention_diagnostics_include_existing_facade_and_contact_sensor(self):
        command = attempt.diagnostic_command(dict(self.old, record_diagnostics=True), Path('/experiment/out'))
        self.assertIn('/ground/gripper/grasp_confirmed', command)
        self.assertIn('/pick_target/contacts', command)


if __name__ == '__main__':
    unittest.main()
