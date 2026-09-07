import importlib.util
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('a6_attempt', ROOT / 'scripts/run_a6_attempt.py')
attempt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(attempt)


class AttemptTests(unittest.TestCase):
    def setUp(self): self.config = json.loads((ROOT / 'configs/a6_pilot.json').read_text())

    def test_spawn_args_preserve_serialized_seed_geometry(self):
        scene = self.config['scenes'][2]
        args = attempt.scene_launch_args(scene)
        self.assertIn('target_x:='+str(scene['target_xy'][0]), args)
        self.assertIn('target_yaw:='+str(scene['target_yaw']), args)
        self.assertIn('bunker_yaw:='+str(scene['bunker_yaw']), args)
        self.assertIn('flight_position_tolerance:=0.05', args)

    def test_box_is_physical_ground_supported_and_not_red_target(self):
        box = self.config['scenes'][2]['boxes'][0]
        sdf = ET.fromstring(attempt.box_sdf(box))
        self.assertEqual(sdf.find('.//static').text, 'true')
        self.assertEqual(sdf.find('.//collision/geometry/box/size').text, '0.15 0.6 1.0')
        self.assertEqual(sdf.find('.//pose').text.split()[2], '0.5')
        self.assertEqual(sdf.find('.//visual/material/ambient').text, '0.4 0.4 0.4 1')

    def test_invalid_slot_cannot_launch_formal_or_extra_method(self):
        with self.assertRaises(ValueError): attempt.slot_spec(self.config, 15)
        with self.assertRaises(ValueError): attempt.slot_spec(self.config, 0)
        slot, scene = attempt.slot_spec(self.config, 14)
        self.assertEqual(slot['method'], 'no_cost')
        self.assertEqual(scene['id'], 'hard')

    def test_common_cli_has_three_windows_and_original_guards(self):
        args = attempt.adapter_args(self.config, Path('/tmp/test-out'), Path('/tmp/sim'), Path('/tmp/rm'))
        self.assertEqual(args[args.index('--max-viewpoints')+1], '3')
        self.assertEqual(args[args.index('--cloud-window-s')+1], '5.0')
        self.assertEqual(args[args.index('--navigation-timeout')+1], '120.0')
        self.assertEqual(args[args.index('--facade-position-tolerance')+1], '0.05')
        self.assertNotIn('--flight-weight', args)

    def test_lost_checker_outcome_is_invalid_but_cannot_erase_established_failure(self):
        status, success, reason = attempt.classify_outcome(None, [], 0, 2)
        self.assertEqual(status, 'INVALID_TRIAL')
        self.assertIsNone(success)
        self.assertIn('checker', reason)
        status, success, reason = attempt.classify_outcome(None, [{'state': 'FAILED', 'reason': 'no confirmed candidate'}], 1, 2)
        self.assertEqual(status, 'VALID_TRIAL')
        self.assertFalse(success)

    def test_observed_physical_lift_failure_is_retained(self):
        result = attempt.classify_outcome({'status': 'FAIL', 'error': 'physical lift below minimum'},
                                          [{'state': 'LIFT'}], 0, 1)
        self.assertEqual(result[:2], ('VALID_TRIAL', False))

    def test_connected_without_valid_odometry_is_not_common_ready(self):
        from types import SimpleNamespace
        self.assertFalse(attempt.state_ready(SimpleNamespace(connected=True, odom_valid=False)))
        self.assertTrue(attempt.state_ready(SimpleNamespace(connected=True, odom_valid=True)))

    def test_action_probe_does_not_wait_on_a_stopped_ros_clock(self):
        import threading
        from types import SimpleNamespace
        release = threading.Event()
        ready = attempt.action_probe(SimpleNamespace(wait_for_server=lambda: release.wait()))
        self.assertFalse(ready.is_set())
        release.set()
        self.assertTrue(ready.wait(1))

    def test_explicitly_missing_physical_measurement_is_not_a_negative_lift(self):
        for label in ('baseline', 'final'):
            physical = dict(status='FAIL', error='target '+label+' z is not finite')
            self.assertEqual(attempt.classify_outcome(physical, [{'state': 'LIFT'}], 0, 1)[:2],
                             ('INVALID_TRIAL', None))

    def test_primary_wrapper_timeout_is_not_overwritten_by_checker_cleanup(self):
        record = dict(reason='common_task_wall_guard_expired')
        attempt.attach_outcome(record, ('VALID_TRIAL', False, 'timed out waiting for LIFT'))
        self.assertEqual(record['reason'], 'common_task_wall_guard_expired')
        self.assertEqual(record['classification_reason'], 'timed out waiting for LIFT')

    def test_node_initializes_only_after_sim_time_parameter_is_ready(self):
        import time
        from types import SimpleNamespace
        values = iter([False, True])
        seen = []
        def get_param(name, default):
            self.assertEqual(name, '/use_sim_time')
            value = next(values); seen.append(value); return value
        def init_node(*args, **kwargs):
            self.assertEqual(seen, [False, True])
            seen.append('initialized')
        attempt.initialize_sim_node(SimpleNamespace(get_param=get_param, init_node=init_node),
                                    SimpleNamespace(poll=lambda: None), time.monotonic()+2)
        self.assertEqual(seen[-1], 'initialized')

    def test_already_exited_process_group_does_not_break_cleanup(self):
        from unittest.mock import patch, Mock
        process = Mock(pid=123, poll=Mock(return_value=None))
        with patch.object(attempt.os, 'killpg', side_effect=ProcessLookupError):
            attempt.stop_process(process)

    def test_truncated_measurements_retain_parsable_terminal_event(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root/'data').mkdir()
            (root/'physical_summary.json').write_text('{"status":')
            (root/'data/events.jsonl').write_text('{"state":"FAILED","reason":"no handoff"}\n{"state":')
            physical, events, errors = attempt.read_measurements(root)
            self.assertIsNone(physical)
            self.assertEqual(events[0]['reason'], 'no handoff')
            self.assertEqual(len(errors), 2)

    def test_launch_composition_only_forwards_common_public_spawn_arguments(self):
        source = '<launch><arg name="gui" default="false"/><include file="$(find sim_platform_bringup)/launch/air_ground_standalone.launch"><arg name="gui" value="$(arg gui)"/></include><node name="observer" pkg="demo" type="observer.py"/></launch>'
        result = ET.fromstring(attempt.runtime_launch(source, [-.5, 0., .15, 0.]))
        include = result.find('include')
        added = include.findall('arg')[1:]
        self.assertEqual([(a.get('name'), a.get('value')) for a in added],
                         [('uav1_init_x', '-0.5'), ('uav1_init_y', '0.0'),
                          ('uav1_init_z', '0.15'), ('uav1_init_yaw', '0.0')])
        for arg in added: include.remove(arg)
        self.assertEqual(ET.tostring(result), ET.tostring(ET.fromstring(source)))


if __name__ == '__main__': unittest.main()
