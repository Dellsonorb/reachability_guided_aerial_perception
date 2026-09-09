import importlib.util
import json
from pathlib import Path
import unittest
from types import SimpleNamespace
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('a6_attempt', ROOT / 'scripts/run_a6_attempt.py')
attempt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(attempt)


class AttemptTests(unittest.TestCase):
    def setUp(self): self.config = json.loads((ROOT / 'configs/a6_pilot.json').read_text())

    def test_development_records_cannot_be_confused_with_pilot_or_formal(self):
        self.assertTrue(hasattr(attempt, 'attempt_kind'))
        self.assertEqual(attempt.attempt_kind('DEVELOPMENT_BATCH'), 'DEVELOPMENT_ATTEMPT')
        self.assertEqual(attempt.attempt_kind('FROZEN_FOR_PILOT'), 'PILOT_ATTEMPT')
        self.assertEqual(attempt.attempt_kind('FROZEN_FOR_FORMAL'), 'FORMAL_ATTEMPT')
        with self.assertRaises(ValueError):
            attempt.attempt_kind('UNSPECIFIED')

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

    def test_proven_pre_task_subscriber_failure_is_not_a_negative_retrieval(self):
        log = '[FATAL] A6 configuration failed: A5 status subscriber did not connect before startup'
        physical = dict(status='FAIL', error='timed out waiting for LIFT')
        result = attempt.classify_outcome(physical, [], 2, None, adapter_log=log)
        self.assertEqual(result[:2], ('INVALID_TRIAL', None))
        self.assertEqual(result[2], 'platform_startup_status_subscriber_not_connected')

    def test_exit_two_alone_or_a_started_task_cannot_prove_platform_invalidity(self):
        log = 'A6 configuration failed: A5 status subscriber did not connect before startup'
        for events, code, text in (([], 2, ''), ([], 1, log),
                                   ([dict(state='A6_TASK_START')], 2, log),
                                   ([dict(state='FAILED', reason='method failed')], 2, log)):
            with self.subTest(events=events, code=code):
                result = attempt.classify_outcome(None, events, code, None, adapter_log=text)
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

    def test_checker_missing_initial_status_is_invalid_only_with_recorded_order_and_completion(self):
        physical = dict(status='FAIL', error='unexpected status TAKEOFF after []')
        events = [dict(state=s) for s in ('PREFLIGHT', 'ARMING', 'COMMAND_CONTROL', 'TAKEOFF', 'LIFT')]
        self.assertEqual(attempt.classify_outcome(physical, events, 0, 1)[:2],
                         ('INVALID_TRIAL', None))
        for incomplete, exit_code in ((events[1:], 0), (events[:-1], 0), (events, 1)):
            self.assertEqual(attempt.classify_outcome(physical, incomplete, exit_code, 1)[:2],
                             ('VALID_TRIAL', False))

    def test_actual_method_failure_keeps_priority_over_checker_startup_loss(self):
        physical = dict(status='FAIL', error='unexpected status TAKEOFF after []')
        events = [dict(state=s) for s in ('PREFLIGHT', 'ARMING', 'COMMAND_CONTROL', 'TAKEOFF', 'LIFT')]
        events.append(dict(state='FAILED', reason='actual execution failure'))
        self.assertEqual(attempt.classify_outcome(physical, events, 0, 1),
                         ('VALID_TRIAL', False, 'actual execution failure'))

    def test_different_order_error_is_not_automatically_a_platform_failure(self):
        physical = dict(status='FAIL', error="unexpected status LIFT after ['PREFLIGHT']")
        events = [dict(state=s) for s in ('PREFLIGHT', 'ARMING', 'COMMAND_CONTROL', 'TAKEOFF', 'LIFT')]
        self.assertEqual(attempt.classify_outcome(physical, events, 0, 1)[:2],
                         ('VALID_TRIAL', False))

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

    def test_checker_start_waits_for_final_adapter_publisher_marker(self):
        import tempfile
        from unittest.mock import patch
        self.assertTrue(hasattr(attempt, 'wait_for_adapter_ready'))
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory)/'adapter.log'
            log.write_text('initializing adapter\n')
            clock = [10.]
            def advance(_):
                clock[0] += .01
                log.write_text('initializing adapter\nA6_ADAPTER_READY\n')
            process = SimpleNamespace(poll=lambda: None)
            with patch.object(attempt.time, 'monotonic', side_effect=lambda: clock[0]), \
                    patch.object(attempt.time, 'sleep', side_effect=advance):
                attempt.wait_for_adapter_ready(process, log, deadline=20.)
            self.assertGreater(clock[0], 10.)

    def test_exited_adapter_does_not_start_a_checker(self):
        import tempfile
        self.assertTrue(hasattr(attempt, 'wait_for_adapter_ready'))
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, 'before.*ready'):
                attempt.wait_for_adapter_ready(SimpleNamespace(poll=lambda: 2),
                                               Path(directory)/'absent.log', deadline=float('inf'))

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

    def test_development_solver_contrast_changes_only_ode_solver(self):
        self.assertTrue(callable(getattr(attempt, 'solver_contrast_world', None)))
        source = '<sdf version="1.6"><world name="w"><include><uri>model://ground_plane</uri></include><physics name="default" type="ode"><max_step_size>0.001</max_step_size><ode><solver><type>quick</type><iters>50</iters></solver><constraints><erp>0.2</erp></constraints></ode></physics></world></sdf>'
        result = ET.fromstring(attempt.solver_contrast_world(source, 'world', 'DEVELOPMENT_BATCH'))
        typ = result.find('world/physics/ode/solver/type')
        self.assertEqual(typ.text, 'world')
        typ.text = 'quick'
        self.assertEqual(ET.tostring(result), ET.tostring(ET.fromstring(source)))
        with self.assertRaises(ValueError):
            attempt.solver_contrast_world(source, 'world', 'FROZEN_FOR_FORMAL')
        minimal = ET.fromstring(attempt.solver_contrast_world('<sdf><world name="w"/></sdf>', 'world', 'DEVELOPMENT_BATCH'))
        self.assertEqual(minimal.find('world/physics/ode/solver/type').text, 'world')


class GripperDiagnosticsTests(unittest.TestCase):
    def test_only_formal_status_adds_exactly_five_passive_topics(self):
        config = json.loads((ROOT / 'configs/a6_pilot2.json').read_text())
        before = json.loads(json.dumps(config))
        output = Path('/tmp/example')
        extra = [
            '/ground/gripper_controller/follow_joint_trajectory/goal',
            '/ground/gripper_controller/follow_joint_trajectory/status',
            '/ground/gripper_controller/follow_joint_trajectory/result',
            '/ground/gripper_controller/follow_joint_trajectory/cancel',
            '/ground/gripper_controller/state',
        ]
        for scope in (None, 'ground_handoff_to_end'):
            scoped = dict(config, diagnostic_image_scope=scope)
            original = attempt.diagnostic_command(scoped, output)
            for status in (None, 'FROZEN_FOR_PILOT', 'FROZEN_FOR_PILOT2'):
                with self.subTest(scope=scope, status=status):
                    legacy = dict(scoped, status=status)
                    if status is None:
                        legacy.pop('status')
                    self.assertEqual(attempt.diagnostic_command(legacy, output), original)
            formal = dict(scoped, status='FROZEN_FOR_FORMAL')
            with self.subTest(scope=scope, status=formal['status']):
                recorded = attempt.diagnostic_command(formal, output)
                self.assertEqual(recorded, original + extra)
                self.assertEqual(len(recorded[7:]), len(set(recorded[7:])))
                self.assertNotIn('/air_ground_pick_demo/status', recorded)
                self.assertIsNone(attempt.diagnostic_command(
                    dict(formal, record_diagnostics=False), output))
        self.assertEqual(config, before)


class ImageDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / 'configs/a6_pilot2.json').read_text())
        self.scoped = dict(self.config, status='FROZEN_FOR_FORMAL',
                           diagnostic_image_scope='ground_handoff_to_end')
        self.images = ['/ground/d435/color/image_raw', '/ground/d435/depth/image_raw']

    def test_scope_removes_only_two_images_and_preserves_default_command(self):
        output = Path('/tmp/example')
        original = attempt.diagnostic_command(self.config, output)
        for topic in self.images:
            self.assertIn(topic, original)
        scoped = attempt.diagnostic_command(
            dict(self.config, diagnostic_image_scope='ground_handoff_to_end'), output)
        self.assertEqual(scoped, [word for word in original if word not in self.images])
        self.assertNotIn('/air_ground_pick_demo/status', scoped)
        for topic in ('/ground/d435/color/camera_info', '/ground/d435/depth/camera_info'):
            self.assertIn(topic, scoped)

    def test_second_recorder_is_opt_in_lz4_and_exactly_two_native_images(self):
        self.assertTrue(hasattr(attempt, 'image_diagnostic_command'))
        output = Path('/tmp/example')
        self.assertIsNone(attempt.image_diagnostic_command(self.config, output))
        self.assertIsNone(attempt.image_diagnostic_command(
            dict(self.scoped, record_diagnostics=False), output))
        self.assertEqual(attempt.image_diagnostic_command(self.scoped, output),
                         ['rosbag', 'record', '--lz4', '--buffsize', '256', '-O',
                          str(output / 'diagnostics-images.bag'), *self.images])

    def test_poll_starts_once_for_each_actual_selection_and_keeps_adapter_exit(self):
        import tempfile
        from unittest.mock import patch
        self.assertTrue(hasattr(attempt, 'wait_for_adapter'))
        for state in ('A5_SELECTED', 'A6_RM4D_SELECTED'):
            with self.subTest(state=state), tempfile.TemporaryDirectory() as temporary:
                path = Path(temporary) / 'events.jsonl'
                clock, waits, selected = [10.], [], []
                event = dict(state=state, ros_time=40., wall_monotonic=10.2)
                def wait(timeout):
                    waits.append(timeout)
                    if len(waits) == 3:
                        return 7
                    clock[0] += timeout
                    path.write_text(json.dumps(event) + '\n' + json.dumps(event) + '\n{"state":')
                    raise attempt.subprocess.TimeoutExpired('adapter', timeout)
                with patch.object(attempt.time, 'monotonic', side_effect=lambda: clock[0]):
                    code = attempt.wait_for_adapter(SimpleNamespace(wait=wait), 12., path, selected.append)
                self.assertEqual(code, 7)
                self.assertEqual(selected, [event])
                self.assertEqual(waits, [.2, .2, .2])

    def test_scores_confirmation_failure_and_absent_setup_events_do_not_start_images(self):
        import tempfile
        self.assertTrue(hasattr(attempt, 'wait_for_adapter'))
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'events.jsonl'
            selected = []
            process = SimpleNamespace(wait=lambda timeout: 1)
            self.assertEqual(attempt.wait_for_adapter(process, float('inf'), path, selected.append), 1)
            path.write_text('\n'.join(json.dumps(dict(state=state, confirmed=True))
                                      for state in ('A6_SCORE', 'A5_ROUND', 'A5_CONFIRMED', 'FAILED')))
            self.assertEqual(attempt.wait_for_adapter(process, float('inf'), path, selected.append), 1)
            self.assertEqual(selected, [])

    def test_poll_keeps_absolute_deadline_and_default_wait_is_unchanged(self):
        from unittest.mock import patch
        self.assertTrue(hasattr(attempt, 'wait_for_adapter'))
        clock, waits = [10.], []
        def wait(timeout):
            waits.append(timeout)
            clock[0] += timeout
            raise attempt.subprocess.TimeoutExpired('adapter', timeout)
        with patch.object(attempt.time, 'monotonic', side_effect=lambda: clock[0]):
            with self.assertRaises(attempt.subprocess.TimeoutExpired):
                attempt.wait_for_adapter(SimpleNamespace(wait=wait), 10.5,
                                         Path('/tmp/nonexistent-a6-events'), lambda event: None)
        self.assertAlmostEqual(clock[0], 10.5)
        self.assertEqual(len(waits), 3)
        self.assertLessEqual(max(waits), .2)
        calls = []
        with patch.object(attempt.time, 'monotonic', return_value=10.):
            result = attempt.wait_for_adapter(
                SimpleNamespace(wait=lambda timeout: calls.append(timeout) or 0),
                20., Path('/tmp/nonexistent-a6-events'))
        self.assertEqual(result, 0)
        self.assertEqual(calls, [10.])

    def test_main_records_image_cleanup_and_start_failure_cannot_erase_method_failure(self):
        import contextlib
        import io
        import tempfile
        from unittest.mock import patch
        for selection, startup_error, setup in [('A5_SELECTED', False, False),
                                                ('A6_RM4D_SELECTED', False, False),
                                                ('A5_SELECTED', True, False),
                                                (None, False, False), (None, False, True)]:
            with self.subTest(selection=selection, startup_error=startup_error, setup=setup), \
                    tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                config_path, output = root / 'config.json', root / 'out'
                config_path.write_text(json.dumps(self.scoped))
                sim = root / 'fake-sim'
                source = sim / 'src/demos/air_ground_pick_demo/launch/air_ground_pick_demo.launch'
                source.parent.mkdir(parents=True)
                source.write_text('<launch><include file="/launch/air_ground_standalone.launch"/></launch>')
                started, stopped, waits = [], [], []
                def popen(command, stdout, **kwargs):
                    name = Path(stdout.name).stem
                    started.append(name)
                    if name == 'diagnostics-images' and startup_error:
                        raise OSError('image recorder unavailable')
                    process = SimpleNamespace(name=name, code=None)
                    process.poll = lambda: process.code
                    def wait(timeout):
                        if name == 'adapter':
                            waits.append(timeout)
                            path = output / 'data/events.jsonl'
                            if selection is not None and len(waits) == 1:
                                path.write_text(json.dumps(dict(state=selection, ros_time=20.,
                                                                 wall_monotonic=30.)) + '\n')
                                raise attempt.subprocess.TimeoutExpired('adapter', timeout)
                            with path.open('a') as stream:
                                stream.write(json.dumps(dict(state='FAILED', reason='actual method failure')) + '\n')
                        process.code = 1
                        return process.code
                    process.wait = wait
                    if name == 'adapter':
                        (output / 'data').mkdir()
                        (output / 'data/events.jsonl').write_text('')
                    return process
                def stop(process):
                    if process is not None:
                        stopped.append(process.name)
                        if process.code is None:
                            process.code = 0
                        if process.name in ('diagnostics', 'diagnostics-images'):
                            (output / (process.name + '.bag')).touch()
                with patch.dict(attempt.os.environ), \
                        patch.object(attempt.subprocess, 'Popen', side_effect=popen), \
                        patch.object(attempt, 'prepare_scene', return_value=10.), \
                        patch.object(attempt, 'wait_for_adapter_ready'), \
                        patch.object(attempt, 'stop_process', side_effect=stop), \
                        contextlib.redirect_stdout(io.StringIO()):
                    mode = ['--setup-scene', 'easy'] if setup else ['--slot', '1']
                    attempt.main(['--config', str(config_path), *mode,
                                  '--output-dir', str(output), '--sim-root', str(sim)])
                record = json.loads((output / 'attempt.json').read_text())
                self.assertEqual(record.get('diagnostic_image_scope'), 'ground_handoff_to_end')
                self.assertEqual(record['status'], 'VALID_TRIAL')
                if setup:
                    self.assertEqual(record['kind'], 'METHOD_INDEPENDENT_SETUP')
                    self.assertNotIn('retrieval_success', record)
                else:
                    self.assertEqual(record['classification_reason'], 'actual method failure')
                    self.assertIs(record['retrieval_success'], False)
                self.assertEqual(record['adapter_exit'], 1)
                self.assertLessEqual(max(waits), .2)
                self.assertIn('diagnostics', stopped)
                self.assertEqual(started.count('diagnostics-images'), int(selection is not None))
                if selection is None:
                    self.assertIsNone(record['diagnostic_image_start_trigger'])
                else:
                    self.assertEqual(record['diagnostic_image_start_trigger']['state'], selection)
                if startup_error:
                    self.assertIn('image recorder unavailable', record['diagnostic_image_error'])
                elif selection is not None:
                    self.assertIn('diagnostics-images', stopped)
                    self.assertEqual(record['diagnostic_image_exit'], 0)
                    self.assertTrue(record['diagnostic_image_bag_finalized'])
                else:
                    self.assertFalse(record['diagnostic_image_bag_finalized'])


if __name__ == '__main__': unittest.main()
