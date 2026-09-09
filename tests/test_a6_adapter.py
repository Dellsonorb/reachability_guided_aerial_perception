"""A6 wrapper behavior over real A5 code with only runtime boundaries replaced."""

import importlib.util
import ast
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SIM_DEMO = Path('/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation') / \
    'src/demos/air_ground_pick_demo/scripts/run_air_ground_pick_demo.py'


class AdapterTests(unittest.TestCase):
    def test_checker_connection_can_recover_from_registration_race_before_task(self):
        publisher = SimpleNamespace(get_num_connections=lambda: int(self.clock.wall >= 103.))
        def advance(seconds): self.clock.wall += seconds
        with patch.object(self.adapter.a5.time, 'sleep', side_effect=advance):
            self.adapter.wait_for_checker(publisher, self.ros, RuntimeError)
        self.assertGreaterEqual(self.clock.wall, 103.)
        self.assertLess(self.clock.wall, 103.02)

    def test_checker_connection_startup_wait_is_still_bounded(self):
        publisher = SimpleNamespace(get_num_connections=lambda: 0)
        def advance(seconds): self.clock.wall += seconds
        with patch.object(self.adapter.a5.time, 'sleep', side_effect=advance):
            with self.assertRaisesRegex(RuntimeError, 'before startup'):
                self.adapter.wait_for_checker(publisher, self.ros, RuntimeError)
        self.assertGreaterEqual(self.clock.wall, 110.)
        self.assertLess(self.clock.wall, 110.02)

    def setUp(self):
        path = ROOT / 'scripts/run_a6_sim.py'
        self.assertTrue(path.exists(), 'A6 ROS adapter module must exist')
        self.path_patch = patch.object(sys, 'path', [str(ROOT / 'scripts')] + sys.path)
        self.path_patch.start()
        self.addCleanup(self.path_patch.stop)
        spec = importlib.util.spec_from_file_location('a6_adapter_under_test', path)
        self.adapter = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.adapter)
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.output = Path(self.temporary.name)
        self.clock = SimpleNamespace(sim=10., wall=100.)
        self.actions, self.statuses, self.worker_calls, self.timers = [], [], [], []
        self.current_pose = [0., 0., 1.5, 0.]
        self.next_goals = [[2., 0., 1.5, 0.], [2., 2., 1.5, 0.]]
        self.raw_top = dict(candidate_id='raw-first', bunker_x=.123456789, bunker_y=-.321987654,
                            bunker_yaw=.987654321, final_score=.000321, reachability_score=.7)
        self.selected = dict(candidate_id='a5-exact', source_id=4, x=.1123456789,
                             y=-.319876543, yaw=.9887654321, relevance=.175)
        self.fail_observe = False
        owner = self

        class Stamp:
            def __init__(self, seconds=0.):
                self.seconds = seconds

            def to_sec(self):
                return self.seconds

            @staticmethod
            def now():
                return Stamp(owner.clock.sim)

        class Timer:
            def __init__(self, duration, callback):
                self.duration, self.callback, self.stopped = duration, callback, False
                owner.timers.append(self)

            def shutdown(self):
                self.stopped = True

        self.Stamp = Stamp
        self.ros = SimpleNamespace(Time=Stamp, Duration=lambda value: value, Timer=Timer,
                                   is_shutdown=lambda: False,
                                   Publisher=lambda *a, **k: SimpleNamespace(unregister=lambda: None),
                                   Subscriber=lambda *a, **k: SimpleNamespace(unregister=lambda: None))
        point_cloud = SimpleNamespace(read_points=lambda cloud, **kwargs: iter(cloud.points))
        modules = {'rospy': self.ros, 'tf2_ros': SimpleNamespace(TransformException=LookupError),
                   'sensor_msgs': SimpleNamespace(point_cloud2=point_cloud),
                   'sensor_msgs.msg': SimpleNamespace(PointCloud2=object),
                   'std_msgs.msg': SimpleNamespace(String=object)}
        self.module_patch = patch.dict(sys.modules, modules)
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)
        self.time_patch = patch.object(self.adapter.time, 'monotonic', side_effect=lambda: self.clock.wall)
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)

        class DemoBase:
            def __init__(self):
                self._lock = threading.RLock()
                self._status_pub = SimpleNamespace(unregister=lambda: None)
                self._status_topic = '/demo/status'
                self._map_frame, self._ground_base_frame = 'map', 'ground/base_link'
                self._view_position, self._view_yaw = [0., 0., 1.5], 0.
                self._takeoff_command, self._fly_to_command = 'TAKEOFF', 'FLY'
                self._hover_command, self._land_command = 'HOVER', 'LAND'
                self._flight_started, self._landed = False, False
                self._placement_mode = 'rm4d'
                self._flight_health_max_age = .5
                self._rm4d_grasp_id = 'grasp-live'
                for key in ('target_size', 'pregrasp_height', 'lift_height',
                            'finger_pad_lower_edge_offset', 'contact_overlap', 'surface_clearance'):
                    setattr(self, '_' + key, .1)
                self._tf_buffer = SimpleNamespace(lookup_transform=owner.lookup)

            def _publish_status(self, state, **details):
                owner.statuses.append((state, details))

            def _wait_preflight(self):
                self._publish_status('PREFLIGHT')

            def _view_pose(self):
                return SimpleNamespace(pose=SimpleNamespace(
                    position=SimpleNamespace(x=0., y=0., z=1.5),
                    orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))

            def _execute_flight(self, command, label, target=None):
                owner.actions.append((command, label))
                if command == 'FLY':
                    p = target.pose.position
                    owner.current_pose[:] = [p.x, p.y, p.z, 0.]

            def _wait_step(self):
                owner.clock.sim += .1
                owner.clock.wall += .05
                self._a5_cloud_callback(SimpleNamespace(
                    header=SimpleNamespace(stamp=Stamp.now(), frame_id='uav1/lidar_link'),
                    points=[(1., 0., -.2)]))
                for timer in owner.timers:
                    if not timer.stopped:
                        timer.callback(None)

            def _air_snapshot(self):
                return SimpleNamespace(velocity=[0., 0., 0.]), None, owner.clock.wall, None

            def _observe_from_air(self):
                return [2., 0., .1]

            def _ground_pose(self):
                return (3., -2.5, 3.14)

            def _request_land(self):
                self._execute_flight(self._land_command, 'landing')
                self._landed = True

            def _execute_pregrasp(self, target, continuation=None):
                if target == 'fail':
                    raise RuntimeError('pregrasp failure')
                return target

            def _execute_cartesian(self, target, label):
                if target == 'fail':
                    raise RuntimeError('cartesian failure')
                return target

            def _close_gripper(self):
                return 'closed'

            def _hold_grasp_confirmation(self):
                return 'retained'

            def _stop_ground(self, required=False):
                owner.actions.append(('STOP_GROUND', required))

            def run(self):
                try:
                    target = self._run_air_phase()
                    self.handoff = self._select_rm4d_candidate(target)
                    self._publish_status('LIFT', grasp_confirmed=True)
                    return True
                except RuntimeError as error:
                    self._publish_status('FAILED', reason=str(error))
                    return False
                finally:
                    if self._flight_started and not self._landed:
                        self._request_land()

        self.base = DemoBase
        self.demo = SimpleNamespace(DemoError=RuntimeError, AirGroundPickDemo=DemoBase,
                                    generate_top_down_grasp=lambda *args: SimpleNamespace(
                                        grasp=SimpleNamespace(position=(2., 0., .1), orientation=(0., 0., 0., 1.))))

    def inherited_method(self, name):
        """Execute the actual frozen body while replacing its imported ROS boundary."""
        tree = ast.parse(SIM_DEMO.read_text())
        base = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'AirGroundPickDemo')
        method = next(node for node in base.body if isinstance(node, ast.FunctionDef) and node.name == name)
        namespace = dict(DemoError=RuntimeError, ApproachError=RuntimeError, GraspError=RuntimeError,
                         moveit_commander=SimpleNamespace(MoveItCommanderException=RuntimeError),
                         tf2_ros=SimpleNamespace(TransformException=LookupError),
                         rospy=SimpleNamespace(logerr=lambda *a: None, logwarn=lambda *a: None))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(SIM_DEMO), 'exec'), namespace)
        return namespace[name]

    def lookup(self, target, source, stamp, timeout):
        import math
        xyz, yaw = (self.current_pose[:3], self.current_pose[3]) if source.startswith('uav1/') else ([3., -2.5, .36], 0.)
        return SimpleNamespace(header=SimpleNamespace(stamp=self.Stamp.now()), transform=SimpleNamespace(
            translation=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
            rotation=SimpleNamespace(x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2))))

    def options(self, method='ours'):
        return self.adapter.build_parser().parse_args([
            '--method', method, '--output-dir', str(self.output), '--core-python', sys.executable,
            '--rm4d-root', str(self.output), '--rm4d-config', str(self.output / 'config.yaml'),
            '--rm4d-map', str(self.output / 'map.npz')])

    def node(self, method='ours'):
        cls = self.adapter.build_adapter_class(self.demo, self.options(method))
        self.assertEqual(cls.__mro__[1].__name__, 'A5AirGroundPickDemo')
        node = cls()
        self.addCleanup(node._a6_close_evidence)
        return node

    def worker(self, python, script, src, request, output, label, timeout):
        self.worker_calls.append(dict(script=str(script), request=request, output=Path(output), label=label))
        if request['op'] == 'init':
            initial = self.output / 'initial.json'
            initial.write_text(json.dumps(dict(result=dict(candidates=[self.raw_top]))))
            return dict(ok=True, initial_file=str(initial), candidate_count=9)
        if self.fail_observe:
            raise self.adapter.WorkerError('deliberate worker failure')
        round_number = len(request['observations'])
        output = Path(request['output_dir'])
        output.mkdir(parents=True, exist_ok=True)
        response = dict(ok=True, round=round_number, stop_reason='VIEW_BUDGET_REACHED' if round_number == 3 else None,
                        next_viewpoint=None if round_number == 3 else self.next_goals[round_number - 1],
                        selected_candidate=self.selected, confirmed_candidate_count=round_number,
                        candidate_count=9, best_task_gain=1., best_generic_gain=2.)
        (output / 'decision.json').write_text(json.dumps(response))
        (output / 'ranking.json').write_text(json.dumps(dict(task_gain=1., generic_gain=2.)))
        return response

    def events(self):
        return [json.loads(line) for line in (self.output / 'events.jsonl').read_text().splitlines()]

    def test_parser_reuses_frozen_shared_defaults_and_enforces_budget(self):
        options = self.options()
        self.assertEqual(options.max_viewpoints, 3)
        self.assertEqual(options.cloud_window_s, 5.)
        self.assertEqual(options.cloud_timeout, 20.)
        self.assertEqual(options.facade_position_tolerance, .05)
        self.assertEqual(options.max_ground_travel, 3.)
        self.assertEqual(options.navigation_timeout, 120.)
        self.assertIsNone(options.view_position)
        options.max_viewpoints = 2
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.adapter.validate_options(self.adapter.build_parser(), options)
        self.assertTrue(hasattr(options, 'check_imports'))

    def test_observe_rounds_route_a6_worker_keep_initial_root_and_both_gains(self):
        node = self.node('generic')
        with patch.object(self.adapter, 'run_worker_request', side_effect=self.worker):
            self.assertTrue(node.run())
        requests = [row['request'] for row in self.worker_calls]
        self.assertEqual([row['op'] for row in requests], ['init', 'observe', 'observe', 'observe'])
        self.assertTrue(all(row['method'] == 'generic' for row in requests))
        self.assertTrue(all(row['script'].endswith('/scripts/a6_core_worker.py') for row in self.worker_calls))
        self.assertEqual(requests[0]['output_dir'], str(self.output))
        for index, request in enumerate(requests[1:], 1):
            expected = self.output / 'rounds' / ('round-%02d' % index)
            self.assertEqual(request['output_dir'], str(expected))
            self.assertEqual(request['initial_file'], str(self.output / 'initial.json'))
            self.assertEqual(len(request['observations']), index)
            self.assertEqual(json.loads((expected / 'ranking.json').read_text())['generic_gain'], 2.)
        self.assertEqual(node.handoff, ((self.selected['x'], self.selected['y'], self.selected['yaw']),
                                       self.selected['candidate_id'], self.selected['relevance'], 9))
        self.assertEqual(len(list(self.output.glob('observation_*.npz'))), 3)
        results = [row for row in self.events() if row['state'] == 'A6_ENV_RESULT']
        self.assertEqual([row['confirmed_candidate_count'] for row in results], [1, 2, 3])
        self.assertEqual(len(results), 3, 'first confirmed candidate must not end sensing')
        events = self.events()
        for result in results:
            decision = next(row for row in events if row['state'] == 'A5_DECISION' and row['round'] == result['round'])
            self.assertLess(events.index(result), events.index(decision))

    def test_fixed_nbvs_only_hover_at_current_pose_despite_stale_requested_pose(self):
        node = self.node('fixed')
        self.next_goals = [[2., 2., 1.5, 1.7], [3., 1., 1.5, -2.]]
        original = self.worker

        def drifting_worker(*args):
            response = original(*args)
            if args[3]['op'] == 'observe':
                self.current_pose[0] += .12
            return response

        with patch.object(self.adapter, 'run_worker_request', side_effect=drifting_worker):
            self.assertTrue(node.run())
        self.assertFalse(any(command == 'FLY' and label == 'A5 next viewpoint' for command, label in self.actions))
        sensing = [row for row in self.events() if row['state'] == 'A5_VIEWPOINT' and row.get('view_role') == 'sensing']
        self.assertEqual(len(sensing), 2)
        self.assertTrue(all(row['rescan'] for row in sensing))
        self.assertEqual([row['goal_map'][0] for row in sensing], [.12, .24])
        metrics = json.loads((self.output / 'metrics.json').read_text())
        self.assertGreater(metrics['paths']['uav_active']['observed_distance_lower_bound_m'], 0.)
        self.assertEqual(metrics['counts']['voted_windows'], 3)

    def assert_shared_execution_screen(self, method):
        node = self.node(method)
        node._execution_clearance = True
        first = dict(self.selected, confirmed=True)
        second = dict(first, candidate_id='exact-alternative', source_id=5, x=.2123456789)
        original = self.worker
        seen = []
        def worker(*args):
            result = original(*args)
            if args[3]['op'] == 'init':
                path = Path(result['initial_file'])
                data = json.loads(path.read_text())
                data['result']['evaluated_candidates'] = [dict(candidate_id=c['candidate_id'],
                                                               joint_configuration=[i]*6)
                                                         for i, c in enumerate((first, second))]
                path.write_text(json.dumps(data))
            else:
                result['assessments'] = [first, second]
            return result
        def preview(target, candidate, seed):
            seen.append((candidate['candidate_id'], seed))
            return dict(feasible=candidate['candidate_id'] == second['candidate_id'],
                        reason='whole-chain prediction')
        node._preview_ground_candidate = preview
        with patch.object(self.adapter, 'run_worker_request', side_effect=worker):
            self.assertTrue(node.run())
        self.assertEqual(seen, [(first['candidate_id'], [0]*6), (second['candidate_id'], [1]*6)])
        self.assertEqual(node.handoff[0], (second['x'], second['y'], second['yaw']))
        self.assertEqual(node.handoff[1], second['candidate_id'])
        self.assertEqual(node._execution_rm_seed, [1]*6)
        self.assertEqual(sum(e['state'] == 'A5_SELECTED' for e in self.events()), 1)
        self.assertEqual(sum(e['state'] == 'GROUND_EXECUTION_SCREEN' for e in self.events()), 2)
        self.assertEqual(len([r for r in self.worker_calls if r['request']['op'] == 'observe']), 3)

    def test_generic_uses_shared_bounded_exact_execution_screen(self):
        self.assert_shared_execution_screen('generic')

    def test_ours_uses_shared_bounded_exact_execution_screen(self):
        self.assert_shared_execution_screen('ours')

    def test_execution_rejection_preserves_successful_active_stage_and_original_confirmation(self):
        node = self.node('ours')
        node._execution_clearance = True
        original = self.worker
        def worker(*args):
            result = original(*args)
            if args[3]['op'] == 'init':
                path = Path(result['initial_file'])
                path.write_text(json.dumps(dict(result=dict(evaluated_candidates=[]))))
            else:
                result['assessments'] = [dict(self.selected, confirmed=True)]
            return result
        def preview(*args):
            self.clock.sim += 40.
            return dict(feasible=False, reason='closure blocked')
        node._preview_ground_candidate = preview
        with patch.object(self.adapter, 'run_worker_request', side_effect=worker):
            self.assertFalse(node.run())
        metrics = json.loads((self.output/'metrics.json').read_text())
        self.assertTrue(metrics['D_env'])
        self.assertFalse(metrics['D_exec'])
        self.assertEqual(metrics['terminal_failure_stage'], 'execution_screen')
        self.assertEqual(metrics['stages']['active']['status'], 'SUCCEEDED')
        self.assertEqual(metrics['stages']['execution_screen']['status'], 'FAILED')
        self.assertEqual(metrics['stages']['active']['duration_sim_s'], metrics['T_active_sim'])
        self.assertGreaterEqual(metrics['stages']['execution_screen']['duration_sim_s'], 40.)

    def test_rm4d_only_uses_initial_query_without_mid_windows_and_original_score(self):
        node = self.node('rm4d_only')
        with patch.object(self.adapter, 'run_worker_request', side_effect=self.worker):
            self.assertTrue(node.run())
        self.assertEqual([row['request']['op'] for row in self.worker_calls], ['init'])
        self.assertEqual(node.handoff, ((self.raw_top['bunker_x'], self.raw_top['bunker_y'], self.raw_top['bunker_yaw']),
                                       self.raw_top['candidate_id'], self.raw_top['final_score'], 1))
        self.assertEqual(node._a5_observations, [])
        states = [row[0] for row in self.statuses]
        for state in ('PREFLIGHT', 'TAKEOFF', 'AIR_VIEW', 'AIR_OBSERVE', 'LANDING'):
            self.assertIn(state, states)
        self.assertNotIn('A5_OBSERVATION', states)
        metrics = json.loads((self.output / 'metrics.json').read_text())
        self.assertEqual(metrics['T_active_sim'], 0.)
        self.assertEqual(metrics['counts']['capture_calls'], 0)

    def test_synchronous_status_and_failure_summary_preserve_original_payload(self):
        node = self.node()
        self.fail_observe = True
        with patch.object(self.adapter, 'run_worker_request', side_effect=self.worker):
            self.assertFalse(node.run())
        metrics = json.loads((self.output / 'metrics.json').read_text())
        self.assertEqual(metrics['terminal_status'], 'FAILED')
        self.assertEqual(metrics['terminal_failure_stage'], 'active')
        self.assertEqual(metrics['counts']['completed_windows'], 1)
        self.assertEqual(metrics['counts']['voted_windows'], 0)
        self.assertEqual(metrics['counts']['capture_calls'], 1)
        failed = next(row for row in self.events() if row['state'] == 'FAILED')
        self.assertEqual(failed['reason'], next(details['reason'] for state, details in self.statuses if state == 'FAILED'))
        self.assertIn('wall_monotonic', failed)
        self.assertIn('ros_time', failed)
        self.assertTrue(all(timer.stopped for timer in self.timers))

    def test_tf_samples_use_public_frames_and_strict_freshness(self):
        node = self.node()
        calls = []

        def lookup(target, source, stamp, timeout):
            calls.append((target, source, timeout))
            result = self.lookup(target, source, stamp, timeout)
            if source == 'ground/base_link':
                result.header.stamp = self.Stamp(self.clock.sim - .6)
            return result

        node._tf_buffer.lookup_transform = lookup
        node._a6_sample_trajectory(None)
        sample = json.loads((self.output / 'trajectory.jsonl').read_text().splitlines()[-1])
        self.assertEqual(calls, [('map', 'uav1/base_link', 0.), ('map', 'ground/base_link', 0.)])
        self.assertEqual(sample['uav']['xyz'], [0., 0., 1.5])
        self.assertIsNone(sample['ground']['xyz'])
        self.assertIn('stale', sample['ground']['missing_reason'])
        node._a6_close_evidence()

    def test_refined_pregrasp_only_marks_exec_ready_after_success(self):
        # The refined implementation is the existing manipulation boundary, not
        # a second A6 planner. Ordinary initial approach must never mark D_exec.
        refine = SimpleNamespace(execute_refined_pregrasp=lambda node, target, grasp, error: (
            self.base._execute_pregrasp(node, target, grasp)))
        with patch.dict(sys.modules, {'a5_manipulation': refine}):
            node = self.node()
        self.assertEqual(node._execute_pregrasp('initial'), 'initial')
        self.assertFalse(any(row['state'] == 'A6_EXEC_READY' for row in self.events()))
        node._a5_refined_grasp = 'refined'
        with self.assertRaisesRegex(RuntimeError, 'pregrasp failure'):
            node._execute_pregrasp('fail')
        self.assertFalse(any(row['state'] == 'A6_EXEC_READY' for row in self.events()))
        self.assertEqual(node._execute_pregrasp('success'), 'success')
        self.assertEqual(sum(row['state'] == 'A6_EXEC_READY' for row in self.events()), 1)
        node._a6_close_evidence()

    def test_stage_wrappers_rethrow_failures_and_record_success_intervals(self):
        node = self.node()
        self.assertEqual(node._execute_cartesian('ok', 'grasp approach'), 'ok')
        self.assertEqual(node._close_gripper(), 'closed')
        self.assertEqual(node._hold_grasp_confirmation(), 'retained')
        with self.assertRaisesRegex(RuntimeError, 'cartesian failure'):
            node._execute_cartesian('fail', 'lift')
        rows = [row for row in self.events() if row['state'] in ('A6_STAGE_END', 'A6_STAGE_FAILED')]
        self.assertEqual([(row['stage'], row['state']) for row in rows],
                         [('descend', 'A6_STAGE_END'), ('close', 'A6_STAGE_END'),
                          ('retention', 'A6_STAGE_END'), ('lift', 'A6_STAGE_FAILED')])
        node._a6_close_evidence()

    def test_return_tf_failure_records_return_stage_before_pose_lookup(self):
        node = self.node()
        node._tf_buffer.lookup_transform = lambda *args: (_ for _ in ()).throw(LookupError('TF missing'))
        with self.assertRaises(LookupError):
            node._a5_fly_and_hover(self.current_pose, 'A5 return to clear landing location')
        rows = self.events()
        self.assertTrue(any(row['state'] == 'A6_STAGE_FAILED' and row['stage'] == 'return' for row in rows))
        node._a6_close_evidence()

    def test_landed_stamp_only_written_after_successful_landing(self):
        node = self.node()
        with patch.object(self.base, '_request_land', side_effect=RuntimeError('landing failed')):
            with self.assertRaises(RuntimeError):
                node._request_land()
        self.assertFalse(any(row['state'] == 'A6_LANDED' for row in self.events()))
        self.assertTrue(any(row['state'] == 'A6_STAGE_FAILED' and row['stage'] == 'landing'
                            for row in self.events()))
        node._a6_close_evidence()

    def test_unhandled_runtime_error_still_has_primary_failure_and_final_summary(self):
        node = self.node()
        with patch.object(self.base, 'run', side_effect=ValueError('unexpected boundary failure')):
            with self.assertRaises(ValueError):
                node.run()
        summary = json.loads((self.output / 'metrics.json').read_text())
        self.assertEqual(summary['terminal_status'], 'FAILED')
        self.assertIn('unexpected boundary failure', summary['terminal_failure_reason'])

    def test_check_imports_loads_inherited_boundary_without_node_or_files(self):
        config, runtime = self.output / 'config.yaml', self.output / 'sim'
        config.write_text('{}')
        demo_file = runtime / self.adapter.a5.DEMO_RELATIVE / 'scripts/run_air_ground_pick_demo.py'
        demo_file.parent.mkdir(parents=True)
        demo_file.write_text('class DemoError(RuntimeError): pass\nclass AirGroundPickDemo: pass\n')
        demo_config = runtime / self.adapter.a5.DEMO_RELATIVE / 'config/demo.yaml'
        demo_config.parent.mkdir()
        demo_config.write_text('{}')
        map_file = self.output / 'map.npz'
        map_file.touch()
        self.ros.myargv = lambda argv: argv
        self.ros.init_node = lambda *a, **k: self.fail('check-imports must not initialize ROS')
        with patch.dict(sys.modules, {'moveit_commander': SimpleNamespace(), 'yaml': SimpleNamespace()}):
            self.assertEqual(self.adapter.main([
                'run_a6_sim.py', '--check-imports', '--method', 'ours', '--sim-root', str(runtime),
                '--output-dir', str(self.output / 'untouched'), '--core-python', sys.executable,
                '--rm4d-root', str(self.output), '--rm4d-config', str(config), '--rm4d-map', str(map_file)]), 0)
        self.assertFalse((self.output / 'untouched').exists())

    def test_unexpected_air_error_is_terminal_before_actual_inherited_finally_cleanup(self):
        node = self.node()
        failure_time = []

        def fail_worker(*args):
            if args[3]['op'] == 'observe':
                failure_time.append(self.clock.sim)
                raise NameError('unexpected active failure')
            return self.worker(*args)

        def delayed_land(owner):
            self.clock.sim += 8.
            self.clock.wall += 10.
            owner._landed = True

        with patch.object(self.base, 'run', self.inherited_method('run')), \
                patch.object(self.base, '_safe_land', self.inherited_method('_safe_land'), create=True), \
                patch.object(self.base, '_request_land', delayed_land), \
                patch.object(self.adapter, 'run_worker_request', side_effect=fail_worker):
            with self.assertRaisesRegex(NameError, 'unexpected active failure'):
                node.run()
        metrics = json.loads((self.output / 'metrics.json').read_text())
        capture = next(row['ros_time'] for row in self.events() if row['state'] == 'A6_CAPTURE_START')
        self.assertAlmostEqual(metrics['T_task_sim'], failure_time[0] - 10.)
        self.assertAlmostEqual(metrics['T_active_sim'], failure_time[0] - capture)
        self.assertEqual(metrics['task_end_sim'], failure_time[0])
        self.assertEqual(metrics['landed_sim'], failure_time[0] + 8.)
        self.assertEqual(metrics['terminal_failure_stage'], 'active')
        self.assertEqual(metrics['stages']['landing']['status'], 'NOT_REACHED')
        self.assertEqual([required for command, required in self.actions if command == 'STOP_GROUND'], [False])
        rows = self.events()
        failed = next(index for index, row in enumerate(rows) if row['state'] == 'FAILED')
        cleanup_landing = next(index for index, row in enumerate(rows)
                               if row['state'] == 'A6_STAGE_START' and row['stage'] == 'landing')
        self.assertLess(failed, cleanup_landing)
        self.assertEqual(sum(row['state'] == 'FAILED' for row in rows), 1)

    def test_ground_entry_tf_failure_preserves_successful_landing(self):
        node = self.node()

        def ground_pose(owner):
            if owner._landed:
                raise RuntimeError('Ground entry TF missing')
            return (3., -2.5, 3.14)

        with patch.object(self.base, 'run', self.inherited_method('run')), \
                patch.object(self.base, '_safe_land', self.inherited_method('_safe_land'), create=True), \
                patch.object(self.base, '_approach_ground', self.inherited_method('_approach_ground'), create=True), \
                patch.object(self.base, '_approach_ground_rm4d', self.inherited_method('_approach_ground_rm4d'), create=True), \
                patch.object(self.base, '_ground_pose', ground_pose), \
                patch.object(self.adapter, 'run_worker_request', side_effect=self.worker):
            self.assertFalse(node.run())
        metrics = json.loads((self.output / 'metrics.json').read_text())
        self.assertEqual(metrics['terminal_failure_stage'], 'ground_navigation')
        self.assertEqual(metrics['stages']['landing']['status'], 'SUCCEEDED')
        self.assertEqual(metrics['stages']['ground_navigation']['status'], 'FAILED')
        self.assertEqual(metrics['stages']['ground_refine']['status'], 'NOT_REACHED')
        self.assertNotIn('GROUND_APPROACH', [state for state, details in self.statuses])

    def test_refined_pregrasp_preparation_failure_follows_completed_refine(self):
        node = self.node()
        node._a6_event('A6_TASK_START')
        node._publish_status('GROUND_OBSERVE')
        self.clock.sim += 2.
        node._publish_status('GROUND_REFINED', target_map=[2., 0., .1])
        self.clock.sim += 1.
        group = SimpleNamespace(get_planning_frame=lambda: 'ground/aubo_i5_base_link')
        with patch.object(self.base, '_initialize_moveit', return_value=group, create=True), \
                patch.object(self.base, '_pose_message', return_value=SimpleNamespace(), create=True), \
                patch.object(node, '_transform_pose', side_effect=LookupError('pregrasp preparation TF missing')):
            with self.assertRaisesRegex(LookupError, 'pregrasp preparation TF missing'):
                node._pick_and_lift(SimpleNamespace(header=SimpleNamespace(stamp=self.Stamp.now())), [2., 0., .1])
        node._publish_status('FAILED', reason='pregrasp preparation TF missing')
        metrics = self.adapter.summarize_metrics(self.events(), [], method='ours')
        self.assertEqual(metrics['terminal_failure_stage'], 'refined_pregrasp')
        self.assertEqual(metrics['stages']['ground_refine']['status'], 'SUCCEEDED')
        self.assertEqual(metrics['stages']['ground_refine']['duration_sim_s'], 2.)
        self.assertEqual(metrics['stages']['refined_pregrasp']['status'], 'FAILED')
        self.assertFalse(metrics['D_exec'])


if __name__ == '__main__':
    unittest.main()
