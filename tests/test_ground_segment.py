"""Ground replay is a conditioned diagnostic, not a new discovery trial."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))


class GroundSegmentTests(unittest.TestCase):
    def setUp(self):
        import run_ground_sim
        self.module = run_ground_sim
        self.events = [dict(state='AIR_HANDOFF', target_map=[1., 2., .0575, .2]),
                       dict(state='A5_SELECTED',
                           candidate_id='exact-1', x=2., y=3., yaw=.4,
                           relevance=.6, confirmed=True, operational=dict(blocked=False,
                           ground_supported=True))]

    def test_preserves_exact_runtime_pose_without_reading_setup_target(self):
        selected, target = self.module.extract_recorded_handoff(self.events)
        self.assertEqual(target, (1., 2., .0575, .2))
        self.assertEqual(selected['x'], self.events[1]['x'])

    def test_unconfirmed_duplicate_or_nonfinite_is_not_valid_replay(self):
        for mutation in ('unconfirmed', 'duplicate', 'nan', 'missing'):
            events = copy.deepcopy(self.events)
            if mutation == 'unconfirmed': events[1]['confirmed'] = False
            if mutation == 'duplicate': events.append(events[1])
            if mutation == 'nan': events[0]['target_map'][0] = float('nan')
            if mutation == 'missing': events.pop(0)
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                self.module.extract_recorded_handoff(events)

    def test_navigation_only_does_not_claim_retrieval(self):
        import run_a6_attempt
        result = run_a6_attempt.navigation_outcome([dict(state='GROUND_STOPPED')], 0)
        self.assertIsNone(result['retrieval_success'])
        self.assertTrue(result['navigation_success'])
        self.assertFalse(run_a6_attempt.navigation_outcome([], 1)['navigation_success'])

    def test_replay_requires_identical_setup(self):
        import run_a6_attempt
        run_a6_attempt.validate_replay_scene({'scene_spec': {'seed': 1}}, {'seed': 1})
        with self.assertRaises(ValueError):
            run_a6_attempt.validate_replay_scene({'scene_spec': {'seed': 1}}, {'seed': 2})

    def test_camera_segment_spawn_uses_only_archived_exact_candidate(self):
        import run_a6_attempt
        scene = dict(target_xy=[1, 2], bunker_xy=[3, 4], bunker_z=.36, bunker_yaw=0)
        changed = run_a6_attempt.ground_candidate_setup(scene, self.events[1])
        self.assertEqual(changed['bunker_xy'], [2., 3.])
        self.assertEqual(changed['bunker_yaw'], .4)
        self.assertEqual(changed['target_xy'], scene['target_xy'])
        self.assertEqual(scene['bunker_xy'], [3, 4])

    def test_post_failure_ik_probe_only_reports_collision_and_never_executes(self):
        from types import SimpleNamespace as N
        from unittest.mock import Mock
        request = N(ik_request=N())
        ros = N(Duration=lambda seconds: seconds)
        state = N(joint_state='diagnostic-solution')
        compute = Mock(return_value=N(error_code=N(val=1), solution=state))
        validity = Mock(return_value=N(valid=False, contacts=[N(contact_body_1='base', contact_body_2='finger')]))
        owner = N(_ground_last_grasp='runtime-grasp', _move_group_name='arm',
                  _end_effector_link='tcp', _robot_state_from_joint_feedback=lambda:'measured',
                  _publish_status=Mock())
        self.module.diagnose_grasp_failure(owner, ros, lambda:request, compute, validity)
        self.assertFalse(request.ik_request.avoid_collisions)
        validity.assert_called_once_with(robot_state=state, group_name='arm')
        payload = owner._publish_status.call_args.kwargs
        self.assertFalse(payload['collision_free'])
        self.assertFalse(payload['executed'])


class GroundReportingTests(unittest.TestCase):
    def setUp(self):
        import run_ground_sim
        import run_a6_attempt
        self.ground, self.attempt = run_ground_sim, run_a6_attempt

    def reduce(self, events, samples=(), **options):
        self.assertTrue(callable(getattr(self.ground, 'summarize_ground_metrics', None)),
                        'Ground replay needs its own real-boundary metric reduction')
        return self.ground.summarize_ground_metrics(events, samples, 'ours', **options)

    @staticmethod
    def event(state, sim, **details):
        return dict(state=state, ros_time=sim, wall_monotonic=100. + sim, **details)

    @staticmethod
    def sample(sim, x):
        return dict(ros_time=sim, wall_monotonic=100. + sim,
                    ground=dict(xyz=[x, 0., 0.], yaw=0., stamp_s=sim))

    def events(self):
        return [self.event('GROUND_REPLAY_START', 10., navigation_only=False),
                self.event('A6_STAGE_START', 10., stage='ground_navigation'),
                self.event('GROUND_STOPPED', 10.1),
                self.event('A6_STAGE_START', 10.1, stage='refined_pregrasp'),
                self.event('A6_STAGE_END', 10.2, stage='refined_pregrasp'),
                self.event('A6_EXEC_READY', 10.2),
                self.event('A6_STAGE_START', 10.2, stage='lift'),
                self.event('A6_STAGE_FAILED', 10.4, stage='lift', reason='wrist velocity'),
                self.event('FAILED', 10.4, reason='wrist velocity'),
                self.event('GROUND_POST_FAILURE_IK_DIAGNOSTIC', 11., executed=False),
                self.event('GROUND_REPLAY_END', 12., success=False)]

    def test_ground_boundaries_exclude_post_failure_probe_and_preserve_inputs(self):
        events = self.events()
        samples = [self.sample(10. + index / 10., index) for index in range(5)]
        samples += [self.sample(11., 100.), self.sample(12., 200.)]
        before = copy.deepcopy((events, samples))
        result = self.reduce(events, samples, conditioned_on_arrival=True,
                             outcome=dict(status='VALID_TRIAL', retrieval_success=False))
        self.assertEqual('GROUND_REPLAY_METRICS', result['kind'])
        self.assertTrue(result['conditioned_on_archived_confirmation'])
        self.assertTrue(result['conditioned_on_arrival'])
        self.assertIsNone(result['navigation_success'])
        self.assertEqual(result['arrival_assessment'], 'NOT_ASSESSED_CONDITIONED_ON_ARRIVAL')
        self.assertAlmostEqual(result['T_task_sim'], .4)
        self.assertAlmostEqual(result['T_exec_ready_sim'], .2)
        self.assertEqual(result['replay_end_sim'], 12.)
        self.assertTrue(result['D_exec'])
        self.assertIsNone(result['D_env'])
        self.assertFalse(result['retrieval_success'])
        self.assertEqual(result['terminal_failure_stage'], 'lift')
        self.assertEqual(result['terminal_failure_reason'], 'wrist velocity')
        self.assertEqual(result['stages']['retention']['status'], 'NOT_REACHED')
        self.assertEqual(result['paths']['ground_total']['distance_m'], 4.)
        self.assertFalse(any(key.startswith('uav') for key in result['paths']))
        self.assertNotIn('takeoff', result['stages'])
        self.assertEqual((events, samples), before)

    def test_navigation_only_has_unassessed_execution_and_retrieval(self):
        events = [self.event('GROUND_REPLAY_START', 1., navigation_only=True),
                  self.event('GROUND_APPROACH', 1.),
                  self.event('GROUND_ARRIVAL_MEASURED', 1.9,
                             actual_pose_map=[.04, 0., .02], goal_pose_map=[0., 0., 0.]),
                  self.event('GROUND_STOPPED', 2.),
                  self.event('GROUND_REPLAY_END', 3., success=True)]
        result = self.reduce(events, outcome=dict(status='VALID_TRIAL', retrieval_success=True))
        self.assertTrue(result['navigation_success'])
        self.assertIsNone(result['D_exec'])
        self.assertIsNone(result['retrieval_success'])
        self.assertEqual(result['retrieval_assessment'], 'NOT_ASSESSED_NAVIGATION_ONLY')
        self.assertEqual(result['T_task_sim'], 2.)
        self.assertEqual(result['stages']['ground_navigation']['duration_sim_s'], 1.)

    def test_historical_latch_completion_is_not_accurate_arrival_validation(self):
        events = [self.event('GROUND_REPLAY_START', 1., navigation_only=False),
                  self.event('GROUND_STOPPED', 2.), self.event('FAILED', 3., reason='lift')]
        result = self.reduce(events)
        self.assertTrue(result['ground_stopped_recorded'])
        self.assertIsNone(result['navigation_success'])
        self.assertEqual(result['arrival_assessment'], 'UNAVAILABLE_FINAL_POSE_EVENT')

    def test_explicit_final_arrival_pose_error_is_projected_without_new_acceptance(self):
        events = [self.event('GROUND_REPLAY_START', 1., navigation_only=False),
                  self.event('GROUND_ARRIVAL_MEASURED', 2.,
                             actual_pose_map=[.03, .04, .07], goal_pose_map=[0., 0., 0.]),
                  self.event('GROUND_STOPPED', 2.01), self.event('FAILED', 3., reason='lift')]
        result = self.reduce(events)
        self.assertTrue(result['navigation_success'])
        self.assertEqual(result['arrival_assessment'], 'RECORDED_FINAL_POSE_AND_STOP')
        self.assertAlmostEqual(result['arrival_xy_error_m'], .05)
        self.assertAlmostEqual(result['arrival_yaw_error_rad'], .07)

    def test_lift_status_alone_never_claims_physical_retrieval(self):
        events = [self.event('GROUND_REPLAY_START', 1., navigation_only=False),
                  self.event('A6_EXEC_READY', 2.), self.event('LIFT', 3.),
                  self.event('GROUND_REPLAY_END', 4., success=True)]
        self.assertIsNone(self.reduce(events)['retrieval_success'])
        self.assertTrue(self.reduce(events, outcome=dict(
            status='VALID_TRIAL', retrieval_success=True))['retrieval_success'])

    def test_no_replay_start_is_unassessed_and_has_no_task_duration(self):
        result = self.reduce([self.event('GROUND_REPLAY_STARTUP_FAILED', 1., reason='checker'),
                              self.event('GROUND_REPLAY_END', 2., success=False)])
        self.assertFalse(result['task_started'])
        self.assertIsNone(result['T_task_sim'])
        self.assertIsNone(result['D_exec'])
        self.assertIsNone(result['navigation_success'])
        self.assertIsNone(result['retrieval_success'])

    def test_clock_reset_and_missing_path_remain_explicit(self):
        events = [self.event('GROUND_REPLAY_START', 10., navigation_only=False),
                  dict(self.event('FAILED', 2.), wall_monotonic=120.),
                  dict(self.event('GROUND_REPLAY_END', 3., success=False), wall_monotonic=121.)]
        result = self.reduce(events)
        self.assertTrue(result['clock_reset_detected'])
        self.assertIsNone(result['T_task_sim'])
        self.assertIsNone(result['paths']['ground_total']['distance_m'])
        self.assertFalse(result['paths']['ground_total']['complete'])

    def test_incomplete_replay_does_not_invent_a_terminal_time(self):
        result = self.reduce([self.event('GROUND_REPLAY_START', 1., navigation_only=False),
                              self.event('GROUND_APPROACH', 2.)])
        self.assertIsNone(result['task_end_sim'])
        self.assertIsNone(result['T_task_sim'])
        self.assertEqual(result['stages']['ground_navigation']['status'], 'IN_PROGRESS')

    def classify(self, events, **options):
        self.assertTrue(callable(getattr(self.attempt, 'ground_replay_outcome', None)),
                        'Ground startup must use its real replay-start boundary')
        return self.attempt.ground_replay_outcome(None, events, 1, None, **options)

    def test_failure_before_replay_start_is_invalid_with_no_negative_outcome(self):
        for events in ([], [dict(state='FAILED', reason='checker did not connect')],
                       [dict(state='GROUND_REPLAY_STARTUP_FAILED', reason='checker did not connect')]):
            for navigation_only in (False, True):
                with self.subTest(events=events, navigation_only=navigation_only):
                    result = self.classify(events, navigation_only=navigation_only)
                    self.assertEqual(result['status'], 'INVALID_TRIAL')
                    self.assertFalse(result['task_started'])
                    self.assertIsNone(result['retrieval_success'])
                    self.assertIsNone(result['navigation_success'])

    def test_failure_after_replay_start_keeps_real_method_failure(self):
        result = self.classify([dict(state='GROUND_REPLAY_START'),
                                dict(state='FAILED', reason='navigation timeout')])
        self.assertEqual(result['status'], 'VALID_TRIAL')
        self.assertTrue(result['task_started'])
        self.assertFalse(result['retrieval_success'])
        self.assertEqual(result['classification_reason'], 'navigation timeout')

    def test_ground_physical_checker_outcome_and_navigation_scope_are_preserved(self):
        self.assertTrue(callable(getattr(self.attempt, 'ground_replay_outcome', None)))
        events = [dict(state='GROUND_REPLAY_START'), dict(state='GROUND_STOPPED'), dict(state='LIFT')]
        result = self.attempt.ground_replay_outcome(dict(status='CHECKS_PASS'), events, 0, 0)
        self.assertTrue(result['retrieval_success'])
        self.assertTrue(result['navigation_success'])
        result = self.attempt.ground_replay_outcome(None, events, 0, None, navigation_only=True)
        self.assertTrue(result['navigation_success'])
        self.assertIsNone(result['retrieval_success'])

    def test_conditioned_arrival_cannot_claim_navigation_in_attempt_outcome(self):
        self.assertTrue(callable(getattr(self.attempt, 'ground_replay_outcome', None)))
        import inspect
        self.assertIn('conditioned_on_arrival', inspect.signature(self.attempt.ground_replay_outcome).parameters)
        result = self.attempt.ground_replay_outcome(
            None, [dict(state='GROUND_REPLAY_START'), dict(state='GROUND_STOPPED'),
                   dict(state='FAILED', reason='IK')], 1, 1, conditioned_on_arrival=True)
        self.assertIsNone(result['navigation_success'])
        self.assertFalse(result['retrieval_success'])

    def test_offline_materialization_preserves_every_original_and_refuses_overwrite(self):
        self.assertTrue(callable(getattr(self.ground, 'materialize_ground_metrics', None)),
                        'completed Ground attempts need exclusive derived metric creation')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'data').mkdir()
            attempt = dict(kind='GROUND_CAMERA_MANIPULATION_DIAGNOSTIC', method='ours',
                           finish_wall=1., status='VALID_TRIAL', retrieval_success=False,
                           conditioned_on_arrival=True)
            (root / 'attempt.json').write_text(json.dumps(attempt))
            (root / 'data/events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in self.events()))
            (root / 'data/trajectory.jsonl').write_text(json.dumps(self.sample(10., 0))+'\n')
            before = {path: path.read_bytes() for path in root.rglob('*') if path.is_file()}
            path = self.ground.materialize_ground_metrics(root)
            result = json.loads(path.read_text())
            self.assertTrue(result['D_exec'])
            self.assertFalse(result['retrieval_success'])
            self.assertTrue(result['conditioned_on_arrival'])
            self.assertEqual({path: path.read_bytes() for path in before}, before)
            with self.assertRaises(FileExistsError):
                self.ground.materialize_ground_metrics(root)

    def test_offline_materialization_rejects_running_and_non_ground_attempts(self):
        self.assertTrue(callable(getattr(self.ground, 'materialize_ground_metrics', None)))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for attempt in (dict(kind='GROUND_SEGMENT_DIAGNOSTIC'),
                            dict(kind='FORMAL_ATTEMPT', finish_wall=1.)):
                (root / 'attempt.json').write_text(json.dumps(attempt))
                with self.assertRaises(ValueError):
                    self.ground.materialize_ground_metrics(root)


if __name__ == '__main__': unittest.main()
