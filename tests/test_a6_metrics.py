"""Simulation-clock accounting independent of ROS and simulator processes."""

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


def event(state, sim, wall=100., **details):
    return dict(state=state, ros_time=sim, wall_monotonic=wall, **details)


def sample(sim, xyz, ground=None):
    body = None if xyz is None else dict(xyz=list(xyz), yaw=0., stamp_s=sim)
    return dict(ros_time=sim, uav=body, ground=body if ground is None else ground)


class MetricsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = ROOT / 'scripts/a6_metrics.py'
        if path.exists():
            spec = importlib.util.spec_from_file_location('a6_metrics_under_test', path)
            cls.metrics = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.metrics)
        else:
            cls.metrics = None

    def setUp(self):
        self.assertIsNotNone(self.metrics, 'A6 simulation-time metrics module must exist')

    def summarize(self, events, samples=(), method='ours'):
        return self.metrics.summarize_metrics(events, samples, method=method)

    def test_efficiency_uses_simulation_time_and_stops_before_cleanup(self):
        result = self.summarize([
            event('A6_TASK_START', 10., 100.), event('A6_CAPTURE_START', 11., 101.),
            event('A6_ACTIVE_STOP', 16., 119.), event('A6_EXEC_READY', 22., 135.),
            event('LIFT', 25., 140.), event('A6_TASK_END', 29., 170.)])
        self.assertEqual(result['T_task_sim'], 15.)
        self.assertEqual(result['T_active_sim'], 5.)
        self.assertEqual(result['T_exec_ready_sim'], 12.)
        self.assertTrue(result['D_exec'])
        self.assertIsNone(result['physical_success'])

    def test_raw_consecutive_paths_and_wrapped_yaw(self):
        samples = [sample(10., (0, 0, 0)), sample(10.1, (3, 0, 0)),
                   sample(10.2, (3, 4, 0))]
        samples[0]['uav']['yaw'] = 3.1
        samples[1]['uav']['yaw'] = -3.1
        samples[2]['uav']['yaw'] = -3.1
        path = self.metrics.path_metrics(samples, 'uav', 10., 10.2, dimensions=3)
        self.assertEqual(path['distance_m'], 7.)
        self.assertTrue(path['complete'])
        self.assertAlmostEqual(path['yaw_travel_rad'], 2 * 3.141592653589793 - 6.2)

    def test_missing_or_long_gaps_make_full_distance_null_without_bridging(self):
        for samples in ([sample(10., (0, 0, 0)), sample(10.1, None),
                         sample(10.2, (3, 0, 0)), sample(10.3, (3, 4, 0))],
                        [sample(10., (0, 0, 0)), sample(10.4, (3, 0, 0)),
                         sample(10.5, (3, 4, 0))]):
            with self.subTest(samples=samples):
                path = self.metrics.path_metrics(samples, 'uav', 10., samples[-1]['ros_time'])
                self.assertFalse(path['complete'])
                self.assertIsNone(path['distance_m'])
                self.assertEqual(path['observed_distance_lower_bound_m'], 4.)

    def test_missing_boundary_samples_are_incomplete(self):
        path = self.metrics.path_metrics([sample(10.5, (0, 0, 0))], 'ground', 10., 11.)
        self.assertIsNone(path['distance_m'])
        self.assertFalse(path['complete'])

    def test_clock_reset_never_uses_wall_time_or_bridges_paths(self):
        result = self.summarize([event('A6_TASK_START', 10., 100.),
                                 event('A6_CAPTURE_START', 11., 110.),
                                 event('FAILED', 2., 140.), event('A6_TASK_END', 3., 150.)])
        self.assertTrue(result['clock_reset_detected'])
        self.assertIsNone(result['T_task_sim'])
        self.assertIsNone(result['T_active_sim'])

    def test_earliest_failure_stage_survives_cleanup_and_downstream_not_reached(self):
        result = self.summarize([event('A6_TASK_START', 1), event('GROUND_APPROACH', 2),
                                 event('GROUND_STOPPED', 3), event('GROUND_OBSERVE', 4),
                                 event('A6_STAGE_START', 5, stage='refined_pregrasp'),
                                 event('A6_STAGE_FAILED', 6, stage='refined_pregrasp', reason='no continuation'),
                                 event('FAILED', 6, reason='no continuation'),
                                 event('A6_STAGE_FAILED', 7, stage='landing', reason='cleanup'),
                                 event('A6_TASK_END', 8)])
        self.assertEqual(result['terminal_failure_stage'], 'refined_pregrasp')
        self.assertEqual(result['terminal_failure_reason'], 'no continuation')
        self.assertEqual(result['stages']['descend']['status'], 'NOT_REACHED')
        self.assertEqual(result['stages']['ground_navigation']['duration_sim_s'], 1.)

    def test_full_environment_count_first_discovery_and_no_discovery_null(self):
        events = [event('A6_TASK_START', 1), event('A6_CAPTURE_START', 2),
                  event('A6_ENV_RESULT', 4, confirmed_candidate_count=0, round=1),
                  event('A6_ENV_RESULT', 8, confirmed_candidate_count=4, round=2),
                  event('A6_ENV_RESULT', 12, confirmed_candidate_count=3, round=3),
                  event('A6_ACTIVE_STOP', 12), event('FAILED', 15)]
        result = self.summarize(events)
        self.assertEqual(result['T_first_env_sim'], 6.)
        self.assertEqual(result['C_env_count_final'], 3)
        self.assertEqual(result['C_env_count_max'], 4)
        self.assertTrue(result['D_env'])
        missing = self.summarize(events[:3] + [event('FAILED', 7)])
        self.assertIsNone(missing['T_first_env_sim'])
        self.assertFalse(missing['D_env'])

    def test_capture_retries_completed_votes_and_view_roles_are_separate(self):
        events = [event('A6_TASK_START', 1),
                  event('A5_VIEWPOINT', 2, view_role='initial', rescan=False),
                  event('A6_CAPTURE_START', 3), event('A5_CAPTURE_RETRY', 4),
                  event('A5_OBSERVATION', 5, round=1),
                  event('A6_ENV_RESULT', 6, round=1, confirmed_candidate_count=0),
                  event('A5_VIEWPOINT', 7, view_role='sensing', rescan=True),
                  event('A6_CAPTURE_START', 8), event('A5_OBSERVATION', 9, round=2),
                  event('A6_ENV_RESULT', 10, round=2, confirmed_candidate_count=2),
                  event('A5_VIEWPOINT', 11, view_role='sensing', rescan=False),
                  event('A6_CAPTURE_START', 12), event('A5_OBSERVATION', 13, round=3),
                  event('A6_ENV_RESULT', 14, round=3, confirmed_candidate_count=3),
                  event('A6_ACTIVE_STOP', 14),
                  event('A5_VIEWPOINT', 15, view_role='return', rescan=False), event('FAILED', 16)]
        counts = self.summarize(events)['counts']
        self.assertEqual(counts, dict(capture_calls=3, discarded_windows=1, completed_windows=3,
                                     voted_windows=3, sensing_visits=3, sensing_rescans=1,
                                     nbv_moves=1, initial_outbound=1, return_actions=1))

    def test_rm4d_has_zero_active_resources_and_na_discovery(self):
        result = self.summarize([event('A6_TASK_START', 10), event('A6_LANDED', 18),
                                 event('FAILED', 20)], method='rm4d_only')
        self.assertEqual(result['T_active_sim'], 0.)
        self.assertEqual(result['counts']['completed_windows'], 0)
        self.assertIsNone(result['T_first_env_sim'])
        self.assertIsNone(result['D_env'])
        self.assertEqual(result['paths']['uav_active']['distance_m'], 0.)

    def test_recovery_landing_cannot_extend_primary_uav_bounds(self):
        result = self.summarize([event('A6_TASK_START', 1), event('FAILED', 3),
                                 event('A6_LANDED', 5), event('A6_TASK_END', 6)])
        self.assertEqual(result['paths']['uav_total']['end_sim'], 3)
        self.assertEqual(result['paths']['uav_cleanup']['start_sim'], 3)
        self.assertEqual(result['landed_sim'], 5)

    def test_cleanup_clock_reset_does_not_invalidate_finished_primary_metrics(self):
        result = self.summarize([event('A6_TASK_START', 10, 100), event('LIFT', 25, 140),
                                 event('A6_TASK_END', 2, 150)])
        self.assertFalse(result['clock_reset_detected'])
        self.assertEqual(result['T_task_sim'], 15)
        self.assertTrue(result['cleanup_clock_reset_detected'])

    def test_empty_handoff_marks_active_stage_failed_after_policy_finishes(self):
        result = self.summarize([event('A6_TASK_START', 1), event('A6_CAPTURE_START', 2),
                                 event('A6_ACTIVE_STOP', 5, stop_reason='NO_PREDICTED_TASK_GAIN'),
                                 event('FAILED', 5, reason='no confirmed candidate')])
        self.assertEqual(result['terminal_failure_stage'], 'active')
        self.assertEqual(result['stages']['active']['status'], 'FAILED')

    def test_failure_after_successful_lift_motion_is_still_lift_failure(self):
        result = self.summarize([event('A6_TASK_START', 1), event('A6_STAGE_START', 2, stage='lift'),
                                 event('A6_STAGE_END', 3, stage='lift'),
                                 event('FAILED', 3, reason='minimum lift not reached')])
        self.assertEqual(result['stages']['lift']['status'], 'FAILED')
        self.assertEqual(result['stages']['retention']['status'], 'NOT_REACHED')


if __name__ == '__main__':
    unittest.main()
