import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('a6_setup_check', Path(__file__).resolve().parents[1] / 'scripts/a6_setup_check.py')
setup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(setup)


class SetupTests(unittest.TestCase):
    def test_ground_check_is_endpoint_geometry_without_candidate_or_relevance(self):
        transform = np.eye(4); transform[2, 3] = 1.5
        ground = np.array([[.05+x*.1, .05+y*.1, 0] for x in range(10) for y in range(10)])
        points = np.vstack([ground, ground, [[.05, .05, .5], [8, 0, 0]]]) - [0, 0, 1.5]
        report = setup.ground_summary(points, transform, [0, 0])
        self.assertEqual(report['ground_cells'], 100)
        self.assertAlmostEqual(report['ground_area_m2'], 1.)
        self.assertTrue(report['ground_observation_usable'])

    def test_overhead_returns_are_not_ground(self):
        transform = np.eye(4)
        report = setup.ground_summary(np.array([[0, 0, .1], [0, 0, .03]]), transform, [0, 0])
        self.assertFalse(report['ground_observation_usable'])

    def test_setup_observes_once_without_query_or_ground_execution(self):
        calls = []
        class Adapter:
            _view_position = [-1.4, 0, 1.2]; _view_yaw = 0
            _takeoff_command = 1
            def _wait_preflight(self): calls.append('preflight')
            def _publish_status(self, state, **details): calls.append(state)
            def _execute_flight(self, command, label): calls.append('takeoff')
            def _a5_fly_and_hover(self, goal, label, force_flight=False): calls.append(('view', goal))
            def _observe_from_air(self): calls.append('rgbd'); return (2., 0., .0575, 0.)
            def _a5_capture(self, goal): calls.append('cloud'); return goal
            def _request_land(self): calls.append('land')
            def _safe_land(self): calls.append('cleanup')
        result = setup.run_setup(Adapter())
        self.assertEqual(calls.count('cloud'), 1)
        self.assertEqual(calls.count('rgbd'), 1)
        self.assertEqual(result['kind'], 'METHOD_INDEPENDENT_SETUP')
        self.assertEqual(result['uav_pose_map'], [-1.4, 0, 1.2, 0])
        self.assertEqual(calls[-1], 'cleanup')

    def test_setup_failure_still_lands_and_never_becomes_method_result(self):
        class Adapter:
            def _wait_preflight(self): raise RuntimeError('not ready')
            def _safe_land(self): self.cleaned = True
        adapter = Adapter()
        with self.assertRaisesRegex(RuntimeError, 'not ready'):
            setup.run_setup(adapter)
        self.assertTrue(adapter.cleaned)


if __name__ == '__main__': unittest.main()
