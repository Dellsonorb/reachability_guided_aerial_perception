import importlib.util
import unittest
from pathlib import Path
import numpy as np


class OfflineCompletionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p = Path(__file__).with_name('offline.py')
        cls.module = None
        if p.exists():
            spec = importlib.util.spec_from_file_location('completion_offline', p)
            cls.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.module)

    def api(self):
        self.assertIsNotNone(self.module, 'standalone offline implementation is missing')
        return self.module

    def test_two_complementary_views_complete_one_candidate(self):
        m = self.api()
        w = np.eye(2)
        lower, upper = m.completion_bounds(np.array([1, 1]), [[0, 1]], w, 2)
        np.testing.assert_array_equal(lower, [[False, True], [True, False]])
        np.testing.assert_array_equal(lower, upper)

    def test_two_votes_cannot_be_added_in_one_window(self):
        m = self.api()
        lower, upper = m.completion_bounds(np.array([0]), [[0]], np.ones((1, 1)), 1)
        self.assertFalse(upper.any())
        self.assertTrue(m.completion_bounds(np.array([0]), [[0]], np.ones((1, 1)), 2)[0].all())

    def test_partial_phases_are_not_guaranteed_or_independent(self):
        m = self.api()
        lower, upper = m.completion_bounds(np.array([1, 1]), [[0, 1]], np.full((1, 2), .5), 1)
        self.assertFalse(lower.any())
        self.assertTrue(upper.all())

    def test_candidate_incidence_not_union_and_inputs_not_mutated(self):
        m = self.api()
        h = np.array([2, 2, 0, 0]); before = h.copy()
        w = np.zeros((1, 4))
        self.assertTrue(m.completion_bounds(h, [[0, 1], [2, 3]], w, 1)[0].all())
        self.assertFalse(m.completion_bounds(h, [[0, 2], [1, 3]], w, 1)[1].any())
        np.testing.assert_array_equal(h, before)
        self.assertFalse(m.completion_bounds(h, [], w, 1)[1].any())

    def test_transition_must_be_in_local_generator(self):
        m = self.api()
        poses = np.array([[0, 0, 1, 0], [2, 0, 1, 0], [-2, 0, 1, 0]])
        allowed, costs = m.transition_graph(poses, [-2, -1, 0, 1, 2], 8, .25)
        self.assertFalse(allowed[1, 2]); self.assertTrue(allowed[0, 1])
        self.assertEqual(costs[0, 1], 2)

    def test_strict_completion_improvement_not_cost_only(self):
        m = self.api()
        lower = np.array([[False, False], [False, True]])
        upper = np.ones((2, 2), dtype=bool)
        result = m.select_sequence(lower, upper, np.ones((2, 2), bool), np.array([[0., 1.], [2., 3.]]), 0)
        self.assertEqual(result['first_index'], 1)
        self.assertTrue(result['strict_improvement'])
        result = m.select_sequence(upper, upper, upper, np.array([[0., 1.], [2., 3.]]), 1)
        self.assertTrue(result['changed']); self.assertFalse(result['strict_improvement'])
        self.assertIsNone(m.select_sequence(lower*False, lower*False, upper, upper.astype(float), 0)['first_index'])

    def test_a5_facade_excludes_yaw_only_but_allows_same_view_rescan(self):
        m = self.api()
        poses = np.array([[0, 0, 1, 0], [0, 0, 1, np.pi/4], [1, 0, 1, 0]])
        allowed, _ = m.transition_graph(poses, [-1, 0, 1], 8, .25, .15)
        self.assertFalse(allowed[0, 1])
        self.assertTrue(allowed[0, 0])
        self.assertTrue(allowed[0, 2])

    def test_blocked_and_clipped_candidates_never_enter_support(self):
        m = self.api()
        cases = [dict(operational=dict(blocked=b), footprint_clipped=c)
                 for b, c in [(False, False), (False, True), (True, False)]]
        self.assertEqual(m.eligible(cases), cases[:1])

    def test_boolean_opportunities_match_direct_two_window_counts(self):
        m = self.api()
        rng = np.random.default_rng(927)
        for _ in range(40):
            h = rng.integers(0, 3, size=6)
            w = rng.integers(0, 2, size=(4, 6))
            supports = [[0, 1, 2], [2, 3, 5]]
            expected = np.array([[any(np.all((h+w[i]+w[j])[s] >= 2) for s in supports)
                                  for j in range(4)] for i in range(4)])
            lower, upper = m.completion_bounds(h, supports, w, 2)
            np.testing.assert_array_equal(lower, expected)
            np.testing.assert_array_equal(upper, expected)

    def test_zero_and_missing_history_never_become_predicted_votes(self):
        m = self.api()
        with self.assertRaises(ValueError):
            m.completion_bounds(np.array([-1]), [[0]], np.ones((1, 1)), 1)
        with self.assertRaises(ValueError):
            m.completion_bounds(np.array([0]), [[0]], np.array([[np.nan]]), 2)

    def test_phase_windows_do_not_combine_incompatible_endpoint_phases(self):
        m = self.api()
        packets = np.array([[1, 0], [0, 1]], dtype=bool)
        phases = m.phase_windows(packets, 1)
        # Marginals allow both cells, but neither whole phase sees both.
        self.assertFalse(phases.all(axis=1).any())
        self.assertTrue(m.phase_windows(packets, 2).all())
        self.assertTrue(m.phase_windows(packets, 3).all())

    def test_actual_runtime_command_not_worker_proposal_identifies_view(self):
        m = self.api()
        rank = dict(candidates=[dict(candidate_id=i, viewpoint=dict(position_xyz=[i, 0, 1], yaw_rad=0))
                                for i in range(2)], best_task_id=0, best_generic_id=1)
        self.assertEqual(m.commanded_index(rank, dict(next_viewpoint=[1, 0, 1, 0])), 1)
        self.assertIsNone(m.commanded_index(rank, dict(next_viewpoint=None)))


if __name__ == '__main__':
    unittest.main()
