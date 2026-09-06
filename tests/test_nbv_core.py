import unittest
from dataclasses import replace

import numpy as np

from task_relevant_uncertainty import SupportState, build_task_uncertainty
from task_relevant_uncertainty.synthetic import make_scenarios
from reachability_guided_nbv import NBVConfig, Viewpoint, rank_viewpoints
from reachability_guided_nbv.core import flight_cost


class RankingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenes = make_scenarios()
        cls.current = Viewpoint((-4, 0, 1.5), 0)

    def rank(self, index=0, **kwargs):
        scene = self.scenes[index]
        task = build_task_uncertainty(scene.a1, scene.a2)
        return rank_viewpoints(task, scene.a2, self.current, candidates=(self.current,), **kwargs), task

    def test_gain_identity_and_generic_comparator(self):
        result, task = self.rank()
        visible = result.visibility[0]
        alpha = -np.expm1(-1 / self.scenes[0].a2.config.unknown_scale)
        expected = alpha * np.nansum(visible * task.task_relevant_uncertainty)
        self.assertGreater(expected, 0)
        self.assertAlmostEqual(result.candidates[0].task_gain, expected)
        self.assertAlmostEqual(result.candidates[0].generic_gain, alpha * visible.sum())
        self.assertEqual(result.best_task.candidate_id, 0)
        self.assertEqual(result.status, 'RANKED')

    def test_default_lattice_runs_end_to_end_with_actual_mount(self):
        scene = self.scenes[0]
        task = build_task_uncertainty(scene.a1, scene.a2)
        result = rank_viewpoints(task, scene.a2, self.current)
        self.assertEqual(len(result.candidates), 200)
        self.assertEqual(result.visibility.shape, (200, *task.grid.shape))
        self.assertEqual(result.candidates[0].viewpoint, self.current)
        self.assertEqual(result.status, 'RANKED')

    def test_free_keeps_positive_but_lower_marginal_gain(self):
        free, _ = self.rank(1)
        unknown, _ = self.rank(0)
        self.assertGreater(free.best_task.task_gain, 0)
        self.assertAlmostEqual(free.best_task.task_gain / unknown.best_task.task_gain, np.exp(-8 / 2))

    def test_low_relevance_reduces_task_not_generic_gain(self):
        high, _ = self.rank(0)
        low, _ = self.rank(2)
        self.assertAlmostEqual(low.best_task.task_gain / high.best_task.task_gain, .2 / .9)
        self.assertAlmostEqual(low.best_task.generic_gain, high.best_task.generic_gain)

    def test_no_support_is_masked_not_generic_exploration_fallback(self):
        scene = self.scenes[0]
        task = build_task_uncertainty(scene.a1, scene.a2)
        task = replace(task, support_state=np.full(task.grid.shape, SupportState.NO_VALIDATED_SUPPORT),
                       task_relevance_at_environment_cell=np.full(task.grid.shape, np.nan),
                       task_relevant_uncertainty=np.full(task.grid.shape, np.nan))
        result = rank_viewpoints(task, scene.a2, self.current, candidates=(self.current,))
        self.assertEqual(result.status, 'NO_PREDICTED_TASK_GAIN')
        self.assertIsNone(result.best_task)
        self.assertGreater(result.best_generic.generic_gain, 0)
        self.assertTrue(np.isnan(result.task_contribution(0)).all())

    def test_stale_a3_a2_pair_is_rejected(self):
        task = build_task_uncertainty(self.scenes[0].a1, self.scenes[0].a2)
        with self.assertRaisesRegex(ValueError, 'snapshot'):
            rank_viewpoints(task, self.scenes[1].a2, self.current)

    def test_alignment_and_numerical_consistency(self):
        scene = self.scenes[0]
        task = build_task_uncertainty(scene.a1, scene.a2)
        changed = replace(scene.a2, grid=replace(scene.a2.grid, origin_xy=(0, 0)))
        with self.assertRaisesRegex(ValueError, 'aligned'):
            rank_viewpoints(task, changed, self.current)
        with self.assertRaisesRegex(ValueError, 'unknown_score'):
            rank_viewpoints(task, replace(scene.a2, observation_count=np.ones(task.grid.shape)), self.current)
        with self.assertRaisesRegex(ValueError, 'U_task'):
            rank_viewpoints(replace(task, task_relevant_uncertainty=task.task_relevant_uncertainty * .5),
                            scene.a2, self.current)

    def test_wrapped_yaw_and_translation_cost(self):
        current = Viewpoint((0, 0, 2), np.pi - .1)
        candidate = Viewpoint((3, 4, 2), -np.pi + .1)
        self.assertAlmostEqual(flight_cost(current, candidate, NBVConfig()), 5 + .25 * .2)

    def test_cost_changes_scores_without_changing_gain_or_visibility(self):
        scene = self.scenes[0]
        task = build_task_uncertainty(scene.a1, scene.a2)
        candidates = (self.current, Viewpoint((-5, 0, 1.5), 0))
        zero = rank_viewpoints(task, scene.a2, self.current, candidates=candidates,
                               config=NBVConfig(flight_weight=0))
        cost = rank_viewpoints(task, scene.a2, self.current, candidates=candidates,
                               config=NBVConfig(flight_weight=2))
        np.testing.assert_array_equal(zero.visibility, cost.visibility)
        self.assertAlmostEqual(cost.candidates[1].task_score, zero.candidates[1].task_gain - 2)
        self.assertEqual(cost.candidates[1].task_gain, zero.candidates[1].task_gain)

    def test_inputs_unchanged_and_output_has_source_masks(self):
        scene = self.scenes[0]
        task = build_task_uncertainty(scene.a1, scene.a2)
        old_u, old_n = task.task_relevant_uncertainty.copy(), scene.a2.observation_count.copy()
        result = rank_viewpoints(task, scene.a2, self.current, candidates=(self.current,))
        np.testing.assert_array_equal(task.task_relevant_uncertainty, old_u)
        np.testing.assert_array_equal(scene.a2.observation_count, old_n)
        np.testing.assert_array_equal(result.best_operational_source, task.best_operational_source)
        np.testing.assert_array_equal(result.a1_cell_state, task.a1_cell_state)
        np.testing.assert_array_equal(result.support_state, task.support_state)
        self.assertFalse(result.visibility.flags.writeable)

    def test_rejected_candidates_are_not_ranked(self):
        scene = self.scenes[0]
        task = build_task_uncertainty(scene.a1, scene.a2)
        result = rank_viewpoints(task, scene.a2, self.current, candidates=(Viewpoint((0, 0, -2), 0),))
        self.assertEqual(result.status, 'NO_VALID_CANDIDATE')
        self.assertIsNone(result.best_task)
        self.assertIsNone(result.best_generic)
        self.assertEqual(result.task_order, ())
        with self.assertRaises(ValueError):
            rank_viewpoints(task, scene.a2, self.current, candidates=())


if __name__ == '__main__':
    unittest.main()
