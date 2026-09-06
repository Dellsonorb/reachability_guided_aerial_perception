import json
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from matplotlib.figure import Figure

import numpy as np

from reachability_guided_nbv import rank_viewpoints
from reachability_guided_nbv.synthetic import make_scenarios
from reachability_guided_nbv.outputs import render_result, render_occlusion_rays, save_result


def evaluate(case):
    return rank_viewpoints(case.task, case.belief, case.current,
                           candidates=case.candidates, config=case.config)


class OfflineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenes = {s.name: s for s in make_scenarios()}

    def test_three_groups_only(self):
        self.assertEqual(set(self.scenes), {'task_vs_generic', 'occlusion', 'flight_cost'})

    def test_large_low_relevance_unknown_does_not_attract_ours(self):
        case = self.scenes['task_vs_generic'].cases[0]
        result = evaluate(case)
        self.assertNotEqual(result.best_task.candidate_id, result.best_generic.candidate_id)
        self.assertGreater(result.best_task.task_gain, result.best_generic.task_gain)
        self.assertGreater(result.best_generic.generic_gain, result.best_task.generic_gain)
        self.assertAlmostEqual(result.best_task.viewpoint.yaw_rad, 0)
        self.assertAlmostEqual(result.best_generic.viewpoint.yaw_rad, -np.pi)

    def test_occlusion_changes_winner_without_changing_a3_support(self):
        clear, blocked = self.scenes['occlusion'].cases
        before, after = evaluate(clear), evaluate(blocked)
        np.testing.assert_array_equal(clear.task.task_relevance_at_environment_cell,
                                      blocked.task.task_relevance_at_environment_cell)
        np.testing.assert_array_equal(clear.task.task_relevant_uncertainty, blocked.task.task_relevant_uncertainty)
        self.assertEqual(before.best_task.candidate_id, 0)
        self.assertEqual(after.best_task.candidate_id, 1)
        self.assertEqual(after.candidates[0].task_gain, 0)
        self.assertGreater(after.candidates[1].task_gain, 0)

    def test_cost_prevents_distant_small_gain_trip(self):
        zero, cost = self.scenes['flight_cost'].cases
        before, after = evaluate(zero), evaluate(cost)
        self.assertEqual(before.best_task.candidate_id, 1)
        self.assertEqual(after.best_task.candidate_id, 0)
        np.testing.assert_array_equal(before.visibility, after.visibility)
        extra_gain = before.candidates[1].task_gain - before.candidates[0].task_gain
        self.assertGreater(extra_gain, 0)
        self.assertLess(extra_gain, cost.config.flight_weight * after.candidates[1].flight_cost)
        self.assertGreater(after.candidates[1].flight_cost, 10)

    def test_outputs_reproduce_gains_and_keep_support_sources(self):
        case = self.scenes['task_vs_generic'].cases[0]
        result = evaluate(case)
        with tempfile.TemporaryDirectory() as tmp:
            paths = save_result(result, case.task, case.belief, tmp)
            summary = json.loads(paths['ranking'].read_text())
            self.assertEqual(summary['gain_semantics'], 'predicted_observation_gain_surrogate')
            with np.load(paths['arrays'], allow_pickle=False) as data:
                np.testing.assert_allclose(np.nansum(data['task_contributions'], axis=(1, 2)),
                                           [r.task_gain for r in result.candidates])
                np.testing.assert_array_equal(data['a3_best_operational_source'], case.task.best_operational_source)
                self.assertIn('a2_observation_count', data)
                self.assertIn('a3_support_state', data)
            render_result(result, case.task, case.belief, Path(tmp) / 'result.png', title=case.name)
            render_occlusion_rays(Path(tmp) / 'rays.png')
            self.assertGreater((Path(tmp) / 'result.png').stat().st_size, 10000)
            self.assertGreater((Path(tmp) / 'rays.png').stat().st_size, 10000)

    def test_plot_labels_the_configured_surrogate_height(self):
        case = self.scenes['task_vs_generic'].cases[0]
        case = replace(case, config=replace(case.config, assumed_height_m=1.25))
        result = evaluate(case)
        original_save = Figure.savefig
        titles = []

        def inspect_and_save(figure, *args, **kwargs):
            titles.append(figure._suptitle.get_text())
            return original_save(figure, *args, **kwargs)

        with tempfile.TemporaryDirectory() as tmp, patch.object(Figure, 'savefig', inspect_and_save):
            render_result(result, case.task, case.belief, Path(tmp) / 'height.png')
        self.assertIn('H_assumed=1.25 m', titles[0])


if __name__ == '__main__':
    unittest.main()
