import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('diagnostics', Path(__file__).resolve().parents[1]/'scripts/a6_scoring_diagnostics.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ScoringDiagnosticsTests(unittest.TestCase):
    def inputs(self):
        alpha = -np.expm1(-.5)
        arrays = dict(visibility=np.array([[[1, 0]], [[0, 1]]], dtype=bool),
                      a2_unknown_score=np.ones((1, 2)), a2_observation_count=np.zeros((1, 2)),
                      a3_task_relevant_uncertainty=np.array([[1., np.nan]]),
                      a3_task_relevance_at_environment_cell=np.array([[1., np.nan]]),
                      delta_unknown=np.ones((1, 2))*alpha,
                      marginal_task_gain=np.array([[alpha, np.nan]]))
        ranking = dict(a2_config=dict(unknown_scale=2.), config=dict(flight_weight=.25),
                       best_task_id=0, best_generic_id=1, task_order=[0, 1], generic_order=[1, 0],
                       candidates=[dict(candidate_id=0, status='VALID', task_gain=alpha,
                                        generic_gain=alpha, flight_cost=1., task_score=alpha-.25, generic_score=alpha-.25),
                                   dict(candidate_id=1, status='VALID', task_gain=0.,
                                        generic_gain=alpha, flight_cost=0., task_score=0., generic_score=alpha)])
        return ranking, arrays

    def test_same_masks_cost_with_weighting_only_can_change_selection(self):
        ranking, arrays = self.inputs()
        report = module.check_snapshot(ranking, arrays)
        self.assertTrue(report['gain_and_cost_identity_pass'])
        self.assertTrue(report['different_selection'])
        self.assertEqual(report['candidate_count'], 2)

    def test_mismatched_generic_penalty_is_detected(self):
        ranking, arrays = self.inputs()
        ranking['candidates'][0]['generic_score'] += .1
        report = module.check_snapshot(ranking, arrays)
        self.assertFalse(report['gain_and_cost_identity_pass'])
        self.assertAlmostEqual(report['maximum_abs_identity_error'], .1)

    def test_missing_candidate_mask_cannot_pass_a_partial_zip(self):
        ranking, arrays = self.inputs()
        arrays['visibility'] = arrays['visibility'][:1]
        self.assertFalse(module.check_snapshot(ranking, arrays)['gain_and_cost_identity_pass'])

    def test_operational_weight_must_explain_task_uncertainty(self):
        ranking, arrays = self.inputs()
        arrays['a3_task_relevance_at_environment_cell'][0, 0] = 0
        self.assertFalse(module.check_snapshot(ranking, arrays)['gain_and_cost_identity_pass'])

    def test_impossible_argmax_is_not_reported_as_an_identity_pass(self):
        ranking, arrays = self.inputs()
        ranking['best_task_id'] = 999
        self.assertFalse(module.check_snapshot(ranking, arrays)['gain_and_cost_identity_pass'])

    def test_nonfinite_candidate_scalar_is_not_ignored_by_python_max(self):
        for key in ('task_gain', 'generic_score'):
            ranking, arrays = self.inputs()
            ranking['candidates'][0][key] = float('nan')
            self.assertFalse(module.check_snapshot(ranking, arrays)['gain_and_cost_identity_pass'])

    def test_infinite_operational_weight_is_not_treated_as_no_support_nan(self):
        ranking, arrays = self.inputs()
        arrays['a3_task_relevance_at_environment_cell'][0, 0] = float('inf')
        self.assertFalse(module.check_snapshot(ranking, arrays)['gain_and_cost_identity_pass'])


if __name__ == '__main__': unittest.main()
