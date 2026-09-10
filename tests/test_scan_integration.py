"""Finite opportunity is a shared score input, never fabricated observation data."""

import json
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from reachability_guided_nbv import SensorModel, Viewpoint, rank_viewpoints
from reachability_guided_nbv.geometry import predict_visibility
from reachability_guided_nbv.outputs import save_result
from task_relevant_uncertainty import build_task_uncertainty
from task_relevant_uncertainty.synthetic import make_scenarios


class HalfWindow:
    """A small schedule stand-in: test the ranker's actual weighted arithmetic."""
    metadata = {'model': 'test_half_window', 'window_packets': 2}

    def predict(self, belief, viewpoint, config):
        prediction = predict_visibility(belief, viewpoint, config=config)
        return SimpleNamespace(status=prediction.status,
                               opportunity=prediction.visible.astype(float) * .5,
                               unoccluded_opportunity=prediction.range_fov.astype(float) * .5)


class ScanRankingIntegrationTests(unittest.TestCase):
    def setUp(self):
        scene = make_scenarios()[0]
        self.belief = scene.a2
        self.task = build_task_uncertainty(scene.a1, scene.a2)
        self.pose = Viewpoint((-4, 0, 1.5), 0)

    def rank(self, **kwargs):
        return rank_viewpoints(self.task, self.belief, self.pose,
                               candidates=(self.pose,), **kwargs)

    def test_fractional_opportunity_weights_both_gains_not_boolean_visibility(self):
        original = self.rank()
        finite = self.rank(scan=HalfWindow())
        self.assertGreater(original.best_task.task_gain, 0)
        np.testing.assert_array_equal(finite.visibility, original.visibility)
        np.testing.assert_array_equal(finite.observation_opportunity, original.visibility * .5)
        self.assertAlmostEqual(finite.best_task.task_gain, original.best_task.task_gain * .5)
        self.assertAlmostEqual(finite.best_generic.generic_gain, original.best_generic.generic_gain * .5)
        self.assertAlmostEqual(float(np.nansum(finite.task_contribution(0))), finite.best_task.task_gain)
        self.assertEqual(finite.acquisition_metadata, dict(HalfWindow.metadata, occlusion={'model': 'raw_occupied_prisms'}))

    def test_legacy_opportunity_is_exact_binary_visibility(self):
        result = self.rank()
        np.testing.assert_array_equal(result.observation_opportunity, result.visibility)
        self.assertEqual(result.acquisition_metadata['model'], 'idealized')

    def test_sensor_mount_mismatch_is_rejected_instead_of_misreported(self):
        schedule = HalfWindow()
        schedule.sensor = SensorModel(T_uav_lidar=np.eye(4))
        with self.assertRaisesRegex(ValueError, 'sensor'):
            self.rank(scan=schedule)

    def test_nonfinite_or_out_of_range_opportunity_cannot_enter_scores(self):
        for value in (np.nan, -0.1, 1.1):
            schedule = HalfWindow()
            schedule.predict = lambda *args, **kwargs: SimpleNamespace(
                status='VALID', opportunity=np.full(self.belief.grid.shape, value),
                unoccluded_opportunity=np.ones(self.belief.grid.shape))
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'opportunity'):
                self.rank(scan=schedule)

    def test_prediction_does_not_write_real_evidence(self):
        before = {name: getattr(self.belief, name).copy() for name in
                  ('state', 'free_evidence', 'occupied_evidence', 'observation_count', 'unknown_score')}
        result = self.rank(scan=HalfWindow())
        for name, array in before.items():
            np.testing.assert_array_equal(array, getattr(self.belief, name))
        self.assertFalse(result.observation_opportunity.flags.writeable)

    def test_saved_gain_input_retains_fractions_and_explicit_semantics(self):
        result = self.rank(scan=HalfWindow())
        with tempfile.TemporaryDirectory() as tmp:
            paths = save_result(result, self.task, self.belief, tmp)
            metadata = json.loads(paths['ranking'].read_text())
            self.assertEqual(metadata['acquisition'], result.acquisition_metadata)
            self.assertIn('observation_opportunity', metadata['formula'])
            with np.load(paths['arrays']) as data:
                np.testing.assert_array_equal(data['observation_opportunity'], result.observation_opportunity)
                np.testing.assert_array_equal(data['unoccluded_opportunity'], result.unoccluded_opportunity)
                from a6_scoring_diagnostics import check_snapshot
                self.assertTrue(check_snapshot(metadata, data)['gain_and_cost_identity_pass'])

    def test_no_occlusion_removes_only_occlusion_not_finite_scan(self):
        from a6_pilot.policy import _without_occlusion
        result = self.rank(scan=HalfWindow())
        changed = _without_occlusion(self.belief, result)
        np.testing.assert_array_equal(changed.observation_opportunity, result.unoccluded_opportunity)
        self.assertAlmostEqual(changed.best_generic.generic_gain,
                               float(np.sum(result.delta_unknown * result.unoccluded_opportunity[0])))

    def test_optional_core_config_preserves_legacy_and_validates_scan_duration(self):
        from sim_active_perception.core import A5Config
        self.assertTrue(hasattr(A5Config(), 'scan_pattern_path'))
        self.assertIsNone(A5Config().scan_pattern_path)
        for value in (0, -1, np.nan):
            with self.subTest(value=value), self.assertRaises(ValueError):
                A5Config(scan_window_s=value)
        with self.assertRaises(ValueError):
            A5Config(scan_publisher_sdf_path='/sensor.sdf')


if __name__ == '__main__':
    unittest.main()
