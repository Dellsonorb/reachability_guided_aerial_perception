import unittest

import numpy as np

from operational_gating import OccupiedClass, PerceivedTarget
from operational_gating.association import (
    associate_returns, geometry_allowance, metric_allowance, validate_reference,
)


def reference():
    return dict(frame_id='map', stamp_s=1., status='AVAILABLE',
                mask=np.ones((41, 41), dtype=bool),
                registered_depth_m=np.full((41, 41), 1.05),
                color_K=np.array([[100., 0, 20], [0, 100., 20], [0, 0, 1]]),
                depth_K=np.array([[80., 0, 20], [0, 80., 20], [0, 0, 1]]),
                T_map_color=np.eye(4))


class TargetAssociationTests(unittest.TestCase):
    def setUp(self):
        self.ref = reference()
        base = PerceivedTarget((0, 0, 1.), 0.)
        self.target = PerceivedTarget(base.center_xyz, base.yaw_rad,
                                      geometry_allowance_m=geometry_allowance(self.ref, base))

    def test_positive_red_depth_and_known_geometry_are_all_required(self):
        labels = associate_returns(np.array([[0., 0., 1.05], [.5, 0., 1.05]]), self.target, self.ref)
        np.testing.assert_array_equal(labels, [OccupiedClass.TARGET, OccupiedClass.ENVIRONMENT])

    def test_wrong_depth_or_missing_depth_remains_ambiguous(self):
        for z in (2., np.nan, 0.):
            with self.subTest(z=z):
                self.ref['registered_depth_m'][20, 20] = z
                self.assertEqual(associate_returns(np.array([[0, 0, 1.05]]), self.target, self.ref)[0],
                                 OccupiedClass.AMBIGUOUS)

    def test_red_mask_boundary_is_not_dilated_to_grant_target(self):
        self.ref['mask'][19, 20] = False
        self.assertEqual(associate_returns(np.array([[0, 0, 1.05]]), self.target, self.ref)[0],
                         OccupiedClass.AMBIGUOUS)

    def test_no_reference_never_manufactures_positive_association(self):
        labels = associate_returns(np.array([[0, 0, 1.05], [.5, 0, 1.05]]), self.target, None)
        np.testing.assert_array_equal(labels, [OccupiedClass.AMBIGUOUS, OccupiedClass.ENVIRONMENT])

    def test_visible_surface_must_also_fit_the_object_geometry(self):
        # A matching optical depth is not enough if the backprojected pixel is
        # geometrically inconsistent with a tiny object under coarse pixels.
        tiny = PerceivedTarget((.0049, 0, 1), 0, size_xyz=(.002, .002, .12))
        self.ref['registered_depth_m'][20, 20] = 1.
        labels = associate_returns(np.array([[.0049, 0, 1.]]), tiny, self.ref)
        self.assertEqual(labels[0], OccupiedClass.AMBIGUOUS)

    def test_full_map_camera_transform_is_used(self):
        self.ref['T_map_color'][:3, 3] = [1, 2, 3]
        target = PerceivedTarget((1, 2, 4), 0, geometry_allowance_m=.06)
        self.assertEqual(associate_returns(np.array([[1, 2, 4.05]]), target, self.ref)[0],
                         OccupiedClass.TARGET)

    def test_rotated_optical_axis_projects_in_camera_not_map_z(self):
        self.ref['T_map_color'][:3, :3] = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
        self.ref['T_map_color'][:3, 3] = [1, 2, 3]
        target = PerceivedTarget((2.05, 2, 3), 0, geometry_allowance_m=.06)
        self.assertEqual(associate_returns(np.array([[2.05, 2, 3]]), target, self.ref)[0],
                         OccupiedClass.TARGET)
        self.ref['registered_depth_m'][20, 20] = 3.
        self.assertEqual(associate_returns(np.array([[2.05, 2, 3]]), target, self.ref)[0],
                         OccupiedClass.AMBIGUOUS)

    def test_allowance_is_declared_sensor_budget_plus_pixel_footprint(self):
        expected = .03 + .002 + .001 + .5 * 3.5 * (np.sqrt(2) / 100 + np.sqrt(2) / 80)
        self.assertAlmostEqual(float(metric_allowance(self.ref, 3.5)), expected)
        self.assertGreater(float(metric_allowance(self.ref, 4)), float(metric_allowance(self.ref, 2)))
        self.assertGreater(self.target.geometry_allowance_m, .033)

    def test_nonfinite_returns_do_not_acquire_target_labels(self):
        self.assertNotEqual(associate_returns(np.array([[np.nan, 0, 1]]), self.target, self.ref)[0],
                            OccupiedClass.TARGET)

    def test_bad_frame_transform_or_intrinsics_rejected(self):
        self.ref['frame_id'] = 'world'
        with self.assertRaises(ValueError):
            validate_reference(self.ref)
        self.ref = reference()
        self.ref['color_K'][0, 0] = 0
        with self.assertRaises(ValueError):
            validate_reference(self.ref)
        self.ref = reference()
        self.ref['T_map_color'][0, 0] = 2
        with self.assertRaises(ValueError):
            validate_reference(self.ref)


if __name__ == '__main__':
    unittest.main()
