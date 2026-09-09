"""Object-aware gates preserve raw A2 evidence and real ground requirements."""

from dataclasses import FrozenInstanceError
import unittest

import numpy as np

from environment_belief import (
    BeliefConfig, EnvironmentBeliefMapper, EnvironmentGridSpec,
    EnvironmentState, PointCloudObservation,
)
from task_relevant_uncertainty.geometry import FootprintSpec, footprint_cells

try:
    import operational_gating as gating
except ModuleNotFoundError:
    gating = None


def cloud(points_map, stamp=0., *, transform=None, valid_return=None):
    transform = np.eye(4) if transform is None else np.array(transform, copy=True)
    if np.array_equal(transform, np.eye(4)):
        transform[2, 3] = 2.
    points_map = np.asarray(points_map, dtype=float).reshape(-1, 3)
    sensor = (points_map - transform[:3, 3]) @ transform[:3, :3]
    return PointCloudObservation(sensor, 'lidar', stamp, transform, valid_return)


class OperationalGatingTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(gating, 'the operational-gating API is not implemented')
        self.assertTrue(hasattr(gating, 'PerceivedTarget'), 'PerceivedTarget is not implemented')
        self.grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)
        self.target = gating.PerceivedTarget((.68, 0., .0575), 0.)
        rows, columns = np.indices(self.grid.shape)
        self.ground = np.column_stack((
            -1.5 + (columns.ravel() + .5) * .1,
            -1.5 + (rows.ravel() + .5) * .1,
            np.zeros(rows.size),
        ))
        self.target_point = np.array([[.57, .01, .06]])
        self.target_cell = (15, 20)

    def alias_observations(self, extra=(), extra_labels=()):
        points = np.vstack((self.ground, self.target_point,
                            np.asarray(extra, dtype=float).reshape(-1, 3)))
        labels = np.array([gating.OccupiedClass.ENVIRONMENT] * len(self.ground)
                          + [gating.OccupiedClass.TARGET] + list(extra_labels))
        return [cloud(points, stamp) for stamp in (1., 2.)], [labels, labels.copy()]

    def test_target_cell_alias_preserves_raw_occupied_but_real_ground_supports_pose(self):
        observations, labels = self.alias_observations()
        mapper = EnvironmentBeliefMapper(self.grid, sensor_frame='lidar')
        for observation in observations:
            mapper.update(observation)
        before = mapper.snapshot()
        view = gating.derive_operational_evidence(self.grid, observations, self.target, labels=labels)
        assessment = gating.assess_footprint(view, (0., 0.), 0.)
        self.assertEqual(before.state[self.target_cell], EnvironmentState.OCCUPIED)
        self.assertEqual(before.free_evidence[self.target_cell], 0)
        self.assertEqual(view.target_occupied_votes[self.target_cell], 2)
        self.assertEqual(view.ground_votes[self.target_cell], 2)
        self.assertFalse(assessment.blocked)
        self.assertFalse(assessment.target_collision)
        self.assertTrue(assessment.ground_supported)
        self.assertEqual(assessment.target_cells, 1)
        self.assertEqual(assessment.ground_missing_cells, 0)
        self.assertEqual(assessment.covered_cells,
                         tuple(footprint_cells(self.grid, (0., 0.), 0.)[0]))
        after = mapper.snapshot()
        for name in ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score'):
            np.testing.assert_array_equal(getattr(before, name), getattr(after, name))

    def test_environment_and_ambiguous_occupied_votes_block_even_with_target(self):
        for occupied_class in (gating.OccupiedClass.ENVIRONMENT, gating.OccupiedClass.AMBIGUOUS):
            with self.subTest(occupied_class=occupied_class):
                observations, labels = self.alias_observations([[.58, .02, .07]], [occupied_class])
                view = gating.derive_operational_evidence(self.grid, observations, self.target, labels=labels)
                assessment = gating.assess_footprint(view, (0., 0.), 0.)
                self.assertEqual(view.target_occupied_votes[self.target_cell], 2)
                self.assertEqual(view.ground_votes[self.target_cell], 0)
                self.assertTrue(assessment.blocked)
                self.assertFalse(assessment.target_collision)
                self.assertFalse(assessment.ground_supported)
                self.assertEqual(assessment.environment_cells + assessment.ambiguous_cells, 1)

    def test_ground_support_is_independent_of_blocking(self):
        observations = [cloud(self.ground, 1), cloud(self.ground, 2), cloud(self.target_point, 3)]
        view = gating.derive_operational_evidence(self.grid, observations, self.target)
        assessment = gating.assess_footprint(view, (0., 0.), 0.)
        self.assertTrue(assessment.ground_supported)
        self.assertTrue(assessment.blocked)
        self.assertEqual(view.ground_votes[self.target_cell], 2)

    def test_missing_association_is_ambiguous_and_cannot_exempt_occupied_returns(self):
        observations, _ = self.alias_observations()
        view = gating.derive_operational_evidence(self.grid, observations, self.target)
        self.assertEqual(view.ambiguous_occupied_votes[self.target_cell], 2)
        self.assertEqual(view.target_occupied_votes.sum(), 0)
        self.assertTrue(gating.assess_footprint(view, (0., 0.), 0.).blocked)

    def test_target_labels_outside_the_expanded_3d_box_become_ambiguous(self):
        points = [[.55, .01, .06], [.58, .04, .06], [.58, .01, .2]]
        view = gating.derive_operational_evidence(
            self.grid, [cloud(points)], self.target,
            labels=[np.full(3, gating.OccupiedClass.TARGET)],
        )
        self.assertEqual(view.target_occupied_votes.sum(), 0)
        self.assertEqual(view.ambiguous_occupied_votes[self.target_cell], 1)
        self.assertTrue(gating.assess_footprint(view, (0., 0.), 0.).blocked)

    def test_target_only_returns_never_supply_ground(self):
        view = gating.derive_operational_evidence(
            self.grid, [cloud(self.target_point, t) for t in (1, 2)], self.target,
            labels=[np.array([gating.OccupiedClass.TARGET])] * 2,
        )
        assessment = gating.assess_footprint(view, (0., 0.), 0.)
        self.assertEqual(view.ground_votes.sum(), 0)
        self.assertFalse(assessment.ground_supported)
        self.assertFalse(assessment.blocked)

    def test_votes_count_windows_not_points_and_preserve_all_occupied_classes(self):
        labels = np.repeat(np.array(list(gating.OccupiedClass)), 20)
        points = np.repeat(self.target_point, len(labels), axis=0)
        view = gating.derive_operational_evidence(
            self.grid, [cloud(points, 1), cloud(points, 2)], self.target, labels=[labels, labels],
        )
        for name in ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes'):
            self.assertEqual(getattr(view, name)[self.target_cell], 2)
            self.assertEqual(getattr(view, name).sum(), 2)
        one = gating.derive_operational_evidence(self.grid, [cloud(np.repeat(self.ground, 3, axis=0))], self.target)
        self.assertTrue(np.all(one.ground_votes == 1))
        self.assertFalse(gating.assess_footprint(one, (0., 0.), 0.).ground_supported)

    def test_continuous_collision_and_closed_contact_are_checked_without_clouds(self):
        for center_x, expected in ((.60, True), (.64, True), (.640001, False)):
            with self.subTest(center_x=center_x):
                target = gating.PerceivedTarget((center_x, 0., .0575), 0.)
                view = gating.derive_operational_evidence(self.grid, [], target)
                result = gating.assess_footprint(view, (0., 0.), 0.)
                self.assertEqual(result.target_collision, expected)
                self.assertEqual(result.blocked, expected)
                self.assertFalse(result.ground_supported)

    def test_rotated_target_uses_rectangle_sat_not_axis_aligned_bounding_box(self):
        target = gating.PerceivedTarget((.2, .2, .1), np.pi / 4, (.4, .04, .2))
        view = gating.derive_operational_evidence(self.grid, [], target)
        footprint = FootprintSpec(.02, .02)
        # Both poses overlap the target's bounding box; only the diagonal one overlaps its rectangle.
        self.assertFalse(gating.assess_footprint(view, (.33, .07), 0., footprint).target_collision)
        self.assertTrue(gating.assess_footprint(view, (.30, .30), np.pi / 4, footprint).target_collision)

    def test_rotating_the_base_changes_target_collision(self):
        target = gating.PerceivedTarget((0., .56, .0575), 0.)
        view = gating.derive_operational_evidence(self.grid, [], target)
        self.assertFalse(gating.assess_footprint(view, (0., 0.), 0.).target_collision)
        self.assertFalse(gating.assess_footprint(view, (0., 0.), np.pi / 2).target_collision)
        nearer = gating.PerceivedTarget((0., .54, .0575), 0.)
        view = gating.derive_operational_evidence(self.grid, [], nearer)
        self.assertFalse(gating.assess_footprint(view, (0., 0.), 0.).target_collision)
        self.assertTrue(gating.assess_footprint(view, (0., 0.), np.pi / 2).target_collision)

    def test_geometry_allowance_applies_to_association_and_collision(self):
        target = gating.PerceivedTarget((.68, 0., .0575), 0., geometry_allowance_m=.04)
        self.assertTrue(target.contains(np.array([[.53, .05, .14]]))[0])
        view = gating.derive_operational_evidence(self.grid, [], target)
        self.assertTrue(gating.assess_footprint(view, (0., 0.), 0.).target_collision)
        self.assertAlmostEqual(target.footprint.half_length_m, .16)
        self.assertAlmostEqual(target.footprint.half_width_m, .0665)

    def test_filtering_matches_raw_a2_including_original_label_alignment(self):
        transform = np.eye(4)
        transform[:3, 3] = [.25, .35, 2.]
        sensor_points = np.array([[0, 0, 0], [0, 0, -.2], [0, 0, -40],
                                  [np.nan, 0, 0], [0, 0, -2], [0, 0, -1],
                                  [4, 0, -2], [0, 0, -1.97], [.35, -.34, -1.94]])
        valid = np.array([True, True, True, True, True, False, True, True, True])
        observation = PointCloudObservation(sensor_points, 'lidar', 0, transform, valid)
        labels = np.array([gating.OccupiedClass.ENVIRONMENT] * 8 + [gating.OccupiedClass.TARGET])
        mapper = EnvironmentBeliefMapper(self.grid, sensor_frame='lidar')
        mapper.update(observation)
        view = gating.derive_operational_evidence(self.grid, [observation], self.target, labels=[labels])
        raw = mapper.snapshot()
        combined = view.environment_occupied_votes + view.ambiguous_occupied_votes + view.target_occupied_votes
        np.testing.assert_array_equal(combined, raw.occupied_evidence)
        np.testing.assert_array_equal(view.ground_votes, raw.free_evidence)
        self.assertEqual(view.target_occupied_votes.sum(), 1)
        self.assertEqual(view.ground_votes.sum(), 1)

    def test_height_and_internal_grid_edges_follow_raw_a2(self):
        x_edge = self.grid.origin_xy[0] + 20 * self.grid.resolution_m
        heights = [-.021, -.02, .02, .03, .049, .05, 3.]
        points = np.array([[x_edge, .25 + i * .1, z] for i, z in enumerate(heights)])
        observation = PointCloudObservation(points, 'lidar', 1., np.eye(4))
        mapper = EnvironmentBeliefMapper(self.grid, sensor_frame='lidar')
        mapper.update(observation)
        view = gating.derive_operational_evidence(self.grid, [observation], self.target,
                                                 labels=[np.zeros(len(points), dtype=int)])
        raw = mapper.snapshot()
        np.testing.assert_array_equal(view.environment_occupied_votes, raw.occupied_evidence)
        np.testing.assert_array_equal(view.ground_votes, raw.free_evidence)
        self.assertEqual(view.environment_occupied_votes[:, 19].sum(), 0)

    def test_transformed_points_use_the_full_sensor_rotation(self):
        transform = np.eye(4)
        transform[:3, :3] = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
        transform[:3, 3] = [1., 1., 2.]
        observation = cloud(self.target_point, transform=transform)
        view = gating.derive_operational_evidence(self.grid, [observation], self.target,
                                                 labels=[np.array([gating.OccupiedClass.TARGET])])
        self.assertEqual(view.target_occupied_votes[self.target_cell], 1)

    def test_clipping_or_no_covered_cells_never_has_ground_support(self):
        observations = [cloud(self.ground, 1), cloud(self.ground, 2)]
        view = gating.derive_operational_evidence(self.grid, observations, self.target)
        for xy in ((-1.45, -1.45), (-5., -5.)):
            result = gating.assess_footprint(view, xy, 0.)
            self.assertTrue(result.footprint_clipped)
            self.assertFalse(result.ground_supported)
        result = gating.assess_footprint(view, (-5., -5.), 0.)
        self.assertEqual(result.covered_cells, ())
        self.assertEqual(result.ground_missing_cells, 0)

    def test_strict_window_stamps_and_consistent_sensor_frame_are_required(self):
        for stamps in ((1, 1), (2, 1)):
            with self.subTest(stamps=stamps), self.assertRaises(ValueError):
                gating.derive_operational_evidence(self.grid, [cloud([], t) for t in stamps], self.target)
        wrong = PointCloudObservation(np.empty((0, 3)), 'other_sensor', 2, np.eye(4))
        with self.assertRaises(ValueError):
            gating.derive_operational_evidence(self.grid, [cloud([], 1), wrong], self.target)

    def test_labels_require_one_aligned_integer_array_per_window_with_known_values(self):
        observation = cloud(self.target_point)
        for labels in ([], [np.array([0]), np.array([0])], [np.array([])],
                       [np.array([3])], [np.array([-1])], [np.array([2.])],
                       [np.array([True])], [np.array([[2]])]):
            with self.subTest(labels=labels), self.assertRaises(ValueError):
                gating.derive_operational_evidence(self.grid, [observation], self.target, labels=labels)

    def test_target_is_immutable_validated_and_contains_oriented_3d_geometry(self):
        center = [1., 2., .1]
        target = gating.PerceivedTarget(center, np.pi / 2, [.4, .1, .2])
        center[0] = 10.
        self.assertEqual(target.center_xyz, (1., 2., .1))
        np.testing.assert_allclose(target.xy_vertices.min(axis=0), [.95, 1.8])
        np.testing.assert_allclose(target.xy_vertices.max(axis=0), [1.05, 2.2])
        np.testing.assert_array_equal(target.contains(np.array([[1., 2.2, .2], [1.1, 2., .1], [1., 2., .21]])),
                                      [True, False, False])
        with self.assertRaises(FrozenInstanceError):
            target.yaw_rad = 0.
        for kwargs in (dict(center_xyz=(0., np.nan, 0.)), dict(yaw_rad=np.inf),
                       dict(yaw_rad=True), dict(size_xyz=(0., .1, .1)),
                       dict(size_xyz=(.1, .1)), dict(geometry_allowance_m=-.01),
                       dict(geometry_allowance_m=np.inf), dict(frame_id='world')):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                gating.PerceivedTarget(**dict(dict(center_xyz=(0., 0., 0.), yaw_rad=0.), **kwargs))

    def test_view_snapshots_are_detached_readonly_and_inputs_are_unchanged(self):
        observations, labels = self.alias_observations()
        point_copies = [o.points_xyz.copy() for o in observations]
        label_copies = [values.copy() for values in labels]
        view = gating.derive_operational_evidence(self.grid, observations, self.target, labels=labels)
        for name in ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes', 'ground_votes'):
            self.assertFalse(getattr(view, name).flags.writeable)
            with self.assertRaises(ValueError):
                getattr(view, name)[0, 0] = 9
        for original, observation in zip(point_copies, observations):
            np.testing.assert_array_equal(original, observation.points_xyz)
        for original, values in zip(label_copies, labels):
            np.testing.assert_array_equal(original, values)
        values = np.zeros(self.grid.shape, dtype=np.int64)
        direct = gating.OperationalEvidenceView(self.grid, BeliefConfig(), self.target,
                                               values, values, values, values)
        values[:] = 99
        self.assertEqual(direct.ground_votes.sum(), 0)
        with self.assertRaises(FrozenInstanceError):
            direct.target = self.target

    def test_invalid_view_arrays_and_assessment_geometry_are_rejected(self):
        good = np.zeros(self.grid.shape, dtype=int)
        for bad in (np.zeros((1, 1)), np.full(self.grid.shape, -1),
                    np.full(self.grid.shape, np.nan), np.full(self.grid.shape, .5)):
            with self.subTest(shape=bad.shape), self.assertRaises(ValueError):
                gating.OperationalEvidenceView(self.grid, BeliefConfig(), self.target, bad, good, good, good)
        view = gating.derive_operational_evidence(self.grid, [], self.target)
        for xy, yaw in (((np.nan, 0), 0), ((0,), 0), ((0, 0), np.inf)):
            with self.subTest(xy=xy, yaw=yaw), self.assertRaises(ValueError):
                gating.assess_footprint(view, xy, yaw)


if __name__ == '__main__':
    unittest.main()
