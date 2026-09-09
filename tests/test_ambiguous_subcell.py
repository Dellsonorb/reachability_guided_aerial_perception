"""Analytic endpoint geometry and conservative retention of ambiguity history."""

from dataclasses import FrozenInstanceError, asdict, replace
import unittest

import numpy as np

from environment_belief import PointCloudObservation, EnvironmentGridSpec
import operational_gating as gating
from test_operational_gating import cloud


class AmbiguousSubcellTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(hasattr(gating, 'AmbiguousEndpointEvidence'),
                        'the optional ambiguous-endpoint sidecar is not implemented')
        self.grid = EnvironmentGridSpec((-2., -2.), 40, 40)
        self.target = gating.PerceivedTarget((1.5, 1.5, .1), 0.)

    def derive(self, windows, *, labels=None, retain=True):
        observations = [cloud(points, index + 1) for index, points in enumerate(windows)]
        return gating.derive_operational_evidence(
            self.grid, observations, self.target, labels=labels,
            retain_ambiguous_endpoints=retain,
        )

    def test_declared_profile_and_closed_disk_rectangle_contact(self):
        self.assertEqual(gating.AMBIGUOUS_ENDPOINT_RADIUS_M, .033)
        self.assertEqual(gating.AMBIGUOUS_ENDPOINT_PROFILE,
                         'public-map-tf-conditional-disk33mm-v1')
        radius = .033
        corner_offset = radius / np.sqrt(2.)
        points = np.array([
            [0., 0.], [.52, .39], [.52 + radius, 0.],
            [.52 + radius - 1e-6, 0.], [.52 + radius + 1e-6, 0.],
            [.52 + corner_offset, .39 + corner_offset],
            [.52 + corner_offset + 1e-6, .39 + corner_offset + 1e-6],
        ])
        expected = [True, True, True, True, False, True, False]
        for yaw in (0., np.pi / 2, -.73):
            with self.subTest(yaw=yaw):
                c, s = np.cos(yaw), np.sin(yaw)
                rotation = np.array([[c, -s], [s, c]])
                center = np.array([.23, -.27])
                np.testing.assert_array_equal(
                    gating.disk_footprint_intersections(points @ rotation.T + center,
                                                        center, yaw), expected,
                )

    def test_all_points_in_one_vote_group_are_retained_and_can_block(self):
        view = self.derive([[[.59, .2, .1], [.545, .2, .1], [.58, .2, .1]]])
        endpoints = view.ambiguous_endpoints
        self.assertEqual(view.ambiguous_occupied_votes.sum(), 1)
        self.assertEqual(endpoints.complete_vote_counts.sum(), 1)
        np.testing.assert_allclose(endpoints.points_xy, [[.59, .2], [.545, .2], [.58, .2]])
        np.testing.assert_array_equal(endpoints.observation_indices, [0, 0, 0])
        np.testing.assert_array_equal(endpoints.row_indices, [0, 1, 2])
        self.assertEqual(len(set(endpoints.cell_ids)), 1)
        self.assertTrue(gating.assess_footprint(view, (0., 0.), 0.).blocked)
        diag = gating.ambiguous_footprint_diagnostics(view, (0., 0.), 0.)
        self.assertEqual(diag.intersecting_endpoint_indices, (1,))
        self.assertEqual(diag.intersecting_endpoint_count, 1)
        self.assertEqual(diag.legacy_fallback_cells, 0)
        self.assertEqual(diag.coarse_ambiguous_cells, 1)
        self.assertEqual(diag.aliased_clear_cells, 0)

    def test_separated_endpoints_release_only_the_coarse_blocker(self):
        windows = [[[.59, .2, .1], [.58, .2, .1]]]
        legacy = self.derive(windows, retain=False)
        view = self.derive(windows)
        before = gating.assess_footprint(legacy, (0., 0.), 0.)
        after = gating.assess_footprint(view, (0., 0.), 0.)
        self.assertTrue(before.blocked)
        self.assertFalse(after.blocked)
        self.assertEqual(after.ambiguous_cells, 1)
        self.assertFalse(after.ground_supported)
        self.assertEqual(set(asdict(before)), set(asdict(after)))
        self.assertEqual(set(asdict(before)), {
            'covered_cells', 'footprint_clipped', 'environment_cells', 'ambiguous_cells',
            'target_cells', 'ground_supported_cells', 'ground_missing_cells',
            'target_collision', 'blocked', 'ground_supported',
        })
        self.assertEqual({k: v for k, v in asdict(before).items() if k != 'blocked'},
                         {k: v for k, v in asdict(after).items() if k != 'blocked'})
        diag = gating.ambiguous_footprint_diagnostics(view, (0., 0.), 0.)
        self.assertEqual(diag.aliased_clear_cell_ids, diag.coarse_ambiguous_cell_ids)
        self.assertEqual(diag.aliased_clear_cells, 1)
        self.assertEqual(diag.intersecting_endpoint_count, 0)

    def test_neighboring_cell_disk_is_checked_even_outside_covered_cells(self):
        view = self.derive([[[0., .415, .1]]])
        result = gating.assess_footprint(view, (0., 0.), 0.)
        self.assertNotIn(int(view.ambiguous_endpoints.cell_ids[0]), result.covered_cells)
        self.assertEqual(result.ambiguous_cells, 0)
        self.assertTrue(result.blocked)
        diag = gating.ambiguous_footprint_diagnostics(view, (0., 0.), 0.)
        self.assertEqual(diag.intersecting_endpoint_indices, (0,))
        self.assertEqual(diag.coarse_ambiguous_cell_ids, ())

    def test_absent_sidecar_and_missing_profile_keep_legacy_cell_block(self):
        view = self.derive([[[.59, .2, .1]]])
        for endpoints, semantics in (
            (None, 'object-aware-v1.1'),
            (replace(view.ambiguous_endpoints, profile=None), 'object-aware-v1.3'),
            (replace(view.ambiguous_endpoints, profile='unknown'), 'object-aware-v1.3'),
            (replace(view.ambiguous_endpoints, radius_m=.034), 'object-aware-v1.3'),
        ):
            with self.subTest(semantics=semantics, endpoints=endpoints):
                fallback = replace(view, ambiguous_endpoints=endpoints)
                self.assertEqual(fallback.operational_semantics, semantics)
                self.assertTrue(gating.assess_footprint(fallback, (0., 0.), 0.).blocked)
                diag = gating.ambiguous_footprint_diagnostics(fallback, (0., 0.), 0.)
                self.assertEqual(diag.legacy_fallback_cells, 1)
                self.assertEqual(diag.aliased_clear_cells, 0)

    def test_missing_history_uses_window_counts_not_the_number_of_endpoints(self):
        windows = [[[.59, .2, .1], [.58, .2, .1]], [[.59, .2, .1]]]
        full = self.derive(windows)
        first_only = self.derive(windows[:1])
        partial = replace(full, ambiguous_endpoints=first_only.ambiguous_endpoints)
        self.assertEqual(full.ambiguous_occupied_votes.sum(), 2)
        self.assertEqual(partial.ambiguous_endpoints.complete_vote_counts.sum(), 1)
        self.assertTrue(gating.assess_footprint(partial, (0., 0.), 0.).blocked)
        self.assertFalse(gating.assess_footprint(full, (0., 0.), 0.).blocked)
        self.assertEqual(gating.ambiguous_footprint_diagnostics(
            partial, (0., 0.), 0.).legacy_fallback_cells, 1)

    def test_partially_retained_single_group_is_not_claimed_complete(self):
        full = self.derive([[[.59, .2, .1], [.545, .2, .1]]])
        sidecar = full.ambiguous_endpoints
        partial = replace(sidecar, points_xy=sidecar.points_xy[:1],
                          observation_indices=sidecar.observation_indices[:1],
                          row_indices=sidecar.row_indices[:1], cell_ids=sidecar.cell_ids[:1],
                          complete_vote_counts=np.zeros(self.grid.shape, dtype=int))
        view = replace(full, ambiguous_endpoints=partial)
        self.assertTrue(gating.assess_footprint(view, (0., 0.), 0.).blocked)
        self.assertEqual(gating.ambiguous_footprint_diagnostics(
            view, (0., 0.), 0.).legacy_fallback_cells, 1)

    def test_partial_history_keeps_known_neighbor_disk_as_well_as_cell_fallback(self):
        full = self.derive([[[0., .415, .1]], [[.59, .2, .1]]])
        first = self.derive([[[0., .415, .1]]])
        view = replace(full, ambiguous_endpoints=first.ambiguous_endpoints)
        diag = gating.ambiguous_footprint_diagnostics(view, (0., 0.), 0.)
        self.assertEqual(diag.intersecting_endpoint_indices, (0,))
        self.assertEqual(diag.legacy_fallback_cells, 1)
        self.assertTrue(gating.assess_footprint(view, (0., 0.), 0.).blocked)

    def test_later_ground_never_clears_historical_endpoint_ambiguity(self):
        rows, cols = np.indices(self.grid.shape)
        ground = np.column_stack((-2. + (cols.ravel() + .5) * .1,
                                  -2. + (rows.ravel() + .5) * .1,
                                  np.zeros(rows.size)))
        view = self.derive([[[.54, .2, .1]], ground, ground])
        result = gating.assess_footprint(view, (0., 0.), 0.)
        self.assertTrue(result.blocked)
        self.assertTrue(result.ground_supported)
        self.assertEqual(view.ambiguous_occupied_votes.sum(), 1)
        self.assertEqual(view.ambiguous_endpoints.complete_vote_counts.sum(), 1)
        np.testing.assert_allclose(view.ambiguous_endpoints.points_xy, [[.54, .2]])

    def test_opt_in_preserves_all_votes_and_ground_suppression(self):
        points = [[.59, .2, .1], [.58, .2, .1], [.59, .2, 0.],
                  [1.5, 1.5, .1], [1.5, 1.5, 0.], [.1, .2, .1]]
        labels = [np.array([1, 1, 0, 2, 0, 0])] * 2
        legacy = self.derive([points, points], labels=labels, retain=False)
        view = self.derive([points, points], labels=labels)
        for name in ('environment_occupied_votes', 'ambiguous_occupied_votes',
                     'target_occupied_votes', 'ground_votes'):
            np.testing.assert_array_equal(getattr(legacy, name), getattr(view, name))
        np.testing.assert_array_equal(view.ambiguous_endpoints.complete_vote_counts,
                                      view.ambiguous_occupied_votes)
        self.assertEqual(view.ambiguous_endpoints.complete_vote_counts.sum(), 2)
        self.assertEqual(len(view.ambiguous_endpoints.points_xy), 4)

    def test_original_source_ids_survive_all_filters_and_target_downgrade(self):
        transform = np.eye(4)
        transform[:3, 3] = [.25, .35, 2.]
        points = np.array([[0, 0, 0], [0, 0, -.2], [0, 0, -40], [np.nan, 0, 0],
                           [0, 0, -2], [0, 0, -1], [4, 0, -2], [0, 0, -1.97],
                           [.34, -.15, -1.9], [1.25, 1.15, -1.9], [.33, -.15, -1.9]])
        valid = np.ones(len(points), dtype=bool)
        valid[5] = False
        observation = PointCloudObservation(points, 'lidar', 2., transform, valid)
        labels = np.full(len(points), gating.OccupiedClass.AMBIGUOUS, dtype=int)
        labels[9:] = gating.OccupiedClass.TARGET
        view = gating.derive_operational_evidence(
            self.grid, [cloud([], 1.), observation], self.target,
            labels=[np.array([], dtype=int), labels], retain_ambiguous_endpoints=True,
        )
        np.testing.assert_array_equal(view.ambiguous_endpoints.observation_indices, [1, 1])
        np.testing.assert_array_equal(view.ambiguous_endpoints.row_indices, [8, 10])
        np.testing.assert_allclose(view.ambiguous_endpoints.points_xy, [[.59, .2], [.58, .2]])
        self.assertEqual(view.target_occupied_votes.sum(), 1)

    def test_environment_target_and_clipping_behavior_are_unchanged(self):
        environment_view = self.derive([[[.59, .2, .1]]],
                                       labels=[np.array([gating.OccupiedClass.ENVIRONMENT])])
        environment_result = gating.assess_footprint(environment_view, (0., 0.), 0.)
        self.assertEqual(environment_result.environment_cells, 1)
        self.assertTrue(environment_result.blocked)
        self.assertFalse(environment_result.target_collision)
        for label in (gating.OccupiedClass.ENVIRONMENT, gating.OccupiedClass.TARGET):
            with self.subTest(label=label):
                target = gating.PerceivedTarget((.6, .0, .1), 0.)
                labels = [np.array([label])]
                for retain in (False, True):
                    view = gating.derive_operational_evidence(
                        self.grid, [cloud([[.59, 0., .1]])], target, labels=labels,
                        retain_ambiguous_endpoints=retain)
                    result = gating.assess_footprint(view, (0., 0.), 0.)
                    self.assertTrue(result.blocked)
                    self.assertTrue(result.target_collision)
        view = self.derive([])
        self.assertEqual(view.operational_semantics, 'object-aware-v1.3')
        self.assertEqual(view.ambiguous_endpoints.points_xy.shape, (0, 2))
        for xy in ((-1.9, -1.9), (-5., -5.)):
            result = gating.assess_footprint(view, xy, 0.)
            self.assertTrue(result.footprint_clipped)
            self.assertFalse(result.ground_supported)

    def test_sidecar_is_detached_readonly_and_counts_are_consistent(self):
        view = self.derive([[[.59, .2, .1]]])
        sidecar = view.ambiguous_endpoints
        points = sidecar.points_xy.copy()
        copy = replace(sidecar, points_xy=points)
        points[:] = 99
        np.testing.assert_allclose(copy.points_xy, [[.59, .2]])
        for name in ('points_xy', 'observation_indices', 'row_indices', 'cell_ids',
                     'complete_vote_counts'):
            self.assertFalse(getattr(copy, name).flags.writeable)
        with self.assertRaises(FrozenInstanceError):
            copy.profile = 'other'
        for changes in (
            dict(points_xy=np.array([[np.nan, 0.]])), dict(points_xy=np.zeros((1, 3))),
            dict(row_indices=np.array([], dtype=int)), dict(row_indices=np.array([-1])),
            dict(cell_ids=np.array([self.grid.width_cells * self.grid.height_cells])),
            dict(complete_vote_counts=np.full(self.grid.shape, -1)),
            dict(complete_vote_counts=sidecar.complete_vote_counts * 2),
            dict(radius_m=-.01), dict(radius_m=np.nan),
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(sidecar, **changes)
        with self.assertRaises(ValueError):
            replace(view, ambiguous_occupied_votes=np.zeros(self.grid.shape, dtype=int))
        with self.assertRaises(ValueError):
            replace(view, ambiguous_endpoints=replace(sidecar, points_xy=np.array([[.1, .1]])))


if __name__ == '__main__':
    unittest.main()
