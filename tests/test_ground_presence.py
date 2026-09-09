"""Measured ground presence never erases the independent operational blocker."""

from dataclasses import replace
import unittest
import numpy as np

from environment_belief import EnvironmentGridSpec
from operational_gating import OccupiedClass, PerceivedTarget, assess_footprint, derive_operational_evidence
from task_relevant_uncertainty.geometry import FootprintSpec
from test_operational_gating import cloud


class GroundPresenceTests(unittest.TestCase):
    def setUp(self):
        self.grid = EnvironmentGridSpec((0., 0.), 10, 10)
        self.target = PerceivedTarget((.9, .9, .0575), 0.)
        self.xy, self.footprint = (.525, .525), FootprintSpec(.015, .015)

    def view(self, occupied=(.59, .59, .10), label=OccupiedClass.AMBIGUOUS,
             ground=True, windows=2, presence=True):
        points = ([occupied] if occupied is not None else []) + ([[.53, .53, 0.]] if ground else [])
        labels = ([label] if occupied is not None else []) + ([OccupiedClass.ENVIRONMENT] if ground else [])
        observations = [cloud(points, stamp) for stamp in range(1, windows + 1)]
        return derive_operational_evidence(
            self.grid, observations, self.target, labels=[np.array(labels, dtype=int)] * windows,
            retain_ambiguous_endpoints=True, retain_ground_presence=presence)

    def assess(self, view):
        return assess_footprint(view, self.xy, 0., self.footprint)

    def test_coarse_alias_does_not_suppress_real_ground_presence(self):
        new, old = self.view(), self.view(presence=False)
        self.assertEqual(new.operational_semantics, 'object-aware-v1.4')
        self.assertEqual(old.operational_semantics, 'object-aware-v1.3')
        self.assertIsNone(old.ground_presence_votes)
        self.assertEqual(new.ground_presence_votes[5, 5], 2)
        self.assertFalse(new.ground_presence_votes.flags.writeable)
        for name in ('ground_votes', 'ambiguous_occupied_votes', 'target_occupied_votes',
                     'environment_occupied_votes'):
            np.testing.assert_array_equal(getattr(new, name), getattr(old, name))
        self.assertEqual(new.ground_votes[5, 5], 0)
        self.assertFalse(self.assess(new).blocked)
        self.assertTrue(self.assess(new).ground_supported)
        self.assertFalse(self.assess(old).ground_supported)

    def test_true_ambiguous_hit_remains_blocked_with_positive_ground(self):
        view = self.view(occupied=(.53, .53, .10))
        self.assertTrue(self.assess(view).ground_supported)
        self.assertTrue(self.assess(view).blocked)

    def test_environment_remains_cell_conservative(self):
        view = self.view(label=OccupiedClass.ENVIRONMENT)
        self.assertEqual(view.ground_presence_votes[5, 5], 2)
        self.assertTrue(self.assess(view).blocked)

    def test_continuous_target_collision_remains_blocked(self):
        view = self.view(occupied=None)
        view = replace(view, target=PerceivedTarget((.525, .525, .0575), 0.))
        self.assertTrue(self.assess(view).ground_supported)
        self.assertTrue(self.assess(view).target_collision)
        self.assertTrue(self.assess(view).blocked)

    def test_occupied_only_and_one_ground_window_cannot_confirm(self):
        for view in (self.view(ground=False), self.view(windows=1)):
            self.assertFalse(self.assess(view).ground_supported)
            self.assertEqual(self.assess(view).ground_missing_cells, 1)

    def test_duplicates_are_one_vote_and_invalid_returns_are_not_evidence(self):
        points = [[.53, .53, 0.], [.53, .53, 0.], [.53, .53, 0.]]
        observations = [cloud(points, 1.), cloud(points, 2., valid_return=[False] * 3)]
        view = derive_operational_evidence(self.grid, observations, self.target,
                                          retain_ground_presence=True)
        self.assertEqual(view.ground_presence_votes[5, 5], 1)
        self.assertFalse(self.assess(view).ground_supported)

    def test_missing_profile_still_blocks_even_when_presence_is_retained(self):
        view = self.view()
        view = replace(view, ambiguous_endpoints=replace(view.ambiguous_endpoints, profile=None))
        self.assertTrue(self.assess(view).ground_supported)
        self.assertTrue(self.assess(view).blocked)

    def test_presence_counts_require_valid_shape_and_integer_nonnegative_values(self):
        view = self.view()
        for bad in (np.zeros((1, 1), dtype=int), np.full(self.grid.shape, -1),
                    np.zeros(self.grid.shape, dtype=float)):
            with self.assertRaises(ValueError):
                replace(view, ground_presence_votes=bad)
