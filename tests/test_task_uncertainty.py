from dataclasses import replace
import unittest

import numpy as np

from environment_belief import EnvironmentBeliefMapper, EnvironmentGridSpec, EnvironmentState
from reachability_guided_aerial_perception import CellState, FieldStatus, GraspTCP, GridSpec, build_field_from_result
from task_relevant_uncertainty import build_task_uncertainty, PoseEnvironmentState, SupportState
from tests.test_environment_belief import observation
from tests.test_field import candidate, result


GRID = EnvironmentGridSpec((-1.5, -1.5), 30, 30)


def interest(candidates):
    return build_field_from_result(
        GraspTCP('g', 'map', (0, 0, 0.4), (0, 0, 0, 1)),
        result(candidates, validation_limit=256),
        GridSpec.centered((0, 0), 3, 3, 0.1),
    )


def ground_map(observations=2):
    mapper = EnvironmentBeliefMapper(GRID)
    rows, cols = np.indices(GRID.shape)
    points = np.column_stack((-1.5 + (cols.ravel() + 0.5) * 0.1,
                              -1.5 + (rows.ravel() + 0.5) * 0.1, np.zeros(900)))
    for stamp in range(observations):
        mapper.update(observation(points, stamp=stamp))
    return mapper.snapshot()


class TaskUncertaintyTests(unittest.TestCase):
    def setUp(self):
        self.a1 = interest([candidate(0.01, 0.01, margin=0.45)])
        self.a2 = EnvironmentBeliefMapper(GRID).snapshot()

    def test_high_unknown_projects_full_footprint_and_records_representative_source(self):
        field = build_task_uncertainty(self.a1, self.a2)
        supported = field.support_state == SupportState.SUPPORTED
        self.assertEqual(supported.sum(), 99)
        np.testing.assert_allclose(field.nominal_task_relevance[supported], 0.9)
        np.testing.assert_allclose(field.task_relevant_uncertainty[supported], 0.9)
        self.assertTrue(np.all(field.best_operational_source[supported] == 465))
        pose = field.poses[0]
        np.testing.assert_allclose(pose.xy, [0.05, 0.05])  # Not candidate (.01,.01).
        self.assertEqual(pose.source_id, 465)
        self.assertEqual(pose.unknown_cells, 99)
        self.assertFalse(pose.blocked)
        self.assertEqual(pose.environment_state, PoseEnvironmentState.UNCONFIRMED)
        self.assertEqual(set(pose.covered_environment_cells), set(np.flatnonzero(supported)))
        self.assertTrue(np.isnan(field.task_relevant_uncertainty[0, 0]))
        self.assertEqual(field.best_operational_source[0, 0], -1)

    def test_known_free_uses_nonzero_score_and_further_observations_lower_product(self):
        first = build_task_uncertainty(self.a1, ground_map(2))
        later = build_task_uncertainty(self.a1, ground_map(8))
        self.assertAlmostEqual(first.task_relevant_uncertainty[15, 15], 0.9 * np.exp(-1))
        self.assertAlmostEqual(later.task_relevant_uncertainty[15, 15], 0.9 * np.exp(-4))
        self.assertGreater(later.task_relevant_uncertainty[15, 15], 0)
        self.assertEqual(first.poses[0].environment_state, PoseEnvironmentState.OBSERVED_GROUND_SUPPORT)

    def test_low_unknown_stays_low(self):
        field = build_task_uncertainty(interest([candidate(0.01, 0.01, margin=0.1)]), self.a2)
        self.assertAlmostEqual(field.task_relevant_uncertainty[15, 15], 0.2)

    def test_off_center_occupied_blocks_entire_pose_without_changing_nominal(self):
        mapper = EnvironmentBeliefMapper(GRID)
        mapper.update(observation([[-0.45, 0.05, 0.1]]))
        occupied = mapper.snapshot()
        field = build_task_uncertainty(self.a1, occupied)
        footprint = field.nominal_support_count > 0
        self.assertTrue(field.poses[0].blocked)
        self.assertEqual(field.poses[0].environment_state, PoseEnvironmentState.A2_OCCUPIED_BLOCKED)
        self.assertEqual(occupied.state[15, 15], EnvironmentState.UNKNOWN)  # Base center not occupied.
        self.assertTrue(np.all(field.support_state[footprint] == SupportState.BLOCKED_ONLY))
        self.assertTrue(np.all(field.task_relevant_uncertainty[footprint] == 0))
        np.testing.assert_allclose(field.nominal_task_relevance[footprint], 0.9)
        self.assertEqual(self.a1.relevance[15, 15], 0.9)

    def test_overlapping_unblocked_lower_pose_survives_and_max_is_not_sum(self):
        a1 = interest([candidate(0.01, 0.01, margin=0.45), candidate(0.65, 0.05, margin=0.1)])
        mapper = EnvironmentBeliefMapper(GRID)
        mapper.update(observation([[-0.45, 0.05, 0.1]]))
        field = build_task_uncertainty(a1, mapper.snapshot())
        self.assertAlmostEqual(field.nominal_task_relevance[15, 18], 0.9)
        self.assertAlmostEqual(field.task_relevant_uncertainty[15, 18], 0.2)
        self.assertEqual(field.nominal_support_count[15, 18], 2)
        self.assertEqual(field.operational_support_count[15, 18], 1)
        self.assertEqual(field.best_nominal_source[15, 18], 465)
        self.assertEqual(field.best_operational_source[15, 18], 471)
        self.assertEqual(np.count_nonzero(field.support_state == SupportState.BLOCKED_ONLY), 54)
        self.assertEqual(np.count_nonzero(field.support_state == SupportState.SUPPORTED), 99)
        before = build_task_uncertainty(a1, self.a2)
        self.assertAlmostEqual(before.task_relevant_uncertainty[15, 18], 0.9)

    def test_ties_use_first_row_major_source_not_candidate_order(self):
        a1 = interest([candidate(0.65, 0.05, margin=0.45), candidate(0.01, 0.01, margin=0.45)])
        field = build_task_uncertainty(a1, self.a2)
        self.assertEqual(field.best_operational_source[15, 18], 465)
        self.assertEqual([p.source_id for p in field.poses], [465, 471])

    def test_a1_unassessed_infeasible_and_a2_unknown_remain_independent(self):
        a1 = interest([candidate(0.01, 0.01, margin=0.45),
                       candidate(1.05, 1.05, valid=False, ik_valid=False)])
        field = build_task_uncertainty(a1, ground_map())
        self.assertEqual(field.a1_cell_state[25, 25], CellState.INFEASIBLE)
        self.assertEqual(field.a1_cell_state[0, 0], CellState.UNASSESSED)
        self.assertEqual(field.environment_state[0, 0], EnvironmentState.FREE)
        self.assertEqual(field.support_state[0, 0], SupportState.NO_VALIDATED_SUPPORT)
        self.assertEqual(field.pose_environment_state[25, 25], PoseEnvironmentState.NOT_PROJECTED)
        self.assertTrue(np.isnan(field.task_relevant_uncertainty[25, 25]))
        # Environment support can extend into a base cell whose A1 is UNASSESSED.
        self.assertEqual(field.a1_cell_state[15, 18], CellState.UNASSESSED)
        self.assertEqual(field.support_state[15, 18], SupportState.SUPPORTED)
        self.assertEqual(len(field.poses), 1)

    def test_no_inverse_reachable_is_preserved_without_inventing_zero_relevance(self):
        field = build_task_uncertainty(interest([]), self.a2)
        self.assertEqual(field.a1_status, FieldStatus.NO_INVERSE_REACHABLE)
        self.assertEqual(field.poses, ())
        self.assertTrue(np.all(np.isnan(field.nominal_task_relevance)))
        self.assertTrue(np.all(field.support_state == SupportState.NO_VALIDATED_SUPPORT))
        self.assertEqual(field.a1_coverage.inverse_reachable, 0)

    def test_clipped_pose_is_unconfirmed_even_if_all_in_grid_cells_are_free(self):
        field = build_task_uncertainty(interest([candidate(-1.45, -1.45)]), ground_map())
        pose = field.poses[0]
        self.assertTrue(pose.footprint_clipped)
        self.assertFalse(pose.blocked)
        self.assertEqual(pose.unknown_cells, 0)
        self.assertEqual(pose.environment_state, PoseEnvironmentState.UNCONFIRMED)
        self.assertGreater(field.task_relevant_uncertainty[0, 0], 0)

    def test_requires_aligned_fixed_map_grid_without_resampling(self):
        for grid in (EnvironmentGridSpec((-1.4, -1.5), 30, 30),
                     EnvironmentGridSpec((-1.5, -1.5), 29, 30),
                     EnvironmentGridSpec((-1.5, -1.5), 30, 30, 0.2)):
            with self.subTest(grid=grid), self.assertRaises(ValueError):
                build_task_uncertainty(self.a1, EnvironmentBeliefMapper(grid).snapshot())

    def test_rejects_missing_yaw_and_invalid_environment_arrays(self):
        with self.assertRaises(ValueError):
            build_task_uncertainty(replace(self.a1, best_yaw=np.full(GRID.shape, np.nan)), self.a2)
        for name, value in (('unknown_score', np.full(GRID.shape, np.nan)),
                            ('unknown_score', np.full(GRID.shape, 1.1)),
                            ('state', np.zeros((2, 2))), ('state', np.full(GRID.shape, 7))):
            with self.subTest(name=name), self.assertRaises(ValueError):
                build_task_uncertainty(self.a1, replace(self.a2, **{name: value}))

    def test_output_arrays_are_detached_and_do_not_mutate_a1_a2(self):
        a1_before = self.a1.relevance.copy()
        a2_before = self.a2.state.copy()
        field = build_task_uncertainty(self.a1, self.a2)
        np.testing.assert_array_equal(self.a1.relevance, a1_before)
        np.testing.assert_array_equal(self.a2.state, a2_before)
        self.assertFalse(field.task_relevant_uncertainty.flags.writeable)
        field.a1_cell_state.setflags(write=True)
        field.a1_cell_state[:] = CellState.INFEASIBLE
        self.assertEqual(self.a1.cell_state[15, 15], CellState.HIGH)
        self.assertEqual(field.a1_coverage, self.a1.coverage)
        self.assertEqual(field.a1_status, self.a1.status)


if __name__ == '__main__':
    unittest.main()
