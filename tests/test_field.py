import copy
import math
import unittest

import numpy as np

from reachability_guided_aerial_perception import (
    CellState,
    FieldConfig,
    FieldStatus,
    GraspTCP,
    GridSpec,
    build_field,
    build_field_from_result,
    candidate_relevance,
)


def candidate(x=0.0, y=0.0, yaw=0.0, margin=0.25, **overrides):
    value = {
        "candidate_id": f"candidate-{yaw}",
        "bunker_x": x,
        "bunker_y": y,
        "bunker_yaw": yaw,
        "rm4d_reachable": True,
        "ik_valid": True,
        "collision_free": True,
        "footprint_collision": False,
        "valid": True,
        "joint_margin_rad": margin,
        "fk_position_residual_m": 0.01,
        "fk_orientation_residual_rad": 0.02,
        "rejection_reason": None,
    }
    value.update(overrides)
    return value


def result(candidates, *, inverse=None, deduplicated=None, validation_limit=None, valid=None):
    if inverse is None:
        inverse = len(candidates)
    if deduplicated is None:
        deduplicated = len(candidates)
    if validation_limit is None:
        validation_limit = len(candidates)
    if valid is None:
        valid = sum(candidate_relevance(item, FieldConfig()) > 0 for item in candidates)
    return {
        "schema_version": 1,
        "frame_id": "map",
        "grasp_id": "g",
        "summary": {
            "inverse_reachable": inverse,
            "deduplicated": deduplicated,
            "validation_limit": validation_limit,
            "evaluated": len(candidates),
            "valid": valid,
        },
        "evaluated_candidates": candidates,
        "candidates": [],
    }


class CandidateRelevanceTests(unittest.TestCase):
    def test_score_is_margin_only_and_residual_independent(self):
        config = FieldConfig()
        first = candidate(margin=0.25, fk_position_residual_m=0.0, fk_orientation_residual_rad=0.0)
        second = candidate(margin=0.25, fk_position_residual_m=999.0, fk_orientation_residual_rad=math.pi)
        self.assertEqual(candidate_relevance(first, config), 0.5)
        self.assertEqual(candidate_relevance(first, config), candidate_relevance(second, config))

    def test_exact_minimum_margin_is_positive_and_saturation_clips(self):
        config = FieldConfig()
        self.assertEqual(candidate_relevance(candidate(margin=0.01), config), 0.02)
        self.assertEqual(candidate_relevance(candidate(margin=0.5), config), 1.0)
        self.assertEqual(candidate_relevance(candidate(margin=2.0), config), 1.0)

    def test_every_validity_gate_is_required(self):
        fields = (
            "rm4d_reachable",
            "ik_valid",
            "collision_free",
            "footprint_collision",
            "valid",
        )
        for field in fields:
            with self.subTest(field=field):
                item = candidate()
                item[field] = True if field == "footprint_collision" else False
                self.assertEqual(candidate_relevance(item, FieldConfig()), 0.0)
        for margin in (None, math.nan, 0.009):
            with self.subTest(margin=margin):
                self.assertEqual(candidate_relevance(candidate(margin=margin), FieldConfig()), 0.0)


class FieldBuilderTests(unittest.TestCase):
    def setUp(self):
        self.grasp = GraspTCP("g", "map", (0.0, 0.0, 0.4), (0, 0, 0, 1))
        self.grid = GridSpec.centered((0.0, 0.0), 0.4, 0.4, 0.2)

    def test_default_grid_is_three_meters_centered_on_grasp(self):
        field = build_field_from_result(self.grasp, result([] , inverse=0, deduplicated=0, validation_limit=0, valid=0))
        self.assertEqual(field.grid.center_xy, (0.0, 0.0))
        self.assertEqual((field.grid.width_m, field.grid.height_m, field.grid.resolution_m), (3.0, 3.0, 0.1))
        self.assertEqual(field.grid.shape, (30, 30))

    def test_result_requires_schema_version_one(self):
        bad = result([], inverse=0, deduplicated=0, validation_limit=0, valid=0)
        bad["schema_version"] = 999
        with self.assertRaises(ValueError):
            build_field_from_result(self.grasp, bad, self.grid)

    def test_yaw_envelope_chooses_highest_margin_and_diagnostics(self):
        low = candidate(0.01, 0.01, yaw=0.4, margin=0.2, fk_position_residual_m=0.1, fk_orientation_residual_rad=0.2)
        high = candidate(0.01, 0.01, yaw=1.2, margin=0.4, fk_position_residual_m=0.3, fk_orientation_residual_rad=0.4)
        field = build_field_from_result(self.grasp, result([low, high]), self.grid)
        cell = self.grid.cell_index(0.01, 0.01)
        assert cell is not None
        self.assertEqual(field.relevance[cell], 0.8)
        self.assertEqual(field.best_yaw[cell], 1.2)
        self.assertEqual(field.best_joint_margin_rad[cell], 0.4)
        self.assertEqual(field.best_fk_position_residual_m[cell], 0.3)
        self.assertEqual(field.best_fk_orientation_residual_rad[cell], 0.4)
        self.assertEqual(field.evaluated_count[cell], 2)
        self.assertEqual(field.feasible_count[cell], 2)
        self.assertEqual(field.cell_state[cell], CellState.HIGH)

    def test_equal_score_keeps_first_candidate(self):
        first = candidate(0.01, 0.01, yaw=0.4, margin=0.2, fk_position_residual_m=0.1)
        second = candidate(0.01, 0.01, yaw=1.2, margin=0.2, fk_position_residual_m=0.3)
        field = build_field_from_result(self.grasp, result([first, second]), self.grid)
        cell = self.grid.cell_index(0.01, 0.01)
        assert cell is not None
        self.assertEqual(field.best_yaw[cell], 0.4)
        self.assertEqual(field.best_fk_position_residual_m[cell], 0.1)

    def test_invalid_only_cell_is_infeasible_and_empty_cell_unassessed(self):
        rejected = candidate(0.01, 0.01, margin=0.2, valid=False)
        field = build_field_from_result(self.grasp, result([rejected], valid=0), self.grid)
        rejected_cell = self.grid.cell_index(0.01, 0.01)
        assert rejected_cell is not None
        self.assertEqual(field.relevance[rejected_cell], 0.0)
        self.assertEqual(field.cell_state[rejected_cell], CellState.INFEASIBLE)
        self.assertTrue(np.isnan(field.best_yaw[rejected_cell]))
        empty_cell = self.grid.cell_index(-0.19, 0.19)
        assert empty_cell is not None
        self.assertTrue(np.isnan(field.relevance[empty_cell]))
        self.assertEqual(field.cell_state[empty_cell], CellState.UNASSESSED)

    def test_null_diagnostics_are_nan_and_out_of_grid_candidates_are_ignored(self):
        inside = candidate(0.01, 0.01, fk_position_residual_m=None, fk_orientation_residual_rad=None)
        outside = candidate(4.0, 4.0, margin=0.5)
        field = build_field_from_result(self.grasp, result([inside, outside]), self.grid)
        cell = self.grid.cell_index(0.01, 0.01)
        assert cell is not None
        self.assertTrue(np.isnan(field.best_fk_position_residual_m[cell]))
        self.assertTrue(np.isnan(field.best_fk_orientation_residual_rad[cell]))
        self.assertEqual(field.coverage.evaluated_candidates, 2)
        self.assertEqual(int(field.evaluated_count.sum()), 1)

    def test_coverage_fraction_and_truncation_are_from_summary_and_raster(self):
        items = [candidate(0.01, 0.01, margin=0.2), candidate(-0.19, 0.19, margin=0.0, valid=False)]
        field = build_field_from_result(self.grasp, result(items, inverse=9, deduplicated=8, validation_limit=2, valid=1), self.grid)
        coverage = field.coverage
        self.assertEqual(coverage.inverse_reachable, 9)
        self.assertEqual(coverage.deduplicated_candidates, 8)
        self.assertEqual(coverage.validation_limit, 2)
        self.assertEqual(coverage.evaluated_candidates, 2)
        self.assertEqual(coverage.valid_candidates, 1)
        self.assertEqual(coverage.evaluated_cells, 2)
        self.assertEqual(coverage.total_cells, 4)
        self.assertEqual(coverage.candidate_validation_fraction, 0.25)
        self.assertEqual(coverage.assessed_cell_fraction, 0.5)
        self.assertTrue(coverage.validation_truncated)

    def test_no_inverse_reachable_is_distinct_and_all_cells_unassessed(self):
        field = build_field_from_result(self.grasp, result([], inverse=0, deduplicated=0, validation_limit=0, valid=0), self.grid)
        self.assertEqual(field.status, FieldStatus.NO_INVERSE_REACHABLE)
        self.assertTrue(np.isnan(field.relevance).all())
        self.assertTrue((field.cell_state == CellState.UNASSESSED).all())
        self.assertTrue((field.evaluated_count == 0).all())
        self.assertTrue((field.feasible_count == 0).all())
        self.assertTrue(np.isnan(field.best_yaw).all())
        self.assertTrue(np.isnan(field.best_joint_margin_rad).all())
        self.assertTrue(np.isnan(field.best_fk_position_residual_m).all())
        self.assertTrue(np.isnan(field.best_fk_orientation_residual_rad).all())

    def test_inverse_reachable_with_only_invalid_candidates_is_partial_and_infeasible(self):
        item = candidate(0.01, 0.01, valid=False)
        field = build_field_from_result(self.grasp, result([item], valid=0), self.grid)
        cell = self.grid.cell_index(0.01, 0.01)
        assert cell is not None
        self.assertEqual(field.status, FieldStatus.PARTIALLY_ASSESSED)
        self.assertEqual(field.cell_state[cell], CellState.INFEASIBLE)

    def test_positive_subthreshold_score_is_low(self):
        item = candidate(0.01, 0.01, margin=0.2)
        field = build_field_from_result(self.grasp, result([item]), self.grid)
        cell = self.grid.cell_index(0.01, 0.01)
        assert cell is not None
        self.assertEqual(field.relevance[cell], 0.4)
        self.assertEqual(field.cell_state[cell], CellState.LOW)

    def test_malformed_result_and_in_grid_candidate_pose_raise(self):
        for bad in ({}, {"frame_id": "map"}, {**result([]), "summary": {}}):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    build_field_from_result(self.grasp, bad, self.grid)
        malformed = result([candidate(0.0, 0.0)])
        malformed["evaluated_candidates"][0]["bunker_x"] = math.nan
        with self.assertRaises(ValueError):
            build_field_from_result(self.grasp, malformed, self.grid)

    def test_evaluated_candidates_require_complete_typed_evidence(self):
        required = (
            "candidate_id", "bunker_x", "bunker_y", "bunker_yaw",
            "rm4d_reachable", "ik_valid", "collision_free", "footprint_collision",
            "valid", "joint_margin_rad", "fk_position_residual_m",
            "fk_orientation_residual_rad", "rejection_reason",
        )
        for name in required:
            item = candidate()
            del item[name]
            bad = result([item], valid=0)
            with self.subTest(missing=name):
                with self.assertRaises(ValueError):
                    build_field_from_result(self.grasp, bad, self.grid)

        for name in ("rm4d_reachable", "ik_valid", "collision_free", "footprint_collision", "valid"):
            item = candidate(**{name: 1})
            with self.subTest(non_bool=name):
                with self.assertRaises(ValueError):
                    build_field_from_result(self.grasp, result([item], valid=0), self.grid)

        for name, value in (
            ("joint_margin_rad", -0.1),
            ("fk_position_residual_m", math.inf),
            ("fk_orientation_residual_rad", -0.1),
        ):
            item = candidate(**{name: value})
            with self.subTest(bad_value=name):
                with self.assertRaises(ValueError):
                    build_field_from_result(self.grasp, result([item], valid=0), self.grid)

        for value in ("", "  ", 1):
            item = candidate(candidate_id=value)
            with self.subTest(candidate_id=value):
                with self.assertRaises(ValueError):
                    build_field_from_result(self.grasp, result([item], valid=0), self.grid)

    def test_builders_do_not_mutate_grasp_or_result(self):
        item = candidate(0.01, 0.01)
        original_result = result([item])
        result_copy = copy.deepcopy(original_result)
        request_before = copy.deepcopy(self.grasp.as_request())
        build_field_from_result(self.grasp, original_result, self.grid)
        self.assertEqual(original_result, result_copy)
        self.assertEqual(self.grasp.as_request(), request_before)


class LiveBuilderTests(unittest.TestCase):
    def test_live_builder_calls_exactly_once_and_ignores_ranked_candidates(self):
        grasp = GraspTCP("g", "map", (0.0, 0.0, 0.4), (0, 0, 0, 1))
        evaluated = candidate(0.0, 0.0, yaw=0.7, margin=0.25)
        malicious_top = candidate(0.0, 0.0, yaw=1.5, margin=0.5)
        returned = result([evaluated])
        returned["candidates"] = [malicious_top]

        class FakeAPI:
            def __init__(self):
                self.calls = []

            def plan(self, request, top_k=None):
                self.calls.append((copy.deepcopy(request), top_k))
                return returned

        api = FakeAPI()
        returned_copy = copy.deepcopy(returned)
        request_before = copy.deepcopy(grasp.as_request())
        field = build_field(grasp, api, GridSpec.centered((0, 0), 0.2, 0.2, 0.1))
        self.assertEqual(api.calls, [(grasp.as_request(), 1)])
        self.assertEqual(returned, returned_copy)
        self.assertEqual(grasp.as_request(), request_before)
        cell = field.grid.cell_index(0, 0)
        assert cell is not None
        self.assertEqual(field.best_yaw[cell], 0.7)
        self.assertEqual(field.relevance[cell], 0.5)


if __name__ == "__main__":
    unittest.main()
