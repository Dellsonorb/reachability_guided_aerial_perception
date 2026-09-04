import dataclasses
import math
import unittest

import numpy as np

from reachability_guided_aerial_perception.model import (
    AssessmentCoverage,
    CellState,
    FieldConfig,
    FieldStatus,
    GraspTCP,
    GridSpec,
    ManipulationInterestField,
    OccupancyGridPayload,
)


class GraspTCPTests(unittest.TestCase):
    def test_request_has_only_the_frozen_grasp_fields(self):
        grasp = GraspTCP("g", "map", (1, 2, 0.4), (0, 0, 0, 1))

        self.assertEqual(
            grasp.as_request(),
            {
                "grasp_id": "g",
                "frame_id": "map",
                "position_xyz": (1.0, 2.0, 0.4),
                "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0),
            },
        )
        self.assertEqual(
            set(grasp.as_request()),
            {"grasp_id", "frame_id", "position_xyz", "quaternion_xyzw"},
        )

    def test_grasp_requires_map_frame_and_unit_quaternion(self):
        with self.assertRaisesRegex(ValueError, "map"):
            GraspTCP("g", "world", (1, 2, 0.4), (0, 0, 0, 1))
        with self.assertRaisesRegex(ValueError, "unit"):
            GraspTCP("g", "map", (1, 2, 0.4), (0, 0, 0, 2))

    def test_grasp_rejects_blank_wrong_shape_and_nonfinite_values(self):
        for grasp_id in ("", "  "):
            with self.assertRaises(ValueError):
                GraspTCP(grasp_id, "map", (1, 2, 0.4), (0, 0, 0, 1))
        for position, quaternion in [
            ((1, 2), (0, 0, 0, 1)),
            ((1, 2, 0.4), (0, 0, 1)),
            ((1, math.inf, 0.4), (0, 0, 0, 1)),
            ((1, 2, 0.4), (0, 0, 0, math.nan)),
        ]:
            with self.assertRaises(ValueError):
                GraspTCP("g", "map", position, quaternion)


class GridSpecTests(unittest.TestCase):
    def test_centered_grid_geometry_and_cell_lookup(self):
        grid = GridSpec.centered((1, 2), 3, 3, 0.1)

        self.assertEqual(grid.origin_xy, (-0.5, 0.5))
        self.assertEqual(grid.shape, (30, 30))
        self.assertEqual(grid.cell_index(1.01, 2.01), (15, 15))
        self.assertIsNone(grid.cell_index(-0.51, 2.0))
        self.assertIsNone(grid.cell_index(2.5, 3.5))

    def test_grid_rejects_nonfinite_nonpositive_or_nondisivible_geometry(self):
        with self.assertRaises(ValueError):
            GridSpec.centered((0, 0), 0, 1, 0.1)
        with self.assertRaises(ValueError):
            GridSpec.centered((0, 0), 1, 1, math.nan)
        with self.assertRaises(ValueError):
            GridSpec.centered((math.inf, 0), 1, 1, 0.1)
        with self.assertRaises(ValueError):
            GridSpec.centered((0, 0), 1.01, 1, 0.1)

    def test_cell_index_uses_half_open_bounds_at_exact_upper_edge(self):
        grid = GridSpec.centered((0, 0), 0.6, 0.6, 0.1)
        self.assertEqual(grid.cell_index(grid.origin_x, grid.origin_y), (0, 0))
        self.assertEqual(grid.cell_index(grid.origin_x + 0.5, grid.origin_y + 0.5), (5, 5))
        self.assertIsNone(grid.cell_index(grid.origin_x + grid.width_m, grid.origin_y + 0.1))
        self.assertIsNone(grid.cell_index(grid.origin_x + 0.1, grid.origin_y + grid.height_m))


class TypeTests(unittest.TestCase):
    def test_enums_have_exact_wire_values(self):
        self.assertEqual(
            list(FieldStatus),
            [FieldStatus.PARTIALLY_ASSESSED, FieldStatus.NO_INVERSE_REACHABLE],
        )
        self.assertEqual(FieldStatus.PARTIALLY_ASSESSED.value, "PARTIALLY_ASSESSED")
        self.assertEqual(FieldStatus.NO_INVERSE_REACHABLE.value, "NO_INVERSE_REACHABLE")
        self.assertEqual(
            list(CellState),
            [
                CellState.UNASSESSED,
                CellState.INFEASIBLE,
                CellState.LOW,
                CellState.HIGH,
            ],
        )
        self.assertEqual(CellState.UNASSESSED.value, -1)
        self.assertEqual(CellState.INFEASIBLE.value, 0)
        self.assertEqual(CellState.LOW.value, 1)
        self.assertEqual(CellState.HIGH.value, 2)

    def test_field_config_is_frozen_with_validated_defaults(self):
        config = FieldConfig()
        self.assertEqual(config.minimum_joint_margin_rad, 0.01)
        self.assertEqual(config.joint_margin_saturation_rad, 0.5)
        self.assertEqual(config.high_relevance_threshold, 0.8)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            config.high_relevance_threshold = 0.7
        with self.assertRaises(ValueError):
            FieldConfig(minimum_joint_margin_rad=-0.1)
        with self.assertRaises(ValueError):
            FieldConfig(minimum_joint_margin_rad=0.5, joint_margin_saturation_rad=0.1)
        with self.assertRaises(ValueError):
            FieldConfig(high_relevance_threshold=math.inf)

    def test_frozen_field_records_validate_grid_shaped_arrays(self):
        grid = GridSpec.centered((0, 0), 0.2, 0.2, 0.1)
        coverage = AssessmentCoverage(
            inverse_reachable=1,
            deduplicated_candidates=4,
            validation_limit=4,
            evaluated_candidates=4,
            valid_candidates=2,
            evaluated_cells=2,
            total_cells=4,
            candidate_validation_fraction=1.0,
            assessed_cell_fraction=0.5,
            validation_truncated=False,
        )
        field = ManipulationInterestField(
            grasp_id="g",
            frame_id="map",
            status=FieldStatus.PARTIALLY_ASSESSED,
            grid=grid,
            relevance=np.zeros(grid.shape),
            cell_state=np.full(grid.shape, CellState.UNASSESSED, dtype=np.int8),
            evaluated_count=np.full(grid.shape, 2, dtype=np.int32),
            feasible_count=np.full(grid.shape, 1, dtype=np.int32),
            best_yaw=np.zeros(grid.shape),
            best_joint_margin_rad=np.zeros(grid.shape),
            best_fk_position_residual_m=np.zeros(grid.shape),
            best_fk_orientation_residual_rad=np.zeros(grid.shape),
            coverage=coverage,
        )
        self.assertEqual(field.relevance.shape, grid.shape)
        with self.assertRaises(ValueError):
            ManipulationInterestField(
                "g",
                "map",
                FieldStatus.PARTIALLY_ASSESSED,
                grid,
                np.zeros((1, 1)),
                np.zeros(grid.shape),
                0,
                0,
                np.zeros(grid.shape),
                np.zeros(grid.shape),
                np.zeros(grid.shape),
                np.zeros(grid.shape),
                coverage,
            )

    def test_coverage_requires_integer_inverse_count_and_consistent_invariants(self):
        valid = dict(
            inverse_reachable=30564,
            deduplicated_candidates=4,
            validation_limit=2,
            evaluated_candidates=2,
            valid_candidates=1,
            evaluated_cells=2,
            total_cells=4,
            candidate_validation_fraction=0.5,
            assessed_cell_fraction=0.5,
            validation_truncated=True,
        )
        coverage = AssessmentCoverage(**valid)
        self.assertEqual(coverage.inverse_reachable, 30564)
        with self.assertRaises(ValueError):
            AssessmentCoverage(**{**valid, "inverse_reachable": True})
        with self.assertRaises(ValueError):
            AssessmentCoverage(**{**valid, "inverse_reachable": -1})
        with self.assertRaises(ValueError):
            AssessmentCoverage(**{**valid, "valid_candidates": 3})
        with self.assertRaises(ValueError):
            AssessmentCoverage(**{**valid, "evaluated_candidates": 5})
        with self.assertRaises(ValueError):
            AssessmentCoverage(**{**valid, "evaluated_cells": 5})
        with self.assertRaises(ValueError):
            AssessmentCoverage(**{**valid, "candidate_validation_fraction": 0.6})
        with self.assertRaises(ValueError):
            AssessmentCoverage(**{**valid, "validation_truncated": False})
        with self.assertRaises(ValueError):
            AssessmentCoverage(**{**valid, "deduplicated_candidates": 0, "evaluated_candidates": 0,
                                  "candidate_validation_fraction": 0.5, "validation_truncated": False})

    def test_ndarray_records_own_read_only_array_copies(self):
        grid = GridSpec.centered((0, 0), 0.2, 0.2, 0.1)
        source_relevance = np.zeros(grid.shape)
        source_state = np.full(grid.shape, CellState.UNASSESSED, dtype=np.int8)
        source_evaluated = np.ones(grid.shape, dtype=np.int32)
        source_feasible = np.zeros(grid.shape, dtype=np.int32)
        source_diagnostic = np.zeros(grid.shape)
        coverage = AssessmentCoverage(1, 1, 1, 1, 0, 1, 4, 1.0, 0.25, False)
        field = ManipulationInterestField(
            "g", "map", FieldStatus.PARTIALLY_ASSESSED, grid,
            source_relevance, source_state, source_evaluated, source_feasible,
            source_diagnostic, source_diagnostic, source_diagnostic, source_diagnostic,
            coverage,
        )
        source_relevance[0, 0] = 1.0
        source_evaluated[0, 0] = 0
        self.assertEqual(field.relevance[0, 0], 0.0)
        self.assertEqual(field.evaluated_count[0, 0], 1)
        with self.assertRaises(ValueError):
            field.relevance[0, 0] = 1.0

        source_data = np.zeros((2, 2), dtype=np.int8)
        payload = OccupancyGridPayload("map", FieldStatus.PARTIALLY_ASSESSED, -0.1, -0.1, 0.1, 2, 2, source_data)
        source_data[0, 0] = 100
        self.assertEqual(payload.data[0, 0], 0)
        with self.assertRaises(ValueError):
            payload.data[0, 0] = 100

    def test_interest_field_arrays_have_minimal_semantic_validation(self):
        grid = GridSpec.centered((0, 0), 0.2, 0.2, 0.1)
        coverage = AssessmentCoverage(1, 1, 1, 1, 0, 1, 4, 1.0, 0.25, False)
        base = dict(
            grasp_id="g", frame_id="map", status=FieldStatus.PARTIALLY_ASSESSED, grid=grid,
            relevance=np.zeros(grid.shape),
            cell_state=np.full(grid.shape, CellState.UNASSESSED, dtype=np.int8),
            evaluated_count=np.ones(grid.shape, dtype=np.int32),
            feasible_count=np.zeros(grid.shape, dtype=np.int32),
            best_yaw=np.full(grid.shape, np.nan),
            best_joint_margin_rad=np.zeros(grid.shape),
            best_fk_position_residual_m=np.zeros(grid.shape),
            best_fk_orientation_residual_rad=np.zeros(grid.shape),
            coverage=coverage,
        )
        for name, value in (
            ("relevance", np.full(grid.shape, "x", dtype=object)),
            ("cell_state", np.full(grid.shape, 9, dtype=np.int8)),
            ("evaluated_count", np.ones(grid.shape, dtype=float)),
            ("evaluated_count", -np.ones(grid.shape, dtype=np.int32)),
            ("feasible_count", np.full(grid.shape, 2, dtype=np.int32)),
            ("best_yaw", np.full(grid.shape, np.inf)),
            ("best_joint_margin_rad", np.full(grid.shape, "x", dtype=object)),
        ):
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    ManipulationInterestField(**{**base, name: value})
        with self.assertRaises(ValueError):
            ManipulationInterestField(
                "g",
                "map",
                FieldStatus.PARTIALLY_ASSESSED,
                grid,
                np.zeros(grid.shape),
                np.zeros(grid.shape),
                np.zeros((1, 1)),
                np.zeros(grid.shape),
                np.zeros(grid.shape),
                np.zeros(grid.shape),
                np.zeros(grid.shape),
                np.zeros(grid.shape),
                coverage,
            )

    def test_occupancy_payload_is_frozen(self):
        payload = OccupancyGridPayload(
            frame_id="map",
            field_status=FieldStatus.PARTIALLY_ASSESSED,
            origin_x=-1.0,
            origin_y=-1.0,
            resolution=0.1,
            width=2,
            height=2,
            data=np.zeros((2, 2), dtype=np.int8),
        )
        self.assertEqual(payload.data.shape, (2, 2))
        with self.assertRaises(dataclasses.FrozenInstanceError):
            payload.width = 3


if __name__ == "__main__":
    unittest.main()
