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


class TypeTests(unittest.TestCase):
    def test_enums_have_exact_wire_values(self):
        self.assertEqual(
            list(FieldStatus),
            [FieldStatus.PARTIALLY_ASSESSED, FieldStatus.NO_INVERSE_REACHABLE],
        )
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
            inverse_reachable=True,
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
            evaluated_count=2,
            feasible_count=1,
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
