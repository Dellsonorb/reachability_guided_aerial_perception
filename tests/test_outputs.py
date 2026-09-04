import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from reachability_guided_aerial_perception import (
    CellState,
    FieldConfig,
    GraspTCP,
    GridSpec,
    build_field_from_result,
    candidate_relevance,
    field_summary,
    save_field_bundle,
    to_occupancy_grid_payload,
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
    if value["valid"] is False and value["rejection_reason"] is None:
        value["rejection_reason"] = "invalid"
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
    rejected = {}
    for item in candidates:
        if item.get("valid") is not True:
            reason = item.get("rejection_reason")
            if isinstance(reason, str):
                rejected[reason] = rejected.get(reason, 0) + 1
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
            "rejected_by_reason": rejected,
        },
        "evaluated_candidates": candidates,
        "candidates": [],
    }


class OutputTests(unittest.TestCase):
    def setUp(self):
        self.grid = GridSpec.centered((0.0, 0.0), 0.4, 0.4, 0.2)
        self.grasp = GraspTCP("g", "map", (0.0, 0.0, 0.4), (0, 0, 0, 1))

    def test_payload_is_exact_nested_wire_dict_and_flattens_in_c_row_major_order(self):
        field = build_field_from_result(
            self.grasp,
            result(
                [
                    candidate(0.01, -0.19, margin=0.0, valid=False),
                    candidate(-0.19, 0.01, margin=0.25),
                    candidate(0.01, 0.01, margin=0.5),
                ],
                valid=2,
            ),
            self.grid,
        )
        payload = to_occupancy_grid_payload(field)
        self.assertEqual(
            payload,
            {
                "header": {"frame_id": "map"},
                "info": {
                    "resolution": 0.2,
                    "width": 2,
                    "height": 2,
                    "origin": {
                        "position": {"x": -0.2, "y": -0.2, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                },
                "field_status": "PARTIALLY_ASSESSED",
                "data": [-1, 0, 50, 100],
            },
        )

    def test_no_inverse_payload_is_all_unassessed(self):
        field = build_field_from_result(
            self.grasp,
            result([], inverse=0, deduplicated=0, validation_limit=0, valid=0),
            self.grid,
        )
        payload = to_occupancy_grid_payload(field)
        self.assertEqual(payload["field_status"], "NO_INVERSE_REACHABLE")
        self.assertEqual(payload["data"], [-1, -1, -1, -1])

    def test_summary_is_json_safe_and_contains_semantics_coverage_and_counts(self):
        field = build_field_from_result(
            self.grasp,
            result([candidate(-0.19, -0.19, margin=0.25), candidate(0.01, -0.19, margin=0.0, valid=False)], valid=1),
            self.grid,
        )
        summary = field_summary(field, FieldConfig())
        self.assertEqual(summary["schema_version"], 1)
        self.assertEqual(summary["field_type"], "validated_manipulation_interest")
        self.assertEqual(summary["status"], "PARTIALLY_ASSESSED")
        self.assertEqual(summary["relevance_semantics"], "joint_margin_only")
        self.assertEqual(
            summary["scoring"],
            {
                "minimum_margin_rad": 0.01,
                "saturation_margin_rad": 0.5,
                "high_threshold": 0.8,
                "residual_role": "diagnostic_only",
            },
        )
        self.assertEqual(
            summary["grid"],
            {
                "origin_x_m": -0.2,
                "origin_y_m": -0.2,
                "resolution_m": 0.2,
                "width": 2,
                "height": 2,
                "shape": [2, 2],
                "convention": "row-major [y, x]; q=(base_link x, base_link y) in map",
            },
        )
        self.assertEqual(summary["grid"]["shape"], [2, 2])
        self.assertEqual(
            set(summary["coverage"]),
            {"inverse_reachable", "deduplicated", "validation_limit", "evaluated", "valid", "rejected_by_reason", "assessed_cells", "total_cells", "coverage_fraction"},
        )
        self.assertEqual(summary["coverage"]["evaluated"], 2)
        self.assertEqual(summary["coverage"]["rejected_by_reason"], {"invalid": 1})
        self.assertEqual(summary["cells"], {"unassessed": 2, "infeasible": 1, "low": 1, "high": 0})
        encoded = json.dumps(summary, allow_nan=False)
        self.assertNotIn("NaN", encoded)

    def test_summary_preserves_rejected_by_reason_coverage(self):
        rejected = candidate(0.01, -0.19, margin=0.0, valid=False, rejection_reason="joint_margin")
        field = build_field_from_result(self.grasp, result([rejected], valid=0), self.grid)
        summary = field_summary(field, FieldConfig())
        self.assertEqual(summary["coverage"]["rejected_by_reason"], {"joint_margin": 1})

    def test_save_rejects_candidate_count_validity_config_and_raster_contradictions(self):
        item = candidate(-0.19, -0.19)
        field = build_field_from_result(self.grasp, result([item], valid=1), self.grid)
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(ValueError):
                save_field_bundle(field, [], temporary)
            with self.assertRaises(ValueError):
                save_field_bundle(field, [candidate(-0.19, -0.19, valid=False)], temporary)
            with self.assertRaises(ValueError):
                save_field_bundle(field, [item], temporary, config=FieldConfig(minimum_joint_margin_rad=0.3))
            with self.assertRaises(ValueError):
                save_field_bundle(field, [candidate(0.01, 0.01)], temporary)

    def test_save_requires_all_candidate_diagnostic_columns(self):
        item = candidate(-0.19, -0.19)
        field = build_field_from_result(self.grasp, result([item], valid=1), self.grid)
        del item["candidate_id"]
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(ValueError):
                save_field_bundle(field, [item], temporary)

    def test_save_bundle_has_exact_files_roundtrip_arrays_and_csv(self):
        items = [
            candidate(-0.19, -0.19, yaw=0.1, margin=0.25),
            candidate(0.01, -0.19, yaw=0.2, margin=0.5, valid=False, fk_position_residual_m=None),
        ]
        field = build_field_from_result(self.grasp, result(items, valid=1), self.grid)
        with tempfile.TemporaryDirectory() as temporary:
            output = save_field_bundle(field, items, temporary)
            self.assertEqual(set(output), {"field", "summary", "candidates"})
            self.assertEqual({path.name for path in output.values()}, {"field.npz", "summary.json", "candidate_diagnostics.csv"})
            with np.load(output["field"]) as arrays:
                self.assertEqual(
                    set(arrays.files),
                    {"relevance", "state", "evaluated_count", "valid_count", "frame_id", "grasp_id", "status", "origin_x_m", "origin_y_m", "resolution_m", "width", "height"},
                )
                np.testing.assert_array_equal(arrays["relevance"], field.relevance)
                np.testing.assert_array_equal(arrays["state"], field.cell_state)
                np.testing.assert_array_equal(arrays["evaluated_count"], field.evaluated_count)
                np.testing.assert_array_equal(arrays["valid_count"], field.feasible_count)
                self.assertEqual(str(arrays["status"]), "PARTIALLY_ASSESSED")
                self.assertEqual(str(arrays["grasp_id"]), "g")
            parsed = json.loads(Path(output["summary"]).read_text())
            self.assertEqual(parsed["status"], "PARTIALLY_ASSESSED")
            with Path(output["candidates"]).open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["field_relevance"], "0.5")
            self.assertEqual(rows[1]["fk_position_residual_m"], "")
            self.assertEqual(
                list(rows[0]),
                [
                    "candidate_id", "bunker_x", "bunker_y", "bunker_yaw", "rm4d_reachable", "ik_valid",
                    "collision_free", "footprint_collision", "joint_margin_rad", "fk_position_residual_m",
                    "fk_orientation_residual_rad", "field_relevance", "valid", "rejection_reason",
                ],
            )

    def test_csv_field_relevance_ignores_fk_residuals(self):
        first = candidate(-0.19, -0.19, yaw=0.1, margin=0.25, fk_position_residual_m=0.0, fk_orientation_residual_rad=0.0)
        second = candidate(-0.19, -0.19, yaw=0.1, margin=0.25, fk_position_residual_m=999.0, fk_orientation_residual_rad=math.pi)
        field = build_field_from_result(self.grasp, result([first, second], valid=2), self.grid)
        with tempfile.TemporaryDirectory() as temporary:
            paths = save_field_bundle(field, [first, second], temporary)
            with Path(paths["candidates"]).open(newline="") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(rows[0]["field_relevance"], rows[1]["field_relevance"])

    def test_outputs_do_not_mutate_field_or_candidates(self):
        item = candidate(-0.19, -0.19)
        before = dict(item)
        field = build_field_from_result(self.grasp, result([item]), self.grid)
        snapshots = {name: getattr(field, name).copy() for name in ("relevance", "cell_state", "best_yaw")}
        with tempfile.TemporaryDirectory() as temporary:
            to_occupancy_grid_payload(field)
            field_summary(field, FieldConfig())
            save_field_bundle(field, [item], temporary)
        self.assertEqual(item, before)
        for name, snapshot in snapshots.items():
            np.testing.assert_array_equal(getattr(field, name), snapshot)

    def test_invalid_inputs_raise_value_error(self):
        field = build_field_from_result(self.grasp, result([], inverse=0, deduplicated=0, validation_limit=0, valid=0), self.grid)
        with self.assertRaises(ValueError):
            to_occupancy_grid_payload(object())
        with self.assertRaises(ValueError):
            field_summary(field, object())
        with self.assertRaises(ValueError):
            save_field_bundle(field, object(), tempfile.gettempdir())
        with self.assertRaises(ValueError):
            save_field_bundle(field, [object()], tempfile.gettempdir())


if __name__ == "__main__":
    unittest.main()
