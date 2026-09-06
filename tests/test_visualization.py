import copy
import os
import subprocess
import sys
import tempfile
import unittest
import warnings
from pathlib import Path
from unittest.mock import patch

import matplotlib
import numpy as np

matplotlib.use("Agg", force=True)

from reachability_guided_aerial_perception import (
    FieldConfig,
    GraspTCP,
    GridSpec,
    build_field_from_result,
    candidate_relevance,
    render_field,
)


def candidate(x=0.0, y=0.0, yaw=0.0, margin=0.25, *, valid=True):
    return {
        "candidate_id": f"candidate-{yaw}",
        "bunker_x": x,
        "bunker_y": y,
        "bunker_yaw": yaw,
        "rm4d_reachable": True,
        "ik_valid": True,
        "collision_free": True,
        "footprint_collision": False,
        "valid": valid,
        "joint_margin_rad": margin,
        "fk_position_residual_m": 0.01,
        "fk_orientation_residual_rad": 0.02,
        "rejection_reason": None if valid else "invalid",
    }


def result(candidates, *, inverse=None, valid=None):
    if inverse is None:
        inverse = len(candidates)
    if valid is None:
        valid = sum(candidate_relevance(item, FieldConfig()) > 0 for item in candidates)
    rejected = {}
    for item in candidates:
        if item["valid"] is not True:
            rejected[item["rejection_reason"]] = rejected.get(item["rejection_reason"], 0) + 1
    return {
        "schema_version": 1,
        "frame_id": "map",
        "grasp_id": "g",
        "summary": {
            "inverse_reachable": inverse,
            "deduplicated": len(candidates),
            "validation_limit": len(candidates),
            "evaluated": len(candidates),
            "valid": valid,
            "rejected_by_reason": rejected,
        },
        "evaluated_candidates": candidates,
    }


class VisualizationTests(unittest.TestCase):
    def setUp(self):
        self.grasp = GraspTCP("g", "map", (0.0, 0.0, 0.4), (0, 0, 0, 1))
        self.grid = GridSpec.centered((0.0, 0.0), 0.4, 0.4, 0.2)

    def test_partial_field_writes_headless_png(self):
        candidates = [candidate(-0.19, -0.19, margin=0.25), candidate(0.01, 0.01, margin=0.5)]
        field = build_field_from_result(self.grasp, result(candidates), self.grid)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "nested" / "field.png"
            returned = render_field(field, self.grasp, candidates, output)
            self.assertEqual(returned, output)
            self.assertGreater(output.stat().st_size, 10_000)

    def test_no_inverse_empty_case_writes_without_warnings(self):
        field = build_field_from_result(
            self.grasp,
            result([], inverse=0, valid=0),
            self.grid,
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "none.png"
            with warnings.catch_warnings():
                warnings.simplefilter("error")
                render_field(field, self.grasp, [], output)
            self.assertTrue(output.exists())

    def test_rasters_use_unsmoothed_interpolation_and_fixed_relevance_range(self):
        candidates = [candidate(0.01, 0.01)]
        field = build_field_from_result(self.grasp, result(candidates), self.grid)
        calls = []
        from matplotlib.axes import Axes

        original = Axes.imshow

        def capture(axes, *args, **kwargs):
            calls.append((args, kwargs.copy()))
            return original(axes, *args, **kwargs)

        with tempfile.TemporaryDirectory() as directory:
            with patch.object(Axes, "imshow", capture):
                render_field(field, self.grasp, candidates, Path(directory) / "field.png")
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(call[1]["interpolation"] == "none" for call in calls))
        self.assertEqual((calls[0][1]["vmin"], calls[0][1]["vmax"]), (0, 1))
        self.assertTrue(np.ma.getmaskarray(calls[0][0][0]).any())

    def test_rejects_invalid_inputs_and_config_mismatch(self):
        candidates = [candidate(0.01, 0.01)]
        field = build_field_from_result(self.grasp, result(candidates), self.grid)
        bad_grasp = GraspTCP("other", "map", (0.0, 0.0, 0.4), (0, 0, 0, 1))
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "field.png"
            for args in (
                (object(), self.grasp, candidates, output),
                (field, bad_grasp, candidates, output),
                (field, self.grasp, [object()], output),
                (field, self.grasp, candidates, directory),
            ):
                with self.subTest(args=args[:3]):
                    with self.assertRaises(ValueError):
                        render_field(*args)
            with self.assertRaises(ValueError):
                render_field(field, self.grasp, candidates, output, FieldConfig(high_relevance_threshold=0.7))

    def test_rejects_candidates_that_do_not_reproduce_the_field(self):
        original = candidate(0.01, 0.01, yaw=0.2, margin=0.25)
        field = build_field_from_result(self.grasp, result([original]), self.grid)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "field.png"
            with self.assertRaises(ValueError):
                render_field(field, self.grasp, [], output)
            for name, value in (
                ("rm4d_reachable", False),
                ("bunker_yaw", 0.3),
                ("fk_position_residual_m", 0.02),
            ):
                altered = copy.deepcopy(original)
                altered[name] = value
                with self.subTest(name=name):
                    with self.assertRaises(ValueError):
                        render_field(field, self.grasp, [altered], output)

    def test_importing_visualization_does_not_change_selected_backend(self):
        source = (
            "import matplotlib; matplotlib.use('svg', force=True); "
            "before = matplotlib.get_backend(); "
            "import reachability_guided_aerial_perception.visualization; "
            "assert matplotlib.get_backend() == before, "
            "(before, matplotlib.get_backend())"
        )
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(Path(__file__).parents[1] / "src")
        completed = subprocess.run(
            [sys.executable, "-c", source],
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_inputs_are_not_mutated(self):
        candidates = [candidate(0.01, 0.01)]
        original_candidates = copy.deepcopy(candidates)
        field = build_field_from_result(self.grasp, result(candidates), self.grid)
        original_relevance = field.relevance.copy()
        with tempfile.TemporaryDirectory() as directory:
            render_field(field, self.grasp, candidates, Path(directory) / "field.png")
        self.assertEqual(candidates, original_candidates)
        np.testing.assert_array_equal(field.relevance, original_relevance)
