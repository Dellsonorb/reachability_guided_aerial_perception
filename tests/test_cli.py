import contextlib
import importlib.util
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from reachability_guided_aerial_perception import FieldConfig, FieldStatus, GraspTCP
from reachability_guided_aerial_perception.cli import (
    build_parser,
    load_grasp,
    main,
    open_frozen_rm4d_api,
    run_scenario,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def valid_candidate(x=0.0, y=0.0, margin=0.5):
    return {
        "candidate_id": "candidate-0",
        "bunker_x": x,
        "bunker_y": y,
        "bunker_yaw": 0.0,
        "rm4d_reachable": True,
        "ik_valid": True,
        "collision_free": True,
        "footprint_collision": False,
        "joint_margin_rad": margin,
        "fk_position_residual_m": 1e-6,
        "fk_orientation_residual_rad": 2e-6,
        "valid": True,
        "rejection_reason": None,
    }


def planner_result(grasp_id, candidates):
    return {
        "schema_version": 1,
        "status": "ok" if candidates else "no_inverse_reachable",
        "frame_id": "map",
        "grasp_id": grasp_id,
        "summary": {
            "inverse_reachable": len(candidates),
            "deduplicated": len(candidates),
            "validation_limit": 256,
            "evaluated": len(candidates),
            "valid": len(candidates),
            "rejected_by_reason": {},
        },
        "candidates": [{"top_k_payload_must_not_be_used": True}],
        "evaluated_candidates": candidates,
    }


class RecordingAPI:
    def __init__(self):
        self.calls = []

    def plan(self, request, top_k=None):
        self.calls.append((request, top_k))
        grasp_id = request["grasp_id"]
        if grasp_id == "no_inverse":
            return planner_result(grasp_id, [])
        x, y, _ = request["position_xyz"]
        return planner_result(grasp_id, [valid_candidate(x, y)])


class CliTests(unittest.TestCase):
    def write_json(self, directory, payload, name="grasp.json"):
        path = Path(directory) / name
        path.write_text(json.dumps(payload))
        return path

    def valid_payload(self):
        return {
            "grasp_id": "g",
            "frame_id": "map",
            "position_xyz": [0.1, -0.2, 0.4],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        }

    def test_load_grasp_accepts_only_the_exact_map_tcp_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            grasp = load_grasp(self.write_json(directory, self.valid_payload()))
        self.assertEqual(
            grasp,
            GraspTCP("g", "map", (0.1, -0.2, 0.4), (0.0, 0.0, 0.0, 1.0)),
        )

    def test_load_grasp_rejects_missing_extra_and_historical_selection_inputs(self):
        cases = {
            "missing": {key: value for key, value in self.valid_payload().items() if key != "grasp_id"},
            "extra": {**self.valid_payload(), "note": "not allowed"},
            "current pose": {**self.valid_payload(), "current_bunker_pose": {"x": 0, "y": 0, "yaw": 0}},
            "obstacles": {**self.valid_payload(), "obstacles": []},
        }
        with tempfile.TemporaryDirectory() as directory:
            for name, payload in cases.items():
                with self.subTest(name=name):
                    with self.assertRaises(ValueError):
                        load_grasp(self.write_json(directory, payload, name.replace(" ", "_") + ".json"))

    def test_load_grasp_delegates_frame_and_numeric_validation_to_grasp_tcp(self):
        cases = {
            "world": {**self.valid_payload(), "frame_id": "world"},
            "position shape": {**self.valid_payload(), "position_xyz": [0, 1]},
            "nonunit quaternion": {**self.valid_payload(), "quaternion_xyzw": [0, 0, 0, 2]},
            "nonfinite": {**self.valid_payload(), "position_xyz": [0, float("nan"), 0]},
        }
        with tempfile.TemporaryDirectory() as directory:
            for name, payload in cases.items():
                with self.subTest(name=name):
                    with self.assertRaises(ValueError):
                        load_grasp(self.write_json(directory, payload, name.replace(" ", "_") + ".json"))

    def test_importing_cli_does_not_import_frozen_baseline_or_pybullet(self):
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(REPOSITORY_ROOT / "src")
        completed = subprocess.run(
            [
                sys.executable,
                "-c",
                "import sys; import reachability_guided_aerial_perception.cli; "
                "assert 'rm4d' not in sys.modules; assert 'pybullet' not in sys.modules",
            ],
            cwd=REPOSITORY_ROOT,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_frozen_loader_imports_from_explicit_root_and_closes_context(self):
        events = []

        class FakeAPI:
            def __enter__(self):
                events.append("enter")
                return self

            def __exit__(self, exc_type, exc_value, traceback):
                events.append("exit")

        class FakeBasePlacementAPI:
            @classmethod
            def from_files(cls, config_path, map_path):
                events.append(("from_files", config_path, map_path, list(sys.path)))
                return FakeAPI()

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "rm4d"
            root.mkdir()
            config = Path(directory) / "config.json"
            map_path = Path(directory) / "rmap.npy"
            config.touch()
            map_path.touch()
            fake_module = type(
                "FakeModule",
                (),
                {
                    "BasePlacementAPI": FakeBasePlacementAPI,
                    "__file__": str(root / "rm4d" / "__init__.py"),
                },
            )
            root_string = str(root.resolve())
            prior_path = ["before-root", root_string, "after-root"]

            def import_rm4d(name):
                events.append(("import", name, list(sys.path)))
                return fake_module

            with patch.object(sys, "path", prior_path.copy()):
                with patch("importlib.import_module", side_effect=import_rm4d) as importer:
                    with open_frozen_rm4d_api(root, config, map_path) as api:
                        self.assertIsInstance(api, FakeAPI)
                self.assertEqual(sys.path, prior_path)
            importer.assert_called_once_with("rm4d")
            self.assertEqual(events[0][0:2], ("import", "rm4d"))
            self.assertEqual(events[0][2][0], root_string)
            self.assertEqual(
                events[1],
                ("from_files", str(config.resolve()), str(map_path.resolve()), prior_path),
            )
            self.assertEqual(events[2:], ["enter", "exit"])

    def test_frozen_loader_rejects_cached_module_outside_requested_root(self):
        calls = []

        class WrongBasePlacementAPI:
            @classmethod
            def from_files(cls, config_path, map_path):
                calls.append((config_path, map_path))

        cached_module = type(
            "CachedModule",
            (),
            {
                "BasePlacementAPI": WrongBasePlacementAPI,
                "__file__": "/some/other/repository/rm4d/__init__.py",
            },
        )
        real_import = __import__("importlib").import_module
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "requested"
            root.mkdir()
            config = Path(directory) / "config.json"
            map_path = Path(directory) / "rmap.npy"
            config.touch()
            map_path.touch()
            with patch.dict(sys.modules, {"rm4d": cached_module}):
                with patch("importlib.import_module", side_effect=real_import) as importer:
                    with self.assertRaisesRegex(ValueError, "requested rm4d_root"):
                        with open_frozen_rm4d_api(root, config, map_path):
                            pass
            importer.assert_not_called()
        self.assertEqual(calls, [])

    def test_frozen_loader_rejects_newly_imported_module_outside_requested_root(self):
        calls = []

        class WrongBasePlacementAPI:
            @classmethod
            def from_files(cls, config_path, map_path):
                calls.append((config_path, map_path))

        imported_module = type(
            "ImportedModule",
            (),
            {
                "BasePlacementAPI": WrongBasePlacementAPI,
                "__file__": "/some/other/repository/rm4d/__init__.py",
            },
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "requested"
            root.mkdir()
            config = Path(directory) / "config.json"
            map_path = Path(directory) / "rmap.npy"
            config.touch()
            map_path.touch()
            prior_path = list(sys.path)
            with patch("importlib.import_module", return_value=imported_module):
                with self.assertRaisesRegex(ValueError, "requested rm4d_root"):
                    with open_frozen_rm4d_api(root, config, map_path):
                        pass
            self.assertEqual(sys.path, prior_path)
        self.assertEqual(calls, [])

    def test_frozen_loader_rejects_module_without_a_path_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "requested"
            root.mkdir()
            config = Path(directory) / "config.json"
            map_path = Path(directory) / "rmap.npy"
            config.touch()
            map_path.touch()
            for module_file in (None, object()):
                imported_module = type(
                    "ImportedModule",
                    (),
                    {"BasePlacementAPI": object(), "__file__": module_file},
                )
                with self.subTest(module_file=module_file):
                    with patch("importlib.import_module", return_value=imported_module):
                        with self.assertRaisesRegex(ValueError, "requested rm4d_root"):
                            with open_frozen_rm4d_api(root, config, map_path):
                                pass

    def test_frozen_loader_restores_sys_path_when_import_raises(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "requested"
            root.mkdir()
            config = Path(directory) / "config.json"
            map_path = Path(directory) / "rmap.npy"
            config.touch()
            map_path.touch()
            root_string = str(root.resolve())
            prior_path = ["first", root_string, "last"]
            observed = []

            def fail_import(name):
                observed.append(list(sys.path))
                raise ImportError("expected import failure")

            with patch.object(sys, "path", prior_path.copy()):
                with patch("importlib.import_module", side_effect=fail_import):
                    with self.assertRaisesRegex(ImportError, "expected import failure"):
                        with open_frozen_rm4d_api(root, config, map_path):
                            pass
                self.assertEqual(sys.path, prior_path)
            self.assertEqual(observed[0][0], root_string)

    def test_frozen_loader_rejects_missing_root_config_or_map_before_import(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "rm4d"
            root.mkdir()
            config = Path(directory) / "config.json"
            map_path = Path(directory) / "rmap.npy"
            config.touch()
            map_path.touch()
            cases = (
                (root / "missing", config, map_path),
                (root, config.with_name("missing.json"), map_path),
                (root, config, map_path.with_name("missing.npy")),
            )
            with patch("importlib.import_module") as importer:
                for values in cases:
                    with self.subTest(values=values):
                        with self.assertRaises(ValueError):
                            with open_frozen_rm4d_api(*values):
                                pass
                importer.assert_not_called()

    def test_parser_requires_explicit_external_and_scenario_paths(self):
        parser = build_parser()
        args = parser.parse_args(
            [
                "--rm4d-root", "/baseline",
                "--rm4d-config", "/baseline/config.json",
                "--rm4d-map", "/baseline/map.npy",
                "--grasp", "grasp.json",
                "--output-dir", "outputs/scenario",
            ]
        )
        self.assertEqual(args.grid_width_m, 3.0)
        self.assertEqual(args.grid_height_m, 3.0)
        self.assertEqual(args.grid_resolution_m, 0.1)

    def test_run_scenario_calls_plan_once_and_writes_exact_outputs(self):
        api = RecordingAPI()
        grasp = GraspTCP("nominal", "map", (0.1, -0.2, 0.4), (0, 0, 0, 1))
        with tempfile.TemporaryDirectory() as directory:
            field, summary = run_scenario(api, grasp, Path(directory), config=FieldConfig())
            self.assertEqual(
                {path.name for path in Path(directory).iterdir()},
                {"field.npz", "summary.json", "candidate_diagnostics.csv", "field.png"},
            )
            disk_summary = json.loads((Path(directory) / "summary.json").read_text())
        self.assertEqual(len(api.calls), 1)
        self.assertEqual(api.calls[0][1], 1)
        self.assertEqual(set(api.calls[0][0]), {"grasp_id", "frame_id", "position_xyz", "quaternion_xyzw"})
        self.assertIs(field.status, FieldStatus.PARTIALLY_ASSESSED)
        self.assertEqual(summary, disk_summary)
        self.assertEqual(summary["cells"]["high"], 1)

    def test_single_scenario_main_treats_no_inverse_as_normal_success(self):
        api = RecordingAPI()
        opened = 0
        closed = 0

        @contextlib.contextmanager
        def opener(*args):
            nonlocal opened, closed
            opened += 1
            try:
                yield api
            finally:
                closed += 1

        payload = {**self.valid_payload(), "grasp_id": "no_inverse"}
        with tempfile.TemporaryDirectory() as directory:
            grasp_path = self.write_json(directory, payload)
            output = Path(directory) / "output"
            stdout = io.StringIO()
            with patch(
                "reachability_guided_aerial_perception.cli.open_frozen_rm4d_api",
                opener,
            ), contextlib.redirect_stdout(stdout):
                return_code = main(
                    [
                        "--rm4d-root", "unused-root",
                        "--rm4d-config", "unused-config",
                        "--rm4d-map", "unused-map",
                        "--grasp", str(grasp_path),
                        "--output-dir", str(output),
                    ]
                )
            printed = json.loads(stdout.getvalue())
        self.assertEqual(return_code, 0)
        self.assertEqual((opened, closed, len(api.calls)), (1, 1, 1))
        self.assertEqual(printed["status"], "NO_INVERSE_REACHABLE")

    def test_checked_in_scenarios_have_the_approved_exact_tcp_values(self):
        expected = {
            "nominal": (
                [0.8404056122469007, -0.28035103486649454, 0.3891261785834021],
                [-0.1919076750399526, 0.6540703700627786, -0.20600231548400175, 0.7020872034739912],
            ),
            "boundary": (
                [-0.8997505649276936, -0.12644810524935426, 0.13710300800865596],
                [-0.06614890261953035, 0.8187602792498319, 0.41049329171564103, 0.3959181547509873],
            ),
            "no_inverse": ([0, 0, 2], [0, 0, 0, 1]),
        }
        for name, (position, quaternion) in expected.items():
            with self.subTest(name=name):
                payload = json.loads((REPOSITORY_ROOT / "examples" / "scenarios" / f"{name}.json").read_text())
                self.assertEqual(
                    payload,
                    {
                        "grasp_id": name,
                        "frame_id": "map",
                        "position_xyz": position,
                        "quaternion_xyzw": quaternion,
                    },
                )

    def test_offline_runner_reuses_one_api_for_exactly_three_scenarios(self):
        script_path = REPOSITORY_ROOT / "scripts" / "run_offline_validation.py"
        spec = importlib.util.spec_from_file_location("a1_offline_validation", script_path)
        module = importlib.util.module_from_spec(spec)
        self.assertIsNotNone(spec.loader)
        spec.loader.exec_module(module)

        api = RecordingAPI()
        opened = 0
        closed = 0

        @contextlib.contextmanager
        def opener(*args):
            nonlocal opened, closed
            opened += 1
            try:
                yield api
            finally:
                closed += 1

        with tempfile.TemporaryDirectory() as directory:
            stdout = io.StringIO()
            with patch.object(module, "open_frozen_rm4d_api", opener), contextlib.redirect_stdout(stdout):
                return_code = module.main(
                    [
                        "--rm4d-root", "unused-root",
                        "--rm4d-config", "unused-config",
                        "--rm4d-map", "unused-map",
                        "--output-root", directory,
                    ]
                )
            output_names = {path.name for path in Path(directory).iterdir()}
        self.assertEqual(return_code, 0)
        self.assertEqual((opened, closed), (1, 1))
        self.assertEqual([call[0]["grasp_id"] for call in api.calls], ["nominal", "boundary", "no_inverse"])
        self.assertTrue(all(call[1] == 1 for call in api.calls))
        self.assertEqual(output_names, {"nominal", "boundary", "no_inverse"})
        self.assertIn("NO_INVERSE_REACHABLE", stdout.getvalue())


if __name__ == "__main__":
    unittest.main()
