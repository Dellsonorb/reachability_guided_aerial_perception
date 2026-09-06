"""Tests for the Python 3.8 boundary; no ROS installation is needed."""

import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SUPPORT_PATH = ROOT / "scripts" / "a5_ros_support.py"
SPEC = importlib.util.spec_from_file_location("a5_ros_support", SUPPORT_PATH)
support = importlib.util.module_from_spec(SPEC) if SUPPORT_PATH.exists() else None
if support is not None:
    SPEC.loader.exec_module(support)


class SupportTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(support, "the ROS-independent support module is missing")

    def test_finite_xyz_drops_nonfinite_and_zero_returns_without_replacing_them(self):
        rows = iter([(1, 2, 3), (0, 0, 0), (float("nan"), 2, 3),
                     (1, float("inf"), 2), (0, 0, 0.01), (-1, 2, -3)])
        points = support.finite_xyz(rows)
        np.testing.assert_array_equal(points, [[1, 2, 3], [0, 0, .01], [-1, 2, -3]])
        self.assertEqual(points.dtype, np.float64)

    def test_empty_xyz_remains_an_empty_three_column_cloud(self):
        self.assertEqual(support.finite_xyz(iter([])).shape, (0, 3))

    def test_xyz_rejects_wrong_shape(self):
        with self.assertRaises(ValueError):
            support.finite_xyz([(1, 2)])

    def test_rigid_transform_preserves_pitch_and_translation(self):
        angle = .35
        transform = support.rigid_transform(
            [.14714489037, 0, .27696863564],
            [0, 2 * math.sin(angle / 2), 0, 2 * math.cos(angle / 2)])
        transformed = transform @ np.array([1., 0., 0., 1.])
        np.testing.assert_allclose(transformed, [math.cos(angle) + .14714489037,
                                               0, -math.sin(angle) + .27696863564, 1])
        np.testing.assert_allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-15)

    def test_rigid_transform_rejects_invalid_quaternion_and_translation(self):
        for position, rotation in [([0, 0, 0], [0, 0, 0, 0]),
                                   ([0, 0, float("nan")], [0, 0, 0, 1]),
                                   ([0, 0, 0], [0, 0, float("inf"), 1])]:
            with self.subTest(position=position, rotation=rotation), self.assertRaises(ValueError):
                support.rigid_transform(position, rotation)

    def test_pose_and_quaternion_round_trip_keeps_map_offset(self):
        quaternion = support.yaw_quaternion(-2.7)
        pose = support.pose_xyzyaw(support.rigid_transform([-2, 3, 1.5], quaternion))
        np.testing.assert_allclose(pose, [-2, 3, 1.5, -2.7])

    def test_settling_requires_yaw_position_and_low_speed(self):
        goal = [0, 0, 1.5, 0]
        self.assertTrue(support.pose_settled(goal, goal, [.01, 0, 0], .1, .1, .1))
        for pose, velocity in [([0, 0, 1.5, .3], [0, 0, 0]),
                               ([.2, 0, 1.5, 0], [0, 0, 0]),
                               (goal, [.1, .1, 0]),
                               (goal, [float("nan"), 0, 0])]:
            with self.subTest(pose=pose, velocity=velocity):
                self.assertFalse(support.pose_settled(pose, goal, velocity, .1, .1, .1))

    def test_settling_wraps_yaw_at_pi(self):
        self.assertTrue(support.pose_settled([0, 0, 1, math.pi - .01],
                                            [0, 0, 1, -math.pi + .01],
                                            [0, 0, 0], .1, .03, .1))

    def test_delayed_cloud_pose_must_match_the_stable_goal_even_after_recovery(self):
        goal = [0, 0, 1.5, 0]
        self.assertTrue(hasattr(support, "capture_pose_settled"), "stamped hover capture gate is missing")
        self.assertFalse(support.capture_pose_settled(
            goal, [.3, 0, 1.5, 0], goal, [0, 0, 0], .1, .1, .1))
        self.assertFalse(support.capture_pose_settled(
            goal, [0, 0, 1.5, .3], goal, [0, 0, 0], .1, .1, .1))

    def test_cloud_acceptance_requires_current_hover_and_low_speed(self):
        goal = [0, 0, 1.5, 0]
        self.assertTrue(hasattr(support, "capture_pose_settled"), "stamped hover capture gate is missing")
        self.assertFalse(support.capture_pose_settled(
            [.3, 0, 1.5, 0], goal, goal, [0, 0, 0], .1, .1, .1))
        self.assertFalse(support.capture_pose_settled(
            goal, goal, goal, [.2, 0, 0], .1, .1, .1))

    def test_stamped_and_current_hover_pose_allow_a_capture(self):
        goal = [0, 0, 1.5, 0]
        self.assertTrue(hasattr(support, "capture_pose_settled"), "stamped hover capture gate is missing")
        self.assertTrue(support.capture_pose_settled(
            goal, [.01, 0, 1.5, .01], goal, [.01, 0, 0], .1, .1, .1))

    def test_scan_must_be_strictly_newer_than_capture_and_previous_scan(self):
        self.assertTrue(support.fresh_scan_stamp(12.1, 11, 12))
        for stamp in [12, 11, 0, float("nan"), float("inf")]:
            with self.subTest(stamp=stamp):
                self.assertFalse(support.fresh_scan_stamp(stamp, 11, 12))
        self.assertFalse(support.fresh_scan_stamp(12.1, 12.2, 12))

    def test_frame_normalization_only_removes_leading_slashes(self):
        self.assertEqual(support.normalized_frame("///uav1/livox"), "uav1/livox")
        self.assertEqual(support.normalized_frame("uav1/livox"), "uav1/livox")
        with self.assertRaises(ValueError):
            support.normalized_frame("///")

    def observation_response(self):
        return {"ok": True, "round": 1, "stop_reason": None,
                "next_viewpoint": [0, 0, 1.5, .5], "selected_candidate": None}

    def test_worker_response_preserves_exact_selected_pose_and_identity(self):
        selected = {"candidate_id": "exact-7", "source_id": 7,
                    "x": .123456, "y": -.987654, "yaw": .345678, "relevance": .72}
        response = dict(self.observation_response(), stop_reason="max_viewpoints",
                        next_viewpoint=None, selected_candidate=selected)
        self.assertIs(support.validate_worker_response(response, "observe", 1), response)
        self.assertIs(response["selected_candidate"], selected)

    def test_worker_response_requires_nonnegative_integer_a1_source_id(self):
        for source_id in ["7", -1, True]:
            selected = {"candidate_id": "exact-7", "source_id": source_id,
                        "x": .123456, "y": -.987654, "yaw": .345678, "relevance": .72}
            response = dict(self.observation_response(), selected_candidate=selected)
            with self.subTest(source_id=source_id), self.assertRaises(support.WorkerError):
                support.validate_worker_response(response, "observe", 1)

    def test_worker_response_rejects_nonnumeric_goal_and_candidate_coordinates(self):
        for bad in ["0.1", True]:
            selected = {"candidate_id": "exact-7", "source_id": 7,
                        "x": bad, "y": -.987654, "yaw": .345678, "relevance": .72}
            for response in [dict(self.observation_response(), next_viewpoint=[bad, 0, 1.5, 0]),
                             dict(self.observation_response(), selected_candidate=selected)]:
                with self.subTest(response=response), self.assertRaises(support.WorkerError):
                    support.validate_worker_response(response, "observe", 1)

    def test_worker_response_rejects_malformed_or_mismatched_results(self):
        good = self.observation_response()
        for response in [[], {"ok": False, "error": "test failure"},
                         dict(good, ok=1), dict(good, round=2),
                         dict(good, next_viewpoint=[0, 0, float("nan"), 0]),
                         dict(good, next_viewpoint=None),
                         dict(good, stop_reason="done"),
                         dict(good, selected_candidate={"candidate_id": "missing pose"})]:
            with self.subTest(response=response), self.assertRaises(support.WorkerError):
                support.validate_worker_response(response, "observe", 1)

    def test_stop_without_confirmed_candidate_remains_a_valid_negative_outcome(self):
        response = dict(self.observation_response(), stop_reason="no_task_gain", next_viewpoint=None)
        support.validate_worker_response(response, "observe", 1)

    def test_init_response_requires_cache_path(self):
        response = {"ok": True, "initial_file": "/tmp/initial.json"}
        self.assertIs(support.validate_worker_response(response, "init"), response)
        with self.assertRaises(support.WorkerError):
            support.validate_worker_response({"ok": True}, "init")

    def write_worker(self, directory, body):
        path = Path(directory) / "worker.py"
        path.write_text("import argparse, json, os, time\n"
                        "from pathlib import Path\n"
                        "p = argparse.ArgumentParser()\n"
                        "p.add_argument('--request')\n"
                        "p.add_argument('--response')\n"
                        "args = p.parse_args()\n" + body, encoding="utf-8")
        return path

    def call_worker(self, directory, body, timeout=2):
        worker = self.write_worker(directory, body)
        return support.run_worker_request(sys.executable, worker, ROOT / "src",
                                          {"op": "init", "grasp": {"exact": .123456}},
                                          directory, "init", timeout)

    def test_worker_uses_request_file_and_captures_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            response = self.call_worker(directory,
                "request = json.loads(Path(args.request).read_text())\n"
                "assert request['grasp']['exact'] == .123456\n"
                "assert os.environ['PYTHONPATH'].split(os.pathsep)[0].endswith('/src')\n"
                "print('worker stdout retained')\n"
                "Path(args.response).write_text(json.dumps({'ok': True, 'initial_file': '/tmp/exact.json'}))\n")
            self.assertEqual(response["initial_file"], "/tmp/exact.json")
            logs = list(Path(directory).glob("init-*/worker.log"))
            self.assertEqual(len(logs), 1)
            self.assertIn("worker stdout retained", logs[0].read_text())
            self.assertTrue(logs[0].with_name("request.json").is_file())
            self.assertTrue(logs[0].with_name("response.json").is_file())

    def test_worker_rejects_nonzero_exit_missing_or_invalid_response(self):
        for body in ["raise SystemExit(3)\n", "print('no response')\n",
                     "Path(args.response).write_text('not json')\n",
                     "Path(args.response).write_text('{\"ok\": false, \"error\": \"bad input\"}')\n"]:
            with self.subTest(body=body), tempfile.TemporaryDirectory() as directory:
                with self.assertRaises(support.WorkerError):
                    self.call_worker(directory, body)

    def test_worker_timeout_is_bounded_and_retains_log(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(support.WorkerError, "timed out"):
                self.call_worker(directory, "print('starting', flush=True)\ntime.sleep(5)\n", .1)
            self.assertIn("starting", next(Path(directory).glob("init-*/worker.log")).read_text())

    def test_core_interpreter_does_not_inherit_ros_python_path(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {"PYTHONPATH": "/ros/python3.8"}):
            response = self.call_worker(directory,
                "assert os.pathsep not in os.environ['PYTHONPATH']\n"
                "assert os.environ['PYTHONPATH'].endswith('/src')\n"
                "Path(args.response).write_text(json.dumps({'ok': True, 'initial_file': '/tmp/exact.json'}))\n")
            self.assertTrue(response["ok"])


class AdapterImportTests(unittest.TestCase):
    def test_importing_adapter_does_not_import_ros_or_the_research_core(self):
        result = subprocess.run(
            [sys.executable, "-c", "import importlib.util, sys; "
             "s = importlib.util.spec_from_file_location('adapter', sys.argv[1]); "
             "m = importlib.util.module_from_spec(s); s.loader.exec_module(m); "
             "assert 'rospy' not in sys.modules; assert 'sim_active_perception' not in sys.modules",
             str(ROOT / "scripts" / "run_a5_sim.py")], capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_help_works_without_ros_and_lists_required_boundary_options(self):
        result = subprocess.run([sys.executable, str(ROOT / "scripts" / "run_a5_sim.py"), "--help"],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        for flag in ["--sim-root", "--output-dir", "--core-python", "--rm4d-root",
                     "--rm4d-config", "--rm4d-map", "--max-viewpoints"]:
            self.assertIn(flag, result.stdout)


if __name__ == "__main__":
    unittest.main()
