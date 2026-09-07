"""Tests for the Python 3.8 boundary; no ROS installation is needed."""

import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace
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

    def test_cloud_window_reexpresses_each_chunk_in_the_last_sensor_pose(self):
        self.assertTrue(hasattr(support, "merge_cloud_chunks"), "cloud-window merge is missing")
        chunks = []
        expected_map = []
        for index, points in enumerate(([[1., 2., -.4], [-.1, .2, .3]],
                                        [[-.2, .7, .8]], [])):
            pitch = -.3 + .2 * index
            transform = support.rigid_transform(
                [.2 * index, -.1 * index, 1.5 + .05 * index],
                [0., math.sin(pitch / 2), 0., math.cos(pitch / 2)])
            points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
            chunks.append(dict(points_xyz=points, T_map_sensor=transform,
                               stamp_s=10.1 + index, frame_id="/uav1/livox"))
            expected_map.append(points @ transform[:3, :3].T + transform[:3, 3])

        merged = support.merge_cloud_chunks(chunks)

        np.testing.assert_allclose(merged["T_map_sensor"], chunks[-1]["T_map_sensor"])
        anchor = merged["T_map_sensor"]
        reconstructed_map = merged["points_xyz"] @ anchor[:3, :3].T + anchor[:3, 3]
        np.testing.assert_allclose(reconstructed_map, np.concatenate(expected_map), atol=1e-12)
        np.testing.assert_array_equal(merged["chunk_point_counts"], [2, 1, 0])
        np.testing.assert_allclose(merged["chunk_stamps_s"], [10.1, 11.1, 12.1])
        np.testing.assert_allclose(merged["chunk_T_map_sensor"],
                                   np.stack([chunk["T_map_sensor"] for chunk in chunks]))
        self.assertEqual(float(merged["stamp_s"]), 12.1)
        self.assertEqual(str(merged["frame_id"]), "uav1/livox")
        np.testing.assert_array_equal(chunks[0]["points_xyz"], [[1., 2., -.4], [-.1, .2, .3]])

    def test_cloud_window_requires_monotonic_positive_stamps_and_one_normalized_frame(self):
        self.assertTrue(hasattr(support, "merge_cloud_chunks"), "cloud-window merge is missing")
        first = dict(points_xyz=np.array([[1., 0., 0.]]), T_map_sensor=np.eye(4),
                     stamp_s=10., frame_id="///uav1/livox")
        second = dict(first, stamp_s=10.1, frame_id="uav1/livox")
        self.assertEqual(str(support.merge_cloud_chunks([first, second])["frame_id"]), "uav1/livox")
        for stamp in (0., -1., 10., 9.9, float("nan"), float("inf")):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                support.merge_cloud_chunks([first, dict(second, stamp_s=stamp)])
        with self.assertRaises(ValueError):
            support.merge_cloud_chunks([first, dict(second, frame_id="uav1/other_sensor")])
        with self.assertRaises(ValueError):
            support.merge_cloud_chunks([])

    def test_cloud_window_rejects_invalid_chunk_geometry(self):
        self.assertTrue(hasattr(support, "merge_cloud_chunks"), "cloud-window merge is missing")
        chunk = dict(points_xyz=np.array([[1., 0., 0.]]), T_map_sensor=np.eye(4),
                     stamp_s=10., frame_id="uav1/livox")
        for points in (np.array([[float("nan"), 0, 0]]), np.ones((2, 2))):
            with self.subTest(points=points), self.assertRaises(ValueError):
                support.merge_cloud_chunks([dict(chunk, points_xyz=points)])
        for matrix in (np.eye(3), np.diag([2., 1., 1., 1.]), np.full((4, 4), np.nan)):
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                support.merge_cloud_chunks([dict(chunk, T_map_sensor=matrix)])

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


class CaptureWindowTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location("a5_capture_adapter", ROOT / "scripts/run_a5_sim.py")
        self.adapter = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.adapter)
        self.clock = SimpleNamespace(sim=10., wall=0.)
        clock = self.clock

        class Stamp:
            def __init__(self, seconds=0.):
                self.seconds = seconds

            def to_sec(self):
                return self.seconds

            @staticmethod
            def now():
                return Stamp(clock.sim)

        self.Stamp = Stamp
        # Replace only ROS transport/time boundaries. Capture, hover gates,
        # coordinate conversion, merging and NPZ persistence run unchanged.
        ros = SimpleNamespace(Time=Stamp, Duration=lambda seconds: seconds, is_shutdown=lambda: False)
        point_cloud = SimpleNamespace(read_points=lambda cloud, **_kwargs: iter(cloud.points))
        modules = {"rospy": ros, "tf2_ros": SimpleNamespace(TransformException=LookupError),
                   "sensor_msgs": SimpleNamespace(point_cloud2=point_cloud),
                   "sensor_msgs.msg": SimpleNamespace(PointCloud2=object),
                   "a5_ros_support": support}
        self.options = SimpleNamespace(cloud_timeout=1., cloud_window_s=.3, tf_timeout=.2,
                                       settle_position_tolerance=.1, settle_yaw_tolerance=.1,
                                       settle_speed=.1, settle_duration=.5, tf_max_age=.5,
                                       uav_base_frame="uav1/base_link",
                                       flight_bounds=[-4., 4., -3., 3., .5, 3.])
        with patch.dict(sys.modules, modules):
            cls = self.adapter.build_adapter_class(
                SimpleNamespace(DemoError=RuntimeError, AirGroundPickDemo=object), self.options)
        self.node = object.__new__(cls)
        self.node._lock = threading.Lock()
        self.node._map_frame = "map"
        self.node._a5_cloud = None
        self.node._a5_previous_stamp = 9.
        self.node._a5_observations = []
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.node._a5_output = Path(self.temporary.name)
        self.goal = [0., 0., 1.5, 0.]
        self.node._a5_wait_settled = lambda _goal: self.goal
        self.node._a5_settled_now = lambda _goal, **_kwargs: (True, self.goal)
        self.node._a5_measured_pose = lambda stamp=None, **_kwargs: (
            self.goal if stamp is None else [.02 * (stamp.to_sec() - 10.), 0., 1.5, 0.])
        self.node._air_snapshot = lambda: (SimpleNamespace(velocity=[0., 0., 0.]), None, 0., None)
        self.statuses = []
        self.node._publish_status = lambda status, **details: self.statuses.append((status, details))
        self.lookup_stamps = []
        self.lookup_timeouts = []

        def lookup(_target, _source, stamp, _timeout):
            seconds = stamp.to_sec()
            self.lookup_stamps.append(seconds)
            self.lookup_timeouts.append(_timeout)
            pitch = .02 * (seconds - 10.)
            return SimpleNamespace(transform=SimpleNamespace(
                translation=SimpleNamespace(x=.01 * (seconds - 10.), y=0., z=1.5),
                rotation=SimpleNamespace(x=0., y=math.sin(pitch / 2), z=0., w=math.cos(pitch / 2))))

        self.node._tf_buffer = SimpleNamespace(lookup_transform=lookup)

    def capture(self, stamps_and_frames):
        messages = iter(stamps_and_frames)

        def advance():
            self.clock.wall += .1
            try:
                stamp, frame = next(messages)
            except StopIteration:
                self.clock.sim += .1
                return
            self.clock.sim = max(self.clock.sim, stamp) + .01
            self.node._a5_cloud = SimpleNamespace(
                header=SimpleNamespace(stamp=self.Stamp(stamp), frame_id=frame),
                points=[(1., 0., -.2), (0., 0., 0.)])

        self.node._wait_step = advance
        with patch.object(self.adapter.time, "monotonic", side_effect=lambda: self.clock.wall):
            return self.node._a5_capture(self.goal)

    def test_capture_collects_full_window_at_own_stamps_as_one_observation(self):
        pose = self.capture([(10.1, "/uav1/livox"), (10.1, "uav1/livox"),
                             (10.05, "uav1/livox"), (10.2, "uav1/livox"),
                             (10.3, "uav1/livox"), (10.5, "uav1/livox")])
        self.assertEqual(self.lookup_stamps, [10.1, 10.2, 10.3, 10.5])
        self.assertEqual(self.lookup_timeouts, [0., 0., 0., 0.])
        self.assertEqual(len(self.node._a5_observations), 1)
        self.assertEqual(self.node._a5_previous_stamp, 10.5)
        np.testing.assert_allclose(pose, [.01, 0., 1.5, 0.])
        with np.load(self.node._a5_observations[0], allow_pickle=False) as data:
            self.assertEqual(data["points_xyz"].shape, (4, 3))
            np.testing.assert_allclose(data["chunk_stamps_s"], self.lookup_stamps)
            np.testing.assert_array_equal(data["chunk_point_counts"], [1, 1, 1, 1])
            self.assertGreaterEqual(data["chunk_stamps_s"][-1] - data["chunk_stamps_s"][0], .3)
            self.assertEqual(float(data["stamp_s"]), 10.5)

    def test_capture_rejects_mixed_frames_without_saving_partial_window(self):
        with self.assertRaisesRegex(RuntimeError, "frame"):
            self.capture([(10.1, "uav1/livox"), (10.2, "uav1/other_sensor")])
        self.assertEqual(self.node._a5_observations, [])

    def test_capture_retains_pending_chunk_until_its_stamped_tf_arrives(self):
        lookup = self.node._tf_buffer.lookup_transform
        attempts = []

        def delayed_lookup(target, source, stamp, timeout):
            attempts.append(stamp.to_sec())
            if len(attempts) == 1:
                raise LookupError("stamped TF has not arrived yet")
            return lookup(target, source, stamp, timeout)

        self.node._tf_buffer.lookup_transform = delayed_lookup
        self.capture([(10.1, "uav1/livox"), (10.2, "uav1/livox"),
                      (10.3, "uav1/livox"), (10.5, "uav1/livox")])
        self.assertEqual(attempts[:2], [10.1, 10.1])
        self.assertEqual(len(self.node._a5_observations), 1)

    def test_capture_checks_header_pose_for_every_chunk(self):
        self.node._a5_measured_pose = lambda stamp=None, **_kwargs: (
            self.goal if stamp is None or stamp.to_sec() < 10.2 else [.3, 0., 1.5, 0.])
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.capture([(10.1, "uav1/livox"), (10.2, "uav1/livox")])
        self.assertEqual(self.node._a5_observations, [])

    def test_capture_checks_current_hover_throughout_the_window(self):
        self.node._a5_settled_now = lambda _goal, **_kwargs: (self.clock.sim < 10.2, self.goal)
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.capture([(10.1, "uav1/livox"), (10.2, "uav1/livox")])
        self.assertEqual(self.node._a5_observations, [])

    def test_capture_discards_interrupted_window_and_recovers_with_only_new_chunks(self):
        self.options.cloud_timeout = 2.
        self.options.cloud_window_s = .25
        self.node._a5_settled_now = lambda _goal, **_kwargs: (
            not 10.24 <= self.clock.sim < 10.3,
            [.3, 0., 1.5, 0.] if 10.24 <= self.clock.sim < 10.3 else self.goal)
        self.capture([(10. + index * .125, "uav1/livox") for index in range(1, 17)])
        self.assertEqual(len(self.node._a5_observations), 1)
        with np.load(self.node._a5_observations[0], allow_pickle=False) as data:
            stamps = data["chunk_stamps_s"]
            self.assertGreaterEqual(stamps[0], 11.125)
            self.assertGreaterEqual(stamps[-1] - stamps[0], self.options.cloud_window_s)
            self.assertNotIn(10.125, stamps)
        retries = [details for status, details in self.statuses if status == "A5_CAPTURE_RETRY"]
        self.assertEqual(len(retries), 1)
        self.assertEqual(retries[0]["reason"], "current_hover_unsettled")
        self.assertEqual(retries[0]["discarded_chunks"], 1)
        self.assertEqual(retries[0]["uav_pose_map"], [.3, 0., 1.5, 0.])
        self.assertEqual(retries[0]["goal_map"], self.goal)
        self.assertEqual(retries[0]["velocity_xyz"], [0., 0., 0.])
        self.assertGreaterEqual(retries[0]["flight_health_age_s"], 0.)

    def test_capture_bad_header_pose_discards_old_chunks_before_recovery(self):
        self.options.cloud_timeout = 2.
        self.options.cloud_window_s = .25
        self.node._a5_measured_pose = lambda stamp=None, **_kwargs: (
            [.3, 0., 1.5, 0.] if stamp is not None and stamp.to_sec() == 10.25 else self.goal)
        self.capture([(10. + index * .125, "uav1/livox") for index in range(1, 17)])
        with np.load(self.node._a5_observations[0], allow_pickle=False) as data:
            self.assertGreaterEqual(data["chunk_stamps_s"][0], 11.125)
            self.assertNotIn(10.125, data["chunk_stamps_s"])
        retries = [details for status, details in self.statuses if status == "A5_CAPTURE_RETRY"]
        self.assertEqual(len(retries), 1)
        self.assertEqual(retries[0]["reason"], "stamped_pose_outside_hover")
        self.assertEqual(retries[0]["stamped_uav_pose_map"], [.3, 0., 1.5, 0.])

    def test_recovery_settling_must_be_continuous_without_a_second_deadline(self):
        self.options.cloud_timeout = 2.
        self.options.cloud_window_s = .25
        self.node._a5_settled_now = lambda _goal, **_kwargs: (
            not (10.24 <= self.clock.sim < 10.3 or 10.74 <= self.clock.sim < 10.8), self.goal)
        self.capture([(10. + index * .125, "uav1/livox") for index in range(1, 17)])
        with np.load(self.node._a5_observations[0], allow_pickle=False) as data:
            self.assertGreaterEqual(data["chunk_stamps_s"][0], 11.625)
        self.assertLessEqual(self.clock.wall, self.options.cloud_timeout)
        self.assertEqual(sum(status == "A5_CAPTURE_RETRY" for status, _ in self.statuses), 1)

    def test_permanent_instability_times_out_once_without_a_partial_observation(self):
        self.node._a5_settled_now = lambda _goal, **_kwargs: (self.clock.sim < 10.2, self.goal)
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.capture([(10.1, "uav1/livox"), (10.2, "uav1/livox")])
        self.assertLessEqual(self.clock.wall, self.options.cloud_timeout + .1)
        self.assertEqual(self.node._a5_observations, [])
        self.assertEqual(list(self.node._a5_output.glob("*.npz")), [])
        self.assertEqual(sum(status == "A5_CAPTURE_RETRY" for status, _ in self.statuses), 1)

    def test_missing_current_tf_breaks_the_partial_window_and_settling_continuity(self):
        self.options.cloud_timeout = 2.
        self.options.cloud_window_s = .25

        def settled(_goal, **_kwargs):
            if 10.24 <= self.clock.sim < 10.3:
                raise LookupError("current pose TF unavailable")
            return True, self.goal

        self.node._a5_settled_now = settled
        self.capture([(10. + index * .125, "uav1/livox") for index in range(1, 17)])
        with np.load(self.node._a5_observations[0], allow_pickle=False) as data:
            self.assertGreaterEqual(data["chunk_stamps_s"][0], 11.125)
            self.assertNotIn(10.125, data["chunk_stamps_s"])

    def test_invalid_transform_remains_an_error_instead_of_a_capture_retry(self):
        lookup = self.node._tf_buffer.lookup_transform

        def invalid_lookup(target, source, stamp, timeout):
            message = lookup(target, source, stamp, timeout)
            if stamp.to_sec() >= 10.2:
                message.transform.rotation = SimpleNamespace(x=0., y=0., z=0., w=0.)
            return message

        self.node._tf_buffer.lookup_transform = invalid_lookup
        with self.assertRaisesRegex(ValueError, "invalid transform quaternion"):
            self.capture([(10.1, "uav1/livox"), (10.2, "uav1/livox")])
        self.assertEqual(self.node._a5_observations, [])
        self.assertFalse(any(status == "A5_CAPTURE_RETRY" for status, _ in self.statuses))

    def test_delayed_core_drift_allows_steady_capture_at_one_measured_anchor(self):
        anchor = [0., 0., 1.35, 0.]
        self.node._a5_measured_pose = lambda stamp=None, **_kwargs: list(anchor)
        settling_goals = []

        def wait_settled(goal):
            settling_goals.append(list(goal))
            self.assertTrue(support.pose_settled(anchor, goal, [0., 0., 0.], .1, .1, .1))
            return list(anchor)

        self.node._a5_wait_settled = wait_settled
        self.node._a5_settled_now = lambda goal, **_kwargs: (
            support.pose_settled(anchor, goal, [0., 0., 0.], .1, .1, .1), list(anchor))
        pose = self.capture([(10.1, "uav1/livox"), (10.2, "uav1/livox"),
                             (10.5, "uav1/livox")])
        self.assertEqual(settling_goals, [anchor])
        self.assertEqual(pose, anchor)
        self.assertEqual(len(self.node._a5_observations), 1)
        anchors = [details for status, details in self.statuses if status == "A5_CAPTURE_ANCHOR"]
        self.assertEqual(len(anchors), 1)
        self.assertEqual(anchors[0]["requested_viewpoint"], self.goal)
        self.assertEqual(anchors[0]["capture_anchor_map"], anchor)
        self.assertAlmostEqual(anchors[0]["position_discrepancy_m"], .15)
        self.assertAlmostEqual(anchors[0]["yaw_discrepancy_rad"], 0.)
        observation = next(details for status, details in self.statuses if status == "A5_OBSERVATION")
        self.assertEqual(observation["requested_viewpoint"], self.goal)
        self.assertEqual(observation["capture_anchor_map"], anchor)

    def test_motion_relative_to_anchor_discards_window_without_following_the_drift(self):
        anchor = [0., 0., 1.35, 0.]
        anchor_reads, used_goals = [], []

        def current():
            return list(anchor) if self.clock.sim < 10.2 else [.2, 0., 1.35, 0.]

        def measured(stamp=None, **_kwargs):
            if stamp is None:
                anchor_reads.append(current())
            return current()

        def settled(goal, **_kwargs):
            used_goals.append(list(goal))
            return support.pose_settled(current(), goal, [0., 0., 0.], .1, .1, .1), current()

        self.node._a5_measured_pose = measured
        self.node._a5_settled_now = settled
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.capture([(10.1, "uav1/livox"), (10.2, "uav1/livox"),
                          (10.5, "uav1/livox")])
        self.assertEqual(anchor_reads, [anchor])
        self.assertTrue(all(goal == anchor for goal in used_goals))
        self.assertEqual(self.node._a5_observations, [])
        retry = next(details for status, details in self.statuses if status == "A5_CAPTURE_RETRY")
        self.assertEqual(retry["discarded_chunks"], 1)
        self.assertEqual(retry["goal_map"], anchor)
        self.assertEqual(retry["requested_viewpoint"], self.goal)
        self.assertEqual(retry["capture_anchor_map"], anchor)

    def test_capture_rejects_measured_anchor_outside_existing_flight_bounds(self):
        self.node._a5_measured_pose = lambda stamp=None, **_kwargs: [0., 0., .4, 0.]
        with self.assertRaisesRegex(RuntimeError, "flight bounds"):
            self.capture([(10.1, "uav1/livox")])
        self.assertEqual(self.node._a5_observations, [])

    def test_capture_anchor_requires_fresh_public_map_tf(self):
        lookup = self.node._tf_buffer.lookup_transform

        def stale_lookup(target, source, stamp, timeout):
            message = lookup(target, source, stamp, timeout)
            message.header = SimpleNamespace(stamp=self.Stamp(self.clock.sim - .6))
            return message

        self.node._tf_buffer.lookup_transform = stale_lookup
        self.node._a5_measured_pose = type(self.node)._a5_measured_pose.__get__(self.node)
        with self.assertRaisesRegex(RuntimeError, "TF is stale"):
            self.capture([(10.1, "uav1/livox"), (10.5, "uav1/livox")])
        self.assertEqual(self.node._a5_observations, [])
        self.assertFalse(any(status == "A5_CAPTURE_ANCHOR" for status, _ in self.statuses))

    def test_capture_wall_timeout_does_not_save_an_incomplete_window(self):
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.capture([(10.1, "uav1/livox")])
        self.assertLessEqual(self.clock.wall, self.options.cloud_timeout + .1)
        self.assertEqual(self.node._a5_observations, [])


class AdapterImportTests(unittest.TestCase):
    def test_cloud_window_defaults_to_five_seconds_and_is_positive_below_timeout(self):
        spec = importlib.util.spec_from_file_location("a5_adapter", ROOT / "scripts/run_a5_sim.py")
        adapter = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(adapter)
        parser = adapter.build_parser()
        required = ["--output-dir", "/tmp/a5-test", "--core-python", sys.executable,
                    "--rm4d-root", "/tmp", "--rm4d-config", "/tmp/config.json",
                    "--rm4d-map", "/tmp/map.npy"]
        defaults = parser.parse_args(required)
        self.assertEqual(getattr(defaults, "cloud_window_s", None), 5.)
        self.assertEqual(defaults.cloud_timeout, 20.)
        for value in ["0", "-1", "nan", "inf", "20", "21"]:
            with self.subTest(value=value), patch("sys.stderr"), self.assertRaises(SystemExit) as raised:
                adapter.validate_options(parser, parser.parse_args(required + ["--cloud-window-s", value]))
            self.assertEqual(raised.exception.code, 2)
        with patch.object(Path, "is_file", return_value=True), patch.object(Path, "is_dir", return_value=True):
            adapter.validate_options(parser, defaults)
            options = parser.parse_args(required + ["--cloud-window-s", "3.0"])
            adapter.validate_options(parser, options)
            self.assertEqual(options.cloud_window_s, 3.)

    def test_ground_travel_is_an_optional_runtime_override_not_a_method_change(self):
        spec = importlib.util.spec_from_file_location("a5_adapter", ROOT / "scripts/run_a5_sim.py")
        adapter = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(adapter)
        parser = adapter.build_parser()
        required = ["--output-dir", "/tmp/a5-test", "--core-python", sys.executable,
                    "--rm4d-root", "/tmp", "--rm4d-config", "/tmp/config.json",
                    "--rm4d-map", "/tmp/map.npy"]
        original = {"max_ground_travel": 1.1, "view_position": [0, 0, 1.5],
                    "view_yaw": 0., "map_frame": "map"}
        defaults = adapter.build_demo_parameters(original, parser.parse_args(required))
        self.assertEqual(defaults["max_ground_travel"], 1.1)
        options = parser.parse_args(required + ["--max-ground-travel", "3.0"])
        self.assertEqual(adapter.build_demo_parameters(original, options)["max_ground_travel"], 3.)
        self.assertEqual(original["max_ground_travel"], 1.1)
        self.assertEqual(defaults["placement_mode"], "rm4d")
        for value in ["0", "-1", "nan", "inf"]:
            with self.subTest(value=value), patch("sys.stderr"), self.assertRaises(SystemExit):
                adapter.validate_options(parser, parser.parse_args(required + ["--max-ground-travel", value]))

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
