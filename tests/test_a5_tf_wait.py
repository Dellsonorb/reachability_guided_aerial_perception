"""A5 pose TF readiness uses wall time while preserving the requested stamp."""

import copy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TransformException(Exception):
    pass


class LookupException(TransformException):
    pass


class ConnectivityException(TransformException):
    pass


class ExtrapolationException(TransformException):
    pass


class InvalidArgumentException(TransformException):
    pass


class ROSInterruptException(Exception):
    pass


class Stamp:
    def __init__(self, seconds=0.):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


class WallClock:
    def __init__(self):
        self.wall = 10.
        self.ros = 222.406
        self.sleeps = []
        self.on_yield = lambda: None

    def monotonic(self):
        return self.wall

    def sleep(self, seconds):
        if not 0. < seconds <= .01:
            raise AssertionError("TF readiness must yield briefly in wall time")
        self.sleeps.append(seconds)
        self.wall += seconds
        self.on_yield()


class TfWallWaitTests(unittest.TestCase):
    def setUp(self):
        self.adapter = load_script("run_a5_sim")
        self.clock = WallClock()
        self.ros = SimpleNamespace(
            Time=Stamp, Duration=Stamp, is_shutdown=lambda: False,
            ROSInterruptException=ROSInterruptException)
        self.errors = (LookupException, ConnectivityException, ExtrapolationException)
        self.stamp = Stamp(222.406)
        self.calls = []
        self.transform = object()

    def lookup(self, operation):
        def call(target, source, stamp, timeout):
            self.calls.append((target, source, stamp, timeout.to_sec(), self.clock.wall))
            return operation()

        self.assertTrue(hasattr(self.adapter, "lookup_transform_wall"),
                        "A5 needs a bounded wall-time TF readiness helper")
        with patch.object(self.adapter, "time", self.clock):
            return self.adapter.lookup_transform_wall(
                SimpleNamespace(lookup_transform=call), "planning", "map",
                self.stamp, self.ros, self.errors)

    def test_clock_catchup_before_tf_delivery_keeps_exact_stamp(self):
        cache = SimpleNamespace(latest=222.405)

        def deliver_callbacks():
            if len(self.clock.sleeps) == 1:
                self.clock.ros = 223.650
            else:
                cache.latest = 222.425

        def transform_when_bracketed():
            if cache.latest < self.stamp.to_sec():
                raise ExtrapolationException("requested 222.406, latest 222.405")
            return self.transform

        self.clock.on_yield = deliver_callbacks
        self.assertIs(self.lookup(transform_when_bracketed), self.transform)
        self.assertEqual(self.clock.ros, 223.650)
        self.assertEqual(len(self.calls), 3)
        self.assertLess(self.clock.wall - 10., .5)
        for target, source, stamp, timeout, _wall in self.calls:
            self.assertEqual((target, source, timeout), ("planning", "map", 0.))
            self.assertIs(stamp, self.stamp)

    def test_missing_tf_expires_with_frozen_ros_clock_and_original_failure(self):
        error = LookupException("missing map to planning transform")

        def missing():
            raise error

        with self.assertRaises(LookupException) as raised:
            self.lookup(missing)
        self.assertIs(raised.exception, error)
        self.assertEqual(self.clock.ros, 222.406)
        self.assertAlmostEqual(self.clock.wall, 10.5)
        self.assertGreater(len(self.calls), 1)
        self.assertTrue(all(call[2] is self.stamp and call[3] == 0. for call in self.calls))
        self.assertTrue(all(call[4] < 10.5 for call in self.calls))

    def test_transform_arriving_after_deadline_does_not_extend_wait(self):
        error = ExtrapolationException("not yet bracketed")
        available = []

        def delayed_yield():
            self.clock.wall = 10.6
            available.append(self.transform)

        def lookup():
            if available:
                return available[0]
            raise error

        self.clock.on_yield = delayed_yield
        with self.assertRaises(ExtrapolationException) as raised:
            self.lookup(lookup)
        self.assertIs(raised.exception, error)
        self.assertEqual(len(self.calls), 1)

    def test_transient_lookup_connectivity_and_extrapolation_failures_can_recover(self):
        failures = [LookupException("frame absent"),
                    ConnectivityException("tree not connected"),
                    ExtrapolationException("stamp not bracketed")]

        def lookup():
            if failures:
                raise failures.pop(0)
            return self.transform

        self.assertIs(self.lookup(lookup), self.transform)
        self.assertEqual(len(self.calls), 4)

    def test_non_tf_exception_propagates_without_retry(self):
        error = ValueError("invalid buffer operation")

        def invalid():
            raise error

        with self.assertRaises(ValueError) as raised:
            self.lookup(invalid)
        self.assertIs(raised.exception, error)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(self.clock.sleeps, [])

    def test_invalid_tf_argument_propagates_without_retry(self):
        error = InvalidArgumentException("invalid frame argument")

        def invalid():
            raise error

        with self.assertRaises(InvalidArgumentException) as raised:
            self.lookup(invalid)
        self.assertIs(raised.exception, error)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(self.clock.sleeps, [])

    def test_shutdown_before_lookup_is_honored(self):
        self.ros.is_shutdown = lambda: True
        with self.assertRaises(ROSInterruptException):
            self.lookup(lambda: self.transform)
        self.assertEqual(self.calls, [])
        self.assertEqual(self.clock.sleeps, [])

    def test_shutdown_during_wait_does_not_retry(self):
        def stop():
            self.ros.is_shutdown = lambda: True

        def missing():
            raise LookupException("unavailable before shutdown")

        self.clock.on_yield = stop
        with self.assertRaises(ROSInterruptException):
            self.lookup(missing)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(len(self.clock.sleeps), 1)


class PoseTransformWiringTests(unittest.TestCase):
    def setUp(self):
        self.adapter = load_script("run_a5_sim")
        self.calls = []
        self.transformed = []
        self.ros = SimpleNamespace(
            Time=Stamp, Duration=Stamp, is_shutdown=lambda: False,
            ROSInterruptException=ROSInterruptException)

        class Base:
            def _transform_pose(self, pose, target_frame, use_latest=False):
                if pose.header.frame_id != target_frame:
                    raise AssertionError("A5 must override cross-frame TF readiness")
                return copy.copy(pose)

        def apply_transform(pose, transform):
            self.transformed.append((pose, transform))
            return SimpleNamespace(header=SimpleNamespace(
                frame_id="tf-frame", stamp=transform.header.stamp), pose=pose.pose)

        demo = SimpleNamespace(
            AirGroundPickDemo=Base, DemoError=RuntimeError, do_transform_pose=apply_transform)
        modules = {
            "rospy": self.ros,
            "tf2_ros": SimpleNamespace(
                TransformException=TransformException, LookupException=LookupException,
                ConnectivityException=ConnectivityException,
                ExtrapolationException=ExtrapolationException),
            "sensor_msgs": SimpleNamespace(point_cloud2=None),
            "sensor_msgs.msg": SimpleNamespace(PointCloud2=object),
            "std_msgs.msg": SimpleNamespace(String=object),
            "a5_ros_support": load_script("a5_ros_support"),
        }
        with patch.dict(sys.modules, modules):
            cls = self.adapter.build_adapter_class(demo, SimpleNamespace())
        self.node = object.__new__(cls)
        self.transform = SimpleNamespace(header=SimpleNamespace(stamp=Stamp(333.)))

        def lookup(target, source, stamp, timeout):
            self.calls.append((target, source, stamp, timeout.to_sec()))
            return self.transform

        self.node._tf_buffer = SimpleNamespace(lookup_transform=lookup)

    def pose(self, stamp):
        return SimpleNamespace(header=SimpleNamespace(frame_id="/map", stamp=stamp),
                               pose=SimpleNamespace(position=(1., 2., 3.)))

    def test_same_frame_keeps_inherited_copy_without_lookup(self):
        for use_latest in (False, True):
            with self.subTest(use_latest=use_latest):
                pose = self.pose(Stamp(222.406))
                result = self.node._transform_pose(pose, "/map", use_latest=use_latest)
                self.assertIsNot(result, pose)
                self.assertIs(result.header, pose.header)
                self.assertIs(result.pose, pose.pose)
        self.assertEqual(self.calls, [])
        self.assertEqual(self.transformed, [])

    def test_exact_and_zero_stamps_are_forwarded_without_replacement(self):
        for seconds in (222.406, 0.):
            with self.subTest(seconds=seconds):
                pose = self.pose(Stamp(seconds))
                result = self.node._transform_pose(pose, "/planning")
                target, source, stamp, timeout = self.calls[-1]
                self.assertEqual((target, source, timeout), ("/planning", "/map", 0.))
                self.assertIs(stamp, pose.header.stamp)
                self.assertIs(self.transformed[-1][0], pose)
                self.assertIs(self.transformed[-1][1], self.transform)
                self.assertEqual(result.header.frame_id, "/planning")
                self.assertIs(result.header.stamp, self.transform.header.stamp)
                self.assertEqual(pose.header.frame_id, "/map")

    def test_explicit_latest_floor_path_requests_zero_and_preserves_input(self):
        pose = self.pose(Stamp(222.406))
        original_stamp = pose.header.stamp
        result = self.node._transform_pose(pose, "planning", use_latest=True)
        self.assertEqual(self.calls[-1][2].to_sec(), 0.)
        self.assertEqual(self.calls[-1][3], 0.)
        self.assertIs(pose.header.stamp, original_stamp)
        self.assertIs(self.transformed[-1][0], pose)
        self.assertEqual(result.header.frame_id, "planning")
        self.assertIs(result.header.stamp, self.transform.header.stamp)


class NativeTfClockOrderTests(unittest.TestCase):
    def test_native_tf_buffer_recovers_exact_stamp_after_clock_catchup(self):
        try:
            import rospy
            import tf2_ros
            from geometry_msgs.msg import TransformStamped
        except ImportError as error:
            self.skipTest("native ROS tf2 is unavailable: %s" % error)

        adapter = load_script("run_a5_sim")
        clock = WallClock()
        buffer = tf2_ros.Buffer(debug=False)
        stamp = rospy.Time.from_sec(222.406)

        def insert(seconds, x):
            transform = TransformStamped()
            transform.header.frame_id = "planning"
            transform.child_frame_id = "map"
            transform.header.stamp = rospy.Time.from_sec(seconds)
            transform.transform.translation.x = x
            transform.transform.rotation.w = 1.
            buffer.set_transform(transform, "clock-order-regression")

        insert(222.405, 1.)

        def catchup_before_tf():
            clock.ros = 223.650

        with patch.object(rospy.Time, "now", side_effect=lambda: rospy.Time.from_sec(clock.ros)), \
                patch.object(rospy, "sleep", side_effect=lambda _duration: catchup_before_tf()):
            with self.assertRaises(tf2_ros.ExtrapolationException):
                buffer.lookup_transform("planning", "map", stamp, rospy.Duration(.5))

        self.assertTrue(hasattr(adapter, "lookup_transform_wall"),
                        "A5 needs a bounded wall-time TF readiness helper")
        clock.ros = 222.406

        def deliver_callbacks():
            if len(clock.sleeps) == 1:
                catchup_before_tf()
            else:
                insert(222.425, 3.)

        clock.on_yield = deliver_callbacks
        with patch.object(adapter, "time", clock), \
                patch.object(rospy.Time, "now", side_effect=lambda: rospy.Time.from_sec(clock.ros)), \
                patch.object(rospy, "is_shutdown", return_value=False), \
                patch.object(rospy, "sleep", side_effect=AssertionError("must use wall sleep")):
            transform = adapter.lookup_transform_wall(
                buffer, "planning", "map", stamp, rospy,
                (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException))

        self.assertEqual(transform.header.stamp, stamp)
        self.assertAlmostEqual(transform.transform.translation.x, 1.1, places=6)
        self.assertEqual(len(clock.sleeps), 2)
        self.assertLess(clock.wall - 10., .5)


if __name__ == "__main__":
    unittest.main()
