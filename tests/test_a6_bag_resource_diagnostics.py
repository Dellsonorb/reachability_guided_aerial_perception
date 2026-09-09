"""Offline public-TF measurement tests; no ROS node or simulator is started."""

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import warnings

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/a6_bag_resource_diagnostics.py'
sys.path.insert(0, str(ROOT / 'scripts'))
from a6_metrics import summarize_metrics

try:
    import rosbag
    import rospy
    import tf2_py
    from geometry_msgs.msg import TransformStamped
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def sample(t, wall=None):
    pose = dict(frame_id='uav1/base_link', xyz=[999., 0., 0.], yaw=0., stamp_s=t)
    return dict(ros_time=t, wall_monotonic=t if wall is None else wall,
                uav=pose, ground=dict(pose, frame_id='ground/base_link'))


def event(state, t, wall=None, **values):
    return dict(state=state, ros_time=t, wall_monotonic=t if wall is None else wall, **values)


def transform(parent, child, stamp, x=0.):
    value = TransformStamped()
    value.header.frame_id, value.child_frame_id = parent, child
    value.header.stamp = rospy.Time.from_sec(stamp)
    value.transform.translation.x = x
    value.transform.rotation.w = 1.
    return value


def tf(receipt, parent, child, stamp, x=0., static=False):
    return ('/tf_static' if static else '/tf',
            TFMessage([transform(parent, child, stamp, x)]), rospy.Time.from_sec(receipt))


def clock(t, receipt=None):
    return '/clock', Clock(rospy.Time.from_sec(t)), rospy.Time.from_sec(t if receipt is None else receipt)


class ImportTests(unittest.TestCase):
    def test_reporter_exists_without_importing_ros_at_module_load(self):
        self.assertTrue(SCRIPT.is_file(), 'secondary bag resource reporter is not implemented')
        code = ('import importlib.util, sys; '
                's=importlib.util.spec_from_file_location("bag_report",sys.argv[1]); '
                'm=importlib.util.module_from_spec(s); s.loader.exec_module(m); '
                'assert not any(k in sys.modules for k in ("rospy","rosbag","tf2_py"))')
        subprocess.run([sys.executable, '-c', code, str(SCRIPT)], check=True)


@unittest.skipUnless(ROS_AVAILABLE, 'installed Noetic offline libraries required')
class BagResourceTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SCRIPT.is_file(), 'secondary bag resource reporter is not implemented')
        spec = importlib.util.spec_from_file_location('bag_resources', SCRIPT)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def test_future_header_and_late_receipt_are_excluded_until_both_are_eligible(self):
        messages = [clock(.9), tf(.9, 'map', 'uav1/base_link', .9, 1.),
                    tf(.91, 'map', 'uav1/base_link', 1.1, 9.),
                    tf(1.2, 'map', 'uav1/base_link', 1., 5.)]
        rows = self.module.reconstruct([sample(1.), sample(1.15)], messages)
        self.assertEqual(rows[0]['uav']['xyz'][0], 1.)
        self.assertEqual(rows[0]['uav']['stamp_s'], .9)
        self.assertEqual(rows[1]['uav']['xyz'][0], 9.)

    def test_chain_uses_latest_common_time_instead_of_different_edge_times(self):
        messages = [clock(.8), tf(.8, 'map', 'odom', .8, 8.),
                    tf(.9, 'odom', 'uav1/base_link', .9, 2.),
                    tf(1., 'map', 'odom', 1., 10.)]
        row = self.module.reconstruct([sample(1.1)], messages)[0]
        self.assertEqual(row['uav']['stamp_s'], .9)
        self.assertAlmostEqual(row['uav']['xyz'][0], 11.)

    def test_missing_stale_and_future_static_chain_edges_remain_missing(self):
        for messages in ([clock(.5), tf(.5, 'odom', 'uav1/base_link', .5)],
                         [clock(.5), tf(.5, 'map', 'uav1/base_link', .5)],
                         [clock(.9), tf(.9, 'map', 'odom', 2., static=True),
                          tf(.9, 'odom', 'uav1/base_link', .9)]):
            with self.subTest(messages=messages):
                row = self.module.reconstruct([sample(1.1)], messages)[0]
                self.assertIsNone(row['uav']['xyz'])
                self.assertTrue(row['uav']['missing_reason'])

    def test_static_zero_stamp_and_prefixed_frames_keep_the_dynamic_timestamp(self):
        messages = [clock(.9), tf(.9, '/map', '/odom', 0., 3., static=True),
                    tf(.95, '/odom', '/uav1/base_link', .95, 2.)]
        before = copy.deepcopy(messages)
        row = self.module.reconstruct([sample(1.)], messages)[0]
        self.assertEqual(row['uav']['xyz'], [5., 0., 0.])
        self.assertEqual(row['uav']['stamp_s'], .95)
        self.assertEqual(messages, before)

    def test_every_original_row_is_reconstructed_without_smoothing_or_new_rows(self):
        original = [sample(1.), sample(1.), sample(1.4)]
        messages = [clock(.9), tf(.9, 'map', 'uav1/base_link', .9, 1.),
                    tf(1.4, 'map', 'uav1/base_link', 1.4, 4.)]
        before = copy.deepcopy(original)
        rows = self.module.reconstruct(original, messages)
        self.assertEqual([(r['ros_time'], r['wall_monotonic']) for r in rows],
                         [(r['ros_time'], r['wall_monotonic']) for r in original])
        self.assertEqual([r['uav']['xyz'][0] for r in rows], [1., 1., 4.])
        self.assertEqual(original, before)
        result = summarize_metrics([event('A6_TASK_START', 1.), event('FAILED', 1.4)], rows, 'ours')
        self.assertIsNone(result['paths']['uav_total']['distance_m'])
        self.assertEqual(result['paths']['uav_total']['gap_count'], 1)

    def test_missing_clock_cannot_claim_bag_measurement_availability(self):
        rows = self.module.reconstruct([sample(1.)], [tf(.9, 'map', 'uav1/base_link', .9)])
        self.assertIsNone(rows[0]['uav']['xyz'])

    def test_sample_reset_invalidates_all_rows_before_consuming_bag_messages(self):
        original = [sample(1., 10.), sample(.5, 11.)]
        consumed = []
        def messages():
            for value in [clock(.9), tf(.9, 'map', 'uav1/base_link', .9)]:
                consumed.append(value)
                yield value
        rows = self.module.reconstruct(original, messages())
        self.assertEqual(consumed, [])
        self.assertEqual([r['ros_time'] for r in rows], [1., .5])
        for row in rows:
            for body in ('uav', 'ground'):
                self.assertIsNone(row[body]['xyz'])
                self.assertIn('clock reset', row[body]['missing_reason'])
        result = summarize_metrics([event('A6_TASK_START', 1., 10.), event('FAILED', .5, 11.)], rows, 'ours')
        self.assertTrue(result['clock_reset_detected'])
        self.assertIsNone(result['paths']['uav_total']['distance_m'])

    def write_attempt(self, directory):
        data = directory / 'data'
        data.mkdir(parents=True)
        events = [event('A6_TASK_START', 1., 10.), event('FAILED', 1.2, 12., reason='retained failure'),
                  event('A6_TASK_END', 1.3, 14.)]
        rows = [sample(1., 10.1), sample(1.2, 12.1), sample(1.3, 13.)]
        for filename, value in [('attempt.json', dict(method='ours', status='VALID_TRIAL', retrieval_success=False)),
                                ('data/metrics.json', summarize_metrics(events, rows, 'ours', False))]:
            (directory / filename).write_text(json.dumps(value))
        for filename, values in [('events.jsonl', events), ('trajectory.jsonl', rows)]:
            (data / filename).write_text(''.join(json.dumps(row) + '\n' for row in values))
        messages = [clock(.9), tf(.9, 'map', 'uav1/base_link', .9, 1.),
                    tf(.9, 'map', 'ground/base_link', .9, 1.),
                    tf(1.2, 'map', 'uav1/base_link', 1.2, 2.),
                    tf(1.2, 'map', 'ground/base_link', 1.2, 2.),
                    tf(1.3, 'map', 'uav1/base_link', 1.3, 10.)]
        # A conflicting transform on an unauthorized topic must have no effect.
        messages.append(('/gazebo/model_states', TFMessage([transform('map', 'uav1/base_link', 1.2, 999.)]),
                         rospy.Time.from_sec(1.2)))
        with rosbag.Bag(str(directory / 'diagnostics.bag'), 'w') as bag:
            for topic, message, receipt in messages:
                bag.write(topic, message, receipt)

    def test_event_only_cleanup_reset_invalidates_even_pre_terminal_rows(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / 'attempt'
            self.write_attempt(directory)
            path = directory / 'data/events.jsonl'
            events = [json.loads(line) for line in path.read_text().splitlines()]
            events[-1]['ros_time'] = .5
            path.write_text(''.join(json.dumps(row) + '\n' for row in events))
            result = self.module.describe_attempt(directory)
        self.assertEqual([row['ros_time'] for row in result['trace']], [1., 1.2, 1.3])
        for row in result['trace']:
            self.assertIsNone(row['bag_clock_s'])
            for body in ('uav', 'ground'):
                self.assertIsNone(row[body]['xyz'])
                self.assertIn('clock reset', row[body]['missing_reason'])
        self.assertIsNone(result['paths']['uav_total']['bag_derived']['distance_m'])

    def test_file_report_keeps_event_boundaries_outcomes_and_raw_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / 'attempt'
            self.write_attempt(directory)
            before = {p: p.read_bytes() for p in directory.rglob('*') if p.is_file()}
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter('always', UserWarning)
                result = self.module.describe_attempt(directory)
            self.assertEqual([str(w.message) for w in caught], [])
            self.assertEqual(before, {p: p.read_bytes() for p in directory.rglob('*') if p.is_file()})
        self.assertEqual(result['measurement_role'], 'SECONDARY_DIAGNOSTIC_ONLY')
        self.assertTrue(result['raw_metrics_reproduced'])
        self.assertTrue(result['non_path_metrics_unchanged'])
        self.assertEqual(result['topics'], ['/tf', '/tf_static', '/clock'])
        self.assertFalse(result['outcome_context']['retrieval_success'])
        self.assertEqual(result['paths']['uav_total']['bag_derived']['distance_m'], 1.)
        self.assertEqual(result['paths']['uav_cleanup']['bag_derived']['distance_m'], 8.)
        self.assertEqual(len(result['trace']), 3)

    def test_cli_stdout_and_exclusive_new_output_never_replace_input(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / 'attempt'
            self.write_attempt(directory)
            command = [sys.executable, str(SCRIPT), '--attempt-dir', str(directory)]
            run = subprocess.run(command, capture_output=True, text=True, check=True)
            self.assertEqual(json.loads(run.stdout)['measurement_role'], 'SECONDARY_DIAGNOSTIC_ONLY')
            output = Path(temporary) / 'secondary.json'
            subprocess.run(command + ['--output', str(output)], capture_output=True, check=True)
            before = output.read_bytes()
            repeated = subprocess.run(command + ['--output', str(output)], capture_output=True)
            self.assertNotEqual(repeated.returncode, 0)
            self.assertEqual(output.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
