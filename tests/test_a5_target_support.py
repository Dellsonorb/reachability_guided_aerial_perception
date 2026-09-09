"""Initial RGB-D reference capture; numerical integration runs with SIM's OpenCV."""

import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SIM = Path('/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation')
HAS_CV = importlib.util.find_spec('cv2') is not None


class Stamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds

    @staticmethod
    def now():
        return Stamp(10.1)


def bundle(stamp=10., depth_stamp=10.04):
    rgb = np.zeros((12, 16, 3), dtype=np.uint8)
    rgb[3:9, 5:11, 0] = 255
    calibration = [40., 0., 8., 0., 40., 6., 0., 0., 1.]

    def message(frame, seconds, data=None, encoding=None):
        return SimpleNamespace(header=SimpleNamespace(frame_id=frame, stamp=Stamp(seconds)),
                               height=12, width=16, data=data, encoding=encoding, K=calibration)
    return (message('uav1/camera_link', stamp, rgb, 'rgb8'),
            message('uav1/camera_depth_frame', depth_stamp,
                    np.full((12, 16), 1000, dtype=np.uint16), '16UC1'),
            message('uav1/camera_link', stamp),
            message('uav1/camera_depth_frame', depth_stamp))


def sample():
    messages = bundle()
    return dict(color_rgb=messages[0].data, depth_raw=messages[1].data,
                color_encoding='rgb8', depth_encoding='16UC1',
                header_frame_ids=[m.header.frame_id for m in messages],
                header_stamps_s=[m.header.stamp.to_sec() for m in messages],
                image_shapes=[[12, 16], [12, 16]], info_shapes=[[12, 16], [12, 16]],
                color_K=messages[2].K, depth_K=messages[3].K)


class TargetSupportTests(unittest.TestCase):
    def setUp(self):
        path = ROOT / 'scripts/a5_target_support.py'
        self.assertTrue(path.is_file(), 'initial target reference support must exist')
        spec = importlib.util.spec_from_file_location('target_support_under_test', path)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.config = dict(camera_optical_frame='uav1/camera_link', target_frame='map',
                           min_pixels=25, ambiguity_ratio=.65, minimum_points=25,
                           min_depth=.25, max_depth=4., sync_slop=.06,
                           max_observation_age=1., max_future_skew=.1, tf_timeout=.25,
                           color_topic='/uav1/camera/color/image_raw',
                           depth_topic='/uav1/camera/depth/image_raw',
                           color_info_topic='/uav1/camera/color/camera_info',
                           depth_info_topic='/uav1/camera/depth/camera_info', sync_queue_size=10)
        self.target, self.size = [2., .1, .0575, .2], [.24, .053, .115]

    def test_matches_exact_accepted_color_stamp_only(self):
        chosen = bundle()
        self.assertIs(self.module.match_bundle([bundle(9.99), chosen, bundle(10.01)], 10.), chosen)
        self.assertIsNone(self.module.match_bundle([bundle(10.01)], 10.))
        self.assertIsNone(self.module.match_bundle([None, ('bad',)], float('nan')))

    def freeze(self, values=None, perception=None):
        color, depth = np.eye(4), np.eye(4)
        color[:3, 3], depth[:3, 3] = [1., 2., 3.], [1.05, 2., 3.]
        return self.module.freeze_reference(
            sample() if values is None else values, target_map=self.target,
            target_size=self.size, accepted_stamp_s=10., now_s=10.1,
            T_map_color=color, T_map_depth=depth, config=self.config, perception=perception)

    def test_bad_frames_shapes_or_stamps_are_rejected_before_segmentation(self):
        for field, value, reason in [('header_frame_ids', ['wrong'] * 4, 'frames'),
                                     ('header_stamps_s', [9., 9., 9., 9.], 'timestamps'),
                                     ('header_stamps_s', [10., 10.2, 10., 10.2], 'timestamps'),
                                     ('image_shapes', [[13, 16], [12, 16]], 'dimensions'),
                                     ('color_K', [float('nan')] * 9, 'calibration'),
                                     ('depth_encoding', '8UC1', 'encoding')]:
            with self.subTest(field=field, value=value):
                values = sample()
                values[field] = value
                with self.assertRaisesRegex(ValueError, reason):
                    self.freeze(values)

    @unittest.skipUnless(HAS_CV, 'real SIM segmentation requires system Python OpenCV')
    def test_image_time_motion_registration_and_numeric_provenance(self):
        perception = self.module._load_perception(SIM)
        original = sample()
        before = copy.deepcopy(original)
        with np.errstate(invalid='raise'):
            reference = self.freeze(original, perception)
        self.assertEqual(reference['status'].item(), 'AVAILABLE')
        self.assertEqual(reference['frame_id'].item(), 'map')
        self.assertEqual(reference['stamp_s'].item(), 10.)
        np.testing.assert_allclose(reference['T_color_depth'][:3, 3], [.05, 0., 0.])
        self.assertTrue(np.isnan(reference['registered_depth_m'][:, :2]).all())
        self.assertEqual(reference['mask'].dtype, np.dtype(bool))
        self.assertEqual(reference['mask'].sum(), 36)
        self.assertEqual(reference['red_surface_points_map'].shape, (36, 3))
        self.assertTrue((reference['red_surface_points_map'][:, 2] == 4.).all())
        np.testing.assert_array_equal(original['depth_raw'], before['depth_raw'])
        np.testing.assert_array_equal(reference['depth_raw'], before['depth_raw'])
        np.testing.assert_allclose(reference['target_pose_map_xyzyaw'], self.target)

    @unittest.skipUnless(HAS_CV, 'real SIM segmentation requires system Python OpenCV')
    def test_no_red_or_no_depth_does_not_create_a_reference(self):
        perception = self.module._load_perception(SIM)
        for key in ('color_rgb', 'depth_raw'):
            with self.subTest(key=key):
                values = sample()
                values[key] = np.zeros_like(values[key])
                with self.assertRaises(ValueError):
                    self.freeze(values, perception)

    @unittest.skipUnless(HAS_CV, 'real SIM segmentation requires system Python OpenCV')
    def test_persisted_depth_excludes_every_pixel_outside_observer_gate(self):
        perception = self.module._load_perception(SIM)
        values = sample()
        values['depth_encoding'] = '32FC1'
        values['depth_raw'] = np.ones((12, 16), dtype=np.float32)
        values['depth_raw'][3, 5:11] = [.2, .25, 4., 4.5, np.nan, np.inf]
        values['depth_raw'][4, 5:7] = [0., -1.]
        original = values['depth_raw'].copy()
        with np.errstate(invalid='raise'):
            reference = self.module.freeze_reference(
                values, target_map=self.target, target_size=self.size,
                accepted_stamp_s=10., now_s=10.1, T_map_color=np.eye(4),
                T_map_depth=np.eye(4), config=self.config, perception=perception)
        self.assertTrue(np.isnan(reference['registered_depth_m'][3, 5:11]).all())
        self.assertTrue(np.isnan(reference['registered_depth_m'][4, 5:7]).all())
        self.assertEqual(reference['mask'].sum(), 36)
        self.assertEqual(reference['red_surface_points_map'].shape, (28, 3))
        self.assertEqual(reference['red_surface_pixels_rc'].shape, (28, 2))
        np.testing.assert_array_equal(reference['depth_raw'], original)
        np.testing.assert_array_equal(values['depth_raw'], original)

    def make_adapter(self, directory, revision='v1.1'):
        statuses = []
        config = self.config

        class Parent:
            def __init__(parent):
                parent._lock = threading.Lock()
                parent._a5_output, parent._map_frame = Path(directory), 'map'
                parent._target_size = [.24, .053, .115]
                parent._tf_buffer = SimpleNamespace(lookup_transform=lambda target, source, stamp, timeout:
                                                    (source, stamp.to_sec()))
                parent._publish_status('PARENT_START')

            def _a5_matrix(parent, transform):
                matrix = np.eye(4)
                if transform[0].endswith('depth_frame'):
                    matrix[0, 3] = .05
                return matrix

            def _publish_status(parent, state, **details):
                statuses.append((state, details))

        class Subscriber:
            def __init__(self, *args, **kwargs):
                pass

        class Synchronizer:
            def __init__(self, subscribers, queue_size, slop):
                pass

            def registerCallback(self, callback):
                self.callback = callback

        interfaces = SimpleNamespace(
            rospy=SimpleNamespace(Time=Stamp, Duration=lambda seconds: seconds),
            message_filters=SimpleNamespace(Subscriber=Subscriber,
                                           ApproximateTimeSynchronizer=Synchronizer),
            Image=object, CameraInfo=object,
            CvBridge=lambda: SimpleNamespace(imgmsg_to_cv2=lambda message, desired_encoding: message.data))
        with patch.object(self.module, '_runtime_interfaces', return_value=interfaces), \
                patch.object(self.module, '_load_config', return_value=config):
            adapter = self.module.build_object_aware_adapter(
                Parent, SimpleNamespace(sim_root=SIM, operational_gating=revision))()
        return adapter, statuses

    def test_v13_revision_reaches_initial_payload_even_without_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter, _ = self.make_adapter(directory, revision='v1.3')
            self.assertEqual(adapter._operational_init['operational_gating'], 'v1.3')
            adapter._publish_status('AIR_HANDOFF', target_map=self.target, observation_stamp=10.)
            self.assertEqual(adapter._operational_init['operational_gating'], 'v1.3')
            self.assertEqual(adapter._operational_init['target_reference_status'], 'UNAVAILABLE')

    def test_v14_revision_reaches_initial_payload_even_without_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter, _ = self.make_adapter(directory, revision='v1.4')
            self.assertEqual(adapter._operational_init['operational_gating'], 'v1.4')
            adapter._publish_status('AIR_HANDOFF', target_map=self.target, observation_stamp=10.)
            self.assertEqual(adapter._operational_init['operational_gating'], 'v1.4')
            self.assertEqual(adapter._operational_init['target_reference_status'], 'UNAVAILABLE')

    def test_missing_reference_preserves_handoff_and_blocks_without_waiting(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter, statuses = self.make_adapter(directory)
            adapter._publish_status('AIR_HANDOFF', target_map=self.target, observation_stamp=10.)
            self.assertEqual([state for state, _ in statuses],
                             ['PARENT_START', 'AIR_HANDOFF', 'TARGET_REFERENCE'])
            self.assertEqual(adapter._operational_init['target_reference_status'], 'UNAVAILABLE')
            self.assertIsNone(adapter._operational_init['target_reference_file'])
            metadata = json.loads((Path(directory) / 'target_reference.json').read_text())
            self.assertEqual(metadata['status'], 'UNAVAILABLE')
            self.assertFalse((Path(directory) / 'target_reference.npz').exists())

    def test_raw_cache_is_bounded_and_missing_exact_sample_is_unavailable(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter, _ = self.make_adapter(directory)
            for number in range(40):
                adapter._target_bundle_callback(*bundle(10. + number))
            self.assertEqual(len(adapter._target_bundles), 32)
            adapter._publish_status('AIR_HANDOFF', target_map=self.target, observation_stamp=10.)
            self.assertEqual(adapter._operational_init['target_reference_status'], 'UNAVAILABLE')

    def test_tf_failure_preserves_handoff_and_does_not_publish_available_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter, statuses = self.make_adapter(directory)
            adapter._target_bundle_callback(*bundle())
            def missing_transform(*args):
                raise LookupError('image-time TF unavailable')
            adapter._tf_buffer.lookup_transform = missing_transform
            adapter._publish_status('AIR_HANDOFF', target_map=self.target, observation_stamp=10.)
            self.assertEqual(adapter._operational_init['target_reference_status'], 'UNAVAILABLE')
            self.assertIn('image-time TF unavailable', adapter._operational_init['target_reference_reason'])
            self.assertEqual(statuses[-2][0], 'AIR_HANDOFF')
            self.assertFalse((Path(directory) / 'target_reference.npz').exists())

    @unittest.skipUnless(HAS_CV, 'real SIM segmentation requires system Python OpenCV')
    def test_malformed_bundle_fails_closed_at_adapter_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter, _ = self.make_adapter(directory)
            messages = bundle()
            messages[2].header.frame_id = 'wrong-camera-info'
            adapter._target_bundle_callback(*messages)
            adapter._publish_status('AIR_HANDOFF', target_map=self.target, observation_stamp=10.)
            self.assertEqual(adapter._operational_init['target_reference_status'], 'UNAVAILABLE')
            self.assertIn('frames differ', adapter._operational_init['target_reference_reason'])
            self.assertFalse((Path(directory) / 'target_reference.npz').exists())

    @unittest.skipUnless(HAS_CV, 'real SIM segmentation requires system Python OpenCV')
    def test_processing_that_exceeds_existing_observation_age_is_unavailable(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter, _ = self.make_adapter(directory)
            adapter._target_bundle_callback(*bundle())
            with patch.object(Stamp, 'now', side_effect=[Stamp(10.1), Stamp(11.2)]):
                adapter._publish_status('AIR_HANDOFF', target_map=self.target, observation_stamp=10.)
            self.assertEqual(adapter._operational_init['target_reference_status'], 'UNAVAILABLE')
            self.assertIn('stale', adapter._operational_init['target_reference_reason'])

    @unittest.skipUnless(HAS_CV, 'real SIM segmentation requires system Python OpenCV')
    def test_adapter_saves_replayable_reference_and_init_payload(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter, statuses = self.make_adapter(directory)
            adapter._target_bundle_callback(*bundle())
            adapter._publish_status('AIR_HANDOFF', target_map=self.target, observation_stamp=10.)
            payload = adapter._operational_init
            self.assertEqual(payload['operational_gating'], 'v1.1')
            self.assertEqual(payload['perceived_target']['center_xyz'], self.target[:3])
            self.assertEqual(payload['target_reference_status'], 'AVAILABLE')
            with np.load(payload['target_reference_file'], allow_pickle=False) as saved:
                self.assertEqual(saved['mask'].sum(), 36)
                np.testing.assert_allclose(saved['T_color_depth'][0, 3], .05)
                self.assertIn('header_stamps_s', saved.files)
                self.assertIn('color_rgb', saved.files)
                self.assertIn('depth_raw', saved.files)
            self.assertEqual(statuses[-1][0], 'TARGET_REFERENCE')
            self.assertEqual(statuses[-1][1]['mask_pixels'], 36)


if __name__ == '__main__':
    unittest.main()
