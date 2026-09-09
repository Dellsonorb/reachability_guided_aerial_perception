"""Optional initial RGB-D evidence for A5 v1.1; no simulator or core imports at load."""

from collections import deque
import importlib.util
import json
import math
from pathlib import Path
import threading

import numpy as np


def match_bundle(bundles, accepted_stamp_s):
    """Match the accepted color stamp exactly; a nearby image is not its evidence."""
    try:
        stamp = float(accepted_stamp_s)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(stamp) or stamp <= 0:
        return None
    for bundle in reversed(tuple(bundles)):
        try:
            if len(bundle) == 4 and bundle[0].header.stamp.to_sec() == stamp:
                return bundle
        except (AttributeError, TypeError):
            continue
    return None


def _rigid_matrix(value):
    matrix = np.asarray(value, dtype=np.float64)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1], rtol=0, atol=1e-9) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), rtol=0, atol=1e-6) or
            not np.isclose(np.linalg.det(matrix[:3, :3]), 1., rtol=0, atol=1e-6)):
        raise ValueError('reference requires proper finite map camera transforms')
    return matrix


def _calibration(value):
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape == (9,):
        matrix = matrix.reshape(3, 3)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all() or
            matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or abs(matrix[2, 2]) <= 1e-12):
        raise ValueError('reference camera calibration is invalid')
    return matrix


def freeze_reference(sample, *, target_map, target_size, accepted_stamp_s, now_s,
                     T_map_color, T_map_depth, config, perception):
    """Build numerical evidence from decoded images and image-time public TF.

    The two camera transforms may come from different times. Their composition
    registers the stationary scene across that skew, without a motion allowance.
    This helper performs no I/O and never supplies inferred TARGET labels.
    """
    frames = tuple(str(value).lstrip('/') for value in sample['header_frame_ids'])
    stamps = np.asarray(sample['header_stamps_s'], dtype=np.float64)
    stamp, now = float(accepted_stamp_s), float(now_s)
    if (len(frames) != 4 or not all(frames) or
            frames[0] != config['camera_optical_frame'].lstrip('/') or
            frames[2] != frames[0] or frames[3] != frames[1] or
            config['target_frame'].lstrip('/') != 'map'):
        raise ValueError('reference image/CameraInfo frames differ from configured frames')
    if (stamps.shape != (4,) or not np.isfinite(stamps).all() or np.any(stamps <= 0) or
            not math.isfinite(stamp) or stamp <= 0 or stamps[0] != stamp or
            not math.isfinite(now) or now <= 0 or
            now - stamps.min() > float(config['max_observation_age']) or
            stamps.max() - now > float(config['max_future_skew']) or
            np.ptp(stamps) > float(config['sync_slop']) + 1e-9):
        raise ValueError('reference timestamps are mismatched, stale or invalid')
    rgb, raw_depth = np.asarray(sample['color_rgb']), np.asarray(sample['depth_raw'])
    if (rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8 or
            raw_depth.ndim != 2 or
            list(sample['image_shapes']) != [list(rgb.shape[:2]), list(raw_depth.shape)] or
            list(sample['info_shapes']) != [list(rgb.shape[:2]), list(raw_depth.shape)]):
        raise ValueError('reference image and CameraInfo dimensions differ')
    encoding = sample['depth_encoding']
    if encoding in ('16UC1', 'mono16') and raw_depth.dtype == np.uint16:
        depth = raw_depth.astype(np.float32) * .001
    elif encoding == '32FC1' and raw_depth.dtype == np.float32:
        depth = raw_depth.copy()
    else:
        raise ValueError('reference depth encoding or decoded type is unsupported')
    color_k, depth_k = _calibration(sample['color_K']), _calibration(sample['depth_K'])
    map_color, map_depth = _rigid_matrix(T_map_color), _rigid_matrix(T_map_depth)
    target, size = np.asarray(target_map, dtype=np.float64), np.asarray(target_size, dtype=np.float64)
    if (target.shape != (4,) or size.shape != (3,) or not np.isfinite(target).all() or
            not np.isfinite(size).all() or np.any(size <= 0)):
        raise ValueError('reference perceived target geometry is invalid')
    if perception is None:
        raise ValueError('reference segmentation implementation is unavailable')
    color_depth = np.linalg.solve(map_color, map_depth)
    try:
        perception.validate_observation_stamps(
            stamps, now, config['max_observation_age'], config['max_future_skew'])
        mask = perception.select_red_component(
            rgb, min_pixels=int(config['min_pixels']), ambiguity_ratio=float(config['ambiguity_ratio']))
        registered = perception.register_depth_to_color(
            depth, depth_k, color_k, rgb.shape[:2], color_depth[:3, :3], color_depth[:3, 3])
        points = perception.backproject_mask(
            mask, registered, color_k, float(config['min_depth']), float(config['max_depth']))
        if len(points) < int(config['minimum_points']):
            raise ValueError('reference has too few valid red depth points')
    except perception.PerceptionError as error:
        raise ValueError(str(error)) from error
    finite = np.isfinite(registered)
    depth_valid = np.zeros(registered.shape, dtype=bool)
    depth_valid[finite] = ((registered[finite] > float(config['min_depth'])) &
                           (registered[finite] < float(config['max_depth'])))
    # The worker must not reuse red pixels that the public observer excluded.
    # Preserve the original images and segmentation separately for replay.
    registered[~depth_valid] = np.nan
    valid = (mask != 0) & depth_valid
    return dict(
        schema_version=np.asarray(1), source=np.asarray('runtime_rgbd'),
        reference_scope=np.asarray('static_air_phase'), status=np.asarray('AVAILABLE'),
        frame_id=np.asarray('map'), stamp_s=np.asarray(stamp),
        mask=np.asarray(mask != 0), registered_depth_m=registered,
        color_rgb=rgb.copy(), depth_raw=raw_depth.copy(), depth_m=depth,
        color_encoding=np.asarray(sample['color_encoding']), depth_encoding=np.asarray(encoding),
        header_frame_ids=np.asarray(sample['header_frame_ids']), header_stamps_s=stamps.copy(),
        image_shapes=np.asarray(sample['image_shapes']), info_shapes=np.asarray(sample['info_shapes']),
        color_K=color_k.copy(), depth_K=depth_k.copy(), T_map_color=map_color.copy(),
        T_map_depth=map_depth.copy(), T_color_depth=color_depth,
        red_surface_points_map=points @ map_color[:3, :3].T + map_color[:3, 3],
        red_surface_pixels_rc=np.column_stack(np.nonzero(valid)),
        target_pose_map_xyzyaw=target.copy(), target_size_xyz=size.copy(),
        acquisition_ros_time_s=np.asarray(now))


def _load_perception(sim_root):
    path = Path(sim_root) / 'src/demos/air_ground_pick_demo/src/air_ground_pick_demo/perception.py'
    spec = importlib.util.spec_from_file_location('a5_reference_sim_perception', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_config(sim_root):
    import yaml
    path = Path(sim_root) / 'src/demos/air_ground_pick_demo/config/air_observer.yaml'
    with path.open(encoding='utf-8') as source:
        return yaml.safe_load(source)


def _runtime_interfaces():
    from types import SimpleNamespace
    import rospy
    import message_filters
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image
    return SimpleNamespace(rospy=rospy, message_filters=message_filters, CvBridge=CvBridge,
                           CameraInfo=CameraInfo, Image=Image)


def build_object_aware_adapter(parent_class, options):
    """Add initial RGB-D evidence capture to an A5-compatible adapter class."""
    runtime = _runtime_interfaces()

    class ObjectAwareAdapter(parent_class):
        def __init__(self):
            self._target_bundles = deque(maxlen=32)
            self._target_bundle_lock = threading.Lock()
            self._target_reference_done = False
            self._operational_init = {'operational_gating': getattr(options, 'operational_gating', 'v1.1'),
                                      'target_reference_file': None,
                                      'target_reference_status': 'UNAVAILABLE'}
            super().__init__()
            self._target_config = _load_config(options.sim_root)
            self._target_bridge = runtime.CvBridge()
            self._target_subscribers = tuple(
                runtime.message_filters.Subscriber(self._target_config[key], message_type)
                for key, message_type in (
                    ('color_topic', runtime.Image), ('depth_topic', runtime.Image),
                    ('color_info_topic', runtime.CameraInfo), ('depth_info_topic', runtime.CameraInfo)))
            self._target_sync = runtime.message_filters.ApproximateTimeSynchronizer(
                self._target_subscribers, queue_size=int(self._target_config['sync_queue_size']),
                slop=float(self._target_config['sync_slop']))
            self._target_sync.registerCallback(self._target_bundle_callback)

        def _target_bundle_callback(self, color, depth, color_info, depth_info):
            with self._target_bundle_lock:
                if not self._target_reference_done:
                    self._target_bundles.append((color, depth, color_info, depth_info))

        def _freeze_target_reference(self, target_map, observation_stamp):
            with self._target_bundle_lock:
                bundle = match_bundle(self._target_bundles, observation_stamp)
                self._target_reference_done = True
                self._target_bundles.clear()
            target = {'center_xyz': list(target_map[:3]), 'yaw_rad': target_map[3],
                      'size_xyz': list(self._target_size)}
            metadata = dict(status='UNAVAILABLE', reason=None, reference_file=None,
                            frame_id=self._map_frame, stamp_s=observation_stamp,
                            perceived_target=target, mask_pixels=0, points=0)
            try:
                if bundle is None:
                    raise ValueError('no cached RGB-D bundle matches accepted AIR_HANDOFF stamp')
                color, depth, color_info, depth_info = bundle
                sample = dict(
                    color_rgb=self._target_bridge.imgmsg_to_cv2(color, desired_encoding='rgb8'),
                    depth_raw=self._target_bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough'),
                    color_encoding=color.encoding, depth_encoding=depth.encoding,
                    header_frame_ids=[message.header.frame_id for message in bundle],
                    header_stamps_s=[message.header.stamp.to_sec() for message in bundle],
                    image_shapes=[[color.height, color.width], [depth.height, depth.width]],
                    info_shapes=[[color_info.height, color_info.width], [depth_info.height, depth_info.width]],
                    color_K=color_info.K, depth_K=depth_info.K)
                matrices = [self._a5_matrix(self._tf_buffer.lookup_transform(
                    self._map_frame, message.header.frame_id.lstrip('/'), message.header.stamp,
                    runtime.rospy.Duration(0.))) for message in (color, depth)]
                perception = _load_perception(options.sim_root)
                reference = freeze_reference(
                    sample, target_map=target_map, target_size=self._target_size,
                    accepted_stamp_s=observation_stamp, now_s=runtime.rospy.Time.now().to_sec(),
                    T_map_color=matrices[0], T_map_depth=matrices[1], config=self._target_config,
                    perception=perception)
                perception.validate_observation_stamps(
                    reference['header_stamps_s'], runtime.rospy.Time.now().to_sec(),
                    self._target_config['max_observation_age'], self._target_config['max_future_skew'])
                path = self._a5_output / 'target_reference.npz'
                with path.open('xb') as destination:
                    np.savez_compressed(destination, **reference)
                metadata.update(status='AVAILABLE', reference_file=str(path),
                                mask_pixels=int(reference['mask'].sum()),
                                points=len(reference['red_surface_points_map']))
            except Exception as error:
                # This optional evidence boundary fails closed without changing
                # the inherited observer's acceptance or execution parameters.
                metadata['reason'] = '%s: %s' % (type(error).__name__, error)
            self._operational_init = dict(
                operational_gating=getattr(options, 'operational_gating', 'v1.1'), perceived_target=target,
                target_reference_file=metadata['reference_file'],
                target_reference_status=metadata['status'], target_reference_reason=metadata['reason'])
            try:
                with (self._a5_output / 'target_reference.json').open('x', encoding='utf-8') as destination:
                    json.dump(metadata, destination, indent=2, sort_keys=True, allow_nan=False)
                    destination.write('\n')
            except OSError as error:
                metadata.update(status='UNAVAILABLE', reason='reference metadata could not be saved: %s' % error,
                                reference_file=None)
                self._operational_init.update(target_reference_file=None, target_reference_status='UNAVAILABLE',
                                              target_reference_reason=metadata['reason'])
            return metadata

        def _publish_status(self, state, **details):
            reference = None
            if state == 'AIR_HANDOFF' and not self._target_reference_done:
                reference = self._freeze_target_reference(details['target_map'], details['observation_stamp'])
            result = super()._publish_status(state, **details)
            if reference is not None:
                self._publish_status('TARGET_REFERENCE', **reference)
            return result

    return ObjectAwareAdapter
