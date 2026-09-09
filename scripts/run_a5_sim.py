#!/usr/bin/python3
"""Run the A5 adapter against an already launched SIM runtime (run_demo=false)."""

import argparse
import importlib.util
import math
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SIM = "/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation"
DEMO_RELATIVE = Path("src/demos/air_ground_pick_demo")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim-root", type=Path, default=Path(DEFAULT_SIM))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--core-python", type=Path, required=True,
                        help="Python 3.10 interpreter with the frozen core/RM4D dependencies")
    parser.add_argument("--rm4d-root", type=Path, required=True)
    parser.add_argument("--rm4d-config", type=Path, required=True)
    parser.add_argument("--rm4d-map", type=Path, required=True)
    parser.add_argument("--rm4d-task-asset", type=Path,
                        help="independent calibrated runtime asset directory; frozen baseline remains unchanged")
    parser.add_argument("--operational-gating", choices=("v1", "v1.1", "v1.3", "v1.4"), default="v1",
                        help="explicit object-aware gate; v1.3 adds sub-cell ambiguity; v1.4 separates measured ground presence")
    parser.add_argument("--support-anchor", choices=("cell_center", "exact_winner"), default="cell_center",
                        help="explicit v1.2 original-winner support; default preserves legacy cell centers")
    parser.add_argument("--max-viewpoints", type=int, default=3,
                        help="observation budget, including the initial capture and rescans")
    parser.add_argument("--flight-bounds", type=float, nargs=6,
                        default=[-4., 4., -3., 3., .5, 3.],
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"))
    parser.add_argument("--xy-offsets-m", type=float, nargs="+", default=[-2., 0., 2.])
    parser.add_argument("--facade-position-tolerance", type=float, default=.15)
    parser.add_argument("--view-position", type=float, nargs=3, default=None)
    parser.add_argument("--view-yaw", type=float, default=None)
    parser.add_argument("--max-ground-travel", type=float, default=None,
                        help="override the inherited SIM ground-travel guard for the initial parking location")
    parser.add_argument("--navigation-timeout", type=float, default=None,
                        help="override the inherited SIM navigation runtime guard in seconds")
    parser.add_argument("--wait-for-status-subscriber", action="store_true",
                        help="wait up to ten seconds for a status transport subscriber before starting")
    parser.add_argument("--settle-position-tolerance", type=float, default=.10)
    parser.add_argument("--settle-yaw-tolerance", type=float, default=.10)
    parser.add_argument("--settle-speed", type=float, default=.10)
    parser.add_argument("--settle-duration", type=float, default=.5)
    parser.add_argument("--settle-timeout", type=float, default=45.)
    parser.add_argument("--cloud-timeout", type=float, default=20.)
    parser.add_argument("--cloud-window-s", type=float, default=5.,
                        help="stable hover duration of fresh Livox chunks per observation")
    parser.add_argument("--core-timeout", type=float, default=900.)
    parser.add_argument("--tf-max-age", type=float, default=.5)
    parser.add_argument("--tf-timeout", type=float, default=1.)
    parser.add_argument("--cloud-topic", default="/uav1/livox/lidar")
    parser.add_argument("--uav-base-frame", default="uav1/base_link")
    parser.add_argument("--check-imports", action="store_true",
                        help="load ROS and the inherited demo without initializing a node or moving robots")
    parser.add_argument("--full-robot-manipulation", action="store_true",
                        help="Use shared perceived-target/payload planning and target-sized gripper preshape")
    parser.add_argument("--execution-clearance", action="store_true",
                        help="Shared development chassis-clearance and whole-manipulation screening")
    return parser


def validate_options(parser, options):
    if options.execution_clearance and not options.full_robot_manipulation:
        parser.error('--execution-clearance requires --full-robot-manipulation')
    if options.max_viewpoints < 1:
        parser.error("--max-viewpoints must be positive")
    if options.max_ground_travel is not None and (
            not math.isfinite(options.max_ground_travel) or options.max_ground_travel <= 0):
        parser.error("--max-ground-travel must be finite and positive")
    if options.navigation_timeout is not None and (
            not math.isfinite(options.navigation_timeout) or options.navigation_timeout <= 0):
        parser.error("--navigation-timeout must be finite and positive")
    for name in ("settle_position_tolerance", "settle_yaw_tolerance", "settle_speed",
                 "settle_duration", "settle_timeout", "cloud_timeout", "cloud_window_s", "core_timeout",
                 "tf_max_age", "tf_timeout", "facade_position_tolerance"):
        if not math.isfinite(getattr(options, name)) or getattr(options, name) <= 0:
            parser.error("--%s must be finite and positive" % name.replace("_", "-"))
    if options.cloud_window_s >= options.cloud_timeout:
        parser.error("--cloud-window-s must be less than --cloud-timeout")
    if (not all(math.isfinite(value) for value in options.flight_bounds)
            or any(options.flight_bounds[index] >= options.flight_bounds[index + 1]
                   for index in (0, 2, 4))):
        parser.error("--flight-bounds must contain finite increasing min/max pairs")
    for values in (options.xy_offsets_m, options.view_position or [],
                   [] if options.view_yaw is None else [options.view_yaw]):
        if not all(math.isfinite(value) for value in values):
            parser.error("view pose and offsets must be finite")
    for name in ("sim_root", "output_dir", "core_python", "rm4d_root", "rm4d_config", "rm4d_map"):
        setattr(options, name, getattr(options, name).expanduser().resolve())
    for path in (options.core_python, options.rm4d_config, options.rm4d_map,
                 options.sim_root / DEMO_RELATIVE / "scripts/run_air_ground_pick_demo.py",
                 options.sim_root / DEMO_RELATIVE / "config/demo.yaml"):
        if not path.is_file():
            parser.error("required file does not exist: %s" % path)
    if not options.rm4d_root.is_dir():
        parser.error("RM4D root does not exist: %s" % options.rm4d_root)
    if options.rm4d_task_asset is not None:
        options.rm4d_task_asset = options.rm4d_task_asset.expanduser().resolve()
        if not options.rm4d_task_asset.is_dir():
            parser.error("RM4D task asset directory does not exist: %s" % options.rm4d_task_asset)


def build_demo_parameters(parameters, options):
    """Keep SIM defaults unless the run explicitly overrides a scene setting."""
    parameters = dict(parameters, placement_mode="rm4d")
    if getattr(options, 'full_robot_manipulation', False):
        parameters['full_robot_manipulation'] = True
    if getattr(options, 'execution_clearance', False):
        parameters['execution_clearance'] = True
    for name in ("view_position", "view_yaw", "max_ground_travel", "navigation_timeout"):
        if getattr(options, name) is not None:
            parameters[name] = getattr(options, name)
    return parameters


def load_demo_module(sim_root):
    path = sim_root / DEMO_RELATIVE / "scripts/run_air_ground_pick_demo.py"
    spec = importlib.util.spec_from_file_location("a5_inherited_air_ground_pick_demo", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def wait_for_status_subscriber(publisher, rospy, error_type, timeout_s=10.):
    """Bound startup until the optional diagnostic status consumer connects."""
    # Match A6's existing allowance for rospy's registration-race reconnect.
    # This happens before task timing, observation windows or physical actions.
    deadline = time.monotonic() + timeout_s
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if publisher.get_num_connections() > 0:
            return
        time.sleep(.01)
    raise error_type("A5 status subscriber did not connect before startup")


def lookup_transform_wall(buffer, target_frame, source_frame, stamp, rospy, transient_errors):
    """Wait at most 0.5 wall seconds for this unchanged TF request to be ready."""
    # MoveIt initialization can hold Python callbacks behind the GIL. A queued
    # /clock jump must not expire the wait before queued TF callbacks can run.
    deadline = time.monotonic() + .5
    last_error = None
    while not rospy.is_shutdown():
        if last_error is not None and time.monotonic() >= deadline:
            raise last_error
        try:
            return buffer.lookup_transform(
                target_frame, source_frame, stamp, rospy.Duration(0))
        except transient_errors as error:
            last_error = error
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise
            time.sleep(min(.01, remaining))
    raise rospy.ROSInterruptException("A5 pose transform interrupted by shutdown")


def build_adapter_class(demo_module, options):
    """Import the ROS boundary lazily so --help and source import stay portable."""
    import numpy as np
    import rospy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2
    from std_msgs.msg import String
    import tf2_ros

    from a5_ros_support import (
        WorkerError, capture_pose_settled, finite_xyz, fresh_scan_stamp, merge_cloud_chunks,
        normalized_frame, pose_settled,
        pose_xyzyaw, rigid_transform, run_worker_request, yaw_quaternion,
    )
    manipulation = sys.modules.get("a5_manipulation")
    if manipulation is None:
        manipulation_spec = importlib.util.spec_from_file_location(
            "a5_manipulation", ROOT / "scripts/a5_manipulation.py")
        manipulation = importlib.util.module_from_spec(manipulation_spec)
        manipulation_spec.loader.exec_module(manipulation)
    execute_refined_pregrasp = manipulation.execute_refined_pregrasp

    DemoError = demo_module.DemoError

    class A5AirGroundPickDemo(demo_module.AirGroundPickDemo):
        def __init__(self):
            super().__init__()
            inherited_status_pub = self._status_pub
            # Acquire the replacement before releasing the inherited handle:
            # rospy shares one topic implementation, so no unregister/register
            # gap can strand a checker already connecting to this same URI.
            self._status_pub = rospy.Publisher(
                self._status_topic, String, queue_size=10, latch=True)
            inherited_status_pub.unregister()
            self._a5_refined_grasp = None
            self._a5_cloud = None
            self._a5_previous_stamp = 0.
            self._a5_observations = []
            self._a5_selected = None
            self._a5_candidate_count = 0
            self._a5_output = options.output_dir
            self._a5_output.mkdir(parents=True, exist_ok=True)
            self._a5_cloud_subscriber = rospy.Subscriber(
                options.cloud_topic, PointCloud2, self._a5_cloud_callback,
                queue_size=1, buff_size=16 * 1024 * 1024)

        def _transform_pose(self, pose, target_frame, use_latest=False):
            if pose.header.frame_id == target_frame:
                return super()._transform_pose(pose, target_frame, use_latest=use_latest)
            transform = lookup_transform_wall(
                self._tf_buffer, target_frame, pose.header.frame_id,
                rospy.Time(0) if use_latest else pose.header.stamp, rospy,
                (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                 tf2_ros.ExtrapolationException))
            result = demo_module.do_transform_pose(pose, transform)
            result.header.frame_id = target_frame
            return result

        def _a5_cloud_callback(self, message):
            with self._lock:
                self._a5_cloud = message

        @staticmethod
        def _a5_matrix(transform):
            t, q = transform.transform.translation, transform.transform.rotation
            return rigid_transform([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

        def _a5_frame_calibration(self):
            """Read the public nominal Ground plane, not a LiDAR-fitted offset."""
            odom_bunker = self._tf_buffer.lookup_transform(
                'ground/odom', self._ground_base_frame, rospy.Time(0),
                rospy.Duration(options.tf_timeout))
            stamp = odom_bunker.header.stamp
            age = rospy.Time.now().to_sec() - stamp.to_sec()
            if not 0 <= age <= options.tf_max_age:
                raise DemoError('A5 Ground map TF is stale')
            map_odom = self._tf_buffer.lookup_transform(
                self._map_frame, 'ground/odom', stamp, rospy.Duration(options.tf_timeout))
            bunker_aubo = self._tf_buffer.lookup_transform(
                self._ground_base_frame, 'ground/aubo_i5_base_link', stamp,
                rospy.Duration(options.tf_timeout))
            return {'T_map_ground_odom': self._a5_matrix(map_odom).tolist(),
                    'T_ground_odom_bunker': self._a5_matrix(odom_bunker).tolist(),
                    'T_bunker_aubo': self._a5_matrix(bunker_aubo).tolist()}

        def _a5_measured_pose(self, stamp=None, timeout_s=None):
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, normalized_frame(options.uav_base_frame),
                rospy.Time(0) if stamp is None else stamp,
                rospy.Duration(options.tf_timeout if timeout_s is None else timeout_s))
            if stamp is None:
                age = rospy.Time.now().to_sec() - transform.header.stamp.to_sec()
                if not 0 <= age <= options.tf_max_age:
                    raise DemoError("A5 UAV map TF is stale")
            return pose_xyzyaw(self._a5_matrix(transform))

        def _a5_settled_now(self, goal, timeout_s=None, acquisition=False):
            pose = self._a5_measured_pose(timeout_s=timeout_s)
            state, _target, received, _target_received = self._air_snapshot()
            settled_goal = pose[:3] + [goal[3]] if acquisition else goal
            bounds = options.flight_bounds
            in_bounds = all(bounds[2 * axis] <= pose[axis] <= bounds[2 * axis + 1]
                            for axis in range(3))
            settled = (state is not None and received is not None
                       and time.monotonic() - received <= self._flight_health_max_age
                       and (not acquisition or in_bounds)
                       and pose_settled(pose, settled_goal, state.velocity,
                                        options.settle_position_tolerance,
                                        options.settle_yaw_tolerance, options.settle_speed))
            return settled, pose

        def _a5_wait_settled(self, goal, reacquire_anchor=False):
            # Flight arrival is relative to its commanded goal. A post-core
            # capture anchor was never commanded: establish it during the
            # stable dwell, rather than requiring return to a transient sample.
            anchor = list(goal)
            deadline, stable_since = time.monotonic() + options.settle_timeout, None
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                pose = None
                try:
                    if reacquire_anchor:
                        settled, pose = self._a5_settled_now(anchor, acquisition=True)
                        settled = settled and pose_settled(
                            pose, anchor, [0., 0., 0.], options.settle_position_tolerance,
                            options.settle_yaw_tolerance, options.settle_speed)
                    else:
                        settled, pose = self._a5_settled_now(goal)
                except (DemoError, tf2_ros.TransformException):
                    settled = False
                now = time.monotonic()
                if settled:
                    stable_since = now if stable_since is None else stable_since
                    if now - stable_since >= options.settle_duration:
                        self._publish_status("A5_SETTLED", uav_pose_map=pose)
                        return pose
                else:
                    stable_since = None
                    if reacquire_anchor and pose is not None:
                        # Preserve yaw, speed, bounds, freshness and dwell gates.
                        # No flight, goal retry or observation is produced here.
                        anchor = list(pose[:3]) + [goal[3]]
                self._wait_step()
            raise DemoError("A5 UAV position/yaw/speed did not settle")

        def _a5_fly_and_hover(self, goal, label, force_flight=False):
            bounds = options.flight_bounds
            if any(not bounds[2 * axis] <= goal[axis] <= bounds[2 * axis + 1]
                   for axis in range(3)):
                raise DemoError("A5 viewpoint is outside configured flight bounds")
            current = self._a5_measured_pose()
            close_position = np.linalg.norm(np.asarray(current[:3]) - goal[:3]) <= options.facade_position_tolerance
            same_pose = pose_settled(current, goal, [0., 0., 0.],
                                     options.facade_position_tolerance,
                                     options.settle_yaw_tolerance, options.settle_speed)
            if close_position and not same_pose:
                raise DemoError("A5 cannot execute a yaw-only goal through the current flight facade")
            self._publish_status("A5_VIEWPOINT", goal_map=list(goal), rescan=bool(same_pose and not force_flight))
            if force_flight or not same_pose:
                target = self._view_pose()
                target.pose.position.x, target.pose.position.y, target.pose.position.z = goal[:3]
                q = yaw_quaternion(goal[3])
                (target.pose.orientation.x, target.pose.orientation.y,
                 target.pose.orientation.z, target.pose.orientation.w) = q
                self._execute_flight(self._fly_to_command, label, target)
            self._a5_wait_settled(goal)
            self._execute_flight(self._hover_command, "A5 hover")

        def _a5_capture(self, requested_goal):
            # Freeze one measured anchor only after a stable post-core dwell.
            goal = list(self._a5_measured_pose())
            bounds = options.flight_bounds
            if any(not bounds[2 * axis] <= goal[axis] <= bounds[2 * axis + 1]
                   for axis in range(3)):
                raise DemoError("A5 measured capture anchor is outside configured flight bounds")
            settled_pose = self._a5_wait_settled(goal, reacquire_anchor=True)
            # Reacquire only XYZ; otherwise yaw tolerance could compound
            # between the stable dwell and the subsequent capture window.
            goal = list(settled_pose[:3]) + [goal[3]]
            capture_metadata = dict(requested_viewpoint=list(requested_goal),
                                    capture_anchor_map=list(goal))
            yaw_delta = goal[3] - requested_goal[3]
            self._publish_status(
                "A5_CAPTURE_ANCHOR", **capture_metadata,
                position_discrepancy_m=float(np.linalg.norm(
                    np.asarray(goal[:3]) - np.asarray(requested_goal[:3]))),
                yaw_discrepancy_rad=math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
            # One stable dwell window is one observation, regardless of packet count.
            capture_start = rospy.Time.now().to_sec()
            deadline = time.monotonic() + options.cloud_timeout
            chunks, pending_cloud = [], None
            previous_stamp = self._a5_previous_stamp
            recovering, stable_since = False, None

            def discard_window(reason, current_pose=None, stamped_pose=None):
                nonlocal chunks, pending_cloud, recovering, stable_since
                if not recovering:
                    state, _target, received, _target_received = self._air_snapshot()
                    self._publish_status(
                        "A5_CAPTURE_RETRY", reason=reason, discarded_chunks=len(chunks),
                        uav_pose_map=current_pose, stamped_uav_pose_map=stamped_pose,
                        goal_map=list(goal),
                        velocity_xyz=None if state is None else [float(value) for value in state.velocity],
                        flight_health_age_s=None if received is None else time.monotonic() - received,
                        **capture_metadata)
                chunks, pending_cloud = [], None
                recovering, stable_since = True, None

            while not rospy.is_shutdown() and time.monotonic() < deadline:
                try:
                    settled, current_pose = self._a5_settled_now(
                        goal, timeout_s=0., acquisition=True)
                except (DemoError, tf2_ros.TransformException):
                    discard_window("current_tf_unavailable")
                    self._wait_step()
                    continue
                if not settled:
                    discard_window("current_hover_unsettled", current_pose)
                    self._wait_step()
                    continue
                if recovering:
                    now = time.monotonic()
                    stable_since = now if stable_since is None else stable_since
                    if now - stable_since >= options.settle_duration:
                        # Require new packet stamps after the entire settling
                        # interval; never carry points across an unstable period.
                        capture_start = rospy.Time.now().to_sec()
                        recovering, stable_since = False, None
                    self._wait_step()
                    continue
                if pending_cloud is None:
                    with self._lock:
                        pending_cloud = self._a5_cloud
                cloud = pending_cloud
                if (cloud is None or not fresh_scan_stamp(
                        cloud.header.stamp.to_sec(), previous_stamp, capture_start)):
                    pending_cloud = None
                    self._wait_step()
                    continue
                frame = normalized_frame(cloud.header.frame_id)
                if chunks and frame != chunks[0]["frame_id"]:
                    raise DemoError("A5 sensor frame changed during the hover cloud window")
                try:
                    # Retry this packet until its exact stamped TF arrives. Zero
                    # TF timeouts keep the wall deadline independent of /clock.
                    transform = self._tf_buffer.lookup_transform(
                        self._map_frame, frame, cloud.header.stamp, rospy.Duration(0.))
                    sensor_matrix = self._a5_matrix(transform)
                    pose = self._a5_measured_pose(cloud.header.stamp, timeout_s=0.)
                except tf2_ros.TransformException:
                    self._wait_step()
                    continue
                try:
                    settled, current_pose = self._a5_settled_now(
                        goal, timeout_s=0., acquisition=True)
                except (DemoError, tf2_ros.TransformException):
                    discard_window("current_tf_unavailable", stamped_pose=pose)
                    self._wait_step()
                    continue
                state = self._air_snapshot()[0]
                stamped_in_bounds = all(
                    bounds[2 * axis] <= pose[axis] <= bounds[2 * axis + 1]
                    for axis in range(3))
                if not settled or state is None or not stamped_in_bounds or not capture_pose_settled(
                        current_pose, pose, goal, state.velocity,
                        options.settle_position_tolerance, options.settle_yaw_tolerance,
                        options.settle_speed):
                    acquisition_goal = current_pose[:3] + [goal[3]]
                    reason = ("stamped_pose_outside_hover" if not stamped_in_bounds or not pose_settled(
                        pose, acquisition_goal, [0., 0., 0.], options.settle_position_tolerance,
                        options.settle_yaw_tolerance, options.settle_speed)
                        else "current_hover_unsettled")
                    discard_window(reason, current_pose, pose)
                    self._wait_step()
                    continue
                points = finite_xyz(point_cloud2.read_points(
                    cloud, field_names=("x", "y", "z"), skip_nans=False))
                stamp = cloud.header.stamp.to_sec()
                chunks.append(dict(points_xyz=points, T_map_sensor=sensor_matrix,
                                   stamp_s=stamp, frame_id=frame))
                previous_stamp, pending_cloud = stamp, None
                if stamp - chunks[0]["stamp_s"] < options.cloud_window_s:
                    self._wait_step()
                    continue
                if time.monotonic() >= deadline:
                    break
                observation = merge_cloud_chunks(chunks)
                path = self._a5_output / ("observation_%02d.npz" % (len(self._a5_observations) + 1))
                if path.exists():
                    raise DemoError("A5 observation already exists; use a fresh output directory: %s" % path)
                np.savez_compressed(str(path), **observation)
                self._a5_previous_stamp = stamp
                self._a5_observations.append(str(path))
                self._publish_status("A5_OBSERVATION", round=len(self._a5_observations),
                                     stamp_s=stamp, sensor_frame=frame,
                                     point_count=len(observation["points_xyz"]), chunk_count=len(chunks),
                                     window_duration_s=stamp - chunks[0]["stamp_s"],
                                     uav_pose_map=pose, observation_file=str(path), **capture_metadata)
                return pose
            raise DemoError("A5 fresh MID360 PointCloud2 window capture timed out")

        def _a5_core(self, request, label):
            self._publish_status("A5_CORE", operation=request["op"], label=label)
            return run_worker_request(options.core_python, ROOT / "scripts/a5_core_worker.py",
                                      ROOT / "src", request, self._a5_output, label,
                                      options.core_timeout)

        def _run_air_phase(self):
            try:
                self._wait_preflight()
                for status in ("ARMING", "COMMAND_CONTROL", "TAKEOFF"):
                    self._publish_status(status)
                self._flight_started = True
                self._execute_flight(self._takeoff_command, "takeoff")
                initial_view = list(self._view_position) + [self._view_yaw]
                self._publish_status("AIR_VIEW")
                self._a5_fly_and_hover(initial_view, "A5 initial fly-to", force_flight=True)
                self._publish_status("AIR_OBSERVE")
                target_map = self._observe_from_air()
                generated = demo_module.generate_top_down_grasp(
                    target_map, self._target_size, self._pregrasp_height, self._lift_height,
                    self._finger_pad_lower_edge_offset, self._contact_overlap, self._surface_clearance)
                initial = self._a5_core({
                    "op": "init", "output_dir": str(self._a5_output),
                    "sim_root": str(options.sim_root), "rm4d_root": str(options.rm4d_root),
                    "rm4d_config": str(options.rm4d_config), "rm4d_map": str(options.rm4d_map),
                    "rm4d_task_asset": str(options.rm4d_task_asset) if options.rm4d_task_asset else None,
                    "grasp": {"grasp_id": self._rm4d_grasp_id, "frame_id": self._map_frame,
                              "position_xyz": list(generated.grasp.position),
                              "quaternion_xyzw": list(generated.grasp.orientation)},
                    "current_bunker_pose": list(self._ground_pose()),
                    "frame_calibration": self._a5_frame_calibration(),
                    "config": {"max_viewpoints": options.max_viewpoints,
                               "support_anchor": getattr(options, 'support_anchor', 'cell_center'),
                               "flight_bounds": options.flight_bounds,
                               "facade_position_tolerance": options.facade_position_tolerance,
                               "xy_offsets_m": options.xy_offsets_m},
                    **getattr(self, '_operational_init', {}),
                }, "init")
                self._a5_candidate_count = initial.get("candidate_count", 0)
                if getattr(self, '_execution_clearance', False):
                    import json
                    original = json.loads(Path(initial['initial_file']).read_text())
                    self._a5_execution_seeds = {row['candidate_id']: row.get('joint_configuration')
                                               for row in original['result']['evaluated_candidates']}
                goal = initial_view
                for round_number in range(1, options.max_viewpoints + 1):
                    pose = self._a5_capture(goal)
                    response = self._a5_core({
                        "op": "observe", "initial_file": initial["initial_file"],
                        "observations": list(self._a5_observations), "uav_pose": pose,
                        "output_dir": str(self._a5_output),
                    }, "round-%02d" % round_number)
                    self._publish_status("A5_DECISION", **{key: response[key] for key in (
                        "round", "stop_reason", "next_viewpoint", "selected_candidate")})
                    if response["stop_reason"] is not None:
                        self._a5_selected = response["selected_candidate"]
                        if getattr(self, '_execution_clearance', False):
                            self._a5_selected = self._screen_ground_candidates(response['assessments'], target_map)
                            self._execution_rm_seed = self._a5_execution_seeds.get(self._a5_selected['candidate_id'])
                        if self._a5_selected is None:
                            suffix = (' passing bounded execution screen' if
                                      getattr(self, '_execution_clearance', False) else '')
                            raise DemoError("A5 stopped (%s) without a confirmed exact candidate%s" %
                                            (response["stop_reason"], suffix))
                        self._a5_candidate_count = response.get("candidate_count", self._a5_candidate_count)
                        self._publish_status("A5_SELECTED", **self._a5_selected)
                        self._a5_fly_and_hover(initial_view, "A5 return to clear landing location")
                        self._publish_status("LANDING")
                        self._request_land()
                        return target_map
                    if round_number == options.max_viewpoints:
                        raise DemoError("A5 core did not stop at the configured observation budget")
                    goal = response["next_viewpoint"]
                    self._a5_fly_and_hover(goal, "A5 next viewpoint")
                raise DemoError("A5 observation loop ended without a selection")
            except (WorkerError, OSError, ValueError) as error:
                raise DemoError("A5 adapter failed: %s" % error) from error

        def _screen_ground_candidates(self, assessments, target_map):
            from a5_execution_selection import select_execution_candidate
            selected = select_execution_candidate(
                assessments, lambda candidate: self._preview_ground_candidate(
                    target_map, candidate, self._a5_execution_seeds.get(candidate['candidate_id'])),
                self._publish_status)
            if selected is None:
                raise DemoError('no confirmed exact candidate passing bounded execution screen')
            return selected

        def _select_rm4d_candidate(self, _target_map):
            if self._a5_selected is None:
                raise DemoError("A5 has no cached exact candidate for the ground stage")
            chosen = self._a5_selected
            return ((chosen["x"], chosen["y"], chosen["yaw"]), chosen["candidate_id"],
                    chosen["relevance"], self._a5_candidate_count)

        def _pick_and_lift(self, sensor_pose, target):
            if getattr(self, "_full_robot_manipulation", False):
                # The grasp-seeded approach and SIM execution must use the
                # same contact-configuration pad geometry after refinement.
                generated = self._generate_ground_grasp(target)
            else:
                generated = demo_module.generate_top_down_grasp(
                    target, self._target_size, self._pregrasp_height,
                    self._lift_height, self._finger_pad_lower_edge_offset,
                    self._contact_overlap, self._surface_clearance)
            group = self._initialize_moveit()
            exact_grasp = self._pose_message(
                generated.grasp, self._map_frame, sensor_pose.header.stamp)
            self._a5_refined_grasp = self._transform_pose(
                exact_grasp, group.get_planning_frame())
            try:
                return super()._pick_and_lift(sensor_pose, target)
            finally:
                self._a5_refined_grasp = None

        def _execute_pregrasp(self, target, continuation=None):
            if getattr(self, '_execution_clearance', False):
                return super()._execute_pregrasp(target, continuation)
            grasp = continuation if continuation is not None else self._a5_refined_grasp
            if grasp is None:
                return super()._execute_pregrasp(target, continuation)
            return execute_refined_pregrasp(
                self, target, grasp, DemoError)

    if getattr(options, 'operational_gating', 'v1') in ('v1.1', 'v1.3', 'v1.4'):
        from a5_target_support import build_object_aware_adapter
        return build_object_aware_adapter(A5AirGroundPickDemo, options)
    return A5AirGroundPickDemo


def main(argv=None):
    argv = sys.argv if argv is None else argv
    parser = build_parser()
    if "--help" in argv or "-h" in argv:
        parser.parse_args(["--help"])
    import rospy
    import moveit_commander
    import yaml

    options = parser.parse_args(rospy.myargv(argv=argv)[1:])
    validate_options(parser, options)
    demo_module = load_demo_module(options.sim_root)
    adapter_class = build_adapter_class(demo_module, options)
    if options.check_imports:
        return 0
    moveit_commander.roscpp_initialize(argv)
    rospy.init_node("a5_sim_active_perception")
    try:
        with (options.sim_root / DEMO_RELATIVE / "config/demo.yaml").open() as stream:
            parameters = build_demo_parameters(yaml.safe_load(stream), options)
        if parameters["map_frame"] != "map":
            raise demo_module.DemoError("A5 core requires the map frame")
        for key, value in parameters.items():
            rospy.set_param("~" + key, value)
        adapter = adapter_class()
        if options.wait_for_status_subscriber:
            rospy.loginfo("A5 adapter ready; waiting for status subscriber")
            wait_for_status_subscriber(
                adapter._status_pub, rospy, demo_module.DemoError)
        return 0 if adapter.run() else 1
    except (demo_module.DemoError, OSError, ValueError) as error:
        rospy.logfatal("A5 configuration failed: %s", error)
        return 2
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
