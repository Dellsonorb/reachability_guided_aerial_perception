#!/usr/bin/python3
"""Run one approved A6 method over the frozen A5 ROS adapter."""

import json
import math
from pathlib import Path
import sys
import threading
import time

import run_a5_sim as a5
from a5_ros_support import WorkerError, normalized_frame, run_worker_request
from a6_metrics import TF_MAX_AGE_S, summarize_metrics


ROOT = Path(__file__).resolve().parents[1]
METHODS = ('rm4d_only', 'fixed', 'generic', 'ours', 'no_occlusion', 'no_cost', 'confirmation', 'deficit')


def build_parser():
    parser = a5.build_parser()
    parser.description = __doc__
    parser.add_argument('--method', choices=METHODS, required=True)
    shared = json.loads((ROOT / 'configs/a6_pilot.json').read_text())['shared_a5_settings']
    destinations = {action.dest for action in parser._actions}
    parser.set_defaults(**{key: value for key, value in shared.items() if key in destinations})
    return parser


def validate_options(parser, options):
    if options.max_viewpoints != 3:
        parser.error('A6 --max-viewpoints must be 3, including the initial window')
    a5.validate_options(parser, options)


def build_adapter_class(demo_module, options):
    """Keep ROS imports lazy; five MID methods execute A5's original air loop."""
    import rospy
    import tf2_ros

    parent = a5.build_adapter_class(demo_module, options)
    DemoError = demo_module.DemoError

    class A6AirGroundPickDemo(parent):
        def __init__(self):
            options.output_dir.mkdir(parents=True, exist_ok=True)
            self._a6_lock = threading.RLock()
            self._a6_events, self._a6_samples = [], []
            self._a6_timer = None
            self._a6_view_role = None
            self._a6_events_file = (options.output_dir / 'events.jsonl').open('x', encoding='utf-8')
            self._a6_trajectory_file = (options.output_dir / 'trajectory.jsonl').open('x', encoding='utf-8')
            try:
                super().__init__()
            except Exception:
                self._a6_close_evidence()
                raise

        def _a6_event(self, state, **details):
            with self._a6_lock:
                row = dict(details, state=state, ros_time=rospy.Time.now().to_sec(),
                           wall_monotonic=time.monotonic())
                self._a6_events_file.write(json.dumps(row, sort_keys=True, allow_nan=False) + '\n')
                self._a6_events_file.flush()
                self._a6_events.append(row)
            return row

        def _publish_status(self, state, **details):
            recorded = dict(details)
            if state == 'A5_VIEWPOINT':
                recorded['view_role'] = self._a6_view_role
            self._a6_event(state, **recorded)
            if state in ('LIFT', 'FAILED'):
                self._a6_sample_trajectory(None)
            return super()._publish_status(state, **details)

        def _a6_sample_trajectory(self, _timer_event):
            with self._a6_lock:
                if self._a6_trajectory_file.closed:
                    return
                now = rospy.Time.now().to_sec()
                row = dict(ros_time=now, wall_monotonic=time.monotonic())
                for body, frame in (('uav', normalized_frame(options.uav_base_frame)),
                                    ('ground', normalized_frame(self._ground_base_frame))):
                    try:
                        transform = self._tf_buffer.lookup_transform(
                            self._map_frame, frame, rospy.Time(0), rospy.Duration(0.))
                        stamp = transform.header.stamp.to_sec()
                        age = now - stamp
                        if not 0 <= age <= TF_MAX_AGE_S:
                            raise ValueError('stale public map TF: age=%s' % age)
                        position = transform.transform.translation
                        rotation = transform.transform.rotation
                        values = [position.x, position.y, position.z, rotation.x,
                                  rotation.y, rotation.z, rotation.w]
                        if not all(math.isfinite(value) for value in values):
                            raise ValueError('nonfinite public map TF')
                        # A5's conversion preserves the raw map translation and
                        # normalizes the measured quaternion solely for yaw.
                        from a5_ros_support import pose_xyzyaw
                        pose = pose_xyzyaw(self._a5_matrix(transform))
                        row[body] = dict(frame_id=frame, xyz=pose[:3], yaw=pose[3],
                                         stamp_s=stamp, age_s=age)
                    except (tf2_ros.TransformException, DemoError, ValueError) as error:
                        row[body] = dict(frame_id=frame, xyz=None, yaw=None, missing_reason=str(error))
                self._a6_trajectory_file.write(json.dumps(row, sort_keys=True, allow_nan=False) + '\n')
                self._a6_trajectory_file.flush()
                self._a6_samples.append(row)

        def _a5_capture(self, requested_goal):
            self._a6_event('A6_CAPTURE_START', round=len(self._a5_observations) + 1,
                           requested_goal=list(requested_goal))
            self._a6_sample_trajectory(None)
            return super()._a5_capture(requested_goal)

        def _a5_core(self, request, label):
            request = dict(request, method=options.method)
            directory = self._a5_output
            if request['op'] == 'observe':
                directory = self._a5_output / 'rounds' / ('round-%02d' % len(request['observations']))
                request['output_dir'] = str(directory)
            self._publish_status('A5_CORE', operation=request['op'], label=label)
            response = run_worker_request(options.core_python, ROOT / 'scripts/a6_core_worker.py',
                                          ROOT / 'src', request, directory, label, options.core_timeout)
            if request['op'] == 'observe':
                self._a6_event('A6_ENV_RESULT', round=response['round'],
                               confirmed_candidate_count=response['confirmed_candidate_count'],
                               candidate_count=response['candidate_count'], output_dir=str(directory),
                               computation_timing=response.get('computation_timing'))
            return response

        def _a5_prepare_handoff(self, response, target_map):
            response = super()._a5_prepare_handoff(response, target_map)
            if response['stop_reason'] is not None:
                self._a6_event('A6_ACTIVE_STOP', round=response['round'], stop_reason=response['stop_reason'])
                self._a6_sample_trajectory(None)
            return response

        def _a5_fly_and_hover(self, goal, label, force_flight=False):
            previous = self._a6_view_role
            self._a6_view_role = ('sensing' if label == 'A5 next viewpoint' else
                                  'return' if label == 'A5 return to clear landing location' else 'initial')
            try:
                if options.method == 'fixed' and label == 'A5 next viewpoint':
                    # A core call may outlast a slow hover drift. A fixed visit
                    # must use the current measured pose, including current yaw.
                    current = list(self._a5_measured_pose())
                    self._publish_status('A5_VIEWPOINT', goal_map=current, rescan=True)
                    self._a5_wait_settled(current)
                    self._execute_flight(self._hover_command, 'A5 hover')
                    return
                if self._a6_view_role == 'return':
                    return self._a6_stage_call('return', super()._a5_fly_and_hover, goal, label, force_flight)
                return super()._a5_fly_and_hover(goal, label, force_flight)
            finally:
                self._a6_view_role = previous

        def _run_air_phase(self):
            if options.method != 'rm4d_only':
                return super()._run_air_phase()
            try:
                self._wait_preflight()
                for status in ('ARMING', 'COMMAND_CONTROL', 'TAKEOFF'):
                    self._publish_status(status)
                self._flight_started = True
                self._execute_flight(self._takeoff_command, 'takeoff')
                initial_view = list(self._view_position) + [self._view_yaw]
                self._publish_status('AIR_VIEW')
                self._a5_fly_and_hover(initial_view, 'A5 initial fly-to', force_flight=True)
                self._publish_status('AIR_OBSERVE')
                target_map = self._observe_from_air()
                generated = demo_module.generate_top_down_grasp(
                    target_map, self._target_size, self._pregrasp_height, self._lift_height,
                    self._finger_pad_lower_edge_offset, self._contact_overlap, self._surface_clearance)
                initial = self._a5_core({
                    'op': 'init', 'output_dir': str(self._a5_output),
                    'sim_root': str(options.sim_root), 'rm4d_root': str(options.rm4d_root),
                    'rm4d_config': str(options.rm4d_config), 'rm4d_map': str(options.rm4d_map),
                    'rm4d_task_asset': str(options.rm4d_task_asset) if options.rm4d_task_asset else None,
                    'grasp': {'grasp_id': self._rm4d_grasp_id, 'frame_id': self._map_frame,
                              'position_xyz': list(generated.grasp.position),
                              'quaternion_xyzw': list(generated.grasp.orientation)},
                    'current_bunker_pose': list(self._ground_pose()),
                    'frame_calibration': self._a5_frame_calibration(),
                    'config': {'max_viewpoints': options.max_viewpoints,
                               'flight_bounds': options.flight_bounds,
                               'facade_position_tolerance': options.facade_position_tolerance,
                               'xy_offsets_m': options.xy_offsets_m},
                }, 'init')
                candidates = json.loads(Path(initial['initial_file']).read_text())['result']['candidates']
                if not candidates:
                    raise DemoError('A6 RM4D-only query returned no candidates')
                first = candidates[0]
                pose = [first[key] for key in ('bunker_x', 'bunker_y', 'bunker_yaw')]
                score = first['final_score']
                if not first.get('candidate_id') or not all(math.isfinite(value) for value in pose + [score]):
                    raise DemoError('A6 original RM4D top candidate is invalid')
                self._a5_selected = dict(first, x=pose[0], y=pose[1], yaw=pose[2])
                self._a5_candidate_count = len(candidates)
                self._a6_event('A6_RM4D_SELECTED', selected_candidate=self._a5_selected,
                               original_final_score=score, candidate_count=len(candidates))
                self._a5_fly_and_hover(initial_view, 'A5 return to clear landing location')
                self._publish_status('LANDING')
                self._request_land()
                return target_map
            except (WorkerError, OSError, ValueError, KeyError, TypeError) as error:
                raise DemoError('A6 RM4D-only adapter failed: %s' % error) from error

        def _screen_ground_candidates(self, assessments, target_map, required=True):
            return self._a6_stage_call('execution_screen', super()._screen_ground_candidates,
                                      assessments, target_map, required)

        def _select_rm4d_candidate(self, target_map):
            if options.method != 'rm4d_only':
                return super()._select_rm4d_candidate(target_map)
            chosen = self._a5_selected
            if chosen is None:
                raise DemoError('A6 has no original RM4D candidate for Ground')
            return ((chosen['x'], chosen['y'], chosen['yaw']), chosen['candidate_id'],
                    chosen['final_score'], self._a5_candidate_count)

        def _request_land(self):
            result = self._a6_stage_call('landing', super()._request_land)
            self._a6_event('A6_LANDED')
            self._a6_sample_trajectory(None)
            return result

        def _a6_record_terminal_exception(self, error, source):
            if not any(row['state'] in ('LIFT', 'FAILED') for row in self._a6_events):
                self._a6_event('FAILED', reason=str(error), source=source)
                self._a6_sample_trajectory(None)

        def _stop_ground(self, required=False):
            # Frozen run() calls this first in finally. An exception outside its
            # handled types is still active here, before either cleanup action.
            # Recording only in our outer run() catch would include recovery.
            pending = sys.exc_info()[1]
            if not required and pending is not None:
                self._a6_record_terminal_exception(pending, 'inherited_cleanup_entry')
            return super()._stop_ground(required=required)

        def _approach_ground(self, target_map):
            # The inherited Ground pose lookup precedes GROUND_APPROACH.
            return self._a6_stage_call('ground_navigation', super()._approach_ground, target_map)

        def _observe_ground_target(self, target_map):
            return self._a6_stage_call('ground_refine', super()._observe_ground_target, target_map)

        def _pick_and_lift(self, sensor_pose, target):
            # A5 generates/transforms the refined grasp before PREGRASP and
            # before _a5_refined_grasp is set. Those preparations belong here.
            self._a6_event('A6_STAGE_START', stage='refined_pregrasp')
            return super()._pick_and_lift(sensor_pose, target)

        def _a6_stage_call(self, stage, operation, *args):
            self._a6_event('A6_STAGE_START', stage=stage)
            try:
                result = operation(*args)
            except Exception as error:
                self._a6_event('A6_STAGE_FAILED', stage=stage, reason=str(error))
                raise
            self._a6_event('A6_STAGE_END', stage=stage)
            return result

        def _execute_pregrasp(self, target, continuation=None):
            if self._a5_refined_grasp is None:
                return super()._execute_pregrasp(target, continuation)
            result = self._a6_stage_call('refined_pregrasp', super()._execute_pregrasp, target, continuation)
            self._a6_event('A6_EXEC_READY')
            return result

        def _execute_cartesian(self, target, label):
            stage = {'grasp approach': 'descend', 'lift': 'lift'}.get(label)
            if stage is None:
                return super()._execute_cartesian(target, label)
            return self._a6_stage_call(stage, super()._execute_cartesian, target, label)

        def _close_gripper(self):
            return self._a6_stage_call('close', super()._close_gripper)

        def _hold_grasp_confirmation(self):
            return self._a6_stage_call('retention', super()._hold_grasp_confirmation)

        def _a6_close_evidence(self):
            if self._a6_timer is not None:
                self._a6_timer.shutdown()
            with self._a6_lock:
                self._a6_events_file.close()
                self._a6_trajectory_file.close()

        def run(self):
            result = None
            try:
                self._a6_event('A6_TASK_START', method=options.method)
                self._a6_sample_trajectory(None)
                self._a6_timer = rospy.Timer(rospy.Duration(.1), self._a6_sample_trajectory)
                result = super().run()
                return result
            except Exception as error:
                self._a6_record_terminal_exception(error, 'adapter_exception')
                self._a6_event('A6_UNHANDLED_ERROR', reason=str(error))
                raise
            finally:
                self._a6_event('A6_TASK_END', adapter_run_result=result)
                self._a6_sample_trajectory(None)
                if self._a6_timer is not None:
                    self._a6_timer.shutdown()
                try:
                    with self._a6_lock:
                        summary = summarize_metrics(self._a6_events, self._a6_samples,
                                                    method=options.method, run_result=result)
                    (self._a5_output / 'metrics.json').write_text(
                        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + '\n', encoding='utf-8')
                finally:
                    self._a6_close_evidence()

    return A6AirGroundPickDemo


def wait_for_checker(publisher, rospy, error_type):
    # rospy may retry a TCPROS connection after publisher registration races.
    # This is common diagnostic startup, before task time or physical actions.
    a5.wait_for_status_subscriber(publisher, rospy, error_type, timeout_s=10.)


def main(argv=None):
    argv = sys.argv if argv is None else argv
    parser = build_parser()
    if '--help' in argv or '-h' in argv:
        parser.parse_args(['--help'])
    import rospy
    import moveit_commander
    import yaml

    options = parser.parse_args(rospy.myargv(argv=argv)[1:])
    validate_options(parser, options)
    demo_module = a5.load_demo_module(options.sim_root)
    adapter_class = build_adapter_class(demo_module, options)
    if options.check_imports:
        return 0
    moveit_commander.roscpp_initialize(argv)
    rospy.init_node('a6_sim_pilot')
    try:
        with (options.sim_root / a5.DEMO_RELATIVE / 'config/demo.yaml').open() as stream:
            parameters = a5.build_demo_parameters(yaml.safe_load(stream), options)
        if parameters['map_frame'] != 'map':
            raise demo_module.DemoError('A6 requires the public map frame')
        for key, value in parameters.items():
            rospy.set_param('~' + key, value)
        adapter = adapter_class()
        # The inherited A5 constructor replaces its temporary status publisher.
        # Let orchestration subscribe the checker only to the final publisher.
        print('A6_ADAPTER_READY', flush=True)
        if options.wait_for_status_subscriber:
            wait_for_checker(adapter._status_pub, rospy, demo_module.DemoError)
        return 0 if adapter.run() else 1
    except (demo_module.DemoError, OSError, ValueError) as error:
        rospy.logfatal('A6 configuration failed: %s', error)
        return 2
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
