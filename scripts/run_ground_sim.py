#!/usr/bin/python3
"""One Ground-only diagnostic conditioned on an archived runtime handoff."""
import argparse
import json
import math
from pathlib import Path
import sys
import time


GROUND_KINDS = ('GROUND_NAVIGATION_DIAGNOSTIC', 'GROUND_SEGMENT_DIAGNOSTIC',
                'GROUND_CAMERA_MANIPULATION_DIAGNOSTIC')
GROUND_STAGES = ('ground_navigation', 'ground_refine', 'refined_pregrasp',
                 'descend', 'close', 'lift', 'retention')


def summarize_ground_metrics(events, samples, method, *, navigation_only=None,
                             conditioned_on_arrival=False, outcome=None):
    """Reduce real replay boundaries; never synthesize flight/discovery events."""
    from a6_metrics import _duration, _reset, _stage_summary, path_metrics

    events, samples = list(events), list(samples)
    start_index = next((i for i, e in enumerate(events) if e['state'] == 'GROUND_REPLAY_START'), None)
    replay = [] if start_index is None else events[start_index:]
    start = replay[0] if replay else None
    ended = next((e for e in replay if e['state'] == 'GROUND_REPLAY_END'), None)
    terminal = next((e for e in replay if e['state'] in ('FAILED', 'LIFT')), ended)
    primary = replay[:replay.index(terminal) + 1] if terminal is not None else replay
    stamp = lambda e: None if e is None else e['ros_time']
    primary_samples = []
    for row in samples if start is not None else ():
        if row.get('wall_monotonic', math.inf) < start.get('wall_monotonic', -math.inf):
            continue
        # Retain the immediate terminal sample, but never the post-failure
        # diagnostic/cleanup samples, even when simulation time later resets.
        if terminal is not None and row.get('wall_monotonic', -math.inf) > terminal.get('wall_monotonic', math.inf):
            if row['ros_time'] == terminal['ros_time']:
                primary_samples.append(row)
            break
        primary_samples.append(row)
    reset = _reset(primary) or _reset(primary_samples)
    stages, failed_stage, failed_reason = _stage_summary(primary, terminal, reset)
    ready = next((e for e in primary if e['state'] == 'A6_EXEC_READY'), None)
    stopped = next((e for e in primary if e['state'] == 'GROUND_STOPPED'), None)
    arrival = next((e for e in primary if e['state'] == 'GROUND_ARRIVAL_MEASURED'), None)
    arrival_xy_error = arrival_yaw_error = None
    if arrival is not None:
        actual, goal = arrival['actual_pose_map'], arrival['goal_pose_map']
        arrival_xy_error = math.hypot(actual[0] - goal[0], actual[1] - goal[1])
        arrival_yaw_error = abs(math.atan2(math.sin(actual[2] - goal[2]), math.cos(actual[2] - goal[2])))
    if navigation_only is None:
        navigation_only = bool((start or {}).get('navigation_only', False))
    outcome = outcome or {}
    retrieval = outcome.get('retrieval_success')
    if navigation_only or start is None or outcome.get('status') != 'VALID_TRIAL' or type(retrieval) is not bool:
        retrieval = None
    assessed = start is not None and terminal is not None
    return dict(
        schema_version=1, kind='GROUND_REPLAY_METRICS', descriptive_only=True,
        method=method, comparison_role='conditioned_ground_diagnostic', efficiency_clock='simulation',
        conditioned_on_archived_confirmation=start is not None,
        conditioned_on_arrival=bool(conditioned_on_arrival), navigation_only=bool(navigation_only),
        task_started=start is not None, clock_reset_detected=reset,
        task_start_sim=stamp(start), task_end_sim=stamp(terminal), replay_end_sim=stamp(ended),
        exec_ready_sim=stamp(ready), T_task_sim=_duration(stamp(start), stamp(terminal), reset),
        T_exec_ready_sim=_duration(stamp(start), stamp(ready), reset),
        D_env=None, D_exec=None if navigation_only else True if ready else False if assessed else None,
        navigation_success=None if conditioned_on_arrival else
                           True if stopped and arrival else False if assessed and not stopped else None,
        ground_stopped_recorded=stopped is not None,
        arrival_assessment='NOT_ASSESSED_CONDITIONED_ON_ARRIVAL' if conditioned_on_arrival else
                           'RECORDED_FINAL_POSE_AND_STOP' if arrival and stopped else
                           'RECORDED_FINAL_POSE_WITHOUT_STOP' if arrival else 'UNAVAILABLE_FINAL_POSE_EVENT',
        arrival_xy_error_m=arrival_xy_error, arrival_yaw_error_rad=arrival_yaw_error,
        retrieval_success=retrieval,
        retrieval_assessment='NOT_ASSESSED_NAVIGATION_ONLY' if navigation_only else
                             'RECORDED_ATTEMPT' if retrieval is not None else 'UNAVAILABLE',
        physical_success=None, adapter_run_result=None if ended is None else ended.get('success'),
        terminal_status=None if terminal is None else terminal['state'],
        terminal_failure_stage=failed_stage, terminal_failure_reason=failed_reason,
        stages={key: stages[key] for key in GROUND_STAGES},
        paths=dict(ground_total=path_metrics(primary_samples, 'ground', stamp(start), stamp(terminal),
                                             dimensions=2, clock_reset=reset)),
        note='Conditioned on the archived confirmed candidate; arrival conditioning is explicit. '
             'No aerial discovery, windows, flight metrics or E2E comparison are inferred. '
             'LIFT and diagnostic IK do not establish physical retrieval; original outcomes remain authoritative.')


def materialize_ground_metrics(attempt_dir):
    """Create one derived report from completed evidence without rewriting it."""
    root = Path(attempt_dir)
    attempt = json.loads((root / 'attempt.json').read_text())
    if attempt.get('kind') not in GROUND_KINDS or attempt.get('finish_wall') is None:
        raise ValueError('metrics require a completed Ground diagnostic attempt')
    missing = []
    def rows(name):
        path = root / 'data' / name
        if not path.exists():
            missing.append(name)
            return []
        return [json.loads(line) for line in path.read_text().splitlines()]
    report = summarize_ground_metrics(
        rows('events.jsonl'), rows('trajectory.jsonl'), attempt['method'],
        navigation_only=attempt['kind'] == 'GROUND_NAVIGATION_DIAGNOSTIC',
        conditioned_on_arrival=attempt.get('conditioned_on_arrival', False), outcome=attempt)
    report.update(source_attempt=str(root.resolve()), source='saved_ground_replay_events_and_online_public_tf',
                  missing_files=missing, outcome_source='attempt.json', original_outcome_unchanged=True)
    destination = root / 'data/metrics.json'
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open('x') as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return destination


def extract_recorded_handoff(events):
    selected = [e for e in events if e.get('state') == 'A5_SELECTED']
    handoffs = [e for e in events if e.get('state') == 'AIR_HANDOFF']
    if len(selected) != 1 or len(handoffs) != 1:
        raise ValueError('replay requires one exact selection and one runtime aerial handoff')
    selected = dict(selected[0])
    target = tuple(handoffs[0]['target_map'])
    operational = selected.get('operational', {})
    if (selected.get('confirmed') is not True or operational.get('blocked') is not False
            or operational.get('ground_supported') is not True):
        raise ValueError('archived candidate must be operationally confirmed')
    if (len(target) != 4 or not selected.get('candidate_id') or
            not all(math.isfinite(v) for v in (*target, *[selected[k] for k in
                                                        ('x', 'y', 'yaw', 'relevance')]))):
        raise ValueError('nonfinite/incomplete archived runtime handoff')
    for key in ('state', 'ros_time', 'wall_monotonic'):
        selected.pop(key, None)
    return selected, target


def diagnose_grasp_failure(owner, rospy, request_type, compute_ik, validity):
    """One post-failure kinematic probe; never plan/execute its solution."""
    request = request_type()
    request.ik_request.group_name = owner._move_group_name
    request.ik_request.ik_link_name = owner._end_effector_link
    request.ik_request.pose_stamped = owner._ground_last_grasp
    request.ik_request.robot_state = owner._robot_state_from_joint_feedback()
    request.ik_request.avoid_collisions = False  # diagnostic ONLY, after the task has failed
    request.ik_request.timeout = rospy.Duration(5.)
    result = compute_ik(request)
    details = dict(ik_error_code=result.error_code.val, executed=False,
                   original_failure_retained=True, collision_free=None, contacts=[])
    if result.error_code.val == 1:
        checked = validity(robot_state=result.solution, group_name=owner._move_group_name)
        details.update(collision_free=checked.valid,
                       contacts=sorted(set((c.contact_body_1, c.contact_body_2) for c in checked.contacts)))
    owner._publish_status('GROUND_POST_FAILURE_IK_DIAGNOSTIC', **details)


def lift_started(events):
    return any(row.get('state') == 'A6_STAGE_START' and
               row.get('stage') in ('lift', 'retention') for row in events)


def post_failure_load_contrast(owner, rospy):
    """Two fixed holds around real opening; never reclassify the failed task."""
    def hold(phase):
        started = rospy.Time.now().to_sec()
        wall_deadline = time.monotonic() + 10.
        owner._publish_status('GROUND_LOAD_DIAGNOSTIC_BEGIN', phase=phase,
                              grasp_confirmed=owner._grasp_confirmation_current(),
                              original_failure_retained=True)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - started < 2.:
            if time.monotonic() >= wall_deadline:
                raise RuntimeError('post-failure diagnostic clock stalled')
            owner._wait_step()
        if rospy.is_shutdown():
            raise RuntimeError('post-failure diagnostic interrupted')
        owner._publish_status('GROUND_LOAD_DIAGNOSTIC_END', phase=phase,
                              grasp_confirmed=owner._grasp_confirmation_current(),
                              original_failure_retained=True)
    hold('closed_hold')
    owner._open_gripper()
    hold('open_hold')


def main(argv=None):
    argv = sys.argv if argv is None else argv
    if '--materialize-metrics' in argv:
        parser = argparse.ArgumentParser(description='Offline Ground metrics; no ROS imports or runtime actions.')
        parser.add_argument('--materialize-metrics', type=Path, action='append', required=True)
        for directory in parser.parse_args(argv[1:]).materialize_metrics:
            print(materialize_ground_metrics(directory))
        return 0
    import run_a6_sim as a6
    import rospy
    import moveit_commander
    import yaml
    parser = a6.build_parser()
    parser.add_argument('--ground-replay-from', type=Path, required=True)
    parser.add_argument('--ground-navigation-only', action='store_true')
    parser.add_argument('--diagnose-grasp-failure', action='store_true')
    parser.add_argument('--post-failure-load-contrast', action='store_true')
    options = parser.parse_args(rospy.myargv(argv=argv)[1:])
    a6.validate_options(parser, options)
    events = [json.loads(line) for line in
              (options.ground_replay_from/'data/events.jsonl').read_text().splitlines()]
    selected, target = extract_recorded_handoff(events)
    demo = a6.a5.load_demo_module(options.sim_root)
    adapter_class = a6.build_adapter_class(demo, options)
    moveit_commander.roscpp_initialize(argv)
    rospy.init_node('ground_segment_diagnostic')
    adapter = None
    success = False
    replay_started = False
    try:
        parameters = a6.a5.build_demo_parameters(yaml.safe_load(
            (options.sim_root/a6.a5.DEMO_RELATIVE/'config/demo.yaml').read_text()), options)
        for key, value in parameters.items(): rospy.set_param('~'+key, value)
        adapter = adapter_class()
        adapter._a5_selected = selected
        adapter._a5_candidate_count = 1  # this diagnostic executes exactly the archived choice
        print('A6_ADAPTER_READY', flush=True)
        if options.wait_for_status_subscriber:
            a6.wait_for_checker(adapter._status_pub, rospy, demo.DemoError)
        adapter._a6_event('GROUND_REPLAY_START', source=str(options.ground_replay_from),
                          conditioned_on_archived_confirmation=True, target_map=target,
                          navigation_only=options.ground_navigation_only)
        replay_started = True
        adapter._a6_event('A5_SELECTED', **selected)
        adapter._a6_timer = rospy.Timer(rospy.Duration(.1), adapter._a6_sample_trajectory)
        adapter._approach_ground(target)
        if not options.ground_navigation_only:
            sensor_pose, refined = adapter._observe_ground_target(target)
            adapter._pick_and_lift(sensor_pose, refined)
        success = True
        return 0
    except Exception as error:
        if adapter is not None:
            adapter._publish_status('FAILED' if replay_started else 'GROUND_REPLAY_STARTUP_FAILED',
                                    reason=str(error))
        rospy.logerr('Ground diagnostic failed: %s', error)
        if (options.post_failure_load_contrast and adapter is not None and
                lift_started(adapter._a6_events)):
            try:
                post_failure_load_contrast(adapter, rospy)
            except Exception as diagnostic_error:
                adapter._publish_status('GROUND_LOAD_DIAGNOSTIC_FAILED',
                                        reason=str(diagnostic_error), original_failure_retained=True)
        if options.diagnose_grasp_failure and adapter is not None and hasattr(adapter, '_ground_last_grasp'):
            from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetStateValidity
            try:
                diagnose_grasp_failure(adapter, rospy, GetPositionIKRequest,
                    rospy.ServiceProxy('/compute_ik', GetPositionIK),
                    rospy.ServiceProxy('/check_state_validity', GetStateValidity))
            except Exception as diagnostic_error:
                adapter._publish_status('GROUND_POST_FAILURE_IK_DIAGNOSTIC',
                                        executed=False, error=str(diagnostic_error))
        return 1
    finally:
        if adapter is not None:
            adapter._stop_ground(required=False)
            adapter._a6_event('GROUND_REPLAY_END', success=success)
            adapter._a6_close_evidence()
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__': raise SystemExit(main())
