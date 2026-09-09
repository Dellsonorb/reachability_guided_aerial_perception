#!/usr/bin/env python3
"""Match recorded public feedback to simultaneous raw-CSV pose increments."""
import argparse
import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np

_spec = importlib.util.spec_from_file_location('pose_validation', Path(__file__).with_name('validate_pose_velocity.py'))
validation = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(validation)
analysis = validation.analysis
PUBLIC_JOINTS = analysis.JOINTS + ('left_outer_knuckle_joint',)


def motion_windows(events):
    active, windows = {}, []
    for event in events:
        stage, state = event.get('stage'), event.get('state')
        if stage not in ('ground_refine', 'refined_pregrasp', 'descend', 'close'):
            continue
        if state == 'A6_STAGE_START':
            active.setdefault(stage, float(event['ros_time']))
        elif state in ('A6_STAGE_END', 'A6_STAGE_FAILED') and stage in active:
            name = stage + '_before_close' if stage in ('ground_refine', 'refined_pregrasp') else stage
            windows.append(dict(name=name, start_s=active.pop(stage), end_s=float(event['ros_time']),
                                boundary='whole stage events; first duplicate START retained'))
    return windows


def reference_rates(data, joint):
    dt = np.diff(data['sim_time_s'])
    delta = analysis.angle_difference(data[joint + '_q_rad'])
    valid = analysis.contiguous(data['sim_time_s'], data['iteration']) & np.isfinite(delta)
    values = np.full(len(dt) + 1, np.nan)
    values[1:][valid] = delta[valid] / dt[valid]
    return values


def read_bag(path):
    import rosbag
    feedback, goals, results = {}, [], []
    with rosbag.Bag(str(path)) as bag:
        topics = [topic for topic in bag.get_type_and_topic_info().topics
                  if topic == '/ground/joint_states' or topic in ('/ground/arm_controller/state', '/ground/gripper_controller/state')
                  or topic.endswith('/arm_controller/follow_joint_trajectory/feedback')
                  or topic.endswith('/arm_controller/follow_joint_trajectory/goal')
                  or topic.endswith('/arm_controller/follow_joint_trajectory/result')]
        for topic, message, receipt in bag.read_messages(topics=topics):
            if topic.endswith('/goal'):
                trajectory = message.goal.trajectory
                if trajectory.points:
                    nominal_start = trajectory.header.stamp.to_sec() or message.goal_id.stamp.to_sec()
                    goals.append(dict(goal_id=message.goal_id.id, receipt_s=receipt.to_sec(),
                                      goal_stamp_s=message.goal_id.stamp.to_sec(), trajectory_header_s=trajectory.header.stamp.to_sec(),
                                      duration_s=trajectory.points[-1].time_from_start.to_sec(),
                                      nominal_end_s=nominal_start + trajectory.points[-1].time_from_start.to_sec(),
                                      joint_names=list(trajectory.joint_names), q_first=list(trajectory.points[0].positions),
                                      q_last=list(trajectory.points[-1].positions)))
                continue
            if topic.endswith('/result'):
                results.append(dict(goal_id=message.status.goal_id.id, header_s=message.header.stamp.to_sec(),
                                    receipt_s=receipt.to_sec(), status=message.status.status,
                                    error_code=message.result.error_code, error_string=message.result.error_string))
                continue
            payload = message.feedback if hasattr(message, 'feedback') else message
            if topic == '/ground/joint_states':
                names, positions, velocities = payload.name, payload.position, payload.velocity
            else:
                names, positions, velocities = payload.joint_names, payload.actual.positions, payload.actual.velocities
            columns = feedback.setdefault(topic, {joint: [] for joint in PUBLIC_JOINTS})
            for index, name in enumerate(names):
                if name in columns:
                    columns[name].append((message.header.stamp.to_nsec(),
                                          positions[index] if index < len(positions) else np.nan,
                                          velocities[index] if index < len(velocities) else np.nan))
    return {topic: {joint: np.asarray(rows, dtype=float).reshape(-1, 3) for joint, rows in columns.items()}
            for topic, columns in feedback.items()}, goals, results


def compare_reference(rows, data, joint):
    timestamps = np.rint(data['sim_time_s'] * 1e9).astype(np.int64)
    if not len(timestamps):
        return dict(matched=0, unmatched=len(rows))
    indices = np.searchsorted(timestamps, rows[:, 0].astype(np.int64))
    bounded = np.minimum(indices, len(timestamps) - 1)
    matched = (indices < len(timestamps)) & (timestamps[bounded] == rows[:, 0].astype(np.int64))
    indices, selected = indices[matched], rows[matched]
    expected = reference_rates(data, joint)[indices]
    native_q = data[joint + '_q_rad'][indices]
    return dict(matched=int(matched.sum()), unmatched=int((~matched).sum()),
                position_wrapped_error_rad=analysis.stats((selected[:, 1] - native_q + np.pi) % (2 * np.pi) - np.pi),
                public_minus_pose_interval_rate_rad_s=analysis.stats(selected[:, 2] - expected),
                public_minus_native_rate_rad_s=analysis.stats(selected[:, 2] - data[joint + '_dq_rad_s'][indices]))


def summarize_feedback(feedback, phases, window):
    out = {}
    for topic, joints in feedback.items():
        out[topic] = {}
        for joint, rows in joints.items():
            selected = rows[(rows[:, 0] >= window['start_s'] * 1e9) & (rows[:, 0] <= window['end_s'] * 1e9)]
            if not len(selected):
                continue
            dt = np.diff(selected[:, 0]) * 1e-9
            increments = analysis.angle_difference(selected[:, 1])
            usable = (dt > 0) & np.isfinite(increments)
            item = dict(samples=len(selected), public_position_rad=analysis.stats(selected[:, 1]),
                        public_velocity_rad_s=analysis.stats(selected[:, 2]),
                        public_header_dt_s=analysis.stats(dt),
                        public_coarse_position_rate_rad_s=analysis.stats(increments[usable] / dt[usable]),
                        raw_csv_reference_available=joint in analysis.JOINTS)
            if joint in analysis.JOINTS:
                item['phase_references'] = {name: compare_reference(selected, values, joint) for name, values in phases.items()}
            out[topic][joint] = item
    return out


def report(result):
    lines = ['# Launch05: public integrated feedback and physical motion', '',
             f"Runtime integrated-mode log confirmed: {result['runtime_integrated_mode_confirmed']}. "
             'The raw native CSV is retained separately; no native values are rewritten.', '',
             '| Window | Wrist unwrapped travel, rad | Public joint-state wrist speed abs p95 / max, rad/s | Max public minus 1 ms before-set pose rate, rad/s | Native wrist dq median, rad/s |',
             '|---|---:|---|---:|---:|']
    for window in result['windows']:
        body = window['raw_phases']['before_base_set']; feedback = window['feedback'].get('/ground/joint_states', {}).get('wrist_3_joint')
        if feedback:
            rate, match = feedback['public_velocity_rad_s'], feedback['phase_references']['before_base_set']
            lines.append(f"| {window['name']} | {body['q_increment_sum_rad']:.8g} | {rate['p95_abs']:.8g} / {rate['max_abs']:.8g} | "
                         f"{match['public_minus_pose_interval_rate_rad_s']['max_abs']:.8g} | {body['native_rate_rad_s']['median']:.8g} |")
    lines += ['', '## Physical target and wrist-child motion', '']
    for window in result['windows']:
        bodies = window['raw_phases']['update_end']['bodies']
        lines.append(f"- {window['name']} ({window['start_s']}–{window['end_s']} s): "
                     f"target XYZ change {bodies['target']['xyz_change_m']} m; "
                     f"wrist-child XYZ change {bodies['wrist_child']['xyz_change_m']} m; "
                     f"target-in-hand displacement max {bodies['target_relative_to_wrist_child']['displacement_from_first_m']['max_abs']} m.")
    lines += ['', '## Limits', ''] + ['- ' + item for item in result['limitations']]
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--attempt-dir', type=Path)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        return 0 if unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(NumericChecks)).wasSuccessful() else 1
    if args.attempt_dir is None or args.output_dir is None or args.output_dir.exists():
        parser.error('--attempt-dir and a new --output-dir are required')
    attempt = json.loads((args.attempt_dir / 'attempt.json').read_text())
    if attempt.get('finish_wall') is None:
        parser.error('analyze only a completed attempt')
    events = [json.loads(line) for line in (args.attempt_dir / 'data/events.jsonl').read_text().splitlines()]
    windows = motion_windows(events)
    windows += [item for item in analysis.event_intervals(events)[0] if item['name'] != 'lift_tail_2s_proxy']
    feedback, goals, results = read_bag(args.attempt_dir / 'diagnostics.bag')
    lift = next(item for item in windows if item['name'] == 'lift')
    lift_goals = [goal for goal in goals if lift['start_s'] <= goal['goal_stamp_s'] <= lift['end_s']]
    for index, goal in enumerate(lift_goals):
        if goal['nominal_end_s'] < lift['end_s']:
            windows.append(dict(name='post_nominal_lift_endpoint_' + str(index), start_s=goal['nominal_end_s'], end_s=lift['end_s'],
                                boundary='recorded goal start stamp + final duration to stage END; nominal controller-start convention'))
        for outcome in results:
            if outcome['goal_id'] == goal['goal_id'] and outcome['header_s'] < lift['end_s']:
                windows.append(dict(name='post_controller_result_' + str(index), start_s=outcome['header_s'], end_s=lift['end_s'],
                                    boundary='public action result header to stage END'))
    data, metadata = analysis.read_trace(args.attempt_dir / 'ground-dynamics.csv', min(item['start_s'] for item in windows) - .002,
                                         max(item['end_s'] for item in windows))
    phases = {name: analysis.subset(data, data['phase'] == index) for index, name in enumerate(analysis.PHASES)}
    logs = [line for line in (args.attempt_dir / 'runtime.log').read_text().splitlines() if 'Ground joint velocity feedback mode:' in line]
    result = dict(diagnostic_only=True, runtime_integrated_mode_confirmed=any('integrated_position_interval_average' in line for line in logs),
                  runtime_mode_log=logs, requested_mode=attempt.get('joint_velocity_feedback'), trace_read=metadata,
                  lift_goals=lift_goals, arm_results=results,
                  terminal_events=[event for event in events if event.get('state') in ('LIFT', 'FAILED')], windows=[],
                  limitations=[
                      'All whole event-defined observation/pregrasp/descent/close/lift/retention phases are retained; no quiet subwindow or value selection is used.',
                      'Post-nominal endpoint uses explicit trajectory stamp if nonzero, otherwise goal-id stamp, plus final duration. Public result-to-stage-END is separately reported.',
                      'Expected feedback reference is the before-base-set WorldUpdateBegin pose difference over the preceding consecutive physics step. Same-header after/end references are also reported, not selected for best fit.',
                      'Public messages are matched by their header nanoseconds to rounded CSV simulation time. No interpolation, timestamp shifts or bag-receipt alignment is used.',
                      'All seven public joint position/velocity distributions are reported. The gripper has no raw per-step CSV column, so its 1 ms derivative is not independently validated by this CSV.',
                      'Public coarse-position differences are descriptive averages over the public message interval, not the 1 ms hardware feedback calculation.',
                      'Raw native ODE rate and integrated-pose feedback have different semantics. The native discrepancy remains visible and is not overwritten.',
                      'Wrist relative-quaternion validation is independent of the native joint angle; other arm joints have raw position references but no simultaneous per-joint quaternion columns.',
                      'Physical target/TCP motion and retention evidence are separate from controller action success; no additional acceptance threshold is applied.'
                  ])
    for window in windows:
        raw_phases = {name: validation.validate_phase(analysis.subset(values,
                      (values['sim_time_s'] >= window['start_s']) & (values['sim_time_s'] <= window['end_s'])))
                      for name, values in phases.items()}
        result['windows'].append(dict(window, raw_phases=raw_phases, feedback=summarize_feedback(feedback, phases, window)))
    result = analysis.clean_json(result)
    args.output_dir.mkdir(parents=True)
    (args.output_dir / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    (args.output_dir / 'REPORT.md').write_text(report(result))
    print(json.dumps(dict(output=str(args.output_dir), mode_confirmed=result['runtime_integrated_mode_confirmed'],
                          windows=[window['name'] for window in result['windows']])))
    return 0


class NumericChecks(unittest.TestCase):
    def test_whole_motion_windows_include_descent_close_and_preserve_first_start(self):
        self.assertIn('motion_windows', globals())
        events = [dict(state='A6_STAGE_START', stage='refined_pregrasp', ros_time=1.),
                  dict(state='A6_STAGE_START', stage='refined_pregrasp', ros_time=1.01),
                  dict(state='A6_STAGE_END', stage='refined_pregrasp', ros_time=2.),
                  dict(state='A6_STAGE_START', stage='descend', ros_time=2.),
                  dict(state='A6_STAGE_END', stage='descend', ros_time=3.),
                  dict(state='A6_STAGE_START', stage='close', ros_time=3.),
                  dict(state='A6_STAGE_END', stage='close', ros_time=4.)]
        windows = motion_windows(events)
        self.assertEqual(['refined_pregrasp_before_close', 'descend', 'close'], [item['name'] for item in windows])
        self.assertEqual(1., windows[0]['start_s'])

    def test_reference_uses_prior_consecutive_step_not_public_sampling_period(self):
        self.assertIn('reference_rates', globals())
        data = dict(sim_time_s=np.array([1., 1.001, 1.002, 1.004]), iteration=np.array([1, 2, 3, 5]),
                    wrist_3_joint_q_rad=np.array([0., .001, .003, .004]))
        rate = reference_rates(data, 'wrist_3_joint')
        self.assertTrue(np.isnan(rate[0]))
        self.assertAlmostEqual(1., rate[1])
        self.assertAlmostEqual(2., rate[2])
        self.assertTrue(np.isnan(rate[3]))


if __name__ == '__main__':
    raise SystemExit(main())
