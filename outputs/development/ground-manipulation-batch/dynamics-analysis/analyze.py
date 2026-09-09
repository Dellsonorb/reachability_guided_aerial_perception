#!/usr/bin/env python3
"""Offline native-rate/pose comparison; never replaces feedback or task outcomes."""

import argparse
from array import array
import csv
import json
from pathlib import Path
import unittest

import numpy as np

PHASES = ('before_base_set', 'after_base_set', 'update_end')
JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def stats(values):
    values = np.asarray(values, dtype=float).ravel()
    finite = values[np.isfinite(values)]
    result = dict(count=int(finite.size), missing=int(values.size - finite.size))
    result.update({key: None for key in ('min', 'max', 'mean', 'median', 'p95_abs', 'max_abs')})
    if finite.size:
        result.update(min=float(finite.min()), max=float(finite.max()), mean=float(finite.mean()),
                      median=float(np.median(finite)),
                      p95_abs=float(np.percentile(np.abs(finite), 95)),
                      max_abs=float(np.abs(finite).max()))
    return result


def finite_sum(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return float(values.sum()) if values.size else None


def angle_difference(values):
    return (np.diff(values) + np.pi) % (2 * np.pi) - np.pi


def contiguous(time, iteration):
    return (np.diff(iteration) == 1) & (np.diff(time) > 0)


def rate_cadence(time, iteration, rate):
    valid = np.isfinite(rate)
    result = dict(even_iteration_rate=stats(rate[(iteration % 2 == 0) & valid]),
                  odd_iteration_rate=stats(rate[(iteration % 2 == 1) & valid]),
                  negative_samples=int(np.sum(rate < 0)), positive_samples=int(np.sum(rate > 0)),
                  dominant_nonzero_fft_hz=None, dominant_abs_fft_over_n=None,
                  lag1_correlation=None, uniform_contiguous_samples=False)
    dt = np.diff(time)
    if (len(rate) < 3 or not valid.all() or not contiguous(time, iteration).all()
            or not np.allclose(dt, dt[0], rtol=1e-6, atol=1e-12)):
        return result
    result['uniform_contiguous_samples'] = True
    centered = rate - rate.mean()
    amplitude = np.abs(np.fft.rfft(centered)) / len(rate)
    if np.std(centered) > 0:
        index = int(np.argmax(amplitude[1:]) + 1)
        result.update(dominant_nonzero_fft_hz=float(np.fft.rfftfreq(len(rate), np.median(dt))[index]),
                      dominant_abs_fft_over_n=float(amplitude[index]),
                      lag1_correlation=float(np.corrcoef(rate[:-1], rate[1:])[0, 1]))
    return result


def joint_motion(time, iteration, q, dq):
    dt, delta = np.diff(time), angle_difference(q)
    valid = (contiguous(time, iteration) & np.isfinite(delta)
             & np.isfinite(dq[:-1]) & np.isfinite(dq[1:]))
    left, right = dq[:-1][valid] * dt[valid], dq[1:][valid] * dt[valid]
    trap = (left + right) / 2
    finite_q = q[np.isfinite(q)]
    return dict(q_rad=stats(q), native_dq_rad_s=stats(dq),
                q_first_rad=float(finite_q[0]) if finite_q.size else None,
                q_last_rad=float(finite_q[-1]) if finite_q.size else None,
                q_span_rad=float(np.ptp(finite_q)) if finite_q.size else None,
                valid_contiguous_steps=int(valid.sum()),
                excluded_steps=int(valid.size - valid.sum()),
                integrated_duration_s=float(dt[valid].sum()),
                q_increment_sum_rad=finite_sum(delta[valid]),
                q_absolute_increment_sum_rad=finite_sum(np.abs(delta[valid])),
                dq_left_integral_rad=finite_sum(left), dq_right_integral_rad=finite_sum(right),
                dq_trapezoid_integral_rad=finite_sum(trap),
                q_minus_dq_trapezoid_rad=finite_sum(delta[valid] - trap),
                q_increment_rate_rad_s=stats(delta[valid] / dt[valid]))


def quat_conjugate(q):
    return q * np.array([1., -1., -1., -1.])


def quat_normalize(q):
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.where(norm > 0, q / norm, np.nan)


def quat_multiply(a, b):
    scalar = a[..., :1] * b[..., :1] - np.sum(a[..., 1:] * b[..., 1:], axis=-1, keepdims=True)
    vector = a[..., :1] * b[..., 1:] + b[..., :1] * a[..., 1:] + np.cross(a[..., 1:], b[..., 1:])
    return np.concatenate((scalar, vector), axis=-1)


def quat_rotate(q, v):
    q = quat_normalize(q)
    return v + 2 * np.cross(q[..., 1:], np.cross(q[..., 1:], v) + q[..., :1] * v)


def relative_pose_increments(parent, child, axis_world):
    parent, child = quat_normalize(parent), quat_normalize(child)
    relative = quat_multiply(quat_conjugate(parent), child)
    increment = quat_normalize(quat_multiply(relative[1:], quat_conjugate(relative[:-1])))
    increment = np.where(increment[:, :1] < 0, -increment, increment)
    norm = np.linalg.norm(increment[:, 1:], axis=1)
    scale = np.full(norm.shape, 2.)
    np.divide(2 * np.arctan2(norm, increment[:, 0]), norm, out=scale, where=norm > 1e-15)
    rotation_vector = increment[:, 1:] * scale[:, None]
    local_axis = quat_rotate(quat_conjugate(parent[:-1]), axis_world[:-1])
    return np.sum(rotation_vector * local_axis, axis=1)


def event_intervals(events):
    active, intervals = {}, []
    release_start = None
    for event in events:
        state, time = event.get('state'), event.get('ros_time')
        stage = event.get('stage')
        if state == 'A6_STAGE_START' and stage in ('lift', 'retention'):
            active[stage] = float(time)
        elif state in ('A6_STAGE_END', 'A6_STAGE_FAILED') and stage in active:
            start = active.pop(stage)
            intervals.append(dict(name=stage, start_s=start, end_s=float(time), boundary='exact stage events'))
            if stage == 'lift':
                intervals.append(dict(name='lift_tail_2s_proxy', start_s=max(start, float(time) - 2),
                                      end_s=float(time), boundary='last 2 s of lift; not a logged settle-start'))
        elif state == 'GROUND_LOAD_DIAGNOSTIC_BEGIN':
            active[event['phase']] = float(time)
            release_start = None
        elif state == 'GROUND_LOAD_DIAGNOSTIC_END' and event.get('phase') in active:
            intervals.append(dict(name=event['phase'], start_s=active.pop(event['phase']),
                                  end_s=float(time), boundary='exact load diagnostic events'))
            if event['phase'] == 'closed_hold':
                release_start = float(time)
        elif state == 'GROUND_LOAD_DIAGNOSTIC_FAILED' and release_start is not None:
            intervals.append(dict(name='release_attempt_failed', start_s=release_start,
                                  end_s=float(time), boundary='closed-hold end to diagnostic failure; not an open hold'))
            release_start = None
    return intervals, sorted(active)


def read_trace(path, start, end):
    packed, kept, total, malformed, unknown = array('d'), 0, 0, 0, 0
    with path.open(newline='') as stream:
        reader = csv.reader(stream)
        names = next(reader)
        if len(names) != len(set(names)):
            raise ValueError('duplicate CSV column names')
        time_id, phase_id = names.index('sim_time_s'), names.index('phase')
        for row in reader:
            total += 1
            if len(row) != len(names):
                malformed += 1
                continue
            time = float(row[time_id])
            if not start <= time <= end:
                continue
            if row[phase_id] not in PHASES:
                unknown += 1
                continue
            row[phase_id] = str(PHASES.index(row[phase_id]))
            packed.extend(float(value) for value in row)
            kept += 1
    matrix = np.frombuffer(packed, dtype=np.float64).reshape(kept, len(names))
    data = {name: matrix[:, i] for i, name in enumerate(names)}
    return data, dict(total_csv_rows=total, retained_rows=kept, malformed_rows=malformed,
                      unknown_phase_rows=unknown, columns=len(names))


def subset(data, mask):
    return {key: values[mask] for key, values in data.items()}


def vector(data, prefix, suffixes):
    return np.column_stack([data[prefix + suffix] for suffix in suffixes])


def quaternion(data, prefix):
    return vector(data, prefix + '_', ('qw', 'qx', 'qy', 'qz'))


def pose_and_target(data):
    out = {}
    for prefix in ('base', 'wrist_parent', 'wrist_child', 'target'):
        position = vector(data, prefix + '_', ('x_m', 'y_m', 'z_m'))
        valid = (data[prefix + '_present'] == 1) & np.isfinite(position).all(axis=1)
        points = position[valid]
        out[prefix] = dict(present_rows=int(valid.sum()), missing_rows=int((~valid).sum()),
                           xyz_start_m=points[0].tolist() if len(points) else None,
                           xyz_end_m=points[-1].tolist() if len(points) else None,
                           xyz_span_m=np.ptp(points, axis=0).tolist() if len(points) else None,
                           xyz_change_m=(points[-1] - points[0]).tolist() if len(points) else None,
                           angular_speed_rad_s=stats(np.linalg.norm(vector(data, prefix + '_',
                                                      ('wx_rad_s', 'wy_rad_s', 'wz_rad_s')), axis=1)))
    target = vector(data, 'target_', ('x_m', 'y_m', 'z_m'))
    child = vector(data, 'wrist_child_', ('x_m', 'y_m', 'z_m'))
    relative = quat_rotate(quat_conjugate(quaternion(data, 'wrist_child')), target - child)
    valid = ((data['target_present'] == 1) & (data['wrist_child_present'] == 1)
             & np.isfinite(relative).all(axis=1))
    positions = relative[valid]
    out['target_relative_to_wrist_child'] = dict(
        present_rows=int(valid.sum()), missing_rows=int((~valid).sum()),
        xyz_start_m=positions[0].tolist() if len(positions) else None,
        xyz_end_m=positions[-1].tolist() if len(positions) else None,
        xyz_span_m=np.ptp(positions, axis=0).tolist() if len(positions) else None,
        displacement_from_first_m=stats(np.linalg.norm(positions - positions[0], axis=1)) if len(positions) else stats([]))
    return out


def phase_summary(data):
    time, iteration = data['sim_time_s'], data['iteration']
    out = dict(rows=len(time), dt_s=stats(np.diff(time)), joints={})
    for joint in JOINTS:
        q, dq = data[joint + '_q_rad'].copy(), data[joint + '_dq_rad_s'].copy()
        absent = data[joint + '_present'] != 1
        q[absent], dq[absent] = np.nan, np.nan
        out['joints'][joint] = joint_motion(time, iteration, q, dq)
        out['joints'][joint]['force_native'] = stats(data[joint + '_force_native'])
    axis = vector(data, 'wrist_axis_world_', ('x', 'y', 'z'))
    parent_w = vector(data, 'wrist_parent_', ('wx_rad_s', 'wy_rad_s', 'wz_rad_s'))
    child_w = vector(data, 'wrist_child_', ('wx_rad_s', 'wy_rad_s', 'wz_rad_s'))
    projection = np.sum((child_w - parent_w) * axis, axis=1)
    increment = relative_pose_increments(quaternion(data, 'wrist_parent'),
                                        quaternion(data, 'wrist_child'), axis)
    valid = contiguous(time, iteration) & np.isfinite(increment)
    delta_q = angle_difference(data['wrist_3_joint_q_rad'])
    out['wrist_comparison'] = dict(
        native_rate_cadence=rate_cadence(time, iteration, data['wrist_3_joint_dq_rad_s']),
        recomputed_relative_rate_rad_s=stats(projection),
        logged_projection_error_rad_s=stats(data['wrist_relative_angular_rate_rad_s'] - projection),
        native_minus_relative_rate_rad_s=stats(data['wrist_3_joint_dq_rad_s'] - projection),
        quaternion_projected_increment_sum_rad=finite_sum(increment[valid]),
        quaternion_projected_rate_rad_s=stats(increment[valid] / np.diff(time)[valid]),
        quaternion_minus_joint_increment_rad=stats((increment - delta_q)[valid]),
        axis_norm=stats(np.linalg.norm(axis, axis=1)))
    out['bodies'] = pose_and_target(data)
    return out


def paired_changes(first, second, next_iteration=False):
    if (len(np.unique(first['iteration'])) != len(first['iteration'])
            or len(np.unique(second['iteration'])) != len(second['iteration'])):
        return dict(error='duplicate iteration identifiers; phase pairing is ambiguous')
    _, left, right = np.intersect1d(first['iteration'] + int(next_iteration),
                                   second['iteration'], return_indices=True)
    result = dict(pairs=len(left))
    for suffix in ('q_rad', 'dq_rad_s', 'force_native'):
        key = 'wrist_3_joint_' + suffix
        result['wrist_delta_' + suffix] = stats(second[key][right] - first[key][left])
    for prefix in ('base', 'wrist_parent', 'wrist_child'):
        for label, suffixes in (('angular', ('wx_rad_s', 'wy_rad_s', 'wz_rad_s')),
                                ('linear', ('vx_m_s', 'vy_m_s', 'vz_m_s'))):
            delta = vector(second, prefix + '_', suffixes)[right] - vector(first, prefix + '_', suffixes)[left]
            result[prefix + '_' + label + '_delta_norm'] = stats(np.linalg.norm(delta, axis=1))
    return result


def clean_json(value):
    if isinstance(value, dict):
        return {key: clean_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [clean_json(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def markdown(result):
    lines = ['# Ground dynamics: offline native-rate / pose comparison', '',
             'Diagnostic only. Original task outcomes and native feedback are unchanged. '
             'Missing/nonfinite values are JSON null with explicit counts; they are not zeros.', '',
             '| Interval / phase | Rows | Wrist q change, rad | Native dq trapezoid, rad | Relative-quaternion change, rad | Median native dq, rad/s | Median pose rate, rad/s |',
             '|---|---:|---:|---:|---:|---:|---:|']
    def fmt(value):
        return 'missing' if value is None else f'{value:.7g}'
    for interval in result['intervals']:
        for phase, summary in interval['phases'].items():
            wrist, comparison = summary['joints']['wrist_3_joint'], summary['wrist_comparison']
            values = [wrist['q_increment_sum_rad'], wrist['dq_trapezoid_integral_rad'],
                      comparison['quaternion_projected_increment_sum_rad'],
                      wrist['native_dq_rad_s']['median'], comparison['quaternion_projected_rate_rad_s']['median']]
            lines.append('| ' + interval['name'] + ' / ' + phase + ' | ' + str(summary['rows'])
                         + ' | ' + ' | '.join(fmt(value) for value in values) + ' |')
    lines += ['', '## Before/after base setters', '',
              '| Interval | Paired rows | Max abs wrist dq change, rad/s | Max abs wrist q change, rad | Max base angular-velocity change norm, rad/s |',
              '|---|---:|---:|---:|---:|']
    for interval in result['intervals']:
        pair = interval['phase_changes']['before_to_after']
        if 'error' not in pair:
            lines.append(f"| {interval['name']} | {pair['pairs']} | " + ' | '.join(fmt(pair[key]['max_abs']) for key in
                         ('wrist_delta_dq_rad_s', 'wrist_delta_q_rad', 'base_angular_delta_norm')) + ' |')
    lines += ['', '## Native-rate cadence at update end', '',
              '| Interval | Mean even / odd iteration dq, rad/s | Negative / positive samples | Largest nonzero FFT bin, Hz |',
              '|---|---|---:|---:|']
    for interval in result['intervals']:
        cadence = interval['phases']['update_end']['wrist_comparison']['native_rate_cadence']
        lines.append(f"| {interval['name']} | " + fmt(cadence['even_iteration_rate']['mean']) + ' / '
                     + fmt(cadence['odd_iteration_rate']['mean']) + f" | {cadence['negative_samples']} / {cadence['positive_samples']} | "
                     + fmt(cadence['dominant_nonzero_fft_hz']) + ' |')
    lines += ['', '## Target diagnostics at update end', '']
    for interval in result['intervals']:
        bodies = interval['phases']['update_end']['bodies']
        lines.append(f"- {interval['name']}: target XYZ change {bodies['target']['xyz_change_m']} m; "
                     f"target displacement in wrist-child coordinates max "
                     f"{fmt(bodies['target_relative_to_wrist_child']['displacement_from_first_m']['max_abs'])} m.")
    lines += ['', '## Limits', ''] + ['- ' + limit for limit in result['limitations']]
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path)
    parser.add_argument('--events', type=Path)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(NumericChecks)
        return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1
    if not all((args.trace, args.events, args.output_dir)):
        parser.error('--trace, --events, --output-dir are required')
    if args.output_dir.exists():
        parser.error('output directory must be new; historical inputs are never overwritten')
    events = [json.loads(line) for line in args.events.read_text().splitlines() if line.strip()]
    intervals, incomplete = event_intervals(events)
    if not intervals:
        parser.error('no completed lift/retention/load interval in events')
    data, metadata = read_trace(args.trace, min(item['start_s'] for item in intervals),
                                max(item['end_s'] for item in intervals))
    result = dict(diagnostic_only=True, source_trace=str(args.trace), source_events=str(args.events),
                  trace_read=metadata, incomplete_event_intervals=incomplete,
                  original_terminal_events=[event for event in events if event.get('state') in ('FAILED', 'LIFT', 'GROUND_REPLAY_END')],
                  closed_hold_recorded=any(item['name'] == 'closed_hold' for item in intervals),
                  open_hold_recorded=any(item['name'] == 'open_hold' for item in intervals),
                  load_diagnostic_failure_events=[event for event in events if event.get('state') == 'GROUND_LOAD_DIAGNOSTIC_FAILED'],
                  intervals=[], limitations=[
                      'No native feedback substitution, actuation change, synthetic vote, or new task success is produced.',
                      'Lift tail is the final 2 s before the stage outcome, not an exact commanded trajectory-end/settle-start timestamp.',
                      'Only positive-time consecutive iteration pairs are integrated; gaps and missing values are excluded and counted.',
                      'Left/right/trapezoid dq integrals are diagnostic quadrature; none is asserted to be the physics integration rule.',
                      'Quaternion increments use parent-frame child orientation and project its shortest-arc rotation onto the local hinge axis.',
                      'GetForce is native joint effort, not a measured gripping/contact reaction wrench.',
                      'Other update-begin callbacks may run after this plugin; after_base_set to update_end includes those callbacks and physics.',
                      'Target Model getters and link poses/twists are diagnostic truth only. Relative target movement is not a retention acceptance test.',
                      'Closed/open labels describe commanded gripper states, not proven loaded/unloaded force conditions.',
                      'A success with instrumentation alone does not establish that instrumentation fixed a prior failure.'
                  ])
    for interval in intervals:
        window = subset(data, (data['sim_time_s'] >= interval['start_s']) & (data['sim_time_s'] <= interval['end_s']))
        phases = {phase: subset(window, window['phase'] == i) for i, phase in enumerate(PHASES)}
        result['intervals'].append(dict(interval, phases={name: phase_summary(rows) for name, rows in phases.items()},
            phase_changes=dict(before_to_after=paired_changes(phases[PHASES[0]], phases[PHASES[1]]),
                               after_to_end=paired_changes(phases[PHASES[1]], phases[PHASES[2]]),
                               end_to_next_before=paired_changes(phases[PHASES[2]], phases[PHASES[0]], True))))
    result = clean_json(result)
    args.output_dir.mkdir(parents=True)
    (args.output_dir / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    (args.output_dir / 'REPORT.md').write_text(markdown(result))
    print(json.dumps(dict(output=str(args.output_dir), rows=metadata['retained_rows'],
                          intervals=[item['name'] for item in intervals])))
    return 0


class NumericChecks(unittest.TestCase):
    def test_failed_release_is_not_an_open_hold(self):
        intervals, _ = event_intervals([
            dict(state='GROUND_LOAD_DIAGNOSTIC_BEGIN', phase='closed_hold', ros_time=46.294),
            dict(state='GROUND_LOAD_DIAGNOSTIC_END', phase='closed_hold', ros_time=48.317),
            dict(state='GROUND_LOAD_DIAGNOSTIC_FAILED', ros_time=49.320)])
        self.assertEqual(['closed_hold', 'release_attempt_failed'], [item['name'] for item in intervals])
        self.assertEqual(48.317, intervals[-1]['start_s'])
        self.assertEqual(49.320, intervals[-1]['end_s'])
        self.assertIn('not an open hold', intervals[-1]['boundary'])

    def test_alternating_native_rate_is_visible_at_physics_cadence(self):
        self.assertIn('rate_cadence', globals())
        iteration = np.arange(200)
        result = rate_cadence(iteration * .001, iteration, .16 * (-1.) ** iteration)
        self.assertAlmostEqual(500., result['dominant_nonzero_fft_hz'])
        self.assertAlmostEqual(.16, result['even_iteration_rate']['mean'])
        self.assertAlmostEqual(-.16, result['odd_iteration_rate']['mean'])

    def test_stationary_angle_with_nonzero_native_rate_is_not_replaced(self):
        self.assertIn('joint_motion', globals())
        result = joint_motion(np.array([0., .001, .002]), np.array([1, 2, 3]),
                              np.zeros(3), np.full(3, -.16))
        self.assertEqual(0., result['q_increment_sum_rad'])
        self.assertAlmostEqual(-.00032, result['dq_trapezoid_integral_rad'])
        self.assertEqual(-.16, result['native_dq_rad_s']['median'])

    def test_relative_quaternion_removes_common_parent_rotation(self):
        self.assertIn('relative_pose_increments', globals())
        # Parent rotates about world Z; child additionally turns .001 about local X.
        parent = np.array([[1., 0, 0, 0], [np.cos(.1), 0, 0, np.sin(.1)]])
        local = np.array([[1., 0, 0, 0], [np.cos(.0005), np.sin(.0005), 0, 0]])
        child = quat_multiply(parent, local)
        axis = quat_rotate(parent, np.array([[1., 0, 0], [1., 0, 0]]))
        increment = relative_pose_increments(parent, child, axis)
        self.assertAlmostEqual(.001, increment[0], places=12)
        self.assertAlmostEqual(.001, relative_pose_increments(parent, -child, axis)[0], places=12)

    def test_missing_data_and_iteration_gaps_are_not_integrated(self):
        self.assertIn('joint_motion', globals())
        result = joint_motion(np.array([0., .001, .004, .005]), np.array([1, 2, 5, 6]),
                              np.array([0., .001, .004, np.nan]), np.ones(4))
        self.assertEqual(1, result['valid_contiguous_steps'])
        self.assertAlmostEqual(.001, result['q_increment_sum_rad'])
        self.assertEqual(1, result['q_rad']['missing'])
        self.assertIsNone(stats(np.array([np.nan]))['median'])


if __name__ == '__main__':
    raise SystemExit(main())
