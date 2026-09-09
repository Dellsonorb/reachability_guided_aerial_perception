#!/usr/bin/env python3
"""Bounded offline validation of a proposed integrated-pose velocity semantic."""

import argparse
import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np

_spec = importlib.util.spec_from_file_location('dynamics_analysis', Path(__file__).with_name('analyze.py'))
analysis = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(analysis)


def rotation_deltas(parent, child, axis_world):
    parent, child = analysis.quat_normalize(parent), analysis.quat_normalize(child)
    relative = analysis.quat_multiply(analysis.quat_conjugate(parent), child)
    delta = analysis.quat_normalize(analysis.quat_multiply(relative[1:], analysis.quat_conjugate(relative[:-1])))
    delta = np.where(delta[:, :1] < 0, -delta, delta)
    norm = np.linalg.norm(delta[:, 1:], axis=1)
    scale = np.full(norm.shape, 2.)
    np.divide(2 * np.arctan2(norm, delta[:, 0]), norm, out=scale, where=norm > 1e-15)
    rotvec = delta[:, 1:] * scale[:, None]
    axis_local = analysis.quat_rotate(analysis.quat_conjugate(parent[:-1]), axis_world[:-1])
    with np.errstate(invalid='ignore', divide='ignore'):
        axis_local /= np.linalg.norm(axis_local, axis=1, keepdims=True)
    projected = np.sum(rotvec * axis_local, axis=1)
    orthogonal = np.linalg.norm(rotvec - projected[:, None] * axis_local, axis=1)
    return projected, orthogonal


def windows_from_events(events):
    windows, active = [], {}
    for event in events:
        state, stage, time = event.get('state'), event.get('stage'), event.get('ros_time')
        if stage in ('ground_refine', 'refined_pregrasp'):
            if state == 'A6_STAGE_START':
                active.setdefault(stage, float(time))
            elif state in ('A6_STAGE_END', 'A6_STAGE_FAILED') and stage in active:
                windows.append(dict(name=stage + '_before_close', start_s=active.pop(stage), end_s=float(time)))
    measured, _ = analysis.event_intervals(events)
    windows.extend(item for item in measured if item['name'] in ('lift_tail_2s_proxy', 'closed_hold'))
    return windows


def validate_phase(data):
    time, iteration = data['sim_time_s'], data['iteration']
    dt, q = np.diff(time), data['wrist_3_joint_q_rad']
    qdelta = analysis.angle_difference(q)
    axis = analysis.vector(data, 'wrist_axis_world_', ('x', 'y', 'z'))
    projected, orthogonal = rotation_deltas(analysis.quaternion(data, 'wrist_parent'),
                                            analysis.quaternion(data, 'wrist_child'), axis)
    consecutive = analysis.contiguous(time, iteration)
    geometry_valid = (np.isfinite(projected) & np.isfinite(orthogonal) & np.isfinite(qdelta)
                      & (data['wrist_parent_present'][:-1] == 1) & (data['wrist_parent_present'][1:] == 1)
                      & (data['wrist_child_present'][:-1] == 1) & (data['wrist_child_present'][1:] == 1)
                      & (data['wrist_3_joint_present'][:-1] == 1) & (data['wrist_3_joint_present'][1:] == 1))
    valid = consecutive & geometry_valid
    pose_rate, quaternion_rate = qdelta[valid] / dt[valid], projected[valid] / dt[valid]
    unwrapped = np.concatenate(([0.], np.cumsum(qdelta))) if len(q) else np.array([])
    out = dict(rows=len(time), dt_s=analysis.stats(dt),
        consecutive_iteration_pairs=int(consecutive.sum()), dt_or_iteration_gap_pairs=int((~consecutive).sum()),
        nonfinite_or_missing_geometry_pairs=int((~geometry_valid).sum()), validated_pairs=int(valid.sum()),
        raw_wrap_crossings=int(np.sum(np.abs(np.diff(q)) > np.pi)),
        unwrapped_joint_span_rad=float(np.ptp(unwrapped)) if len(q) and consecutive.all() and geometry_valid.all() else None,
        q_increment_sum_rad=analysis.finite_sum(qdelta[valid]),
        quaternion_increment_sum_rad=analysis.finite_sum(projected[valid]),
        increment_discrepancy_rad=analysis.stats(qdelta[valid] - projected[valid]),
        rate_discrepancy_rad_s=analysis.stats(pose_rate - quaternion_rate),
        orthogonal_increment_norm_rad=analysis.stats(orthogonal[valid]),
        orthogonal_rate_norm_rad_s=analysis.stats(orthogonal[valid] / dt[valid]),
        pose_rate_rad_s=analysis.stats(pose_rate), quaternion_rate_rad_s=analysis.stats(quaternion_rate),
        native_rate_rad_s=analysis.stats(data['wrist_3_joint_dq_rad_s']),
        native_right_endpoint_minus_pose_rate_rad_s=analysis.stats(data['wrist_3_joint_dq_rad_s'][1:][valid] - pose_rate),
        all_joint_pose_rate_rad_s={}, bodies=analysis.pose_and_target(data))
    for joint in analysis.JOINTS:
        delta = analysis.angle_difference(data[joint + '_q_rad'])
        usable = (consecutive & np.isfinite(delta) & (data[joint + '_present'][:-1] == 1)
                  & (data[joint + '_present'][1:] == 1))
        out['all_joint_pose_rate_rad_s'][joint] = dict(stats=analysis.stats(delta[usable] / dt[usable]),
                                                     excluded_pairs=int(usable.size - usable.sum()))
    return out


def report(result):
    lines = ['# Offline validation of integrated-pose velocity semantics', '',
             'No feedback correction is implemented or approved by this analysis. '
             'The proposed quantity is a one-step average pose velocity, not necessarily the instantaneous native ODE rate.', '',
             '| Run / window (update end) | Wrist unwrapped span, rad | Max / p95 angle discrepancy, rad | Max / p95 rate discrepancy, rad/s | Max off-axis rate, rad/s | Pose speed abs p95 / max, rad/s |',
             '|---|---:|---|---|---:|---|']
    def pair(stats):
        return f"{stats['max_abs']:.8g} / {stats['p95_abs']:.8g}" if stats['count'] else 'missing'
    for run in result['runs']:
        for window in run['windows']:
            data = window['phases']['update_end']
            span = data['unwrapped_joint_span_rad']
            speed = data['pose_rate_rad_s']
            lines.append(f"| {run['name']} / {window['name']} | {span:.8g} | "
                         + pair(data['increment_discrepancy_rad']) + ' | ' + pair(data['rate_discrepancy_rad_s'])
                         + f" | {data['orthogonal_rate_norm_rad_s']['max_abs']:.8g} | {speed['p95_abs']:.8g} / {speed['max_abs']:.8g} |")
    lines += ['', '## Scope, missing data and physical state', '']
    for run in result['runs']:
        lines.append(f"- {run['name']}: {run['interpretation']}; {run['trace_read']}.")
        for window in run['windows']:
            data = window['phases']['update_end']; body = data['bodies']
            lines.append(f"  - {window['name']} {window['start_s']}–{window['end_s']} s: "
                         f"{data['validated_pairs']} validated pairs; {data['dt_or_iteration_gap_pairs']} dt/iteration gaps; "
                         f"{data['nonfinite_or_missing_geometry_pairs']} missing/nonfinite geometry pairs. "
                         f"Target XYZ change {body['target']['xyz_change_m']} m; "
                         f"relative-to-wrist maximum movement {body['target_relative_to_wrist_child']['displacement_from_first_m']['max_abs']} m.")
    lines += ['', '## Limitations', ''] + ['- ' + item for item in result['limitations']]
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch-dir', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        return 0 if unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(NumericChecks)).wasSuccessful() else 1
    if args.output_dir is None or args.output_dir.exists():
        parser.error('--output-dir must name a new directory')
    result = dict(diagnostic_only=True, feedback_changes=False, runs=[], limitations=[
        'Independent link-quaternion validation is available only for wrist3. Other five joint pose speeds are descriptive, not independently validated here.',
        'Unwrapped delta uses shortest angular difference each step; an actual rotation exceeding pi per step would be ambiguous.',
        'Quaternion comparison removes parent motion: Rrelative = Rparent inverse * Rchild, delta Rrelative projected on the parent-local joint axis. Off-axis residuals are reported, not discarded.',
        'A finite-difference quantity is a one-step interval-average velocity. Its agreement with actual pose motion does not establish equivalence to native instantaneous solver velocity.',
        'No filtering, threshold relaxation, native-rate substitution, trial relabelling, or controller change is made.',
        'Event-bounded pre-close motion and measured open-gripper width support an unloaded movement classification; contact reaction forces are not recorded.',
        'Loaded holds retain grasp confirmation and elevated target motion. Failed opening attempts are not certified unloaded states.',
        'The world-solver launch04 reported LCP failure. Its complete trace is a failed-physics reference only, never adoption evidence.',
        'A future feedback implementation still requires startup/reset/dt handling, actual readSim-period checks, control stability review and bounded physical validation with original acceptance.'
    ])
    for name in ('launch-02-easy-generic-original', 'launch-03-easy-measured-release', 'launch-04-easy-world-solver'):
        directory = args.batch_dir / name
        events = [json.loads(line) for line in (directory / 'data/events.jsonl').read_text().splitlines() if line.strip()]
        windows = windows_from_events(events)
        world_solver = name.startswith('launch-04')
        data, metadata = analysis.read_trace(directory / 'ground-dynamics.csv',
                                            -np.inf if world_solver else min(item['start_s'] for item in windows),
                                            np.inf if world_solver else max(item['end_s'] for item in windows))
        if world_solver:
            windows = [dict(name='entire_trace_failed_physics', start_s=float(data['sim_time_s'].min()),
                            end_s=float(data['sim_time_s'].max()))]
        run = dict(name=name, trace_read=metadata, windows=[],
                   interpretation='LCP-failed world solver, not adoption evidence' if world_solver else 'original quick solver; pre-close motion and loaded holds',
                   relevant_events=[event for event in events if event.get('state') in ('GROUND_REFINED', 'GRASP', 'GROUND_LOAD_DIAGNOSTIC_BEGIN', 'GROUND_LOAD_DIAGNOSTIC_END', 'FAILED')])
        for window in windows:
            selected = analysis.subset(data, (data['sim_time_s'] >= window['start_s']) & (data['sim_time_s'] <= window['end_s']))
            run['windows'].append(dict(window, phases={phase: validate_phase(analysis.subset(selected, selected['phase'] == index))
                                                       for index, phase in enumerate(analysis.PHASES)}))
        result['runs'].append(run)
    result = analysis.clean_json(result)
    args.output_dir.mkdir(parents=True)
    (args.output_dir / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    (args.output_dir / 'REPORT.md').write_text(report(result))
    print(json.dumps(dict(output=str(args.output_dir), runs=[run['name'] for run in result['runs']])))
    return 0


class NumericChecks(unittest.TestCase):
    def test_large_hinge_motion_with_moving_parent_and_world_axis(self):
        self.assertIn('rotation_deltas', globals())
        hinge = np.linspace(-3.2, 3.2, 101)
        parent_angle = np.linspace(-1., 1., 101)
        parent = np.column_stack((np.cos(parent_angle / 2), np.zeros(101),
                                  np.sin(parent_angle / 2), np.zeros(101)))
        local = np.column_stack((np.cos(hinge / 2), np.sin(hinge / 2), np.zeros((101, 2))))
        child = analysis.quat_multiply(parent, local)
        axis = analysis.quat_rotate(parent, np.tile([1., 0, 0], (101, 1)))
        projected, orthogonal = rotation_deltas(parent, child, axis)
        self.assertLess(np.max(np.abs(projected - np.diff(hinge))), 1e-13)
        self.assertLess(np.max(orthogonal), 1e-13)
        wrapped = (hinge + np.pi) % (2 * np.pi) - np.pi
        self.assertLess(np.max(np.abs(analysis.angle_difference(wrapped) - projected)), 1e-13)

    def test_off_axis_rotation_is_not_hidden_by_projection(self):
        self.assertIn('rotation_deltas', globals())
        parent = np.array([[1., 0, 0, 0], [1., 0, 0, 0]])
        child = np.array([[1., 0, 0, 0], [np.cos(.0005), 0, np.sin(.0005), 0]])
        projected, orthogonal = rotation_deltas(parent, child, np.array([[1., 0, 0], [1., 0, 0]]))
        self.assertAlmostEqual(0., projected[0], places=14)
        self.assertAlmostEqual(.001, orthogonal[0], places=14)


if __name__ == '__main__':
    raise SystemExit(main())
