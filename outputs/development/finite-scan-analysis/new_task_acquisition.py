#!/usr/bin/env python3
"""Read new task files once; compare saved acquisition with actual next windows.

This never changes task files, observations, votes, code, configuration or Git.
Use a new output filename for each snapshot while an online task is running.
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from finite_scan_probe import ROOT, HERE, old, packets_at_transform
from prism_scan_probe import pose_stats
from operational_occlusion_review import load_state
from reachability_guided_nbv.finite_scan import FiniteScan
from reachability_guided_nbv.operational_occlusion import build_operational_occlusion


def events_from(path):
    if not path.exists():
        return []
    text = path.read_text()
    lines = text.splitlines()
    events = []
    for i, line in enumerate(lines):
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            if i == len(lines) - 1 and not text.endswith('\n'):
                break
            raise
    return events


def viable(assessments):
    return [a for a in assessments if not a['representative_blocked']
            and not a['operational']['blocked'] and not a['footprint_clipped']]


def union(assessments):
    return np.unique(np.concatenate([a['operational']['covered_cells'] for a in assessments])).astype(int) if assessments else np.array([], dtype=int)


def compare(opportunity, actual, ids):
    p, ground = opportunity[ids], actual[ids]
    possible = p > 0
    return dict(cells=len(ids), actual_ground_cells=int(ground.sum()), possible_cells=int(possible.sum()),
        possible_with_ground=int(np.count_nonzero(possible & ground)),
        possible_without_ground=int(np.count_nonzero(possible & ~ground)),
        ground_outside_possible=int(np.count_nonzero(~possible & ground)),
        phase_opportunity_sum=float(p.sum()),
        phase_opportunity_on_cells_without_ground=float(p[~ground].sum()),
        one_minus_opportunity_on_actual_ground=float((1 - p[ground]).sum()),
        possible_without_ground_cell_ids=ids[possible & ~ground].tolist(),
        ground_outside_possible_cell_ids=ids[~possible & ground].tolist())


def phase_audit(observation, pattern, emitted, matrix, grid, config):
    mapped = observation['points_xyz'] @ observation['T_map_sensor'][:3, :3].T + observation['T_map_sensor'][:3, 3]
    phases, errors, offset = [], [], 0
    moving, nominal = [np.zeros(grid.width_cells * grid.height_cells, dtype=int) for _ in range(2)]
    for count, recorded in zip(observation['chunk_point_counts'], observation['chunk_T_map_sensor']):
        stop = offset + int(count)
        original = (mapped[offset:stop] - recorded[:3, 3]) @ recorded[:3, :3]
        phase_row, error = old.infer_packet_phase(original, pattern)
        phases.append(phase_row // 10000)
        errors.append(error)
        rays = pattern['directions'][phase_row:phase_row + 10000][emitted[phase_row:phase_row + 10000]]
        moving += packets_at_transform(rays, recorded, grid, config).sum(axis=0)
        nominal += packets_at_transform(rays, matrix, grid, config).sum(axis=0)
        offset = stop
    steps = np.diff(phases) % 80
    return dict(packet_count=len(phases), unique_phases=len(set(phases)), phases=phases,
        phase_step_counts={str(i): int(np.count_nonzero(steps == i)) for i in np.unique(steps)},
        every_retained_ray_matches_inferred_packet=True, maximum_angle_error_deg=float(max(errors))), nominal, moving


def draw_comparison(path, grid, union_ids, ideal, finite, actual, title):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.collections import LineCollection
    from matplotlib.figure import Figure
    from matplotlib.lines import Line2D
    if path.exists():
        raise FileExistsError(path)
    mask = np.zeros(grid.shape, dtype=bool)
    mask.ravel()[union_ids] = True
    segments = []
    for row, col in zip(*np.nonzero(mask)):
        x, y = np.array(grid.origin_xy) + grid.resolution_m * np.array([col, row])
        r = grid.resolution_m
        if col == 0 or not mask[row, col - 1]:
            segments.append([(x, y), (x, y + r)])
        if col == grid.width_cells - 1 or not mask[row, col + 1]:
            segments.append([(x + r, y), (x + r, y + r)])
        if row == 0 or not mask[row - 1, col]:
            segments.append([(x, y), (x + r, y)])
        if row == grid.height_cells - 1 or not mask[row + 1, col]:
            segments.append([(x, y + r), (x + r, y + r)])
    figure = Figure(figsize=(12.5, 4.3), layout='constrained')
    FigureCanvasAgg(figure)
    axes = figure.subplots(1, 3)
    for axis, values, label in zip(axes, (ideal, finite, actual.astype(float)),
                                  ('Old center + raw prisms', 'Finite scan + operational geometry', 'Recorded ground presence')):
        shown = axis.imshow(values.reshape(grid.shape), origin='lower', extent=grid.extent,
                            interpolation='none', cmap='viridis', vmin=0, vmax=1, aspect='equal')
        axis.add_collection(LineCollection(segments, colors='black', linewidths=1.0))
        if label != 'Recorded ground presence':
            stats = compare(values, actual, union_ids)
            for flag, style, color in (((values > 0) & ~actual, 'x', '#ed4b3c'),
                                       ((values == 0) & actual, 'o', '#d42de8')):
                cells = union_ids[flag[union_ids]]
                rr, cc = np.divmod(cells, grid.width_cells)
                axis.scatter(grid.origin_xy[0] + (cc + .5) * grid.resolution_m,
                             grid.origin_xy[1] + (rr + .5) * grid.resolution_m,
                             marker=style, color=color, s=16, linewidths=.8,
                             **({'facecolors': 'none'} if style == 'o' else {}))
            label += f"\nUnion FP {stats['possible_without_ground']}, FN {stats['ground_outside_possible']}"
        else:
            label += f'\n{int(actual[union_ids].sum())}/{len(union_ids)} union cells'
        axis.set(title=label, xlabel='Map x (m)', ylabel='Map y (m)')
    figure.colorbar(shown, ax=axes, shrink=.75, label='Phase fraction / binary indicator')
    figure.suptitle(title + '\nSame prior state and requested pose; phase fraction is not return probability', fontsize=11)
    axes[2].legend(handles=[Line2D([], [], color='black', label='Viable footprint union'),
        Line2D([], [], color='#ed4b3c', marker='x', linestyle='', label='Predicted; no ground'),
        Line2D([], [], color='#d42de8', marker='o', fillstyle='none', linestyle='', label='Ground; not predicted')],
        loc='upper right', fontsize=7)
    figure.savefig(path, dpi=170)
    figure.clear()


def analyze_run(directory, do_phases, pattern_cache, plot_prefix=None):
    data = directory / 'attempt/data'
    events = events_from(data / 'events.jsonl')
    task_path = directory / 'task.json'
    task = json.loads(task_path.read_text()) if task_path.exists() else {}
    complete_events = [e for e in events if e.get('state') == 'A6_TASK_END']
    failures = [e for e in events if e.get('state') == 'FAILED']
    result = dict(run=directory.name, method=task.get('slots', [{}])[0].get('method'),
        agent_version=task.get('runtime_versions', {}).get('agent'),
        task_end_recorded=bool(complete_events), last_event=None if not events else events[-1]['state'],
        first_failure=None if not failures else failures[0], windows=[], prediction_pairs=[])
    snapshots = {}
    for folder in sorted((data / 'rounds').glob('round-*')):
        if not (folder / 'decision.json').exists():
            continue
        try:
            snapshots[int(folder.name.split('-')[1])] = load_state(folder)
        except (OSError, ValueError, EOFError) as exc:
            result.setdefault('pending_snapshot_reads', []).append(dict(path=str(folder), reason=str(exc)))
    obs_events = {e['round']: e for e in events if e.get('state') == 'A5_OBSERVATION'}
    for number, event in sorted(obs_events.items()):
        if number not in snapshots:
            result['windows'].append(dict(window=number, status='OBSERVED_SNAPSHOT_PENDING',
                recorded_point_count=event['point_count'], recorded_chunk_count=event['chunk_count']))
            continue
        after = snapshots[number]
        observation = old.load_npz(data / f'observation_{number:02d}.npz')
        grid, config = after['belief'].grid, after['belief'].config
        sensor = old.SensorModel(**{k: after['ranking']['sensor'][k] for k in
                                    ('T_uav_lidar', 'min_elevation_deg', 'max_elevation_deg')})
        requested = event['requested_viewpoint']
        pose = old.Viewpoint(requested[:3], requested[3])
        points = observation['points_xyz']
        mapped = points @ observation['T_map_sensor'][:3, :3].T + observation['T_map_sensor'][:3, 3]
        ranges = np.linalg.norm(points, axis=1)
        ground_counts = old.counts(old.cell_ids(mapped, grid), grid.width_cells * grid.height_cells,
            (ranges > config.min_range_m) & (ranges < config.max_range_m) &
            (np.abs(mapped[:, 2] - config.ground_z_m) <= config.ground_tolerance_m))
        actual = ground_counts > 0
        presence = after['evidence']['ground_presence_votes'].ravel()
        before_presence = (np.zeros_like(presence) if number == 1 else
                           snapshots[number - 1]['evidence']['ground_presence_votes'].ravel())
        increment = presence - before_presence
        assert np.array_equal(increment, actual.astype(increment.dtype)), 'actual endpoints do not reproduce saved presence increment'
        window = dict(window=number, status='OBSERVATION_AND_PRESENCE_VERIFIED',
            point_count=len(points), retained_ground_points_in_grid=int(ground_counts.sum()),
            actual_ground_cells=int(actual.sum()), chunk_count=len(observation['chunk_point_counts']),
            actual_window_span_s=float(observation['chunk_stamps_s'][-1] - observation['chunk_stamps_s'][0]),
            presence_increment_matches_actual_endpoints=True,
            pose_drift=pose_stats(observation, pose, sensor),
            confirmed_candidate_count=after['decision']['confirmed_candidate_count'])
        result['windows'].append(window)
        if number == 1 or number - 1 not in snapshots:
            continue
        before = snapshots[number - 1]
        candidates = [c for c in before['ranking']['candidates'] if np.allclose(
            [*c['viewpoint']['position_xyz'], c['viewpoint']['yaw_rad']], requested, rtol=0, atol=1e-10)]
        assert len(candidates) == 1, 'executed requested viewpoint must identify one prior ranking candidate'
        candidate = candidates[0]
        index = candidate['candidate_id']
        opportunity = before['fields']['observation_opportunity'][index].ravel()
        assert np.array_equal(opportunity > 0, before['fields']['visibility'][index].ravel())
        acquisition = before['ranking']['acquisition']
        scan = FiniteScan.from_csv(acquisition['scan_path'], window_s=acquisition['window_s'],
            packet_rows=acquisition['packet_rows'], packet_period_s=acquisition['packet_period_s'],
            publisher_sdf_path=acquisition.get('publisher_sdf_path'), sensor=sensor)
        occlusion = build_operational_occlusion(before['belief'], before['operational'],
            assumed_height_m=before['ranking']['config']['assumed_height_m'])
        replay = scan.predict(before['belief'], pose, config=old.NBVConfig(**before['ranking']['config']), occlusion=occlusion)
        assert np.array_equal(replay.opportunity.ravel(), opportunity), 'saved selected opportunity differs from frozen model replay'
        ideal = old.predict_visibility(before['belief'], pose, sensor=sensor,
            config=old.NBVConfig(**before['ranking']['config'])).visible.ravel().astype(float)
        finite_raw = scan.predict(before['belief'], pose,
            config=old.NBVConfig(**before['ranking']['config'])).opportunity.ravel()
        before_viable, after_viable = [viable(s['decision']['assessments']) for s in (before, after)]
        confirmed = [a for a in after['decision']['assessments'] if a['confirmed']]
        scopes = {label: compare(opportunity, actual, ids) for label, ids in (
            ('all_grid', np.arange(len(actual))), ('before_viable_union', union(before_viable)),
            ('after_viable_union', union(after_viable)), ('after_confirmed_union', union(confirmed)))}
        pair = dict(prediction_round=number - 1, observed_window=number, candidate=candidate,
            requested_viewpoint=requested, saved_opportunity_reproduced_exactly=True,
            acquisition=acquisition, scopes=scopes,
            old_center_raw_prism_scopes={label: compare(ideal, actual, ids) for label, ids in (
                ('all_grid', np.arange(len(actual))), ('after_viable_union', union(after_viable)),
                ('after_confirmed_union', union(confirmed)))},
            finite_raw_prism_scopes={label: compare(finite_raw, actual, ids) for label, ids in (
                ('all_grid', np.arange(len(actual))), ('after_viable_union', union(after_viable)),
                ('after_confirmed_union', union(confirmed)))},
            exact_after_viable_footprints=[])
        if plot_prefix is not None and 'hard' in directory.name:
            plot = plot_prefix.with_name(plot_prefix.name + f'_{directory.name}_window{number}.png')
            draw_comparison(plot, grid, union(after_viable), ideal, opportunity, actual,
                            f'{directory.name}, observed window {number}')
            pair['comparison_plot'] = str(plot)
        for a in after_viable:
            ids = np.array(a['operational']['covered_cells'], dtype=int)
            pair['exact_after_viable_footprints'].append(dict(source_id=a['source_id'],
                candidate_id=a['candidate_id'], confirmed=bool(a['confirmed']), cells=len(ids),
                prior_presence_histogram=np.bincount(before_presence[ids], minlength=number + 1).tolist(),
                posterior_presence_histogram=np.bincount(presence[ids], minlength=number + 1).tolist(),
                previously_missing_cells=int(np.count_nonzero(before_presence[ids] < config.free_observations)),
                still_missing_cells=int(np.count_nonzero(presence[ids] < config.free_observations)),
                still_missing_cell_ids=ids[presence[ids] < config.free_observations].tolist(),
                prediction_vs_actual=compare(opportunity, actual, ids)))
        if do_phases:
            if acquisition['scan_path'] not in pattern_cache:
                pattern_cache[acquisition['scan_path']] = old.load_scan_pattern(Path(acquisition['scan_path']))
            phase, nominal, moving = phase_audit(observation, pattern_cache[acquisition['scan_path']], scan.emission_mask,
                old.sensor_transform(pose, sensor), grid, config)
            pair['phase_audit'] = phase
            pair['phase_audit']['scopes'] = {}
            for label, ids in [('all_grid', np.arange(len(actual))), ('after_viable_union', union(after_viable))]:
                pair['phase_audit']['scopes'][label] = dict(
                    nominal_observed_phase_scan_only=compare((nominal > 0).astype(float), actual, ids),
                    moving_observed_phase_scan_only=compare((moving > 0).astype(float), actual, ids),
                    nominal_vs_moving_cell_mask_difference=int(np.count_nonzero(((nominal > 0) != (moving > 0))[ids])))
        result['prediction_pairs'].append(pair)
    return result


def report(result):
    lines = ['# New task acquisition snapshot', '', result['generated_at_utc'], '',
        'Saved phase opportunities are nominal geometric fractions, not calibrated return probabilities. '
        'Only actual recorded endpoints supply the verified presence increments. This is a read-only snapshot; '
        'unfinished tasks and unavailable windows remain pending.', '',
        '| Run | Window | Confirmed candidates | Ground cells | Maximum requested-pose error (m) |',
        '|---|---:|---:|---:|---:|']
    for run in result['runs']:
        for window in run['windows']:
            if 'actual_ground_cells' in window:
                lines.append(f"| {run['run']} | {window['window']} | {window['confirmed_candidate_count']} | "
                    f"{window['actual_ground_cells']} | {window['pose_drift']['position_error_norm_max_m']:.4f} |")
    lines += ['', '| Run / observed window | Union cells | Actual ground | Old center FP/FN | Finite raw-prism FP/FN | Saved finite operational FP/FN |',
              '|---|---:|---:|---:|---:|---:|']
    for run in result['runs']:
        for pair in run['prediction_pairs']:
            s = pair['scopes']['after_viable_union']
            old_s = pair['old_center_raw_prism_scopes']['after_viable_union']
            raw_s = pair['finite_raw_prism_scopes']['after_viable_union']
            lines.append(f"| {run['run']} / {pair['observed_window']} | {s['cells']} | {s['actual_ground_cells']} | "
                f"{old_s['possible_without_ground']}/{old_s['ground_outside_possible']} | "
                f"{raw_s['possible_without_ground']}/{raw_s['ground_outside_possible']} | "
                f"{s['possible_without_ground']}/{s['ground_outside_possible']} |")
    lines += ['', 'Current status:', '']
    for run in result['runs']:
        lines.append(f"- {run['run']}: last event `{run['last_event']}`; task-end record {run['task_end_recorded']}; "
                     f"{len(run['prediction_pairs'])} verified prediction/observation pairs.")
        if run['first_failure']:
            lines.append('  First recorded failure: ' + run['first_failure'].get('reason', str(run['first_failure'])))
        for pair in run['prediction_pairs']:
            if pair.get('comparison_plot'):
                lines.append(f"  [Window {pair['observed_window']} comparison plot]({Path(pair['comparison_plot']).name})")
    lines += ['', 'The accompanying JSON contains exact-footprint presence histograms, missing-cell IDs, '
        'recorded pose ranges, saved-model equality checks and optional packet-phase/moving-ray diagnostics. '
        'No phase opportunity is added to a vote array or treated as an observation.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phases', action='store_true')
    parser.add_argument('--plots', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    markdown = output.with_suffix('.md')
    if output.parent != HERE or not output.name.startswith('new_task_') or output.exists() or markdown.exists():
        parser.error('choose a new new_task_*.json filename directly inside finite-scan-analysis')
    directories = sorted(p for p in (ROOT / 'outputs/development/finite-scan').glob('launch-*')
                         if p.is_dir() and 2 <= int(p.name.split('-')[1]) <= 5)
    cache = {}
    result = dict(kind='new_task_saved_acquisition_vs_actual_ground',
        generated_at_utc=datetime.now(timezone.utc).isoformat(), no_mapper_updates=True,
        no_observations_generated=True, no_gt_geometry_used=True,
        runs=[analyze_run(p, args.phases, cache, output.with_suffix('') if args.plots else None) for p in directories])
    with output.open('x') as target:
        json.dump(result, target, indent=2, allow_nan=False)
        target.write('\n')
    with markdown.open('x') as target:
        target.write(report(result))
    print(json.dumps(dict(output=str(output), report=str(markdown),
        runs=[dict(run=r['run'], windows=len(r['windows']), pairs=len(r['prediction_pairs']),
                   last_event=r['last_event']) for r in result['runs']])))


if __name__ == '__main__':
    main()
