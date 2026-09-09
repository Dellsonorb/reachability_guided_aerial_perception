#!/usr/bin/env python3
"""Apply the existing scan-pattern diagnostic to eight saved Hard windows.

No mapper updates or runtime policy calls. Generated output must be a new file
inside this analysis directory. Phase-validation failure aborts this analysis.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
import diagnose_hard_nbv_ground as existing
from diagnose_opportunities import RUNS

PATTERN = Path('/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation/'
               'install/p450-clean/share/sim_platform_assets/models/MID360/scan_mode/mid360.csv')


def group(detail, ids, predicted=None):
    ground = detail['ground_points'][ids] > 0
    missing = ~ground
    band = detail['scheduled_ground_band_bbox_rays'][ids] > 0
    foreground = detail['foreground_ray_points'][ids] > 0
    fov = detail['center_fov_chunk_count'][ids] > 0
    result = dict(cells=len(ids), actual_ground_cells=int(ground.sum()), missing_ground_cells=int(missing.sum()),
                  missing_no_scheduled_band_opportunity=int(np.count_nonzero(missing & ~band)),
                  missing_with_scheduled_band_opportunity=int(np.count_nonzero(missing & band)),
                  missing_with_foreground_witness=int(np.count_nonzero(missing & foreground)),
                  missing_with_band_and_foreground=int(np.count_nonzero(missing & band & foreground)),
                  missing_with_band_without_foreground=int(np.count_nonzero(missing & band & ~foreground)),
                  missing_no_realized_center_fov=int(np.count_nonzero(missing & ~fov)))
    if predicted is not None:
        pred = predicted[ids]
        result.update(predicted_cells=int(pred.sum()), predicted_missing=int(np.count_nonzero(pred & missing)),
                      predicted_missing_no_scheduled_band=int(np.count_nonzero(pred & missing & ~band)),
                      predicted_missing_with_foreground=int(np.count_nonzero(pred & missing & foreground)),
                      predicted_missing_no_realized_center_fov=int(np.count_nonzero(pred & missing & ~fov)))
    return result


def cell(detail, index, predicted=None, elevation=None):
    row = {key: int(detail[key][index]) for key in (
        'ground_points', 'all_endpoint_points', 'ground_height_band_points',
        'scheduled_ground_plane_rays', 'scheduled_ground_band_bbox_rays',
        'foreground_ray_points', 'center_fov_chunk_count', 'center_range_chunk_count')}
    row.update(center_elevation_min_deg=float(detail['center_elevation_min_deg'][index]),
               center_elevation_max_deg=float(detail['center_elevation_max_deg'][index]),
               center_azimuth_min_deg=float(detail['center_azimuth_min_deg'][index]),
               center_azimuth_max_deg=float(detail['center_azimuth_max_deg'][index]),
               foreground_example=detail['witness_examples'].get(int(index)))
    if predicted is not None:
        row.update(planned_visible=bool(predicted[index]), planned_center_elevation_deg=float(elevation[index]),
                   prior_model_visible_chunk_count=int(detail['prior_model_visible_chunk_count'][index]))
    return row


def analyze(name, pattern):
    directory = ROOT / 'outputs/development/multiscene-paired' / name
    snapshots = []
    for number in (1, 2):
        p = directory / 'data/rounds' / f'round-{number:02d}'
        ranking, decision = [json.loads((p / f'{key}.json').read_text()) for key in ('ranking', 'decision')]
        arrays, evidence = [existing.load_npz(p / f'{key}.npz') for key in ('fields', 'operational_evidence')]
        grid = existing.EnvironmentGridSpec(**ranking['a3_summary']['grid'])
        config = existing.BeliefConfig(**ranking['a2_config'])
        belief = existing.EnvironmentBeliefGrid(grid, config,
                    **{key: arrays['a2_' + key] for key in existing.BELIEF_NAMES})
        snapshots.append(dict(ranking=ranking, decision=decision, arrays=arrays, evidence=evidence, belief=belief))
    sensor = existing.SensorModel(**{key: snapshots[0]['ranking']['sensor'][key] for key in
                           ('T_uav_lidar', 'min_elevation_deg', 'max_elevation_deg')})
    targets = existing.ground_targets(belief)
    viable = [a for a in snapshots[1]['decision']['assessments'] if not a['representative_blocked']
              and not a['operational']['blocked'] and not a['footprint_clipped']]
    union = np.unique(np.concatenate([a['operational']['covered_cells'] for a in viable]))
    presence = [s['evidence']['ground_presence_votes'].ravel() for s in snapshots]
    zero_after_two = union[presence[1][union] == 0]
    details, windows, cumulative = [], [], np.zeros(len(targets), dtype=int)
    predicted = elevation = None
    for number, snapshot in enumerate(snapshots, 1):
        observation = existing.load_npz(directory / 'data' / f'observation_{number:02d}.npz')
        prior = None if number == 1 else snapshots[0]
        detail = existing.observation_details(observation, grid, config, sensor, targets,
                                             None if prior is None else prior['belief'], pattern)
        cumulative += detail['ground_points'] > 0
        assert np.array_equal(cumulative, presence[number - 1])
        assert detail['summary']['actual_ground_cells_without_scheduled_band_opportunity'] == 0
        if prior is not None:
            point = prior['decision']['next_viewpoint']
            matching = [c for c in prior['ranking']['candidates'] if np.allclose(
                [*c['viewpoint']['position_xyz'], c['viewpoint']['yaw_rad']], point, rtol=0, atol=1e-12)]
            assert len(matching) == 1
            candidate = matching[0]
            predicted = prior['arrays']['visibility'][candidate['candidate_id']].ravel()
            viewpoint = existing.Viewpoint(**candidate['viewpoint'])
            replay = existing.predict_visibility(prior['belief'], viewpoint, sensor=sensor,
                                                  config=existing.NBVConfig(**prior['ranking']['config']))
            assert np.array_equal(predicted, replay.visible.ravel())
            elevation = existing.fov_geometry(targets, existing.sensor_transform(viewpoint, sensor), config, sensor)[1]
        summary = dict(detail['summary'])
        phases = summary.pop('inferred_packet_phase_start_rows')
        summary['unique_packet_phase_count'] = len(set(phases))
        summary['unique_packet_phase_start_rows'] = sorted(set(phases))
        summary.update(round=number, presence_reproduced_exactly=True,
                       viable_union=group(detail, union, predicted),
                       union_zero_presence_after_two=group(detail, zero_after_two, predicted))
        windows.append(summary)
        details.append(detail)
        print(f'{name} window {number}: phase and presence verified; '
              f'{summary["chunk_count"]} chunks, {summary["point_count"]} returns', flush=True)
    winners = []
    for a in viable:
        ids = np.array(a['operational']['covered_cells'])
        zero = ids[presence[1][ids] == 0]
        winners.append(dict(source_id=a['source_id'], candidate_id=a['candidate_id'], cells=len(ids),
            presence_histograms=[np.bincount(g[ids], minlength=3).tolist() for g in presence],
            zero_presence_after_two_cell_ids=zero.tolist(),
            windows=[group(detail, ids, None if index == 0 else predicted) for index, detail in enumerate(details)]))
    cells = [dict(cell_id=int(index), ground_center_map=targets[index].tolist(),
                  windows=[cell(details[0], index), cell(details[1], index, predicted, elevation)])
             for index in zero_after_two]
    return dict(run=name, windows=windows, exact_winners=winners,
                union_zero_presence_after_two_cells=cells)


def trace_positive_band_cells(pattern):
    """Resolve the seven residual positive-band/no-foreground cells, read-only."""
    name, ids = 'launch-06-hard01-ours', (301, 380, 419, 498, 537, 616, 655)
    data = ROOT / 'outputs/development/multiscene-paired' / name / 'data'
    ranking = json.loads((data / 'rounds/round-02/ranking.json').read_text())
    grid = existing.EnvironmentGridSpec(**ranking['a3_summary']['grid'])
    config = existing.BeliefConfig(**ranking['a2_config'])
    saved = existing.load_npz(data / 'observation_02.npz')
    mapped = saved['points_xyz'] @ saved['T_map_sensor'][:3, :3].T + saved['T_map_sensor'][:3, 3]
    traces, start = {str(i): [] for i in ids}, 0
    for ci, (count, matrix) in enumerate(zip(saved['chunk_point_counts'], saved['chunk_T_map_sensor'])):
        stop, origin = start + int(count), matrix[:3, 3]
        original = (mapped[start:stop] - origin) @ matrix[:3, :3]
        phase, _ = existing.infer_packet_phase(original, pattern)
        directions = pattern['directions'][phase:phase + 10000] @ matrix[:3, :3].T
        unit = original / np.linalg.norm(original, axis=1)[:, None]
        for index in ids:
            row, col = divmod(index, grid.width_cells)
            lo = np.array([grid.origin_xy[0] + col * grid.resolution_m,
                           grid.origin_xy[1] + row * grid.resolution_m,
                           config.ground_z_m - config.ground_tolerance_m])
            hi = lo + [grid.resolution_m, grid.resolution_m, 2 * config.ground_tolerance_m]
            assert np.linalg.norm(np.maximum(np.maximum(lo - origin, origin - hi), 0)) > config.min_range_m
            hits = existing.segments_intersect_box(origin, origin + config.max_range_m * directions, lo, hi)
            for ray in np.flatnonzero(hits):
                matching = np.flatnonzero(np.linalg.norm(unit - pattern['directions'][phase + ray], axis=1) < 1e-6)
                assert len(matching) <= 1
                endpoint = None if len(matching) == 0 else mapped[start + matching[0]]
                trace = dict(chunk=ci, pattern_row=int(phase + ray),
                             matched_retained_row=None if endpoint is None else int(start + matching[0]),
                             measured_endpoint_map=None if endpoint is None else endpoint.tolist(),
                             measured_cell=None if endpoint is None else int(existing.cell_ids(endpoint[None, :], grid)[0]),
                             measured_ground=None if endpoint is None else bool(abs(endpoint[2] - config.ground_z_m) <= config.ground_tolerance_m))
                traces[str(index)].append(trace)
        start = stop
    return dict(run=name, window=2, diagnostic_only=True, no_new_observations=True,
                note='Closed ray/cell accepted-height-prism intersection using existing slab helper; '
                     'matched endpoints are existing retained rows, not generated returns.', traces=traces)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trace-positive-band', action='store_true',
                        help='Only trace the seven residual Hard01 Ours window-two band cases')
    args = parser.parse_args()
    output = args.output.resolve()
    if output.parent != HERE or output.exists():
        parser.error('output must be a new file directly inside this analysis directory')
    pattern = existing.load_scan_pattern(PATTERN)
    if args.trace_positive_band:
        result = trace_positive_band_cells(pattern)
        with output.open('x') as destination:
            json.dump(result, destination, indent=2, allow_nan=False)
            destination.write('\n')
        print(json.dumps(dict(output=str(output), cells=len(result['traces']))))
        return
    results = [analyze(name, pattern) for name in RUNS]
    result = dict(kind='recorded_hard_first_two_windows_scan_pattern_diagnostic',
        existing_implementation='scripts/diagnose_hard_nbv_ground.py',
        scan_pattern=str(PATTERN), existing_reader_hash_check_passed=True,
        no_new_observations=True, no_mapper_updates=True, no_gt_geometry=True,
        limitations=[
            'Phase is inferred from observed directions and verified against every retained return; it is not directly logged.',
            'Scheduled rays use saved per-packet public TF and no within-packet deskew. Ground-band counts use conservative XY segment bounding boxes.',
            'Zero band opportunity excludes a ray reaching this cell in the accepted height band under this recorded geometry model; it is not geometric truth.',
            'Positive band counts do not establish intersection, an unobstructed path, a ground return or support.',
            'Foreground witnesses are actual endpoints projected along their measured rays, not ground observations or full-cell occlusion proofs.',
            'Recorded finite packets do not establish coverage for longer unseen windows or hypothetical poses. No density threshold or score is tuned.',
            'Per-group foreground, FOV and band counts can overlap. No-foreground-witness does not prove no occlusion.',
        ], runs=results)
    with output.open('x') as destination:
        json.dump(result, destination, indent=2, allow_nan=False)
        destination.write('\n')
    print(json.dumps(dict(output=str(output), runs=len(results), verified_windows=8)))


if __name__ == '__main__':
    main()
