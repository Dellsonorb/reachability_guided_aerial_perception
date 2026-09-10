#!/usr/bin/env python3
"""Detached scan-opportunity diagnostics. Never generates support or observations."""

import argparse
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts'),
                str(ROOT / 'outputs/development/task-handoff-analysis')]
import diagnose_hard_nbv_ground as old
from diagnose_opportunities import RUNS
from scan_pattern_diagnosis import PATTERN


def packets_at_transform(directions, matrix, grid, config, packet_ids=None):
    rays = directions @ matrix[:3, :3].T
    good = rays[:, 2] < -1e-12
    ray_ids = np.flatnonzero(good)
    ray = rays[good]
    travel = (config.ground_z_m - matrix[2, 3]) / ray[:, 2]
    points = matrix[:3, 3] + travel[:, None] * ray
    cells = old.cell_ids(points, grid)
    valid = (cells >= 0) & (travel > config.min_range_m) & (travel < config.max_range_m)
    packet = ray_ids // 10000 if packet_ids is None else packet_ids[ray_ids]
    size = grid.width_cells * grid.height_cells
    return np.bincount(packet[valid] * size + cells[valid], minlength=80 * size).reshape(80, size)


class LevelHoverCache:
    """Exact ground-plane slopes for the fixed mount, retaining CSV packet IDs."""

    def __init__(self, directions, sensor):
        rays = directions @ sensor.T_uav_lidar[:3, :3].T
        valid = rays[:, 2] < -1e-12
        self.packet = np.flatnonzero(valid) // 10000
        self.slopes = -rays[valid, :2] / rays[valid, 2, None]
        self.distance_scale = -1 / rays[valid, 2]
        self.sensor = sensor

    def packets(self, viewpoint, grid, config):
        matrix = old.sensor_transform(viewpoint, self.sensor)
        height = matrix[2, 3] - config.ground_z_m
        travel = height * self.distance_scale
        valid = (travel > config.min_range_m) & (travel < config.max_range_m)
        slope = self.slopes[valid]
        c, s = np.cos(viewpoint.yaw_rad), np.sin(viewpoint.yaw_rad)
        xy = height * np.column_stack((c * slope[:, 0] - s * slope[:, 1],
                                      s * slope[:, 0] + c * slope[:, 1])) + matrix[:2, 3]
        cells = old.cell_ids(xy, grid)
        inside = cells >= 0
        size = grid.width_cells * grid.height_cells
        return np.bincount(self.packet[valid][inside] * size + cells[inside],
                           minlength=80 * size).reshape(80, size)


def phase_probability(packet_counts, count=50):
    # Each start uses consecutive packets; no independence assumption or phase oracle.
    twice = np.concatenate((packet_counts, packet_counts), axis=0)
    cumulative = np.concatenate((np.zeros_like(twice[:1]), np.cumsum(twice, axis=0)))
    windows = cumulative[np.arange(80) + count] - cumulative[np.arange(80)]
    return (windows > 0).mean(axis=0), windows


def errors(prediction, ground, ids):
    prediction, ground = prediction[ids], ground[ids]
    return dict(predicted=int(prediction.sum()), actual=int(ground.sum()),
                tp=int(np.count_nonzero(prediction & ground)),
                fp=int(np.count_nonzero(prediction & ~ground)),
                fn=int(np.count_nonzero(~prediction & ground)),
                tn=int(np.count_nonzero(~prediction & ~ground)))


def probability_errors(probability, ground, ids):
    p, y = probability[ids], ground[ids]
    return dict(sum_opportunity=float(p.sum()), brier=float(np.mean((p - y) ** 2)),
                expected_false_positives=float(p[~y].sum()),
                expected_false_negatives=float((1 - p[y]).sum()),
                zero=int(np.count_nonzero(p == 0)), one=int(np.count_nonzero(p == 1)),
                fractional=int(np.count_nonzero((p > 0) & (p < 1))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.resolve().parent != HERE or args.output.exists():
        parser.error('output must be new and directly inside finite-scan-analysis')
    start = perf_counter()
    pattern = old.load_scan_pattern(PATTERN)
    result = dict(kind='finite_window_scan_opportunity_probe', pattern_path=str(PATTERN),
                  pattern_sha256=pattern['sha256'], no_observations_generated=True,
                  no_mapper_updates=True, no_gt_geometry=True, window_packets=50,
                  window_seconds=5, phase_prior='uniform over 80 packet starts', runs=[])
    for name in RUNS:
        root = ROOT / 'outputs/development/multiscene-paired' / name / 'data'
        events = [json.loads(line) for line in (root / 'events.jsonl').read_text().splitlines()]
        obs_events = [e for e in events if e['state'] == 'A5_OBSERVATION']
        snapshots = []
        for number in (1, 2):
            folder = root / 'rounds' / f'round-{number:02d}'
            ranking = json.loads((folder / 'ranking.json').read_text())
            decision = json.loads((folder / 'decision.json').read_text())
            fields = old.load_npz(folder / 'fields.npz')
            grid = old.EnvironmentGridSpec(**ranking['a3_summary']['grid'])
            config = old.BeliefConfig(**ranking['a2_config'])
            belief = old.EnvironmentBeliefGrid(grid, config,
                **{key: fields['a2_' + key] for key in old.BELIEF_NAMES})
            snapshots.append(dict(ranking=ranking, decision=decision, fields=fields, belief=belief))
        sensor = old.SensorModel(**{k: ranking['sensor'][k] for k in
                                    ('T_uav_lidar', 'min_elevation_deg', 'max_elevation_deg')})
        cache = LevelHoverCache(pattern['directions'], sensor)
        viable = [a for a in snapshots[1]['decision']['assessments'] if not a['representative_blocked']
                  and not a['operational']['blocked'] and not a['footprint_clipped']]
        union = np.unique(np.concatenate([a['operational']['covered_cells'] for a in viable]))
        targets = old.ground_targets(belief)
        all_ids = np.arange(len(targets))
        windows = []
        for number in (1, 2):
            observed = old.load_npz(root / f'observation_{number:02d}.npz')
            event = next(e for e in obs_events if e['round'] == number)
            xyz = event['requested_viewpoint']
            pose = old.Viewpoint(xyz[:3], xyz[3])
            matrix = old.sensor_transform(pose, sensor)
            before = perf_counter()
            packet_counts = cache.packets(pose, grid, config)
            probability, phase_windows = phase_probability(packet_counts)
            timing = perf_counter() - before
            direct = packets_at_transform(pattern['directions'], matrix, grid, config)
            assert np.array_equal(packet_counts, direct), 'slope-cache differs from full transform'
            mapped = observed['points_xyz'] @ observed['T_map_sensor'][:3, :3].T + observed['T_map_sensor'][:3, 3]
            ranges = np.linalg.norm(observed['points_xyz'], axis=1)
            accepted = (ranges > config.min_range_m) & (ranges < config.max_range_m)
            ground = old.counts(old.cell_ids(mapped, grid), len(targets), accepted &
                                (np.abs(mapped[:, 2] - config.ground_z_m) <= config.ground_tolerance_m)) > 0
            _, _, inside_range, inside_fov = old.fov_geometry(targets, matrix, config, sensor)
            center = inside_range & inside_fov
            known_visibility = np.ones_like(center)
            if number == 2:
                prior = snapshots[0]
                replay = old.predict_visibility(prior['belief'], pose, sensor=sensor,
                    config=old.NBVConfig(**prior['ranking']['config']))
                selected = next(c for c in prior['ranking']['candidates'] if np.allclose(
                    [*c['viewpoint']['position_xyz'], c['viewpoint']['yaw_rad']], xyz, rtol=0, atol=1e-12))
                assert np.array_equal(replay.visible, prior['fields']['visibility'][selected['candidate_id']])
                known_visibility = ~replay.occluded.ravel() & (prior['belief'].state.ravel() != old.EnvironmentState.OCCUPIED)
            phases, moving_counts, offset = [], np.zeros(len(targets), dtype=np.int64), 0
            for count, transform in zip(observed['chunk_point_counts'], observed['chunk_T_map_sensor']):
                stop = offset + int(count)
                original = (mapped[offset:stop] - transform[:3, 3]) @ transform[:3, :3]
                phase_row, angular_error = old.infer_packet_phase(original, pattern)
                phase = phase_row // 10000
                phases.append(phase)
                moving_counts += packets_at_transform(pattern['directions'][phase_row:phase_row + 10000],
                                                       transform, grid, config).sum(axis=0)
                offset = stop
            phase_steps = np.diff(phases) % 80
            nominal_recorded_phase = packet_counts[phases].sum(axis=0) > 0
            models = {'center_fov': center, 'center_with_known_occlusion': center & known_visibility,
                      'nominal_any_phase': probability > 0, 'nominal_every_phase': probability == 1,
                      'nominal_recorded_phases': nominal_recorded_phase,
                      'moving_recorded_phase': moving_counts > 0,
                      'nominal_every_phase_with_known_occlusion': (probability == 1) & known_visibility,
                      'center_intersect_every_phase': center & (probability == 1),
                      'center_known_intersect_every_phase': center & known_visibility & (probability == 1)}
            scopes = {}
            for label, ids in (('all_grid', all_ids), ('viable_union', union)):
                scopes[label] = dict(cells=len(ids), models={k: errors(v, ground, ids) for k, v in models.items()},
                    phase_averaged=probability_errors(probability, ground, ids),
                    phase_averaged_known_occlusion=probability_errors(probability * known_visibility, ground, ids),
                    phase_averaged_center_known=probability_errors(probability * known_visibility * center, ground, ids),
                    actual_phase_vs_any_phase_cells=int(np.count_nonzero((nominal_recorded_phase != (probability > 0))[ids])),
                    nominal_vs_moving_actual_phase_cells=int(np.count_nonzero((nominal_recorded_phase != (moving_counts > 0))[ids])))
            missing_pred = union[(center & known_visibility & ~ground)[union]]
            summary = dict(window=number, planned_viewpoint=xyz, chunk_count=len(phases),
                phase_start=phases[0], recorded_phases=phases,
                phase_steps_histogram={str(i): int(np.count_nonzero(phase_steps == i)) for i in np.unique(phase_steps)},
                static_cached_prediction_s=timing, cached_equals_direct=True, scopes=scopes,
                nominal_known_visible_missing_union_cells=[dict(cell_id=int(i), probability=float(probability[i]),
                    nominal_cycle_rays=int(packet_counts[:, i].sum()),
                    nominal_recorded_phase_rays=int(packet_counts[phases, i].sum()),
                    moving_recorded_phase_rays=int(moving_counts[i])) for i in missing_pred])
            windows.append(summary)
            print(name, number, json.dumps(scopes['viable_union']), flush=True)
        candidate_times, candidate_packet_counts = [], []
        before = perf_counter()
        for candidate in snapshots[0]['ranking']['candidates']:
            tick = perf_counter()
            packets = cache.packets(old.Viewpoint(**candidate['viewpoint']), grid, config)
            phase_probability(packets)
            candidate_times.append(perf_counter() - tick)
            candidate_packet_counts.append(int(np.count_nonzero(packets)))
        benchmark = dict(candidate_count=len(candidate_times), batch_seconds=perf_counter() - before,
                         seconds_quantiles=np.quantile(candidate_times, [0, .5, .9, 1]).tolist(),
                         downward_cache_rays=len(cache.packet),
                         packet_cell_storage_int64_bytes=80 * len(targets) * 8)
        result['runs'].append(dict(run=name, windows=windows, benchmark=benchmark))
    result['elapsed_s'] = perf_counter() - start
    with args.output.open('x') as destination:
        json.dump(result, destination, indent=2, allow_nan=False)
        destination.write('\n')
    print(json.dumps(dict(output=str(args.output), elapsed_s=result['elapsed_s'])))


if __name__ == '__main__':
    main()
