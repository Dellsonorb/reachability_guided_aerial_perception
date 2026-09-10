#!/usr/bin/env python3
"""Actual scheduled-ray / known-prism opportunity; detached from runtime evidence."""

import argparse
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from finite_scan_probe import HERE, ROOT, RUNS, PATTERN, LevelHoverCache, old, phase_probability, errors, probability_errors


def cache_endpoints(cache, pose, grid, config):
    matrix = old.sensor_transform(pose, cache.sensor)
    origin = matrix[:3, 3]
    height = origin[2] - config.ground_z_m
    travel = height * cache.distance_scale
    valid = (travel > config.min_range_m) & (travel < config.max_range_m)
    slope = cache.slopes[valid]
    c, s = np.cos(pose.yaw_rad), np.sin(pose.yaw_rad)
    xy = height * np.column_stack((c * slope[:, 0] - s * slope[:, 1],
                                  s * slope[:, 0] + c * slope[:, 1])) + origin[:2]
    cells = old.cell_ids(xy, grid)
    inside = cells >= 0
    endpoints = np.column_stack((xy[inside], np.full(inside.sum(), config.ground_z_m)))
    return origin, endpoints, cells[inside], cache.packet[valid][inside]


def filter_prisms(origin, endpoints, cells, packets, belief, assumed_height=1):
    lower, upper = old.occupied_boxes(belief, assumed_height)
    if origin[2] <= belief.config.ground_z_m or np.any(np.all((origin >= lower) & (origin <= upper), axis=1)):
        return np.zeros((80, belief.grid.width_cells * belief.grid.height_cells), dtype=int), len(endpoints), 0
    valid = belief.state.ravel()[cells] != old.EnvironmentState.OCCUPIED
    endpoints, cells, packets = endpoints[valid], cells[valid], packets[valid]
    active = np.ones(len(cells), dtype=bool)
    # Exact existing slab helper, with cheap XY bounding-box rejection first.
    segment_lo, segment_hi = np.minimum(endpoints, origin), np.maximum(endpoints, origin)
    for lo, hi in zip(lower, upper):
        possible = active & np.all(segment_lo <= hi, axis=1) & np.all(segment_hi >= lo, axis=1)
        indices = np.flatnonzero(possible)
        active[indices] &= ~old.segments_intersect_box(origin, endpoints[indices], lo, hi)
    size = belief.grid.width_cells * belief.grid.height_cells
    counts = np.bincount(packets[active] * size + cells[active], minlength=80 * size).reshape(80, size)
    return counts, len(endpoints), int(active.sum())


def run_model(cache, pose, belief, count=51):
    origin, endpoints, cells, packets = cache_endpoints(cache, pose, belief.grid, belief.config)
    counts, considered, unblocked = filter_prisms(origin, endpoints, cells, packets, belief)
    probability, windows = phase_probability(counts, count)
    return probability, counts, dict(grid_rays=considered, unblocked_grid_rays=unblocked)


def moving_known(pattern, observation, phases, belief):
    counts = np.zeros(belief.grid.width_cells * belief.grid.height_cells, dtype=int)
    for matrix, phase in zip(observation['chunk_T_map_sensor'], phases):
        origin = matrix[:3, 3]
        directions = pattern['directions'][phase * 10000:(phase + 1) * 10000]
        emitted = pattern['emitted'][phase * 10000:(phase + 1) * 10000]
        rays = directions[emitted] @ matrix[:3, :3].T
        rays = rays[rays[:, 2] < -1e-12]
        travel = (belief.config.ground_z_m - origin[2]) / rays[:, 2]
        ends = origin + travel[:, None] * rays
        cells = old.cell_ids(ends, belief.grid)
        valid = (travel > belief.config.min_range_m) & (travel < belief.config.max_range_m) & (cells >= 0)
        packet_counts, _, _ = filter_prisms(origin, ends[valid], cells[valid], np.zeros(valid.sum(), dtype=int), belief)
        counts += packet_counts.sum(axis=0)
    return counts


def pose_stats(observation, pose, sensor):
    body = observation['chunk_T_map_sensor'] @ np.linalg.inv(sensor.T_uav_lidar)
    rotation = body[:, :3, :3]
    rpy = np.degrees(np.column_stack((np.arctan2(rotation[:, 2, 1], rotation[:, 2, 2]),
                                      np.arcsin(-rotation[:, 2, 0]),
                                      np.arctan2(rotation[:, 1, 0], rotation[:, 0, 0]))))
    offsets = body[:, :3, 3] - pose.position_xyz
    return dict(planned_xyz=list(pose.position_xyz), recorded_first_xyz=body[0, :3, 3].tolist(),
        recorded_last_xyz=body[-1, :3, 3].tolist(), xyz_offset_min_m=offsets.min(axis=0).tolist(),
        xyz_offset_max_m=offsets.max(axis=0).tolist(),
        position_error_norm_max_m=float(np.linalg.norm(offsets, axis=1).max()),
        roll_pitch_yaw_min_deg=rpy.min(axis=0).tolist(), roll_pitch_yaw_max_deg=rpy.max(axis=0).tolist())


def completion_inventory(masks, ground, viable):
    # Deliberately optimistic arithmetic upper bound for a frozen first state.
    # Does not create votes or assign independent probabilities to whole footprints.
    completed = np.zeros((len(masks), len(masks)), dtype=int)
    best_missing = np.full_like(completed, len(ground))
    for assessment in viable:
        ids = assessment['operational']['covered_cells']
        view = masks[:, ids].astype(np.int8)
        missing = np.count_nonzero(ground[ids] + view[:, None, :] + view[None, :, :] < 2, axis=-1)
        completed += missing == 0
        best_missing = np.minimum(best_missing, missing)
    return dict(ordered_pair_count=len(masks) ** 2, pairs_with_any_complete_footprint=int(np.count_nonzero(completed)),
                maximum_completed_footprints=int(completed.max()), minimum_missing_cells=int(best_missing.min()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--midpoints', action='store_true')
    args = parser.parse_args()
    if args.output.resolve().parent != HERE or args.output.exists():
        parser.error('output must be new and directly inside finite-scan-analysis')
    baseline = json.loads((HERE / 'finite-scan-probe.json').read_text())
    pattern = old.load_scan_pattern(PATTERN)
    raw = np.loadtxt(PATTERN, delimiter=',', skiprows=1)
    # Gazebo plugin calls roundf; all ratios here are nonnegative except possible boundaries.
    horizontal_ratio = np.float32(np.radians(raw[:, 1]) / (6.2831852 / 99))
    vertical_ratio = np.float32((np.radians(raw[:, 2]) - np.pi / 2 + .925024488) / ((.122173046 + .925024488) / 359))
    hindex = np.floor(horizontal_ratio + .5).astype(int)
    vindex = np.where(vertical_ratio >= 0, np.floor(vertical_ratio + .5), np.ceil(vertical_ratio - .5)).astype(int)
    valid = (hindex >= 0) & (hindex < 100) & (vindex >= 0) & (vindex < 360)
    pattern['emitted'] = valid
    result = dict(kind='exact_ray_known_prism_scan_probe', window_packets=51,
        rationale='capture requires last-first>=5 seconds; at10Hz this requires51consecutive packets',
        no_new_observations=True, no_mapper_updates=True, no_gt_geometry=True,
        csv_index_filter=dict(accepted=int(valid.sum()), rejected=int((~valid).sum()),
                             raw_azimuth_minmax_deg=[float(raw[:, 1].min()), float(raw[:, 1].max())],
                             elevation_minmax_deg=[float(pattern['elevation'].min()), float(pattern['elevation'].max())],
                             horizontal_index_minmax=[int(hindex.min()), int(hindex.max())],
                             vertical_index_minmax=[int(vindex.min()), int(vindex.max())]), runs=[])
    start = perf_counter()
    for name in RUNS:
        root = ROOT / 'outputs/development/multiscene-paired' / name / 'data'
        previous = next(r for r in baseline['runs'] if r['run'] == name)
        snapshots = []
        for number in (1, 2):
            folder = root / 'rounds' / f'round-{number:02d}'
            ranking = json.loads((folder / 'ranking.json').read_text())
            decision = json.loads((folder / 'decision.json').read_text())
            fields, evidence = [old.load_npz(folder / f'{n}.npz') for n in ('fields', 'operational_evidence')]
            grid = old.EnvironmentGridSpec(**ranking['a3_summary']['grid'])
            config = old.BeliefConfig(**ranking['a2_config'])
            belief = old.EnvironmentBeliefGrid(grid, config, **{k: fields['a2_' + k] for k in old.BELIEF_NAMES})
            snapshots.append(dict(ranking=ranking, decision=decision, fields=fields, evidence=evidence, belief=belief))
        prior = snapshots[0]
        sensor = old.SensorModel(**{k: ranking['sensor'][k] for k in ('T_uav_lidar', 'min_elevation_deg', 'max_elevation_deg')})
        cache = LevelHoverCache(pattern['directions'], sensor)
        body_down = (pattern['directions'] @ sensor.T_uav_lidar[:3, :3].T)[:, 2] < -1e-12
        keep = valid[body_down]
        cache.packet, cache.slopes, cache.distance_scale = [array[keep] for array in
            (cache.packet, cache.slopes, cache.distance_scale)]
        viable = [a for a in snapshots[1]['decision']['assessments'] if not a['representative_blocked']
                  and not a['operational']['blocked'] and not a['footprint_clipped']]
        union = np.unique(np.concatenate([a['operational']['covered_cells'] for a in viable]))
        poses = []
        for number in (1, 2):
            observation = old.load_npz(root / f'observation_{number:02d}.npz')
            xyz = previous['windows'][number - 1]['planned_viewpoint']
            pose = old.Viewpoint(xyz[:3], xyz[3])
            poses.append(dict(window=number, **pose_stats(observation, pose, sensor)))
        observation = old.load_npz(root / 'observation_02.npz')
        bodypoints = observation['points_xyz']
        mapped = bodypoints @ observation['T_map_sensor'][:3, :3].T + observation['T_map_sensor'][:3, 3]
        ranges = np.linalg.norm(bodypoints, axis=1)
        actual = old.counts(old.cell_ids(mapped, grid), grid.width_cells * grid.height_cells,
            (ranges > config.min_range_m) & (ranges < config.max_range_m) &
            (np.abs(mapped[:, 2] - config.ground_z_m) <= config.ground_tolerance_m)) > 0
        before = perf_counter()
        probability, counts, ray_info = run_model(cache, pose, prior['belief'])
        selected_time = perf_counter() - before
        p50, _ = phase_probability(counts, 50)
        moving = moving_known(pattern, observation, previous['windows'][1]['recorded_phases'], prior['belief'])
        center_prediction = old.predict_visibility(prior['belief'], pose, sensor=sensor,
                                                   config=old.NBVConfig(**ranking['config']))
        raw_probability, _ = phase_probability(cache.packets(pose, grid, config), 51)
        center_prism_gate = ~center_prediction.occluded.ravel() & (prior['belief'].state.ravel() != old.EnvironmentState.OCCUPIED)
        center_prism_probability = raw_probability * center_prism_gate
        scopes = {}
        for label, ids in [('all_grid', np.arange(len(actual))), ('viable_union', union)]:
            scopes[label] = dict(cells=len(ids), actual=int(actual[ids].sum()),
                models={k: errors(v, actual, ids) for k, v in {
                    'saved_center_fov_prism': center_prediction.visible.ravel(),
                    'scan_center_prism_any_phase': center_prism_probability > 0,
                    'scan_center_prism_every_phase': center_prism_probability == 1,
                    'scan_ray_prism_any_phase': probability > 0,
                    'scan_ray_prism_every_phase': probability == 1,
                    'moving_recorded_ray_prism': moving > 0}.items()},
                scan_ray_prism_probability=probability_errors(probability, actual, ids),
                scan_center_prism_probability=probability_errors(center_prism_probability, actual, ids),
                added_vs_center_prism_actual_ground=int(np.count_nonzero((probability > 0)[ids] & (center_prism_probability == 0)[ids] & actual[ids])),
                added_vs_center_prism_no_actual_ground=int(np.count_nonzero((probability > 0)[ids] & (center_prism_probability == 0)[ids] & ~actual[ids])),
                p50_p51_difference_count=int(np.count_nonzero(p50[ids] != probability[ids])))
        candidate_times = []
        all_masks = []
        candidate_list = [old.Viewpoint(**c['viewpoint']) for c in prior['ranking']['candidates']]
        if args.midpoints:
            from reachability_guided_nbv.model import generate_candidates
            base = candidate_list[0]
            candidate_list = list(generate_candidates(base, sensor=sensor,
                config=old.NBVConfig(**{**ranking['config'], 'xy_offsets_m': [-2, -1, 0, 1, 2]})))
        midpoint_t0 = perf_counter()
        for candidate in candidate_list:
            before = perf_counter()
            p, _, _ = run_model(cache, candidate, prior['belief'])
            candidate_times.append(perf_counter() - before)
            all_masks.append(p > 0)
        benchmark = dict(candidate_count=len(candidate_times), batch_s=perf_counter() - midpoint_t0,
                         seconds_quantiles=np.quantile(candidate_times, [0, .5, .9, 1]).tolist())
        first_viable = [a for a in prior['decision']['assessments'] if not a['representative_blocked']
                        and not a['operational']['blocked'] and not a['footprint_clipped']]
        presence = prior['evidence']['ground_presence_votes'].ravel()
        masks = np.array(all_masks)
        inventory = completion_inventory(masks, presence, first_viable)
        original_indices = [i for i, p in enumerate(candidate_list) if p in
                            [old.Viewpoint(**c['viewpoint']) for c in prior['ranking']['candidates']]]
        original_inventory = completion_inventory(masks[original_indices], presence, first_viable)
        row = dict(run=name, window=2, pose_stats=poses, occupied_prism_count=int(np.count_nonzero(prior['belief'].state == old.EnvironmentState.OCCUPIED)),
                   selected_prediction_s=selected_time, ray_info=ray_info, scopes=scopes, benchmark=benchmark,
                   first_state_original_lattice=original_inventory, first_state_test_lattice=inventory,
                   lattice_offsets_m=[-2, -1, 0, 1, 2] if args.midpoints else [-2, 0, 2],
                   inventory_note='Any-positive phase opportunity only; optimistic frozen pair upper bound; no probability of completion, no actual acquisition or flight feasibility claim')
        result['runs'].append(row)
        print(name, json.dumps(dict(scopes=scopes, benchmark=benchmark,
                                   original_inventory=original_inventory, test_inventory=inventory)), flush=True)
    result['elapsed_s'] = perf_counter() - start
    with args.output.open('x') as destination:
        json.dump(result, destination, indent=2, allow_nan=False)
        destination.write('\n')
    print(json.dumps(dict(output=str(args.output), elapsed_s=result['elapsed_s'])))


if __name__ == '__main__':
    main()
