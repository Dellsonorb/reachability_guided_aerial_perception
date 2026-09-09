#!/usr/bin/env python3
"""Offline Hard NBV/ground diagnosis using recorded endpoints and public TF only.

This produces no observations or votes. Ray extensions are diagnostic witnesses,
not inferred FREE space, ground support, or reconstructed scene geometry.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from environment_belief import BeliefConfig, EnvironmentBeliefGrid, EnvironmentGridSpec, EnvironmentState
from reachability_guided_nbv.geometry import (
    ground_targets, occupied_boxes, predict_visibility, segments_intersect_box,
    sensor_transform,
)
from reachability_guided_nbv.model import NBVConfig, SensorModel, Viewpoint


RUNS = ('launch-10-hard-generic', 'launch-12-hard-ours')
BELIEF_NAMES = ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score')
SCAN_SHA256 = 'aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a'


def load_npz(path):
    with np.load(path, allow_pickle=False) as saved:
        return {key: saved[key].copy() for key in saved.files}


def cell_ids(points, grid):
    """A2's exact half-open XY convention; -1 denotes outside the grid."""
    x, y = np.asarray(points)[:, :2].T
    xe = grid.origin_xy[0] + np.arange(grid.width_cells + 1) * grid.resolution_m
    ye = grid.origin_xy[1] + np.arange(grid.height_cells + 1) * grid.resolution_m
    cols, rows = np.searchsorted(xe, x, side='right') - 1, np.searchsorted(ye, y, side='right') - 1
    inside = (cols >= 0) & (cols < grid.width_cells) & (rows >= 0) & (rows < grid.height_cells)
    return np.where(inside, rows * grid.width_cells + cols, -1)


def counts(ids, size, selected=None):
    ids = ids if selected is None else ids[selected]
    return np.bincount(ids[ids >= 0], minlength=size)


def fov_geometry(targets, transform, config, sensor):
    relative = (targets - transform[:3, 3]) @ transform[:3, :3]
    distance = np.linalg.norm(relative, axis=1)
    elevation = np.degrees(np.arctan2(relative[:, 2], np.hypot(relative[:, 0], relative[:, 1])))
    in_range = (distance > config.min_range_m) & (distance < config.max_range_m)
    in_fov = (elevation >= sensor.min_elevation_deg) & (elevation <= sensor.max_elevation_deg)
    return distance, elevation, in_range, in_fov


def load_scan_pattern(path):
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != SCAN_SHA256:
        raise ValueError('scan pattern must match the frozen SIM asset manifest')
    values = np.loadtxt(path, delimiter=',', skiprows=1)
    assert values.shape == (800000, 3)
    azimuth, elevation = values[:, 1] % 360, 90 - values[:, 2]
    az, el = np.radians(azimuth), np.radians(elevation)
    directions = np.column_stack((np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)))
    return dict(path=str(path), sha256=digest, azimuth=azimuth, elevation=elevation, directions=directions)


def infer_packet_phase(points, pattern):
    """Identify the unique 10000-row packet block, then verify EVERY return.

    Phase is inferred from observed directions, not timestamps or a new scan.
    Both original runtime logs record samples=10000, downsample=1, size=800000.
    """
    az = np.degrees(np.arctan2(points[:, 1], points[:, 0])) % 360
    az[np.isclose(az, 360., rtol=0, atol=1e-8)] = 0.
    el = np.degrees(np.arctan2(points[:, 2], np.hypot(points[:, 0], points[:, 1])))
    candidates = set(range(80))
    for index in (0, len(points) // 3, 2 * len(points) // 3, len(points) - 1):
        matches = np.flatnonzero((np.abs(pattern['azimuth'] - az[index]) < 1e-4)
                                 & (np.abs(pattern['elevation'] - el[index]) < 1e-4))
        candidates &= set(matches // 10000)
    if len(candidates) != 1:
        raise ValueError('recorded directions do not uniquely identify the packet phase')
    start = int(candidates.pop()) * 10000
    order = np.argsort(pattern['azimuth'][start:start + 10000]) + start
    sorted_az = pattern['azimuth'][order]
    lo, hi = np.searchsorted(sorted_az, az - 1e-4), np.searchsorted(sorted_az, az + 1e-4, side='right')
    maximum = 0.
    for row, (left, right) in enumerate(zip(lo, hi)):
        if left == right:
            raise ValueError('recorded ray azimuth does not match inferred packet')
        ids = order[left:right]
        error = np.maximum(np.abs(pattern['azimuth'][ids] - az[row]),
                           np.abs(pattern['elevation'][ids] - el[row]))
        maximum = max(maximum, float(error.min()))
    if maximum >= 1e-4:
        raise ValueError('recorded ray elevation does not match inferred packet')
    return start, maximum


def scheduled_ray_opportunities(directions, matrix, grid, config):
    """Diagnostic angle opportunities; neither a return simulation nor votes.

    A ray's ground-height-band segment is conservatively bounded in XY. Zero
    opportunities excludes every possible endpoint in that band for the saved
    chunk transform. Positive bbox overlap is not proof of an intersection.
    """
    origin = matrix[:3, 3]
    rays = directions @ matrix[:3, :3].T
    rays = rays[rays[:, 2] < -1e-12]
    t = (config.ground_z_m - origin[2]) / rays[:, 2]
    points = origin + t[:, None] * rays
    inside_range = (t > config.min_range_m) & (t < config.max_range_m)
    plane = counts(cell_ids(points, grid), grid.width_cells * grid.height_cells, inside_range)
    near = np.maximum((config.ground_z_m + config.ground_tolerance_m - origin[2]) / rays[:, 2], config.min_range_m)
    far = np.minimum((config.ground_z_m - config.ground_tolerance_m - origin[2]) / rays[:, 2], config.max_range_m)
    ends_a, ends_b = origin + near[:, None] * rays, origin + far[:, None] * rays
    xylo, xyhi = np.minimum(ends_a[:, :2], ends_b[:, :2]), np.maximum(ends_a[:, :2], ends_b[:, :2])
    low = np.floor((xylo - grid.origin_xy) / grid.resolution_m).astype(int)
    high = np.floor((xyhi - grid.origin_xy) / grid.resolution_m).astype(int)
    eligible = (near <= far) & np.all(high >= 0, axis=1) & np.all(low < [grid.width_cells, grid.height_cells], axis=1)
    low = np.maximum(low[eligible], 0)
    high = np.minimum(high[eligible], [grid.width_cells - 1, grid.height_cells - 1])
    bbox = np.zeros(grid.shape, dtype=int)
    for lo, hi in zip(low, high):
        bbox[lo[1]:hi[1] + 1, lo[0]:hi[0] + 1] += 1
    return plane, bbox.ravel()


def observation_details(saved, grid, config, sensor, targets, prior_belief=None, pattern=None):
    points = saved['points_xyz']
    transform = saved['T_map_sensor']
    mapped = points @ transform[:3, :3].T + transform[:3, 3]
    ranges = np.hypot.reduce(points, axis=1)
    accepted = np.all(np.isfinite(points), axis=1) & (ranges > config.min_range_m) & (ranges < config.max_range_m)
    ids = cell_ids(mapped, grid)
    height = mapped[:, 2] - config.ground_z_m
    ground = accepted & (np.abs(height) <= config.ground_tolerance_m)
    size = len(targets)
    out = dict(ground_points=counts(ids, size, ground), all_endpoint_points=counts(ids, size, accepted),
               rejected_height_points=counts(ids, size, accepted & ~ground),
               ground_height_band_points=counts(ids, size, accepted & (height > config.ground_tolerance_m)
                                                & (height < config.obstacle_min_height_m)))
    chunk_counts, matrices = saved['chunk_point_counts'], saved['chunk_T_map_sensor']
    assert int(chunk_counts.sum()) == len(points) and len(chunk_counts) == len(matrices)
    assert np.array_equal(matrices[-1], transform)
    fov, in_ranges, distances, elevations, azimuths, center_vis, center_occ = [], [], [], [], [], [], []
    projected_foreground = np.zeros(size, dtype=int)
    projected_any = np.zeros(size, dtype=int)
    projected_ground = np.zeros(size, dtype=int)
    projected_rejected_height = np.zeros(size, dtype=int)
    projection_cell_difference = 0
    projection_reconstructed_error = 0.
    start = 0
    lower, upper = (occupied_boxes(prior_belief, 1.) if prior_belief is not None else ([], []))
    witness_examples = {}
    phases, angular_errors = [], []
    scheduled_plane, scheduled_band_bbox = np.zeros(size, dtype=int), np.zeros(size, dtype=int)
    for ci, (count, matrix) in enumerate(zip(chunk_counts, matrices)):
        stop = start + int(count)
        block = mapped[start:stop]
        origin = matrix[:3, 3]
        original_points = (block - origin) @ matrix[:3, :3]
        roundtrip = original_points @ matrix[:3, :3].T + origin
        projection_reconstructed_error = max(projection_reconstructed_error, float(np.max(np.abs(roundtrip - block))))
        if pattern is not None:
            phase, error = infer_packet_phase(original_points, pattern)
            phases.append(phase)
            angular_errors.append(error)
            plane, band = scheduled_ray_opportunities(pattern['directions'][phase:phase + 10000], matrix, grid, config)
            scheduled_plane += plane
            scheduled_band_bbox += band
        distance, elevation, within_range, within_fov = fov_geometry(targets, matrix, config, sensor)
        relative = (targets - origin) @ matrix[:3, :3]
        azimuths.append(np.degrees(np.arctan2(relative[:, 1], relative[:, 0])))
        distances.append(distance)
        in_ranges.append(within_range)
        fov.append(within_range & within_fov)
        elevations.append(elevation)
        occluded = np.zeros(size, dtype=bool)
        if prior_belief is not None:
            for lo, hi in zip(lower, upper):
                occluded |= segments_intersect_box(origin, targets, lo, hi)
            center_vis.append(within_range & within_fov & ~occluded
                              & (prior_belief.state.ravel() != EnvironmentState.OCCUPIED))
            center_occ.append(occluded)
        # Each retained endpoint retains its actual chunk's ray origin. Extend
        # downward rays to the declared A2 ground plane; this is never a vote.
        direction = block - origin
        down = direction[:, 2] < -1e-12
        scale = np.zeros(len(block))
        scale[down] = (config.ground_z_m - origin[2]) / direction[down, 2]
        projected = origin + scale[:, None] * direction
        projected_ranges = np.linalg.norm(projected - origin, axis=1)
        eligible = (accepted[start:stop] & down & (scale > 0)
                    & (projected_ranges > config.min_range_m) & (projected_ranges < config.max_range_m))
        plane_ids = cell_ids(projected, grid)
        projected_any += counts(plane_ids, size, eligible)
        foreground = eligible & (height[start:stop] >= config.obstacle_min_height_m) & (scale > 1)
        projected_foreground += counts(plane_ids, size, foreground)
        projected_ground += counts(plane_ids, size, eligible & ground[start:stop])
        projected_rejected_height += counts(plane_ids, size, eligible & ~ground[start:stop])
        projection_cell_difference += int(np.count_nonzero(eligible & ground[start:stop]
                                                           & (plane_ids != ids[start:stop])))
        for row in np.flatnonzero(foreground & (plane_ids >= 0)):
            cell = int(plane_ids[row])
            if cell not in witness_examples:
                witness_examples[cell] = dict(observation_row=start + int(row), chunk_index=ci,
                    sensor_origin_map=origin.tolist(), measured_endpoint_map=block[row].tolist(),
                    projected_ground_map=projected[row].tolist(), endpoint_height_m=float(height[start + row]))
        start = stop
    out.update(center_range_chunk_count=np.sum(in_ranges, axis=0), center_fov_chunk_count=np.sum(fov, axis=0),
               center_range_min_m=np.min(distances, axis=0), center_range_max_m=np.max(distances, axis=0),
               center_azimuth_min_deg=np.min(azimuths, axis=0), center_azimuth_max_deg=np.max(azimuths, axis=0),
               center_elevation_min_deg=np.min(elevations, axis=0), center_elevation_max_deg=np.max(elevations, axis=0),
               foreground_ray_points=projected_foreground, retained_ray_ground_plane_points=projected_any,
               projected_ground_ray_points=projected_ground, projected_rejected_height_ray_points=projected_rejected_height,
               prior_model_visible_chunk_count=None if not center_vis else np.sum(center_vis, axis=0),
               prior_model_occluded_chunk_count=None if not center_occ else np.sum(center_occ, axis=0),
               witness_examples=witness_examples)
    if pattern is not None:
        out.update(scheduled_ground_plane_rays=scheduled_plane, scheduled_ground_band_bbox_rays=scheduled_band_bbox)
    packet_rpy = []
    for matrix in matrices:
        base = matrix @ np.linalg.inv(sensor.T_uav_lidar)
        r = base[:3, :3]
        packet_rpy.append([np.degrees(np.arctan2(r[2, 1], r[2, 2])),
                           np.degrees(np.arcsin(-r[2, 0])), np.degrees(np.arctan2(r[1, 0], r[0, 0]))])
    out['summary'] = dict(point_count=len(points), accepted_ground_point_count=int(ground.sum()),
        grid_ground_point_count=int(out['ground_points'].sum()), grid_ground_cells=int(np.count_nonzero(out['ground_points'])),
        rejected_range_points=int(np.count_nonzero(~accepted)), chunk_count=len(matrices),
        window_duration_s=float(saved['chunk_stamps_s'][-1] - saved['chunk_stamps_s'][0]),
        accepted_grid_ground_height_quantiles_m=np.quantile(height[ground & (ids >= 0)], [0, .01, .5, .99, 1]).tolist(),
        ground_endpoint_vs_plane_projection_different_cell_points=projection_cell_difference,
        chunk_roundtrip_max_abs_error_m=projection_reconstructed_error,
        observed_base_rpy_min_deg=np.min(packet_rpy, axis=0).tolist(),
        observed_base_rpy_max_deg=np.max(packet_rpy, axis=0).tolist())
    if pattern is not None:
        out['summary'].update(inferred_packet_phase_start_rows=phases,
            all_retained_rays_match_inferred_pattern_packet=True,
            actual_ground_cells_without_scheduled_band_opportunity=int(np.count_nonzero(
                (out['ground_points'] > 0) & (scheduled_band_bbox == 0))),
            packet_pattern_max_angular_error_deg=max(angular_errors),
            pattern_phase_inferred_from_returns_not_recorded_directly=True)
    return out


def describe_run(directory, pattern=None):
    data = directory / 'data'
    snapshots, observations = [], []
    for n in range(1, 4):
        rd = data / 'rounds' / f'round-{n:02d}'
        ranking = json.loads((rd / 'ranking.json').read_text())
        decision = json.loads((rd / 'decision.json').read_text())
        arrays = load_npz(rd / 'fields.npz')
        evidence = load_npz(rd / 'operational_evidence.npz')
        grid = EnvironmentGridSpec(**ranking['a3_summary']['grid'])
        config = BeliefConfig(**ranking['a2_config'])
        belief = EnvironmentBeliefGrid(grid, config, **{key: arrays['a2_' + key] for key in BELIEF_NAMES})
        sensor = SensorModel(**{key: value for key, value in ranking['sensor'].items()
                               if key in ('T_uav_lidar', 'min_elevation_deg', 'max_elevation_deg')})
        snapshots.append(dict(ranking=ranking, decision=decision, arrays=arrays, evidence=evidence, belief=belief))
        observations.append(load_npz(data / f'observation_{n:02d}.npz'))
    targets = ground_targets(belief)
    final_viable = [a for a in snapshots[-1]['decision']['assessments'] if not a['operational']['blocked']]
    union = np.unique(np.concatenate([a['operational']['covered_cells'] for a in final_viable]))
    details, windows, checks = [], [], []
    cumulative = np.zeros(len(targets), dtype=int)
    for n, (observation, snapshot) in enumerate(zip(observations, snapshots), 1):
        prior = None if n == 1 else snapshots[n - 2]
        detail = observation_details(observation, grid, config, sensor, targets,
                                     None if prior is None else prior['belief'], pattern)
        before = cumulative.copy()
        cumulative += detail['ground_points'] > 0
        exact = np.array_equal(cumulative, snapshot['evidence']['ground_presence_votes'].ravel())
        checks.append(exact)
        if pattern is not None:
            checks.append(detail['summary']['actual_ground_cells_without_scheduled_band_opportunity'] == 0)
        predicted = None
        chosen = None
        if prior is not None:
            point = prior['decision']['next_viewpoint']
            matching = [c for c in prior['ranking']['candidates']
                        if np.allclose([*c['viewpoint']['position_xyz'], c['viewpoint']['yaw_rad']], point, rtol=0, atol=1e-12)]
            assert len(matching) == 1
            chosen = matching[0]
            predicted = prior['arrays']['visibility'][chosen['candidate_id']].ravel()
            replay = predict_visibility(prior['belief'], Viewpoint(**chosen['viewpoint']), sensor=sensor,
                                        config=NBVConfig(**prior['ranking']['config']))
            checks.append(bool(np.array_equal(predicted, replay.visible.ravel())))
            detail['planned_center_elevation_deg'] = fov_geometry(
                targets, sensor_transform(Viewpoint(**chosen['viewpoint']), sensor), config, sensor)[1]
        ground = detail['ground_points'] > 0
        summary = dict(round=n, **detail['summary'], cumulative_grid_presence_vote_sum=int(cumulative.sum()),
                       cumulative_grid_supported_cells=int(np.count_nonzero(cumulative >= config.free_observations)),
                       presence_exact_match=exact, final_viable_union_cells=len(union),
                       final_viable_union_ground_cells=int(ground[union].sum()),
                       final_viable_union_supported_cells=int(np.count_nonzero(cumulative[union] >= config.free_observations)))
        if predicted is not None:
            missing = predicted & ~ground
            realized_fov = detail['center_fov_chunk_count']
            realized_vis = detail['prior_model_visible_chunk_count']
            measured_foreground = detail['foreground_ray_points'] > 0
            any_retained_ray = detail['retained_ray_ground_plane_points'] > 0
            summary.update(chosen_candidate=chosen, requested_same_pose=chosen['flight_cost'] == 0,
                predicted_visible_cells=int(predicted.sum()), predicted_with_actual_ground_cells=int((predicted & ground).sum()),
                predicted_without_actual_ground_cells=int(missing.sum()), actual_ground_outside_prediction_cells=int((~predicted & ground).sum()),
                predicted_missing_no_center_fov_at_any_chunk=int(np.count_nonzero(missing & (realized_fov == 0))),
                predicted_missing_model_visible_at_any_chunk=int(np.count_nonzero(missing & (realized_vis > 0))),
                predicted_missing_foreground_ray_witness_cells=int(np.count_nonzero(missing & measured_foreground)),
                predicted_missing_no_retained_ray_intersects_plane_cell=int(np.count_nonzero(missing & ~any_retained_ray)),
                final_viable_union_predicted_cells=int(predicted[union].sum()),
                final_viable_union_predicted_actual_cells=int((predicted & ground)[union].sum()),
                final_viable_union_newly_supported_cells=int(np.count_nonzero((before[union] < 2) & (cumulative[union] >= 2))))
            if pattern is not None:
                summary.update(predicted_missing_no_scheduled_ground_plane_ray_cells=int(np.count_nonzero(
                    missing & (detail['scheduled_ground_plane_rays'] == 0))),
                    predicted_missing_no_scheduled_ground_band_bbox_ray_cells=int(np.count_nonzero(
                        missing & (detail['scheduled_ground_band_bbox_rays'] == 0))))
        detail.update(predicted=predicted, cumulative=cumulative.copy())
        windows.append(summary)
        details.append(detail)
    winners = []
    for winner in final_viable:
        ids = np.asarray(winner['operational']['covered_cells'])
        final_missing = ids[cumulative[ids] < config.free_observations]
        cells = []
        for cell in final_missing:
            rows = []
            for detail in details:
                row = {key: int(detail[key][cell]) for key in ('ground_points', 'all_endpoint_points',
                    'rejected_height_points', 'ground_height_band_points', 'center_range_chunk_count',
                    'center_fov_chunk_count', 'foreground_ray_points', 'retained_ray_ground_plane_points',
                    'projected_ground_ray_points', 'projected_rejected_height_ray_points')}
                row.update(center_elevation_min_deg=float(detail['center_elevation_min_deg'][cell]),
                           center_elevation_max_deg=float(detail['center_elevation_max_deg'][cell]),
                           center_range_min_m=float(detail['center_range_min_m'][cell]),
                           center_range_max_m=float(detail['center_range_max_m'][cell]),
                           center_azimuth_min_deg=float(detail['center_azimuth_min_deg'][cell]),
                           center_azimuth_max_deg=float(detail['center_azimuth_max_deg'][cell]),
                           planned_center_elevation_deg=None if detail['predicted'] is None
                               else float(detail['planned_center_elevation_deg'][cell]),
                           cumulative_presence=int(detail['cumulative'][cell]),
                           prior_prediction_visible=None if detail['predicted'] is None else bool(detail['predicted'][cell]),
                           prior_model_visible_chunks=None if detail['prior_model_visible_chunk_count'] is None
                               else int(detail['prior_model_visible_chunk_count'][cell]),
                           measured_foreground_example=detail['witness_examples'].get(int(cell)))
                if pattern is not None:
                    row.update(scheduled_ground_plane_rays=int(detail['scheduled_ground_plane_rays'][cell]),
                               scheduled_ground_band_bbox_rays=int(detail['scheduled_ground_band_bbox_rays'][cell]))
                rows.append(row)
            cells.append(dict(cell_id=int(cell), ground_center_map=targets[cell].tolist(), windows=rows))
        per_window = []
        for detail in details:
            supported = detail['cumulative'][ids] >= config.free_observations
            pred = detail['predicted']
            per_window.append(dict(ground_cells=int(np.count_nonzero(detail['ground_points'][ids])),
                ground_points=int(detail['ground_points'][ids].sum()), supported_cells=int(supported.sum()),
                missing_cells=int((~supported).sum()),
                predicted_cells=None if pred is None else int(pred[ids].sum()),
                predicted_with_actual_ground_cells=None if pred is None else int(np.count_nonzero(pred[ids] & (detail['ground_points'][ids] > 0))),
                final_missing_with_foreground_witness_cells=int(np.count_nonzero(detail['foreground_ray_points'][final_missing])),
                final_missing_outside_center_fov_all_chunks=int(np.count_nonzero(detail['center_fov_chunk_count'][final_missing] == 0))))
        winners.append(dict(source_id=winner['source_id'], candidate_id=winner['candidate_id'],
            pose_xyyaw=[winner['x'], winner['y'], winner['yaw']], covered_cells=len(ids),
            final_presence_histogram=np.bincount(cumulative[ids], minlength=4).tolist(),
            windows=per_window, final_missing_cells=cells))
    events = [json.loads(line) for line in (data / 'events.jsonl').read_text().splitlines()]
    event_types = ('A5_VIEWPOINT', 'A5_CAPTURE_ANCHOR', 'A5_OBSERVATION')
    return dict(run=directory.name, diagnostic_only=True, source_windows=3, ground_plane_m=config.ground_z_m,
        ground_tolerance_m=config.ground_tolerance_m, ground_support_required_windows=config.free_observations,
        all_reproduction_checks_pass=all(checks), grid=dict(origin_xy=grid.origin_xy, shape=grid.shape, resolution_m=grid.resolution_m),
        windows=windows, exact_final_viable_winners=winners,
        execution_events=[e for e in events if e.get('state') in event_types])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--scan-pattern', type=Path, help='optional installed frozen MID360 CSV; hash is checked')
    args = parser.parse_args()
    source, output = args.source_dir.resolve(), args.output_dir.resolve()
    if output.exists() or output == source or source in output.parents:
        parser.error('output must be new and outside the historical source batch')
    pattern = None if args.scan_pattern is None else load_scan_pattern(args.scan_pattern.resolve())
    runs = [describe_run(source / name, pattern) for name in RUNS]
    result = dict(kind='hard_recorded_nbv_ground_diagnosis', no_gt_geometry=True, no_new_observations=True,
        no_votes_added=True, all_reproduction_checks_pass=all(run['all_reproduction_checks_pass'] for run in runs),
        scan_pattern=None if pattern is None else dict(path=pattern['path'], sha256=pattern['sha256'], rows=800000,
            packet_rows=10000, downsample=1, schedule_use='angle-opportunity diagnostics only, never ground evidence',
            angle_mapping='azimuth=CSV column 2; elevation=90deg-CSV column 3',
            azimuth_band_min_elevation_deg={str([lo, hi]): float(pattern['elevation'][
                ((pattern['azimuth'] + 180) % 360 - 180 >= lo)
                & ((pattern['azimuth'] + 180) % 360 - 180 < hi)].min())
                for lo, hi in [(-30, -20), (-20, -10), (-10, 0), (0, 10), (10, 20), (20, 30)]}),
        limitations=[
            'Ground-plane ray extension is diagnostic only and does not establish ground existence or full-cell occlusion.',
            'Unreturned points are absent. If the optional frozen scan pattern is supplied, packet ray directions are inferred and verified against every retained return; they do not add observations or votes.',
            'Pattern packet phase is inferred from endpoint directions, not logged directly; per-chunk pose applies to all packet rays, with no within-packet deskew.',
            'Ground-height-band opportunities use conservative XY segment bounding boxes; positive overlap does not prove an actual ray-cell intersection or return.',
            'Center FOV is not full-cell visibility; accepted endpoints can occur in cells with out-of-FOV centers.',
            'Per-chunk transforms are recorded public TF; no within-packet timestamps/deskew or calibrated TF error are available.',
            'The 1 m occupied-cell prisms are the existing A4 surrogate, not reconstructed or ground-truth geometry.',
            'Only selected views followed by a recorded window are prediction/measurement pairs; no fourth-window outcome is imputed.'
        ], runs=runs)
    output.mkdir(parents=True)
    (output / 'diagnostic.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(output=str(output), all_reproduction_checks_pass=result['all_reproduction_checks_pass'],
                         runs=[dict(run=r['run'], windows=len(r['windows']),
                                    viable_final_winners=len(r['exact_final_viable_winners'])) for r in runs])))
    return 0 if result['all_reproduction_checks_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
