#!/usr/bin/env python3
"""Read-only Moderate endpoint/rectangle arithmetic, NOT a v1.3 runtime gate.

Replays the existing association and evidence rules. Reports distances to all
retained ambiguous endpoints as well as the old overlapping-cell subset. Does
not choose an uncertainty radius, rewrite a snapshot, or reselect candidates.
"""

import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec, PointCloudObservation
from operational_gating import OccupiedClass, PerceivedTarget, derive_operational_evidence
from operational_gating.association import (
    _interior_mask, associate_returns, geometry_allowance, metric_allowance,
)
from operational_gating.io import _load_reference
from task_relevant_uncertainty.geometry import FootprintSpec, footprint_vertices


def distances(points_xy, xy, yaw):
    """Exact Euclidean point-to-closed-padded-rectangle distance; no gating."""
    delta = np.asarray(points_xy, dtype=float) - xy
    c, s = np.cos(yaw), np.sin(yaw)
    local = delta @ np.array([[c, -s], [s, c]])
    return np.linalg.norm(np.maximum(np.abs(local) - [.52, .39], 0.), axis=1)


def diagnose(data):
    """Replay the recorded first window and return JSON-compatible geometry."""
    data = Path(data)
    directory = data / 'rounds/round-01'
    initial = json.loads((data / 'initial.json').read_text())
    ranking = json.loads((directory / 'ranking.json').read_text())
    decision = json.loads((directory / 'decision.json').read_text())
    grid = EnvironmentGridSpec(**ranking['a3_summary']['grid'])
    config = BeliefConfig(**ranking['a2_config'])
    target = PerceivedTarget(**initial['perceived_target'])
    reference = _load_reference(data / 'target_reference.npz', target)
    target = replace(target, geometry_allowance_m=geometry_allowance(reference, target))
    with np.load(data / 'observation_01.npz', allow_pickle=False) as saved:
        observation = PointCloudObservation(saved['points_xyz'], str(saved['frame_id'].item()),
                                            float(saved['stamp_s']), saved['T_map_sensor'])
        counts = saved['chunk_point_counts'].copy()
        stamps = saved['chunk_stamps_s'].copy()
        matrices = saved['chunk_T_map_sensor'].copy()
    if (counts.sum() != len(observation.points_xyz)
            or len(counts) != len(stamps) or len(counts) != len(matrices)
            or not np.all(np.diff(stamps) > 0)):
        raise ValueError('saved packet counts, stamps and transforms must align with endpoints')
    points = observation.points_xyz @ observation.T_map_sensor[:3, :3].T + observation.T_map_sensor[:3, 3]
    labels = associate_returns(points, target, reference)
    view = derive_operational_evidence(grid, [observation], target, labels=[labels], config=config)
    vote_names = ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes', 'ground_votes')
    with np.load(directory / 'operational_evidence.npz', allow_pickle=False) as saved:
        equal = all(np.array_equal(saved[n], getattr(view, n)) for n in vote_names)
    if not equal:
        raise ValueError('frozen per-cell vote replay must match before interpreting geometry')
    x0, x1, y0, y1 = grid.extent
    ranges = np.linalg.norm(observation.points_xyz, axis=1)
    usable = (observation.valid_return & np.all(np.isfinite(points), axis=1)
              & (ranges > config.min_range_m) & (ranges < config.max_range_m)
              & (points[:, 0] >= x0) & (points[:, 0] < x1)
              & (points[:, 1] >= y0) & (points[:, 1] < y1)
              & (points[:, 2] - config.ground_z_m >= config.obstacle_min_height_m))
    occupied_counts = {label.name: int(np.count_nonzero(usable & (labels == label))) for label in OccupiedClass}
    ids = np.flatnonzero(usable & (labels == OccupiedClass.AMBIGUOUS))
    col = np.searchsorted(x0 + np.arange(grid.width_cells + 1) * grid.resolution_m, points[ids, 0], side='right') - 1
    row = np.searchsorted(y0 + np.arange(grid.height_cells + 1) * grid.resolution_m, points[ids, 1], side='right') - 1
    cell_ids = row * grid.width_cells + col
    chunks = np.searchsorted(np.cumsum(counts), ids, side='right')
    offsets = np.r_[0, np.cumsum(counts)]
    t, k = reference['T_map_color'], reference['color_K']
    optical = (points[ids] - t[:3, 3]) @ t[:3, :3]
    interior = _interior_mask(reference['mask'])
    c, s = np.cos(target.yaw_rad), np.sin(target.yaw_rad)
    local = (points[ids] - target.center_xyz) @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    records = []
    for n, index in enumerate(ids):
        association = dict(inside_expanded_target=bool(target.contains(points[index:index + 1])[0]),
                           target_local_xyz=local[n].tolist(), in_camera=False)
        if optical[n, 2] > 0:
            u, v = np.rint(optical[n, :2] / optical[n, 2] * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]).astype(int)
            h, w = reference['mask'].shape
            if 0 <= u < w and 0 <= v < h:
                z = float(reference['registered_depth_m'][v, u])
                valid = np.isfinite(z) and z > 0
                allowance = float(metric_allowance(reference, optical[n, 2]))
                surface_ok = False
                if valid:
                    surface = np.array([[(u - k[0, 2]) * z / k[0, 0], (v - k[1, 2]) * z / k[1, 1], z]])
                    surface_ok = bool(target.contains(surface @ t[:3, :3].T + t[:3, 3])[0])
                association.update(in_camera=True, pixel_uv=[int(u), int(v)],
                    raw_mask=bool(reference['mask'][v, u]), interior_mask=bool(interior[v, u]),
                    optical_depth_m=float(optical[n, 2]), registered_depth_m=z if valid else None,
                    depth_residual_m=abs(z - float(optical[n, 2])) if valid else None,
                    association_allowance_m=allowance,
                    depth_match=bool(valid and abs(z - optical[n, 2]) <= allowance),
                    backprojected_surface_inside_target=surface_ok)
        records.append(dict(observation_row=int(index), chunk_index=int(chunks[n]),
            retained_row_in_chunk=int(index - offsets[chunks[n]]),
            chunk_stamp_s=float(stamps[chunks[n]]), cell_id=int(cell_ids[n]),
            map_xyz=points[index].tolist(), association=association))
    rows = []
    for candidate in decision['assessments']:
        op = candidate['operational']
        if op['target_collision']:
            continue
        xy, yaw = [candidate['x'], candidate['y']], candidate['yaw']
        d = distances(points[ids, :2], xy, yaw)
        old_subset = np.isin(cell_ids, op['covered_cells'])
        nearest = int(np.argmin(d))
        env_ids = np.flatnonzero(usable & (labels == OccupiedClass.ENVIRONMENT))
        rows.append(dict(candidate_id=candidate['candidate_id'], source_id=candidate['source_id'],
            xy=xy, yaw_rad=yaw, footprint_vertices_xy=footprint_vertices(xy, yaw, FootprintSpec()).tolist(),
            legacy_ambiguous_cells=sorted(set(int(i) for i in cell_ids[old_subset])),
            legacy_ambiguous_endpoint_rows=ids[old_subset].tolist(),
            minimum_legacy_blocking_endpoint_distance_m=float(d[old_subset].min()) if np.any(old_subset) else None,
            minimum_all_ambiguous_endpoint_distance_m=float(d[nearest]),
            nearest_ambiguous_endpoint_row=int(ids[nearest]),
            all_ambiguous_endpoint_distances_m=d.tolist(),
            minimum_environment_endpoint_distance_m=float(distances(points[env_ids, :2], xy, yaw).min()) if len(env_ids) else None,
            environment_cells=op['environment_cells'],
            ground_supported_cells=op['ground_supported_cells'],
            ground_required_cells=len(op['covered_cells']),
            frozen_confirmed=candidate['confirmed']))
    return dict(diagnostic_only=True, selection_changed=False, frozen_vote_replay_equal=equal,
        source_data=str(data), frame_id='map', grid=ranking['a3_summary']['grid'],
        padded_half_dimensions_m=[.52, .39],
        target=dict(center_xyz=target.center_xyz, yaw_rad=target.yaw_rad, size_xyz=target.size_xyz,
                    recorded_geometry_allowance_m=target.geometry_allowance_m,
                    expanded_vertices_xy=target.xy_vertices.tolist()),
        occupied_endpoint_counts=occupied_counts, ambiguous_endpoints=records,
        continuous_target_clear_winners=rows, total_physical_uncertainty_radius_m=None,
        radius_status='NOT_ESTABLISHED_FROM_RECORDED_PUBLIC_TF_SOURCE_TIMING',
        note='Distances are not authorization to unblock. No proposed v1.3 state or score is produced.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source, output = args.data_dir.resolve(), args.output.resolve()
    if output == source.parent or source.parent in output.parents:
        parser.error('write the derived report outside the recorded source attempt')
    report = diagnose(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')


if __name__ == '__main__':
    main()
