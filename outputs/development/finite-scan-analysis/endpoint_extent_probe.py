#!/usr/bin/env python3
"""Recorded endpoint extents in viable-footprint raw-occupied cells, read-only."""

import argparse
import json
from pathlib import Path

import numpy as np

from finite_scan_probe import HERE, ROOT, RUNS, old
from operational_gating.core import PerceivedTarget


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.resolve().parent != HERE or args.output.exists():
        parser.error('output must be new and directly inside finite-scan-analysis')
    result = dict(kind='recorded_endpoint_extents_in_viable_raw_occupied_cells',
                  no_mapper_updates=True, no_gt_geometry=True, no_observations_generated=True, runs=[])
    for name in RUNS:
        data = ROOT / 'outputs/development/multiscene-paired' / name / 'data'
        rounds = []
        for number in (1, 2):
            folder = data / 'rounds' / f'round-{number:02d}'
            ranking, decision, summary = [json.loads((folder / f'{n}.json').read_text())
                                          for n in ('ranking', 'decision', 'operational_summary')]
            evidence, fields = [old.load_npz(folder / f'{n}.npz') for n in ('operational_evidence', 'fields')]
            target = PerceivedTarget(**summary['target'])
            grid = old.EnvironmentGridSpec(**ranking['a3_summary']['grid'])
            viable = [a for a in decision['assessments'] if not a['representative_blocked']
                      and not a['operational']['blocked'] and not a['footprint_clipped']]
            union = np.unique(np.concatenate([a['operational']['covered_cells'] for a in viable]))
            occupied = union[fields['a2_state'].ravel()[union] == old.EnvironmentState.OCCUPIED]
            points = evidence['ambiguous_endpoint_xy']
            ids = evidence['ambiguous_endpoint_cell_ids']
            mapped = []
            for observation_index in range(number):
                observation = old.load_npz(data / f'observation_{observation_index + 1:02d}.npz')
                transform = observation['T_map_sensor']
                mapped.append(observation['points_xyz'] @ transform[:3, :3].T + transform[:3, 3])
            full_points = np.array([mapped[int(oi)][int(ri)] for oi, ri in
                zip(evidence['ambiguous_endpoint_observation_indices'], evidence['ambiguous_endpoint_row_indices'])])
            assert np.allclose(full_points[:, :2], points, rtol=0, atol=1e-12)
            cells = []
            radius = float(evidence['ambiguous_endpoint_radius_m'])
            for cell in occupied:
                cell = int(cell)
                row, col = divmod(cell, grid.width_cells)
                lo = np.array(grid.origin_xy) + grid.resolution_m * np.array([col, row])
                hi = lo + grid.resolution_m
                selected = points[ids == cell]
                extra = {}
                if len(selected):
                    delta = selected - target.center_xyz[:2]
                    c, s = np.cos(target.yaw_rad), np.sin(target.yaw_rad)
                    local = np.column_stack((c * delta[:, 0] + s * delta[:, 1], -s * delta[:, 0] + c * delta[:, 1]))
                    outside = np.maximum(np.abs(local) - (np.array(target.size_xyz[:2]) / 2 + target.geometry_allowance_m), 0)
                    distances = np.linalg.norm(outside, axis=1)
                    bboxlo, bboxhi = selected.min(axis=0) - radius, selected.max(axis=0) + radius
                    clipped = np.maximum(np.minimum(hi, bboxhi) - np.maximum(lo, bboxlo), 0)
                    z = full_points[ids == cell, 2]
                    extra = dict(ambiguous_xy_min=selected.min(axis=0).tolist(),
                        ambiguous_xy_max=selected.max(axis=0).tolist(),
                        bbox_including_radius_min=bboxlo.tolist(), bbox_including_radius_max=bboxhi.tolist(),
                        bbox_fraction_of_cell=float(np.prod(clipped) / grid.resolution_m ** 2),
                        point_z_minmax_m=[float(z.min()), float(z.max())],
                        ambiguous_center_distance_to_expanded_target_obb_minmax_m=[float(distances.min()), float(distances.max())],
                        ambiguous_disks_intersecting_expanded_target_obb=int(np.count_nonzero(distances <= radius)))
                cells.append(dict(cell_id=cell, cell_lower_xy=lo.tolist(), cell_upper_xy=hi.tolist(),
                    ground_presence=int(evidence['ground_presence_votes'].ravel()[cell]),
                    evidence={k: int(evidence[k].ravel()[cell]) for k in
                        ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes')},
                    ambiguous_endpoint_count=len(selected),
                    complete_ambiguous_window_votes=int(evidence['ambiguous_endpoint_complete_vote_counts'].ravel()[cell]),
                    viable_source_ids=[a['source_id'] for a in viable if cell in a['operational']['covered_cells']], **extra))
            rounds.append(dict(round=number, target=summary['target'], target_vertices_xy=target.xy_vertices.tolist(),
                target_expanded_z_minmax_m=(np.array(target.center_xyz[2]) + np.array([-1, 1]) *
                                           (target.size_xyz[2] / 2 + target.geometry_allowance_m)).tolist(),
                ambiguous_radius_m=radius, ambiguous_profile_available=bool(evidence['ambiguous_endpoint_profile_available']),
                sidecar_matches_actual_retained_rows=True, cells=cells))
        result['runs'].append(dict(run=name, rounds=rounds))
        print(name, json.dumps(rounds), flush=True)
    result['limitations'] = [
        'Bounding boxes enclose existing 33mm endpoint disks, not complete unseen obstacle geometry.',
        'Endpoint height ranges are diagnostic measured heights and do not calibrate a future obstruction-height bound.',
        'The target dimensions and allowance are existing perceived/declared geometry, not scene GT.',
        'The operational gate and all source evidence are unchanged.'
    ]
    with args.output.open('x') as destination:
        json.dump(result, destination, indent=2, allow_nan=False)
        destination.write('\n')


if __name__ == '__main__':
    main()
