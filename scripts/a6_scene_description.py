#!/usr/bin/env python3
"""Describe saved A6 first windows against proposal targets after scene freeze.

Offline manipulation checks only: no scene selection, trial filtering, reruns,
policy changes, simulator access, or new A1 queries. Target mismatches are data.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from environment_belief import EnvironmentGridSpec, EnvironmentState
from reachability_guided_aerial_perception import GridSpec
from reachability_guided_nbv.geometry import segments_intersect_box, sensor_transform
from reachability_guided_nbv.model import SensorModel, Viewpoint
from task_relevant_uncertainty.geometry import CONTACT_TOLERANCE_M, FootprintSpec, footprint_cells


FOOTPRINT = FootprintSpec(.52, .39)
O_TASK_TARGETS = {'easy': (0., .10), 'moderate': (.25, .50), 'hard': (.60, .85)}


def _true_boxes(scene):
    """Serialized A6 entities are axis-aligned boxes supported at map z=0."""
    bounds = []
    for box in scene['boxes']:
        center, size = np.asarray(box['center_xy'], float), np.asarray(box['size_xyz'], float)
        if (center.shape != (2,) or size.shape != (3,)
                or not np.all(np.isfinite(np.r_[center, size])) or np.any(size <= 0)
                or box['yaw'] != 0.):
            raise ValueError('A6 description requires finite positive boxes with serialized yaw=0')
        lower = np.r_[center - size[:2] / 2., 0.]
        bounds.append((lower, lower + size))
    return bounds


def _footprint_intersects_box(xy, yaw, lower, upper):
    """Closed rectangle/AABB SAT in XY; uses full exact footprint, no raster."""
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    axes = np.vstack((np.eye(2), rotation.T))
    delta = (lower[:2] + upper[:2]) / 2. - xy
    footprint_radius = np.abs(axes @ rotation) @ [FOOTPRINT.half_length_m, FOOTPRINT.half_width_m]
    box_radius = np.abs(axes) @ ((upper[:2] - lower[:2]) / 2.)
    return bool(np.all(np.abs(axes @ delta) <= footprint_radius + box_radius + CONTACT_TOLERANCE_M))


def _initial_grid(initial, ranking):
    """Reproduce frozen A1 grid construction without rerunning its query."""
    config = initial['config']
    a1 = GridSpec.centered(initial['grasp']['position_xyz'][:2],
                           config['grid_width_m'], config['grid_height_m'], .1)
    grid = EnvironmentGridSpec(a1.origin_xy, a1.width_cells, a1.height_cells, a1.resolution_m)
    saved = EnvironmentGridSpec(**ranking['a3_summary']['grid'])
    if (grid.shape != saved.shape or grid.resolution_m != saved.resolution_m
            or not np.allclose(grid.origin_xy, saved.origin_xy, rtol=0., atol=1e-12)):
        raise ValueError('saved ranking grid does not match initial A1 grid')
    return grid


def _true_visibility(grid, ranking, boxes):
    """Ground-center range/FOV and true-box rays, without any A2 state gate."""
    sensor_data, config = ranking['sensor'], ranking['a2_config']
    if sensor_data['horizontal_fov_deg'] != 360:
        raise ValueError('A6 MID360 description requires the frozen 360-degree horizontal FOV')
    sensor = SensorModel(np.asarray(sensor_data['T_uav_lidar']),
                         sensor_data['min_elevation_deg'], sensor_data['max_elevation_deg'])
    transform = sensor_transform(Viewpoint(**ranking['current']), sensor)
    origin = transform[:3, 3]
    rows, cols = np.indices(grid.shape)
    targets = np.column_stack((grid.origin_xy[0] + (cols.ravel() + .5) * grid.resolution_m,
                               grid.origin_xy[1] + (rows.ravel() + .5) * grid.resolution_m,
                               np.full(rows.size, config['ground_z_m'])))
    local = (targets - origin) @ transform[:3, :3]
    distance = np.linalg.norm(local, axis=1)
    elevation = np.degrees(np.arctan2(local[:, 2], np.hypot(local[:, 0], local[:, 1])))
    range_fov = ((distance > config['min_range_m']) & (distance < config['max_range_m'])
                 & (elevation >= sensor.min_elevation_deg) & (elevation <= sensor.max_elevation_deg))
    occluded = np.zeros(rows.size, dtype=bool)
    eligible = np.flatnonzero(range_fov)
    for lower, upper in boxes:
        occluded[eligible] |= segments_intersect_box(origin, targets[eligible], lower, upper)
    return origin, range_fov, occluded


def describe_snapshot(scene, initial, ranking, decision, arrays):
    """Describe a completed first window; never return an admission decision."""
    if decision['round'] != 1:
        raise ValueError('scene description requires the first observation window')
    grid = _initial_grid(initial, ranking)
    nominal = np.asarray(arrays['a3_nominal_task_relevance'])
    observations = np.asarray(arrays['a2_observation_count'])
    state = np.asarray(arrays['a2_state'])
    if any(array.shape != grid.shape for array in (nominal, observations, state)):
        raise ValueError('snapshot arrays must match the initial A1 grid')
    if (np.any(np.isinf(nominal)) or not np.all(np.isfinite(observations))
            or np.any(observations < 0)):
        raise ValueError('nominal support and observation counts contain invalid values')
    boxes = _true_boxes(scene)
    origin, range_fov, occluded = _true_visibility(grid, ranking, boxes)
    catalog_union = np.zeros(nominal.size, dtype=bool)
    clear_high_union = catalog_union.copy()
    candidates = []
    for candidate in decision['assessments']:
        xy, yaw = np.array([candidate['x'], candidate['y']]), candidate['yaw']
        ids, clipped = footprint_cells(grid, xy, yaw, FOOTPRINT)
        catalog_union[ids] = True
        intersects = any(_footprint_intersects_box(xy, yaw, lo, hi) for lo, hi in boxes)
        high = candidate['relevance'] >= .7
        clear = not clipped and not intersects and bool(len(ids))
        if high and clear:
            clear_high_union[ids] = True
        candidates.append(dict(candidate_id=candidate['candidate_id'], relevance=candidate['relevance'],
                               footprint_cells=len(ids), footprint_clipped=clipped,
                               true_box_intersection=intersects, nominal_clear=clear,
                               high_relevance_nominal_clear=bool(high and clear),
                               full_initial_ground_center_visibility=bool(clear and np.all(
                                   range_fov[ids] & ~occluded[ids]))))
    high_patch = (nominal.ravel() >= .7) & clear_high_union
    eligible_high = high_patch & range_fov
    blocked_high = eligible_high & occluded
    never = observations.ravel() == 0
    low_nominal = nominal.ravel() <= .1
    unsupported = ~catalog_union
    irrelevant = never & (low_nominal | unsupported)
    task_unknown = never & high_patch
    count = lambda mask: int(np.count_nonzero(mask))
    cell_area = grid.resolution_m ** 2
    eligible_count = count(eligible_high)
    o_task = count(blocked_high) / eligible_count if eligible_count else None
    a_irrel, a_task = count(irrelevant) * cell_area, count(task_unknown) * cell_area
    ratio = a_irrel / max(a_task, .1)
    clear_count = sum(c['high_relevance_nominal_clear'] for c in candidates)
    visible_count = sum(c['high_relevance_nominal_clear'] and c['full_initial_ground_center_visibility']
                        for c in candidates)
    lower, upper = O_TASK_TARGETS[scene['id']]
    targets = dict(o_task=dict(reference_range=[lower, upper], measured=o_task,
                               met=None if o_task is None else lower <= o_task <= upper),
                   nominal_clear_high_relevance_footprint=dict(minimum=1, measured=clear_count,
                                                                met=clear_count >= 1))
    if scene['id'] == 'easy':
        targets['full_initial_footprint_ground_center_visibility'] = dict(
            minimum=1, measured=visible_count, met=visible_count >= 1)
    if scene['id'] == 'hard':
        targets.update(a_irrel_m2=dict(minimum=3., measured=a_irrel, met=a_irrel >= 3.),
                       irrelevant_to_task_unknown_ratio=dict(minimum=2., measured=ratio, met=ratio >= 2.))
    return dict(scene_id=scene['id'], seed=scene['seed'], round=1,
                grid=dict(origin_xy=list(grid.origin_xy), shape=list(grid.shape),
                          resolution_m=grid.resolution_m, area_m2=nominal.size * cell_area),
                nominal_view_source='ranking.current with saved frozen MID360 transform; level UAV',
                actual_initial_view=ranking['current'], sensor_origin_map_m=origin.tolist(),
                sensor=ranking['sensor'], true_boxes=scene['boxes'], catalog_candidate_count=len(candidates),
                catalog_union_cells=count(catalog_union), nominal_clear_high_relevance_footprints=clear_count,
                full_initial_visible_high_relevance_footprints=visible_count,
                high_support_patch_cells=count(high_patch), high_support_patch_area_m2=count(high_patch) * cell_area,
                eligible_high_support_cells=eligible_count,
                true_box_occluded_high_support_cells=count(blocked_high), o_task=o_task,
                o_task_undefined_reason='no_high_support_ground_centers_in_initial_range_fov'
                if o_task is None else None,
                never_observed_cells=count(never), state_unknown_cells=count(state == EnvironmentState.UNKNOWN),
                candidate_relative_unsupported_never_observed_cells=count(never & unsupported),
                low_nominal_never_observed_cells=count(never & low_nominal),
                irrelevant_never_observed_cells=count(irrelevant), task_never_observed_cells=count(task_unknown),
                a_irrel_m2=a_irrel, a_task_unknown_m2=a_task,
                irrelevant_to_task_unknown_ratio=ratio, ratio_denominator_floor_m2=.1,
                targets=targets, proposal_mismatches=[key for key, value in targets.items() if value['met'] is False],
                undefined_targets=[key for key, value in targets.items() if value['met'] is None],
                catalog_footprints=candidates)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--results-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    scenes = {scene['id']: scene for scene in config['scenes']}
    reports, unavailable = [], []
    for record_path in sorted(args.results_dir.glob('*/attempt.json')):
        record = json.loads(record_path.read_text())
        directory = record_path.parent
        window = directory / 'data' / 'rounds' / 'round-01'
        paths = [directory / 'data' / 'initial.json', window / 'ranking.json',
                 window / 'decision.json', window / 'fields.npz']
        missing = [str(path) for path in paths if not path.is_file()]
        if missing:
            unavailable.append(dict(attempt=str(directory), actual_method=record['method'], missing=missing,
                                    reason='no_complete_saved_first_window; RM4D-only has no MID360 window'))
            continue
        initial, ranking, decision = (json.loads(path.read_text()) for path in paths[:3])
        # Prefer the actual serialized attempt specification and expose any
        # mismatch with the fixed configuration; never silently relabel it.
        scene = record['scene_spec']
        with np.load(paths[3], allow_pickle=False) as arrays:
            report = describe_snapshot(scene, initial, ranking, decision, arrays)
        report.update(snapshot=str(window), attempt=str(directory), actual_method=record['method'],
                      slot=record['slot'], attempt_status=record.get('status'),
                      serialized_scene_matches_config=scene == scenes[record['scene']],
                      serialized_seed_matches_config=record['seed'] == scenes[record['scene']]['seed'])
        reports.append(report)
    result = dict(kind='post_freeze_first_window_scene_description', descriptive_only=True,
                  config=str(args.config), results_dir=str(args.results_dir), snapshot_count=len(reports),
                  snapshots=reports, unavailable_snapshots=unavailable,
                  note='Proposal targets are descriptive and never select, reject, rerun or relabel scenes/trials. '
                       'N=0 is never-observed evidence, not state UNKNOWN. NaN nominal values are not zero. '
                       'Outside the exact catalog union means candidate-relative unsupported, not globally unreachable. '
                       'High support uses all unclipped relevance>=.7 exact catalog footprints clear of serialized boxes '
                       'and saved A3 nominal relevance>=.7; A2 FREE/blocked/confirmed never gates nominal-clear. '
                       'Catalog union includes in-grid cells of clipped footprints. Footprint cells use closed overlap; '
                       'rays end at their ground centers. Only serialized scene boxes are geometric occluders. '
                       'These checks do not establish continuous visibility of every footprint point, Ground routes, '
                       'arm clearance, side-view access, or complete physical feasibility.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(descriptive_only=True, snapshot_count=len(reports),
                         snapshots_with_proposal_mismatch=sum(bool(r['proposal_mismatches']) for r in reports),
                         unavailable_snapshot_count=len(unavailable))))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
