#!/usr/bin/env python3
"""Independent read-only replay of operational prediction geometry and bounds."""

import argparse
from dataclasses import replace
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from finite_scan_probe import ROOT, HERE, RUNS, PATTERN, old, errors, probability_errors
from completion_phases_probe import window_masks
from operational_gating.core import OperationalEvidenceView, PerceivedTarget
from operational_gating.subcell import AmbiguousEndpointEvidence
from reachability_guided_nbv.finite_scan import FiniteScan
from reachability_guided_nbv.model import generate_candidates
from reachability_guided_nbv.operational_occlusion import OperationalOcclusion, build_operational_occlusion


def load_state(folder):
    ranking, decision, summary = [json.loads((folder / f'{name}.json').read_text())
                                 for name in ('ranking', 'decision', 'operational_summary')]
    fields, evidence = [old.load_npz(folder / f'{name}.npz') for name in ('fields', 'operational_evidence')]
    grid = old.EnvironmentGridSpec(**ranking['a3_summary']['grid'])
    config = old.BeliefConfig(**ranking['a2_config'])
    belief = old.EnvironmentBeliefGrid(grid, config, **{k: fields['a2_' + k] for k in old.BELIEF_NAMES})
    sidecar = AmbiguousEndpointEvidence(**{name: evidence[saved] for name, saved in (
        ('points_xy', 'ambiguous_endpoint_xy'), ('observation_indices', 'ambiguous_endpoint_observation_indices'),
        ('row_indices', 'ambiguous_endpoint_row_indices'), ('cell_ids', 'ambiguous_endpoint_cell_ids'),
        ('complete_vote_counts', 'ambiguous_endpoint_complete_vote_counts'))},
        profile=str(evidence['ambiguous_endpoint_profile']), radius_m=float(evidence['ambiguous_endpoint_radius_m']))
    operational = OperationalEvidenceView(grid, config, PerceivedTarget(**summary['target']),
        **{k: evidence[k] for k in ('environment_occupied_votes', 'ambiguous_occupied_votes',
            'target_occupied_votes', 'ground_votes', 'ground_presence_votes')}, ambiguous_endpoints=sidecar)
    return dict(ranking=ranking, decision=decision, fields=fields, evidence=evidence, belief=belief,
                operational=operational)


def independent_cylinder(origin, endpoint, radius=.033, height=1.):
    v = endpoint - origin
    low, high = 0., 1.
    if abs(v[2]) < 1e-12:
        if not 0 <= origin[2] <= height:
            return False
    else:
        a, b = sorted(((0 - origin[2]) / v[2], (height - origin[2]) / v[2]))
        low, high = max(low, a), min(high, b)
    a = np.dot(v[:2], v[:2])
    b = 2 * np.dot(origin[:2], v[:2])
    c = np.dot(origin[:2], origin[:2]) - radius ** 2
    if a == 0:
        return low <= high and c <= 1e-12
    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False
    first, second = (-b - np.sqrt(discriminant)) / (2 * a), (-b + np.sqrt(discriminant)) / (2 * a)
    return max(low, first) <= min(high, second) + 1e-12


def audit_cylinders():
    rng = np.random.default_rng(917)
    occlusion = OperationalOcclusion(np.empty((0, 3)), np.empty((0, 3)),
        PerceivedTarget((100, 100, 100), .4), np.array([[0., 0.]]), .033, 0., 1.)
    mismatches, checked = [], 0
    for _ in range(30):
        origin = rng.uniform([-.2, -.2, -.5], [.2, .2, 1.5])
        ends = rng.uniform([-.2, -.2, -.5], [.2, .2, 1.5], size=(400, 3))
        production = occlusion.blocked(origin, ends)
        independent = np.array([independent_cylinder(origin, end) for end in ends])
        mismatches.extend(np.flatnonzero(production != independent).tolist())
        checked += len(ends)
    assert not mismatches
    return dict(independent_quadratic_comparisons=checked, mismatches=len(mismatches))


def inventory(views, predictions, ground, viable, transition_tolerance):
    masks = np.array([p.opportunity.ravel() > 0 for p in predictions])
    legal = np.ones((len(views), len(views)), dtype=bool)
    for i, a in enumerate(views):
        for j, b in enumerate(views):
            if a != b and np.linalg.norm(np.asarray(a.position_xyz) - b.position_xyz) <= transition_tolerance:
                legal[i, j] = False
    pair_winners, best_missing = {}, len(ground)
    for assessment in viable:
        ids = assessment['operational']['covered_cells']
        m = masks[:, ids].astype(np.int8)
        missing = np.count_nonzero(ground[ids] + m[:, None, :] + m[None, :, :] < 2, axis=-1)
        best_missing = min(best_missing, int(missing[legal].min()))
        for i, j in np.argwhere((missing == 0) & legal):
            pair_winners.setdefault((int(i), int(j)), []).append(assessment)
    joint = []
    cache = {}
    for (i, j), assessments in pair_winners.items():
        for k in (i, j):
            if k not in cache:
                cache[k] = window_masks(predictions[k].packet_hits)
        winners = []
        for assessment in assessments:
            ids = assessment['operational']['covered_cells']
            a, b = [cache[k][:, ids].astype(np.int8) for k in (i, j)]
            complete = np.all(ground[ids] + a[:, None, :] + b[None, :, :] >= 2, axis=-1)
            winners.append(dict(source_id=assessment['source_id'], complete_phase_start_pairs=int(complete.sum())))
        joint.append(dict(pair=[i, j], poses=[dict(xyz=views[k].position_xyz, yaw=views[k].yaw_rad) for k in (i, j)],
                          exact_winners=winners))
    return dict(candidates=len(views), legal_ordered_view_pairs=int(legal.sum()),
        minimum_missing_cells=best_missing, any_positive_completing_view_pairs=len(joint),
        jointly_possible_view_pairs=sum(any(w['complete_phase_start_pairs'] > 0 for w in p['exact_winners']) for p in joint),
        joint_phase_inventory=joint)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.resolve().parent != HERE or args.output.exists():
        parser.error('output must be new and directly inside finite-scan-analysis')
    profile = json.loads((ROOT / 'configs/current_sim_task.json').read_text())['shared_a5_settings']
    bounds = np.array(profile['flight_bounds'])
    tolerance = profile['facade_position_tolerance']
    scan = FiniteScan.from_csv(PATTERN, publisher_sdf_path=PATTERN.parent.parent / 'MID360.sdf')
    result = dict(kind='independent_operational_occlusion_review', no_mapper_updates=True,
        no_observations_generated=True, no_gt_geometry=True, cylinder_audit=audit_cylinders(),
        flight_bounds=bounds.tolist(), facade_position_tolerance_m=tolerance,
        phase_start_pair_semantics='enumerated joint geometric opportunities, not independent probabilities', runs=[])
    for name in RUNS:
        data = ROOT / 'outputs/development/multiscene-paired' / name / 'data'
        before, after = [load_state(data / 'rounds' / f'round-{n:02d}') for n in (1, 2)]
        belief, context = before['belief'], before['operational']
        saved = {k: getattr(belief, k).copy() for k in old.BELIEF_NAMES}
        geometry = build_operational_occlusion(belief, context)
        nbv = old.NBVConfig(**before['ranking']['config'])
        selected = before['decision']['next_viewpoint']
        pose = old.Viewpoint(selected[:3], selected[3])
        observation = old.load_npz(data / 'observation_02.npz')
        points = observation['points_xyz']
        mapped = points @ observation['T_map_sensor'][:3, :3].T + observation['T_map_sensor'][:3, 3]
        ranges = np.linalg.norm(points, axis=1)
        actual = old.counts(old.cell_ids(mapped, belief.grid), belief.state.size,
            (ranges > belief.config.min_range_m) & (ranges < belief.config.max_range_m) &
            (np.abs(mapped[:, 2] - belief.config.ground_z_m) <= belief.config.ground_tolerance_m)) > 0
        old_prediction = scan.predict(belief, pose, config=nbv)
        start = perf_counter()
        prediction = scan.predict(belief, pose, config=nbv, occlusion=geometry)
        selected_seconds = perf_counter() - start
        viable = [a for a in after['decision']['assessments'] if not a['representative_blocked']
                  and not a['operational']['blocked'] and not a['footprint_clipped']]
        union = np.unique(np.concatenate([a['operational']['covered_cells'] for a in viable]))
        scopes = {}
        for label, ids in [('all_grid', np.arange(belief.state.size)), ('viable_union', union)]:
            scopes[label] = dict(old_raw_prism=errors(old_prediction.opportunity.ravel() > 0, actual, ids),
                operational_geometry=errors(prediction.opportunity.ravel() > 0, actual, ids),
                phase_fraction=probability_errors(prediction.opportunity.ravel(), actual, ids))
        cell = 781
        row = dict(run=name, window=2, geometry_metadata=geometry.metadata, selected_prediction_seconds=selected_seconds,
            scopes=scopes, cell781=dict(old_opportunity=float(old_prediction.opportunity.ravel()[cell]),
                new_opportunity=float(prediction.opportunity.ravel()[cell]), actual_second_ground=bool(actual[cell]),
                presence_before=int(before['evidence']['ground_presence_votes'].ravel()[cell]),
                presence_after=int(after['evidence']['ground_presence_votes'].ravel()[cell])))
        current = old.Viewpoint(**before['ranking']['candidates'][0]['viewpoint'])
        inventories = []
        for offsets in [(-2, 0, 2), (-2, -1, 0, 1, 2)]:
            generated = list(generate_candidates(current, sensor=scan.sensor, config=replace(nbv, xy_offsets_m=offsets)))
            views = [v for v in generated if np.all(np.asarray(v.position_xyz) >= bounds[::2])
                     and np.all(np.asarray(v.position_xyz) <= bounds[1::2])
                     and (v == current or np.linalg.norm(np.asarray(v.position_xyz) - current.position_xyz) > tolerance)]
            start = perf_counter()
            predictions = [scan.predict(belief, v, config=nbv, occlusion=geometry) for v in views]
            elapsed = perf_counter() - start
            first_viable = [a for a in before['decision']['assessments'] if not a['representative_blocked']
                            and not a['operational']['blocked'] and not a['footprint_clipped']]
            info = inventory(views, predictions, before['evidence']['ground_presence_votes'].ravel(), first_viable, tolerance)
            info.update(offsets_m=list(offsets), unfiltered_candidate_count=len(generated), prediction_seconds=elapsed)
            inventories.append(info)
        row['first_state_inventories'] = inventories
        for k, original in saved.items():
            assert np.array_equal(getattr(belief, k), original)
        row['belief_unchanged'] = True
        result['runs'].append(row)
        print(name, json.dumps(dict(scopes=scopes, cell781=row['cell781'],
            inventories=[{k: v for k, v in i.items() if k != 'joint_phase_inventory'} for i in inventories])), flush=True)
    with args.output.open('x') as destination:
        json.dump(result, destination, indent=2, allow_nan=False)
        destination.write('\n')


if __name__ == '__main__':
    main()
