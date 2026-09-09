#!/usr/bin/env python3
"""Read-only arithmetic on saved Hard beliefs, exact footprints and visibility.

Run from any directory; emits JSON to stdout and never writes runtime evidence.
This is a frozen-snapshot opportunity inventory, not predicted observations,
mapper updates, candidate reselection, trajectory planning or retrieval proof.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src'))

from environment_belief import BeliefConfig, EnvironmentBeliefGrid, EnvironmentGridSpec
from reachability_guided_nbv.geometry import predict_visibility
from reachability_guided_nbv.model import NBVConfig, SensorModel, Viewpoint
from task_relevant_uncertainty.geometry import FootprintSpec, footprint_cells

RUNS = ('launch-05-hard01-generic', 'launch-06-hard01-ours',
        'launch-11-hard02-ours', 'launch-12-hard02-generic')
BELIEF_NAMES = ('state', 'occupied_evidence', 'free_evidence',
                'observation_count', 'unknown_score')


def npz(path):
    with np.load(path, allow_pickle=False) as saved:
        return {key: saved[key].copy() for key in saved.files}


def analyze_round(directory, number):
    path = directory / 'data/rounds' / f'round-{number:02d}'
    paths = [path / name for name in ('ranking.json', 'decision.json',
                                     'fields.npz', 'operational_evidence.npz')]
    ranking, decision = [json.loads(p.read_text()) for p in paths[:2]]
    fields, evidence = [npz(p) for p in paths[2:]]
    grid = EnvironmentGridSpec(**ranking['a3_summary']['grid'])
    config = BeliefConfig(**ranking['a2_config'])
    belief = EnvironmentBeliefGrid(grid, config,
                                  **{key: fields['a2_' + key] for key in BELIEF_NAMES})
    sensor = SensorModel(**{key: ranking['sensor'][key] for key in
                           ('T_uav_lidar', 'min_elevation_deg', 'max_elevation_deg')})
    nbv = NBVConfig(**ranking['config'])
    candidates = ranking['candidates']
    assert [c['candidate_id'] for c in candidates] == list(range(len(candidates)))
    masks = fields['visibility'].reshape(len(candidates), -1)
    valid = np.array([c['candidate_id'] for c in candidates if c['status'] == 'VALID'])
    for c in candidates:
        replay = predict_visibility(belief, Viewpoint(**c['viewpoint']), sensor=sensor, config=nbv)
        assert replay.status == c['status']
        assert np.array_equal(masks[c['candidate_id']], replay.visible.ravel())
    ground = evidence['ground_presence_votes'].ravel()
    assert np.all((ground >= 0) & (ground <= number))
    remaining = 3 - number
    required = config.free_observations
    assert required == 2 and remaining in (1, 2)
    chosen = decision.get('policy_best_candidate_id', ranking['best_task_id'])
    chosen_index = int(np.flatnonzero(valid == chosen)[0])
    assert np.allclose([*candidates[chosen]['viewpoint']['position_xyz'],
                        candidates[chosen]['viewpoint']['yaw_rad']],
                       decision['next_viewpoint'], rtol=0, atol=1e-12)
    anchors = {a['candidate_id']: a for a in ranking['a3_summary']['winner_anchors']}
    viable = [a for a in decision['assessments'] if not a['representative_blocked']
              and not a['operational']['blocked'] and not a['footprint_clipped']]
    dimensions = (len(valid),) * remaining
    completed = np.zeros(dimensions, dtype=int)
    best_missing_any = np.full(dimensions, grid.width_cells * grid.height_cells, dtype=int)
    footprint_rows = []
    for a in viable:
        anchor = anchors[a['candidate_id']]
        for key in ('x', 'y', 'yaw', 'source_id'):
            assert anchor[key] == a[key]
        ids, clipped = footprint_cells(grid, (a['x'], a['y']), a['yaw'],
                                       FootprintSpec(**ranking['a3_summary']['footprint']))
        assert not clipped and np.array_equal(ids, a['operational']['covered_cells'])
        assert int(np.count_nonzero(ground[ids] < required)) == a['operational']['ground_missing_cells']
        assert a['confirmed'] == bool(np.all(ground[ids] >= required))
        visibility = masks[valid][:, ids].astype(np.int16)
        # Integer opportunities exist only in this detached arithmetic expression.
        # Neither saved ground nor any mapper/operational object is changed.
        arithmetic = (ground[ids] + visibility if remaining == 1 else
                      ground[ids] + visibility[:, None, :] + visibility[None, :, :])
        missing = np.count_nonzero(arithmetic < required, axis=-1)
        succeeds = missing == 0
        completed += succeeds
        best_missing_any = np.minimum(best_missing_any, missing)
        best_position = tuple(np.argwhere(missing == missing.min())[0])
        best_ids = [int(valid[i]) for i in best_position]
        best_cells = ids[arithmetic[best_position] < required]
        footprint_rows.append(dict(
            candidate_id=a['candidate_id'], source_id=a['source_id'],
            pose_xyyaw=[a[key] for key in ('x', 'y', 'yaw')],
            cells=len(ids), recorded_presence_histogram=np.bincount(ground[ids], minlength=4).tolist(),
            zero_vote_cells=int(np.count_nonzero(ground[ids] == 0)),
            under_supported_cells_invisible_in_all_saved_candidates=ids[
                (ground[ids] < required) & ~np.any(visibility, axis=0)].tolist(),
            completing_ordered_choices=int(succeeds.sum()),
            minimum_missing_cells=int(missing.min()),
            example_minimum_missing_choice=best_ids,
            example_minimum_missing_cell_ids=best_cells.tolist(),
            example_minimum_missing_viewpoints=[candidates[i]['viewpoint'] for i in best_ids],
            selected_first_minimum_missing=int(np.min(missing[chosen_index])),
            selected_first_completing_choices=int(np.sum(succeeds[chosen_index])),
            completing_repeat_ids=(valid[np.diag(succeeds)].tolist() if remaining == 2 else None),
            completing_one_view_ids=(valid[succeeds].tolist() if remaining == 1 else None),
        ))
    choices = [[int(valid[i]) for i in indices] for indices in np.argwhere(completed > 0)]
    examples = []
    for ids in choices[:10]:
        same_position_different_yaw = remaining == 2 and ids[0] != ids[1] and np.allclose(
            candidates[ids[0]]['viewpoint']['position_xyz'],
            candidates[ids[1]]['viewpoint']['position_xyz'], rtol=0, atol=1e-12)
        examples.append(dict(candidate_ids=ids, viewpoints=[candidates[i]['viewpoint'] for i in ids],
                             yaw_only_change_between_choices=bool(same_position_different_yaw)))
    return dict(
        round=number, remaining_windows=remaining, support_required_windows=required,
        candidates=len(valid), viable_exact_winners=len(viable),
        recorded_confirmed=decision['confirmed_candidate_count'],
        selected_nbv_id=int(chosen), method=decision.get('policy_method', 'ours'),
        selected_nbv=candidates[chosen], task_order=ranking['task_order'], generic_order=ranking['generic_order'],
        sources=[str(p.relative_to(ROOT)) for p in paths],
        all_saved_masks_and_viable_footprints_reproduced=True,
        ordered_choices_examined=int(completed.size), completing_ordered_choices=len(choices),
        max_simultaneously_completed_winners=int(completed.max()),
        minimum_missing_cells_any_winner=int(best_missing_any.min()),
        selected_first_minimum_missing_any_winner=int(best_missing_any[chosen_index].min()),
        selected_first_completing_choices=int(np.count_nonzero(completed[chosen_index])),
        completing_examples=examples, exact_winners=footprint_rows,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--summary', action='store_true')
    args = parser.parse_args()
    runs = [dict(run=name, rounds=[analyze_round(ROOT / 'outputs/development/multiscene-paired' / name, n)
                                  for n in (1, 2)]) for name in RUNS]
    result = dict(kind='frozen_saved_visibility_footprint_opportunity_arithmetic',
                  inputs='saved sensor runtime beliefs, exact winner assessments and NBV masks',
                  no_gt_geometry=True, no_mapper_updates=True, no_reselection=True,
                  model='g(c) + sum(saved_visible_view_j(c)) >= 2 for EVERY footprint cell c',
                  limitations=[
                      'Views and masks are frozen at each real snapshot. Two-view pairs are arithmetic inventories, not replanned trajectories.',
                      'Repetition is allowed. Cross pairs at identical XYZ but different yaw are not generated as yaw-only moves by the current facade-aware candidate filter.',
                      'Each future saved-visible cell receives exactly one ideal opportunity; this is not measured ground support or a calibrated return model.',
                      'Other gates are frozen. New sensor evidence can alter blocking, visibility, current pose and later enumerated candidates.',
                      'A zero count excludes completion only within the stated frozen inventory and model. Real cells can be observed outside center visibility masks.',
                      'A positive count proves arithmetic opportunity only, not acquisition, Ground navigation, full-robot planning or retrieval feasibility.',
                  ], runs=runs)
    if args.summary:
        keys = ('round', 'remaining_windows', 'candidates', 'viable_exact_winners',
                'selected_nbv_id', 'ordered_choices_examined', 'completing_ordered_choices',
                'minimum_missing_cells_any_winner', 'selected_first_minimum_missing_any_winner',
                'selected_first_completing_choices')
        result = dict(all_saved_masks_and_viable_footprints_reproduced=True,
                      runs=[dict(run=run['run'], rounds=[{k: r[k] for k in keys} for r in run['rounds']])
                            for run in runs])
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
