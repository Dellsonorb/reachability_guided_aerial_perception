#!/usr/bin/env python3
"""Frozen footprint/phase inventory. No counterfactual sensing or evidence update."""

import argparse
from dataclasses import replace
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from finite_scan_probe import ROOT, HERE, RUNS, PATTERN, old, LevelHoverCache, phase_probability
from reachability_guided_nbv.finite_scan import FiniteScan
from reachability_guided_nbv.model import generate_candidates


def window_masks(hits, horizon=51):
    return np.array([np.any(hits[(start + np.arange(horizon)) % len(hits)], axis=0)
                     for start in range(len(hits))]).reshape(len(hits), -1)


def summarize(masks, ground, viable):
    pairs = {}
    best = None
    for a in viable:
        ids = a['operational']['covered_cells']
        view = masks[:, ids].astype(np.int8)
        missing = np.count_nonzero(ground[ids] + view[:, None, :] + view[None, :, :] < 2, axis=-1)
        index = np.unravel_index(np.argmin(missing), missing.shape)
        score = int(missing[index])
        if best is None or score < best['missing']:
            best = dict(missing=score, pair=list(index), assessment=a,
                        residual=np.array(ids)[ground[ids] + view[index[0]] + view[index[1]] < 2])
        for i, j in np.argwhere(missing == 0):
            pairs.setdefault((int(i), int(j)), []).append(a)
    return best, pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.resolve().parent != HERE or args.output.exists():
        parser.error('output must be new and directly inside finite-scan-analysis')
    model = FiniteScan.from_csv(PATTERN, publisher_sdf_path=PATTERN.parent.parent / 'MID360.sdf')
    cache = LevelHoverCache(model.directions, model.sensor)
    body_down = (model.directions @ model.sensor.T_uav_lidar[:3, :3].T)[:, 2] < -1e-12
    keep = model.emission_mask[body_down]
    cache.packet, cache.slopes, cache.distance_scale = [a[keep] for a in (cache.packet, cache.slopes, cache.distance_scale)]
    result = dict(kind='frozen_footprint_joint_phase_inventory', no_observations_generated=True,
                  no_mapper_updates=True, no_gt_geometry=True, model_metadata=model.metadata, runs=[])
    for name in RUNS:
        folder = ROOT / 'outputs/development/multiscene-paired' / name / 'data/rounds/round-01'
        ranking, decision = [json.loads((folder / f'{f}.json').read_text()) for f in ('ranking', 'decision')]
        fields, evidence = [old.load_npz(folder / f'{f}.npz') for f in ('fields', 'operational_evidence')]
        grid = old.EnvironmentGridSpec(**ranking['a3_summary']['grid'])
        config = old.BeliefConfig(**ranking['a2_config'])
        belief = old.EnvironmentBeliefGrid(grid, config, **{k: fields['a2_' + k] for k in old.BELIEF_NAMES})
        nbv = old.NBVConfig(**ranking['config'])
        ground = evidence['ground_presence_votes'].ravel()
        viable = [a for a in decision['assessments'] if not a['representative_blocked']
                  and not a['operational']['blocked'] and not a['footprint_clipped']]
        original = [old.Viewpoint(**c['viewpoint']) for c in ranking['candidates']]
        views = list(generate_candidates(original[0], sensor=model.sensor,
                                         config=replace(nbv, xy_offsets_m=(-2, -1, 0, 1, 2))))
        predictions, times = [], []
        for view in views:
            start = perf_counter()
            predictions.append(model.predict(belief, view, config=nbv))
            times.append(perf_counter() - start)
        masks = np.array([p.opportunity.ravel() > 0 for p in predictions])
        inventories = []
        for label, indices in [('original_2m', [i for i, v in enumerate(views) if v in original]),
                               ('midpoint_1m', list(range(len(views))))]:
            best, possible_pairs = summarize(masks[indices], ground, viable)
            pair = [indices[i] for i in best['pair']]
            raw_p = [phase_probability(cache.packets(views[i], grid, config), 51)[0] for i in pair]
            residuals = []
            for cell in best['residual']:
                cell = int(cell)
                residuals.append(dict(cell_id=cell, raw_a2_state=int(belief.state.ravel()[cell]),
                    ground_presence=int(ground[cell]),
                    evidence={k: int(evidence[k].ravel()[cell]) for k in ('target_occupied_votes',
                        'environment_occupied_votes', 'ambiguous_occupied_votes', 'ground_votes')},
                    pair_scan_only_phase_opportunity=[float(p[cell]) for p in raw_p],
                    pair_ray_prism_phase_opportunity=[float(predictions[i].opportunity.ravel()[cell]) for i in pair]))
            joint = []
            for (i, j), assessments in possible_pairs.items():
                i, j = indices[i], indices[j]
                windows = [window_masks(predictions[k].packet_hits) for k in (i, j)]
                winners = []
                for assessment in assessments:
                    ids = assessment['operational']['covered_cells']
                    a, b = [w[:, ids].astype(np.int8) for w in windows]
                    complete = np.all(ground[ids] + a[:, None, :] + b[None, :, :] >= 2, axis=-1)
                    winners.append(dict(source_id=assessment['source_id'], candidate_id=assessment['candidate_id'],
                        cells=len(ids), complete_phase_start_pairs=int(complete.sum()), total_phase_start_pairs=6400))
                joint.append(dict(pair=[i, j], poses=[dict(xyz=views[k].position_xyz, yaw=views[k].yaw_rad) for k in (i, j)],
                                  exact_winners=winners))
            inventories.append(dict(lattice=label, candidates=len(indices),
                sum_prediction_s=float(np.array(times)[indices].sum()),
                best_any_positive_pair=pair, best_pair_poses=[dict(xyz=views[i].position_xyz, yaw=views[i].yaw_rad) for i in pair],
                best_missing_cells=best['missing'], best_source_id=best['assessment']['source_id'],
                residual_cells=residuals, any_positive_completing_pairs=len(possible_pairs),
                joint_phase_inventory=joint))
        row = dict(run=name, inventories=inventories)
        result['runs'].append(row)
        print(name, json.dumps(row), flush=True)
    result['limitations'] = [
        'The frozen belief and exact footprints do not change after hypothetical views.',
        'Any-positive cell masks alone are an optimistic upper bound because cells share packet phases.',
        'Enumerated phase-start pairs demonstrate joint geometric opportunity conditional on both poses, not independent probabilities, future timing feasibility or task success.',
        'No drift, dropout or unknown occluders enter the hypothetical models.',
        'Residual vote classifications are recorded endpoint categories; they do not authorize changing occupied geometry or operational gates.'
    ]
    with args.output.open('x') as destination:
        json.dump(result, destination, indent=2, allow_nan=False)
        destination.write('\n')


if __name__ == '__main__':
    main()
