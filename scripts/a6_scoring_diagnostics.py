#!/usr/bin/env python3
"""Read saved shared rankings: numerical gain/cost identities, not extra trials."""

import argparse
import json
from pathlib import Path
import numpy as np


def check_snapshot(ranking, arrays):
    alpha = -np.expm1(-1./ranking['a2_config']['unknown_scale'])
    delta = alpha*np.asarray(arrays['a2_unknown_score'])
    marginal = alpha*np.asarray(arrays['a3_task_relevant_uncertainty'])
    errors = [float(np.max(np.abs(delta-arrays['delta_unknown'])))]
    same_nan = np.array_equal(np.isnan(marginal), np.isnan(arrays['marginal_task_gain']))
    expected_u = arrays['a2_unknown_score']*arrays['a3_task_relevance_at_environment_cell']
    same_nan = same_nan and np.array_equal(np.isnan(expected_u), np.isnan(arrays['a3_task_relevant_uncertainty']))
    supported = np.isfinite(expected_u)
    if np.any(supported):
        errors.append(float(np.max(np.abs(expected_u[supported]-arrays['a3_task_relevant_uncertainty'][supported]))))
    finite = np.isfinite(marginal)
    if np.any(finite):
        errors.append(float(np.max(np.abs(marginal[finite]-arrays['marginal_task_gain'][finite]))))
    visibility = arrays['visibility']
    opportunity = arrays['observation_opportunity'] if 'observation_opportunity' in arrays else visibility
    opportunity_valid = (opportunity.shape == visibility.shape
                         and np.all(np.isfinite(opportunity) & (opportunity >= 0) & (opportunity <= 1))
                         and np.array_equal(opportunity > 0, visibility))
    comparisons = []
    for row, mask, weight in zip(ranking['candidates'], visibility, opportunity):
        task = float(np.nansum(marginal*weight))
        generic = float(np.sum(delta*weight))
        penalty = ranking['config']['flight_weight']*row['flight_cost']
        errors.extend((abs(task-row['task_gain']), abs(generic-row['generic_gain'])))
        if row['status'] == 'VALID':
            errors.extend((abs(task-penalty-row['task_score']), abs(generic-penalty-row['generic_score'])))
        comparisons.append(dict(candidate_id=row['candidate_id'], task_gain=task,
                                generic_gain=generic, common_penalty=penalty,
                                summed_observation_opportunity=float(np.sum(weight)),
                                visible_cells=int(np.count_nonzero(mask))))
    numerically_valid = bool(np.all(np.isfinite(errors)) and not any(np.any(np.isinf(arrays[key])) for key in
                              ('a3_task_relevance_at_environment_cell', 'a3_task_relevant_uncertainty',
                               'marginal_task_gain')))
    maximum = max(errors) if numerically_valid else None
    valid = [i for i, row in enumerate(ranking['candidates']) if row['status']=='VALID' and i<len(comparisons)]
    order = lambda gain: sorted(valid, key=lambda i: (-(comparisons[i][gain]-comparisons[i]['common_penalty']),
                                                     ranking['candidates'][i]['flight_cost'], i))
    task_order, generic_order = order('task_gain'), order('generic_gain')
    best_o = task_order[0] if task_order and any(comparisons[i]['task_gain']>0 for i in valid) else None
    best_g = generic_order[0] if generic_order else None
    argmax_valid = (best_o==ranking['best_task_id'] and best_g==ranking['best_generic_id']
                    and task_order==list(ranking['task_order']) and generic_order==list(ranking['generic_order'])
                    and all(row['candidate_id']==i for i,row in enumerate(ranking['candidates'])))
    return dict(candidate_count=len(ranking['candidates']), shared_visibility_shape=list(visibility.shape),
                gain_and_cost_identity_pass=bool(same_nan and numerically_valid and argmax_valid and opportunity_valid
                                                and len(visibility)==len(ranking['candidates']) and maximum<1e-9),
                argmax_identity_pass=argmax_valid, finite_scalar_checks_pass=numerically_valid,
                maximum_abs_identity_error=maximum, best_ours_id=best_o, best_generic_id=best_g,
                different_selection=bool(best_o is not None and best_g is not None and best_o!=best_g),
                task_uncertainty_mass=float(np.nansum(arrays['a3_task_relevant_uncertainty'])),
                generic_unknown_mass=float(np.sum(arrays['a2_unknown_score'])),
                never_observed_area_m2=float(np.count_nonzero(arrays['a2_observation_count']==0)*.01),
                candidates=comparisons)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    reports = []
    for path in sorted(args.results_dir.glob('*/data/rounds/round-*/ranking.json')):
        ranking = json.loads(path.read_text())
        with np.load(path.with_name('fields.npz'), allow_pickle=False) as arrays:
            report = check_snapshot(ranking, arrays)
        decision = json.loads(path.with_name('decision.json').read_text())
        report.update(snapshot=str(path.parent), actual_method=decision.get('policy_method', 'ours'),
                      round=decision['round'], confirmed_candidate_count=decision['confirmed_candidate_count'])
        reports.append(report)
    result = dict(kind='same_state_shared_mask_scoring_diagnostics', snapshots=reports,
                  snapshot_count=len(reports),
                  all_identity_checks_pass=bool(reports) and all(r['gain_and_cost_identity_pass'] for r in reports),
                  differing_selection_snapshots=sum(r['different_selection'] for r in reports),
                  note='Both scores use each saved snapshot\'s single ordered candidate set, visibility tensor and cost. '
                       'This is not a claim that divergent closed-loop trials share later states. '
                       'Ablation snapshots also retain the original shared ranking; variant decisions are separate.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='snapshots'}))
    return 0 if result['all_identity_checks_pass'] else 1


if __name__ == '__main__': raise SystemExit(main())
