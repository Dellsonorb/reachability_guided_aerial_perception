"""Select comparators on one shared acquisition/execution profile."""

from dataclasses import asdict, replace

import numpy as np

from reachability_guided_aerial_perception import FieldConfig, candidate_relevance
from reachability_guided_nbv.geometry import predict_visibility
from reachability_guided_nbv.model import readonly
from sim_active_perception.core import A5Config, decide
from task_relevant_uncertainty import SupportState


METHODS = ('ours', 'generic', 'fixed', 'no_occlusion', 'no_cost')
DEVELOPMENT_METHODS = ('confirmation',)
VIEW_BUDGET = 3


def _without_occlusion(belief, ranking):
    """Remove only known-prism occlusion, preserving the acquisition model."""
    evaluations, visibility, opportunities = [], [], []
    for original in ranking.candidates:
        if ranking.unoccluded_opportunity is None:
            prediction = predict_visibility(belief, original.viewpoint,
                                            sensor=ranking.sensor, config=ranking.config)
            opportunity = prediction.range_fov.astype(float)
        else:
            opportunity = ranking.unoccluded_opportunity[original.candidate_id]
        visible = opportunity > 0
        task_gain = float(np.nansum(ranking.marginal_task_gain * opportunity))
        generic_gain = float(np.sum(ranking.delta_unknown * opportunity))
        valid = original.status == 'VALID'
        penalty = ranking.config.flight_weight * original.flight_cost
        evaluations.append(replace(
            original, task_gain=task_gain, generic_gain=generic_gain,
            task_score=task_gain - penalty if valid else None,
            generic_score=generic_gain - penalty if valid else None,
            visible_cells=int(visible.sum()),
            visible_task_cells=int(np.count_nonzero(visible & (ranking.support_state == SupportState.SUPPORTED))),
        ))
        visibility.append(visible)
        opportunities.append(opportunity)
    valid_ids = [row.candidate_id for row in evaluations if row.status == 'VALID']
    task_order = tuple(sorted(valid_ids, key=lambda i: (-evaluations[i].task_score, evaluations[i].flight_cost, i)))
    generic_order = tuple(sorted(valid_ids, key=lambda i: (-evaluations[i].generic_score, evaluations[i].flight_cost, i)))
    status = ('NO_VALID_CANDIDATE' if not valid_ids else
              'NO_PREDICTED_TASK_GAIN' if not any(evaluations[i].task_gain > 0 for i in valid_ids) else 'RANKED')
    return replace(ranking, candidates=tuple(evaluations), visibility=readonly(visibility),
                   observation_opportunity=readonly(opportunities),
                   task_order=task_order, generic_order=generic_order, status=status)


def _stop_reason(ranking, order, gain_name, score_name, round_count):
    if not order:
        return 'NO_VALID_CANDIDATE'
    if not any(getattr(ranking.candidates[i], gain_name) > 0 for i in order):
        return 'NO_PREDICTED_GENERIC_GAIN' if gain_name == 'generic_gain' else 'NO_PREDICTED_TASK_GAIN'
    if getattr(ranking.candidates[order[0]], score_name) <= 0:
        return 'NONPOSITIVE_SCORE'
    return 'VIEW_BUDGET_REACHED' if round_count >= VIEW_BUDGET else None


def decide_policy(field, raw, belief, current, *, method, round_count, config=A5Config(), operational=None):
    """Return frozen shared diagnostics and the selected method's next action.

    A6 requires the approved three-window budget. Ours is the exact A5 result
    within that contract. Extra ``policy_*`` keys for other methods are
    explicit variant diagnostics; the original ``best_task_*`` values and the
    returned ranking always describe the unchanged shared A5 calculation.
    ``policy_visibility`` stores only the selected candidate's flattened cell
    indices, with its shape, so decision JSON does not repeat every mask.
    """
    if method not in METHODS + DEVELOPMENT_METHODS:
        raise ValueError(f'unsupported A6 method: {method}')
    if config.max_viewpoints != VIEW_BUDGET:
        raise ValueError('A6 max_viewpoints must be 3')
    choice, ranking = decide(field, raw, belief, current, round_count=round_count,
                              config=config, operational=operational)
    if method == 'ours':
        return choice, ranking
    if method == 'confirmation':
        from .confirmation import apply_confirmation
        return apply_confirmation(choice, ranking, operational, config, round_count), ranking

    variant = ranking
    gain_name, score_name = 'task_gain', 'task_score'
    if method == 'no_cost':
        _, variant = decide(field, raw, belief, current, round_count=round_count,
                            config=replace(config, flight_weight=0), operational=operational)
    elif method == 'no_occlusion':
        variant = _without_occlusion(belief, ranking)
    elif method == 'generic':
        gain_name, score_name = 'generic_gain', 'generic_score'

    order = variant.generic_order if method == 'generic' else variant.task_order
    if method == 'fixed':
        order = tuple(row.candidate_id for row in variant.candidates if row.viewpoint == current)
        stop = 'VIEW_BUDGET_REACHED' if round_count >= VIEW_BUDGET else None
        next_pose = None if stop else [*current.position_xyz, current.yaw_rad]
    else:
        stop = _stop_reason(variant, order, gain_name, score_name, round_count)
        best = variant.candidates[order[0]] if order else None
        next_pose = None if stop or best is None else [*best.viewpoint.position_xyz, best.viewpoint.yaw_rad]
    best = variant.candidates[order[0]] if order else None
    visibility = None if best is None else dict(candidate_id=best.candidate_id,
                                                shape=list(variant.grid.shape),
                                                cell_ids=np.flatnonzero(variant.visibility[best.candidate_id]).tolist(),
                                                opportunity=variant.observation_opportunity[best.candidate_id][
                                                    variant.visibility[best.candidate_id]].tolist())
    choice.update(stop_reason=stop, next_viewpoint=next_pose, policy_method=method,
                  policy_best_candidate_id=None if best is None else best.candidate_id,
                  policy_best_gain=None if best is None else getattr(best, gain_name),
                  policy_best_score=None if best is None else getattr(best, score_name),
                  policy_order=list(order), policy_candidates=[asdict(row) for row in variant.candidates],
                  policy_visibility=visibility)
    return choice, ranking


def rm4d_top_one(raw):
    """Keep the original RM4D top-1 and ranking fields, with Ground pose aliases."""
    if not raw['candidates']:
        return None
    first = raw['candidates'][0]
    return dict(first, x=first['bunker_x'], y=first['bunker_y'], yaw=first['bunker_yaw'],
                relevance=candidate_relevance(first, FieldConfig()))
