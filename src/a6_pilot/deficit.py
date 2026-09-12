"""Explicit one-window candidate deficit control; never a belief updater."""
import time
import numpy as np


def plan_deficit(h, supports, opportunity, *, valid, first_cost, remaining):
    started=time.perf_counter()
    h=np.asarray(h).ravel();w=np.asarray(opportunity,float)
    valid=np.asarray(valid,bool);cost=np.asarray(first_cost,float)
    if (remaining not in (0,1,2) or w.ndim!=2 or w.shape[1]!=len(h)
        or not np.all(np.isfinite(h)&(h>=0)&(h==np.floor(h)))
        or not np.all(np.isfinite(w)&(w>=0)&(w<=1))
        or valid.shape!=(len(w),) or cost.shape!=(len(w),)
        or not np.all(np.isfinite(cost)&(cost>=0))):
        raise ValueError('invalid deficit counts/opportunities/view catalogue')
    cells=[np.asarray(s,int) for s in supports]
    if any(s.ndim!=1 or not len(s) or np.any((s<0)|(s>=len(h))) for s in cells):
        raise ValueError('each support must contain valid grid cell indices')
    d=np.maximum(0,2-h.astype(float))
    feasible=[i for i,s in enumerate(cells) if np.all(d[s]<=remaining)]
    result=dict(version='candidate-deficit-v1',remaining_windows=int(remaining),view_ids=[],
        maximizing_candidate_indices=[],progress=None,vote_budget_possible_candidates=len(feasible),
        predicted_votes_are_measurements=False,semantics='nominal_fractional_missing_tickets_not_completion_probability')
    def finish(status):
        result.update(status=status,solver_wall_s=time.perf_counter()-started)
        return result
    if not cells:return finish('NO_CURRENT_SUPPORT')
    if any(np.all(d[s]==0)for s in cells):return finish('ALREADY_CONFIRMED')
    if not remaining:return finish('VIEW_BUDGET_REACHED')
    if not feasible:return finish('SUPPORT_BUDGET_IMPOSSIBLE')
    if not valid.any():return finish('NO_NOMINAL_PROGRESS')
    # Max over candidates, not over the union of their footprint cells.
    progress=np.array([w[:,cells[i][d[cells[i]]>0]].sum(axis=1)/d[cells[i]].sum() for i in feasible])
    score=progress.max(axis=0)
    best=int(min(np.flatnonzero(valid),key=lambda i:(-score[i],cost[i],i)))
    if score[best]<=0:return finish('NO_NOMINAL_PROGRESS')
    result.update(view_ids=[best],progress=float(score[best]),
                  maximizing_candidate_indices=[feasible[i]for i in np.flatnonzero(progress[:,best]==score[best])])
    return finish('NOMINAL_DEFICIT_PROGRESS')


def apply_deficit(choice, ranking, operational, config, round_count):
    if operational is None or operational.ground_presence_votes is None:
        raise ValueError('deficit requires real operational ground-presence votes')
    if (choice.get('anchor_semantics')!='exact-validated-winner-v1.2'
        or ranking.acquisition_metadata.get('model')!='finite_scan_v1'
        or operational.config.free_observations!=2):
        raise ValueError('deficit requires exact winners, finite scan and two ground votes')
    eligible=[a for a in choice['assessments']if not a['footprint_clipped'] and not a['operational']['blocked']]
    views=ranking.candidates
    plan=plan_deficit(operational.ground_presence_votes,[a['operational']['covered_cells']for a in eligible],
        ranking.observation_opportunity.reshape(len(views),-1),valid=[v.status=='VALID'for v in views],
        first_cost=[v.flight_cost for v in views],remaining=max(0,config.max_viewpoints-round_count))
    plan.update(maximizing_source_ids=[eligible[i]['source_id']for i in plan['maximizing_candidate_indices']],
                ours_next_viewpoint=choice['next_viewpoint'],ours_stop_reason=choice['stop_reason'],
                fallback_to_ours=plan['status']in ('NO_CURRENT_SUPPORT','NO_NOMINAL_PROGRESS','ALREADY_CONFIRMED'))
    updated=dict(choice,policy_method='deficit',deficit_plan=plan)
    if plan['view_ids']:
        v=views[plan['view_ids'][0]].viewpoint
        updated.update(stop_reason=None,next_viewpoint=[*v.position_xyz,v.yaw_rad])
    elif plan['status']in ('SUPPORT_BUDGET_IMPOSSIBLE','VIEW_BUDGET_REACHED'):
        updated.update(stop_reason=plan['status'],next_viewpoint=None)
    return updated
