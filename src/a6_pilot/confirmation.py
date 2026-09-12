"""Opt-in candidate-confirmation planning, never an environment belief updater.

The horizon is at most two windows in the current shared catalogue. All-phase
and optimistic labels refer only to the fixed nominal sensor/occlusion model.
"""
import time
import numpy as np


def plan_confirmation(h, supports, opportunity, *, poses, valid, first_cost,
                      remaining, offsets, yaw_samples, yaw_cost, facade_tolerance):
    started = time.perf_counter()
    h, w = np.asarray(h).ravel(), np.asarray(opportunity, dtype=float)
    poses, valid, first_cost = np.asarray(poses, float), np.asarray(valid, bool), np.asarray(first_cost, float)
    if (remaining not in (0,1,2) or w.ndim!=2 or w.shape[1]!=len(h)
            or not np.all(np.isfinite(h) & (h>=0) & (h==np.floor(h)))
            or not np.all(np.isfinite(w) & (w>=0) & (w<=1))
            or poses.shape!=(len(w),4) or not np.isfinite(poses).all()
            or valid.shape!=(len(w),) or first_cost.shape!=(len(w),)
            or not np.all(np.isfinite(first_cost) & (first_cost>=0))):
        raise ValueError('invalid completion counts/opportunities/view catalogue')
    cells = [np.asarray(s, dtype=int) for s in supports]
    if any(s.ndim!=1 or not len(s) or np.any((s<0)|(s>=len(h))) for s in cells):
        raise ValueError('each support must contain valid grid cell indices')
    deficits = np.maximum(0, 2-h)
    feasible = [i for i,s in enumerate(cells) if np.all(deficits[s]<=remaining)]
    result = dict(version='candidate-confirmation-v1', status=None, remaining_windows=int(remaining),
        view_ids=[], nominal_tier=0, supported_candidate_indices=[], total_flight_cost=None,
        vote_budget_possible_candidates=len(feasible), one_step_deficit_view_id=None,
        one_step_deficit_reduction=None, predicted_votes_are_measurements=False,
        search_scope='current_catalogue_local_transition_subgraph',
        guarantee_scope='fixed_nominal_phase_envelopes_not_real_return_probability')

    def finish(status):
        result.update(status=status, solver_wall_s=time.perf_counter()-started)
        return result

    if not cells: return finish('NO_CURRENT_SUPPORT')
    if any(np.all(deficits[s]==0) for s in cells): return finish('ALREADY_CONFIRMED')
    if not remaining: return finish('VIEW_BUDGET_REACHED')
    if not feasible: return finish('SUPPORT_BUDGET_IMPOSSIBLE')
    if not valid.any(): return finish('NO_NOMINAL_PLAN')

    # Diagnostic only: most fractional missing-ticket progress for any one
    # viable candidate, then common travel cost and stable view ID. No rollout.
    progress = np.zeros(len(w))
    for s in cells:
        total = deficits[s].sum()
        if total:
            progress = np.maximum(progress, w[:,s[deficits[s]>0]].sum(axis=1)/total)
    ids = np.flatnonzero(valid)
    greedy = min(ids, key=lambda i: (-progress[i],first_cost[i],i))
    result.update(one_step_deficit_view_id=int(greedy), one_step_deficit_reduction=float(progress[greedy]))

    displacement = poses[None,:,:3]-poses[:,None,:3]
    distance = np.linalg.norm(displacement,axis=2)
    yaw = (poses[None,:,3]-poses[:,None,3]+np.pi)%(2*np.pi)-np.pi
    offsets = np.asarray(offsets)
    allowed = (np.isclose(displacement[:,:,0,None],offsets,rtol=0,atol=1e-8).any(axis=2)
        & np.isclose(displacement[:,:,1,None],offsets,rtol=0,atol=1e-8).any(axis=2)
        & np.isclose(displacement[:,:,2],0,rtol=0,atol=1e-8)
        & np.isclose(yaw*yaw_samples/(2*np.pi),np.rint(yaw*yaw_samples/(2*np.pi)),rtol=0,atol=1e-8)
        & (distance>facade_tolerance))
    np.fill_diagonal(allowed,True)
    allowed &= valid[:,None]&valid[None,:]
    pair_cost = first_cost[:,None]+distance+yaw_cost*np.abs(yaw)
    result['allowed_two_step_sequences'] = int(allowed.sum()) if remaining==2 else 0
    for tier, hit in ((2,w==1),(1,w>0)):
        one = np.zeros(len(w),bool)
        two = np.zeros((len(w),len(w)),bool) if remaining==2 else None
        for i in feasible:
            s=cells[i]; zero=s[h[s]==0]; single=s[h[s]==1]
            if not len(zero): one |= hit[:,single].all(axis=1)
            if two is not None:
                both=hit[:,zero].all(axis=1)
                missing=(~hit[:,single]).astype(np.int32)
                two |= both[:,None]&both[None,:]&(missing@missing.T==0)
        first=np.flatnonzero(one&valid)
        if len(first):
            selected=[int(min(first,key=lambda i:(first_cost[i],i)))]
            cost=first_cost[selected[0]]
        elif two is not None and np.any(two&allowed):
            selected=list(map(int,min(np.argwhere(two&allowed),key=lambda ij:(pair_cost[tuple(ij)],int(ij[0]),int(ij[1])))))
            cost=pair_cost[tuple(selected)]
        else:
            continue
        supported=[i for i,s in enumerate(cells) if np.all(h[s]+hit[selected][:,s].sum(axis=0)>=2)]
        result.update(view_ids=selected, nominal_tier=tier, total_flight_cost=float(cost),
                      supported_candidate_indices=supported)
        return finish('ALL_PHASE_NOMINAL_PLAN' if tier==2 else 'OPTIMISTIC_NOMINAL_PLAN')
    return finish('NO_NOMINAL_PLAN')


def apply_confirmation(choice, ranking, operational, config, round_count):
    """Adapt only the explicit variant; shared Ours choice/ranking stay intact."""
    if operational is None or operational.ground_presence_votes is None:
        raise ValueError('confirmation requires real operational ground-presence votes')
    if (choice.get('anchor_semantics')!='exact-validated-winner-v1.2'
            or ranking.acquisition_metadata.get('model')!='finite_scan_v1'
            or operational.config.free_observations!=2):
        raise ValueError('confirmation requires exact winners, finite scan and two ground votes')
    candidates=[a for a in choice['assessments'] if not a['footprint_clipped'] and not a['operational']['blocked']]
    views=ranking.candidates
    plan=plan_confirmation(operational.ground_presence_votes,
        [a['operational']['covered_cells'] for a in candidates],
        ranking.observation_opportunity.reshape(len(views),-1),
        poses=[[*v.viewpoint.position_xyz,v.viewpoint.yaw_rad] for v in views],
        valid=[v.status=='VALID' for v in views], first_cost=[v.flight_cost for v in views],
        remaining=max(0,config.max_viewpoints-round_count), offsets=ranking.config.xy_offsets_m,
        yaw_samples=ranking.config.yaw_samples, yaw_cost=ranking.config.yaw_cost_m_per_rad,
        facade_tolerance=config.facade_position_tolerance)
    plan['supported_source_ids']=[candidates[i]['source_id'] for i in plan['supported_candidate_indices']]
    plan['ours_next_viewpoint']=choice['next_viewpoint']
    plan['ours_stop_reason']=choice['stop_reason']
    plan['fallback_to_ours']=plan['status'] in ('NO_NOMINAL_PLAN','NO_CURRENT_SUPPORT','ALREADY_CONFIRMED')
    updated=dict(choice,policy_method='confirmation',completion_plan=plan)
    if plan['view_ids']:
        view=views[plan['view_ids'][0]].viewpoint
        updated.update(stop_reason=None,next_viewpoint=[*view.position_xyz,view.yaw_rad])
    elif plan['status'] in ('SUPPORT_BUDGET_IMPOSSIBLE','VIEW_BUDGET_REACHED'):
        updated.update(stop_reason=plan['status'],next_viewpoint=None)
    return updated
