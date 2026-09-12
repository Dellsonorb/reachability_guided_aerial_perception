#!/usr/bin/env python3
"""Read-only factor comparison. Future sensor states are calibration labels only."""
import csv
import json
import sys
from pathlib import Path
from collections import Counter
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from analysis.completion_aware.offline import (read_json,read_npz,eligible,pose_array,
    commanded_index,transition_calibration,write_csv)
from a6_pilot.confirmation import plan_confirmation
from a6_pilot.deficit import plan_deficit


def windows_first(h,s,w,args):
    """Same core/graph/envelopes; only horizon precedes phase preference."""
    if args['remaining']==2:
        one=plan_confirmation(h,s,w,**dict(args,remaining=1))
        if one['view_ids']:return one
    return plan_confirmation(h,s,w,**args)


def effective_action(plan,ours_action):
    if plan['view_ids']:return plan['view_ids'][0]
    if plan['status']in ('SUPPORT_BUDGET_IMPOSSIBLE','VIEW_BUDGET_REACHED'):return None
    return ours_action


def tasks():
    root=ROOT/'outputs/paper1-final-eval-v1'
    rows=list(csv.DictReader((root/'slots.csv').open()))
    for r in rows:
        yield dict(cohort='old212',task=r['slot'],scene=r['scene'],method=r['method'],
                   success=r['retrieval_success']=='True',role=r['comparison_role']),root/r['selected_attempt']/'data'
    for p in sorted((ROOT/'outputs/development/confirmation-v1').glob('slot-*')):
        a=read_json(p/'attempt/attempt.json')
        yield dict(cohort='online_v1',task=p.name,scene=a['scene'],method=a['method'],
                   success=a['retrieval_success'],role='development'),p/'attempt/data'


def main():
    states=[];transitions=[];accounting=[]
    for meta,data in tasks():
        paths=sorted((data/'rounds').glob('round-*/decision.json'))
        initial=read_json(data/'initial.json') if paths else None
        accounting.append(dict(meta,snapshots=len(paths)))
        events=[json.loads(l)for l in (data/'events.jsonl').read_text().splitlines()]
        commanded={e['round']:e for e in events if e.get('state')=='A5_DECISION'}
        prior=None
        for path in paths:
            p=path.parent;d=read_json(path);r=read_json(p/'ranking.json');n=d['round']
            fields=read_npz(p/'fields.npz');h=read_npz(p/'operational_evidence.npz')['ground_presence_votes'].ravel()
            w=fields['observation_opportunity'].reshape(len(r['candidates']),-1);q=eligible(d['assessments'])
            supports=[a['operational']['covered_cells']for a in q]
            args=dict(poses=pose_array(r),valid=np.array([c['status']=='VALID'for c in r['candidates']]),
                first_cost=np.array([c['flight_cost']for c in r['candidates']]),remaining=max(0,3-n),
                offsets=r['config']['xy_offsets_m'],yaw_samples=r['config']['yaw_samples'],
                yaw_cost=r['config']['yaw_cost_m_per_rad'],facade_tolerance=initial['config']['facade_position_tolerance'])
            old=plan_confirmation(h,supports,w,**args);wf=windows_first(h,supports,w,args)
            one=plan_deficit(h,supports,w,**{k:args[k]for k in ('valid','first_cost','remaining')})
            actual=commanded_index(r,commanded.get(n))
            best=r['best_task_id']
            ours_action=(best if r['status']=='RANKED' and best is not None
                         and r['candidates'][best]['task_score']>0 and n<3 else None)
            row=dict(meta,round=n,prior_confirmed=bool(d['confirmed_candidate_count']),
                actual_view=actual,ours_view=r['best_task_id'],generic_view=r['best_generic_id'],
                phase_status=old['status'],phase_views=old['view_ids'],phase_tier=old['nominal_tier'],
                phase_sources=[q[i]['source_id']for i in old['supported_candidate_indices']],
                windows_status=wf['status'],windows_views=wf['view_ids'],windows_tier=wf['nominal_tier'],
                deficit_status=one['status'],deficit_views=one['view_ids'],deficit_progress=one['progress'],
                legacy_deficit_view=old['one_step_deficit_view_id'],
                deficit_sources=[q[i]['source_id']for i in one['maximizing_candidate_indices']],
                phase_solver_s=old['solver_wall_s'],deficit_solver_s=one['solver_wall_s'])
            row.update(phase_action=effective_action(old,ours_action),windows_action=effective_action(wf,ours_action),
                       deficit_action=effective_action(one,ours_action),ours_action=ours_action)
            if len(old['view_ids'])==2:
                hit=w==1 if old['nominal_tier']==2 else w>0
                repeated=np.zeros(len(w),bool)
                for s in supports:repeated|=np.all(h[s]+2*hit[:,s]>=2,axis=1)
                ids=np.flatnonzero(repeated & args['valid'])
                row['same_tier_repeat_exists']=bool(len(ids))
                row['same_tier_repeat_min_cost']=float(args['first_cost'][ids].min()) if len(ids) else None
                row['phase_cost']=old['total_flight_cost']
            states.append(row)
            current=(r,d,h,fields)
            if prior is not None:
                previous,previous_row=prior
                if previous_row['actual_view'] is not None:
                    c=transition_calibration(previous,current,previous_row['actual_view'])
                    transitions.append(dict(meta,from_round=n-1,prior_confirmed=previous_row['prior_confirmed'],**c))
                    # Only assess an original two-step forecast when its first
                    # action was actually commanded; no off-policy next-state claim.
                    if len(previous_row['phase_views'])==2 and previous_row['phase_views'][0]==previous_row['actual_view']:
                        lookup={a['source_id']:a for a in d['assessments']}
                        previous_row['matched_first_action_rollout']=dict(next_status=old['status'],
                            next_confirmed=bool(d['confirmed_candidate_count']),
                            sources=[dict(source_id=s,blocked=lookup[s]['operational']['blocked'],
                                missing_cells=lookup[s]['operational']['ground_missing_cells'],
                                zero_cells=int(np.count_nonzero(h[lookup[s]['operational']['covered_cells']]==0)))
                                for s in previous_row['phase_sources']])
            prior=current,row
        if len(accounting)%40==0:print('tasks read',len(accounting),flush=True)
    summary=dict(tasks=len(accounting),states=len(states),transitions=len(transitions),groups={})
    for cohort,method,success in sorted({(r['cohort'],r['method'],r['success'])for r in states}):
        base=[r for r in states if (r['cohort'],r['method'],r['success'])==(cohort,method,success)]
        for n in (1,2):
            group=[r for r in base if r['round']==n and not r['prior_confirmed']]
            if not group:continue
            name=f'{cohort}/{method}/{success}/round{n}'
            summary['groups'][name]=dict(states=len(group),phase_status=dict(Counter(r['phase_status']for r in group)),
                phase_vs_windows_first=sum(r['phase_action']!=r['windows_action']for r in group),
                phase_vs_deficit=sum(r['phase_action']!=r['deficit_action']for r in group),
                raw_phase_vs_deficit_proposal=sum(r['phase_views'][:1]!=r['deficit_views'][:1]for r in group),
                deficit_vs_legacy=sum((r['deficit_views']or[None])[0]!=r['legacy_deficit_view']for r in group),
                phase_2_to_windows_1=sum(len(r['phase_views'])==2 and len(r['windows_views'])==1 for r in group),
                phase_matches_command=sum(r['phase_action']==r['actual_view']for r in group),
                deficit_matches_command=sum(r['deficit_action']==r['actual_view']for r in group))
    summary['calibration']={}
    for cohort,method in sorted({(t['cohort'],t['method'])for t in transitions}):
        group=[t for t in transitions if (t['cohort'],t['method'])==(cohort,method)and not t['prior_confirmed']]
        predicted=[t for t in group if t['predicted_confirmation_all_phase']]
        summary['calibration'][f'{cohort}/{method}']=dict(transitions=len(group),
            all_phase_confirmation_predictions=len(predicted),
            actual_confirmation_when_predicted=sum(t['actual_confirmation']for t in predicted),
            actual_prior_support_when_predicted=sum(t['actual_prior_candidate_support_complete']for t in predicted),
            needed_predicted_hits=sum(t['needed_all_phase_hits']for t in group),
            needed_predicted_misses=sum(t['needed_all_phase_misses']for t in group))
    out=Path(__file__).parent/'offline_results';out.mkdir(parents=True,exist_ok=True)
    (out/'states.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n'for r in states))
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    write_csv(out/'transitions.csv',transitions);write_csv(out/'accounting.csv',accounting)
    print(json.dumps({k:v for k,v in summary.items()if k!='groups'},indent=2))


if __name__=='__main__':main()
