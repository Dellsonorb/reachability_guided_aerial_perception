#!/usr/bin/env python3
"""Reuse existing extraction; new four runs versus preserved development references."""
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'src'))
from analysis.confirmation_development.summarize_online import summarize,read
from analysis.completion_aware.offline import write_csv
from analysis.confirmation_factors.analyze import (read_npz,eligible,pose_array,plan_confirmation,windows_first)
from analysis.completion_aware.offline import commanded_index,transition_calibration


def main():
    records=[];comparisons=[]
    old=read(ROOT/'analysis/confirmation_development/online_results/summary.json')['records']
    for p in sorted((ROOT/'outputs/development/confirmation-factors').glob('slot-*')):
        if not (p/'entry.json').exists():continue
        r,task=summarize(p)
        if not r['finished']:continue
        r.pop('confirmation_solver_wall_s')
        initial=read(p/'attempt/data/initial.json')
        prior=None
        r['actual_transition_calibration']=[]
        for w in r['rounds']:
            folder=p/'attempt/data/rounds'/f"round-{w['window']:02d}"
            d=read(folder/'decision.json')
            w['deficit_plan']=d.get('deficit_plan')
            rank=read(folder/'ranking.json');q=eligible(d['assessments'])
            h=read_npz(folder/'operational_evidence.npz')['ground_presence_votes'].ravel()
            fields=read_npz(folder/'fields.npz')
            opportunity=fields['observation_opportunity'].reshape(len(rank['candidates']),-1)
            args=dict(poses=pose_array(rank),valid=[c['status']=='VALID'for c in rank['candidates']],
                first_cost=[c['flight_cost']for c in rank['candidates']],remaining=max(0,3-w['window']),
                offsets=rank['config']['xy_offsets_m'],yaw_samples=rank['config']['yaw_samples'],
                yaw_cost=rank['config']['yaw_cost_m_per_rad'],facade_tolerance=initial['config']['facade_position_tolerance'])
            supports=[a['operational']['covered_cells']for a in q]
            phase=plan_confirmation(h,supports,opportunity,**args)
            wf=windows_first(h,supports,opportunity,args)
            w['posthoc_same_state_factors']=dict(phase_status=phase['status'],phase_views=phase['view_ids'],
                windows_status=wf['status'],windows_views=wf['view_ids'],not_executed_counterfactual=True)
            current=(rank,d,h,fields)
            if prior is not None:
                previous,previous_row=prior
                actual=commanded_index(previous[0],dict(next_viewpoint=previous_row['actual_next']))
                if actual is not None:
                    calibration=transition_calibration(previous,current,actual)
                    by_id={a['source_id']:a for a in d['assessments']}
                    prediction=previous[3]['observation_opportunity'].reshape(len(previous[0]['candidates']),-1)[actual]
                    selected=set((previous_row['deficit_plan']or{}).get('maximizing_source_ids',[]))
                    details=[]
                    for a in previous[1]['assessments']:
                        if a['source_id'] not in selected:continue
                        cells=a['operational']['covered_cells'];now=by_id[a['source_id']]
                        details.append(dict(source_id=a['source_id'],next_blocked=now['operational']['blocked'],
                            next_clipped=now['footprint_clipped'],next_confirmed=now['confirmed'],
                            cells_still_missing=[dict(cell=int(x),prior_votes=int(previous[2][x]),
                                next_votes=int(h[x]),predicted_opportunity=float(prediction[x]))
                                for x in cells if h[x]<2]))
                    r['actual_transition_calibration'].append(dict(from_window=previous_row['window'],
                        actual_view_id=actual,**calibration,selected_supports=details))
            prior=current,w
        r['deficit_solver_wall_s']=sum((w['deficit_plan']or{}).get('solver_wall_s',0)for w in r['rounds'])
        r['reference_scope']='cross_batch_development_shared_components_not_randomized_same_commit_pair'
        records.append(r)
        matches=[x for x in old if x['scene']==r['scene']]
        profile=lambda t:{k:v for k,v in t.items()if k not in ('slots','runtime_versions')}
        for x in matches:
            t=read(ROOT/'outputs/development/confirmation-v1'/x['name']/'task.json')
            comparisons.append(dict(scene=r['scene'],reference_method=x['method'],
                same_profile=profile(task)==profile(t),same_runtime_commits=task['runtime_versions']==t['runtime_versions'],
                reference_name=x['name'],current_name=r['name'],reference_success=x['retrieval'],current_success=r['retrieval'],
                reference_confirmed=x['confirmed'],current_confirmed=r['confirmed'],
                reference_D_exec=x['D_exec'],current_D_exec=r['D_exec'],
                reference_windows=x['windows'],current_windows=r['windows']))
    summary=dict(planned=4,starts=len(list((ROOT/'outputs/development/confirmation-factors').glob('slot-*/entry.json'))),
        completed=len(records),development_only=True,records=records,references=old,comparisons=comparisons)
    if records:
        summary['elapsed_to_last_finished_wall_s']=max(r['finish_wall']for r in records)-min(r['start_wall']for r in records)
    out=Path(__file__).parent/'online_results';out.mkdir(parents=True,exist_ok=True)
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    write_csv(out/'tasks.csv',[{k:v for k,v in r.items()if not isinstance(v,(dict,list))}for r in records])
    print(json.dumps(dict(completed=len(records),starts=summary['starts'],
        cases=[{k:r[k]for k in ('scene','confirmed','D_exec','retrieval','windows')}for r in records]),indent=2))


if __name__=='__main__':main()
