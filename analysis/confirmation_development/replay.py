#!/usr/bin/env python3
"""Exercise the NEW policy on all retained states, without modifying old data."""
import csv
import json
from pathlib import Path
from types import SimpleNamespace as NS
from collections import Counter
import numpy as np
from a6_pilot.confirmation import apply_confirmation
from sim_active_perception.core import A5Config
from reachability_guided_nbv.model import Viewpoint,NBVConfig


def main():
    root=Path('outputs/paper1-final-eval-v1')
    output=Path(__file__).parent/'replay_results'
    rows=list(csv.DictReader((root/'slots.csv').open()))
    result=[]
    for row in rows:
        data=root/row['selected_attempt']/'data'
        if not int(row['windows']): continue
        initial=json.loads((data/'initial.json').read_text())
        config=A5Config(**initial['config'])
        events=[json.loads(l) for l in (data/'events.jsonl').read_text().splitlines()]
        commanded={e['round']:e['next_viewpoint'] for e in events if e.get('state')=='A5_DECISION'}
        for n in range(1,int(row['windows'])+1):
            p=data/'rounds'/f'round-{n:02d}'
            rank=json.loads((p/'ranking.json').read_text()); choice=json.loads((p/'decision.json').read_text())
            with np.load(p/'fields.npz') as z: w=z['observation_opportunity'].copy()
            with np.load(p/'operational_evidence.npz') as z: h=z['ground_presence_votes'].copy()
            baseline_before=h.copy()
            views=[NS(candidate_id=v['candidate_id'],viewpoint=Viewpoint(**v['viewpoint']),status=v['status'],flight_cost=v['flight_cost']) for v in rank['candidates']]
            ranking=NS(candidates=views,observation_opportunity=w,config=NBVConfig(**rank['config']),acquisition_metadata=rank['acquisition'])
            best=None if rank['best_task_id'] is None else rank['candidates'][rank['best_task_id']]
            stop=(rank['status'] if rank['status']!='RANKED' else 'NONPOSITIVE_SCORE' if best['task_score']<=0 else 'VIEW_BUDGET_REACHED' if n>=3 else None)
            old_view=None if stop or best is None else best['viewpoint']['position_xyz']+[best['viewpoint']['yaw_rad']]
            # Supply original Ours output, regardless of which method collected this state.
            choice=dict(choice,stop_reason=stop,next_viewpoint=old_view)
            new=apply_confirmation(choice,ranking,NS(ground_presence_votes=h,config=NS(**rank['a2_config'])),config,n)
            np.testing.assert_array_equal(h,baseline_before)
            plan=new['completion_plan']
            if plan['view_ids']:
                assert len(plan['view_ids'])<=3-n
                hit=w.reshape(len(w),-1)==1 if plan['nominal_tier']==2 else w.reshape(len(w),-1)>0
                lookup={a['source_id']:a for a in choice['assessments']}
                for sid in plan['supported_source_ids']:
                    a=lookup[sid];s=a['operational']['covered_cells']
                    assert not a['operational']['blocked'] and not a['footprint_clipped']
                    assert np.all(h.ravel()[s]+hit[plan['view_ids']][:,s].sum(axis=0)>=2)
            result.append(dict(slot=int(row['slot']),scene=row['scene'],method=row['method'],role=row['comparison_role'],
                historical_retrieval=row['retrieval_success']=='True',round=n,already_confirmed=choice['confirmed_candidate_count']>0,
                historical_command=commanded.get(n),ours_command=old_view,new_command=new['next_viewpoint'],
                new_stop=new['stop_reason'],changed_from_same_state_ours=new['next_viewpoint']!=old_view,
                new_vs_one_step_different=bool(plan['view_ids']) and plan['view_ids'][0]!=plan['one_step_deficit_view_id'],
                completion_plan=plan))
    output.mkdir(parents=True,exist_ok=True)
    (output/'states.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in result))
    summary=dict(tasks_accounted=len(rows),states=len(result),primary_groups={})
    predicates=dict(ours_success=lambda r:r['method']=='ours' and r['historical_retrieval'],
        ours_failure=lambda r:r['method']=='ours' and not r['historical_retrieval'],
        generic=lambda r:r['method']=='generic')
    for name,keep in predicates.items():
        group=[r for r in result if r['role']=='primary' and keep(r)]
        rounds={}
        for n in (1,2,3):
            part=[r for r in group if r['round']==n]
            rounds[n]=dict(states=len(part),status=dict(Counter(r['completion_plan']['status'] for r in part)),
                changed_from_ours=sum(r['changed_from_same_state_ours'] for r in part),
                differs_from_one_step=sum(r['new_vs_one_step_different'] for r in part),
                solver_wall_s_max=max((r['completion_plan']['solver_wall_s'] for r in part),default=None))
        summary['primary_groups'][name]=rounds
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__': main()
