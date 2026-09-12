#!/usr/bin/env python3
"""Read the bounded development attempts; do not rerun/reclassify any task."""
import csv
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
BATCH=ROOT/'outputs/development/confirmation-v1'


def read(path):
    return json.loads(path.read_text()) if path.exists() else {}


def summarize(directory):
    entry=read(directory/'entry.json'); task=read(directory/'task.json')
    attempt=read(directory/'attempt/attempt.json'); data=directory/'attempt/data'
    metrics=read(data/'metrics.json'); physical=read(directory/'attempt/physical_summary.json')
    events=[json.loads(l) for l in (data/'events.jsonl').read_text().splitlines()] if (data/'events.jsonl').exists() else []
    rounds=[]
    for p in sorted((data/'rounds').glob('round-*/decision.json')):
        d=read(p); plan=d.get('completion_plan'); timings=d.get('computation_timing',{})
        ranking=read(p.parent/'ranking.json')
        chosen=next((c for c in ranking.get('candidates',[]) if d['next_viewpoint'] is not None
                     and np.allclose([*c['viewpoint']['position_xyz'],c['viewpoint']['yaw_rad']],
                                     d['next_viewpoint'],rtol=0,atol=1e-9)),None)
        by_id={c['candidate_id']:c for c in ranking.get('candidates',[])}
        handed=next((e for e in events if e.get('state')=='A5_DECISION' and e.get('round')==d['round']),{})
        a=d['assessments'];viable=[q for q in a if not q['operational']['blocked'] and not q['footprint_clipped']]
        rounds.append(dict(window=d['round'],confirmed=d['confirmed_candidate_count'],
            min_missing_cells=min((q['operational']['ground_missing_cells'] for q in viable),default=None),
            candidate_count=len(a),viable=len(viable),
            operational_blocked=sum(q['operational']['blocked'] for q in a),
            raw_grid_blocked=sum(q['occupied_cells']>0 for q in a),
            worker_proposed_next=d['next_viewpoint'],worker_stop=d['stop_reason'],plan=plan,timing=timings,
            handoff_decision_recorded=bool(handed),actual_next=handed.get('next_viewpoint'),actual_stop=handed.get('stop_reason'),
            same_state_ranking=dict(worker_proposal=chosen,ours=by_id.get(ranking.get('best_task_id')),
                                    generic=by_id.get(ranking.get('best_generic_id'))),
            support_states=[dict(source_id=q['source_id'],blocked=q['operational']['blocked'],
                                 clipped=q['footprint_clipped'],confirmed=q['confirmed'],
                                 missing_cells=q['operational']['ground_missing_cells']) for q in a]))
    stages=metrics.get('stages',{})
    start=next((e['ros_time'] for e in events if e.get('state')=='A6_STAGE_START' and e.get('stage')=='ground_navigation'),None)
    end=next((e['ros_time'] for e in events if e.get('state') in ('LIFT','FAILED')),None)
    clocks=[e['ros_time'] for e in events if 'ros_time' in e]
    valid_clock=all(b>=a for a,b in zip(clocks,clocks[1:])) and not metrics.get('clock_reset_detected')
    ground=None if start is None or end is None or end<start or not valid_clock else end-start
    # Actual accepted-window packet poses, with the existing descriptive .2m/.2rad thresholds.
    positions=[]
    for p in sorted(data.glob('observation_*.npz')):
        rank=read(data/'rounds'/'round-01'/'ranking.json')
        if not rank: break
        with np.load(p) as z:
            transform=z['chunk_T_map_sensor'][0] if 'chunk_T_map_sensor' in z else z['T_map_sensor']
            body=transform@np.linalg.inv(np.asarray(rank['sensor']['T_uav_lidar']))
            positions.append([*body[:3,3],float(np.arctan2(body[1,0],body[0,0]))])
    changes=[]
    for a,b in zip(positions,positions[1:]):
        delta=np.asarray(b)-a; yaw=abs((delta[3]+np.pi)%(2*np.pi)-np.pi)
        changes.append(dict(displacement_m=float(np.linalg.norm(delta[:3])),yaw_rad=float(yaw),
                            location_changed=bool(np.linalg.norm(delta[:3])>.2 or yaw>.2)))
    slot=task.get('slots',[{}])[0]
    selected=next((e for e in events if e.get('state')=='A5_SELECTED'),None)
    arrival=next((e for e in events if e.get('state')=='GROUND_ARRIVAL_MEASURED'),None)
    arrival_xy=arrival_yaw=None
    if arrival:
        delta=np.asarray(arrival['actual_pose_map'])-arrival['goal_pose_map']
        arrival_xy=float(np.linalg.norm(delta[:2]))
        arrival_yaw=float(abs((delta[2]+np.pi)%(2*np.pi)-np.pi))
    paths=metrics.get('paths',{})
    row=dict(name=directory.name,scene=slot.get('scene'),method=slot.get('method'),
        seed=attempt.get('seed'),start_wall=entry.get('start_wall'),finish_wall=entry.get('finish_wall'),
        launcher_exit_code=entry.get('exit_code'),
        finished='finish_wall' in entry,status=attempt.get('status'),retrieval=attempt.get('retrieval_success'),
        confirmed=metrics.get('D_env'),D_exec=metrics.get('D_exec'),
        windows=metrics.get('counts',{}).get('completed_windows'),
        location_changes=sum(c['location_changed'] for c in changes) if positions else None,
        active_sim_s=metrics.get('T_active_sim'),ground_sim_s=ground,task_sim_s=metrics.get('T_task_sim'),
        first_confirmation_sim_s=metrics.get('T_first_env_sim'),
        policy_wall_s=sum(r['timing'].get('policy_wall_s',0) for r in rounds) if rounds else None,
        worker_observe_wall_s=sum(r['timing'].get('worker_observe_wall_s',0) for r in rounds) if rounds else None,
        confirmation_solver_wall_s=sum((r['plan'] or {}).get('solver_wall_s',0) for r in rounds) if rounds else None,
        first_terminal_cause=metrics.get('terminal_failure_reason') or attempt.get('reason') or attempt.get('classification_reason') or entry.get('error'),
        failure_stage=metrics.get('terminal_failure_stage'),runtime_versions=task.get('runtime_versions'),
        selected_source=None if selected is None else selected.get('source_id'),
        arrival_xy_error_m=arrival_xy,arrival_yaw_error_rad=arrival_yaw,
        uav_active_distance_m=paths.get('uav_active',{}).get('distance_m') if paths.get('uav_active',{}).get('complete') else None,
        physical_target_lift_m=physical.get('physical_target',{}).get('target_lift_m'),
        human_control_intervention=False,physical=physical,rounds=rounds,paths=paths,
        actual_window_positions=positions,actual_location_transitions=changes)
    for name in ('execution_screen','ground_navigation','ground_refine','refined_pregrasp','descend','close','lift','retention'):
        row[name]=stages.get(name,{}).get('status','NOT_REACHED')
    return row,task


def main():
    manifest=read(ROOT/'configs/confirmation_dev_v1.json')
    records=[];profiles={}
    for directory in sorted(BATCH.glob('slot-*')):
        if not (directory/'entry.json').exists(): continue
        row,task=summarize(directory);records.append(row)
        profiles[row['name']]={k:v for k,v in task.items() if k not in ('slots','runtime_versions')}
    pairs=[]
    for scene in dict.fromkeys(s['scene'] for s in manifest['slots']):
        rows=[r for r in records if r['scene']==scene and r['finished']]
        pair=dict(scene=scene,complete=len(rows)==2 and {r['method'] for r in rows}=={'ours','confirmation'})
        if pair['complete']:
            pair.update(same_versions=rows[0]['runtime_versions']==rows[1]['runtime_versions'],
                same_profile=profiles[rows[0]['name']]==profiles[rows[1]['name']])
            pair['outcomes']={r['method']:r['retrieval'] for r in rows}
        pairs.append(pair)
    summary=dict(development_only=True,planned=manifest['planned_tasks'],starts=len(records),
        completed=sum(r['finished'] for r in records),pairs=pairs,records=records,
        caution='Old scenes are development; no statistical superiority or off-policy retrieval claim.')
    if records and all(r['finished'] for r in records):
        summary['batch_elapsed_wall_s']=max(r['finish_wall'] for r in records)-min(r['start_wall'] for r in records)
    out=Path(__file__).parent/'online_results';out.mkdir(parents=True,exist_ok=True)
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    flat=[{k:v for k,v in r.items() if not isinstance(v,(dict,list))} for r in records]
    if flat:
        with (out/'tasks.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(flat[0]),lineterminator='\n');writer.writeheader();writer.writerows(flat)
    print(json.dumps(dict(completed=summary['completed'],starts=len(records),pairs=pairs),indent=2))


if __name__=='__main__':main()
