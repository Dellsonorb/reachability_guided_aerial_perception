#!/usr/bin/env python3
"""Summarize only completed tasks in this bounded development batch."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def read(path):
    return json.loads(path.read_text()) if path.exists() else {}


def main():
    records = []
    profiles = {}
    for directory in sorted((HERE.parent / 'finite-scan').glob('launch-*')):
        task = read(directory / 'task.json')
        attempt = read(directory / 'attempt/attempt.json')
        if not attempt.get('finish_wall'):
            continue
        metrics = read(directory / 'attempt/data/metrics.json')
        physical = read(directory / 'attempt/physical_summary.json')
        events_path = directory / 'attempt/data/events.jsonl'
        events = [json.loads(line) for line in events_path.read_text().splitlines()] if events_path.exists() else []
        selected = next((e for e in events if e['state'] == 'A5_SELECTED'), {})
        screens = [e for e in events if e['state'] == 'GROUND_EXECUTION_SCREEN']
        rounds = []
        for path in sorted((directory / 'attempt/data/rounds').glob('round-*/decision.json')):
            decision = read(path)
            rank = read(path.parent / 'ranking.json')
            assessments = decision.get('assessments', [])
            by_id = {c['candidate_id']: c for c in rank.get('candidates', [])}
            viable = [c for c in assessments if not c.get('operational', {}).get('blocked', True) and not c['footprint_clipped']]
            rounds.append(dict(round=path.parent.name,
                candidate_count=len(assessments),
                raw_grid_blocked=sum(c['occupied_cells'] > 0 for c in assessments),
                operational_blocked=sum(c.get('operational', {}).get('blocked', False) for c in assessments),
                confirmed=sum(c['confirmed'] for c in assessments),
                min_missing_real_ground_cells=min((c['operational']['ground_missing_cells'] for c in viable), default=None),
                operational_summary=read(path.parent / 'operational_summary.json'),
                best_generic=by_id.get(rank.get('best_generic_id')),
                best_task=by_id.get(rank.get('best_task_id')),
                acquisition=rank.get('acquisition')))
        takeoff = metrics.get('stages', {}).get('takeoff', {}).get('start_sim')
        landed = metrics.get('landed_sim')
        profiles[directory.name] = {k: v for k, v in task.items() if k not in ('slots', 'runtime_versions')}
        records.append(dict(name=directory.name, path=str(directory.relative_to(ROOT)),
            method=attempt['method'], scene=attempt['scene'], seed=attempt['seed'],
            runtime_versions=task.get('runtime_versions'), raw_status=attempt['status'],
            retrieval_success=attempt.get('retrieval_success'), D_exec=metrics.get('D_exec'),
            confirmed_by_window=[r['confirmed'] for r in rounds],
            terminal_reason=attempt.get('classification_reason'),
            counts=metrics.get('counts'), stages=metrics.get('stages'), paths=metrics.get('paths'),
            first_confirmation_active_sim_s=metrics.get('T_first_env_sim'),
            active_sim_s=metrics.get('T_active_sim'), task_sim_s=metrics.get('T_task_sim'),
            takeoff_to_landing_sim_s=None if takeoff is None or landed is None else landed-takeoff,
            selected_source_id=selected.get('source_id'),
            selected_pose=[selected[k] for k in ('x', 'y', 'yaw')] if selected else None,
            candidate_fallback_observed=any(s['rank'] > 1 for s in screens) if screens else None,
            screens=screens, rounds=rounds,
            camera_events=[e for e in events if e['state'] in ('GROUND_VIEW_REJECTED', 'GROUND_VIEW_PLAN')],
            physical=physical, human_control_intervention=False))
    pairs = []
    for scene in ('paired-hard-01', 'paired-moderate-01'):
        pair = [r for r in records if r['scene'] == scene]
        if len(pair) == 2:
            assert {r['method'] for r in pair} == {'generic', 'ours'}
            assert pair[0]['runtime_versions'] == pair[1]['runtime_versions']
            assert profiles[pair[0]['name']] == profiles[pair[1]['name']]
            pairs.append([r['name'] for r in pair])
    result = dict(development_only=True, completed_starts=len(records), online_start_cap=6,
        original_results_preserved=True, formal_statistics=False,
        same_version_pairs=pairs, baseline_start1_precedes_finite_model=True, records=records)
    (HERE / 'task-results.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(completed_starts=len(records), checked_retrievals=sum(r['retrieval_success'] is True for r in records),
                         same_version_pairs=len(pairs), formal=False)))


if __name__ == '__main__':
    main()
