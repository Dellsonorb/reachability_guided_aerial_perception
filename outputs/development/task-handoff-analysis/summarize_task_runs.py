#!/usr/bin/env python3
"""Descriptive projection of this four-start development stage; no reruns."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def read(path):
    return json.loads(path.read_text())


def main():
    records, profiles = [], []
    for directory in sorted((HERE.parent / 'task-handoff').glob('launch-*')):
        task = read(directory / 'task.json')
        attempt = read(directory / 'attempt/attempt.json')
        metrics = read(directory / 'attempt/data/metrics.json')
        physical = read(directory / 'attempt/physical_summary.json')
        events = [json.loads(line) for line in (directory / 'attempt/data/events.jsonl').read_text().splitlines()]
        assert attempt.get('finish_wall') is not None
        selected = next((e for e in events if e['state'] == 'A5_SELECTED'), {})
        screens = [e for e in events if e['state'] == 'GROUND_EXECUTION_SCREEN']
        rankings = []
        for path in sorted((directory / 'attempt/data/rounds').glob('round-*/ranking.json')):
            rank = read(path)
            by_id = {c['candidate_id']: c for c in rank['candidates']}
            rankings.append(dict(round=path.parent.name,
                best_generic=by_id.get(rank['best_generic_id']),
                best_task=by_id.get(rank['best_task_id'])))
        takeoff = metrics['stages']['takeoff']['start_sim']
        landed = metrics.get('landed_sim')
        records.append(dict(name=directory.name, path=str(directory.relative_to(ROOT)),
            method=attempt['method'], scene=attempt['scene'], seed=attempt['seed'],
            runtime_versions=task['runtime_versions'], raw_status=attempt['status'],
            retrieval_success=attempt['retrieval_success'], D_exec=metrics['D_exec'],
            confirmed_by_window=[e['confirmed_candidate_count'] for e in metrics['environment_decisions']],
            terminal_reason=attempt.get('classification_reason'),
            counts=metrics['counts'], stages=metrics['stages'], paths=metrics['paths'],
            first_confirmation_active_sim_s=metrics['T_first_env_sim'],
            active_sim_s=metrics['T_active_sim'], task_sim_s=metrics['T_task_sim'],
            takeoff_to_landing_sim_s=None if takeoff is None or landed is None else landed-takeoff,
            selected_source_id=selected.get('source_id'),
            selected_pose=[selected[k] for k in ('x', 'y', 'yaw')] if selected else None,
            candidate_fallback_observed=any(s['rank'] > 1 for s in screens) if screens else None,
            screens=screens,
            camera_events=[e for e in events if e['state'] in ('GROUND_VIEW_REJECTED', 'GROUND_VIEW_PLAN')],
            physical=physical, same_state_scores=rankings,
            human_control_intervention=False))
        profiles.append({k: v for k, v in task.items() if k not in ('slots', 'runtime_versions')})
    assert len(records) == 4
    assert all(profile == profiles[0] for profile in profiles)
    for first in (0, 2):
        assert records[first]['runtime_versions'] == records[first+1]['runtime_versions']
        assert [r['method'] for r in records[first:first+2]] == ['generic', 'ours']
    result = dict(development_only=True, online_starts=4, online_start_cap=4,
        original_results_preserved=True, formal_statistics=False,
        same_version_pairs=[[records[i]['name'], records[i+1]['name']] for i in (0, 2)],
        comparative_effect_not_established='Each pair contains a pre-NBV runtime failure; do not infer a weighting effect.',
        records=records)
    (HERE / 'task-results.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(starts=4, checked_retrievals=sum(r['retrieval_success'] for r in records),
                         paired_profile_check='matched', formal=False)))


if __name__ == '__main__':
    main()
