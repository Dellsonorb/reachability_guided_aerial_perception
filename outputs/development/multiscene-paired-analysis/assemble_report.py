#!/usr/bin/env python3
"""This batch's read-only descriptive projection; never rerun/reclassify trials."""
import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BATCH = HERE.parent / 'multiscene-paired'


def read(path):
    return json.loads(path.read_text())


def main():
    config = read(ROOT / 'configs/dev_multiscene_paired.json')
    mechanism = {a['slot']: a for a in read(HERE / 'mechanism.json')['attempts']}
    records = []
    for directory in sorted(BATCH.glob('launch-*')):
        attempt = read(directory / 'attempt.json')
        metrics = read(directory / 'data/metrics.json')
        physical = read(directory / 'physical_summary.json')
        events = [json.loads(line) for line in (directory / 'data/events.jsonl').read_text().splitlines()]
        slot = config['slots'][attempt['slot'] - 1]
        scene = next(s for s in config['scenes'] if s['id'] == slot['scene'])
        assert all(attempt[k] == slot[k] for k in ('slot', 'scene', 'method'))
        assert attempt['scene_spec'] == scene and attempt.get('finish_wall') is not None
        assert attempt['full_robot_manipulation'] is True
        assert attempt['execution_clearance'] == 'chassis-clearance-v1'
        assert attempt['joint_velocity_feedback'] == 'integrated_pose_interval_velocity'
        assert not attempt.get('ground_replay_from') and not attempt.get('conditioned_on_arrival')
        selected = next((e for e in events if e['state'] == 'A5_SELECTED'), {})
        lifted = next((e for e in events if e['state'] == 'LIFT'), {})
        screens = [e for e in events if e['state'] == 'GROUND_EXECUTION_SCREEN']
        plans = [e for e in events if e['state'] == 'GROUND_MANIPULATION_PLAN']
        views = [e for e in events if e['state'] in ('GROUND_VIEW_REJECTED', 'GROUND_VIEW_PLAN')]
        windows = metrics['environment_decisions']
        final_gate = mechanism[attempt['slot']]['rounds'][-1]
        takeoff = metrics['stages']['takeoff']['start_sim']
        landed = metrics.get('landed_sim')
        first = next((e['round'] for e in windows if e['confirmed_candidate_count'] > 0), None)
        records.append(dict(
            slot=attempt['slot'], scene=attempt['scene'], seed=attempt['seed'], method=attempt['method'],
            attempt_dir=str(directory.relative_to(ROOT)), status=attempt['status'],
            retrieval_success=attempt.get('retrieval_success'), physical_status=physical.get('status'),
            adapter_completed_lift=bool(lifted), terminal_reason=attempt.get('classification_reason'),
            method_terminal_state=metrics['terminal_status'],
            method_failure_stage=metrics.get('terminal_failure_stage'),
            first_method_failure=next((e for e in events if e['state'] == 'FAILED'), None),
            confirmed_by_window=[e['confirmed_candidate_count'] for e in windows],
            first_discovery_window=first, first_discovery_active_sim_s=metrics.get('T_first_env_sim'),
            D_env=metrics['D_env'], D_exec=metrics['D_exec'],
            selected_source_id=selected.get('source_id'),
            selected_pose_xyyaw=[selected.get(k) for k in ('x', 'y', 'yaw')] if selected else None,
            ground_stopped_event=next((e for e in events if e['state'] == 'GROUND_STOPPED'), None),
            refined_event=next((e for e in events if e['state'] == 'GROUND_REFINED'), None),
            counts=metrics['counts'], stages=metrics['stages'], paths=metrics['paths'],
            active_sim_s=metrics['T_active_sim'], task_terminal_sim_s=metrics['T_task_sim'],
            takeoff_stage_to_landing_sim_s=None if landed is None or takeoff is None else landed-takeoff,
            physical_checker_target_lift_m=physical.get('physical_target', {}).get('target_lift_m'),
            reported_tcp_lift_m=lifted.get('tcp_lift'),
            candidate_screens=screens, arrival_manipulation_plans=plans, camera_events=views,
            candidate_fallback_observed=any(e['rank'] > 1 for e in screens) if screens else None,
            raw_grid_blocked=final_gate['raw_grid_blocked_exact']['count'],
            operational_blocked=final_gate['objectaware_blocked_exact']['count'],
            target_alias_retained=final_gate['target_alias_retained_exact']['count'],
            association=final_gate['occupied_classes'],
            derived_target_allowance_m=final_gate['target']['geometry_allowance_m'],
            human_robot_control_intervention=False,
            intervention_source='Operator ledger: only ordinary runner launch/read-only monitoring; no pose/control override.',
            runtime_flags=dict(full_robot_manipulation=attempt['full_robot_manipulation'],
                               execution_clearance=attempt['execution_clearance'],
                               joint_velocity_feedback=attempt['joint_velocity_feedback'])))
    assert [r['slot'] for r in records] == list(range(1, 13))
    assert all(r['status'] == 'VALID_TRIAL' and type(r['retrieval_success']) is bool for r in records)
    pairs = []
    for scene in config['scenes']:
        members = {r['method']: r for r in records if r['scene'] == scene['id']}
        assert set(members) == {'generic', 'ours'}
        g, o = members['generic'], members['ours']
        assert (ROOT / g['attempt_dir'] / 'runtime.launch').read_bytes() == (
            ROOT / o['attempt_dir'] / 'runtime.launch').read_bytes()
        pairs.append(dict(scene=scene['id'], seed=scene['seed'],
                          generic_slot=g['slot'], ours_slot=o['slot'],
                          generic_retrieval=g['retrieval_success'], ours_retrieval=o['retrieval_success'],
                          generic_D_exec=g['D_exec'], ours_D_exec=o['D_exec'],
                          generic_adapter_lift=g['adapter_completed_lift'], ours_adapter_lift=o['adapter_completed_lift']))
    result = dict(development_only=True, formal_statistics=False,
                  actual_run_agent_commit='c8b5b1bb47a332f9020487fe4dcf866c2403df34',
                  baseline=config['baseline_versions'], runtime_code_changed_during_batch=False,
                  planned_tasks=12, completed_tasks=len(records), reserve_starts_used=0, reserve_starts_unused=4,
                  invalid_trials=0, original_results_preserved=True, records=records, pairs=pairs,
                  limitations=[
                      'Missing path/physical metrics stay null, not zero; observed lower bounds are not full paths.',
                      'Slot3 original checker FAIL remains false although adapter completed lift; native motion diagnosis is separate.',
                      'The six unfiltered development seeds happened to all have negative target yaw; no positive-yaw/general coverage claim.',
                      'Recorded successful stages are not continuous clearance guarantees;12mm remains a development allowance.',
                      'No hypothesis test, formal effect size or sample-size selection is performed.'])
    (HERE / 'results.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    rows = []
    for r in records:
        row = {k: r[k] for k in ('slot', 'scene', 'seed', 'method', 'status', 'retrieval_success',
                                 'adapter_completed_lift', 'D_env', 'D_exec', 'selected_source_id',
                                 'first_discovery_window', 'active_sim_s', 'task_terminal_sim_s',
                                 'takeoff_stage_to_landing_sim_s', 'terminal_reason')}
        row['confirmed_by_window'] = '/'.join(map(str, r['confirmed_by_window']))
        row.update({k: r['counts'][k] for k in ('completed_windows', 'nbv_moves', 'sensing_rescans')})
        for name in ('uav_active', 'uav_total', 'uav_cleanup', 'ground_total'):
            for key in ('distance_m', 'complete', 'missing_samples', 'observed_distance_lower_bound_m'):
                row[name + '_' + key] = r['paths'][name][key]
        for name in ('ground_navigation', 'ground_refine', 'refined_pregrasp', 'descend', 'close', 'lift', 'retention'):
            for key in ('status', 'duration_sim_s'):
                row[name + '_' + key] = r['stages'][name][key]
        rows.append(row)
    with (HERE / 'tasks.csv').open('w', newline='') as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(dict(completed=len(records), raw_retrieval_passes=sum(r['retrieval_success'] for r in records),
                          adapter_lifts=sum(r['adapter_completed_lift'] for r in records), same_version_pairs=len(pairs))))


if __name__ == '__main__':
    main()
