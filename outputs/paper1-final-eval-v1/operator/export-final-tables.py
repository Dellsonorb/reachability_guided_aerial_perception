#!/usr/bin/env python3
"""One-study, offline table export. Never launches robots or changes source records.

Run with the existing RM4D numerical Python and PYTHONPATH=src:scripts, from
the AGENT root. The frozen analyzer remains the source of primary inference.
"""
from collections import Counter
import csv
import json
from pathlib import Path
import statistics

import numpy as np

from a6_operational_diagnostics import describe_round
from a6_scoring_diagnostics import check_snapshot


ROOT = Path(__file__).resolve().parents[1]
ANALYSIS = json.loads((ROOT / 'analysis-latest.json').read_text())
assert ANALYSIS['status'] == 'COMPLETE' and not ANALYSIS['protocol_issues']
assert ANALYSIS['counts']['valid_completed_slots'] == 212
ROWS = ANALYSIS['slots']
STAGES = ('aerial_observe', 'execution_screen', 'ground_navigation', 'ground_refine',
          'refined_pregrasp', 'descend', 'close', 'lift', 'retention')
BY_SLOT = {r['slot']: r for r in ROWS}
TIERS = {p['scene']: t for t, group in ANALYSIS['tiers'].items() for p in group['pairs']}


def write_json(name, value):
    (ROOT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def table(name, rows):
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with (ROOT / name).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, lineterminator='\n')
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v, separators=(',', ':')) if isinstance(v, (dict, list))
                             else v for k, v in row.items()})


def stage(row, name):
    return (row.get('stages') or {}).get(name, {}).get('status', 'NOT_REACHED')


def values_summary(values):
    good = [v for v in values if type(v) in (int, float) and np.isfinite(v)]
    return dict(n=len(good), missing=len(values)-len(good),
                mean=statistics.mean(good) if good else None,
                median=statistics.median(good) if good else None)


slots, activations, retention, mechanisms, scoring = [], [], [], [], []
for row in ROWS:
    directory = ROOT / row['selected_attempt']
    events = [json.loads(line) for line in (directory / 'data/events.jsonl').read_text().splitlines()]
    terminal = next((e for e in events if e['state'] in ('FAILED', 'LIFT')), None)
    task_events = events[:events.index(terminal)+1] if terminal is not None else events
    selected = next((e for e in reversed(task_events) if e['state'] == 'A5_SELECTED'), None)
    screens = [e for e in task_events if e['state'] == 'GROUND_EXECUTION_SCREEN']
    selected_activation = next(a for a in row['attempts'] if a['attempt_dir'] == row['selected_attempt'])
    metrics = selected_activation.get('metrics') or {}
    env_events = metrics.get('environment_decisions') or []
    locations = row['observation_locations'] or {}
    flat = {k: row.get(k) for k in ('slot', 'scene', 'seed', 'method', 'comparison_role', 'status',
            'retrieval_success', 'physical_status', 'selected_attempt', 'failure_stage', 'failure_reason',
            'D_env', 'D_exec', 'C_env_count_max', 'C_env_count_final', 'T_first_env_sim',
            'T_exec_ready_sim', 'windows', 'nonzero_offset_commands', 'T_active_sim',
            'T_ground_sim', 'T_task_sim')}
    flat.update(tier=TIERS[row['scene']], first_terminal_event=terminal,
                first_confirmation_window=next((e.get('round') for e in env_events
                                               if (e.get('confirmed_candidate_count') or 0) > 0), None),
                screened_candidate=any(e.get('feasible') is True for e in screens),
                execution_screen_attempts=len(screens), selected_candidate_event=selected,
                human_task_intervention=False,
                actual_location_changes=locations.get('resolved_location_changes'),
                actual_translation_changes=locations.get('resolved_translation_changes'),
                actual_yaw_only_changes=locations.get('resolved_yaw_only_changes'),
                observation_locations=locations, distance_records=row.get('paths'),
                counts=row.get('counts'), analysis_missing=row.get('analysis_missing'))
    flat.update({name+'_status': stage(row, name) for name in STAGES})
    flat.update({name+'_time_sim_s': (row.get('stages') or {}).get(name, {}).get('duration_sim_s')
                 for name in STAGES})
    slots.append(flat)
    for attempt in row['attempts']:
        raw = attempt['raw_attempt']
        entry = json.loads((ROOT / attempt['attempt_dir']).parent.joinpath('entry.json').read_text())
        activations.append(dict(slot=row['slot'], scene=row['scene'], method=row['method'],
                                attempt_dir=attempt['attempt_dir'], selected=attempt['attempt_dir']==row['selected_attempt'],
                                status=attempt['effective_status'], retrieval_success=raw.get('retrieval_success'),
                                reason=raw.get('reason'), classification_reason=raw.get('classification_reason'),
                                entry_start_wall=entry.get('start_wall'), activation_wall=raw.get('activation_wall'),
                                finish_wall=raw.get('finish_wall'), entry=entry,
                                runtime_versions=raw.get('runtime_versions'),
                                missing_files=attempt['missing_files'], read_errors=attempt['read_errors']))
        note_path = ROOT / attempt['attempt_dir'] / 'retention.json'
        retention.append(dict(attempt_dir=attempt['attempt_dir'], note=json.loads(note_path.read_text())))
    for rd in sorted((directory / 'data/rounds').glob('round-*')):
        if not (rd / 'ranking.json').exists():
            continue
        ranking = json.loads((rd / 'ranking.json').read_text())
        decision = json.loads((rd / 'decision.json').read_text())
        op_summary = json.loads((rd / 'operational_summary.json').read_text())
        with np.load(rd / 'operational_evidence.npz', allow_pickle=False) as arrays:
            op = describe_round(decision, op_summary, arrays)
        with np.load(rd / 'fields.npz', allow_pickle=False) as arrays:
            scores = check_snapshot(ranking, arrays)
        common = dict(slot=row['slot'], scene=row['scene'], method=row['method'], round=decision['round'],
                      snapshot=str(rd.relative_to(ROOT)))
        mechanisms.append(dict(common, status=op['status'], association=op['association_status'],
                               classes=op['occupied_classes'], ground_votes_sum=op['ground_votes_sum'],
                               ground_presence_votes_sum=op_summary.get('votes', {}).get('ground_presence_votes'),
                               raw_grid_blocked=op['raw_grid_blocked_exact']['count'],
                               operational_blocked=op['objectaware_blocked_exact']['count'],
                               representative_only_blocked=op['representative_only_blocked']['count'],
                               target_alias_retained=op['target_alias_retained_exact']['count'],
                               confirmed=op['confirmed']['count'],
                               exact_winner_anchors=ranking['a3_summary'].get('winner_anchors'),
                               anchor_semantics=ranking['a3_summary'].get('anchor_semantics'),
                               unavailable=op['unavailable']))
        scores.pop('candidates')
        scoring.append(dict(common, **scores))

table('slots.csv', slots)
table('activations.csv', activations)
table('mechanism-rounds.csv', mechanisms)
table('same-state-scoring.csv', scoring)
write_json('retention-index.json', retention)
flat_by_slot = {r['slot']: r for r in slots}
pairs = []
for pair in ANALYSIS['primary']['pairs']:
    out = dict(scene=pair['scene'], seed=pair['seed'], tier=TIERS[pair['scene']], category=pair['category'])
    for method in ('ours', 'generic'):
        r = flat_by_slot[pair[method]['slot']]
        out.update({method+'_'+k: r[k] for k in ('slot', 'retrieval_success', 'D_env', 'screened_candidate',
                   'D_exec', 'windows', 'actual_location_changes', 'T_active_sim', 'T_ground_sim',
                   'T_task_sim', 'failure_stage', 'failure_reason')})
    pairs.append(out)
table('pairs.csv', pairs)

groups = {}
joint = {p['scene'] for p in pairs if p['category'] == 'a'}
for method in ('ours', 'generic', 'fixed', 'rm4d_only', 'no_cost', 'no_occlusion'):
    group = [r for r in slots if r['method'] == method]
    original = [BY_SLOT[r['slot']] for r in group]
    groups[method] = dict(n=len(group), successes=sum(r['retrieval_success'] for r in group),
        windows=dict(Counter(r['windows'] for r in group)),
        stages={name:dict(Counter(stage(r, name) for r in original)) for name in STAGES},
        confirmed=sum(r['D_env'] is True for r in group), screened=sum(r['screened_candidate'] for r in group),
        D_exec=sum(r['D_exec'] is True for r in group),
        failure_stages=dict(Counter(r['failure_stage'] for r in group if not r['retrieval_success'])),
        failure_reasons=dict(Counter(r['failure_reason'] for r in group if not r['retrieval_success'])),
        missing_metrics={k:sum(r[k] is None for r in group) for k in
                         ('windows','actual_location_changes','T_active_sim','T_ground_sim','T_task_sim')},
        actual_location_changes=dict(Counter(r['actual_location_changes'] for r in group)),
        actual_yaw_only_changes=dict(Counter(r['actual_yaw_only_changes'] for r in group)),
        command_counts=dict(Counter(r['nonzero_offset_commands'] for r in group)),
        complete_distance_records={k:sum((r.get('paths') or {}).get(k,{}).get('complete') is True
                                        for r in original) for k in ('uav_active','uav_total','ground_total')},
        joint_success_metrics={k:values_summary([r[k] for r in group if r['scene'] in joint])
            for k in ('windows','actual_location_changes','T_active_sim','T_ground_sim','T_task_sim')}
            if method in ('ours','generic') else None)
write_json('descriptive-summary.json', dict(groups=groups,
    same_state=dict(snapshots=len(scoring), all_pass=all(r['gain_and_cost_identity_pass'] for r in scoring),
                    different_selections=sum(r['different_selection'] for r in scoring),
                    maximum_error=max(r['maximum_abs_identity_error'] for r in scoring)),
    mechanism=dict(snapshots=len(mechanisms), statuses=dict(Counter(r['status'] for r in mechanisms)),
                   representative_only_blocked_sum=sum(r['representative_only_blocked'] or 0 for r in mechanisms),
                   target_alias_retained_snapshot_count=sum((r['target_alias_retained'] or 0)>0 for r in mechanisms)),
    retention=dict(records=len(retention), removed_image_bags=sum(r['note'].get('images_removed') is True for r in retention),
                   removed_image_bytes=sum(r['note'].get('images_original_bytes') or 0 for r in retention
                                           if r['note'].get('images_removed') is True)),
    note='These are descriptive exports. Primary inference remains analysis-latest.json; no records reclassified.'))
assert len(slots)==212 and len(pairs)==96 and len(activations)==214
assert all(r['gain_and_cost_identity_pass'] for r in scoring)
print(json.dumps(dict(slots=len(slots), pairs=len(pairs), activations=len(activations),
                      snapshots=len(scoring), all_shared_scoring_checks_pass=True)))
