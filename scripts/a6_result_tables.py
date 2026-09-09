#!/usr/bin/env python3
"""Project an existing A6 collector summary to descriptive JSON/CSV tables.

No trial execution, bag reads, outcome reclassification, or statistical tests.
JSON null and CSV empty cells mean unavailable; recorded zeros remain zeros.
"""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path

from a6_metrics import STAGES


SCALARS = ('D_env', 'D_exec', 'C_env_count_final', 'C_env_count_max',
           'T_first_env_sim', 'T_active_sim', 'T_task_sim', 'T_exec_ready_sim',
           'metrics_terminal_status', 'failure_stage', 'failure_reason')
COUNTS = ('capture_calls', 'discarded_windows', 'completed_windows', 'voted_windows',
          'sensing_visits', 'sensing_rescans', 'nbv_moves', 'initial_outbound', 'return_actions')
PATHS = ('uav_active', 'uav_total', 'ground_total')
PATH_FIELDS = ('distance_m', 'complete', 'missing_samples', 'gap_count',
               'observed_distance_lower_bound_m')
STAGE_FIELDS = ('status', 'duration_sim_s')
AVAILABLE_FIELDS = ('C_env_count_final', 'C_env_count_max', 'first_discovery_window',
                    'T_first_env_sim', 'T_active_sim', 'T_task_sim', 'T_exec_ready_sim',
                    *COUNTS, *(path + '_' + field for path in PATHS for field in PATH_FIELDS),
                    *('stage_' + stage + '_' + field for stage in STAGES for field in STAGE_FIELDS))


def describe_results(config, summary):
    """Flatten literal collector fields; join only its selected raw metrics."""
    scenes = {scene['id']: scene for scene in config['scenes']}
    specs = config['slots']
    supplied = summary.get('slots', [])
    by_slot = {row['slot']: row for row in supplied}
    scheduled = {slot['slot'] for slot in specs}
    if len(by_slot) != len(supplied) or len(scheduled) != len(specs):
        raise ValueError('duplicate slot IDs cannot be projected')
    if set(by_slot) - scheduled:
        raise ValueError('summary contains slots not listed in the supplied configuration')
    rows = []
    for spec in specs:
        scene, source = scenes[spec['scene']], by_slot.get(spec['slot'], {})
        identity = dict(slot=spec['slot'], scene=spec['scene'], seed=scene['seed'], method=spec['method'])
        if source and any(source.get(key) != value for key, value in identity.items()):
            raise ValueError('summary/config metadata mismatch for slot %s' % spec['slot'])
        attempts = source.get('attempts')
        selected_name = source.get('selected_attempt')
        selected = next((a for a in attempts or [] if a.get('attempt_dir') == selected_name), {}) \
            if selected_name is not None else {}
        decisions = (selected.get('metrics') or {}).get('environment_decisions')
        first_window = None
        if spec['method'] != 'rm4d_only':
            for decision in decisions or []:
                count = decision.get('confirmed_candidate_count')
                if type(count) in (int, float) and count > 0:
                    first_window = decision.get('round')
                    break
        row = dict(identity, tier=scene.get('tier', scene['id']), status=source.get('status'),
                   retrieval_success=source.get('retrieval_success'), selected_attempt=selected_name,
                   invalid_activation_count=None if attempts is None or any(
                       a.get('effective_status') is None for a in attempts) else sum(
                       a.get('effective_status') == 'INVALID_TRIAL' for a in attempts),
                   D_env_applicable=spec['method'] != 'rm4d_only',
                   first_discovery_window=first_window)
        row.update({key: source.get(key) for key in SCALARS})
        if not row['D_env_applicable']:
            row['D_env'] = None
        counts, paths, stages = (source.get(key) or {} for key in ('counts', 'paths', 'stages'))
        row.update({key: counts.get(key) for key in COUNTS})
        for name in PATHS:
            row.update({name + '_' + key: (paths.get(name) or {}).get(key) for key in PATH_FIELDS})
        for name in STAGES:
            row.update({'stage_' + name + '_' + key: (stages.get(name) or {}).get(key)
                        for key in STAGE_FIELDS})
        rows.append(row)
    grouped = {}
    for row in rows:
        grouped.setdefault((row['tier'], row['method']), []).append(row)
    groups = []
    for (tier, method), members in grouped.items():
        endpoints = {}
        for endpoint in ('retrieval_success', 'D_env', 'D_exec'):
            endpoints[endpoint] = dict(
                numerator=sum(row[endpoint] is True for row in members),
                denominator=sum(type(row[endpoint]) is bool for row in members),
                not_applicable=sum(not row['D_env_applicable'] for row in members)
                if endpoint == 'D_env' else 0)
        groups.append(dict(tier=tier, method=method, scheduled_slots=len(members),
                           status_counts=dict(Counter(row['status'] for row in members
                                                      if row['status'] is not None)),
                           missing_status_slots=sum(row['status'] is None for row in members),
                           endpoints=endpoints, available_counts={
                               key: sum(row[key] is not None for row in members) for key in AVAILABLE_FIELDS}))
    return dict(schema_version=1, descriptive_only=True, resource_source='original_online_metrics',
                missing_semantics='JSON null / CSV empty means unavailable, not zero or failure; '
                                  'recorded NOT_REACHED stages remain distinct from FAILED.',
                discovery_source='selected attempt metrics.environment_decisions, first positive count in saved order',
                group_semantics='All scheduled slots by tier/method, without success filtering. '
                                'Boolean denominators count literal available booleans; '
                                'available_counts use the scheduled_slots denominator.',
                rows=rows, groups=groups)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--summary', type=Path, required=True, help='Existing collector JSON, not a results directory')
    parser.add_argument('--output-json', type=Path, required=True, help='New descriptive report path')
    parser.add_argument('--output-csv', type=Path, required=True, help='New flat slot table path')
    args = parser.parse_args(argv)
    report = describe_results(json.loads(args.config.read_text()), json.loads(args.summary.read_text()))
    if args.output_json.resolve() == args.output_csv.resolve():
        raise ValueError('JSON and CSV outputs must be distinct')
    for path in (args.output_json, args.output_csv):
        if path.exists():
            raise FileExistsError('output must be a new file: ' + str(path))
        path.parent.mkdir(parents=True, exist_ok=True)
    with args.output_json.open('x') as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False) + '\n')
    with args.output_csv.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(report['rows'][0]) if report['rows'] else ['slot'])
        writer.writeheader()
        writer.writerows(report['rows'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
