#!/usr/bin/python3
"""Summarize the predefined A6 pilot slots from retained offline JSON evidence.

This script never runs a trial or replaces an outcome. Directory order defines
rerun links. Only one completed VALID_TRIAL with a literal binary outcome can
supply a slot's primary result; duplicate valid trials are reported as ambiguous.
"""

import argparse
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PRIMARY_METHODS = ('ours', 'generic')
METRIC_FIELDS = (
    'C_env_count_final', 'C_env_count_max', 'D_env', 'D_exec',
    'first_env_sim', 'exec_ready_sim', 'task_start_sim', 'task_end_sim',
    'active_start_sim', 'active_end_sim', 'landed_sim',
    'T_first_env_sim', 'T_exec_ready_sim', 'T_task_sim', 'T_active_sim',
    'counts', 'paths', 'stages', 'clock_reset_detected',
    'cleanup_clock_reset_detected',
)


def read_object(path, label, missing, errors):
    """Missing ancillary evidence is recorded, never turned into a zero."""
    if not path.exists():
        missing.append(label)
        return None
    try:
        result = json.loads(path.read_text())
        if not isinstance(result, dict):
            raise ValueError('expected a JSON object')
        return result
    except (OSError, ValueError) as error:
        errors.append(label + ': ' + str(error))
        return None


def evidence(method, activation=None):
    """Project recorded metrics without inferring physical success from LIFT."""
    activation = activation or {}
    record = activation.get('raw_attempt') or {}
    metrics = activation.get('metrics') or {}
    physical = activation.get('physical_summary') or {}
    result = {key: metrics.get(key) for key in METRIC_FIELDS}
    result['D_env_applicable'] = method != 'rm4d_only'
    if method == 'rm4d_only':
        result['D_env'] = None
    result['retrieval_success'] = (
        record.get('retrieval_success')
        if activation.get('effective_status') == 'VALID_TRIAL'
        and type(record.get('retrieval_success')) is bool else None)
    result['physical_status'] = physical.get('status', record.get('physical_status'))
    result['metrics_terminal_status'] = metrics.get('terminal_status')
    result['failure_stage'] = metrics.get('terminal_failure_stage')
    result['failure_reason'] = next((reason for reason in (
        metrics.get('terminal_failure_reason'), record.get('classification_reason'),
        record.get('reason'), physical.get('error')) if reason is not None), None)
    return result


def summarize_slot(spec, attempts):
    valid = [a for a in attempts if a['effective_status'] == 'VALID_TRIAL']
    selected = None
    if len(valid) > 1:
        status = 'AMBIGUOUS_DUPLICATE_VALID'
    elif valid:
        if type(valid[0]['raw_attempt'].get('retrieval_success')) is bool:
            status, selected = 'VALID_TRIAL', valid[0]
        else:
            status = 'VALID_OUTCOME_MISSING'
    elif not attempts:
        status = 'NOT_RUN'
    elif any(a['effective_status'] == 'RUNNING' for a in attempts):
        status = 'RUNNING'
    elif all(a['effective_status'] == 'INVALID_TRIAL' for a in attempts):
        status = 'INVALID_TRIAL'
    else:
        status = 'UNCLASSIFIED'
    role = ('primary' if spec['method'] in PRIMARY_METHODS else
            'secondary_ablation' if spec['method'] in ('no_cost', 'no_occlusion') else
            'secondary_baseline')
    return dict(spec, status=status, comparison_role=role,
                selected_attempt=None if selected is None else selected['attempt_dir'],
                duplicate_valid_attempts=[a['attempt_dir'] for a in valid] if len(valid) > 1 else [],
                attempts=attempts, **evidence(spec['method'], selected))


def pair_outcome(row, first_activation=False):
    """Return a binary result plus its source/status for a single paired slot."""
    if row is None:
        return dict(slot=None, status='NOT_SCHEDULED', retrieval_success=None, attempt_dir=None)
    result = dict(slot=row['slot'], status=row['status'],
                  retrieval_success=row['retrieval_success'], attempt_dir=row['selected_attempt'])
    if first_activation:
        result.update(retrieval_success=None, attempt_dir=None)
        if row['status'] in ('AMBIGUOUS_DUPLICATE_VALID', 'NOT_RUN', 'RUNNING'):
            return result
        first = row['attempts'][0]
        result.update(status=first['effective_status'], attempt_dir=first['attempt_dir'])
        if first['effective_status'] == 'INVALID_TRIAL':
            result['retrieval_success'] = False
        elif first['effective_status'] == 'VALID_TRIAL':
            success = first['raw_attempt'].get('retrieval_success')
            if type(success) is bool:
                result['retrieval_success'] = success
    return result


def paired_comparison(scenes, rows, first_activation=False):
    """Descriptive paired binary counts; no inference or unpaired denominator."""
    by_seed_method = {(row['seed'], row['method']): row for row in rows}
    result = dict(methods=list(PRIMARY_METHODS), n_complete_pairs=0,
                  both_success=0, neither_success=0, ours_only_b=0, generic_only_c=0,
                  paired_risk_difference=None, pairs=[], incomplete_pairs=[])
    for scene in scenes:
        pair = dict(seed=scene['seed'], scene=scene['id'])
        for method in PRIMARY_METHODS:
            pair[method] = pair_outcome(by_seed_method.get((scene['seed'], method)), first_activation)
        ours, generic = (pair[method]['retrieval_success'] for method in PRIMARY_METHODS)
        if type(ours) is not bool or type(generic) is not bool:
            result['incomplete_pairs'].append(pair)
            continue
        category = ('both_success' if ours and generic else
                    'ours_only_b' if ours else 'generic_only_c' if generic else 'neither_success')
        pair['category'] = category
        result[category] += 1
        result['pairs'].append(pair)
    result['n_complete_pairs'] = len(result['pairs'])
    if result['n_complete_pairs']:
        result['paired_risk_difference'] = (
            (result['ours_only_b'] - result['generic_only_c']) / result['n_complete_pairs'])
    return result


def summarize_pilot(config, results_dir):
    """Collect only matching predefined pilot attempts, in stable path order."""
    results_dir = Path(results_dir)
    scenes = {scene['id']: scene for scene in config['scenes']}
    specs = [dict(slot, seed=scenes[slot['scene']]['seed']) for slot in config['slots']]
    by_slot = {spec['slot']: spec for spec in specs}
    if len(by_slot) != len(specs):
        raise ValueError('configuration contains duplicate slot numbers')
    if len({(s['seed'], s['method']) for s in specs}) != len(specs):
        raise ValueError('configuration contains duplicate seed/method slots')
    attempts, excluded = {spec['slot']: [] for spec in specs}, []
    paths = sorted(results_dir.rglob('attempt.json'), key=lambda path: path.as_posix())
    for path in paths:
        directory = path.parent
        relative = directory.relative_to(results_dir).as_posix()
        missing, errors = [], []
        record = read_object(path, 'attempt.json', missing, errors)
        reason = None
        if record is None:
            reason = 'unreadable_attempt_record'
        elif record.get('kind') != 'PILOT_ATTEMPT':
            reason = 'not_a_pilot_attempt'
        elif record.get('slot') not in by_slot:
            reason = 'slot_not_scheduled'
        else:
            spec = by_slot[record['slot']]
            if any(record.get(key) != spec[key] for key in ('scene', 'seed', 'method')):
                reason = 'scheduled_slot_metadata_mismatch'
        if reason is not None:
            excluded.append(dict(attempt_dir=relative, reason=reason, raw_attempt=record,
                                 missing_files=missing, read_errors=errors))
            continue
        physical = read_object(directory / 'physical_summary.json', 'physical_summary.json', missing, errors)
        metrics = read_object(directory / 'data/metrics.json', 'data/metrics.json', missing, errors)
        completed = record.get('finish_wall') is not None
        status = record.get('status', 'UNCLASSIFIED') if completed else 'RUNNING'
        previous = attempts[record['slot']]
        previous_dir = previous[-1]['attempt_dir'] if previous else None
        previous.append(dict(attempt_dir=relative, rerun_of=previous_dir,
                             completed=completed, effective_status=status,
                             raw_attempt=record, physical_summary=physical, metrics=metrics,
                             missing_files=missing, read_errors=errors))
    rows = [summarize_slot(spec, attempts[spec['slot']]) for spec in specs]
    all_attempts = [attempt for row in rows for attempt in row['attempts']]
    counts = dict(
        scheduled_slots=len(rows), total_activations=len(all_attempts),
        completed_activations=sum(a['completed'] for a in all_attempts),
        running_activations=sum(a['effective_status'] == 'RUNNING' for a in all_attempts),
        invalid_activations=sum(a['effective_status'] == 'INVALID_TRIAL' for a in all_attempts),
        valid_completed_activations=sum(a['effective_status'] == 'VALID_TRIAL' for a in all_attempts),
        valid_completed_slots=sum(row['status'] == 'VALID_TRIAL' for row in rows),
        not_run_slots=sum(row['status'] == 'NOT_RUN' for row in rows),
        ambiguous_slots=sum(row['status'] == 'AMBIGUOUS_DUPLICATE_VALID' for row in rows),
        valid_success_slots=sum(row['retrieval_success'] is True for row in rows),
        valid_failure_slots=sum(row['retrieval_success'] is False for row in rows),
        excluded_records=len(excluded),
    )
    primary = paired_comparison(config['scenes'], rows)
    sensitivity = paired_comparison(config['scenes'], rows, first_activation=True)
    sensitivity['definition'] = (
        'First directory-ordered activation per slot: completed INVALID_TRIAL becomes failure; '
        'completed VALID_TRIAL keeps its binary outcome. Unrun, running, ambiguous and '
        'unknown outcomes are excluded. This is separate from the completed-valid primary analysis.')
    return dict(schema_version=1, descriptive_only=True, efficiency_clock='simulation',
                config_status=config.get('status'), counts=counts, slots=rows,
                primary_comparison=primary, first_activation_invalid_as_failure=sensitivity,
                excluded_attempts=excluded)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/a6_pilot.json')
    parser.add_argument('--results-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    report = summarize_pilot(json.loads(args.config.read_text()), args.results_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
