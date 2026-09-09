#!/usr/bin/env python3
"""Describe saved v1.1 gate evidence without replay, GT, queries or policy changes.

Class cell counts overlap. Exact TARGET alias retention is not confirmation,
execution readiness or retrieval. Missing evidence is null, never a zero count.
"""

import argparse
import json
from pathlib import Path
from zipfile import BadZipFile

import numpy as np


def _group(rows, key):
    if rows is None:
        return dict(ids=None, count=None, unavailable_ids=None)
    ids = [row['candidate_id'] for row in rows if row.get(key) is True]
    missing = [row['candidate_id'] for row in rows if row.get(key) is None]
    return dict(ids=ids, count=None if missing else len(ids), unavailable_ids=missing)


def _positive(value):
    return None if value is None else bool(value > 0)


def describe_round(decision, operational_summary, arrays):
    """Project saved exact assessments and vote arrays; do not infer missing gates."""
    unavailable = [name for name, value in (
        ('decision.json', decision), ('operational_summary.json', operational_summary),
        ('operational_evidence.npz', arrays)) if value is None]
    decision, summary = decision or {}, operational_summary or {}
    applicable = decision.get('policy_method') != 'rm4d_only'
    association = summary.get('association_status', 'UNAVAILABLE')
    if association != 'AVAILABLE':
        unavailable.append('target_reference')
    classes = {}
    for label in ('TARGET', 'ENVIRONMENT', 'AMBIGUOUS'):
        key = label.lower() + '_occupied_votes'
        values = None if arrays is None or key not in arrays else np.asarray(arrays[key])
        classes[label] = dict(
            vote_sum=summary.get('votes', {}).get(key) if values is None else int(values.sum()),
            occupied_cell_count=None if values is None else int(np.count_nonzero(values > 0)))
        if values is None:
            unavailable.append(key)
    ground = None if arrays is None or 'ground_votes' not in arrays else arrays['ground_votes']
    if ground is None and summary.get('votes', {}).get('ground_votes') is None:
        unavailable.append('ground_votes')
    rows = None if decision.get('assessments') is None else []
    if rows is None:
        unavailable.append('assessments')
    for candidate in decision.get('assessments') or []:
        gate = candidate.get('operational') or {}
        row = {key: candidate.get(key) for key in (
            'candidate_id', 'source_id', 'occupied_cells', 'representative_blocked',
            'footprint_clipped', 'confirmed')}
        row.update({key: gate.get(key) for key in (
            'target_cells', 'environment_cells', 'ambiguous_cells', 'target_collision',
            'ground_supported', 'ground_supported_cells', 'ground_missing_cells')})
        row['pose_xyyaw'] = ([candidate[key] for key in ('x', 'y', 'yaw')]
                             if all(key in candidate for key in ('x', 'y', 'yaw')) else None)
        row['operational_blocked'] = gate.get('blocked')
        row['raw_grid_blocked'] = _positive(row['occupied_cells'])
        row['environment_blocked'] = _positive(row['environment_cells'])
        row['ambiguous_blocked'] = _positive(row['ambiguous_cells'])
        exact, representative = row['operational_blocked'], row['representative_blocked']
        row['combined_blocked'] = (True if exact is True or representative is True else
                                   False if exact is False and representative is False else None)
        row['representative_only_blocked'] = (
            bool(representative and not exact) if exact is not None and representative is not None else None)
        alias_fields = (row['occupied_cells'], row['target_cells'], exact, row['target_collision'])
        row['target_alias_retained_exact'] = (
            bool(row['occupied_cells'] > 0 and row['target_cells'] > 0
                 and not exact and not row['target_collision'])
            if association == 'AVAILABLE' and all(value is not None for value in alias_fields) else None)
        unavailable.extend('candidate:%s:%s' % (row['candidate_id'], key)
                           for key, value in row.items() if value is None)
        rows.append(row)
    result = dict(
        status='UNAVAILABLE' if not decision else 'PARTIAL' if unavailable else 'AVAILABLE',
        mechanism_applicable=applicable, round=decision.get('round'),
        operational_semantics=summary.get('operational_semantics', decision.get('operational_semantics')),
        association_status=association, reference_file=summary.get('reference_file'),
        target=summary.get('target'), observation_windows=summary.get('observation_windows'),
        occupied_classes=classes, occupied_class_cell_counts_exclusive=False,
        ground_votes_sum=summary.get('votes', {}).get('ground_votes') if ground is None else int(np.sum(ground)),
        candidate_count_recorded=decision.get('candidate_count'),
        confirmed_candidate_count_recorded=decision.get('confirmed_candidate_count'),
        candidates=rows, unavailable=unavailable,
        representative_raw_grid_blocked=None,
        representative_blocking_causes=None,
        representative_evidence_note='Saved assessments contain representative_blocked only; '
                                     'raw representative occupancy and blocker causes are not recorded.')
    for name, key in (
        ('raw_grid_blocked_exact', 'raw_grid_blocked'), ('objectaware_blocked_exact', 'operational_blocked'),
        ('representative_blocked', 'representative_blocked'),
        ('representative_only_blocked', 'representative_only_blocked'),
        ('combined_blocked', 'combined_blocked'),
        ('target_alias_retained_exact', 'target_alias_retained_exact'), ('confirmed', 'confirmed')):
        result[name] = _group(rows, key)
    result['blocked_causes'] = {name: _group(rows, key) for name, key in (
        ('environment', 'environment_blocked'), ('ambiguous', 'ambiguous_blocked'),
        ('target_collision', 'target_collision'))}
    if not applicable:
        result.update(status='NOT_APPLICABLE', candidates=None, unavailable=[],
                      occupied_classes={key: dict(vote_sum=None, occupied_cell_count=None) for key in classes},
                      ground_votes_sum=None, confirmed_candidate_count_recorded=None)
        for key in ('raw_grid_blocked_exact', 'objectaware_blocked_exact', 'representative_blocked',
                    'representative_only_blocked', 'combined_blocked', 'target_alias_retained_exact', 'confirmed'):
            result[key] = _group(None, '')
        result['blocked_causes'] = {key: _group(None, '') for key in result['blocked_causes']}
    return result


def _read_json(path, unavailable):
    try:
        value = json.loads(path.read_text())
        if not isinstance(value, dict):
            raise ValueError('expected JSON object')
        return value
    except (OSError, ValueError) as error:
        unavailable.append(path.name if not path.exists() else path.name + ': ' + str(error))
        return None


def _describe_attempt(directory):
    unavailable = []
    attempt = _read_json(directory / 'attempt.json', unavailable) or {}
    metrics = _read_json(directory / 'data/metrics.json', unavailable) or {}
    method = attempt.get('method', metrics.get('method'))
    if attempt.get('kind') == 'METHOD_INDEPENDENT_SETUP' or method == 'SETUP_CHECK':
        return None
    applicable = method != 'rm4d_only'
    rounds = []
    events = metrics.get('environment_decisions')
    if applicable:
        paths = set((directory / 'data/rounds').glob('round-*'))
        for event in events or []:
            if event.get('round') is not None:
                paths.add(directory / 'data/rounds' / ('round-%02d' % event['round']))
        for path in sorted(paths):
            missing = []
            decision = _read_json(path / 'decision.json', missing)
            summary = _read_json(path / 'operational_summary.json', missing)
            arrays = None
            try:
                with (path / 'operational_evidence.npz').open('rb') as source, np.load(source, allow_pickle=False) as data:
                    arrays = {key: data[key] for key in data.files}
            except (OSError, ValueError, BadZipFile) as error:
                missing.append('operational_evidence.npz: ' + str(error))
            report = describe_round(decision, summary, arrays)
            report.update(snapshot=str(path), read_errors=missing)
            if report['round'] is None and path.name[6:].isdigit():
                report['round'] = int(path.name[6:])
            rounds.append(report)
        if not rounds:
            unavailable.append('data/rounds/round-*')
    first = next((event.get('round') for event in events or []
                  if (event.get('confirmed_candidate_count') or 0) > 0), None) if applicable else None
    outcomes = {key: metrics.get(key) for key in (
        'D_env', 'D_exec', 'C_env_count_final', 'C_env_count_max', 'first_env_sim',
        'exec_ready_sim', 'T_first_env_sim', 'T_exec_ready_sim', 'physical_success',
        'terminal_status', 'terminal_failure_stage', 'terminal_failure_reason')}
    outcomes['retrieval_success'] = attempt.get('retrieval_success')
    if not applicable:
        for key in ('D_env', 'C_env_count_final', 'C_env_count_max', 'first_env_sim', 'T_first_env_sim'):
            outcomes[key] = None
    return dict(
        attempt=str(directory), method=method, slot=attempt.get('slot'), seed=attempt.get('seed'),
        attempt_status=attempt.get('status'), classification_reason=attempt.get('classification_reason'),
        mechanism_status=('NOT_APPLICABLE' if not applicable else 'UNAVAILABLE' if not rounds else
                          'AVAILABLE' if all(r['status'] == 'AVAILABLE' for r in rounds) else 'PARTIAL'),
        rounds=rounds, confirmed_final=rounds[-1]['confirmed'] if rounds else _group(None, ''),
        first_confirmed_window=first,
        first_confirmed_window_source='metrics.environment_decisions' if applicable and events is not None else None,
        outcomes=outcomes, unavailable=unavailable)


def describe_results(results_dir):
    """Join existing attempts, including attempts with absent expected round files."""
    root = Path(results_dir)
    directories = {path.parent for path in root.rglob('attempt.json')}
    directories.update(path.parents[1] for path in root.rglob('data/metrics.json'))
    directories.update(path.parents[1] for path in root.rglob('data/rounds'))
    attempts = []
    for path in sorted(directories):
        report = _describe_attempt(path)
        if report is not None:
            attempts.append(report)
    return dict(kind='saved_objectaware_v11_mechanism_diagnostics', schema_version=1,
                descriptive_only=True, attempt_count=len(attempts), attempts=attempts,
                note='Exact TARGET alias retention does not imply confirmation, D_exec or retrieval. '
                     'Class cell counts and blocker causes can overlap. Outcomes are recorded projections; '
                     'no gate replay, label inference, ground vote inference or outcome reclassification.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, help='Optional new report file; existing files are never overwritten.')
    args = parser.parse_args(argv)
    result = describe_results(args.results_dir)
    serialized = json.dumps(result, indent=2, allow_nan=False) + '\n'
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as output:
            output.write(serialized)
    else:
        print(serialized, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
