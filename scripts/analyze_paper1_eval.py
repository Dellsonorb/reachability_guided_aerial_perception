#!/usr/bin/env python3
"""Offline Paper 1 paired evaluation; reads retained attempts, never runs robots.

Use the existing numerical Python environment (NumPy/SciPy). Only a complete,
protocol-compliant manifest receives the single aggregate primary test.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np

from analyze_a6_formal import binomial_interval, exact_mcnemar
from summarize_a6_pilot import pair_outcome, summarize_pilot

COHORT = 'paper1-eval-finite-scan-v1-final'
METHODS = ('ours', 'generic', 'fixed', 'rm4d_only', 'no_cost', 'no_occlusion')
BOOTSTRAP_SEED = 2026091024
RUNTIME_KEYS = ('cohort', 'status', 'initial_view', 'uav_launch_pose', 'acquisition_model',
                'observation_windows', 'window_sim_s', 'capture_wall_guard_s', 'task_wall_guard_s',
                'shared_a5_settings', 'operational_gating', 'support_anchor', 'handoff_stop',
                'record_diagnostics', 'diagnostic_image_scope')


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_config(config):
    if config.get('status') != 'FROZEN_FOR_EVALUATION' or config.get('cohort') != COHORT:
        raise ValueError('only the frozen Paper 1 evaluation cohort is accepted')
    scenes, slots, limits = config['scenes'], config['slots'], config['limits']
    for key in ('primary_scenes', 'planned_tasks', 'reserve_starts', 'total_starts',
                'max_replacements_per_slot'):
        if type(limits.get(key)) is not int or limits[key] < 0:
            raise ValueError('explicit nonnegative integer limits required: ' + key)
    if (len(scenes) != limits['primary_scenes'] or not scenes or
            len(slots) != limits['planned_tasks'] or
            limits['total_starts'] != len(slots)+limits['reserve_starts'] or
            limits['reserve_starts'] > 12 or limits['total_starts'] > 224 or
            limits['max_replacements_per_slot'] != 1):
        raise ValueError('manifest counts or replacement limits disagree')
    ids = {s['id'] for s in scenes}
    if len(ids) != len(scenes) or len({s['seed'] for s in scenes}) != len(scenes):
        raise ValueError('duplicate scene IDs or seeds')
    for slot in slots:
        if slot['scene'] not in ids or slot['method'] not in METHODS:
            raise ValueError('unknown scheduled scene or method')
    for scene in scenes:
        for method in ('ours', 'generic'):
            if sum(s['scene'] == scene['id'] and s['method'] == method for s in slots) != 1:
                raise ValueError('exactly one original Ours/Generic pair per scene required')


def validate_production_manifest(config):
    """CLI freezes the study size; smaller internally consistent unit fixtures use analyze()."""
    validate_config(config)
    limits = config['limits']
    if any(limits[k] != v for k, v in dict(primary_scenes=96, planned_tasks=212,
            reserve_starts=12, total_starts=224, max_replacements_per_slot=1).items()):
        raise ValueError('production Paper 1 requires 96 scenes, 212 tasks, 12 reserve, 224 cap')
    version = config.get('evaluation_version')
    if not isinstance(version, dict) or any(not isinstance(version.get(k), str) or not version[k]
                                          for k in ('sim_commit', 'rm4d_baseline_commit')):
        raise ValueError('production evaluation version metadata and SIM/RM4D pins required')
    if Counter(s.get('tier') for s in config['scenes']) != dict(easy=38, moderate=28, hard=30):
        raise ValueError('production IID tier draw must remain 38/28/30')
    expected = dict(ours=96, generic=96, fixed=6, rm4d_only=6, no_cost=4, no_occlusion=4)
    if Counter(s['method'] for s in config['slots']) != expected:
        raise ValueError('production method counts differ from frozen allocation')
    contexts, ablations = set(), set()
    for tier in ('easy', 'moderate', 'hard'):
        group = [s for s in config['scenes'] if s['tier'] == tier]
        group.sort(key=lambda s: s.get('generation_index', config['scenes'].index(s)))
        contexts.update(s['id'] for s in group[:2])
        if tier == 'hard':
            ablations.update(s['id'] for s in group[:4])
    for method in METHODS[2:]:
        if {s['scene'] for s in config['slots'] if s['method'] == method} != (
                contexts if method in ('fixed', 'rm4d_only') else ablations):
            raise ValueError('production auxiliary membership differs from first generated IDs')


def comparison(scenes, rows, other='generic', first_activation=False):
    by_key = {(r['scene'], r['method']): r for r in rows}
    result = dict(n_scheduled=len(scenes), n_complete_pairs=0, a=0, b=0, c=0, d=0,
                  pairs=[], risk_difference=None, risk_difference_bounds=None)
    lower, upper = 0, 0
    for scene in scenes:
        pair = dict(scene=scene['id'], seed=scene['seed'])
        values = []
        for method in ('ours', other):
            row = by_key[(scene['id'], method)]
            value = pair_outcome(row)
            if first_activation:
                value.update(retrieval_success=None, attempt_dir=None)
                if row['attempts']:
                    first = row['attempts'][0]
                    raw, status = first['raw_attempt'], first['effective_status']
                    value.update(status=status, attempt_dir=first['attempt_dir'])
                    value['retrieval_success'] = (False if status == 'INVALID_TRIAL' else
                        raw.get('retrieval_success') if status == 'VALID_TRIAL' and
                        type(raw.get('retrieval_success')) is bool else None)
                starts = [e['raw_entry'].get('start_wall') for e in row.get('entry_only_activations', [])]
                first_time = (row['attempts'][0]['raw_attempt'].get('activation_wall')
                              if row['attempts'] else None)
                if starts and (not finite(first_time) or any(not finite(t) or t <= first_time for t in starts)):
                    value.update(status='UNRESOLVED_ENTRY_ONLY', retrieval_success=None, attempt_dir=None)
            pair[method] = value
            values.append(value['retrieval_success'])
        ours, control = values
        lower += (int(ours) if type(ours) is bool else 0) - (int(control) if type(control) is bool else 1)
        upper += (int(ours) if type(ours) is bool else 1) - (int(control) if type(control) is bool else 0)
        category = None
        if all(type(v) is bool for v in values):
            category = 'a' if ours and control else 'b' if ours else 'c' if control else 'd'
            result[category] += 1
            result['n_complete_pairs'] += 1
        pair['category'] = category
        result['pairs'].append(pair)
    n = len(scenes)
    if n:
        result['risk_difference_bounds'] = [lower/n, upper/n]
        if result['n_complete_pairs'] == n:
            result['risk_difference'] = (result['b']-result['c'])/n
            bi, ci = binomial_interval(result['b'], n, .025), binomial_interval(result['c'], n, .025)
            result['conservative_exact_rd_interval'] = [bi[0]-ci[1], bi[1]-ci[0]]
    return result


def primary_statistics(table):
    n, b, c = table['n_scheduled'], table['b'], table['c']
    bi, ci = binomial_interval(b, n, .025), binomial_interval(c, n, .025)
    return dict(n=n, a=table['a'], b=b, c=c, d=table['d'],
                ours_successes=table['a']+b, generic_successes=table['a']+c,
                risk_difference=(b-c)/n, p_value=exact_mcnemar(b, c),
                test='single two-sided exact McNemar; IID mixture; alpha=0.05',
                b_97_5_percent_interval=bi, c_97_5_percent_interval=ci,
                conservative_exact_rd_interval=[bi[0]-ci[1], bi[1]-ci[0]],
                interval='two aggregate 97.5% Clopper-Pearson intervals; Bonferroni >=95%')


def read_events(directory, missing):
    path = directory/'data/events.jsonl'
    if not path.exists():
        missing.append('events missing')
        return None
    events = []
    try:
        for line in path.read_text().splitlines():
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError('event must be an object')
            events.append(row)
            if row.get('state') in ('LIFT', 'FAILED'):
                # Later cleanup events cannot extend the primary Ground span.
                break
    except (OSError, ValueError) as error:
        missing.append('events unreadable: '+str(error))
        return None
    return events


def rigid(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8) or
            not np.allclose(matrix[:3, :3].T@matrix[:3, :3], np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6)):
        raise ValueError('finite rigid 4x4 transform required')
    return matrix


def saved_mount(directory, config, explicit, missing):
    paths = sorted((directory/'data').glob('rounds/round-*/ranking.json'))
    paths += sorted((directory/'data').glob('init-*/request.json'))
    for path in paths:
        try:
            data = json.loads(path.read_text())
            for obj in (data, data.get('config', {})):
                value = obj.get('sensor', {}).get('T_uav_lidar', obj.get('T_uav_lidar'))
                if value is not None:
                    return rigid(value), str(path.relative_to(directory))
        except (OSError, ValueError, TypeError, AttributeError) as error:
            missing.append('saved sensor mount unreadable: '+str(error))
            return None, None
    value = config.get('sensor_mount_T_uav_lidar', explicit)
    if value is None:
        missing.append('sensor mount missing')
        return None, None
    try:
        return rigid(value), 'manifest' if 'sensor_mount_T_uav_lidar' in config else 'explicit CLI'
    except (ValueError, TypeError) as error:
        missing.append('sensor mount invalid: '+str(error))
        return None, None


def locations(directory, events, count, config, explicit, missing):
    observations = [e for e in events or [] if e.get('state') == 'A5_OBSERVATION']
    by_round = {e['round']: e for e in observations if type(e.get('round')) is int and 1 <= e['round'] <= 3}
    n = count if count is not None else max(by_round, default=0)
    mount, source = saved_mount(directory, config, explicit, missing)
    poses = []
    for number in range(1, n+1):
        event = by_round.get(number)
        pose = None
        try:
            if event is None:
                raise ValueError('accepted observation event missing')
            if mount is None:
                raise ValueError('no known sensor mount')
            filename = 'observation_%02d.npz' % event['round']
            with np.load(directory/'data'/filename, allow_pickle=False) as data:
                matrix = rigid(data['chunk_T_map_sensor'][0]) @ np.linalg.inv(mount)
            pose = [*matrix[:3, 3].tolist(), float(np.arctan2(matrix[1, 0], matrix[0, 0]))]
        except (OSError, ValueError, TypeError, KeyError, IndexError) as error:
            missing.append('packet pose round %s: %s' % (number, error))
        poses.append(pose)
    transitions = []
    for i, (start, stop) in enumerate(zip(poses, poses[1:])):
        row = dict(from_window=i+1, to_window=i+2, displacement_m=None, yaw_change_rad=None,
                   translation=None, yaw=None, category=None)
        if start is not None and stop is not None:
            distance = float(np.linalg.norm(np.array(stop[:3])-start[:3]))
            yaw = abs(math.atan2(math.sin(stop[3]-start[3]), math.cos(stop[3]-start[3])))
            trans, rotate = distance > .2, yaw > .2
            row.update(displacement_m=distance, yaw_change_rad=yaw, translation=trans, yaw=rotate,
                       category='translation_and_yaw' if trans and rotate else 'translation_only'
                       if trans else 'yaw_only' if rotate else 'below_reporting_thresholds')
        transitions.append(row)
    complete = (events is not None and count is not None and len(observations) == len(by_round) == count and
                mount is not None and all(p is not None for p in poses))
    return dict(definition='first accepted packet pose per window; not traveled distance',
                mount_source=source, translation_threshold_m=.2, yaw_threshold_rad=.2,
                poses_map_xyz_yaw=poses, transitions=transitions,
                missing_transitions=sum(t['category'] is None for t in transitions) if count is not None else None,
                resolved_translation_changes=sum(t['translation'] is True for t in transitions) if complete else None,
                resolved_yaw_only_changes=sum(t['category'] == 'yaw_only' for t in transitions) if complete else None,
                resolved_location_changes=sum(t['translation'] or t['yaw'] for t in transitions) if complete else None)


def reduce_slot(row, results_dir, config, explicit):
    missing = []
    row.update(analysis_missing=missing, T_ground_sim=None, observation_locations=None,
               windows=None, nonzero_offset_commands=None)
    if row['selected_attempt'] is None:
        return
    directory = results_dir/row['selected_attempt']
    events = read_events(directory, missing)
    counts = row.get('counts') or {}
    windows = counts.get('completed_windows')
    if type(windows) is not int or not 0 <= windows <= 3:
        windows = None
    row['windows'] = windows
    stamps = [e.get('ros_time') for e in events or []]
    clock_ok = (events is not None and all(finite(t) for t in stamps) and
                all(b >= a for a, b in zip(stamps, stamps[1:])) and not row.get('clock_reset_detected'))
    if events is not None and not clock_ok:
        missing.append('Ground span unavailable: missing/reset simulation clock')
    if clock_ok:
        start = next((e['ros_time'] for e in events if e.get('state') == 'A6_STAGE_START'
                      and e.get('stage') == 'ground_navigation'), None)
        end = next((e['ros_time'] for e in events if e.get('state') in ('LIFT', 'FAILED')), None)
        if start is not None and end is not None and end >= start:
            row['T_ground_sim'] = end-start
    row['observation_locations'] = locations(directory, events, windows, config, explicit, missing)
    # Goals are compared with the saved decision's current pose, never relabeled
    # as observed movement. Missing decision references leave this count unknown.
    previous, decisions, command_flags = None, {}, []
    for event in events or []:
        if event.get('state') == 'A5_OBSERVATION':
            previous = event.get('uav_pose_map')
        elif event.get('state') == 'A5_DECISION' and event.get('next_viewpoint') is not None:
            decisions[tuple(event['next_viewpoint'])] = previous
        elif event.get('state') == 'A5_VIEWPOINT' and event.get('view_role') == 'sensing':
            goal = event.get('goal_map')
            origin = decisions.get(tuple(goal)) if isinstance(goal, list) else None
            if isinstance(origin, list) and len(origin) == 4 and len(goal) == 4 and all(map(finite, origin+goal)):
                delta = np.asarray(goal)-origin
                delta[3] = math.atan2(math.sin(delta[3]), math.cos(delta[3]))
                command_flags.append(bool(np.any(np.abs(delta) > 1e-8)))
            else:
                command_flags.append(None)
    if events is not None and all(flag is not None for flag in command_flags):
        row['nonzero_offset_commands'] = sum(command_flags)


def bootstrap(differences):
    values = np.asarray(differences, dtype=float)
    result = dict(n_pairs=len(values), mean_difference_ours_minus_generic=None,
                  median_difference_ours_minus_generic=None, mean_difference_interval=None,
                  median_difference_interval=None, bootstrap_seed=BOOTSTRAP_SEED,
                  bootstrap_resamples=10000, sparse=len(values) < 10, degenerate=None)
    if len(values):
        draws = np.random.default_rng(BOOTSTRAP_SEED).choice(values, (10000, len(values)))
        result.update(mean_difference_ours_minus_generic=float(np.mean(values)),
                      median_difference_ours_minus_generic=float(np.median(values)),
                      mean_difference_interval=np.quantile(draws.mean(axis=1), [.025, .975]).tolist(),
                      median_difference_interval=np.quantile(np.median(draws, axis=1), [.025, .975]).tolist(),
                      degenerate=bool(np.all(values == values[0])))
    return result


def conditional_efficiency(primary, rows):
    by_slot = {r['slot']: r for r in rows}
    joint = [p for p in primary['pairs'] if p['category'] == 'a']
    metrics = {}
    for metric in ('windows', 'resolved_location_changes', 'T_active_sim', 'T_ground_sim', 'T_task_sim'):
        differences = []
        for pair in joint:
            values = []
            for method in ('ours', 'generic'):
                row = by_slot[pair[method]['slot']]
                values.append((row.get('observation_locations') or {}).get(metric)
                              if metric == 'resolved_location_changes' else row.get(metric))
            if all(finite(v) and v >= 0 for v in values):
                differences.append(values[0]-values[1])
        metrics[metric] = dict(bootstrap(differences), missing_pairs=len(joint)-len(differences))
    return dict(definition='conditional on both succeeding; descriptive whole-scene paired bootstrap',
                joint_success_scenes=len(joint), joint_success_fraction=len(joint)/primary['n_scheduled'],
                metrics=metrics)


def curves(rows, n):
    result = {}
    for method in ('ours', 'generic'):
        group = [r for r in rows if r['method'] == method]
        result[method] = {}
        for metric in ('windows', 'T_active_sim', 'T_task_sim'):
            thresholds = [1, 2, 3] if metric == 'windows' else sorted(
                {0.} | {r[metric] for r in group if finite(r.get(metric)) and r[metric] >= 0})
            curve = []
            for threshold in thresholds:
                lower = upper = 0
                for row in group:
                    success, value = row['retrieval_success'], row.get(metric)
                    if success is False:
                        continue
                    known = finite(value) and value >= 0
                    if success is True and known and value <= threshold:
                        lower += 1
                    if not known or value <= threshold:
                        upper += 1
                curve.append(dict(threshold=threshold, lower=lower/n, upper=upper/n, denominator=n))
            result[method][metric] = curve
    return result


def entry_only_records(results_dir, rows, issues, excluded=()):
    by_slot = {r['slot']: r for r in rows}
    counted = {a['attempt_dir']: a['raw_attempt'] for r in rows for a in r['attempts']}
    counted.update((a['attempt_dir'], a['raw_attempt']) for a in excluded
                   if (a.get('raw_attempt') or {}).get('cohort') == COHORT)
    entries = []
    for path in sorted(results_dir.rglob('entry.json')):
        try:
            record = json.loads(path.read_text())
            if not isinstance(record, dict):
                raise ValueError('entry must be an object')
        except (OSError, ValueError) as error:
            issues.append('unreadable entry record: %s: %s' % (path, error))
            continue
        if record.get('cohort') != COHORT:
            if record.get('kind') == 'EVALUATION_ENTRY':
                issues.append('evaluation entry cohort mismatch: '+str(path))
            continue
        row = by_slot.get(record.get('slot')) if type(record.get('slot')) is int else None
        own = counted.get(str((path.parent/'attempt').relative_to(results_dir)))
        matched = (record.get('kind') == 'EVALUATION_ENTRY' and row is not None and
                   all(record.get(k) == row[k] for k in ('scene', 'seed', 'method')) and
                   (own is None or all(record.get(k) == own.get(k) for k in ('slot', 'scene', 'seed', 'method'))))
        if not matched:
            issues.append('evaluation entry metadata mismatch: '+str(path))
        if own is not None:
            continue
        entry = dict(entry_dir=str(path.parent.relative_to(results_dir)), raw_entry=record,
                     status='UNRESOLVED_ENTRY_ONLY', retrieval_success=None, scheduled_metadata_match=matched)
        entries.append(entry)
        if matched:
            row.setdefault('entry_only_activations', []).append(entry)
            if row['status'] == 'NOT_RUN':
                row['status'] = 'UNRESOLVED_ENTRY_ONLY'
    if entries:
        issues.append('entry-only activations lack attempt classification; no automatic INVALID or rerun')
    return entries


def check_loaded_config(raw, directory, config, issues):
    scene = next(s for s in config['scenes'] if s['id'] == raw['scene'])
    if (raw.get('scene_spec') != scene or raw.get('full_robot_manipulation') is not True or
            raw.get('joint_velocity_feedback') != 'integrated_pose_interval_velocity' or
            raw.get('execution_clearance') != 'chassis-clearance-v1' or not raw.get('ground_dynamics_csv') or
            raw.get('physics_state_use') != 'diagnosis_only_never_algorithm_input'):
        issues.append('slot %s: loaded execution scene/flags differ from freeze' % raw['slot'])
    try:
        snapshot = json.loads((directory.parent/'task.json').read_text())
        if not isinstance(snapshot, dict) or any(snapshot.get(k) != config.get(k) for k in RUNTIME_KEYS):
            raise ValueError('runtime fields differ')
    except (OSError, ValueError) as error:
        issues.append('slot %s: saved task configuration unavailable/mismatched: %s' % (raw['slot'], error))


def analyze(config, results_dir, T_uav_lidar=None):
    """Pure reduction also supports explicit small unit fixtures; production CLI fixes N=96."""
    validate_config(config)
    results_dir = Path(results_dir)
    summary = summarize_pilot(config, results_dir)
    rows, limits, issues = summary['slots'], config['limits'], []
    invalid_diagnostics = []
    entries = entry_only_records(results_dir, rows, issues, summary['excluded_attempts'])
    for row in rows:
        history = row['attempts']
        if all(finite(a['raw_attempt'].get('activation_wall')) for a in history):
            history.sort(key=lambda a: (a['raw_attempt']['activation_wall'], a['attempt_dir']))
            for i, attempt in enumerate(history):
                attempt['rerun_of'] = history[i-1]['attempt_dir'] if i else None
        elif history:
            issues.append('slot %s: missing activation chronology' % row['slot'])
        if len(history) > 1:
            first = history[0]
            reason = first['raw_attempt'].get('reason') or first['raw_attempt'].get('classification_reason')
            if len(history) > 2 or first['effective_status'] != 'INVALID_TRIAL' or not reason:
                issues.append('slot %s: replacement requires one completed documented INVALID_TRIAL' % row['slot'])
            elif (not finite(first['raw_attempt'].get('finish_wall')) or
                  not finite(history[1]['raw_attempt'].get('activation_wall')) or
                  first['raw_attempt']['finish_wall'] > history[1]['raw_attempt'].get('activation_wall', -math.inf)):
                issues.append('slot %s: replacement chronology unavailable or began before invalid classification completed' % row['slot'])
        for attempt in history:
            audit = issues if attempt['attempt_dir'] == row['selected_attempt'] else []
            if 'evaluation_version' in config and attempt['raw_attempt'].get('evaluation_version') != config['evaluation_version']:
                audit.append('slot %s: evaluation version mismatch' % row['slot'])
            version = attempt['raw_attempt'].get('runtime_versions')
            if 'evaluation_version' in config and (not isinstance(version, dict) or
                    any(not isinstance(version.get(key), str) or not version[key]
                        for key in ('agent', 'sim', 'rm4d'))):
                audit.append('slot %s: runtime versions missing' % row['slot'])
            expected = config.get('evaluation_version', {})
            for key, pin in (('sim', 'sim_commit'), ('rm4d', 'rm4d_baseline_commit')):
                if pin in expected and (version if isinstance(version, dict) else {}).get(key) != expected[pin]:
                    audit.append('slot %s: pinned %s runtime differs' % (row['slot'], key))
            if expected:
                check_loaded_config(attempt['raw_attempt'], results_dir/attempt['attempt_dir'], config, audit)
            if attempt['effective_status'] == 'INVALID_TRIAL':
                invalid_diagnostics.append(dict(attempt_dir=attempt['attempt_dir'],
                                                runtime_versions=version, audit_issues=audit))
        reduce_slot(row, results_dir, config, T_uav_lidar)
    rejected_starts = [(a.get('raw_attempt') or {}) for a in summary['excluded_attempts']
                       if (a.get('raw_attempt') or {}).get('cohort') == COHORT]
    rejected_by_slot = Counter(a.get('slot') for a in rejected_starts)
    replacements = sum(max(0, len(r['attempts'])+len(r.get('entry_only_activations', []))+
                           rejected_by_slot[r['slot']]-1) for r in rows)
    replacements += sum(not e['scheduled_metadata_match'] for e in entries)
    total = summary['counts']['total_activations']+len(entries)+len(rejected_starts)
    if replacements > limits['reserve_starts'] or total > limits['total_starts']:
        issues.append('infrastructure reserve or total activation cap exceeded')
    if summary['excluded_attempts']:
        issues.append('excluded attempt records: resolve cohort/slot mismatch before final inference')
    versions = {json.dumps(a['raw_attempt']['runtime_versions'], sort_keys=True)
                for r in rows for a in r['attempts'] if a['attempt_dir'] == r['selected_attempt']
                and 'runtime_versions' in a['raw_attempt']}
    if len(versions) > 1:
        issues.append('runtime versions differ within the fixed cohort')
    complete = not issues and all(r['status'] == 'VALID_TRIAL' for r in rows)
    primary = comparison(config['scenes'], rows)
    tiers = {tier: comparison([s for s in config['scenes'] if s.get('tier') == tier], rows)
             for tier in sorted({s.get('tier', 'unspecified') for s in config['scenes']})}
    auxiliary = {}
    for method in METHODS[2:]:
        selected = {r['scene'] for r in rows if r['method'] == method}
        if selected:
            auxiliary[method] = comparison([s for s in config['scenes'] if s['id'] in selected], rows, method)
    return dict(schema_version=1, cohort=COHORT, status='COMPLETE' if complete else 'INCOMPLETE',
                scope='Paper 1' if len(config['scenes']) == 96 and len(rows) == 212 else
                      'smaller-manifest calculation; not production Paper 1 inference',
                runtime_versions=[json.loads(v) for v in sorted(versions)],
                invalid_attempt_diagnostics=invalid_diagnostics,
                efficiency_clock='simulation', counts=dict(summary['counts'], replacement_starts=replacements,
                    total_activations=total, entry_only_activations=len(entries),
                    excluded_cohort_activations=len(rejected_starts),
                    not_run_slots=sum(r['status'] == 'NOT_RUN' for r in rows)),
                protocol_issues=issues, primary=primary,
                primary_inference=primary_statistics(primary) if complete else None,
                first_activation_invalid_as_failure=comparison(config['scenes'], rows, first_activation=True),
                tiers=tiers, auxiliary=auxiliary, slots=rows,
                conditional_efficiency=conditional_efficiency(primary, rows),
                resource_curves=curves(rows, len(config['scenes'])),
                failure_stages=dict(Counter(r['failure_stage'] or 'unrecorded' for r in rows
                                            if r['retrieval_success'] is False)),
                entry_only_activations=entries,
                excluded_attempts=summary['excluded_attempts'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--results-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--T-uav-lidar', type=Path, help='fallback JSON 4x4 mount, analysis only')
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    validate_production_manifest(config)
    mount = json.loads(args.T_uav_lidar.read_text()) if args.T_uav_lidar else None
    report = analyze(config, args.results_dir, mount)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
