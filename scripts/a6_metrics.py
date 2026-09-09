"""Pure Python 3.8 accounting for the A6 JSONL simulation-time evidence."""

import math


MAX_SAMPLE_GAP_S = .3
TF_MAX_AGE_S = .5
STAGES = ('preflight', 'takeoff', 'initial_view', 'aerial_observe', 'initial_query',
          'active', 'execution_screen', 'return', 'landing', 'ground_navigation', 'ground_refine',
          'refined_pregrasp', 'descend', 'close', 'lift', 'retention')
STATUS_STAGE = {'PREFLIGHT': 'preflight', 'TAKEOFF': 'takeoff',
                'AIR_VIEW': 'initial_view', 'AIR_OBSERVE': 'aerial_observe',
                'LANDING': 'landing', 'GROUND_APPROACH': 'ground_navigation',
                'GROUND_OBSERVE': 'ground_refine'}


def _reset(rows):
    times = [row['ros_time'] for row in rows]
    return any(later < earlier for earlier, later in zip(times, times[1:]))


def _duration(start, end, clock_reset=False):
    return None if clock_reset or start is None or end is None or end < start else end - start


def path_metrics(samples, body, start, end, dimensions=3, clock_reset=False):
    """Sum raw valid neighboring samples, never crossing a missing sample/gap.

    A complete distance is unavailable if any part of the requested interval
    lacks evidence. The separately named observed distance remains a lower
    bound. Sampling/TF stamps are simulation time; yaw is accumulated separately.
    """
    result = dict(start_sim=start, end_sim=end, complete=False, distance_m=None,
                  observed_distance_lower_bound_m=0., yaw_travel_rad=None,
                  observed_yaw_lower_bound_rad=0., sample_count=0,
                  missing_samples=0, gap_count=0, clock_reset=bool(clock_reset))
    if start is None or end is None or end < start:
        return result
    samples = list(samples)
    if clock_reset or _reset(samples):
        result['clock_reset'] = True
        return result
    rows = [row for row in samples if start <= row['ros_time'] <= end]
    result['sample_count'] = len(rows)
    complete = bool(rows)
    if not rows:
        return result
    if rows[0]['ros_time'] - start > MAX_SAMPLE_GAP_S + 1e-9:
        result['gap_count'] += 1
        complete = False
    if end - rows[-1]['ros_time'] > MAX_SAMPLE_GAP_S + 1e-9:
        result['gap_count'] += 1
        complete = False
    previous = None
    for row in rows:
        pose = row.get(body)
        try:
            xyz = pose['xyz']
            yaw = float(pose['yaw'])
            age = row['ros_time'] - pose['stamp_s']
            valid = (len(xyz) == 3 and all(math.isfinite(value) for value in xyz)
                     and math.isfinite(yaw) and 0 <= age <= TF_MAX_AGE_S + 1e-9)
        except (TypeError, KeyError, ValueError):
            valid = False
        if not valid:
            result['missing_samples'] += 1
            complete = False
            previous = None
            continue
        if previous is not None:
            gap = row['ros_time'] - previous[0]
            if gap > MAX_SAMPLE_GAP_S + 1e-9:
                result['gap_count'] += 1
                complete = False
            else:
                result['observed_distance_lower_bound_m'] += math.sqrt(sum(
                    (xyz[index] - previous[1][index]) ** 2 for index in range(dimensions)))
                delta = yaw - previous[2]
                result['observed_yaw_lower_bound_rad'] += abs(math.atan2(math.sin(delta), math.cos(delta)))
        previous = row['ros_time'], xyz, yaw
    result['complete'] = complete
    if complete:
        result['distance_m'] = result['observed_distance_lower_bound_m']
        result['yaw_travel_rad'] = result['observed_yaw_lower_bound_rad']
    return result


def _stage_summary(events, terminal, clock_reset):
    stages = {name: dict(status='NOT_REACHED', start_sim=None, end_sim=None,
                         duration_sim_s=None) for name in STAGES}
    current, failure_stage, failure_reason = None, None, None

    def finish(name, timestamp, status):
        if name is not None and (stages[name]['status'] == 'IN_PROGRESS' or
                                 (status == 'FAILED' and stages[name]['status'] == 'SUCCEEDED')):
            stages[name].update(status=status, end_sim=timestamp,
                                duration_sim_s=_duration(stages[name]['start_sim'], timestamp, clock_reset))

    for row in events:
        state, timestamp = row['state'], row['ros_time']
        new_stage = STATUS_STAGE.get(state)
        if state == 'A5_CORE' and row.get('operation') == 'init':
            new_stage = 'initial_query'
        elif state == 'A6_CAPTURE_START':
            new_stage = 'active'
        elif state == 'A5_VIEWPOINT' and row.get('view_role') == 'return':
            new_stage = 'return'
        elif state == 'A6_STAGE_START':
            new_stage = row['stage']
        if new_stage is not None and new_stage != current:
            finish(current, timestamp, 'SUCCEEDED')
            current = new_stage
            if stages[current]['status'] == 'NOT_REACHED':
                stages[current].update(status='IN_PROGRESS', start_sim=timestamp)
        if state in ('A6_STAGE_END', 'A6_STAGE_FAILED'):
            stage = row['stage']
            failed = state == 'A6_STAGE_FAILED'
            finish(stage, timestamp, 'FAILED' if failed else 'SUCCEEDED')
            if failed and failure_stage is None:
                failure_stage, failure_reason = stage, row.get('reason')
        elif state in ('GROUND_STOPPED', 'GROUND_REFINED', 'A6_LANDED', 'A6_ACTIVE_STOP'):
            stage = {'GROUND_STOPPED': 'ground_navigation', 'A6_LANDED': 'landing',
                     'GROUND_REFINED': 'ground_refine', 'A6_ACTIVE_STOP': 'active'}[state]
            finish(stage, timestamp, 'SUCCEEDED')
        elif state == 'FAILED':
            finish(current, timestamp, 'FAILED')
            if failure_stage is None:
                failure_stage, failure_reason = current, row.get('reason')
        if row is terminal:
            finish(current, timestamp, 'FAILED' if state == 'FAILED' else 'SUCCEEDED')
            break
    return stages, failure_stage, failure_reason


def summarize_metrics(events, samples, method, run_result=None):
    """Reduce retained evidence without treating controller LIFT as physical truth."""
    events, samples = list(events), list(samples)
    first = lambda state: next((row for row in events if row['state'] == state), None)
    stamp = lambda row: None if row is None else row['ros_time']
    terminal = next((row for row in events if row['state'] in ('LIFT', 'FAILED')), None)
    task_start, task_end = stamp(first('A6_TASK_START')), stamp(terminal or first('A6_TASK_END'))
    primary = events[:events.index(terminal) + 1] if terminal is not None else events
    # The terminal sample is taken immediately after the terminal event. Its
    # wall stamp identifies the sequence boundary even if /clock resets during
    # later cleanup; wall time is never used to calculate a robot duration.
    primary_samples = []
    for row in samples:
        if terminal is not None and row.get('wall_monotonic', -math.inf) > terminal['wall_monotonic']:
            if row['ros_time'] == terminal['ros_time']:
                primary_samples.append(row)
            break
        primary_samples.append(row)
    reset = _reset(primary) or _reset(primary_samples)
    cleanup_reset = _reset(events[len(primary) - 1:]) if terminal is not None else False
    active_start = stamp(first('A6_CAPTURE_START'))
    active_end = stamp(first('A6_ACTIVE_STOP')) if first('A6_ACTIVE_STOP') else task_end
    landed, ended = stamp(first('A6_LANDED')), stamp(first('A6_TASK_END'))
    ready = next((row for row in primary if row['state'] == 'A6_EXEC_READY'), None)
    environment = [row for row in primary if row['state'] == 'A6_ENV_RESULT']
    discovered = next((row for row in environment if row['confirmed_candidate_count'] > 0), None)
    count = lambda state: sum(row['state'] == state for row in primary)
    views = [row for row in primary if row['state'] == 'A5_VIEWPOINT']
    sensing = [row for row in views if row.get('view_role') == 'sensing']
    initial_count = sum(row.get('view_role') == 'initial' for row in views)
    counts = dict(capture_calls=count('A6_CAPTURE_START'), discarded_windows=count('A5_CAPTURE_RETRY'),
                  completed_windows=count('A5_OBSERVATION'), voted_windows=len(environment),
                  sensing_visits=initial_count + len(sensing),
                  sensing_rescans=sum(bool(row['rescan']) for row in sensing),
                  nbv_moves=sum(not row['rescan'] for row in sensing), initial_outbound=initial_count,
                  return_actions=sum(row.get('view_role') == 'return' for row in views))
    uav_end = landed if landed is not None and task_end is not None and landed <= task_end else task_end
    paths = dict(uav_total=path_metrics(primary_samples, 'uav', task_start, uav_end, clock_reset=reset),
                 ground_total=path_metrics(primary_samples, 'ground', task_start, task_end, dimensions=2, clock_reset=reset),
                 uav_active=path_metrics(primary_samples, 'uav', active_start, active_end, clock_reset=reset),
                 uav_cleanup=path_metrics(samples, 'uav', task_end, ended, clock_reset=cleanup_reset))
    if method == 'rm4d_only':
        paths['uav_active'].update(start_sim=None, end_sim=None, complete=True, distance_m=0.,
                                   yaw_travel_rad=0.)
    stages, failed_stage, failed_reason = _stage_summary(primary, terminal, reset)
    return dict(schema_version=1, method=method, efficiency_clock='simulation',
                clock_reset_detected=reset, cleanup_clock_reset_detected=cleanup_reset,
                task_start_sim=task_start, task_end_sim=task_end,
                active_start_sim=active_start, active_end_sim=active_end if active_start is not None else None,
                landed_sim=landed, first_env_sim=stamp(discovered), exec_ready_sim=stamp(ready),
                T_task_sim=_duration(task_start, task_end, reset),
                T_active_sim=0. if method == 'rm4d_only' else _duration(active_start, active_end, reset),
                T_first_env_sim=_duration(active_start, stamp(discovered), reset),
                T_exec_ready_sim=_duration(task_start, stamp(ready), reset),
                D_exec=ready is not None, D_env=None if method == 'rm4d_only' else discovered is not None,
                C_env_count_final=None if not environment else environment[-1]['confirmed_candidate_count'],
                C_env_count_max=None if not environment else max(row['confirmed_candidate_count'] for row in environment),
                environment_decisions=environment, counts=counts, paths=paths, stages=stages,
                terminal_status=None if terminal is None else terminal['state'],
                terminal_failure_stage=failed_stage, terminal_failure_reason=failed_reason,
                adapter_run_result=run_result, physical_success=None)
