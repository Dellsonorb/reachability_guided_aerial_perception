#!/usr/bin/env python3
"""Completion bounds from recorded snapshots; no ROS, truth, or vote updates.

The first three functions are the entire predictive core. Outcomes and later
snapshots are used only by the reporting/calibration code below them.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def completion_bounds(h, supports, opportunity, steps):
    """OR of candidate AND requirements, bounded without phase independence.

    Rows are first views; columns are second views (one column for steps=1).
    Repeated views represent separate windows, never multiple votes in one.
    Supports must already exclude blocked/clipped candidates in this snapshot.
    """
    h, w = np.asarray(h).ravel(), np.asarray(opportunity)
    if (steps not in (1, 2) or w.ndim != 2 or w.shape[1] != len(h)
            or not np.all(np.isfinite(h) & (h >= 0) & (h == np.floor(h)))
            or not np.all(np.isfinite(w) & (w >= 0) & (w <= 1))):
        raise ValueError('require integer observed counts, finite opportunities and 1/2 steps')
    n = len(w)
    bounds = []
    for hit in (w == 1, w > 0):
        complete = np.zeros((n, n if steps == 2 else 1), dtype=bool)
        for cells in supports:
            cells = np.asarray(cells, dtype=int)
            if not len(cells):
                continue
            if np.any((cells < 0) | (cells >= len(h))):
                raise ValueError('support cell outside grid')
            zero, one = cells[h[cells] == 0], cells[h[cells] == 1]
            if steps == 1:
                if not len(zero):
                    complete[:, 0] |= hit[:, one].all(axis=1)
            else:
                both = hit[:, zero].all(axis=1)
                missing = (~hit[:, one]).astype(np.int32)
                # Each one-vote cell needs at least one of the two views.
                complete |= both[:, None] & both[None, :] & (missing @ missing.T == 0)
        bounds.append(complete)
    assert np.all(~bounds[0] | bounds[1])
    return tuple(bounds)


def transition_graph(poses, offsets, yaw_samples, yaw_cost, facade_position_tolerance=0.05):
    """Restricted graph: saved views also generated locally from first view."""
    poses = np.asarray(poses, dtype=float)
    delta = poses[None, :, :3] - poses[:, None, :3]
    yaw = (poses[None, :, 3] - poses[:, None, 3] + np.pi) % (2*np.pi) - np.pi
    offsets = np.asarray(offsets)
    allowed = (np.isclose(delta[:, :, 0, None], offsets, rtol=0, atol=1e-8).any(axis=2)
               & np.isclose(delta[:, :, 1, None], offsets, rtol=0, atol=1e-8).any(axis=2)
               & np.isclose(delta[:, :, 2], 0, rtol=0, atol=1e-8)
               & np.isclose(yaw*yaw_samples/(2*np.pi), np.rint(yaw*yaw_samples/(2*np.pi)), rtol=0, atol=1e-8))
    # A5 rejects displaced/yaw-only commands inside facade position tolerance.
    distance = np.linalg.norm(delta, axis=2)
    allowed &= distance > facade_position_tolerance
    # The unchanged current view remains available for another sensor window.
    np.fill_diagonal(allowed, True)
    return allowed, distance + yaw_cost*np.abs(yaw)


def select_sequence(lower, upper, allowed, costs, historical_first=None):
    """Completion tier first, unchanged motion-cost proxy second, stable IDs last."""
    tier = np.where(lower, 2, np.where(upper, 1, 0))
    tier = np.where(allowed, tier, -1)
    best = int(tier.max())
    hist_tier = None if historical_first is None else int(tier[historical_first].max())
    result = dict(best_tier=best, historical_first_tier=hist_tier,
                  first_index=None, second_index=None, total_cost=None,
                  changed=None, strict_improvement=None)
    if best <= 0:
        return result
    choices = np.argwhere(tier == best)
    i, j = min(choices, key=lambda ij: (float(costs[tuple(ij)]), int(ij[0]), int(ij[1])))
    result.update(first_index=int(i), second_index=int(j), total_cost=float(costs[i, j]),
                  changed=None if historical_first is None else bool(i != historical_first),
                  strict_improvement=None if hist_tier is None else bool(best > hist_tier))
    return result


def read_json(path):
    return json.loads(path.read_text())


def phase_windows(packet_hits, horizon):
    """Keep joint cell masks at each cyclic scan start; never observed votes."""
    hits = np.asarray(packet_hits, dtype=bool)
    if horizon < 1:
        raise ValueError('positive packet horizon required')
    if horizon >= len(hits):
        return np.broadcast_to(hits.any(axis=0), hits.shape).copy()
    cumulative = np.concatenate((np.zeros_like(hits[:1], dtype=np.int32),
        np.cumsum(np.concatenate((hits, hits)), axis=0, dtype=np.int32)))
    starts = np.arange(len(hits))
    return cumulative[starts+horizon] - cumulative[starts] > 0


def read_npz(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k] for k in z.files}


def eligible(assessments):
    return [a for a in assessments if not a['operational']['blocked'] and not a['footprint_clipped']]


def pose_array(ranking):
    candidates = ranking['candidates']
    assert [c['candidate_id'] for c in candidates] == list(range(len(candidates)))
    return np.array([c['viewpoint']['position_xyz']+[c['viewpoint']['yaw_rad']] for c in candidates])


def commanded_index(ranking, event):
    view = None if event is None else event.get('next_viewpoint')
    if view is None:
        return None
    poses = pose_array(ranking)
    yaw = (poses[:, 3]-view[3]+np.pi) % (2*np.pi)-np.pi
    matches = np.flatnonzero(np.all(np.isclose(poses[:, :3], view[:3], rtol=0, atol=1e-8), axis=1)
                            & np.isclose(yaw, 0, rtol=0, atol=1e-8))
    if len(matches) != 1:
        raise ValueError('historical commanded view does not uniquely match snapshot catalogue')
    return int(matches[0])


def candidate_stats(assessment, h, round_number):
    op = assessment['operational']
    cells = op['covered_cells']
    values = h[cells]
    valid = not op['blocked'] and not assessment['footprint_clipped']
    missing, zero = int(np.count_nonzero(values < 2)), int(np.count_nonzero(values == 0))
    assert missing == op['ground_missing_cells']
    assert assessment['confirmed'] == bool(valid and len(cells) and missing == 0)
    return dict(source_id=assessment['source_id'], candidate_id=assessment['candidate_id'],
                evaluation_index=assessment['evaluation_index'], relevance=assessment['relevance'],
                viable=valid, blocked=op['blocked'], clipped=assessment['footprint_clipped'],
                cells=len(cells), zero_cells=zero, one_vote_cells=int(np.count_nonzero(values == 1)),
                missing_cells=missing, missing_tickets=int(np.maximum(0, 2-values).sum()),
                confirmed=assessment['confirmed'],
                within_remaining_vote_budget=bool(valid and len(cells) and np.all(np.maximum(0, 2-values) <= 3-round_number)))


def snapshot_prediction(ranking, decision, h, fields, historical, remaining, facade_tolerance):
    """Current-state inputs only. No historical outcome or future sensor packet."""
    candidates = eligible(decision['assessments'])
    supports = [a['operational']['covered_cells'] for a in candidates]
    w = fields['observation_opportunity'].reshape(len(ranking['candidates']), -1)
    lower, upper = completion_bounds(h, supports, w, remaining)
    poses = pose_array(ranking)
    valid = np.array([c['status'] == 'VALID' for c in ranking['candidates']])
    first_cost = np.array([c['flight_cost'] for c in ranking['candidates']])
    if remaining == 2:
        config = ranking['config']
        allowed, travel = transition_graph(poses, config['xy_offsets_m'], config['yaw_samples'], config['yaw_cost_m_per_rad'], facade_tolerance)
        allowed &= valid[:, None] & valid[None, :]
        costs = first_cost[:, None] + travel
    else:
        allowed, costs = valid[:, None], first_cost[:, None]
    choice = select_sequence(lower, upper, allowed, costs, historical)
    choice.update(all_phase_sequence_exists=bool(np.any(lower & allowed)),
                  any_phase_sequence_not_excluded=bool(np.any(upper & allowed)),
                  allowed_sequences=int(allowed.sum()),
                  historical_view_id=historical,
                  predicted_first_pose=None if choice['first_index'] is None else poses[choice['first_index']].tolist(),
                  predicted_second_pose=None if choice['first_index'] is None or remaining == 1 else poses[choice['second_index']].tolist())
    # Both one-step policies on exactly this state, not crossed method histories.
    for name, key in [('generic', 'best_generic_id'), ('ours', 'best_task_id')]:
        idx = ranking.get(key)
        choice[name+'_first_tier'] = None if idx is None else int(np.where(allowed[idx], np.where(lower[idx], 2, np.where(upper[idx], 1, 0)), -1).max())
    return choice


def transition_calibration(prior, after, historical):
    """Next saved observation is an outcome label, never a predictor input."""
    rank, decision, h, fields = prior
    _, next_decision, next_h, _ = after
    delta = next_h - h
    if np.any((delta < 0) | (delta > 1)):
        raise ValueError('saved ground support must add at most one vote per accepted window')
    actual_hit = delta == 1
    w = fields['observation_opportunity'][historical].ravel()
    possible, certain = w > 0, w == 1
    assessments = eligible(decision['assessments'])
    union = np.unique(np.concatenate([a['operational']['covered_cells'] for a in assessments])) if assessments else np.array([], int)
    needed = union[h[union] < 2]
    result = dict(model_all_phase_hit_cells=int(certain.sum()), all_phase_actual_miss_cells=int((certain & ~actual_hit).sum()),
                  model_any_phase_hit_cells=int(possible.sum()), any_phase_actual_miss_cells=int((possible & ~actual_hit).sum()),
                  actual_hit_cells=int(actual_hit.sum()), actual_outside_model_cells=int((actual_hit & ~possible).sum()),
                  needed_cells=len(needed), needed_all_phase_hits=int(certain[needed].sum()),
                  needed_all_phase_misses=int((certain[needed] & ~actual_hit[needed]).sum()),
                  needed_actual_hits=int(actual_hit[needed].sum()))
    single_w = w[None]
    supports = [a['operational']['covered_cells'] for a in assessments]
    lo, hi = completion_bounds(h, supports, single_w, 1)
    lookup = {a['source_id']: a for a in next_decision['assessments']}
    actual_support, newly_blocked, false_certificate = [], 0, 0
    for a in assessments:
        cells = a['operational']['covered_cells']
        actual_support.append(bool(np.all(next_h[cells] >= 2)))
        newly_blocked += int(lookup[a['source_id']]['operational']['blocked'])
        own_lo, _ = completion_bounds(h, [cells], single_w, 1)
        false_certificate += int(own_lo.any() and not np.all(next_h[cells] >= 2))
    result.update(predicted_confirmation_all_phase=bool(lo.any()), predicted_confirmation_any_phase=bool(hi.any()),
                  actual_prior_candidate_support_complete=any(actual_support),
                  actual_confirmation=bool(next_decision['confirmed_candidate_count']),
                  newly_blocked_prior_viable=newly_blocked, all_phase_candidate_support_false_positive=false_certificate)
    return result


def write_csv(path, rows):
    if not rows:
        return
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, keys, lineterminator='\n'); writer.writeheader()
        writer.writerows({k: json.dumps(v, separators=(',', ':')) if isinstance(v, (dict, list)) else v for k, v in r.items()} for r in rows)


def analyze(results, outdir):
    results, outdir = Path(results).resolve(), Path(outdir).resolve()
    if outdir == results or results in outdir.parents:
        raise ValueError('derived analysis must be outside original results')
    slots = list(csv.DictReader((results/'slots.csv').open()))
    if len(slots) != 212 or len({r['slot'] for r in slots}) != 212:
        raise ValueError('require all 212 selected task records')
    trajectory, candidate_rows, predictions, calibrations, accounting = [], [], [], [], []
    for count, row in enumerate(slots, 1):
        meta = dict(slot=int(row['slot']), scene=row['scene'], method=row['method'], tier=row['tier'],
                    role=row['comparison_role'], retrieval_success=row['retrieval_success']=='True',
                    confirmation_failure='VIEW_BUDGET_REACHED' in row['failure_reason'] and row['D_env']=='False')
        data = results/row['selected_attempt']/'data'
        events = [json.loads(line) for line in (data/'events.jsonl').read_text().splitlines()]
        decisions = {e['round']: e for e in events if e.get('state') == 'A5_DECISION'}
        selected = next((e for e in events if e.get('state') == 'A5_SELECTED'), None)
        fixed_source = None
        snapshots = []
        missing = []
        for n in range(1, int(row['windows'])+1):
            directory = data/'rounds'/f'round-{n:02d}'
            files = ['ranking.json', 'decision.json', 'fields.npz', 'operational_evidence.npz']
            absent = [name for name in files if not (directory/name).is_file()]
            if absent:
                missing.append(dict(round=n, files=absent)); continue
            ranking, decision = read_json(directory/'ranking.json'), read_json(directory/'decision.json')
            fields, evidence = read_npz(directory/'fields.npz'), read_npz(directory/'operational_evidence.npz')
            h = evidence['ground_presence_votes'].ravel()
            assert ranking['a2_config']['free_observations'] == 2
            assert decision['anchor_semantics'] == 'exact-validated-winner-v1.2'
            assert np.array_equal(fields['a2_observation_count'].shape, evidence['ground_presence_votes'].shape)
            stats = [candidate_stats(a, h, n) for a in decision['assessments']]
            viable = [s for s in stats if s['viable']]
            if n == 1 and viable:
                fixed_source = min(viable, key=lambda s: (s['missing_tickets'], -s['relevance'], s['evaluation_index']))['source_id']
            fixed = next((s for s in stats if s['source_id'] == fixed_source), None)
            for s in stats:
                candidate_rows.append(meta | dict(round=n, fixed_round1_anchor=s['source_id']==fixed_source,
                    eventual_selected_anchor=selected is not None and s['source_id']==selected['source_id']) | s)
            record = meta | dict(round=n, snapshot=str(directory.relative_to(results)),
                    candidates=len(stats), viable=len(viable), confirmed=decision['confirmed_candidate_count'],
                    vote_budget_possible_candidates=sum(s['within_remaining_vote_budget'] for s in viable),
                    minimum_zero_cells=min((s['zero_cells'] for s in viable), default=None),
                    minimum_missing_cells=min((s['missing_cells'] for s in viable), default=None),
                    minimum_missing_tickets=min((s['missing_tickets'] for s in viable), default=None),
                    fixed_source=fixed_source, fixed_missing_tickets=None if fixed is None else fixed['missing_tickets'],
                    fixed_missing_cells=None if fixed is None else fixed['missing_cells'],
                    fixed_viable=None if fixed is None else fixed['viable'],
                    task_mass=decision['task_uncertainty_mass'])
            trajectory.append(record)
            snapshot = (ranking, decision, h, fields)
            historical = commanded_index(ranking, decisions.get(n))
            if n < 3:
                runtime_config = read_json(data/'initial.json')['config']
                pred = snapshot_prediction(*snapshot, historical, 3-n, runtime_config['facade_position_tolerance'])
                predictions.append(meta | dict(round=n, already_confirmed=decision['confirmed_candidate_count']>0) | pred)
            if snapshots and snapshots[-1][0] == n-1:
                prior_n, prior = snapshots[-1]
                prior_hist = commanded_index(prior[0], decisions.get(prior_n))
                if prior_hist is not None:
                    calibrations.append(meta | dict(from_round=prior_n, to_round=n, historical_view_id=prior_hist)
                                        | transition_calibration(prior, snapshot, prior_hist))
            snapshots.append((n, snapshot))
        accounting.append(meta | dict(status=row['status'], failure_reason=row['failure_reason'],
                          recorded_windows=int(row['windows']), available_snapshots=len(snapshots), missing=missing,
                          evidence_applicability='BYPASS' if row['method']=='rm4d_only' else 'OBSERVATION_PIPELINE'))
        if count % 20 == 0:
            print(f'processed {count}/212 tasks; {len(trajectory)} snapshots', flush=True)
    outdir.mkdir(parents=True, exist_ok=True)
    tables = dict(accounting=accounting, trajectories=trajectory, candidates=candidate_rows,
                  predictions=predictions, actual_transitions=calibrations)
    for name, rows in tables.items():
        write_csv(outdir/(name+'.csv'), rows)
    summary = summarize(tables)
    (outdir/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(json.dumps(summary, indent=2), flush=True)
    return tables, summary


def summarize(tables):
    summary = dict(tasks=len(tables['accounting']), snapshots=len(tables['trajectories']),
                   candidate_records=len(tables['candidates']), transitions=len(tables['actual_transitions']),
                   missing_snapshots=[r for r in tables['accounting'] if r['missing']],
                   evidence_semantics='fixed-snapshot nominal model bounds, not measured counterfactual retrieval')
    groups = {
        'generic_confirmation_failures': lambda r: r['role']=='primary' and r['method']=='generic' and r['confirmation_failure'],
        'ours_successes': lambda r: r['role']=='primary' and r['method']=='ours' and r['retrieval_success'],
        'ours_confirmation_failures': lambda r: r['role']=='primary' and r['method']=='ours' and r['confirmation_failure'],
        'generic_successes': lambda r: r['role']=='primary' and r['method']=='generic' and r['retrieval_success'],
        'all_primary': lambda r: r['role']=='primary',
    }
    for name, keep in groups.items():
        part = {'tasks': sum(keep(r) for r in tables['accounting']), 'rounds': {}}
        for n in (1, 2, 3):
            tr = [r for r in tables['trajectories'] if keep(r) and r['round']==n]
            pr = [r for r in tables['predictions'] if keep(r) and r['round']==n]
            unresolved = [r for r in pr if not r['already_confirmed']]
            part['rounds'][n] = dict(available=len(tr), any_confirmed=sum(r['confirmed']>0 for r in tr),
                none_within_vote_budget=sum(r['vote_budget_possible_candidates']==0 for r in tr),
                all_phase_completion=sum(r['all_phase_sequence_exists'] for r in pr),
                optimistic_completion=sum(r['any_phase_sequence_not_excluded'] for r in pr),
                recommendations=sum(r['first_index'] is not None for r in pr),
                historical_commands=sum(r['historical_view_id'] is not None for r in pr),
                changed=sum(r['changed'] is True for r in pr),
                strict_completion_tier_improvement=sum(r['strict_improvement'] is True for r in pr),
                historical_first_impossible=sum(r['historical_first_tier']==0 for r in pr),
                unresolved_states=len(unresolved),
                unresolved_all_phase_completion=sum(r['all_phase_sequence_exists'] for r in unresolved),
                unresolved_optimistic_completion=sum(r['any_phase_sequence_not_excluded'] for r in unresolved),
                minimum_missing_median=None if not tr else float(np.median([r['minimum_missing_cells'] for r in tr if r['minimum_missing_cells'] is not None])))
        ca = [r for r in tables['actual_transitions'] if keep(r)]
        part['actual_transition_validation'] = dict(n=len(ca),
            all_phase_predicted_confirmations=sum(r['predicted_confirmation_all_phase'] for r in ca),
            all_phase_predictions_actual_confirmed=sum(r['predicted_confirmation_all_phase'] and r['actual_confirmation'] for r in ca),
            all_phase_predictions_actual_support_complete=sum(r['predicted_confirmation_all_phase'] and r['actual_prior_candidate_support_complete'] for r in ca),
            any_phase_predicted_confirmations=sum(r['predicted_confirmation_any_phase'] for r in ca),
            actual_confirmations=sum(r['actual_confirmation'] for r in ca),
            actual_confirmation_outside_optimistic_prediction=sum(r['actual_confirmation'] and not r['predicted_confirmation_any_phase'] for r in ca),
            needed_all_phase_hits=sum(r['needed_all_phase_hits'] for r in ca),
            needed_all_phase_misses=sum(r['needed_all_phase_misses'] for r in ca))
        part['actual_transition_validation']['all_phase_candidate_support_false_positives'] = sum(r['all_phase_candidate_support_false_positive'] for r in ca)
        summary[name] = part
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--results', type=Path, default=Path('outputs/paper1-final-eval-v1'))
    parser.add_argument('--outdir', type=Path, default=Path(__file__).parent/'results')
    args = parser.parse_args()
    analyze(args.results, args.outdir)
