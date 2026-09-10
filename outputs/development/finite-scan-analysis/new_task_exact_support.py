#!/usr/bin/env python3
"""Read-only exact-winner/non-winner audit of completed saved decisions.

Use the existing diagnostic-only evaluator, without selecting alternatives or
changing ground votes. Geometry is observed/perceived operational geometry,
never simulator ground truth. Output snapshots must have new filenames.
"""

import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from operational_occlusion_review import ROOT, HERE, load_state
from new_task_acquisition import events_from
from operational_gating import ambiguous_footprint_diagnostics
from reachability_guided_aerial_perception import GraspTCP
from replay_exact_support import nonwinner_diagnostics
from sim_active_perception.core import A5Config, assess_candidates, build_support_task, candidate_catalog
from sim_active_perception.worker import make_field


def plain(value):
    return json.loads(json.dumps(value, allow_nan=False))


def classify(row, context, footprint, *, gate_key='operational'):
    gate = row[gate_key]
    amb = asdict(ambiguous_footprint_diagnostics(context, (row['x'], row['y']), row['yaw'], footprint))
    causes = []
    if gate['environment_cells']:
        causes.append('observed_environment_full_cells')
    if amb['intersecting_endpoint_indices']:
        causes.append('observed_ambiguous_endpoint_disks')
    if amb['legacy_fallback_cell_ids']:
        causes.append('incomplete_ambiguous_history_full_cells')
    if gate['target_collision']:
        causes.append('perceived_expanded_target_rectangle')
    assert bool(causes) == gate['blocked']
    ids = np.asarray(gate['covered_cells'], dtype=int)
    presence = context.ground_presence_votes.ravel()[ids]
    missing = ids[presence < context.config.free_observations]
    assert len(missing) == gate['ground_missing_cells']
    return dict(blocker_causes=causes, geometry_blocked=gate['blocked'],
        footprint_clipped=gate['footprint_clipped'], ground_supported=gate['ground_supported'],
        missing_ground_cell_ids=missing.tolist(),
        ground_presence_histogram={str(int(v)): int(np.count_nonzero(presence == v)) for v in np.unique(presence)},
        ambiguous_endpoint_intersections=len(amb['intersecting_endpoint_indices']),
        ambiguous_fallback_cell_ids=amb['legacy_fallback_cell_ids'],
        coarse_ambiguous_but_aliased_clear_cell_ids=amb['aliased_clear_cell_ids'])


def summarize_winners(rows):
    causes = Counter(c for row in rows for c in row['audit']['blocker_causes'])
    return dict(exact_winners=len(rows), geometry_blocked=sum(r['audit']['geometry_blocked'] for r in rows),
        geometry_clear_but_ground_missing=sum(not r['audit']['geometry_blocked'] and bool(r['audit']['missing_ground_cell_ids']) for r in rows),
        geometry_clear_and_ground_supported=sum(not r['audit']['geometry_blocked'] and r['audit']['ground_supported'] for r in rows),
        confirmed=sum(r['confirmed'] for r in rows), footprint_clipped=sum(r['footprint_clipped'] for r in rows),
        blocked_but_ground_supported=sum(r['audit']['geometry_blocked'] and r['audit']['ground_supported'] for r in rows),
        blocked_and_ground_missing=sum(r['audit']['geometry_blocked'] and bool(r['audit']['missing_ground_cell_ids']) for r in rows),
        overlapping_blocker_counts=dict(causes),
        geometry_clear_sources=[r['source_id'] for r in rows if not r['audit']['geometry_blocked']],
        confirmed_sources=[r['source_id'] for r in rows if r['confirmed']])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    if out.parent != HERE or not out.name.startswith('new_task_') or out.exists() or out.with_suffix('.md').exists():
        parser.error('new output must be directly inside finite-scan-analysis with new_task_ prefix')
    result = dict(created_utc=datetime.now(timezone.utc).isoformat(), diagnostic_only=True,
        selection_changed=False, no_gt_geometry=True, no_generated_observations=True,
        geometry_semantics='observed/perceived operational gate, not simulator truth or full executable feasibility',
        nonwinner_semantics='existing replay_exact_support.nonwinner_diagnostics; exact clear non-winners do not replace frozen winners',
        runs=[])
    for run in sorted((ROOT / 'outputs/development/finite-scan').glob('launch-*')):
        slot = int(run.name.split('-')[1])
        if not 2 <= slot <= 5:
            continue
        data = run / 'attempt/data'
        if not (data / 'initial.json').exists():
            continue
        initial = json.loads((data / 'initial.json').read_text())
        config = A5Config(**initial['config'])
        raw = initial['result']
        field = make_field(GraspTCP(**initial['grasp']), raw, config)
        events = events_from(data / 'events.jsonl')
        entry = dict(run=run.name, data_path=str(data),
            task_end_recorded=any(e.get('state') == 'A6_TASK_END' for e in events),
            last_event=events[-1].get('state') if events else None, rounds=[])
        for folder in sorted((data / 'rounds').glob('round-*')):
            if not (folder / 'decision.json').exists():
                continue
            state = load_state(folder)
            belief, context = state['belief'], state['operational']
            original_presence = context.ground_presence_votes.copy()
            task = build_support_task(field, raw, belief, config, operational=context)
            assessments = assess_candidates(field, belief, candidate_catalog(field, raw), task=task, operational=context)
            checks = dict(exact_anchors=plain([asdict(a) for a in task.winner_anchors]) == state['ranking']['a3_summary']['winner_anchors'],
                exact_assessments=plain(assessments) == state['decision']['assessments'])
            assert all(checks.values()), (run.name, folder.name, checks)
            winners = [dict(row, audit=classify(row, context, task.footprint)) for row in plain(assessments)]
            alternatives = nonwinner_diagnostics(field, raw, context, task.footprint)
            for cell in alternatives['cells']:
                cell['winner']['audit'] = classify(cell['winner'], context, task.footprint, gate_key='gate')
                for candidate in cell['unblocked_nonwinners']:
                    candidate['audit'] = classify(candidate, context, task.footprint, gate_key='gate')
            alt_rows = [a for c in alternatives['cells'] for a in c['unblocked_nonwinners']]
            alternatives['unblocked_nonwinners_ground_supported'] = sum(a['gate']['ground_supported'] for a in alt_rows)
            alternatives['unblocked_nonwinners_ground_missing'] = sum(bool(a['audit']['missing_ground_cell_ids']) for a in alt_rows)
            alternatives['unblocked_nonwinners_clipped'] = sum(a['gate']['footprint_clipped'] for a in alt_rows)
            alternatives['blocked_winner_cells_with_ground_supported_nonwinner'] = sum(
                any(a['gate']['ground_supported'] for a in c['unblocked_nonwinners']) for c in alternatives['cells'])
            checks['presence_unchanged'] = bool(np.array_equal(original_presence, context.ground_presence_votes))
            assert checks['presence_unchanged']
            row = dict(round=int(folder.name.split('-')[-1]), checks=checks,
                summary=summarize_winners(winners), frozen_selected_candidate=state['decision'].get('selected_candidate'),
                winner_assessments=winners, nonwinner_diagnostics=alternatives)
            entry['rounds'].append(row)
            print(json.dumps(dict(run=run.name, round=row['round'], summary=row['summary'],
                nonwinners={k: v for k, v in alternatives.items() if k != 'cells'})), flush=True)
        entry['latest_complete_decision_round'] = entry['rounds'][-1]['round'] if entry['rounds'] else None
        result['runs'].append(entry)
    result['all_saved_assessments_reproduced'] = all(all(s['checks'].values()) for r in result['runs'] for s in r['rounds'])
    with out.open('x') as dest:
        json.dump(result, dest, indent=2, allow_nan=False)
        dest.write('\n')
    lines = ['# New task exact-support diagnostic snapshot', '', result['created_utc'], '',
        'The frozen per-cell winners and their selections are unchanged. Blocking means the existing observed/perceived operational gate—not simulator ground truth or full executable feasibility. Real ground presence is evaluated separately; no extra votes are introduced.', '',
        '| Run / round | Exact winners | Geometry blocked | Clear, ground missing | Confirmed | Blocked-winner cells with clear non-winner | Clear non-winners (ground supported) |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for run in result['runs']:
        for row in run['rounds']:
            s, n = row['summary'], row['nonwinner_diagnostics']
            lines.append(f"| {run['run']} / {row['round']} | {s['exact_winners']} | {s['geometry_blocked']} | {s['geometry_clear_but_ground_missing']} | {s['confirmed']} | {n['winner_blocked_nonwinner_unblocked_cells']} | {n['unblocked_nonwinner_count']} ({n['unblocked_nonwinners_ground_supported']}) |")
    lines += ['', 'Latest available decision details:', '']
    for run in result['runs']:
        if not run['rounds']:
            continue
        row = run['rounds'][-1]
        lines.append(f"- {run['run']}, round {row['round']}: overlapping blocking causes {row['summary']['overlapping_blocker_counts']}; confirmed sources {row['summary']['confirmed_sources']}. Task-end recorded: {run['task_end_recorded']}.")
        for cell in row['nonwinner_diagnostics']['cells']:
            alternatives = cell['unblocked_nonwinners']
            lines.append(f"  Source {cell['source_id']}: winner blocked by {cell['winner']['audit']['blocker_causes']}; {len(alternatives)} exact-clear alternatives, {sum(a['gate']['ground_supported'] for a in alternatives)} with sufficient real ground support. Alternatives remain diagnostic, unselected, and unexecuted.")
    lines += ['', 'All saved exact anchors and candidate assessments were reproduced. The JSON retains each unchanged winner, exact non-winner pose, blocker class, missing ground-cell ID, and real presence histogram. Counts across rounds are repeated snapshots, not independent candidates or runs.', '']
    with out.with_suffix('.md').open('x') as dest:
        dest.write('\n'.join(lines))
    print(json.dumps(dict(output=str(out), all_saved_assessments_reproduced=result['all_saved_assessments_reproduced'])))


if __name__ == '__main__':
    main()
