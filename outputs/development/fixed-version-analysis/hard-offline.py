#!/usr/bin/env python3
"""Bounded offline audit of the four completed fixed-version Hard tasks.

Reuse the existing acquisition/phase and diagnostic-only non-winner evaluators.
No runtime changes, new viewpoint selection, sensing, simulator or GT geometry.
"""

from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / 'outputs/development/finite-scan-analysis'))
from new_task_acquisition import analyze_run, events_from
from new_task_exact_support import plain, classify, summarize_winners
from operational_occlusion_review import load_state
from reachability_guided_aerial_perception import GraspTCP
from replay_exact_support import nonwinner_diagnostics
from sim_active_perception.core import A5Config, assess_candidates, build_support_task, candidate_catalog
from sim_active_perception.worker import make_field

NAMES = ('slot-12-hard02-generic', 'slot-05-hard01-generic',
         'slot-06-hard01-ours', 'slot-11-hard02-ours')


def audit_exact(run, acquisition):
    data = run / 'attempt/data'
    initial = json.loads((data / 'initial.json').read_text())
    config, raw = A5Config(**initial['config']), initial['result']
    field = make_field(GraspTCP(**initial['grasp']), raw, config)
    states, audits = {}, []
    for n in (1, 2, 3):
        state = states[n] = load_state(data / 'rounds' / f'round-{n:02d}')
        belief, context = state['belief'], state['operational']
        original = context.ground_presence_votes.copy()
        task = build_support_task(field, raw, belief, config, operational=context)
        assessments = assess_candidates(field, belief, candidate_catalog(field, raw), task=task, operational=context)
        checks = dict(exact_anchors=plain([asdict(a) for a in task.winner_anchors]) == state['ranking']['a3_summary']['winner_anchors'],
                      exact_assessments=plain(assessments) == state['decision']['assessments'])
        assert all(checks.values()), (run.name, n, checks)
        rows = [dict(a, audit=classify(a, context, task.footprint)) for a in plain(assessments)]
        checks['presence_unchanged'] = bool(np.array_equal(original, context.ground_presence_votes))
        assert checks['presence_unchanged']
        audits.append(dict(round=n, checks=checks, summary=summarize_winners(rows), assessments=rows))
    final, context = states[3], states[3]['operational']
    alternatives = nonwinner_diagnostics(field, raw, context, task.footprint)
    for group in alternatives['cells']:
        group['winner']['audit'] = classify(group['winner'], context, task.footprint, gate_key='gate')
        for a in group['unblocked_nonwinners']:
            a['audit'] = classify(a, context, task.footprint, gate_key='gate')
    alts = [a for g in alternatives['cells'] for a in g['unblocked_nonwinners']]
    alternatives['clear_nonwinners_with_real_ground_support'] = sum(a['gate']['ground_supported'] for a in alts)
    alternatives['clear_nonwinners_missing_real_ground'] = sum(a['gate']['ground_missing_cells'] > 0 for a in alts)
    missing = sorted(set(i for a in audits[-1]['assessments'] if not a['audit']['geometry_blocked']
                         for i in a['audit']['missing_ground_cell_ids']))
    cells = []
    for i in missing:
        row = dict(cell_id=i, real_presence_by_window=[int(states[n]['evidence']['ground_presence_votes'].ravel()[i]) for n in (1, 2, 3)],
            raw_a2_state_by_window=[int(states[n]['belief'].state.ravel()[i]) for n in (1, 2, 3)],
            final_class_votes={k: int(final['evidence'][k].ravel()[i]) for k in
                ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes')},
            affected_clear_sources=[a['source_id'] for a in audits[-1]['assessments']
                if not a['audit']['geometry_blocked'] and i in a['audit']['missing_ground_cell_ids']],
            selected_forecasts=[])
        for pair in acquisition['prediction_pairs']:
            n, index = pair['observed_window'], pair['candidate']['candidate_id']
            row['selected_forecasts'].append(dict(window=n,
                nominal_opportunity=float(states[n - 1]['fields']['observation_opportunity'][index].ravel()[i]),
                unoccluded_opportunity=float(states[n - 1]['fields']['unoccluded_opportunity'][index].ravel()[i])))
        cells.append(row)
    return dict(rounds=audits, final_nonwinner_diagnostic=alternatives,
                final_clear_missing_cells=cells, saved_final_selection=final['decision']['selected_candidate'],
                saved_final_stop_reason=final['decision']['stop_reason'])


def main():
    outputs = [HERE / f'hard-{name}.json' for name in NAMES]
    if any(p.exists() for p in outputs):
        raise FileExistsError('Preserve original snapshots; use new explicit output filenames.')
    cache = {}
    for name, output in zip(NAMES, outputs):
        run = ROOT / 'outputs/development/fixed-version' / name
        events = events_from(run / 'attempt/data/events.jsonl')
        assert events[-1]['state'] == 'A6_TASK_END', 'completed slots only'
        start = perf_counter()
        acquisition = analyze_run(run, True, cache, HERE / 'hard')
        exact = audit_exact(run, acquisition)
        result = dict(generated_at_utc=datetime.now(timezone.utc).isoformat(), run=name,
            diagnostic_only=True, no_gt_geometry_used=True, no_sim_contact=True,
            selection_changed=False, no_generated_observations=True,
            acquisition=acquisition, exact=exact, elapsed_s=perf_counter() - start)
        with output.open('x') as dest:
            json.dump(result, dest, indent=2, allow_nan=False)
            dest.write('\n')
        print(json.dumps(dict(output=str(output), elapsed_s=result['elapsed_s'],
            windows=len(acquisition['windows']), pairs=len(acquisition['prediction_pairs']),
            final=exact['rounds'][-1]['summary'], nonwinners={k: v for k, v in exact['final_nonwinner_diagnostic'].items() if k != 'cells'},
            union_comparisons=[dict(window=p['observed_window'],
                stats=p['scopes']['after_viable_union'], phase=p['phase_audit']['scopes']['after_viable_union'])
                for p in acquisition['prediction_pairs']])), flush=True)


if __name__ == '__main__':
    main()
