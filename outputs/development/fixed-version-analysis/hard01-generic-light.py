#!/usr/bin/env python3
"""Lightweight, completed-slot-only checks; no phase/model/ranking replay.

Reuses saved-state loading, endpoint binning, and mask comparison helpers.
Run with nice and one BLAS thread while the parent completes the serial batch.
"""

from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / 'outputs/development/finite-scan-analysis'))
from new_task_acquisition import events_from, compare, viable, union
from operational_occlusion_review import load_state
from finite_scan_probe import old
from prism_scan_probe import pose_stats

RUN = ROOT / 'outputs/development/fixed-version/slot-05-hard01-generic'
DATA = RUN / 'attempt/data'
OUT = HERE / 'hard01-generic-light.json'


def main():
    if OUT.exists():
        raise FileExistsError('Preserve the prior report; choose a new explicit output in the script.')
    start = perf_counter()
    events = events_from(DATA / 'events.jsonl')
    assert events[-1]['state'] == 'A6_TASK_END' and events[-1]['adapter_run_result']
    observed = {e['round']: e for e in events if e['state'] == 'A5_OBSERVATION'}
    assert sorted(observed) == [1, 2, 3]
    task = json.loads((RUN / 'task.json').read_text())
    states = {n: load_state(DATA / 'rounds' / f'round-{n:02d}') for n in observed}
    result = dict(generated_at_utc=datetime.now(timezone.utc).isoformat(), run=RUN.name,
        runtime_versions=task['runtime_versions'], completed_slot_only=True,
        no_sim_contact=True, no_gt_geometry_used=True, no_generated_observations=True,
        no_model_or_phase_replay=True, selection_changed=False, windows=[], prediction_pairs=[])
    for n, event in observed.items():
        after = states[n]
        belief, evidence = after['belief'], after['evidence']
        config, grid = belief.config, belief.grid
        observation = old.load_npz(DATA / f'observation_{n:02d}.npz')
        points = observation['points_xyz']
        matrix = observation['T_map_sensor']
        mapped = points @ matrix[:3, :3].T + matrix[:3, 3]
        ranges = np.linalg.norm(points, axis=1)
        counts = old.counts(old.cell_ids(mapped, grid), belief.state.size,
            (ranges > config.min_range_m) & (ranges < config.max_range_m) &
            (np.abs(mapped[:, 2] - config.ground_z_m) <= config.ground_tolerance_m))
        actual = counts > 0
        presence = evidence['ground_presence_votes'].ravel()
        previous = np.zeros_like(presence) if n == 1 else states[n - 1]['evidence']['ground_presence_votes'].ravel()
        assert np.array_equal(presence - previous, actual.astype(presence.dtype))
        requested = event['requested_viewpoint']
        sensor = old.SensorModel(**{k: after['ranking']['sensor'][k] for k in
            ('T_uav_lidar', 'min_elevation_deg', 'max_elevation_deg')})
        result['windows'].append(dict(window=n, ground_presence_increment_reproduced=True,
            actual_ground_cells=int(actual.sum()), retained_ground_points=int(counts.sum()),
            chunk_count=len(observation['chunk_point_counts']),
            actual_span_s=float(observation['chunk_stamps_s'][-1] - observation['chunk_stamps_s'][0]),
            confirmed=after['decision']['confirmed_candidate_count'],
            pose_drift=pose_stats(observation, old.Viewpoint(requested[:3], requested[3]), sensor)))
        if n == 1:
            continue
        before = states[n - 1]
        candidates = [c for c in before['ranking']['candidates'] if np.allclose(
            [*c['viewpoint']['position_xyz'], c['viewpoint']['yaw_rad']], requested, rtol=0, atol=1e-10)]
        assert len(candidates) == 1
        index = candidates[0]['candidate_id']
        p = before['fields']['observation_opportunity'][index].ravel()
        u = before['fields']['unoccluded_opportunity'][index].ravel()
        assert np.array_equal(p > 0, before['fields']['visibility'][index].ravel())
        scopes = dict(all_grid=np.arange(belief.state.size),
            before_viable_union=union(viable(before['decision']['assessments'])),
            after_viable_union=union(viable(after['decision']['assessments'])),
            after_confirmed_union=union([a for a in after['decision']['assessments'] if a['confirmed']]))
        result['prediction_pairs'].append(dict(observed_window=n, candidate=candidates[0],
            requested_viewpoint=requested, saved_opportunity_mask_consistent=True,
            scopes={name: compare(p, actual, ids) for name, ids in scopes.items()},
            saved_unoccluded_scopes={name: compare(u, actual, ids) for name, ids in scopes.items()},
            viable_false_negatives_removed_by_saved_occlusion=[int(i) for i in scopes['after_viable_union']
                if actual[i] and p[i] == 0 and u[i] > 0]))
    final = states[3]
    rows = final['decision']['assessments']
    causes = Counter()
    clear = []
    for a in rows:
        op, amb = a['operational'], a['ambiguous_subcell']
        for name, flag in [('perceived_target_rectangle', op['target_collision']),
            ('observed_environment_cells', op['environment_cells'] > 0),
            ('ambiguous_endpoint_disks', bool(amb['intersecting_endpoint_indices'])),
            ('ambiguous_history_fallback', bool(amb['legacy_fallback_cell_ids']))]:
            causes[name] += int(flag)
        if not op['blocked'] and not a['representative_blocked'] and not a['footprint_clipped']:
            ids = np.asarray(op['covered_cells'], dtype=int)
            values = final['evidence']['ground_presence_votes'].ravel()[ids]
            clear.append(dict(source_id=a['source_id'], candidate_id=a['candidate_id'], confirmed=a['confirmed'],
                cells=len(ids), missing_cells=ids[values < final['belief'].config.free_observations].tolist(),
                real_presence_histogram={str(int(v)): int(np.count_nonzero(values == v)) for v in np.unique(values)}))
    result['final_exact_winners'] = dict(total=len(rows), blocked=sum(a['operational']['blocked'] for a in rows),
        clear=len(clear), confirmed=sum(a['confirmed'] for a in rows), overlapping_blocker_counts=dict(causes),
        clear_candidates=clear, saved_selected=final['decision']['selected_candidate'])
    result['deferred'] = ['nominal vs actual packet-phase replay', 'full finite-model equality replay',
                          'existing diagnostic-only nonwinner evaluator']
    result['elapsed_s'] = perf_counter() - start
    with OUT.open('x') as dest:
        json.dump(result, dest, indent=2, allow_nan=False)
        dest.write('\n')
    print(json.dumps(dict(output=str(OUT), elapsed_s=result['elapsed_s'], windows=len(result['windows']),
        pairs=len(result['prediction_pairs']), final={k: v for k, v in result['final_exact_winners'].items()
            if k not in ('clear_candidates', 'saved_selected')})))


if __name__ == '__main__':
    main()
