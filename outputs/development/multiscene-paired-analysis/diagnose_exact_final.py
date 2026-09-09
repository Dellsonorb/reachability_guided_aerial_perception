#!/usr/bin/env python3
"""Reuse frozen functions on final recorded states; alternatives are diagnostic only."""
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np
from environment_belief import BeliefConfig, EnvironmentGridSpec, PointCloudObservation
from operational_gating.io import build_operational_context
from reachability_guided_aerial_perception import GraspTCP
from replay_exact_support import nonwinner_diagnostics
from sim_active_perception.core import (A5Config, assess_candidates, build_support_task,
                                        candidate_catalog, replay_observations)
from sim_active_perception.worker import make_field

HERE = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
plain = lambda value: json.loads(json.dumps(value, allow_nan=False))


def main():
    reports = []
    for attempt in sorted((HERE.parent / 'multiscene-paired').glob('launch-*')):
        data, final = attempt / 'data', attempt / 'data/rounds/round-03'
        initial, ranking, decision = (read(p) for p in
                                     (data / 'initial.json', final / 'ranking.json', final / 'decision.json'))
        config = A5Config(**initial['config'])
        field = make_field(GraspTCP(**initial['grasp']), initial['result'], config)
        grid = EnvironmentGridSpec(**ranking['a3_summary']['grid'])
        observations = []
        for number in (1, 2, 3):
            with np.load(data / f'observation_{number:02d}.npz', allow_pickle=False) as saved:
                observations.append(PointCloudObservation(saved['points_xyz'], str(saved['frame_id'].item()),
                    float(saved['stamp_s']), saved['T_map_sensor'],
                    saved['valid_return'] if 'valid_return' in saved.files else None))
        belief = replay_observations(grid, observations, BeliefConfig(**ranking['a2_config']))
        operational, _ = build_operational_context(initial, grid, observations, belief.config)
        task = build_support_task(field, initial['result'], belief, config, operational=operational)
        assessments = assess_candidates(field, belief, candidate_catalog(field, initial['result']),
                                        task=task, operational=operational)
        checks = dict(exact_anchors=plain([asdict(a) for a in task.winner_anchors]) ==
                      ranking['a3_summary']['winner_anchors'],
                      exact_assessments=plain(assessments) == decision['assessments'])
        with np.load(final / 'fields.npz', allow_pickle=False) as saved:
            checks['raw_a2'] = all(np.array_equal(saved['a2_' + key], getattr(belief, key)) for key in
                ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score'))
        with np.load(final / 'operational_evidence.npz', allow_pickle=False) as saved:
            checks['operational_votes'] = all(np.array_equal(saved[key], getattr(operational, key)) for key in
                ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes',
                 'ground_votes', 'ground_presence_votes'))
        assert all(checks.values()), (attempt.name, checks)
        alternatives = nonwinner_diagnostics(field, initial['result'], operational, task.footprint)
        reports.append(dict(attempt=attempt.name, checks=checks, nonwinner_diagnostics=alternatives))
    result = dict(diagnostic_only=True, selection_changed=False, all_checks_pass=True, reports=reports)
    (HERE / 'exact-final.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(final_states=len(reports), all_checks_pass=True,
                         nonwinner_selection_implemented=False)))


if __name__ == '__main__':
    main()
