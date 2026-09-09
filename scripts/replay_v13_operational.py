#!/usr/bin/env python3
"""Development-only v1.2/v1.3 comparison on original runtime observations.

No new trial, simulated sensor update, alternative candidate selection or GT.
Uses the existing A2/A3/A4/A5 functions and writes only new derived outputs.
"""

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

import numpy as np

from a6_pilot.policy import decide_policy
from environment_belief import BeliefConfig, EnvironmentGridSpec, PointCloudObservation
from operational_gating.io import build_operational_context, save_operational
from reachability_guided_aerial_perception import GraspTCP
from reachability_guided_nbv import Viewpoint
from reachability_guided_nbv.outputs import save_result, render_result
from replay_exact_support import nonwinner_diagnostics
from sim_active_perception.core import A5Config, build_support_task, replay_observations
from sim_active_perception.worker import make_field


def replay_round(data, number, output=None, *, include_v14=False):
    data = Path(data).resolve()
    if output is not None:
        output = Path(output).resolve()
        if (output.exists() or output == data or data in output.parents
                or (data.name == 'data' and data.parent in output.parents)):
            raise ValueError('output must be new and outside the recorded source attempt')
    initial = json.loads((data / 'initial.json').read_text())
    if initial.get('target_reference_file'):
        # Use this attempt's bundled runtime observation after relocation;
        # never rewrite the original record or fabricate a missing reference.
        initial = dict(initial, target_reference_file=str(data / 'target_reference.npz'))
    config = replace(A5Config(**initial['config']), support_anchor='exact_winner')
    field = make_field(GraspTCP(**initial['grasp']), initial['result'], config)
    grid = EnvironmentGridSpec(field.grid.origin_xy, field.grid.width_cells, field.grid.height_cells)
    observations = []
    for index in range(1, number + 1):
        with np.load(data / ('observation_%02d.npz' % index), allow_pickle=False) as saved:
            observations.append(PointCloudObservation(saved['points_xyz'], str(saved['frame_id'].item()),
                                                       float(saved['stamp_s']), saved['T_map_sensor']))
    saved_ranking = data / ('rounds/round-%02d/ranking.json' % number)
    if saved_ranking.exists():
        current = Viewpoint(**json.loads(saved_ranking.read_text())['current'])
    else:
        requests = list(data.glob('round-%02d-*/request.json' % number))
        if len(requests) != 1:
            raise ValueError('require one original stamped current-pose request per round')
        pose = json.loads(requests[0].read_text())['uav_pose']
        current = Viewpoint(tuple(pose[:3]), pose[3])
    belief = replay_observations(grid, observations, BeliefConfig(ground_z_m=config.ground_z_m))
    result = dict(development_replay_not_trial=True, source_data=str(data), round=number,
                  current=asdict(current), historical_outcome_unchanged=True)
    contexts, tasks = [], []
    checks = {}
    versions = [('v12', 'v1.1'), ('v13', 'v1.3')]
    if include_v14:
        versions.append(('v14', 'v1.4'))
    for version, revision in versions:
        operational, metadata = build_operational_context(dict(initial, operational_gating=revision),
                                                           grid, observations, belief.config)
        task = build_support_task(field, initial['result'], belief, config, operational=operational)
        choice, ranking = decide_policy(field, initial['result'], belief, current, method='ours',
                                         round_count=number, config=config, operational=operational)
        generic, generic_ranking = decide_policy(field, initial['result'], belief, current, method='generic',
                                                 round_count=number, config=config, operational=operational)
        contexts.append(operational)
        tasks.append(task)
        rows = choice['assessments']
        poses = {p.source_id: p for p in task.poses}
        checks[version + '_shared_exact_gate'] = all(
            p['operational']['blocked'] == poses[p['source_id']].blocked for p in rows)
        checks[version + '_same_state_fairness'] = (
            [asdict(c) for c in ranking.candidates] == [asdict(c) for c in generic_ranking.candidates]
            and np.array_equal(ranking.visibility, generic_ranking.visibility)
            and choice['assessments'] == generic['assessments'])
        factor = 1 - np.exp(-1 / belief.config.unknown_scale)
        checks[version + '_gain_formula'] = all(np.isclose(
            c.task_gain, factor * np.nansum(ranking.visibility[c.candidate_id]
                                            * task.task_relevant_uncertainty), rtol=1e-12, atol=1e-12)
            for c in ranking.candidates)
        result[version] = dict(
            operational_semantics=task.operational_semantics,
            blocked=sum(r['operational']['blocked'] for r in rows),
            confirmed=sum(r['confirmed'] for r in rows),
            target_collision=sum(r['operational']['target_collision'] for r in rows),
            raw_grid_blocked=sum(r['occupied_cells'] > 0 for r in rows),
            task_uncertainty_mass=float(np.nansum(task.task_relevant_uncertainty)),
            ours_stop_reason=choice['stop_reason'], ours_next_viewpoint=choice['next_viewpoint'],
            generic_stop_reason=generic['stop_reason'], generic_next_viewpoint=generic['next_viewpoint'],
            best_task_gain=None if ranking.best_task is None else ranking.best_task.task_gain,
            best_task_score=None if ranking.best_task is None else ranking.best_task.task_score,
            assessments=rows,
            nonwinner_diagnostics=nonwinner_diagnostics(field, initial['result'], operational, task.footprint),
            association_votes=metadata)
        if output is not None and version == versions[-1][0]:
            directory = Path(output)
            save_operational(directory, operational, metadata)
            save_result(ranking, task, belief, directory)
            render_result(ranking, task, belief, directory / 'nbv.png',
                          title=revision + ' development replay: recorded observations, no new flight')
    for name in ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes', 'ground_votes'):
        checks['unchanged_' + name] = bool(np.array_equal(getattr(contexts[0], name), getattr(contexts[1], name)))
    checks['unchanged_nominal'] = bool(np.array_equal(tasks[0].nominal_task_relevance,
                                                     tasks[1].nominal_task_relevance, equal_nan=True))
    checks['same_exact_winners'] = tasks[0].winner_anchors == tasks[1].winner_anchors
    if include_v14:
        for name in ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes', 'ground_votes'):
            checks['v14_unchanged_' + name] = bool(np.array_equal(
                getattr(contexts[1], name), getattr(contexts[2], name)))
        checks['v14_unchanged_task_uncertainty'] = bool(np.array_equal(
            tasks[1].task_relevant_uncertainty, tasks[2].task_relevant_uncertainty, equal_nan=True))
        checks['v14_same_exact_winners'] = tasks[1].winner_anchors == tasks[2].winner_anchors
    result.update(checks=checks, all_common_checks_pass=all(checks.values()))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--include-v14', action='store_true', help='also compare measured ground-presence semantics')
    args = parser.parse_args()
    data, output = args.data_dir.resolve(), args.output_dir.resolve()
    if output == data or data in output.parents or output.exists():
        parser.error('output must be new and outside the recorded data directory')
    # Do not place a development report among the original A6 attempt files.
    if data.name == 'data' and data.parent in output.parents:
        parser.error('output must be outside the recorded attempt')
    output.mkdir(parents=True)
    rounds = len(list(data.glob('observation_*.npz')))
    if not rounds:
        parser.error('no original observations')
    reports = [replay_round(data, n, output / ('round-%02d' % n), include_v14=args.include_v14)
               for n in range(1, rounds + 1)]
    (output / 'replay.json').write_text(json.dumps(dict(development_replay_not_trial=True, rounds=reports),
                                                 indent=2, allow_nan=False) + '\n')
    return 0 if all(r['all_common_checks_pass'] for r in reports) else 1


if __name__ == '__main__':
    raise SystemExit(main())
