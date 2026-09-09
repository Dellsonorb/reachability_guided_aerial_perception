#!/usr/bin/env python3
"""Read-only replay of one flat-layout v1.2 natural attempt; JSON to stdout."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec, PointCloudObservation
from operational_gating.io import build_operational_context
from reachability_guided_aerial_perception import GraspTCP
from reachability_guided_nbv import Viewpoint
from replay_exact_support import nonwinner_diagnostics
from sim_active_perception.core import A5Config, build_support_task, decide, replay_observations
from sim_active_perception.worker import make_field
from task_relevant_uncertainty.core import FIELD_ARRAY_NAMES
from task_relevant_uncertainty.outputs import field_summary


A2_NAMES = ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score')
OP_NAMES = ('environment_occupied_votes', 'ambiguous_occupied_votes',
            'target_occupied_votes', 'ground_votes')


def plain(value):
    return json.loads(json.dumps(value, allow_nan=False))


def require_checks(checks, context):
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise ValueError(f'{context}: mismatched ' + ', '.join(failed))


def replay(attempt_dir):
    directory = Path(attempt_dir).resolve()
    initial = json.loads((directory / 'initial.json').read_text())
    if initial.get('target_reference_file'):
        initial = dict(initial, target_reference_file=str(directory / 'target_reference.npz'))
    config = A5Config(**initial['config'])
    raw = initial['result']
    field = make_field(GraspTCP(**initial['grasp']), raw, config)
    grid = EnvironmentGridSpec(field.grid.origin_xy, field.grid.width_cells, field.grid.height_cells)
    reports = []
    for window in sorted(directory.glob('round-*')):
        request = json.loads((window / 'request.json').read_text())
        response = json.loads((window / 'response.json').read_text())
        observations = []
        for old_path in request['observations']:
            with np.load(directory / Path(old_path).name, allow_pickle=False) as saved:
                # Match worker.observe exactly: these windows have no valid_return input.
                observations.append(PointCloudObservation(
                    saved['points_xyz'], str(saved['frame_id'].item()),
                    float(saved['stamp_s'].item()), saved['T_map_sensor']))
        belief = replay_observations(grid, observations, BeliefConfig(ground_z_m=config.ground_z_m))
        operational, _ = build_operational_context(initial, grid, observations, belief.config)
        if operational is None:
            raise ValueError('natural v1.2 replay requires recorded operational evidence')
        pose = request['uav_pose']
        choice, ranking = decide(
            field, raw, belief, Viewpoint(tuple(pose[:3]), pose[3]),
            round_count=len(observations), config=config, operational=operational)
        task = build_support_task(field, raw, belief, config, operational=operational)
        diagnostic = nonwinner_diagnostics(field, raw, operational)
        assessments = choice['assessments']
        alpha = -np.expm1(-1 / belief.config.unknown_scale)
        errors = [abs(candidate.task_gain - alpha * np.nansum(
            task.task_relevant_uncertainty * ranking.visibility[candidate.candidate_id]))
            for candidate in ranking.candidates]
        checks = dict(
            decision_equals_saved_response=plain(choice) == response,
            a4_delta_formula=bool(np.array_equal(ranking.delta_unknown, alpha * belief.unknown_score)),
            a4_marginal_formula=bool(np.array_equal(
                ranking.marginal_task_gain, alpha * task.task_relevant_uncertainty, equal_nan=True)),
            a4_task_gain_formula=all(error <= 1e-10 for error in errors),
            a4_task_score_formula=all(
                candidate.task_score == candidate.task_gain - config.flight_weight * candidate.flight_cost
                for candidate in ranking.candidates if candidate.status == 'VALID'),
            a4_generic_gain_formula=all(candidate.generic_gain == float(np.sum(
                alpha * belief.unknown_score * ranking.visibility[candidate.candidate_id]))
                for candidate in ranking.candidates),
            a4_generic_score_formula=all(
                candidate.generic_score == candidate.generic_gain - config.flight_weight * candidate.flight_cost
                for candidate in ranking.candidates if candidate.status == 'VALID'))
        require_checks(checks, window.name)
        reports.append(dict(
            round=len(observations), checks=checks,
            confirmed=choice['confirmed_candidate_count'], candidate_count=choice['candidate_count'],
            blocking=sum(row['operational']['blocked'] for row in assessments),
            blocked_source_ids=[row['source_id'] for row in assessments if row['operational']['blocked']],
            unblocked_source_ids=[row['source_id'] for row in assessments if not row['operational']['blocked']],
            blocking_reasons={name: sum(bool(row['operational'][name]) for row in assessments)
                              for name in ('footprint_clipped', 'environment_cells',
                                           'ambiguous_cells', 'target_collision')},
            support_poses_blocked=sum(pose.blocked for pose in task.poses),
            positive_M_cells=int(np.count_nonzero(task.task_relevance_at_environment_cell > 0)),
            uncertainty_mass=float(np.nansum(task.task_relevant_uncertainty)),
            confirmed_ids=[dict(source_id=row['source_id'], candidate_id=row['candidate_id'])
                           for row in assessments if row['confirmed']],
            selected=choice['selected_candidate'],
            source_543=next((row for row in assessments if row['source_id'] == 543), None),
            nonwinner_diagnostics=diagnostic,
            environment_cells=choice['environment_cells'],
            total_observation_votes=choice['total_observation_votes'],
            operational_votes={name: int(getattr(operational, name).sum()) for name in OP_NAMES},
            a4_candidate_count=len(ranking.candidates), a4_status=ranking.status,
            best_task_id=None if ranking.best_task is None else ranking.best_task.candidate_id,
            best_generic_id=None if ranking.best_generic is None else ranking.best_generic.candidate_id,
            best_task_gain=choice['best_task_gain'], best_task_score=choice['best_task_score'],
            a4_task_gain_formula_max_abs_error=max(errors, default=0.0),
            stop_reason=choice['stop_reason'], next_viewpoint=choice['next_viewpoint']))
    if not reports:
        raise ValueError('no saved round snapshots to replay')

    checks = {}
    with np.load(directory / 'a2/belief.npz', allow_pickle=False) as saved:
        checks.update({'a2_' + name: bool(np.array_equal(
            saved[name], getattr(belief, name), equal_nan=True)) for name in A2_NAMES})
    with np.load(directory / 'operational_evidence.npz', allow_pickle=False) as saved:
        checks.update({'operational_' + name: bool(np.array_equal(
            saved[name], getattr(operational, name), equal_nan=True)) for name in OP_NAMES})
    with np.load(directory / 'fields.npz', allow_pickle=False) as saved:
        checks.update({'a3_' + name: bool(np.array_equal(
            saved['a3_' + name], getattr(task, name), equal_nan=True)) for name in FIELD_ARRAY_NAMES})
        checks.update({'fields_a2_' + name: bool(np.array_equal(
            saved['a2_' + name], getattr(belief, name), equal_nan=True)) for name in A2_NAMES})
        for name in ('visibility', 'delta_unknown', 'marginal_task_gain'):
            checks['a4_' + name] = bool(np.array_equal(
                saved[name], getattr(ranking, name), equal_nan=True))
        checks['a4_task_contributions'] = bool(np.array_equal(
            saved['task_contributions'], np.stack([
                ranking.task_contribution(i) for i in range(len(ranking.candidates))]), equal_nan=True))
    saved_ranking = json.loads((directory / 'ranking.json').read_text())
    comparable = dict(
        status=ranking.status, current=asdict(ranking.current), config=asdict(ranking.config),
        sensor=dict(T_uav_lidar=ranking.sensor.T_uav_lidar.tolist(),
                    min_elevation_deg=ranking.sensor.min_elevation_deg,
                    max_elevation_deg=ranking.sensor.max_elevation_deg,
                    horizontal_fov_deg=360, yaw_equivalent=ranking.sensor.yaw_equivalent),
        a2_config=asdict(belief.config), a3_summary=field_summary(task),
        a3_poses=[asdict(pose) for pose in task.poses], task_order=ranking.task_order,
        generic_order=ranking.generic_order,
        best_task_id=None if ranking.best_task is None else ranking.best_task.candidate_id,
        best_generic_id=None if ranking.best_generic is None else ranking.best_generic.candidate_id,
        candidates=[asdict(candidate) for candidate in ranking.candidates])
    checks.update({'ranking_' + name: plain(value) == saved_ranking[name]
                   for name, value in comparable.items()})
    checks['final_decision_json'] = plain(choice) == json.loads((directory / 'decision.json').read_text())
    require_checks(checks, 'final artifacts')
    return dict(
        kind='EXACT_ANCHOR_V12_NATURAL_REPLAY', source_attempt=str(directory),
        diagnostic_only=True, selection_changed=False, original_outcome_unchanged=True,
        support_anchor=config.support_anchor, rounds=reports,
        final_artifact_checks=checks, final_artifact_checks_count=len(checks))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    args = parser.parse_args()
    print(json.dumps(replay(args.run_directory), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
