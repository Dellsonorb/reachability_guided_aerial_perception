#!/usr/bin/env python3
"""Saved-state v1.2 regression, not corrected trials or multi-support policy."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec, PointCloudObservation
from operational_gating import assess_footprint
from operational_gating.io import build_operational_context
from reachability_guided_aerial_perception import GraspTCP, candidate_relevance
from reachability_guided_nbv import NBVConfig, SensorModel, Viewpoint, rank_viewpoints
from reachability_guided_nbv.outputs import save_result
from sim_active_perception.core import A5Config, assess_candidates, candidate_catalog, replay_observations
from sim_active_perception.worker import make_field
from task_relevant_uncertainty import build_task_uncertainty
from task_relevant_uncertainty.core import FIELD_ARRAY_NAMES
from task_relevant_uncertainty.geometry import FootprintSpec


def nonwinner_diagnostics(field, raw, operational, footprint=FootprintSpec()):
    """Count exact-clear non-winners without selecting them or inventing votes."""
    winners = candidate_catalog(field, raw)
    groups = {}
    for index, candidate in enumerate(raw['evaluated_candidates']):
        relevance = candidate_relevance(candidate, field.config)
        cell = field.grid.cell_index(candidate['bunker_x'], candidate['bunker_y'])
        if cell is None or relevance <= 0:
            continue
        source = cell[0] * field.grid.width_cells + cell[1]
        gate = assess_footprint(operational, (candidate['bunker_x'], candidate['bunker_y']),
                                candidate['bunker_yaw'], footprint)
        groups.setdefault(source, []).append(dict(
            candidate_id=candidate['candidate_id'], evaluation_index=index,
            x=candidate['bunker_x'], y=candidate['bunker_y'], yaw=candidate['bunker_yaw'],
            relevance=relevance, gate=asdict(gate)))
    cells = []
    for winner in winners:
        candidates = groups[winner['source_id']]
        chosen = next(row for row in candidates if row['evaluation_index'] == winner['evaluation_index'])
        alternatives = [row for row in candidates
                        if row['evaluation_index'] != winner['evaluation_index'] and not row['gate']['blocked']]
        if chosen['gate']['blocked'] and alternatives:
            cells.append(dict(source_id=winner['source_id'], winner=chosen, unblocked_nonwinners=alternatives))
    return dict(diagnostic_only=True, selection_changed=False,
                winner_blocked_nonwinner_unblocked_cells=len(cells),
                unblocked_nonwinner_count=sum(len(row['unblocked_nonwinners']) for row in cells),
                cells=cells, interpretation='geometric_gate_only_not_confirmed_or_executable')


def _plain(value):
    return json.loads(json.dumps(value, allow_nan=False))


def _checks_pass(checks, context):
    failed = [key for key, matches in checks.items() if not matches]
    if failed:
        raise ValueError(f'{context}: mismatched ' + ', '.join(failed))


def replay_attempt(attempt_dir, output_dir=None):
    """Compare anchors on saved states; never simulate new sensing or execution."""
    attempt_dir = Path(attempt_dir).resolve()
    data = attempt_dir / 'data'
    initial = json.loads((data / 'initial.json').read_text())
    if initial.get('operational_gating') != 'v1.1':
        raise ValueError('recorded replay requires original v1.1 operational evidence')
    if initial.get('target_reference_file'):
        # Recorded paths name the old runtime checkout; use this attempt's
        # bundled observation. Never rewrite its initial.json or invent a reference.
        initial = dict(initial, target_reference_file=str(data / 'target_reference.npz'))
    config = A5Config(**initial['config'])
    field = make_field(GraspTCP(**initial['grasp']), initial['result'], config)
    raw = initial['result']
    grid = EnvironmentGridSpec(field.grid.origin_xy, field.grid.width_cells, field.grid.height_cells)
    catalog = candidate_catalog(field, raw)
    if output_dir is not None:
        output_dir = Path(output_dir).resolve()
        if output_dir == attempt_dir or attempt_dir in output_dir.parents:
            raise ValueError('derived output must be outside the original attempt')
        output_dir.mkdir(parents=True, exist_ok=False)
    observations, reports = [], []
    for directory in sorted((data / 'rounds').glob('round-*')):
        number = int(directory.name.split('-')[-1])
        ranking = json.loads((directory / 'ranking.json').read_text())
        decision = json.loads((directory / 'decision.json').read_text())
        with np.load(data / f'observation_{number:02d}.npz', allow_pickle=False) as saved:
            observations.append(PointCloudObservation(
                saved['points_xyz'], str(saved['frame_id'].item()), float(saved['stamp_s']),
                saved['T_map_sensor'], saved['valid_return'] if 'valid_return' in saved.files else None))
        belief = replay_observations(grid, observations, BeliefConfig(**ranking['a2_config']))
        operational, _ = build_operational_context(initial, grid, observations, belief.config)
        checks = {}
        with np.load(directory / 'a2/belief.npz', allow_pickle=False) as saved:
            for name in ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score'):
                checks['raw_a2_' + name] = bool(np.array_equal(saved[name], getattr(belief, name)))
        with np.load(directory / 'operational_evidence.npz', allow_pickle=False) as saved:
            for name in ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes', 'ground_votes'):
                checks['operational_' + name] = bool(np.array_equal(saved[name], getattr(operational, name)))
        old = build_task_uncertainty(field, belief, operational=operational)
        new = build_task_uncertainty(field, belief, operational=operational,
                                     evaluated_candidates=raw['evaluated_candidates'])
        before = assess_candidates(field, belief, catalog, task=old, operational=operational)
        after = assess_candidates(field, belief, catalog, task=new, operational=operational)
        checks['assessments'] = _plain(before) == decision['assessments']
        checks['a3_poses'] = _plain([asdict(p) for p in old.poses]) == ranking['a3_poses']
        viewpoints = tuple(Viewpoint(**row['viewpoint']) for row in ranking['candidates'])
        sensor = SensorModel(**{key: ranking['sensor'][key] for key in
                                 ('T_uav_lidar', 'min_elevation_deg', 'max_elevation_deg')})
        kwargs = dict(candidates=viewpoints, sensor=sensor, config=NBVConfig(**ranking['config']))
        current = Viewpoint(**ranking['current'])
        old_ranking = rank_viewpoints(old, belief, current, **kwargs)
        new_ranking = rank_viewpoints(new, belief, current, **kwargs)
        with np.load(directory / 'fields.npz', allow_pickle=False) as saved:
            for name in FIELD_ARRAY_NAMES:
                checks['a3_' + name] = bool(np.array_equal(saved['a3_' + name], getattr(old, name), equal_nan=True))
            checks['a4_visibility'] = bool(np.array_equal(saved['visibility'], old_ranking.visibility))
            checks['a4_delta'] = bool(np.array_equal(saved['delta_unknown'], old_ranking.delta_unknown))
        checks['a4_candidates'] = _plain([asdict(c) for c in old_ranking.candidates]) == ranking['candidates']
        _checks_pass(checks, f'{attempt_dir.name}/{directory.name} legacy replay')
        alpha = -np.expm1(-1 / belief.config.unknown_scale)
        inputs = dict(
            viewpoints_unchanged=tuple(c.viewpoint for c in old_ranking.candidates) == tuple(c.viewpoint for c in new_ranking.candidates),
            visibility_unchanged=bool(np.array_equal(old_ranking.visibility, new_ranking.visibility)),
            delta_unknown_unchanged=bool(np.array_equal(old_ranking.delta_unknown, new_ranking.delta_unknown)),
            flight_cost_unchanged=all(a.flight_cost == b.flight_cost for a, b in zip(old_ranking.candidates, new_ranking.candidates)),
            generic_gains_unchanged=all(a.generic_gain == b.generic_gain for a, b in zip(old_ranking.candidates, new_ranking.candidates)),
            generic_order_unchanged=old_ranking.generic_order == new_ranking.generic_order,
            task_marginal_formula=bool(np.array_equal(new_ranking.marginal_task_gain,
                                                       alpha * new.task_relevant_uncertainty, equal_nan=True)),
            task_gain_formula=all(np.isclose(row.task_gain, alpha * np.nansum(
                new.task_relevant_uncertainty * new_ranking.visibility[row.candidate_id]), rtol=0, atol=1e-10)
                                 for row in new_ranking.candidates),
            exact_operational_gates_unchanged=all(a['operational'] == b['operational'] for a, b in zip(before, after)),
            original_relevance_unchanged=all(a.relevance == float(field.relevance[a.row, a.col]) for a in new.poses),
        )
        _checks_pass(inputs, f'{attempt_dir.name}/{directory.name} A4 input')
        reports.append(dict(
            round=number, legacy_replay_checks=checks, a4_input_checks=inputs,
            v11_operational_cells=int(np.count_nonzero(old.task_relevance_at_environment_cell > 0)),
            v12_operational_cells=int(np.count_nonzero(new.task_relevance_at_environment_cell > 0)),
            v11_uncertainty_mass=float(np.nansum(old.task_relevant_uncertainty)),
            v12_uncertainty_mass=float(np.nansum(new.task_relevant_uncertainty)),
            v11_support_blocked=sum(p.blocked for p in old.poses),
            v12_support_blocked=sum(p.blocked for p in new.poses),
            v11_confirmed=sum(c['confirmed'] for c in before), v12_confirmed=sum(c['confirmed'] for c in after),
            v11_a4_status=old_ranking.status, v12_a4_status=new_ranking.status,
            v11_best_task_id=None if old_ranking.best_task is None else old_ranking.best_task.candidate_id,
            v12_best_task_id=None if new_ranking.best_task is None else new_ranking.best_task.candidate_id,
            v12_best_task_gain=None if new_ranking.best_task is None else new_ranking.best_task.task_gain,
            target=asdict(operational.target),
            candidates=[dict(source_id=a['source_id'], candidate_id=a['candidate_id'], v11=a, v12=b)
                        for a, b in zip(before, after)],
            nonwinner_diagnostics=nonwinner_diagnostics(field, raw, operational)))
        if output_dir is not None:
            save_result(new_ranking, new, belief, output_dir / directory.name)
            _render_comparison(old, new, output_dir / directory.name / 'support-comparison.png')
    if not reports:
        raise ValueError('no saved round snapshots to replay')
    return dict(kind='EXACT_ANCHOR_V12_RECORDED_REGRESSION', diagnostic_only=True,
                source_attempt=str(attempt_dir), original_outcome_unchanged=True,
                note='Same recorded states, not a v1.2 closed-loop trial or final statistical sample.', rounds=reports)


def _render_comparison(old, new, output_path):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    figure = Figure(figsize=(11, 9), layout='constrained')
    FigureCanvasAgg(figure)
    for axes, task, version in zip(figure.subplots(2, 2), (old, new), ('v1.1 cell center', 'v1.2 exact winner')):
        for axis, values, name in zip(axes,
                (task.task_relevance_at_environment_cell, task.task_relevant_uncertainty), ('M_operational', 'U_task')):
            picture = axis.imshow(values, origin='lower', extent=task.grid.extent, interpolation='none',
                                  vmin=0, vmax=1, cmap='viridis')
            axis.set(title=f'{version}: {name}', xlabel='map x (m)', ylabel='map y (m)')
            figure.colorbar(picture, ax=axis, shrink=.75)
    figure.suptitle('Recorded-state regression only | identical A1, A2 and operational evidence\n'
                   'No new ground votes, confirmation or physical outcome is invented')
    figure.savefig(output_path, dpi=140)
    figure.clear()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--attempt', type=Path, action='append', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    root = args.output_dir.resolve()
    for attempt in args.attempt:
        source = attempt.resolve()
        if root == source or source in root.parents:
            parser.error('output must be outside every original attempt')
    root.mkdir(parents=True, exist_ok=False)
    reports = [replay_attempt(attempt, root / attempt.name) for attempt in args.attempt]
    (root / 'replay.json').write_text(json.dumps(dict(diagnostic_only=True, attempts=reports),
                                               indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(attempts=len(reports), rounds=sum(len(r['rounds']) for r in reports),
                          original_outcomes_unchanged=True)))


if __name__ == '__main__':
    main()
