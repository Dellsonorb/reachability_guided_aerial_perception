#!/usr/bin/env python3
"""Read-only consistency diagnostics for frozen exact-support snapshots."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from environment_belief import BeliefConfig, EnvironmentBeliefGrid, EnvironmentGridSpec
from operational_gating import OperationalEvidenceView, PerceivedTarget
from reachability_guided_aerial_perception import GraspTCP
from replay_exact_support import nonwinner_diagnostics
from sim_active_perception.core import A5Config, assess_candidates, build_support_task, candidate_catalog
from sim_active_perception.worker import make_field
from task_relevant_uncertainty.core import FIELD_ARRAY_NAMES


_BELIEF_NAMES = ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score')
_OPERATIONAL_NAMES = ('environment_occupied_votes', 'ambiguous_occupied_votes',
                      'target_occupied_votes', 'ground_votes')


def _plain(value):
    """Normalize tuples/dataclasses to the representation used by JSON snapshots."""
    return json.loads(json.dumps(value, allow_nan=False))


def check_snapshot(initial, ranking, decision, arrays, operational_summary, operational_arrays):
    """Rebuild and compare one frozen snapshot without replaying observations."""
    checks = {
        'config_exact_winner': initial.get('config', {}).get('support_anchor') == 'exact_winner',
        'initial_operational_semantics': initial.get('operational_gating') == 'v1.1',
        'decision_anchor_semantics': decision.get('anchor_semantics') == 'exact-validated-winner-v1.2',
        'ranking_anchor_semantics': ranking.get('a3_summary', {}).get('anchor_semantics')
                                    == 'exact-validated-winner-v1.2',
    }
    anchors, poses, assessments, diagnostics = [], [], [], None
    try:
        config = A5Config(**initial['config'])
        raw = initial['result']
        field = make_field(GraspTCP(**initial['grasp']), raw, config)
        grid = EnvironmentGridSpec(**ranking['a3_summary']['grid'])
        belief_config = BeliefConfig(**ranking['a2_config'])
        belief = EnvironmentBeliefGrid(grid, belief_config,
                                       **{name: arrays['a2_' + name] for name in _BELIEF_NAMES})
        operational = OperationalEvidenceView(
            grid, belief_config, PerceivedTarget(**operational_summary['target']),
            **{name: operational_arrays[name] for name in _OPERATIONAL_NAMES})
        task = build_support_task(field, raw, belief, config, operational=operational)
        checks['task_operational_semantics'] = task.operational_semantics == 'object-aware-v1.1'
        checks['decision_operational_semantics'] = (
            decision.get('operational_semantics') == task.operational_semantics)
        checks['ranking_operational_semantics'] = (
            ranking['a3_summary'].get('operational_semantics') == task.operational_semantics)
        checks['operational_summary_semantics'] = (
            operational_summary.get('operational_semantics') == task.operational_semantics)
        catalog = candidate_catalog(field, raw)
        assessments = assess_candidates(field, belief, catalog, task=task, operational=operational)
        anchors = [asdict(anchor) for anchor in task.winner_anchors]
        poses = [asdict(pose) for pose in task.poses]
        checks['winner_anchors'] = _plain(anchors) == ranking['a3_summary'].get('winner_anchors')
        checks['support_poses'] = _plain(poses) == ranking.get('a3_poses')
        checks['assessments'] = _plain(assessments) == decision.get('assessments')
        checks['representative_blocked_is_operational'] = all(
            row['representative_blocked'] == row['operational']['blocked'] for row in assessments)
        for name in FIELD_ARRAY_NAMES:
            checks['a3_' + name] = bool(np.array_equal(
                arrays['a3_' + name], getattr(task, name), equal_nan=True))
        diagnostics = nonwinner_diagnostics(field, raw, operational, task.footprint)
    except (KeyError, TypeError, ValueError) as error:
        checks['reconstruction'] = False
        reconstruction_error = str(error)
    else:
        checks['reconstruction'] = True
        reconstruction_error = None
    return dict(diagnostic_only=True, selection_changed=False, checks=checks,
                all_checks_pass=all(checks.values()), reconstruction_error=reconstruction_error,
                winner_anchors=_plain(anchors), support_poses=_plain(poses),
                assessments=_plain(assessments), nonwinner_diagnostics=diagnostics)


def _load_snapshot(directory):
    data = directory.parents[1]
    initial = json.loads((data / 'initial.json').read_text())
    ranking = json.loads((directory / 'ranking.json').read_text())
    decision = json.loads((directory / 'decision.json').read_text())
    operational_summary = json.loads((directory / 'operational_summary.json').read_text())
    with np.load(directory / 'fields.npz', allow_pickle=False) as saved:
        arrays = {name: saved[name].copy() for name in saved.files}
    with np.load(directory / 'operational_evidence.npz', allow_pickle=False) as saved:
        operational_arrays = {name: saved[name].copy() for name in saved.files}
    return check_snapshot(initial, ranking, decision, arrays, operational_summary, operational_arrays)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    directories = sorted(path for path in args.results_dir.glob('*/data/rounds/round-*') if path.is_dir())
    if not directories:
        parser.error('no saved round snapshots found')
    output = args.output.resolve()
    for attempt in {directory.parents[2].resolve() for directory in directories}:
        if output == attempt or attempt in output.parents:
            parser.error('output must be outside every source attempt')
    reports = []
    for directory in directories:
        try:
            report = _load_snapshot(directory)
        except (OSError, KeyError, TypeError, ValueError) as error:
            report = dict(diagnostic_only=True, selection_changed=False, checks={'snapshot_load': False},
                          all_checks_pass=False, reconstruction_error=str(error))
        reports.append(dict(snapshot=str(directory), **report))
    result = dict(diagnostic_only=True, selection_changed=False, snapshot_count=len(reports),
                  all_checks_pass=all(report['all_checks_pass'] for report in reports), snapshots=reports)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    if not result['all_checks_pass']:
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
