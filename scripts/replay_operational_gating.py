#!/usr/bin/env python3
"""Replay saved Hard decisions with no retained RGB-D association reference.

This is an offline regression and geometry diagnostic, not a corrected trial
or a runtime replay.
The original Pilot-1 tree is read-only; new artifacts require a fresh directory.
"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec, PointCloudObservation
from operational_gating import PerceivedTarget, assess_footprint
from operational_gating.io import build_operational_context
from reachability_guided_aerial_perception import GraspTCP
from sim_active_perception.core import A5Config, assess_candidates, candidate_catalog, replay_observations
from sim_active_perception.worker import make_field
from task_relevant_uncertainty import build_task_uncertainty
from task_relevant_uncertainty.geometry import CONTACT_TOLERANCE_M, footprint_cells, footprint_vertices


HARD_SLOTS = (
    'slot-10-hard-generic-attempt-01', 'slot-11-hard-fixed-attempt-01',
    'slot-12-hard-ours-attempt-02', 'slot-13-hard-no-occlusion-attempt-01',
    'slot-14-hard-no-cost-attempt-01',
)
LIMITATION = (
    'Pilot-1 did not retain the corresponding RGB-D segmentation/depth reference. '
    'Association is UNAVAILABLE: no occupied return is assigned TARGET and no '
    'candidate is unlocked by a guessed object label. Continuous object geometry '
    'is a separate diagnostic. These results do not correct or relabel any trial.'
)
RAW_ARRAY_NAMES = ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score')


def load_observation(path):
    with np.load(path, allow_pickle=False) as data:
        return PointCloudObservation(
            data['points_xyz'], str(data['frame_id'].item()), float(data['stamp_s'].item()),
            data['T_map_sensor'], data['valid_return'] if 'valid_return' in data.files else None,
        )


def _load_slot(slot):
    slot = Path(slot).resolve()
    attempt = json.loads((slot / 'attempt.json').read_text())
    if attempt.get('status') != 'VALID_TRIAL' or attempt.get('scene') != 'hard':
        raise ValueError('replay requires a saved VALID_TRIAL in the Hard scene')
    data = slot / 'data'
    initial = json.loads((data / 'initial.json').read_text())
    events = [json.loads(line) for line in (data / 'events.jsonl').read_text().splitlines() if line.strip()]
    handoffs = [event for event in events if event.get('state') == 'AIR_HANDOFF']
    if len(handoffs) != 1:
        raise ValueError('replay requires exactly one runtime AIR_HANDOFF target')
    target_pose = handoffs[0]['target_map']
    if len(target_pose) != 4:
        raise ValueError('AIR_HANDOFF target_map must contain x/y/z/yaw')
    target = PerceivedTarget(tuple(target_pose[:3]), target_pose[3])
    # In-memory context only: old initial.json and all trial outcomes stay intact.
    context = dict(initial, operational_gating='v1.1', perceived_target=asdict(target),
                   target_reference_file=None,
                   target_reference_status='UNAVAILABLE_PILOT1_RGBD_NOT_RETAINED')
    config = A5Config(**initial['config'])
    field = make_field(GraspTCP(**initial['grasp']), initial['result'], config)
    grid = EnvironmentGridSpec(field.grid.origin_xy, field.grid.width_cells,
                               field.grid.height_cells, field.grid.resolution_m)
    return slot, attempt, context, field, grid, BeliefConfig(ground_z_m=config.ground_z_m)


def _compare_saved_belief(path, belief):
    expected = {name: np.asarray(getattr(belief, name)) for name in RAW_ARRAY_NAMES}
    expected.update(frame_id=np.asarray(belief.frame_id), origin_xy=np.asarray(belief.origin_xy),
                    resolution_m=np.asarray(belief.resolution_m), width_cells=np.asarray(belief.width_cells),
                    height_cells=np.asarray(belief.height_cells))
    with np.load(path, allow_pickle=False) as saved:
        if set(saved.files) != set(expected):
            raise ValueError(f'raw A2 stored fields differ: {path}')
        checks = {name: bool(saved[name].dtype == value.dtype and np.array_equal(saved[name], value))
                  for name, value in expected.items()}
    if not all(checks.values()):
        wrong = ', '.join(name for name, matches in checks.items() if not matches)
        raise ValueError(f'raw A2 mismatch ({wrong}): {path}')
    return checks


def _round_observations(slot, directory):
    requests = []
    for path in sorted(directory.glob('*/request.json')):
        value = json.loads(path.read_text())
        if value.get('op') == 'observe':
            requests.append(value)
    if len(requests) != 1:
        raise ValueError(f'require one saved observe request for {directory}')
    paths = []
    for recorded in requests[0]['observations']:
        # Portability: resolve the recorded file basename within this exact slot.
        name = Path(recorded).name
        if not name.startswith('observation_') or not name.endswith('.npz'):
            raise ValueError('saved request observation must name an observation NPZ')
        path = slot / 'data' / name
        paths.append(path)
    if len(paths) != int(directory.name.split('-')[-1]):
        raise ValueError('saved round number and observation history length disagree')
    return paths, [load_observation(path) for path in paths]


def _compact_assessment(assessment):
    values = asdict(assessment)
    values['covered_cell_count'] = len(values.pop('covered_cells'))
    return values


def replay_slot(slot):
    slot = Path(slot).resolve()
    slot, attempt, context, field, grid, config = _load_slot(slot)
    catalog = candidate_catalog(field, context['result'])
    rounds = []
    for directory in sorted((slot / 'data/rounds').glob('round-*')):
        paths, observations = _round_observations(slot, directory)
        belief = replay_observations(grid, observations, config)
        checks = _compare_saved_belief(directory / 'a2/belief.npz', belief)
        operational, metadata = build_operational_context(context, grid, observations, config)
        if metadata['association_status'] != 'UNAVAILABLE' or np.any(operational.target_occupied_votes):
            raise ValueError('saved-reference-free replay must never assign TARGET labels')
        # With no TARGET exception, operational ground is exactly the raw A2 ground count.
        if not np.array_equal(operational.ground_votes, belief.free_evidence):
            raise ValueError('reference-free operational ground differs from raw A2')
        legacy_task = build_task_uncertainty(field, belief)
        derived_task = build_task_uncertainty(field, belief, operational=operational)
        nominal_equal = np.array_equal(legacy_task.nominal_task_relevance,
                                      derived_task.nominal_task_relevance, equal_nan=True)
        if not nominal_equal:
            raise ValueError('nominal task relevance changed')
        legacy = assess_candidates(field, belief, catalog, task=legacy_task)
        derived = assess_candidates(field, belief, catalog, task=derived_task, operational=operational)
        saved_decision = json.loads((directory / 'decision.json').read_text())
        if legacy != saved_decision['assessments']:
            raise ValueError(f'saved v1 assessments differ: {directory}')
        representatives = {pose.source_id: pose for pose in legacy_task.poses}
        candidates = []
        for old, new in zip(legacy, derived):
            representative = representatives[old['source_id']]
            rep_derived = assess_footprint(operational, representative.xy, representative.yaw)
            exact_count = old['free_cells'] + old['occupied_cells'] + old['unknown_cells']
            candidates.append(dict(
                candidate_id=old['candidate_id'], source_id=old['source_id'],
                exact_pose_xyyaw=[old['x'], old['y'], old['yaw']],
                representative_pose_xyyaw=[*representative.xy, representative.yaw],
                v1=dict(exact_blocked=old['occupied_cells'] > 0,
                        exact_ground_supported=bool(exact_count and not old['footprint_clipped']
                                                    and old['free_cells'] == exact_count),
                        exact_occupied_cells=old['occupied_cells'], exact_unknown_cells=old['unknown_cells'],
                        exact_free_cells=old['free_cells'], confirmed=old['confirmed'],
                        representative_blocked=old['representative_blocked'],
                        representative_supported=bool(not representative.blocked
                                                      and not representative.footprint_clipped
                                                      and representative.unknown_cells == 0)),
                v11=dict(exact={key: value for key, value in new['operational'].items()
                                if key != 'covered_cells'},
                         representative=_compact_assessment(rep_derived), confirmed=new['confirmed']),
            ))
        totals = dict(
            catalog=len(candidates),
            v1_exact_blocked=sum(c['v1']['exact_blocked'] for c in candidates),
            v11_exact_blocked=sum(c['v11']['exact']['blocked'] for c in candidates),
            v1_representative_blocked=sum(c['v1']['representative_blocked'] for c in candidates),
            v11_representative_blocked=sum(c['v11']['representative']['blocked'] for c in candidates),
            v1_representative_supported=sum(c['v1']['representative_supported'] for c in candidates),
            v11_representative_supported=sum(not c['v11']['representative']['blocked']
                                            and c['v11']['representative']['ground_supported'] for c in candidates),
            v1_confirmed=sum(c['v1']['confirmed'] for c in candidates),
            v11_confirmed=sum(c['v11']['confirmed'] for c in candidates),
            v11_exact_target_collision=sum(c['v11']['exact']['target_collision'] for c in candidates),
            v11_exact_environment_blocked=sum(c['v11']['exact']['environment_cells'] > 0 for c in candidates),
            v11_exact_ambiguous_blocked=sum(c['v11']['exact']['ambiguous_cells'] > 0 for c in candidates),
            v11_exact_ground_supported=sum(c['v11']['exact']['ground_supported'] for c in candidates),
        )
        rounds.append(dict(
            round=directory.name, observation_files=[str(path.relative_to(slot)) for path in paths],
            observation_stamps_s=[float(o.stamp_s) for o in observations],
            raw_a2_exact_checks=checks, saved_v1_assessments_exact=True,
            nominal_task_relevance_unchanged=bool(nominal_equal), operational_metadata=metadata,
            target_occupied_votes=int(operational.target_occupied_votes.sum()),
            saved_stop_reason=saved_decision.get('stop_reason'), totals=totals, candidates=candidates,
        ))
    if not rounds:
        raise ValueError('saved slot has no observation rounds')
    return dict(slot=slot.name, source_path=str(slot), method=attempt['method'],
                saved_trial_status=attempt['status'], saved_retrieval_success=attempt.get('retrieval_success'),
                association_status='UNAVAILABLE', limitation=LIMITATION, target=context['perceived_target'],
                source_representative_guard_preserved=True, rounds=rounds)


def rectangle_separation_m(first, second):
    """Exact polygon gap for closed convex rectangles; zero for contact/overlap."""
    first, second = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
    if (first.shape != (4, 2) or second.shape != (4, 2)
            or not np.all(np.isfinite(first)) or not np.all(np.isfinite(second))):
        raise ValueError('rectangles must have four finite XY vertices')
    separated = False
    for polygon in (first, second):
        edges = np.roll(polygon, -1, axis=0) - polygon
        norms = np.linalg.norm(edges, axis=1)
        if np.any(norms == 0):
            raise ValueError('rectangle edges must be nonzero')
        axes = np.column_stack((-edges[:, 1], edges[:, 0])) / norms[:, None]
        p, q = first @ axes.T, second @ axes.T
        separated |= bool(np.any(p.max(axis=0) < q.min(axis=0) - CONTACT_TOLERANCE_M)
                          or np.any(q.max(axis=0) < p.min(axis=0) - CONTACT_TOLERANCE_M))
    if not separated:
        return 0.
    distances = []
    for vertices, polygon in ((first, second), (second, first)):
        ends = np.roll(polygon, -1, axis=0)
        for point in vertices:
            for start, end in zip(polygon, ends):
                edge = end - start
                fraction = np.clip(np.dot(point - start, edge) / np.dot(edge, edge), 0., 1.)
                distances.append(np.linalg.norm(point - start - fraction * edge))
    return float(min(distances))


def geometry_example(slot):
    slot, _, context, field, grid, _ = _load_slot(slot)
    catalog = candidate_catalog(field, context['result'])
    candidate = next(c for c in catalog if c['candidate_id'] == 'candidate-000008' and c['source_id'] == 585)
    target = PerceivedTarget(**context['perceived_target'])
    exact = footprint_vertices((candidate['x'], candidate['y']), candidate['yaw'])
    target_vertices = target.xy_vertices
    row, column = divmod(candidate['source_id'], grid.width_cells)
    rep_xy = (grid.origin_xy[0] + (column + .5) * grid.resolution_m,
              grid.origin_xy[1] + (row + .5) * grid.resolution_m)
    exact_cells, _ = footprint_cells(grid, (candidate['x'], candidate['y']), candidate['yaw'])
    representative_cells, _ = footprint_cells(grid, rep_xy, candidate['yaw'])
    cell_row, cell_column = divmod(781, grid.width_cells)
    lower = np.asarray(grid.origin_xy) + np.array([cell_column, cell_row]) * grid.resolution_m
    cell_vertices = lower + grid.resolution_m * np.array([[0, 0], [1, 0], [1, 1], [0, 1]])
    return dict(
        slot=slot.name, candidate_id=candidate['candidate_id'], source_id=candidate['source_id'],
        continuous_separation_m=rectangle_separation_m(exact, target_vertices),
        exact_overlaps_occupied_cell_781=bool(781 in exact_cells),
        representative_overlaps_occupied_cell_781=bool(781 in representative_cells),
        exact_vertices_xy=exact.tolist(), target_vertices_xy=target_vertices.tolist(),
        representative_vertices_xy=footprint_vertices(rep_xy, candidate['yaw']).tolist(),
        cell_781_vertices_xy=cell_vertices.tolist(),
        interpretation='GEOMETRY_REGRESSION_ONLY_NOT_A_CORRECTED_TRIAL',
        association_status='UNAVAILABLE', limitation=LIMITATION,
    )


def _validate_output(pilot_root, output_dir):
    root, output = Path(pilot_root).resolve(), Path(output_dir).resolve()
    if output == root or root in output.parents or output in root.parents:
        raise ValueError('output must be a separate directory outside the Pilot-1 tree')
    if output.exists():
        raise FileExistsError(f'replay output already exists: {output}')
    return output


def write_report(pilot_root, output_dir, report):
    output = _validate_output(pilot_root, output_dir)
    output.mkdir(parents=True, exist_ok=False)
    (output / 'replay.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return output


def render_geometry(example, output_path):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.patches import Polygon

    figure = Figure(figsize=(8, 6))
    FigureCanvasAgg(figure)
    axis = figure.subplots()
    figure.subplots_adjust(left=.11, right=.98, bottom=.17, top=.85)
    for name, color, label, line in (
            ('representative_vertices_xy', '#777777', 'A3 representative footprint', '--'),
            ('exact_vertices_xy', '#3366aa', 'Exact padded footprint', '-'),
            ('target_vertices_xy', '#d39016', 'Perceived known-size target', '-'),
            ('cell_781_vertices_xy', '#bd3b37', 'Occupied cell 781', '-')):
        vertices = np.asarray(example[name])
        axis.add_patch(Polygon(vertices, closed=True, fill=True, alpha=.16, facecolor=color))
        closed = np.vstack((vertices, vertices[0]))
        axis.plot(closed[:, 0], closed[:, 1], color=color, linestyle=line, label=label)
    axis.set_aspect('equal')
    axis.autoscale_view()
    axis.set_xlabel('Map x (m)')
    axis.set_ylabel('Map y (m)')
    axis.legend(loc='lower left', fontsize=9)
    axis.grid(alpha=.2)
    axis.set_title('Hard Ours: candidate 000008 / source 585\n'
                   f'Continuous separation: {example["continuous_separation_m"]:.6f} m')
    figure.text(.5, .035, 'RGB-D reference unavailable: no TARGET labels. Geometry only; trial outcome unchanged.',
                ha='center', fontsize=9)
    figure.savefig(output_path, dpi=180)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pilot-root', type=Path, default=Path('outputs/a6/pilot-20260908'))
    parser.add_argument('--output-dir', type=Path, default=Path('outputs/a6/operational-v11/hard-replay'))
    parser.add_argument('--no-figure', action='store_true')
    args = parser.parse_args(argv)
    _validate_output(args.pilot_root, args.output_dir)
    slots = [replay_slot(args.pilot_root / name) for name in HARD_SLOTS]
    example = geometry_example(args.pilot_root / 'slot-12-hard-ours-attempt-02')
    final_totals = {name: sum(slot['rounds'][-1]['totals'][name] for slot in slots)
                    for name in slots[0]['rounds'][-1]['totals']}
    report = dict(kind='RECORDED_HARD_DIAGNOSTIC_NOT_A_TRIAL', limitation=LIMITATION,
                  association_status='UNAVAILABLE', raw_a2_unchanged=True,
                  pilot1_outcomes_unchanged=True, slot_count=len(slots),
                  replayed_round_count=sum(len(slot['rounds']) for slot in slots),
                  final_round_totals=final_totals, slots=slots, geometry_example=example)
    output = write_report(args.pilot_root, args.output_dir, report)
    if not args.no_figure:
        render_geometry(example, output / 'ours-cell-781-geometry.png')
    print(LIMITATION)
    print(json.dumps(dict(output_dir=str(output), slot_count=len(slots),
                          round_count=report['replayed_round_count'], final_round_totals=final_totals), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
