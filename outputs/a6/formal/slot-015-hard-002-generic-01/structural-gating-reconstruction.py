"""One saved-activation diagnostic; never launch ROS, query RM4D, or edit inputs.

From the repository root:
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/a6-review-mpl \
XDG_CACHE_HOME=/tmp/a6-review-cache PYTHONPATH=src:scripts \
/media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python \
outputs/a6/formal/slot-015-hard-002-generic-01/structural-gating-reconstruction.py
"""

from dataclasses import asdict
import json
from pathlib import Path

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Polygon, Rectangle

from a6_scoring_diagnostics import check_snapshot
from environment_belief import BeliefConfig, EnvironmentGridSpec, PointCloudObservation
from operational_gating import PerceivedTarget, assess_footprint
from operational_gating.association import _interior_mask, associate_returns, metric_allowance
from operational_gating.io import _load_reference, build_operational_context
from reachability_guided_aerial_perception import GraspTCP
from sim_active_perception.core import A5Config, assess_candidates, candidate_catalog, replay_observations
from sim_active_perception.worker import make_field
from task_relevant_uncertainty import build_task_uncertainty
from task_relevant_uncertainty.core import FIELD_ARRAY_NAMES
from task_relevant_uncertainty.geometry import footprint_vertices


HERE = Path(__file__).resolve().parent
DATA = HERE / 'data'
read = lambda path: json.loads(path.read_text())
plain = lambda value: json.loads(json.dumps(value))
initial = read(DATA / 'initial.json')
config = A5Config(**initial['config'])
field = make_field(GraspTCP(**initial['grasp']), initial['result'], config)
catalog = candidate_catalog(field, initial['result'])
grid = EnvironmentGridSpec(field.grid.origin_xy, field.grid.width_cells, field.grid.height_cells)
belief_config = BeliefConfig(**read(DATA / 'rounds/round-01/ranking.json')['a2_config'])
reference = _load_reference(initial['target_reference_file'], PerceivedTarget(**initial['perceived_target']))
observations, rounds, cell_provenance = [], [], []

for number in range(1, 4):
    directory = DATA / f'rounds/round-{number:02d}'
    decision, ranking = read(directory / 'decision.json'), read(directory / 'ranking.json')
    with np.load(DATA / f'observation_{number:02d}.npz', allow_pickle=False) as saved:
        observations.append(PointCloudObservation(saved['points_xyz'], str(saved['frame_id'].item()),
                                                  float(saved['stamp_s']), saved['T_map_sensor']))
    belief = replay_observations(grid, observations, belief_config)
    operational, metadata = build_operational_context(initial, grid, observations, belief_config)
    with np.load(directory / 'a2/belief.npz', allow_pickle=False) as saved:
        for name in ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score'):
            assert np.array_equal(saved[name], getattr(belief, name)), (number, name)
    with np.load(directory / 'operational_evidence.npz', allow_pickle=False) as saved:
        for name in ('environment_occupied_votes', 'ambiguous_occupied_votes', 'target_occupied_votes', 'ground_votes'):
            assert np.array_equal(saved[name], getattr(operational, name)), (number, name)
        assert np.array_equal(saved['target_vertices_xy'], operational.target.xy_vertices)
    assert plain(metadata['target']) == read(directory / 'operational_summary.json')['target']
    task = build_task_uncertainty(field, belief, operational=operational)
    assessments = plain(assess_candidates(field, belief, catalog, task=task, operational=operational))
    assert assessments == decision['assessments']
    assert plain([asdict(pose) for pose in task.poses]) == ranking['a3_poses']
    with np.load(directory / 'fields.npz', allow_pickle=False) as saved:
        for name in FIELD_ARRAY_NAMES:
            assert np.array_equal(getattr(task, name), saved['a3_' + name], equal_nan=True), (number, name)
        scoring = check_snapshot(ranking, saved)
        best_id = decision['policy_best_candidate_id']
        assert decision['policy_visibility']['cell_ids'] == np.flatnonzero(saved['visibility'][best_id]).tolist()
        assert decision['policy_visibility']['shape'] == list(grid.shape)
    assert scoring['gain_and_cost_identity_pass']
    assert decision['policy_method'] == 'generic'
    assert decision['policy_candidates'] == ranking['candidates']
    assert decision['policy_order'] == ranking['generic_order']
    assert best_id == ranking['generic_order'][0]
    best = ranking['candidates'][best_id]
    assert decision['policy_best_gain'] == best['generic_gain']
    assert decision['policy_best_score'] == best['generic_score']
    assert best['generic_gain'] > 0 and best['generic_score'] > 0
    assert decision['stop_reason'] == ('VIEW_BUDGET_REACHED' if number == 3 else None)
    assert decision['next_viewpoint'] == (None if number == 3 else
                                         [*best['viewpoint']['position_xyz'], best['viewpoint']['yaw_rad']])
    assert decision['confirmed_candidate_count'] == 0 and decision['selected_candidate'] is None
    poses = {pose.source_id: pose for pose in task.poses}
    representatives = [assess_footprint(operational, pose.xy, pose.yaw) for pose in task.poses]
    exact = next(row for row in assessments if row['source_id'] == 624)
    representative = poses[624]
    rep_gate = assess_footprint(operational, representative.xy, representative.yaw)
    noncolliding = []
    for row in assessments:
        if not row['operational']['target_collision']:
            pose = poses[row['source_id']]
            gate = assess_footprint(operational, pose.xy, pose.yaw)
            noncolliding.append(dict(candidate_id=row['candidate_id'], source_id=row['source_id'],
                                    exact=row['operational'], representative=asdict(gate)))
    rounds.append(dict(round=number, observation_points=len(observations[-1].points_xyz),
                       observation_stamp_s=observations[-1].stamp_s,
                       raw_a2_replay_pass=True, operational_replay_pass=True,
                       exact_assessment_count=len(assessments), exact_assessments_match=True,
                       a3_pose_assessments_match=True, a3_arrays_match_count=len(FIELD_ARRAY_NAMES),
                       scoring_identity_pass=True, scoring_max_abs_error=scoring['maximum_abs_identity_error'],
                       generic_policy_action_match=True, stop_reason=decision['stop_reason'],
                       generic_best_id=best_id, generic_best_gain=best['generic_gain'],
                       generic_best_score=best['generic_score'], next_viewpoint=decision['next_viewpoint'],
                       shared_ranking_status=ranking['status'],
                       counts=dict(exact_blocked=sum(row['operational']['blocked'] for row in assessments),
                                   exact_target_collision=sum(row['operational']['target_collision'] for row in assessments),
                                   exact_environment_blocked=sum(row['operational']['environment_cells'] > 0 for row in assessments),
                                   exact_ambiguous_blocked=sum(row['operational']['ambiguous_cells'] > 0 for row in assessments),
                                   representative_blocked=sum(gate.blocked for gate in representatives),
                                   representative_target_collision=sum(gate.target_collision for gate in representatives),
                                   representative_environment_blocked=sum(gate.environment_cells > 0 for gate in representatives),
                                   representative_ambiguous_blocked=sum(gate.ambiguous_cells > 0 for gate in representatives),
                                   confirmed=0),
                       nominal_positive_cells=int(np.count_nonzero(task.nominal_task_relevance > 0)),
                       operational_positive_cells=int(np.count_nonzero(task.task_relevance_at_environment_cell > 0)),
                       task_uncertainty_mass=float(np.nansum(task.task_relevant_uncertainty)),
                       source624_exact=exact, source624_representative_gate=asdict(rep_gate),
                       noncolliding_exact_candidates=noncolliding))
    observation = observations[-1]
    points = observation.points_xyz @ observation.T_map_sensor[:3, :3].T + observation.T_map_sensor[:3, 3]
    labels = associate_returns(points, operational.target, reference)
    cols = np.floor((points[:, 0] - grid.origin_xy[0]) / grid.resolution_m)
    rows = np.floor((points[:, 1] - grid.origin_xy[1]) / grid.resolution_m)
    ranges = np.linalg.norm(observation.points_xyz, axis=1)
    usable = ((ranges > belief_config.min_range_m) & (ranges < belief_config.max_range_m)
              & (points[:, 2] - belief_config.ground_z_m >= belief_config.obstacle_min_height_m))
    ids = np.flatnonzero(usable & (rows == 19) & (cols == 21))
    provenance = dict(round=number, cell_id=781, occupied_endpoint_count=len(ids),
                      accumulated_ambiguous_votes=int(operational.ambiguous_occupied_votes.ravel()[781]),
                      points=[])
    for index in ids:
        point = points[index]
        optical = (point - reference['T_map_color'][:3, 3]) @ reference['T_map_color'][:3, :3]
        k = reference['color_K']
        u, v = np.rint(optical[:2] / optical[2] * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]).astype(int)
        depth = float(reference['registered_depth_m'][v, u])
        provenance['points'].append(dict(map_xyz=point.tolist(), original_occupied_class=int(labels[index]),
            inside_expanded_perceived_box=bool(operational.target.contains(point[None])[0]),
            pixel_uv=[int(u), int(v)], red_mask=bool(reference['mask'][v, u]),
            eroded_interior_mask=bool(_interior_mask(reference['mask'])[v, u]),
            reference_depth_m=depth if np.isfinite(depth) else None,
            return_optical_depth_m=float(optical[2]),
            depth_abs_error_m=abs(depth - float(optical[2])) if np.isfinite(depth) else None,
            allowed_depth_error_m=float(metric_allowance(reference, optical[2]))))
    cell_provenance.append(provenance)

exact_vertices = footprint_vertices((exact['x'], exact['y']), exact['yaw'])
rep_vertices = footprint_vertices(representative.xy, representative.yaw)
target_vertices = operational.target.xy_vertices
y_gap = float(target_vertices[:, 1].min() - exact_vertices[:, 1].max())
y_overlap = float(rep_vertices[:, 1].max() - target_vertices[:, 1].min())
assert y_gap > 0 and not exact['operational']['target_collision'] and rep_gate.target_collision
report = dict(kind='SECONDARY_DIAGNOSTIC_ONLY', activation=HERE.name,
    scope='Saved runtime-perceived geometry/NPZ only; no GT, new sensing, RM4D queries or outcome rewrites.',
    reproduction_command=__doc__.split('From the repository root:\n')[1].strip(),
    source_script=Path(__file__).name,
    checks=dict(rounds=3, exact_assessments=102, representative_assessments=102,
                a3_arrays=3 * len(FIELD_ARRAY_NAMES), raw_a2_arrays=15, operational_vote_arrays=12,
                scoring_snapshots=3, actual_generic_actions=3, all_pass=True),
    perceived_target=asdict(operational.target), reference_stamp_s=reference['stamp_s'],
    allowance_recomputed_from_frozen_sensor_formula=True,
    source624_geometry=dict(candidate_id=exact['candidate_id'], source_id=624,
        row=15, col=24, exact_xy=[exact['x'], exact['y']], exact_yaw=exact['yaw'],
        representative_xy=list(representative.xy), representative_yaw=representative.yaw,
        center_displacement_xy_m=(np.array(representative.xy)-[exact['x'], exact['y']]).tolist(),
        exact_vertices_xy=exact_vertices.tolist(), representative_vertices_xy=rep_vertices.tolist(),
        expanded_target_vertices_xy=target_vertices.tolist(),
        exact_separating_map_y_gap_m=y_gap, representative_map_y_projection_overlap_m=y_overlap,
        exact_continuous_SAT_collision=False, representative_continuous_SAT_collision=True,
        representative_ambiguous_cell_ids=[779, 780, 781],
        representative_target_cell_ids=[779, 780], representative_environment_cell_ids=[],
        final_exact_ground_supported_cells=87, final_exact_covered_cells=88,
        final_exact_ground_missing_cells=1, final_exact_confirmed=False),
    shared_ambiguous_cell781=dict(row=19, col=21,
        bounds_xy=[grid.origin_xy[0]+2.1, grid.origin_xy[0]+2.2,
                   grid.origin_xy[1]+1.9, grid.origin_xy[1]+2.0], windows=cell_provenance,
        note='Inside the expanded geometry does not establish TARGET identity. The three returns retain their original AMBIGUOUS labels.'),
    rounds=rounds,
    assessment=dict(scientific_stop=True, scope='This fixed initial catalog and accumulated evidence, not every scene.',
        mechanism='All representatives already blocked after window1. Source624 is exact-clear but its fixed cell-center representative intersects the expanded target and ambiguous cells. Other exact candidates have continuous target collisions or accumulated ambiguous blockers. No further permitted ground sensing can unblock these representatives: geometry/reference/catalog are fixed and occupied class votes only accumulate. M_operational and U_task remain zero.',
        qualification='The remaining source624 ground cell is genuinely unconfirmed. This diagnostic does not assert physical navigation/grasp feasibility, assign GT labels, invent ground votes, or relabel the valid recorded VIEW_BUDGET_REACHED outcome.',
        policy_boundary='Actual Generic selections match shared generic ranking. The saved shared Ours ranking has NO_PREDICTED_TASK_GAIN in all three snapshots; no Ours trial was launched to demonstrate the lock.'))
(HERE / 'structural-gating-diagnostic.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')

figure = Figure(figsize=(11, 5), layout='constrained')
FigureCanvasAgg(figure)
axes = figure.subplots(1, 2)
for axis in axes:
    for cell in (779, 780, 781):
        row, col = divmod(cell, 40)
        corner = (grid.origin_xy[0] + col * .1, grid.origin_xy[1] + row * .1)
        axis.add_patch(Rectangle(corner, .1, .1, facecolor='#d3b1dd', edgecolor='#9a6aa8', alpha=.45,
                                 label='AMBIGUOUS cell (TARGET may coexist)' if cell == 779 else None))
    for vertices, color, style, label in (
            (target_vertices, '#c94444', '-', 'Perceived target + frozen allowance'),
            (exact_vertices, '#087f8c', '-', 'Exact source624 padded footprint'),
            (rep_vertices, '#d37a13', '--', 'Cell-center representative footprint')):
        axis.add_patch(Polygon(vertices, fill=False, edgecolor=color, linestyle=style, linewidth=2, label=label))
    pts = np.array([row['map_xyz'] for row in cell_provenance[0]['points']])
    axis.scatter(pts[:, 0], pts[:, 1], color='#662d91', s=22, zorder=5, label='Cell781 AMBIGUOUS returns')
    axis.set(aspect='equal', xlabel='map x (m)', ylabel='map y (m)')
    axis.grid(alpha=.2)
axes[0].set(xlim=(1.7, 3.0), ylim=(-1.0, .15), title='Fixed catalog geometry')
axes[0].legend(fontsize=7, loc='lower left')
axes[1].set(xlim=(1.79, 2.16), ylim=(-.195, .05), title='Boundary zoom: exact-clear, representative-blocked')
gap_x = target_vertices[np.argmin(target_vertices[:, 1]), 0]
axes[1].annotate('', xy=(gap_x, target_vertices[:, 1].min()), xytext=(gap_x, exact_vertices[:, 1].max()),
                 arrowprops=dict(arrowstyle='<->', color='#087f8c'))
axes[1].text(1.805, -.184, f'Exact y gap: {1000*y_gap:.3f} mm', color='#087f8c', fontsize=8)
axes[1].text(1.805, .026, f'Representative y overlap: {1000*y_overlap:.3f} mm', color='#a9650e', fontsize=8)
figure.suptitle('SECONDARY DIAGNOSTIC ONLY | slot15 Hard-002 Generic\nRuntime-perceived geometry; not GT or a physical feasibility certificate', fontsize=11)
figure.savefig(HERE / 'structural-gating-geometry.png', dpi=150)
figure.clear()
print(json.dumps(report['checks']))
