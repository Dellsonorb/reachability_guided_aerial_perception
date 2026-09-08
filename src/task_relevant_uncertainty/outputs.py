"""Small NPZ/JSON products and raw-grid local Agg visualizations for A3."""

from dataclasses import asdict, fields
import json
from pathlib import Path

from matplotlib import colormaps
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.figure import Figure
from matplotlib.patches import Polygon
import numpy as np

from reachability_guided_aerial_perception.model import CellState
from .core import FIELD_ARRAY_NAMES, PoseEnvironmentState, SupportState, TaskRelevantUncertaintyField
from .geometry import CONTACT_TOLERANCE_M, footprint_vertices


def field_summary(field):
    if not isinstance(field, TaskRelevantUncertaintyField):
        raise ValueError('field must be TaskRelevantUncertaintyField')
    coverage = {f.name: getattr(field.a1_coverage, f.name) for f in fields(field.a1_coverage)}
    coverage['rejected_by_reason'] = dict(coverage['rejected_by_reason'])
    values = field.task_relevant_uncertainty[np.isfinite(field.task_relevant_uncertainty)]
    object_aware = field.operational_semantics == 'object-aware-v1.1'
    summary = {
        'schema_version': 1, 'field_type': 'task_relevant_observation_deficit',
        'frame_id': field.frame_id, 'grasp_id': field.grasp_id,
        'grid': {'origin_xy': list(field.grid.origin_xy), 'frame_id': field.frame_id,
                 'resolution_m': float(field.grid.resolution_m),
                 'width_cells': int(field.grid.width_cells), 'height_cells': int(field.grid.height_cells)},
        'a1_status': field.a1_status.value, 'a1_coverage': coverage,
        'footprint': asdict(field.footprint), 'cell_overlap': 'closed_rectangle_intersection_including_contact',
        'contact_tolerance_m': CONTACT_TOLERANCE_M,
        'cells': {s.name.lower(): int(np.count_nonzero(field.support_state == s)) for s in SupportState},
        'pose_states': {s.name.lower(): int(np.count_nonzero(field.pose_environment_state == s))
                        for s in PoseEnvironmentState if object_aware or s != PoseEnvironmentState.OPERATIONAL_BLOCKED},
        'projected_poses': len(field.poses), 'clipped_poses': sum(p.footprint_clipped for p in field.poses),
        'uncertainty_max': float(values.max()) if len(values) else None,
        'uncertainty_mean_over_nominal_support': float(values.mean()) if len(values) else None,
        'nominal_formula': 'max R(q) over all validated-support representative footprints containing x',
        'operational_formula': 'max R(q) over non-A2-occupied-blocked supporting footprints containing x',
        'uncertainty_formula': 'unknown_score * task_relevance_at_environment_cell',
        'blocked_semantics': 'A2-occupied-blocked_not_navigation_infeasible',
        'representative_pose': 'A1_cell_center_plus_stored_best_yaw',
        'representative_pose_ik_validated': False, 'free_forces_zero_score': False,
        'unknown_blocks_pose': False, 'score_semantics': 'heuristic_not_probability_or_information_gain',
        'no_validated_support_values': 'NaN_not_zero_not_global_task_irrelevance',
        'blocked_only_values': 'nominal_preserved_operational_and_uncertainty_zero',
        'observed_ground_support_semantics': 'A2_ground_support_not_full_clearance',
        'smoothing': False,
    }
    if object_aware:
        summary.update(
            operational_semantics=field.operational_semantics,
            operational_formula='max R(q) over object-aware-unblocked supporting footprints containing x',
            blocked_semantics='environment_or_ambiguous_or_target_geometry_not_navigation_infeasible',
            observed_ground_support_semantics='derived_actual_ground_votes_not_raw_A2_FREE_not_full_clearance')
    if field.anchor_semantics == 'exact-validated-winner-v1.2':
        summary.update(
            anchor_semantics=field.anchor_semantics,
            winner_anchors=[asdict(anchor) for anchor in field.winner_anchors],
            anchor_tie_rule='first_original_evaluation_index_among_equal_candidate_relevance',
            representative_pose='original_frozen_evaluated_candidate_per_A1_HIGH_LOW_cell',
            representative_pose_ik_validated=True,
            representative_pose_validation='original_A1_IK_validity_gates_not_new_IK_or_navigation_clearance')
    return summary


def save_task_field(field, output_dir):
    """Save a builder output; overwrite only the three named files."""
    summary = field_summary(field)
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    paths = {name: directory / filename for name, filename in (
        ('field', 'field.npz'), ('summary', 'summary.json'), ('supports', 'supports.json'))}
    np.savez_compressed(paths['field'], **{name: getattr(field, name) for name in FIELD_ARRAY_NAMES},
                        frame_id=field.frame_id, origin_xy=field.grid.origin_xy,
                        resolution_m=field.grid.resolution_m,
                        width_cells=field.grid.width_cells, height_cells=field.grid.height_cells)
    poses = []
    for pose in field.poses:
        record = asdict(pose)
        record['environment_state'] = pose.environment_state.name
        poses.append(record)
    sources = {'source_id_convention': 'base row * width_cells + col; -1 means no winner',
               'covered_environment_cells': 'row-major flat environment cell ids',
               'tie_rule': 'first row-major base source', 'poses': poses}
    for name, data in (('summary', summary), ('supports', sources)):
        paths[name].write_text(json.dumps(data, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return paths


def render_task_field(field, output_path, title='A3 footprint-aware task uncertainty'):
    """Eight panels; NaN support is gray and score color scales are fixed [0,1]."""
    field_summary(field)
    path = Path(output_path)
    if path.suffix.lower() != '.png':
        raise ValueError('output_path must have a .png suffix')
    figure = Figure(figsize=(18, 8.5), layout='constrained')
    FigureCanvasAgg(figure)
    axes = figure.subplots(2, 4).ravel()
    raster = dict(origin='lower', interpolation='none', aspect='equal', extent=field.grid.extent)
    cmap = colormaps['viridis'].with_extremes(bad='#dedede')
    gate = 'object-aware' if field.operational_semantics == 'object-aware-v1.1' else 'A2-occupied'

    def score(axis, values, label):
        im = axis.imshow(values, cmap=cmap, vmin=0, vmax=1, **raster)
        axis.set_title(label, fontsize=10)
        figure.colorbar(im, ax=axis, shrink=0.75)

    def category(axis, values, labels, colors, label):
        count = len(labels)
        im = axis.imshow(values, cmap=ListedColormap(colors),
                         norm=BoundaryNorm(np.arange(-1.5, count - 0.5), count), **raster)
        axis.set_title(label, fontsize=10)
        bar = figure.colorbar(im, ax=axis, ticks=np.arange(-1, count - 1), shrink=0.75)
        bar.ax.set_yticklabels(labels, fontsize=7)

    try:
        # Reconstruct displayed A1 scalar cells from preserved states and sources.
        a1 = np.full(field.grid.shape, np.nan)
        a1[field.a1_cell_state == CellState.INFEASIBLE] = 0
        for pose in field.poses:
            a1[pose.row, pose.col] = pose.relevance
        score(axes[0], a1, 'A1 R + representative footprints')
        environment = field.environment_state.copy()
        environment[environment == 100] = 1
        category(axes[1], environment, ['UNKNOWN', 'FREE', 'OCCUPIED'],
                 ['#bdbdbd', '#35a872', '#d34b3f'], 'A2 endpoint state')
        score(axes[2], field.unknown_score, 'A2 unknown_score (heuristic)')
        score(axes[3], field.nominal_task_relevance, 'M_nominal: pure manipulation support')
        score(axes[4], field.task_relevance_at_environment_cell, f'M_operational: {gate} gate')
        score(axes[5], field.task_relevant_uncertainty, 'U_task = unknown_score * M_operational')
        category(axes[6], field.support_state, ['NO VALIDATED\nSUPPORT', 'BLOCKED ONLY', 'SUPPORTED'],
                 ['#dedede', '#d34b3f', '#35a872'], 'Environment-cell support state')
        source_image = np.full(field.grid.shape, -1, dtype=int)
        labels, colors = ['NONE'], ['#dedede']
        for i, pose in enumerate(field.poses):
            source_image[field.best_operational_source == pose.source_id] = i
            labels.append(str(pose.source_id))
            colors.append(colormaps['tab10'](i % 10))
        category(axes[7], source_image, labels, colors, 'Best operational source (base cell id)')
        for pose in field.poses:
            corners = footprint_vertices(pose.xy, pose.yaw, field.footprint)
            for axis in (axes[0], axes[1]):
                color = '#d62728' if pose.blocked else '#0066cc'
                axis.add_patch(Polygon(corners, closed=True, fill=False, edgecolor=color,
                                       linestyle='--' if pose.blocked else '-', linewidth=1.5))
                axis.annotate(str(pose.source_id), pose.xy, color='black', fontsize=7,
                              bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))
        for axis in axes:
            axis.set_xlabel('map x (m)')
            axis.set_ylabel('map y (m)')
            axis.set_xlim(field.grid.extent[:2])
            axis.set_ylim(field.grid.extent[2:])
        figure.suptitle(title + f'\nDashed red = {gate}-blocked only; not navigation infeasible', fontsize=13)
        path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(path, dpi=140)
    finally:
        figure.clear()
    return path
