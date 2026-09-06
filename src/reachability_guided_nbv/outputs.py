"""Inspectable NPZ/JSON rankings and headless, unsmoothed research plots."""

from dataclasses import asdict
import json
from pathlib import Path

from matplotlib import colormaps
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Polygon, Rectangle
import numpy as np

from environment_belief import EnvironmentState
from task_relevant_uncertainty.core import FIELD_ARRAY_NAMES
from task_relevant_uncertainty.geometry import footprint_vertices
from task_relevant_uncertainty.outputs import field_summary
from .geometry import segments_intersect_box, sensor_transform
from .model import Viewpoint


def save_result(result, task, belief, output_dir):
    """Save a ranking and its A3/A2 input arrays; no separate execution framework."""
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    paths = {'ranking': directory / 'ranking.json', 'arrays': directory / 'fields.npz'}
    summary = {
        'gain_semantics': 'predicted_observation_gain_surrogate', 'status': result.status,
        'formula': 'G_task = (1-exp(-1/tau)) * sum(V * U_task)',
        'visibility_semantics': 'idealized_one_new_informative_endpoint_opportunity',
        'unknown_occludes': False, 'height_semantics': 'assumed_occlusion_surrogate_not_A2_measured_height',
        'flight_cost_semantics': 'straight_line_plus_yaw_proxy_not_path_or_energy',
        'current': asdict(result.current), 'config': asdict(result.config),
        'sensor': {'T_uav_lidar': result.sensor.T_uav_lidar.tolist(),
                   'min_elevation_deg': result.sensor.min_elevation_deg,
                   'max_elevation_deg': result.sensor.max_elevation_deg,
                   'horizontal_fov_deg': 360, 'yaw_equivalent': result.sensor.yaw_equivalent},
        'a2_config': asdict(belief.config), 'a3_summary': field_summary(task),
        'a3_poses': [asdict(p) for p in task.poses],
        'task_order': result.task_order, 'generic_order': result.generic_order,
        'best_task_id': None if result.best_task is None else result.best_task.candidate_id,
        'best_generic_id': None if result.best_generic is None else result.best_generic.candidate_id,
        'candidates': [asdict(c) for c in result.candidates],
    }
    paths['ranking'].write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    arrays = {f'a3_{name}': getattr(task, name) for name in FIELD_ARRAY_NAMES}
    arrays.update({f'a2_{name}': getattr(belief, name) for name in
                   ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score')})
    np.savez_compressed(paths['arrays'], **arrays, visibility=result.visibility,
                        delta_unknown=result.delta_unknown, marginal_task_gain=result.marginal_task_gain,
                        task_contributions=np.stack([result.task_contribution(i) for i in range(len(result.candidates))]))
    return paths


def render_result(result, task, belief, output_path, *, title='A4 predicted observation gain'):
    figure = Figure(figsize=(15, 11), layout='constrained')
    FigureCanvasAgg(figure)
    axes = figure.subplots(3, 2).ravel()
    raster = dict(origin='lower', interpolation='none', aspect='equal', extent=belief.grid.extent)
    cmap = colormaps['viridis'].with_extremes(bad='#dedede')

    def panel(axis, values, label, maximum=1):
        im = axis.imshow(values, vmin=0, vmax=maximum, cmap=cmap, **raster)
        axis.set_title(label, fontsize=10)
        figure.colorbar(im, ax=axis, shrink=.65)
        axis.set_xlabel('map x (m)')
        axis.set_ylabel('map y (m)')
        for pose in task.poses:
            axis.add_patch(Polygon(footprint_vertices(pose.xy, pose.yaw, task.footprint),
                                   fill=False, edgecolor='#d35c13', linewidth=.9))
        rows, cols = np.nonzero(belief.state == EnvironmentState.OCCUPIED)
        if len(rows):
            axis.scatter(belief.origin_xy[0] + (cols + .5) * .1,
                         belief.origin_xy[1] + (rows + .5) * .1, c='#df2020', marker='s', s=6)

    panel(axes[0], belief.unknown_score, 'A2 unknown_score; orange = BUNKER footprints')
    panel(axes[1], task.task_relevant_uncertainty, 'A3 U_task; gray = NO_VALIDATED_SUPPORT')
    selected = (result.best_task, result.best_generic)
    for axis, candidate, label in zip(axes[2:4], selected, ('Ours', 'Generic')):
        if candidate is None:
            panel(axis, np.zeros(belief.grid.shape), f'{label}: no selection')
            continue
        visible = result.visibility[candidate.candidate_id]
        panel(axis, visible.astype(float),
              f'{label} #{candidate.candidate_id}: predicted visibility (not observed free)')
        x, y, _ = candidate.viewpoint.position_xyz
        yaw = candidate.viewpoint.yaw_rad
        axis.plot(x, y, 'rx', markersize=8)
        axis.arrow(x, y, .7 * np.cos(yaw), .7 * np.sin(yaw), color='red', width=.025)
    if result.best_task is not None:
        panel(axes[4], result.task_contribution(result.best_task.candidate_id),
              'Ours: per-cell predicted task gain', maximum=-np.expm1(-1 / belief.config.unknown_scale))
    else:
        panel(axes[4], result.marginal_task_gain, 'No predicted task gain')
    ids = [c.candidate_id for c in result.candidates if c.status == 'VALID']
    axes[5].bar(np.asarray(ids) - .2, [result.candidates[i].task_score for i in ids], .4, label='Ours score')
    axes[5].bar(np.asarray(ids) + .2, [result.candidates[i].generic_score for i in ids], .4, label='Generic score')
    axes[5].set_xticks(ids)
    axes[5].set_xlabel('Shared candidate id')
    axes[5].set_ylabel('Surrogate gain - lambda * flight proxy')
    axes[5].set_title(f'lambda={result.config.flight_weight:g}; scores have different task weighting', fontsize=10)
    axes[5].legend()
    axes[5].grid(axis='y', alpha=.2)
    figure.suptitle(title + f'\nH_assumed={result.config.assumed_height_m:g} m surrogate; '
                   'UNKNOWN transparent; no real scan or flight', fontsize=13)
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        figure.savefig(path, dpi=130)
    finally:
        figure.clear()
    return path


def render_occlusion_rays(output_path):
    """Side view of the same ground ray above / through two alternative prisms."""
    origin = sensor_transform(Viewpoint((0, 0, 1.5), 0))[:3, 3]
    target = np.array([4.05, .05, 0])
    figure = Figure(figsize=(11, 3.7), layout='constrained')
    FigureCanvasAgg(figure)
    for axis, x in zip(figure.subplots(1, 2), (1., 3.)):
        hit = bool(segments_intersect_box(origin, target[None], [x, 0, 0], [x + .1, .1, 1])[0])
        axis.add_patch(Rectangle((x, 0), .1, 1, facecolor='#d34b3f', alpha=.65))
        axis.plot([origin[0], target[0]], [origin[2], target[2]], 'o-', color='#2e6ca8')
        axis.axhline(0, color='black', linewidth=.8)
        axis.set(xlim=(-.1, 4.4), ylim=(-.1, 2), xlabel='map x (m)', ylabel='map z (m)',
                 title='THROUGH prism: blocked' if hit else 'ABOVE prism: visible')
        axis.text(x + .15, .8, 'H_assumed = 1 m\nnot measured height', fontsize=9)
        axis.grid(alpha=.2)
    figure.suptitle('Same XY crossing, different 3D intersection; no 2D ray-carving')
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        figure.savefig(path, dpi=140)
    finally:
        figure.clear()
    return path
