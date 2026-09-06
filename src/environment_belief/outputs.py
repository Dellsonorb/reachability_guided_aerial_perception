"""Compact A2 outputs and local Agg plots; no ROS or global backend changes."""

from dataclasses import asdict
import json
from pathlib import Path

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.figure import Figure
import numpy as np

from .core import EnvironmentBeliefGrid, EnvironmentState


def belief_summary(belief: EnvironmentBeliefGrid):
    count = belief.observation_count
    return {
        'schema_version': 1,
        'field_type': 'endpoint_environment_belief',
        'frame_id': belief.frame_id,
        'grid': {
            'origin_xy': list(belief.origin_xy),
            'resolution_m': float(belief.resolution_m),
            'width_cells': int(belief.width_cells),
            'height_cells': int(belief.height_cells),
            'array_convention': 'row-major [y, x]; half-open cells',
        },
        'config': asdict(belief.config),
        'cells': {state.name.lower(): int(np.count_nonzero(belief.state == state))
                  for state in EnvironmentState},
        'observed_cells': int(np.count_nonzero(count)),
        'informative_cell_observations': int(count.sum()),
        'free_votes': int(belief.free_evidence.sum()),
        'occupied_votes': int(belief.occupied_evidence.sum()),
        'score_semantics': 'observation_deficit_heuristic_not_probability',
        'unknown_score_formula': 'exp(-observation_count / unknown_scale)',
        'unknown_score_min': float(belief.unknown_score.min()),
        'unknown_score_mean': float(belief.unknown_score.mean()),
        'free_semantics': 'repeated_ground_support_without_observed_obstacle',
        'scene_assumption': 'static_horizontal_ground',
        'occupied_priority': True,
        'ray_carving': False,
    }


def save_belief(belief: EnvironmentBeliefGrid, output_dir):
    """Save a mapper snapshot; overwrites only these named products."""
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    arrays = {name: getattr(belief, name) for name in (
        'state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score')}
    belief_path = directory / 'belief.npz'
    np.savez_compressed(belief_path, **arrays, frame_id=belief.frame_id,
                        origin_xy=belief.origin_xy, resolution_m=belief.resolution_m,
                        width_cells=belief.width_cells, height_cells=belief.height_cells)
    summary_path = directory / 'summary.json'
    summary_path.write_text(json.dumps(belief_summary(belief), allow_nan=False,
                                       indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return {'belief': belief_path, 'summary': summary_path}


def render_belief(belief: EnvironmentBeliefGrid, output_path):
    """Raw state, observation-deficit score and accumulated observation count."""
    path = Path(output_path)
    if path.suffix.lower() != '.png':
        raise ValueError('output_path must have a .png suffix')
    figure = Figure(figsize=(14, 4.8), layout='constrained')
    FigureCanvasAgg(figure)
    axes = figure.subplots(1, 3)
    raster = dict(origin='lower', interpolation='none', extent=belief.grid.extent, aspect='equal')
    colors = ListedColormap(['#bdbdbd', '#35a872', '#d34b3f'])
    categories = np.full(belief.state.shape, 0, dtype=np.int8)
    categories[belief.state == EnvironmentState.FREE] = 1
    categories[belief.state == EnvironmentState.OCCUPIED] = 2
    try:
        state = axes[0].imshow(categories, cmap=colors,
                               norm=BoundaryNorm([-0.5, 0.5, 1.5, 2.5], 3), **raster)
        axes[0].set_title('Ground-surface belief')
        bar = figure.colorbar(state, ax=axes[0], ticks=[0, 1, 2], shrink=0.75)
        bar.ax.set_yticklabels(['UNKNOWN', 'FREE', 'OCCUPIED'])
        score = axes[1].imshow(belief.unknown_score, cmap='magma', vmin=0, vmax=1, **raster)
        axes[1].set_title('unknown_score (not a probability)')
        figure.colorbar(score, ax=axes[1], shrink=0.75)
        count = axes[2].imshow(belief.observation_count, cmap='Blues', vmin=0,
                               vmax=max(1, int(belief.observation_count.max())), **raster)
        axes[2].set_title('Informative observation count')
        figure.colorbar(count, ax=axes[2], shrink=0.75)
        for axis in axes:
            axis.set_xlabel('map x (m)')
            axis.set_ylabel('map y (m)')
        summary = belief_summary(belief)
        figure.suptitle(f"A2 endpoint belief | observed cells {summary['observed_cells']}/{belief.state.size}")
        path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(path, dpi=140)
    finally:
        figure.clear()
    return path
