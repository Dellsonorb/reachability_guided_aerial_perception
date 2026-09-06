#!/usr/bin/env python3
"""Run exactly the three approved deterministic A2 demonstrations."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from matplotlib import colormaps
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
import numpy as np

from environment_belief import EnvironmentBeliefMapper, EnvironmentState
from environment_belief.outputs import belief_summary, render_belief, save_belief
from environment_belief.synthetic import make_scenarios


def plot_observations(scene, output_path):
    """Scene truth is displayed here only, never provided to the belief mapper."""
    figure = Figure(figsize=(12, 5), layout='constrained')
    FigureCanvasAgg(figure)
    axes = figure.subplots(1, 2)
    try:
        for box in scene.boxes:
            x, y, z = box.lower
            dx, dy, dz = np.subtract(box.upper, box.lower)
            axes[0].add_patch(Rectangle((x, y), dx, dy, color='#d34b3f', alpha=0.18))
            axes[1].add_patch(Rectangle((x, z), dx, dz, color='#d34b3f', alpha=0.18))
        for i, (cloud, label) in enumerate(zip(scene.observations, scene.labels)):
            origin = cloud.T_map_sensor[:3, 3]
            hits = (cloud.points_xyz[cloud.valid_return] @ cloud.T_map_sensor[:3, :3].T + origin)
            color = colormaps['tab10'](i)
            # Display only a subset of rays, but all actual return endpoints.
            shown = hits[::max(1, len(hits) // 16)]
            for axis, vertical in zip(axes, (1, 2)):
                for hit in shown:
                    axis.plot([origin[0], hit[0]], [origin[vertical], hit[vertical]],
                              color=color, linewidth=0.6, alpha=0.3)
                axis.scatter(hits[:, 0], hits[:, vertical], s=8, color=color, alpha=0.65)
                axis.scatter(origin[0], origin[vertical], marker='*', s=100,
                             color=color, edgecolors='black', linewidths=0.4, label=label)
        axes[0].set(xlim=scene.grid.extent[:2], ylim=scene.grid.extent[2:],
                    xlabel='map x (m)', ylabel='map y (m)', title='XY endpoints and sensor positions')
        axes[0].set_aspect('equal')
        axes[1].axhline(0, color='black', linewidth=1)
        axes[1].set(xlim=scene.grid.extent[:2], ylim=(-0.1, 2.5), xlabel='map x (m)',
                    ylabel='map z (m)', title='XZ projection: first hits and actual ray heights')
        axes[1].legend(fontsize=8, loc='upper right')
        figure.suptitle(f'{scene.name} | synthetic ground/boxes shown for reference only')
        figure.savefig(output_path, dpi=150)
    finally:
        figure.clear()


def plot_progress(scene, history, output_path):
    figure = Figure(figsize=(4 * len(history), 7), layout='constrained')
    FigureCanvasAgg(figure)
    axes = figure.subplots(2, len(history), squeeze=False)
    raster = dict(origin='lower', interpolation='none', extent=scene.grid.extent, aspect='equal')
    colors = ListedColormap(['#bdbdbd', '#35a872', '#d34b3f'])
    try:
        for i, (belief, label) in enumerate(zip(history, scene.labels)):
            categories = np.zeros(scene.grid.shape, dtype=np.int8)
            categories[belief.state == EnvironmentState.FREE] = 1
            categories[belief.state == EnvironmentState.OCCUPIED] = 2
            state_image = axes[0, i].imshow(categories, cmap=colors,
                                           norm=BoundaryNorm([-0.5, 0.5, 1.5, 2.5], 3), **raster)
            score_image = axes[1, i].imshow(belief.unknown_score, vmin=0, vmax=1, cmap='magma', **raster)
            axes[0, i].set_title(label, fontsize=10)
            for axis in axes[:, i]:
                axis.set_xlabel('map x (m)')
                axis.set_ylabel('map y (m)')
                for row, col in scene.probe_cells.values():
                    x = scene.grid.origin_xy[0] + (col + 0.5) * scene.grid.resolution_m
                    y = scene.grid.origin_xy[1] + (row + 0.5) * scene.grid.resolution_m
                    axis.scatter(x, y, marker='s', s=70, facecolors='none', edgecolors='#00bfff')
        bar = figure.colorbar(state_image, ax=list(axes[0]), ticks=[0, 1, 2], shrink=0.7)
        bar.ax.set_yticklabels(['UNKNOWN', 'FREE', 'OCCUPIED'])
        figure.colorbar(score_image, ax=list(axes[1]), shrink=0.7, label='unknown_score (heuristic)')
        figure.suptitle(f'{scene.name} | cumulative state / unknown_score; blue square = probe')
        figure.savefig(output_path, dpi=140)
    finally:
        figure.clear()


def check_scenario(scene, history):
    if scene.name == 'clear':
        valid = (np.count_nonzero(history[0].state == EnvironmentState.FREE) == 0
                 and np.count_nonzero(history[-1].state == EnvironmentState.FREE) == 576)
    elif scene.name == 'overflight_obstacle':
        cell = scene.probe_cells['box_edge']
        valid = (all(b.state[cell] == EnvironmentState.UNKNOWN and b.unknown_score[cell] == 1
                     for b in history[:2])
                 and all(b.state[cell] == EnvironmentState.OCCUPIED for b in history[2:])
                 and history[-1].free_evidence[cell] == 2)
    else:
        cell = scene.probe_cells['hidden_ground']
        valid = (history[0].observation_count[cell] == 0
                 and history[1].state[cell] == EnvironmentState.UNKNOWN
                 and history[2].state[cell] == EnvironmentState.FREE
                 and history[2].free_evidence[cell] == 2)
    if not valid:
        raise RuntimeError(f'{scene.name}: expected endpoint/multiview semantics failed')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, default=Path('outputs/a2'))
    args = parser.parse_args(argv)
    for scene in make_scenarios():
        mapper = EnvironmentBeliefMapper(scene.grid, sensor_frame='lidar')
        history, steps = [], []
        for label, cloud in zip(scene.labels, scene.observations):
            update = mapper.update(cloud)
            belief = mapper.snapshot()
            history.append(belief)
            probes = {}
            for name, cell in scene.probe_cells.items():
                probes[name] = {
                    'row_col': list(cell), 'state': EnvironmentState(int(belief.state[cell])).name,
                    'occupied_evidence': int(belief.occupied_evidence[cell]),
                    'free_evidence': int(belief.free_evidence[cell]),
                    'observation_count': int(belief.observation_count[cell]),
                    'unknown_score': float(belief.unknown_score[cell]),
                }
            steps.append({'label': label, 'update': asdict(update),
                          'cells': belief_summary(belief)['cells'], 'probes': probes})
        check_scenario(scene, history)
        directory = args.output_root / scene.name
        save_belief(history[-1], directory)
        render_belief(history[-1], directory / 'belief.png')
        plot_observations(scene, directory / 'observations.png')
        plot_progress(scene, history, directory / 'progress.png')
        sequence = {'scene': scene.name, 'boxes_map': [asdict(box) for box in scene.boxes],
                    'sensor_frame': 'lidar', 'steps': steps}
        (directory / 'sequence.json').write_text(
            json.dumps(sequence, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        arrays = {name: np.stack([getattr(b, name) for b in history]) for name in (
            'state', 'free_evidence', 'occupied_evidence', 'observation_count', 'unknown_score')}
        for i, cloud in enumerate(scene.observations):
            arrays.update({f'view_{i}_{name}': getattr(cloud, name) for name in (
                'points_xyz', 'T_map_sensor', 'valid_return', 'stamp_s')})
        np.savez_compressed(directory / 'sequence.npz', **arrays)
        print(json.dumps({'scene': scene.name, 'views': len(history),
                          'cells': belief_summary(history[-1])['cells'],
                          'probes': steps[-1]['probes']}, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
