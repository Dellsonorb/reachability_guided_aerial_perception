"""A6 method routing at the existing NPZ/JSON numerical worker boundary."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec, PointCloudObservation
from reachability_guided_aerial_perception import GraspTCP
from reachability_guided_nbv import Viewpoint
from sim_active_perception.core import A5Config, replay_observations
from sim_active_perception.worker import initialize, make_field, render_result, save_belief, save_result, write_json
from task_relevant_uncertainty import build_task_uncertainty
from .policy import decide_policy, rm4d_top_one


def observe(request):
    initial = json.loads(Path(request['initial_file']).read_text())
    grasp, config = GraspTCP(**initial['grasp']), A5Config(**initial['config'])
    raw = initial['result']
    field = make_field(grasp, raw, config)
    grid = EnvironmentGridSpec(field.grid.origin_xy, field.grid.width_cells, field.grid.height_cells)
    paths = request['observations']
    if not paths or len(paths) > config.max_viewpoints:
        raise ValueError('observation history must have 1..max_viewpoints frames')
    observations = []
    for path in paths:
        with np.load(path, allow_pickle=False) as data:
            observations.append(PointCloudObservation(data['points_xyz'], str(data['frame_id'].item()),
                                                       float(data['stamp_s'].item()), data['T_map_sensor']))
    belief = replay_observations(grid, observations, BeliefConfig(ground_z_m=config.ground_z_m))
    pose = request['uav_pose']
    if len(pose) != 4:
        raise ValueError('uav_pose must be [x,y,z,yaw]')
    method = request.get('method', 'ours')
    choice, ranking = decide_policy(field, raw, belief, Viewpoint(tuple(pose[:3]), pose[3]),
                                    method=method, round_count=len(paths), config=config)
    task = build_task_uncertainty(field, belief)
    directory = Path(request['output_dir'])
    save_belief(belief, directory / 'a2')
    save_result(ranking, task, belief, directory)
    render_result(ranking, task, belief, directory / 'nbv.png', title=f'A6 {method} observation {len(paths)}')
    write_json(directory / 'decision.json', choice)
    return choice


def rm4d_select(request):
    initial = json.loads(Path(request['initial_file']).read_text())
    selected = rm4d_top_one(initial['result'])
    choice = dict(ok=True, selected_candidate=selected,
                  stop_reason='RM4D_TOP_ONE_SELECTED' if selected is not None else 'NO_RM4D_CANDIDATE')
    if request.get('output_dir'):
        write_json(Path(request['output_dir']) / 'decision.json', choice)
    return choice


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--response', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        request = json.loads(args.request.read_text())
        if request['op'] == 'init':
            response = initialize(request)
        elif request['op'] == 'observe':
            response = observe(request)
        elif request['op'] == 'rm4d_select':
            response = rm4d_select(request)
        else:
            raise ValueError('unsupported core operation')
    except Exception as error:
        write_json(args.response, {'ok': False, 'error': f'{type(error).__name__}: {error}'})
        print(f'A6 core failed: {error}', file=sys.stderr)
        return 1
    write_json(args.response, response)
    return 0
