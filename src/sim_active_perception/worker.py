"""Small file boundary between ROS Noetic and the existing Python 3.10 core."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

import numpy as np

from environment_belief import BeliefConfig, EnvironmentGridSpec, PointCloudObservation
from environment_belief.outputs import save_belief
from reachability_guided_aerial_perception import GraspTCP, GridSpec, build_field_from_result
from reachability_guided_aerial_perception.cli import open_frozen_rm4d_api
from reachability_guided_aerial_perception.outputs import save_field_bundle
from reachability_guided_nbv import Viewpoint
from reachability_guided_nbv.outputs import render_result, save_result
from task_relevant_uncertainty import build_task_uncertainty
from .core import A5Config, candidate_catalog, decide, replay_observations


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def make_field(grasp, raw, config):
    grid = GridSpec.centered(grasp.position_xyz[:2], config.grid_width_m, config.grid_height_m, .1)
    return build_field_from_result(grasp, raw, grid=grid)


def save_initial(grasp, raw, config, output_dir, query_request=None):
    field = make_field(grasp, raw, config)
    directory = Path(output_dir).resolve()
    initial_path = directory / 'initial.json'
    write_json(initial_path, dict(grasp=grasp.as_request(), result=raw, config=asdict(config),
                                  query_request=query_request))
    save_field_bundle(field, raw['evaluated_candidates'], directory / 'a1')
    return dict(ok=True, initial_file=str(initial_path), candidate_count=len(candidate_catalog(field, raw)),
                a1_status=field.status.value, evaluated=raw['summary']['evaluated'],
                baseline_valid=raw['summary']['valid'])


def initialize(request):
    grasp = GraspTCP(**request['grasp'])
    config = A5Config(**request.get('config', {}))
    # Reuse the already frozen SIM numerical interface, without changing exact TCP.
    integration_path = Path(request['sim_root']) / 'src/integrations/rm4d_sim_integration/src'
    sys.path.insert(0, str(integration_path))
    from rm4d_sim_integration.geometry import PoseValues, build_rm4d_request
    query = build_rm4d_request('map', PoseValues(grasp.position_xyz, grasp.quaternion_xyzw),
                               request['current_bunker_pose'], grasp.grasp_id)
    with open_frozen_rm4d_api(request['rm4d_root'], request['rm4d_config'], request['rm4d_map']) as api:
        raw = api.plan(query, top_k=1)
    return save_initial(grasp, raw, config, request['output_dir'], query)


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
    choice, ranking = decide(field, raw, belief, Viewpoint(tuple(pose[:3]), pose[3]),
                              round_count=len(paths), config=config)
    task = build_task_uncertainty(field, belief)
    directory = Path(request['output_dir'])
    save_belief(belief, directory / 'a2')
    save_result(ranking, task, belief, directory)
    render_result(ranking, task, belief, directory / 'nbv.png', title=f'A5 SIM observation {len(paths)}')
    write_json(directory / 'decision.json', choice)
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
        else:
            raise ValueError('unsupported core operation')
    except Exception as error:
        write_json(args.response, {'ok': False, 'error': f'{type(error).__name__}: {error}'})
        print(f'A5 core failed: {error}', file=sys.stderr)
        return 1
    write_json(args.response, response)
    return 0
