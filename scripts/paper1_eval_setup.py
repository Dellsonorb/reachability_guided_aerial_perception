#!/usr/bin/python3
"""Offline nominal setup description only: no sensor, RM4D, planning or Gazebo."""

import argparse
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from a6_scene import depth_admission, read_sensor_contract, rotation_rpy


def rectangles_overlap(first, second):
    def vertices(box):
        corners = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*np.array(box['size_xyz'][:2])/2.
        rotation = rotation_rpy(0,0,box['yaw'])[:2,:2]
        return corners @ rotation.T + np.asarray(box['center_xy']), rotation
    a, ra = vertices(first)
    b, rb = vertices(second)
    for axis in np.vstack((ra.T,rb.T)):
        pa, pb = a @ axis, b @ axis
        if pa.max() < pb.min()-1e-12 or pb.max() < pa.min()-1e-12:
            return False
    return True


def describe_scene(scene, view, contract, horizontal_fov, image_aspect):
    row = dict(scene_id=scene['id'], seed=scene['seed'],
               **depth_admission(scene,view,contract))
    corners = np.array(list(itertools.product((-1.,1.),repeat=3)))*np.array(contract['target_size'])/2.
    points = corners @ rotation_rpy(0,0,scene['target_yaw']).T
    points += np.array([*scene['target_xy'],scene['target_z']])
    body = rotation_rpy(0,0,view[3])
    optical = body @ rotation_rpy(*contract['optical_rpy'])
    origin = np.array(view[:3]) + body @ np.array(contract['optical_xyz'])
    camera = (points-origin) @ optical
    tan_x = math.tan(horizontal_fov/2.)
    tan_y = tan_x/image_aspect
    inside = ((camera[:,2] > contract['min_depth']) & (camera[:,2] < contract['max_depth'])
              & (np.abs(camera[:,0]) < camera[:,2]*tan_x)
              & (np.abs(camera[:,1]) < camera[:,2]*tan_y))
    row['nominal_target_inside_depth_and_fov'] = bool(inside.all())
    row['optical_corner_xyz_bounds_m'] = [camera.min(axis=0).tolist(),camera.max(axis=0).tolist()]
    # Existing padded BUNKER footprint is a necessary spawn separation check,
    # not a new whole-robot IK/route admission test. No candidates are queried.
    bodies = [dict(name='target',center_xy=scene['target_xy'],size_xyz=contract['target_size'],
                   yaw=scene['target_yaw']),
              dict(name='bunker_padded_footprint',center_xy=scene['bunker_xy'],
                   size_xyz=[1.04,.78,1.],yaw=scene['bunker_yaw']), *scene['boxes']]
    row['spawn_xy_overlaps'] = [[a['name'],b['name']] for a,b in itertools.combinations(bodies,2)
                                if rectangles_overlap(a,b)]
    row['setup_checks_pass'] = row['nominal_target_inside_depth_and_fov'] and not row['spawn_xy_overlaps']
    return row


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    sim = Path(config['evaluation_version']['sim_root'])
    contract = read_sensor_contract(sim)
    sensor_path = sim/'install/p450-clean/share/prometheus_gazebo/gazebo_models/sensor_models/D435i/model.sdf'
    camera = ET.parse(sensor_path).find('.//sensor[@name="color"]/camera')
    fov = float(camera.findtext('horizontal_fov'))
    width,height = (int(camera.findtext('image/'+key)) for key in ('width','height'))
    rows = [describe_scene(s,config['initial_view'],contract,fov,width/height) for s in config['scenes']]
    report = dict(cohort=config['cohort'], simulation_started=False, method_calls=0,
                  check='necessary nominal depth/FOV and spawn XY separation; no live admission',
                  geometry_sources=contract['sources']+[str(sensor_path)],
                  color_horizontal_fov_rad=fov, image_width=width,image_height=height,
                  frozen_observer_depth_limits_m=[contract['min_depth'],contract['max_depth']],
                  all_setup_checks_pass=all(row['setup_checks_pass'] for row in rows),scenes=rows,
                  limitations='Does not assess live occlusion/detection, stowed-link self collision, route, candidate count, support, IK or retrieval. Nominal robot models/initial joints remain the existing evaluated setup.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        stream.write(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('OFFLINE SETUP:',len(rows),'scenes;',sum(r['setup_checks_pass'] for r in rows),
          'pass; no scene removed or changed; no simulations')
    return 0 if report['all_setup_checks_pass'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
