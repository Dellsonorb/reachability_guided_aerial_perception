#!/usr/bin/python3
"""Read-only necessary RGB-D scene admission; no controller or Gazebo calls."""

import argparse
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def rotation_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def read_sensor_contract(sim_root):
    """Use the public optical transform; requires Noetic's existing PyYAML."""
    import yaml
    root = Path(sim_root)
    launch = root / 'src/platform/sim_platform_bringup/launch/p450_runtime.launch'
    observer = root / 'src/demos/air_ground_pick_demo/config/air_observer.yaml'
    demo = root / 'src/demos/air_ground_pick_demo/config/demo.yaml'
    params = {p.attrib['name']: p.attrib.get('value') for p in ET.parse(launch).iter('param')}
    config = yaml.safe_load(observer.read_text())
    robot = yaml.safe_load(demo.read_text())
    return dict(optical_xyz=[float(params['D435i/offset_'+a]) for a in ('x', 'y', 'z')],
                optical_rpy=[float(params['D435i/offset_'+a]) for a in ('roll', 'pitch', 'yaw')],
                min_depth=float(config['min_depth']), max_depth=float(config['max_depth']),
                target_size=list(robot['target_size']),
                sources=[str(launch), str(observer), str(demo)])


def depth_admission(scene, view, contract):
    """A linear optical-depth bound over the entire oriented brick cuboid.

    No overlap proves this nominal setup is inadmissible. Overlap alone does
    not establish FOV, pixel count, occlusion, live bootstrap, or method success.
    Assumes the design's static horizontal ground and level nominal UAV.
    """
    view = np.asarray(view, dtype=float)
    center = np.asarray([*scene['target_xy'], scene['target_z']], dtype=float)
    size = np.asarray(contract['target_size'], dtype=float)
    mount = np.asarray(contract['optical_xyz'], dtype=float)
    rpy = np.asarray(contract['optical_rpy'], dtype=float)
    yaw = float(scene['target_yaw'])
    lower, upper = float(contract['min_depth']), float(contract['max_depth'])
    if (view.shape != (4,) or center.shape != (3,) or size.shape != (3,)
            or mount.shape != (3,) or rpy.shape != (3,)
            or not np.all(np.isfinite(np.r_[view, center, size, mount, rpy, yaw, lower, upper]))
            or np.any(size <= 0) or not 0 < lower < upper):
        raise ValueError('finite poses, positive target size and ordered depth bounds required')
    corners = np.array(list(itertools.product((-1., 1.), repeat=3))) * size / 2
    world_corners = corners @ rotation_rpy(0, 0, yaw).T + center
    body = rotation_rpy(0, 0, view[3])
    optical_rotation = body @ rotation_rpy(*rpy)
    optical_origin = view[:3] + body @ mount
    depths = (world_corners - optical_origin) @ optical_rotation[:, 2]
    minimum, maximum = float(depths.min()), float(depths.max())
    overlap = maximum > lower and minimum < upper
    return dict(status='DEPTH_OPPORTUNITY_ONLY' if overlap else 'SCENE_SETUP_INADMISSIBLE',
                depth_overlap=overlap, optical_depth_bounds_m=[minimum, maximum],
                frozen_depth_limits_m=[lower, upper], nominal_view_map=view.tolist(),
                optical_origin_map=optical_origin.tolist(),
                reason=None if overlap else 'entire_nominal_brick_outside_frozen_aerial_depth_gate')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-root', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    contract = read_sensor_contract(args.sim_root)
    reports = [dict(scene_id=s['id'], seed=s['seed'], **depth_admission(s, config['initial_view'], contract))
               for s in config['scenes']]
    report = dict(check='necessary_rgbd_depth_admission', simulation_started=False,
                  activated_attempts=0, config=str(args.config), sensor_contract=contract,
                  scenes=reports)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if all(r['depth_overlap'] for r in reports) else 2


if __name__ == '__main__':
    raise SystemExit(main())
