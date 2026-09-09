#!/usr/bin/python3
"""One method-independent RGB-D/ground-return/hover check; never query RM4D."""

import json
from pathlib import Path
import sys
import numpy as np


def ground_summary(points, transform, target_xy):
    """Raw endpoint plane coverage only, no A1 catalog, A2 votes or task scores."""
    points = np.asarray(points, dtype=float)
    transform = np.asarray(transform, dtype=float)
    mapped = points @ transform[:3, :3].T + transform[:3, 3]
    origin = np.asarray(target_xy) - 2.
    inside = np.all((mapped[:, :2] >= origin) & (mapped[:, :2] < origin + 4.), axis=1)
    ground = mapped[inside & (np.abs(mapped[:, 2]) <= .02)]
    cells = np.floor((ground[:, :2] - origin) / .1).astype(int)
    count = len(np.unique(cells, axis=0)) if len(cells) else 0
    return dict(ground_cells=count, ground_area_m2=count * .01,
                ground_observation_usable=bool(count >= 100),
                ground_point_count=len(ground),
                ground_z_mean_m=None if not len(ground) else float(ground[:, 2].mean()),
                ground_z_abs_max_m=None if not len(ground) else float(np.abs(ground[:, 2]).max()))


def run_setup(adapter):
    try:
        adapter._wait_preflight()
        for status in ('ARMING', 'COMMAND_CONTROL', 'TAKEOFF'):
            adapter._publish_status(status)
        adapter._flight_started = True
        adapter._execute_flight(adapter._takeoff_command, 'takeoff')
        view = list(adapter._view_position) + [adapter._view_yaw]
        adapter._publish_status('AIR_VIEW')
        adapter._a5_fly_and_hover(view, 'A6 setup initial fly-to', force_flight=True)
        adapter._publish_status('AIR_OBSERVE')
        target = adapter._observe_from_air()
        pose = adapter._a5_capture(view)
        adapter._publish_status('LANDING')
        adapter._request_land()
        return dict(kind='METHOD_INDEPENDENT_SETUP', aerial_target_map=list(target), uav_pose_map=pose)
    finally:
        adapter._safe_land()


def main(argv=None):
    import rospy
    import moveit_commander
    import yaml
    import run_a5_sim as a5
    argv = sys.argv if argv is None else argv
    parser = a5.build_parser()
    parser.add_argument('--scene-file', type=Path, required=True)
    parser.add_argument('--scene-id', required=True)
    options = parser.parse_args(rospy.myargv(argv=argv)[1:])
    a5.validate_options(parser, options)
    scene = next(s for s in json.loads(options.scene_file.read_text())['scenes'] if s['id'] == options.scene_id)
    module = a5.load_demo_module(options.sim_root)
    parent = a5.build_adapter_class(module, options)
    options.output_dir.mkdir(parents=True, exist_ok=True)

    class SetupAdapter(parent):
        def _publish_status(self, state, **details):
            with (options.output_dir / 'events.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(state=state, ros_time=rospy.Time.now().to_sec(), **details))+'\n')
            return super()._publish_status(state, **details)

    moveit_commander.roscpp_initialize(argv)
    rospy.init_node('a6_method_independent_setup')
    report = dict(kind='METHOD_INDEPENDENT_SETUP', scene_id=scene['id'], seed=scene['seed'],
                  initial_view=list(options.view_position)+[options.view_yaw], status='FAILED')
    try:
        parameters = a5.build_demo_parameters(yaml.safe_load(
            (options.sim_root / a5.DEMO_RELATIVE / 'config/demo.yaml').read_text()), options)
        for key, value in parameters.items(): rospy.set_param('~'+key, value)
        adapter = SetupAdapter()
        report.update(run_setup(adapter))
        with np.load(adapter._a5_observations[0], allow_pickle=False) as data:
            report.update(ground_summary(data['points_xyz'], data['T_map_sensor'], scene['target_xy']))
            report['window_sim_s'] = float(data['chunk_stamps_s'][-1]-data['chunk_stamps_s'][0])
        report['status'] = 'PASS' if report['ground_observation_usable'] else 'GROUND_COVERAGE_INSUFFICIENT'
    except Exception as error:
        report['reason'] = '%s: %s' % (type(error).__name__, error)
        rospy.logerr(report['reason'])
    finally:
        (options.output_dir / 'setup_summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        moveit_commander.roscpp_shutdown()
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__': raise SystemExit(main())
