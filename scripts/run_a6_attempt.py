#!/usr/bin/python3
"""Run ONE explicit pilot slot or method-independent setup in a fresh SIM.

No matrix generation or automatic replacement. Every invocation has its own
directory; the caller reviews INVALID status before requesting a rerun.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SIM = Path('/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation')
RM = Path('/tmp/rm4d-aubo-baseline-v1.14uZXq/repo')
CORE = '/media/lu/P450_PAPER/RM4D_AUBO/conda-env/bin/python'
PX4 = '/media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4'


def slot_spec(config, number):
    for slot in config['slots']:
        if slot['slot'] == number:
            return slot, next(s for s in config['scenes'] if s['id'] == slot['scene'])
    raise ValueError('only prelisted pilot slots 1..14 may run')


def scene_launch_args(scene):
    values = dict(target_x=scene['target_xy'][0], target_y=scene['target_xy'][1],
                  target_z=scene['target_z'], target_yaw=scene['target_yaw'],
                  bunker_x=scene['bunker_xy'][0], bunker_y=scene['bunker_xy'][1],
                  bunker_z=scene['bunker_z'], bunker_yaw=scene['bunker_yaw'],
                  flight_position_tolerance=.05)
    return [str(k)+':='+str(v) for k, v in values.items()]


def box_sdf(box):
    size = ' '.join(str(v) for v in box['size_xyz'])
    pose = ' '.join(str(v) for v in [*box['center_xy'], box['size_xyz'][2]/2., 0, 0, box['yaw']])
    return ('<sdf version="1.6"><model name="'+box['name']+'"><static>true</static><pose>'+pose+'</pose>'
            '<link name="body"><collision name="collision"><geometry><box><size>'+size+'</size></box></geometry></collision>'
            '<visual name="visual"><geometry><box><size>'+size+'</size></box></geometry><material>'
            '<ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>'
            '</link></model></sdf>')


def runtime_launch(source, launch_pose):
    """Forward existing public spawn arguments; retain every demo node/setting."""
    root = ET.fromstring(source)
    include = next(node for node in root.findall('include')
                   if node.get('file', '').endswith('/launch/air_ground_standalone.launch'))
    for axis, value in zip(('x', 'y', 'z', 'yaw'), launch_pose):
        ET.SubElement(include, 'arg', name='uav1_init_'+axis, value=str(value))
    return ET.tostring(root, encoding='unicode')


def adapter_args(config, output, sim, rm):
    view = config['initial_view']
    args = ['--sim-root', str(sim), '--output-dir', str(output), '--core-python', CORE,
            '--rm4d-root', str(rm), '--rm4d-config', str(rm/'configs/mr4_offline_base_placement.json'),
            '--rm4d-map', str(ROOT/'assets/rm4d_ground_task_v1/rmap.npy'),
            '--rm4d-task-asset', str(ROOT/'assets/rm4d_ground_task_v1'),
            '--max-viewpoints', str(config['observation_windows']),
            '--cloud-window-s', str(config['window_sim_s']),
            '--cloud-timeout', str(config['capture_wall_guard_s']),
            '--view-position', *map(str, view[:3]), '--view-yaw', str(view[3])]
    for key, value in config['shared_a5_settings'].items():
        if key in ('grid_resolution_m', 'grid_width_m', 'grid_height_m', 'ground_z_m', 'flight_weight'):
            continue  # fixed core defaults, no alternative method parameters
        args += ['--'+key.replace('_', '-')]
        args += list(map(str, value)) if isinstance(value, list) else [str(value)]
    if 'operational_gating' in config:
        args += ['--operational-gating', config['operational_gating']]
    return args


def stop_process(process):
    if process is None or process.poll() is not None: return
    for sig, seconds in ((signal.SIGINT, 35), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
        if process.poll() is not None: return
        try:
            os.killpg(process.pid, sig)  # only this invocation's own child session
        except ProcessLookupError:
            return  # child exited between poll and signal
        try:
            process.wait(timeout=seconds)
            return
        except subprocess.TimeoutExpired:
            continue


def diagnostic_command(config, output):
    """Native-rate failure diagnostics; no GT or demo-status subscriptions.

    Demo status is already in events; recording it could incorrectly satisfy
    the checker's connection wait. Do not activate extra depth point clouds.
    """
    if not config.get('record_diagnostics', False):
        return None
    topics = [
        '/clock', '/tf', '/tf_static', '/rosout_agg',
        '/ground/move_base/goal', '/ground/move_base/status',
        '/ground/move_base/result', '/ground/move_base/cancel',
        '/ground/nav_cmd_vel', '/ground/cmd_vel', '/ground/odom', '/ground/scan',
        '/ground/move_base/global_costmap/costmap', '/ground/move_base/local_costmap/costmap',
        '/ground/move_base/NavfnROS/plan', '/ground/move_base/DWAPlannerROS/local_plan',
        '/ground/d435/color/image_raw', '/ground/d435/depth/image_raw',
        '/ground/d435/color/camera_info', '/ground/d435/depth/camera_info',
        '/ground_observer/status', '/ground_observer/target_pose',
        '/ground/gripper/grasp_confirmed', '/pick_target/contacts',
        '/ground/joint_states', '/ground/arm_controller/state',
        '/ground/arm_controller/follow_joint_trajectory/goal',
        '/ground/arm_controller/follow_joint_trajectory/status',
        '/ground/arm_controller/follow_joint_trajectory/result',
        '/ground/arm_controller/follow_joint_trajectory/cancel',
    ]
    return ['rosbag', 'record', '--lz4', '--buffsize', '256',
            '-O', str(output/'diagnostics.bag'), *topics]


def state_ready(state):
    return bool(state.connected and state.odom_valid)


def wait_for_adapter_ready(process, log_path, deadline):
    """Start the checker only after A5's temporary publisher is replaced."""
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError('adapter exited before final publisher was ready')
        try:
            if 'A6_ADAPTER_READY' in log_path.read_text().splitlines():
                return
        except FileNotFoundError:
            pass
        time.sleep(.01)
    raise subprocess.TimeoutExpired('adapter initialization', 0)


def action_probe(client):
    """Negotiate normally without letting a stopped ROS clock block setup."""
    ready = threading.Event()
    def wait():
        if client.wait_for_server(): ready.set()
    threading.Thread(target=wait, daemon=True).start()
    return ready


def classify_outcome(physical, events, adapter_exit, checker_exit, adapter_log=''):
    # This exact failure is raised before adapter.run(), not by a task stage.
    # Exit code 2 or a later checker timeout alone cannot establish invalidity.
    if (not events and adapter_exit == 2 and
            'A6 configuration failed: A5 status subscriber did not connect before startup' in adapter_log):
        return 'INVALID_TRIAL', None, 'platform_startup_status_subscriber_not_connected'
    failed = next((e for e in events if e.get('state') == 'FAILED'), None)
    if failed:
        return 'VALID_TRIAL', False, failed.get('reason', 'method_reported_failure')
    if physical is not None:
        initial_states = ('PREFLIGHT', 'ARMING', 'COMMAND_CONTROL', 'TAKEOFF')
        recorded_initial = [e.get('state') for e in events if e.get('state') in initial_states]
        if (physical.get('status') == 'FAIL' and adapter_exit == 0 and checker_exit == 1
                and physical.get('error') == 'unexpected status TAKEOFF after []'
                and recorded_initial == list(initial_states)
                and any(e.get('state') == 'LIFT' for e in events)):
            # The completed adapter recorded the correct sequence; the checker
            # lost it during TCPROS connection turnover and never measured lift.
            return 'INVALID_TRIAL', None, 'checker_missed_initial_status_sequence'
        if physical.get('status') in ('PASS', 'CHECKS_PASS'):
            return 'VALID_TRIAL', True, None
        if adapter_exit == 0 and physical.get('error') in (
                'target baseline z is not finite', 'target final z is not finite'):
            return 'INVALID_TRIAL', None, physical['error']
        return 'VALID_TRIAL', False, physical.get('error', 'physical_check_failed')
    if adapter_exit == 0 and checker_exit is not None and checker_exit != 0:
        return 'INVALID_TRIAL', None, 'checker_primary_measurement_missing_exit_'+str(checker_exit)
    return 'VALID_TRIAL', False, 'primary_measurement_missing_independence_unproven'


def attach_outcome(record, outcome):
    status, success, reason = outcome
    record.update(status=status, retrieval_success=success, classification_reason=reason)
    if reason: record.setdefault('reason', reason)


def read_measurements(output):
    physical, events, errors = None, [], []
    path = output/'physical_summary.json'
    if path.exists():
        try: physical = json.loads(path.read_text())
        except (ValueError, OSError) as error: errors.append('physical_summary: '+str(error))
    path = output/'data/events.jsonl'
    if path.exists():
        try:
            for number, line in enumerate(path.read_text().splitlines(), 1):
                try: events.append(json.loads(line))
                except ValueError as error: errors.append('event line %d: %s' % (number, error))
        except OSError as error: errors.append('events: '+str(error))
    return physical, events, errors


def initialize_sim_node(rospy, runtime, deadline):
    # rospy chooses its clock subscription at init_node, not when roslaunch
    # later installs /use_sim_time. Wait before initializing this parent node.
    while time.monotonic() < deadline:
        if runtime.poll() is not None: raise RuntimeError('SIM exited before clock setup')
        try:
            ready = rospy.get_param('/use_sim_time', False)
        except (OSError, RuntimeError):
            ready = False  # master may still be starting
        if ready:
            rospy.init_node('a6_scene_setup', anonymous=True, disable_signals=True)
            return
        time.sleep(.2)
    raise RuntimeError('simulation clock parameter startup timeout')


def prepare_scene(scene, runtime):
    import actionlib
    import rospy
    import tf2_ros
    from std_msgs.msg import Bool
    from gazebo_msgs.srv import SpawnModel, GetModelState
    from geometry_msgs.msg import Pose
    from prometheus_msgs.msg import UAVState
    from robot_runtime_interfaces.msg import FlightCommandAction
    from move_base_msgs.msg import MoveBaseAction
    from moveit_msgs.msg import MoveGroupAction
    deadline = time.monotonic()+180
    initialize_sim_node(rospy, runtime, deadline)
    while time.monotonic() < deadline:
        if runtime.poll() is not None: raise RuntimeError('SIM exited before readiness')
        try:
            if rospy.wait_for_message('/ground/runtime_ready', Bool, timeout=1).data: break
        except rospy.ROSException:
            pass
    else: raise RuntimeError('Ground platform startup timeout')
    rospy.wait_for_service('/gazebo/get_model_state', timeout=30)
    get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    while time.monotonic() < deadline:
        if get_model('pick_target', 'world').success: break
        time.sleep(.2)
    else: raise RuntimeError('target was not spawned')
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=30)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    for box in scene['boxes']:
        pose = Pose(); pose.orientation.w = 1
        response = spawn(box['name'], box_sdf(box), '', pose, 'world')
        if not response.success: raise RuntimeError(response.status_message)
    tf_buffer = tf2_ros.Buffer(); listener = tf2_ros.TransformListener(tf_buffer)
    clients = [(name, actionlib.SimpleActionClient(name, kind)) for name, kind in (
        ('/uav1/runtime/flight', FlightCommandAction), ('/ground/move_base', MoveBaseAction),
        ('/move_group', MoveGroupAction))]
    probes = {name: action_probe(client) for name, client in clients}
    last_diagnostic = {}
    while time.monotonic() < deadline:
        try:
            state = rospy.wait_for_message('/uav1/prometheus/state', UAVState, timeout=1)
            now = rospy.Time.now().to_sec()
            transforms = [tf_buffer.lookup_transform('map', frame, rospy.Time(0), rospy.Duration(0))
                          for frame in ('uav1/base_link', 'ground/base_link')]
            ages = [now-t.header.stamp.to_sec() for t in transforms]
            last_diagnostic = dict(connected=state.connected, odom_valid=state.odom_valid, tf_ages=ages, now=now)
            ready = state_ready(state) and now > 0 and all(0 <= age <= .5 for age in ages)
            if ready:
                connected = {name: ready.is_set() for name, ready in probes.items()}
                last_diagnostic['action_connections'] = connected
                ready = all(connected.values())
            if ready:
                before = now; time.sleep(.3)
                if rospy.Time.now().to_sec() > before: return now
        except (rospy.ROSException, tf2_ros.TransformException) as error:
            last_diagnostic = dict(error=str(error))
        time.sleep(.1)
    raise RuntimeError('common platform/TF/action readiness timeout: '+json.dumps(last_diagnostic))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/a6_pilot.json')
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--slot', type=int)
    choice.add_argument('--setup-scene', choices=('easy', 'moderate', 'hard'))
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--sim-root', type=Path, default=SIM)
    parser.add_argument('--rm4d-root', type=Path, default=RM)
    parser.add_argument('--setup-view', type=float, nargs=4)
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    if args.setup_scene:
        scene = next(s for s in config['scenes'] if s['id'] == args.setup_scene)
        slot = dict(scene=scene['id'], method='SETUP_CHECK', slot=None)
        if args.setup_view: config['initial_view'] = args.setup_view
    else:
        if config['status'] != 'FROZEN_FOR_PILOT': raise ValueError('freeze setup/protocol before activating pilot slots')
        if args.setup_view: raise ValueError('pilot pose overrides are not allowed')
        slot, scene = slot_spec(config, args.slot)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    environment = os.environ.copy()
    environment.update(P450_PX4_ROOT=PX4, SIM_ROOT=str(args.sim_root),
                       ROS_MASTER_URI='http://127.0.0.1:11951', GAZEBO_MASTER_URI='http://127.0.0.1:11952',
                       ROS_LOG_DIR=str(output/'ros'), MPLCONFIGDIR='/tmp/a6-mpl', XDG_CACHE_HOME='/tmp/a6-cache')
    os.environ.update(environment)
    children, logs = [], []
    record = dict(**slot, seed=scene['seed'], scene_spec=scene, initial_view=config['initial_view'],
                  uav_launch_pose=config['uav_launch_pose'],
                  config_path=str(args.config.resolve()), protocol=config['protocol'],
                  operational_gating=config.get('operational_gating', 'v1'),
                  kind='METHOD_INDEPENDENT_SETUP' if args.setup_scene else 'PILOT_ATTEMPT',
                  status='INVALID_TRIAL', task_started=False, activation_wall=time.time())
    def save(): (output/'attempt.json').write_text(json.dumps(record, indent=2, allow_nan=False)+'\n')
    def start(command, name):
        log = (output/(name+'.log')).open('w'); logs.append(log)
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=environment,
                                 cwd=str(ROOT), start_new_session=True)
        children.append(child)
        return child
    runtime = checker = adapter = recorder = None
    save()
    try:
        source = args.sim_root/'src/demos/air_ground_pick_demo/launch/air_ground_pick_demo.launch'
        launch_file = output/'runtime.launch'
        launch_file.write_text(runtime_launch(source.read_text(), config['uav_launch_pose']))
        runtime = start(['roslaunch', str(launch_file), 'gui:=false', 'run_demo:=false',
                         'enable_mid360:=true', 'px4_workdir:=sitl_a6_%d_%d' % (time.time_ns(), os.getpid()),
                         *scene_launch_args(scene)], 'runtime')
        print('START', slot, output, flush=True)
        record['ready_sim'] = prepare_scene(scene, runtime)
        print('READY', slot, record['ready_sim'], flush=True)
        record_command = diagnostic_command(config, output)
        if record_command is not None:
            record['diagnostic_command'] = record_command
            recorder = start(record_command, 'diagnostics')
        common = adapter_args(config, output/'data', args.sim_root, args.rm4d_root)
        if args.setup_scene:
            command = ['/usr/bin/python3', str(ROOT/'scripts/a6_setup_check.py'), *common,
                       '--scene-file', str(args.config.resolve()), '--scene-id', scene['id']]
        else:
            command = ['/usr/bin/python3', str(ROOT/'scripts/run_a6_sim.py'), *common,
                       '--method', slot['method'], '--wait-for-status-subscriber']
        task_deadline = time.monotonic()+config['task_wall_guard_s']
        adapter = start(command, 'adapter')
        if not args.setup_scene:
            wait_for_adapter_ready(adapter, output/'adapter.log', task_deadline)
            checker = start(['/usr/bin/python3', str(args.sim_root/'scripts/check_air_ground_pick_demo.py'),
                             '--summary', str(output/'physical_summary.json'), '--timeout', '1250',
                             '--maximum-ground-travel', '3.0'], 'physical_checker')
        record.update(task_started=True, status='VALID_TRIAL'); save()
        record['adapter_exit'] = adapter.wait(timeout=max(0., task_deadline-time.monotonic()))
        if checker is not None:
            try: record['checker_exit'] = checker.wait(timeout=5)
            except subprocess.TimeoutExpired: record['checker_exit'] = None
        record['runtime_exit_during_task'] = runtime.poll()
    except subprocess.TimeoutExpired:
        record['reason'] = 'common_task_wall_guard_expired'
        record['status'] = 'VALID_TRIAL' if record['task_started'] else 'INVALID_TRIAL'
    except Exception as error:
        record['reason'] = '%s: %s' % (type(error).__name__, error)
        print('ERROR', record['reason'], flush=True)
    finally:
        # Checker starts last now, but must remain alive for adapter cleanup.
        stop_process(adapter)
        for child in reversed(children): stop_process(child)
        for log in logs: log.close()
        record['runtime_exit'] = None if runtime is None else runtime.poll()
        if recorder is not None:
            record['diagnostic_exit'] = recorder.poll()
            record['diagnostic_bag_finalized'] = (output/'diagnostics.bag').is_file()
        physical, events, errors = read_measurements(output)
        if errors: record['measurement_read_errors'] = errors
        if physical is not None: record['physical_status'] = physical.get('status')
        if record['task_started'] and not args.setup_scene:
            adapter_log = ''
            if record.get('adapter_exit') == 2 and not events and not errors:
                try: adapter_log = (output/'adapter.log').read_text()
                except OSError: pass
            attach_outcome(record, classify_outcome(physical, events, record.get('adapter_exit'),
                                                    record.get('checker_exit'), adapter_log))
            if record.get('classification_reason') == 'platform_startup_status_subscriber_not_connected':
                record.update(task_started=False, adapter_launched=True,
                              startup_failure='status consumer did not connect before adapter.run; no task events')
        record['finish_wall'] = time.time(); save()
    print('DONE', json.dumps(record), flush=True)
    return 0


if __name__ == '__main__': raise SystemExit(main())
