#!/usr/bin/python3
"""Run ONE explicit frozen slot or method-independent setup in a fresh SIM.

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
IMAGE_TOPICS = ('/ground/d435/color/image_raw', '/ground/d435/depth/image_raw')


def attempt_kind(status):
    kinds = {'FROZEN_FOR_PILOT': 'PILOT_ATTEMPT',
             'FROZEN_FOR_FORMAL': 'FORMAL_ATTEMPT',
             'DEVELOPMENT_BATCH': 'DEVELOPMENT_ATTEMPT'}
    if status not in kinds:
        raise ValueError('freeze setup/protocol before activating slots')
    return kinds[status]


def slot_spec(config, number):
    for slot in config['slots']:
        if slot['slot'] == number:
            return slot, next(s for s in config['scenes'] if s['id'] == slot['scene'])
    raise ValueError('only slots explicitly listed in the supplied configuration may run')


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


def solver_contrast_world(source, solver, status):
    """One development physics contrast; retain every other world setting."""
    if status != 'DEVELOPMENT_BATCH' or solver not in ('quick', 'world'):
        raise ValueError('ODE solver contrast is development only')
    root = ET.fromstring(source)
    world = root.find('world')
    physics = world.find('physics')
    if physics is None:
        physics = ET.SubElement(world, 'physics', name='default_physics', type='ode')
    if physics.get('type') != 'ode':
        raise ValueError('solver contrast requires an ODE source world')
    node = physics
    for tag in ('ode', 'solver', 'type'):
        child = node.find(tag)
        node = ET.SubElement(node, tag) if child is None else child
    node.text = solver
    return ET.tostring(root, encoding='unicode')


def runtime_launch(source, launch_pose, world=None):
    """Forward existing public spawn arguments; retain every demo node/setting."""
    root = ET.fromstring(source)
    include = next(node for node in root.findall('include')
                   if node.get('file', '').endswith('/launch/air_ground_standalone.launch'))
    for axis, value in zip(('x', 'y', 'z', 'yaw'), launch_pose):
        ET.SubElement(include, 'arg', name='uav1_init_'+axis, value=str(value))
    if world is not None:
        ET.SubElement(include, 'arg', name='world', value=str(world))
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
    if 'support_anchor' in config:
        args += ['--support-anchor', config['support_anchor']]
    if 'handoff_stop' in config:
        args += ['--handoff-stop', config['handoff_stop']]
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
        '/air_observer/target_pose', '/uav1/prometheus/state',
        '/ground/move_base/goal', '/ground/move_base/status',
        '/ground/move_base/result', '/ground/move_base/cancel',
        '/ground/nav_cmd_vel', '/ground/cmd_vel', '/ground/odom', '/ground/scan',
        '/ground/move_base/global_costmap/costmap', '/ground/move_base/local_costmap/costmap',
        '/ground/move_base/NavfnROS/plan', '/ground/move_base/DWAPlannerROS/local_plan',
        '/ground/d435/color/image_raw', '/ground/d435/depth/image_raw',
        '/ground/d435/color/camera_info', '/ground/d435/depth/camera_info',
        '/ground_observer/status', '/ground_observer/target_pose', '/ground_observer/surface_cue',
        '/ground/gripper/grasp_confirmed', '/pick_target/contacts',
        '/ground/joint_states', '/ground/arm_controller/state',
        '/ground/arm_controller/follow_joint_trajectory/goal',
        '/ground/arm_controller/follow_joint_trajectory/status',
        '/ground/arm_controller/follow_joint_trajectory/result',
        '/ground/arm_controller/follow_joint_trajectory/cancel',
    ]
    if config.get('status') == 'FROZEN_FOR_FORMAL':
        topics += [
            '/ground/gripper_controller/follow_joint_trajectory/goal',
            '/ground/gripper_controller/follow_joint_trajectory/status',
            '/ground/gripper_controller/follow_joint_trajectory/result',
            '/ground/gripper_controller/follow_joint_trajectory/cancel',
            '/ground/gripper_controller/state',
        ]
    if config.get('diagnostic_image_scope') == 'ground_handoff_to_end':
        topics = [topic for topic in topics if topic not in IMAGE_TOPICS]
    return ['rosbag', 'record', '--lz4', '--buffsize', '256',
            '-O', str(output/'diagnostics.bag'), *topics]


def image_diagnostic_command(config, output):
    if (not config.get('record_diagnostics', False)
            or config.get('diagnostic_image_scope') != 'ground_handoff_to_end'):
        return None
    return ['rosbag', 'record', '--lz4', '--buffsize', '256',
            '-O', str(output/'diagnostics-images.bag'), *IMAGE_TOPICS]


def wait_for_adapter(process, deadline, events_path, on_selection=None):
    """Observe existing flushed handoff events without delaying robot action."""
    if on_selection is None:
        return process.wait(timeout=max(0., deadline-time.monotonic()))
    while True:
        if time.monotonic() >= deadline:
            return process.wait(timeout=0)
        if on_selection is not None:
            try:
                lines = events_path.read_text().splitlines()
            except OSError:
                lines = []  # startup/setup need not have an events file yet
            for line in lines:
                try:
                    event = json.loads(line)
                except ValueError:
                    continue  # an append may not yet contain a whole JSON row
                if isinstance(event, dict) and event.get('state') in ('A5_SELECTED', 'A6_RM4D_SELECTED'):
                    callback, on_selection = on_selection, None
                    callback(event)
                    break
        try:
            return process.wait(timeout=min(.2, max(0., deadline-time.monotonic())))
        except subprocess.TimeoutExpired:
            if time.monotonic() >= deadline:
                raise


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


def validate_replay_scene(recorded, scene):
    if recorded['scene_spec'] != scene:
        raise ValueError('Ground replay must use the archived scene setup unchanged')


def ground_dynamics_environment(status, output, replay_from):
    """Physical state is a diagnostic output, never an algorithm observation."""
    if status != 'DEVELOPMENT_BATCH':
        raise ValueError('physics tracing is confined to development runs')
    return {'P450_GROUND_DYNAMICS_CSV': str(output / 'ground-dynamics.csv')}


def integrated_feedback_environment(status):
    if status != 'DEVELOPMENT_BATCH':
        raise ValueError('integrated-pose velocity validation is development only')
    return {'P450_GROUND_INTEGRATED_VELOCITY': '1'}


def runtime_environment(sim_root, output):
    environment = os.environ.copy()
    environment.pop('P450_GROUND_DYNAMICS_CSV', None)
    environment.pop('P450_GROUND_INTEGRATED_VELOCITY', None)
    environment.setdefault('P450_PX4_ROOT', PX4)
    environment.update(SIM_ROOT=str(sim_root),
                       # This runner is single-host. Bind TCPROS locally so an
                       # unrelated old Gazebo cannot reconnect to a recycled
                       # ROS port via the host's LAN address (observed 33701).
                       ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1', ROS_IPV6='off',
                       ROS_MASTER_URI='http://127.0.0.1:11951', GAZEBO_MASTER_URI='http://127.0.0.1:11952',
                       ROS_LOG_DIR=str(output/'ros'), MPLCONFIGDIR='/tmp/a6-mpl', XDG_CACHE_HOME='/tmp/a6-cache')
    return environment


def navigation_outcome(events, adapter_exit):
    success = adapter_exit == 0 and any(e.get('state') == 'GROUND_STOPPED' for e in events)
    return dict(retrieval_success=None, navigation_success=success,
                classification_reason='navigation_only_retrieval_not_assessed')


def ground_replay_outcome(physical, events, adapter_exit, checker_exit, *, navigation_only=False,
                          conditioned_on_arrival=False):
    """Separate initialization from real Ground execution at its saved boundary."""
    started = any(e.get('state') == 'GROUND_REPLAY_START' for e in events)
    if not started:
        reason = next((e.get('reason') for e in events if e.get('state') in
                       ('GROUND_REPLAY_STARTUP_FAILED', 'FAILED') and e.get('reason')),
                      'ground_replay_not_started')
        return dict(status='INVALID_TRIAL', task_started=False, retrieval_success=None,
                    navigation_success=None, classification_reason=reason)
    if navigation_only:
        return dict(status='VALID_TRIAL', task_started=True, **navigation_outcome(events, adapter_exit))
    status, success, reason = classify_outcome(physical, events, adapter_exit, checker_exit)
    return dict(status=status, task_started=True, retrieval_success=success,
                navigation_success=None if conditioned_on_arrival else
                                   any(e.get('state') == 'GROUND_STOPPED' for e in events),
                classification_reason=reason)


def ground_candidate_setup(scene, selected):
    """Camera/manipulation diagnostic conditioned on arrival; target unchanged."""
    return dict(scene, bunker_xy=[selected['x'], selected['y']], bunker_yaw=selected['yaw'])


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
    choice.add_argument('--setup-scene', help='Explicit scene ID from the supplied configuration')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--sim-root', type=Path, default=SIM)
    parser.add_argument('--rm4d-root', type=Path, default=RM)
    parser.add_argument('--setup-view', type=float, nargs=4)
    parser.add_argument('--ground-replay-from', type=Path)
    parser.add_argument('--ground-navigation-only', action='store_true')
    parser.add_argument('--ground-at-candidate', action='store_true')
    parser.add_argument('--diagnose-grasp-failure', action='store_true')
    parser.add_argument('--ground-dynamics', action='store_true')
    parser.add_argument('--post-failure-load-contrast', action='store_true')
    parser.add_argument('--ode-solver', choices=('quick', 'world'))
    parser.add_argument('--integrated-joint-velocity', action='store_true')
    parser.add_argument('--full-robot-manipulation', action='store_true')
    parser.add_argument('--execution-clearance', action='store_true')
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    if args.ode_solver is not None and config['status'] != 'DEVELOPMENT_BATCH':
        raise ValueError('ODE solver contrast is development only')
    if args.integrated_joint_velocity:
        integrated_feedback_environment(config['status'])
    if args.full_robot_manipulation and config['status'] != 'DEVELOPMENT_BATCH':
        raise ValueError('full-robot manipulation validation is development only')
    if args.execution_clearance and (config['status'] != 'DEVELOPMENT_BATCH' or
                                     not args.full_robot_manipulation):
        raise ValueError('execution clearance requires full-robot development mode')
    if args.setup_scene:
        scene = next(s for s in config['scenes'] if s['id'] == args.setup_scene)
        slot = dict(scene=scene['id'], method='SETUP_CHECK', slot=None)
        if args.setup_view: config['initial_view'] = args.setup_view
    else:
        attempt_kind(config['status'])
        if args.setup_view: raise ValueError('method pose overrides are not allowed')
        slot, scene = slot_spec(config, args.slot)
    if args.ground_navigation_only and args.ground_replay_from is None:
        raise ValueError('navigation-only requires an archived confirmed Ground replay')
    if args.ground_at_candidate and (args.ground_replay_from is None or args.ground_navigation_only):
        raise ValueError('camera-only candidate setup requires a Ground replay, not a navigation test')
    if args.diagnose_grasp_failure and args.ground_replay_from is None:
        raise ValueError('post-failure IK probe is confined to Ground development diagnostics')
    if args.post_failure_load_contrast and (not args.ground_dynamics or args.ground_navigation_only
                                            or args.ground_replay_from is None):
        raise ValueError('load contrast requires a traced Ground manipulation replay')
    if args.ground_dynamics:
        ground_dynamics_environment(config['status'], args.output_dir, args.ground_replay_from)
    launch_scene = scene
    if args.ground_replay_from is not None:
        if args.setup_scene or config['status'] != 'DEVELOPMENT_BATCH':
            raise ValueError('Ground replay is a development diagnostic only')
        validate_replay_scene(json.loads((args.ground_replay_from/'attempt.json').read_text()), scene)
        from run_ground_sim import extract_recorded_handoff
        selected, _target = extract_recorded_handoff([json.loads(line) for line in
                                 (args.ground_replay_from/'data/events.jsonl').read_text().splitlines()])
        if args.ground_at_candidate:
            launch_scene = ground_candidate_setup(scene, selected)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    environment = runtime_environment(args.sim_root, output)
    if args.integrated_joint_velocity:
        environment.update(integrated_feedback_environment(config['status']))
    if args.ground_dynamics:
        environment.update(ground_dynamics_environment(config['status'], output, args.ground_replay_from))
    os.environ.update(environment)
    children, logs = [], []
    expected_kind = (attempt_kind(config['status']) if not args.setup_scene
                     else 'METHOD_INDEPENDENT_SETUP')
    record = dict(**slot, seed=scene['seed'], scene_spec=scene, initial_view=config['initial_view'],
                  uav_launch_pose=config['uav_launch_pose'],
                  config_path=str(args.config.resolve()), protocol=config['protocol'],
                  operational_gating=config.get('operational_gating', 'v1'),
                  handoff_stop=config.get('handoff_stop', 'legacy'),
                  kind='METHOD_INDEPENDENT_SETUP' if args.setup_scene else expected_kind,
                  status='INVALID_TRIAL', task_started=False, activation_wall=time.time())
    if args.ground_dynamics:
        record.update(ground_dynamics_csv=environment['P450_GROUND_DYNAMICS_CSV'],
                      physics_state_use='diagnosis_only_never_algorithm_input',
                      post_failure_load_contrast=args.post_failure_load_contrast)
    record['joint_velocity_feedback'] = ('integrated_pose_interval_velocity' if args.integrated_joint_velocity
                                         else 'native_ode_rate')
    record['full_robot_manipulation'] = args.full_robot_manipulation
    record['execution_clearance'] = 'chassis-clearance-v1' if args.execution_clearance else None
    if args.ground_replay_from is not None:
        record.update(kind='GROUND_NAVIGATION_DIAGNOSTIC' if args.ground_navigation_only else
                      'GROUND_SEGMENT_DIAGNOSTIC', ground_replay_from=str(args.ground_replay_from.resolve()),
                      conditioned_on_archived_confirmation=True)
        if args.ground_at_candidate:
            record.update(kind='GROUND_CAMERA_MANIPULATION_DIAGNOSTIC',
                          conditioned_on_arrival=True, launch_scene_spec=launch_scene)
    def save(): (output/'attempt.json').write_text(json.dumps(record, indent=2, allow_nan=False)+'\n')
    def start(command, name):
        log = (output/(name+'.log')).open('w'); logs.append(log)
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=environment,
                                 cwd=str(ROOT), start_new_session=True)
        children.append(child)
        return child
    runtime = checker = adapter = recorder = image_recorder = None
    image_command = image_diagnostic_command(config, output)
    if image_command is not None:
        record.update(diagnostic_image_scope=config['diagnostic_image_scope'],
                      diagnostic_image_command=image_command,
                      diagnostic_image_path=str(output/'diagnostics-images.bag'),
                      diagnostic_image_start_trigger=None)
    def start_images(event):
        nonlocal image_recorder
        record['diagnostic_image_start_trigger'] = {
            key: event.get(key) for key in ('state', 'ros_time', 'wall_monotonic')}
        try:
            image_recorder = start(image_command, 'diagnostics-images')
        except Exception as error:
            record['diagnostic_image_error'] = '%s: %s' % (type(error).__name__, error)
        save()
    save()
    try:
        source = args.sim_root/'src/demos/air_ground_pick_demo/launch/air_ground_pick_demo.launch'
        launch_file = output/'runtime.launch'
        world = None
        if args.ode_solver is not None:
            world_source = args.sim_root/'src/platform/sim_platform_bringup/worlds/air_ground_v1.world'
            world = output/'solver-contrast.world'
            world.write_text(solver_contrast_world(world_source.read_text(), args.ode_solver, config['status']))
            record.update(ode_solver=args.ode_solver, physics_contrast_world=str(world))
            save()
        launch_file.write_text(runtime_launch(source.read_text(), config['uav_launch_pose'], world))
        runtime = start(['roslaunch', str(launch_file), 'gui:=false', 'run_demo:=false',
                         'enable_mid360:=true', 'px4_workdir:=sitl_a6_%d_%d' % (time.time_ns(), os.getpid()),
                         *scene_launch_args(launch_scene)], 'runtime')
        print('START', slot, output, flush=True)
        record['ready_sim'] = prepare_scene(scene, runtime)
        print('READY', slot, record['ready_sim'], flush=True)
        record_command = diagnostic_command(config, output)
        if record_command is not None and args.ground_dynamics:
            record_command += ['/gazebo/model_states', '/gazebo/link_states',
                               '/ground/gripper_controller/state',
                               '/ground/gripper_controller/follow_joint_trajectory/goal',
                               '/ground/gripper_controller/follow_joint_trajectory/result']  # diagnostic recorder only
        if record_command is not None:
            record['diagnostic_command'] = record_command
            recorder = start(record_command, 'diagnostics')
        common = adapter_args(config, output/'data', args.sim_root, args.rm4d_root)
        if args.full_robot_manipulation:
            common += ['--full-robot-manipulation']
        if args.execution_clearance:
            common += ['--execution-clearance']
        if args.setup_scene:
            command = ['/usr/bin/python3', str(ROOT/'scripts/a6_setup_check.py'), *common,
                       '--scene-file', str(args.config.resolve()), '--scene-id', scene['id']]
        elif args.ground_replay_from is not None:
            command = ['/usr/bin/python3', str(ROOT/'scripts/run_ground_sim.py'), *common,
                       '--method', slot['method'], '--ground-replay-from', str(args.ground_replay_from.resolve())]
            command += (['--ground-navigation-only'] if args.ground_navigation_only else
                        ['--wait-for-status-subscriber'])
            if args.diagnose_grasp_failure: command += ['--diagnose-grasp-failure']
            if args.post_failure_load_contrast: command += ['--post-failure-load-contrast']
        else:
            command = ['/usr/bin/python3', str(ROOT/'scripts/run_a6_sim.py'), *common,
                       '--method', slot['method'], '--wait-for-status-subscriber']
        task_deadline = time.monotonic()+config['task_wall_guard_s']
        adapter = start(command, 'adapter')
        if not args.setup_scene and not args.ground_navigation_only:
            wait_for_adapter_ready(adapter, output/'adapter.log', task_deadline)
            checker = start(['/usr/bin/python3', str(args.sim_root/'scripts/check_air_ground_pick_demo.py'),
                             '--summary', str(output/'physical_summary.json'), '--timeout', '1250',
                             '--maximum-ground-travel', '3.0'] +
                            (['--ground-only'] if args.ground_replay_from else []), 'physical_checker')
        record.update(task_started=args.ground_replay_from is None,
                      status='VALID_TRIAL' if args.ground_replay_from is None else 'RUNNING'); save()
        record['adapter_exit'] = wait_for_adapter(
            adapter, task_deadline, output/'data/events.jsonl',
            start_images if image_command is not None else None)
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
        if image_command is not None:
            record['diagnostic_image_exit'] = None if image_recorder is None else image_recorder.poll()
            record['diagnostic_image_bag_finalized'] = (output/'diagnostics-images.bag').is_file()
        physical, events, errors = read_measurements(output)
        if errors: record['measurement_read_errors'] = errors
        if physical is not None: record['physical_status'] = physical.get('status')
        if args.ground_replay_from is not None:
            record.update(ground_replay_outcome(
                physical, events, record.get('adapter_exit'), record.get('checker_exit'),
                navigation_only=args.ground_navigation_only, conditioned_on_arrival=args.ground_at_candidate))
        elif record['task_started'] and not args.setup_scene:
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
        if args.ground_replay_from is not None:
            from run_ground_sim import materialize_ground_metrics
            try:
                materialize_ground_metrics(output)
            except (OSError, ValueError, KeyError, TypeError) as error:
                record['metrics_error'] = '%s: %s' % (type(error).__name__, error)
                save()
    print('DONE', json.dumps(record), flush=True)
    return 0


if __name__ == '__main__': raise SystemExit(main())
