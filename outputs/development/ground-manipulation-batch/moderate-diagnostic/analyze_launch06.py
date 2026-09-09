#!/usr/bin/env python3
"""Offline public-bag/model audit. Prints JSON; never calls ROS services/nodes.

Inputs are accepted perception events, JointState, arm/gripper action traces,
public TF, and the existing rendered collision model. No simulator state topics,
spawn target coordinates, contact-plugin output, or physical-checker data.
"""
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
AGENT = HERE.parents[3]
RUN = HERE.parent / 'launch-06-moderate-full-robot'
SIM = Path('/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation')
spec = importlib.util.spec_from_file_location('geometry_audit', HERE / 'analyze_geometry.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)
from air_ground_pick_demo.grasp import conservative_jaw_opening, generate_top_down_grasp, quaternion_matrix
from ground_manipulator_runtime.renderer import render_ground_robot
from tf.transformations import quaternion_from_matrix
import rosbag


def transform(position, quaternion):
    result = np.eye(4)
    result[:3, 3] = position
    result[:3, :3] = quaternion_matrix(quaternion)
    return result


def main():
    events = [json.loads(line) for line in (RUN / 'data/events.jsonl').read_text().splitlines()]
    selected = next(e for e in events if e['state'] == 'A5_SELECTED')
    refined = next(e for e in events if e['state'] == 'GROUND_REFINED')
    preshape = next(e for e in events if e['state'] == 'GROUND_GRIPPER_PRESHAPE')
    failed = next(e for e in events if e['state'] == 'FAILED')
    object_update = next(e for e in events if e['state'] == 'GROUND_MANIPULATION_SCENE' and e['source'] == 'accepted_refined')
    plan = [e for e in events if e['state'] == 'GROUND_MANIPULATION_PLAN']
    cutoffs = {'preshape': preshape['ros_time'], 'failed': failed['ros_time']}
    samples, controllers, goals, results, tf_edges = {}, {}, [], [], {}
    topics = ['/ground/joint_states', '/ground/arm_controller/state',
              '/ground/gripper_controller/state', '/ground/arm_controller/follow_joint_trajectory/goal',
              '/ground/arm_controller/follow_joint_trajectory/result', '/tf', '/tf_static']
    with rosbag.Bag(str(RUN / 'diagnostics.bag'), 'r') as bag:
        for topic, message, record_time in bag.read_messages(topics=topics):
            if topic in ('/tf', '/tf_static'):
                for t in message.transforms:
                    if t.header.stamp.to_sec() <= failed['ros_time']:
                        p, q = t.transform.translation, t.transform.rotation
                        tf_edges[t.child_frame_id] = (t.header.frame_id, transform(
                            (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)), t.header.stamp.to_sec())
                continue
            stamp = message.header.stamp.to_sec()
            if topic.endswith('/goal'):
                trajectory = message.goal.trajectory
                goals.append({'stamp': stamp, 'trajectory_stamp': trajectory.header.stamp.to_sec(),
                              'duration': trajectory.points[-1].time_from_start.to_sec(),
                              'joint_names': list(trajectory.joint_names),
                              'endpoint': list(trajectory.points[-1].positions)})
            elif topic.endswith('/result'):
                results.append({'stamp': stamp, 'status': message.status.status,
                                'error_code': message.result.error_code,
                                'error_string': message.result.error_string})
            elif topic == '/ground/joint_states':
                for label, cutoff in cutoffs.items():
                    if stamp <= cutoff:
                        samples[label] = {'stamp': stamp, 'positions': dict(zip(message.name, message.position))}
            elif stamp <= failed['ros_time']:
                controllers[topic] = {'stamp': stamp, 'joint_names': list(message.joint_names),
                                     'actual_position': list(message.actual.positions),
                                     'desired_position': list(message.desired.positions),
                                     'position_error': list(message.error.positions)}
    robot = ET.fromstring(render_ground_robot(SIM / 'src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro'))
    links = {l.get('name'): l for l in robot.findall('link')}
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    box = links['ground/base_link'].find('collision')
    center = geometry.origin(box.find('origin'))[:3, 3]
    half = np.fromstring(box.find('geometry/box').get('size'), sep=' ')/2
    lower, upper = center-half, center+half
    target = refined['target_map']
    target_map = transform(target[:3], (0., 0., math.sin(target[3]/2), math.cos(target[3]/2)))
    declared = object_update['target_pose']
    base_from_map = transform(declared[:3], declared[3:]) @ np.linalg.inv(target_map)
    generated = generate_top_down_grasp(target, [.24, .053, .115], .15, .15, .0156, .020, .010)
    required = base_from_map @ transform(generated.pregrasp.position, generated.pregrasp.orientation)
    endpoint = next(goal for goal in reversed(goals) if goal['stamp'] <= failed['ros_time'])
    planned = dict(samples['preshape']['positions'])
    planned.update(zip(endpoint['joint_names'], endpoint['endpoint']))
    planned_actual_gripper = dict(planned)
    planned_actual_gripper['left_outer_knuckle_joint'] = samples['failed']['positions']['left_outer_knuckle_joint']
    actual_preshape_gripper = dict(samples['failed']['positions'])
    actual_preshape_gripper['left_outer_knuckle_joint'] = samples['preshape']['positions']['left_outer_knuckle_joint']
    cases = {'executed_feedback': samples['failed']['positions'], 'planned_endpoint': planned,
             'planned_arm_actual_gripper': planned_actual_gripper,
             'actual_arm_preshape_gripper': actual_preshape_gripper,
             'public_tf_meshes': samples['failed']['positions']}
    def tf_chain(frame):
        if frame == 'ground/base_link':
            return np.eye(4)
        parent, value, stamp = tf_edges[frame]
        return tf_chain(parent) @ value
    output = {'run': str(RUN), 'candidate': {k: selected[k] for k in ('source_id', 'x', 'y', 'yaw')},
              'accepted_refined': refined, 'preshape': preshape, 'failure': failed,
              'planning_outcomes': plan, 'joint_samples': samples,
              'controller_samples_at_failure': controllers, 'arm_goals': goals, 'arm_results': results,
              'T_base_map_from_perceived_scene': base_from_map.tolist(),
              'required_pregrasp_T_base_tcp': required.tolist(),
              'chassis_box_base_minmax_m': [lower.tolist(), upper.tolist()], 'cases': {}}
    for label, positions in cases.items():
        cache = {'ground/base_link': np.eye(4)}
        def fk(link):
            if label == 'public_tf_meshes':
                return tf_chain(link)
            if link in cache:
                return cache[link]
            joint = joints[link]
            value = fk(joint.find('parent').get('link')) @ geometry.origin(joint.find('origin'))
            if joint.get('type') in ('revolute', 'continuous'):
                mimic = joint.find('mimic')
                angle = positions[joint.get('name')] if mimic is None else (
                    positions[mimic.get('joint')]*float(mimic.get('multiplier', 1)) + float(mimic.get('offset', 0)))
                motion = np.eye(4)
                motion[:3, :3] = geometry.axis_rotation(np.fromstring(joint.find('axis').get('xyz'), sep=' '), angle)
                value = value @ motion
            cache[link] = value
            return value
        actual = fk('ground/gripper_tcp_link')
        rotation_error = actual[:3, :3] @ required[:3, :3].T
        detail = {'gripper_joint': positions['left_outer_knuckle_joint'], 'T_base_tcp': actual.tolist(),
                  'tcp_quaternion_xyzw_base': quaternion_from_matrix(actual).tolist(),
                  'tcp_minus_required_base_xyz_m': (actual[:3, 3]-required[:3, 3]).tolist(),
                  'tcp_position_error_norm_m': float(np.linalg.norm(actual[:3, 3]-required[:3, 3])),
                  'tcp_rotation_error_rad': math.acos(float(np.clip((np.trace(rotation_error)-1)/2, -1, 1))),
                  'collision_meshes': {}}
        for link_name in ('ground/left_finger', 'ground/left_outer_knuckle'):
            collision = links[link_name].find('collision')
            mesh = collision.find('geometry/mesh')
            package, relative = mesh.get('filename')[len('package://'):].split('/', 1)
            mesh_path = next(SIM.glob('src/**/' + package + '/package.xml')).parent / relative
            raw = mesh_path.read_bytes()
            dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
            vertices = np.frombuffer(raw, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(float)
            vertices *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            frame = fk(link_name) @ geometry.origin(collision.find('origin'))
            vertices = vertices @ frame[:3, :3].T + frame[:3, 3]
            inside = np.all((vertices > lower+1e-9) & (vertices < upper-1e-9), axis=1)
            entry = {'inside_vertex_count': int(inside.sum()),
                     'collision_origin_T_base': frame.tolist(),
                     'triangle_box_intersections': int(geometry.triangle_box_hits(vertices, center, half).sum()),
                     'maximum_axis_gap_m': float(np.maximum(lower-vertices.max(axis=0), vertices.min(axis=0)-upper).max())}
            if inside.any():
                depth = np.minimum(vertices[inside]-lower, upper-vertices[inside]).min(axis=1)
                index = int(np.argmax(depth))
                entry['witness_xyz_base_m'] = vertices[inside][index].tolist()
                entry['witness_inside_depth_m'] = float(depth[index])
            detail['collision_meshes'][link_name] = entry
        output['cases'][label] = detail
    output['public_tf_T_base_tcp'] = tf_chain('ground/gripper_tcp_link').tolist()
    output['public_tf_edge_stamps'] = {key: value[2] for key, value in tf_edges.items()
                                      if key.startswith('ground/')}
    planned_tcp = np.array(output['cases']['planned_endpoint']['T_base_tcp'])
    measured_tcp = np.array(output['cases']['executed_feedback']['T_base_tcp'])
    public_tcp = np.array(output['public_tf_T_base_tcp'])
    def compare_tcp(first, second):
        delta = first[:3, 3]-second[:3, 3]
        rotation = first[:3, :3] @ second[:3, :3].T
        return {'first_minus_second_xyz_m': delta.tolist(),
                'position_error_norm_m': float(np.linalg.norm(delta)),
                'orientation_error_rad': math.acos(float(np.clip((np.trace(rotation)-1)/2, -1, 1)))}
    output['measured_minus_planned_tcp'] = compare_tcp(measured_tcp, planned_tcp)
    output['public_tf_minus_planned_tcp'] = compare_tcp(public_tcp, planned_tcp)
    output['public_tf_minus_feedback_fk_tcp'] = compare_tcp(public_tcp, measured_tcp)
    output['failure_conservative_aperture_m'] = conservative_jaw_opening(
        samples['failed']['positions']['left_outer_knuckle_joint'], .0952, .93)
    output['scope'] = {
        'inputs': 'accepted perception events, public JointState/controller/action/TF topics, existing rendered URDF and STL collision meshes',
        'no_gt': True, 'no_runtime_calls': True,
        'mesh_test': 'strict vertex-inside-box plus triangle-box separating-axis test; not AABB-overlap inference',
        'first_collision': 'first recorded full-state validity failure, not proof of first physical contact time',
        'planned_gripper': 'recorded preshape feedback; exact planning-service request state was not recorded',
        'unverified': 'physical contact force, all arm branches, attached payload and complete lift path'}
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
