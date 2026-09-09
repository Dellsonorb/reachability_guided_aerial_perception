#!/usr/bin/env python3
"""Offline accepted-target/public-feedback AG95 closure geometry audit.

No ROS node/service, simulator-state topic, GT target, or production edit.
Prints JSON. Finite sweep checks are not continuous/full-arm certificates.
"""
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('launch06_audit', HERE / 'analyze_launch06.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)
geometry, transform, SIM = audit.geometry, audit.transform, audit.SIM
import rosbag
from air_ground_pick_demo.grasp import generate_top_down_grasp
from ground_manipulator_runtime.renderer import render_ground_robot

RUN = HERE.parent / 'launch-07-easy-full-e2e'


def main():
    events = [json.loads(line) for line in (RUN / 'data/events.jsonl').read_text().splitlines()]
    refined = next(e for e in events if e['state'] == 'GROUND_REFINED')
    scene_event = next(e for e in events if e['state'] == 'GROUND_MANIPULATION_SCENE' and e['source'] == 'accepted_refined')
    preshape = next(e for e in events if e['state'] == 'GROUND_GRIPPER_PRESHAPE')
    failed = next(e for e in events if e['state'] == 'FAILED')
    close = next(e for e in events if e['state'] == 'A6_STAGE_START' and e['stage'] == 'close')
    cutoffs = {'close_start': close['ros_time'], 'failure': failed['ros_time']}
    samples, goals, gripper_goals, edges = {}, [], [], {}
    topics = ['/ground/joint_states', '/tf', '/tf_static',
              '/ground/arm_controller/follow_joint_trajectory/goal',
              '/ground/gripper_controller/follow_joint_trajectory/goal']
    with rosbag.Bag(str(RUN / 'diagnostics.bag'), 'r') as bag:
        for topic, msg, _ in bag.read_messages(topics=topics):
            if topic in ('/tf', '/tf_static'):
                for t in msg.transforms:
                    if t.header.stamp.to_sec() <= failed['ros_time']:
                        p, q = t.transform.translation, t.transform.rotation
                        edges[t.child_frame_id] = (t.header.frame_id, transform(
                            (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)), t.header.stamp.to_sec())
            elif topic == '/ground/joint_states':
                for name, cutoff in cutoffs.items():
                    if msg.header.stamp.to_sec() <= cutoff:
                        samples[name] = {'stamp': msg.header.stamp.to_sec(),
                                         'positions': dict(zip(msg.name, msg.position))}
            else:
                entry = {'stamp': msg.header.stamp.to_sec(),
                         'names': list(msg.goal.trajectory.joint_names),
                         'endpoint': list(msg.goal.trajectory.points[-1].positions)}
                (goals if '/arm_controller/' in topic else gripper_goals).append(entry)
    robot = ET.fromstring(render_ground_robot(SIM / 'src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro'))
    links = {l.get('name'): l for l in robot.findall('link')}
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    box = links['ground/base_link'].find('collision')
    base_center = geometry.origin(box.find('origin'))[:3, 3]
    base_half = np.fromstring(box.find('geometry/box').get('size'), sep=' ')/2
    meshes = {}
    for name, link in links.items():
        if not any(word in name for word in ('ag95', 'finger', 'knuckle')):
            continue
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                continue
            package, relative = mesh.get('filename')[len('package://'):].split('/', 1)
            path = next(SIM.glob('src/**/' + package + '/package.xml')).parent / relative
            raw = path.read_bytes()
            dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
            points = np.frombuffer(raw, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(float)
            points *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            meshes[name] = (points, geometry.origin(collision.find('origin')))

    def model_fk(link, positions, cache):
        if link in cache:
            return cache[link]
        joint = joints[link]
        matrix = model_fk(joint.find('parent').get('link'), positions, cache) @ geometry.origin(joint.find('origin'))
        if joint.get('type') in ('continuous', 'revolute'):
            mimic = joint.find('mimic')
            q = positions[joint.get('name')] if mimic is None else positions[mimic.get('joint')]*float(mimic.get('multiplier', 1))+float(mimic.get('offset', 0))
            motion = np.eye(4)
            motion[:3, :3] = geometry.axis_rotation(np.fromstring(joint.find('axis').get('xyz'), sep=' '), q)
            matrix = matrix @ motion
        cache[link] = matrix
        return matrix

    def public_tf(link):
        if link == 'ground/base_link':
            return np.eye(4)
        parent, value, stamp = edges[link]
        return public_tf(parent) @ value

    target = refined['target_map']
    target_map = transform(target[:3], (0, 0, math.sin(target[3]/2), math.cos(target[3]/2)))
    scene_pose = scene_event['target_pose']
    target_base = transform(scene_pose[:3], scene_pose[3:])
    base_map = target_base @ np.linalg.inv(target_map)
    target_half = np.array(scene_event['target_size'])/2
    grasp = generate_top_down_grasp(target, scene_event['target_size'], .15, .15, .0156, .020, .010).grasp
    nominal = base_map @ transform(grasp.position, grasp.orientation)
    alpha = math.radians(44.691)
    contact_q = math.acos(math.cos(alpha)+(.053-.0952)/.110)-alpha
    rise = .055*(math.sin(alpha+contact_q)-math.sin(alpha))
    measured = {name: model_fk('ground/gripper_tcp_link', sample['positions'], {'ground/base_link': np.eye(4)})
                for name, sample in samples.items()}
    last_goal = goals[-1]
    planned_positions = dict(samples['close_start']['positions'])
    planned_positions.update(zip(last_goal['names'], last_goal['endpoint']))
    planned = model_fk('ground/gripper_tcp_link', planned_positions, {'ground/base_link': np.eye(4)})
    cases = {'nominal_old': nominal, 'measured_close_start_old': measured['close_start'],
             'measured_failure_old': measured['failure'], 'public_tf_old': public_tf('ground/gripper_tcp_link')}
    for name in ('nominal', 'measured_close_start', 'measured_failure', 'public_tf'):
        corrected = cases[name+'_old'].copy()
        corrected[2, 3] += rise
        cases[name+'_contact_height'] = corrected
    start_q = samples['failure']['positions']['left_outer_knuckle_joint']
    # Match existing runtime step and independently inspect a finer, fixed 0.5-mm
    # maximum arc-displacement grid. Neither grid certifies continuous clearance.
    runtime_intervals = max(1, math.ceil(abs(contact_q-start_q)*.110/.005))
    dense_intervals = max(1, math.ceil(abs(contact_q-start_q)*.110/.0005))
    q_values = sorted(set(np.linspace(start_q, contact_q, runtime_intervals+1).tolist()+
                          np.linspace(start_q, contact_q, dense_intervals+1).tolist()))
    allowed = {'ground/left_finger', 'ground/right_finger', 'ground/left_finger_pad', 'ground/right_finger_pad'}

    def box_check(points, center, half):
        lower, upper = center-half, center+half
        axis_gaps = np.maximum(lower-points.max(axis=0), points.min(axis=0)-upper)
        result = {'axis_gaps_m': axis_gaps.tolist(), 'triangle_intersections': 0, 'inside_vertices': 0}
        if axis_gaps.max() > 1e-10:
            return result
        hits = geometry.triangle_box_hits(points, center, half)
        inside = np.all((points > lower+1e-9) & (points < upper-1e-9), axis=1)
        result.update(triangle_intersections=int(hits.sum()), inside_vertices=int(inside.sum()))
        if inside.any():
            depth = np.minimum(points[inside]-lower, upper-points[inside]).min(axis=1)
            idx = int(np.argmax(depth))
            result.update(witness=points[inside][idx].tolist(), depth_m=float(depth[idx]))
        if hits.any():
            result['triangle_witness'] = points.reshape(-1, 3, 3)[hits][0].tolist()
        return result

    output = {'run': str(RUN), 'accepted_refined': refined, 'accepted_scene': scene_event,
              'preshape': preshape, 'close': close, 'failure': failed, 'joint_samples': samples,
              'arm_goals': goals, 'gripper_goals': gripper_goals,
              'planned_descend_T_base_tcp': planned.tolist(),
              'T_base_target': target_base.tolist(), 'nominal_old_T_base_tcp': nominal.tolist(),
              'contact_q_rad': contact_q, 'geometry_derived_tcp_rise_m': rise,
              'existing_pad_edge_m': .0156, 'contact_consistent_pad_edge_m': .0156-rise,
              'runtime_grid': np.linspace(start_q, contact_q, runtime_intervals+1).tolist(),
              'q_grid': q_values, 'cases': {},
              'public_tf_edge_stamps': {name: edge[2] for name, edge in edges.items()
                                        if any(part in name for part in ('finger', 'knuckle', 'wrist', 'forearm', 'shoulder', 'upper_arm', 'tcp'))},
              'scope': 'No GT/no runtime. Gripper-target/chassis sampled closure audit, not full arm IK/path or continuous clearance proof.'}
    for case, tcp in cases.items():
        detail = {'T_base_tcp': tcp.tolist(), 'forbidden_target_hits': [], 'chassis_hits': [], 'contact_pads': {},
                  'intended_target_first_hits': {},
                  'minimum_noncontact_target_axis_gap_m': None}
        delta = tcp[:3, 3]-nominal[:3, 3]
        delta_rotation = tcp[:3, :3] @ nominal[:3, :3].T
        detail['tcp_minus_nominal_base_xyz_m'] = delta.tolist()
        detail['tcp_minus_nominal_target_xyz_m'] = (target_base[:3, :3].T @ delta).tolist()
        detail['orientation_error_rad'] = math.acos(float(np.clip((np.trace(delta_rotation)-1)/2, -1, 1)))
        min_noncontact = math.inf
        for q in q_values:
            positions = dict(samples['failure']['positions'])
            positions['left_outer_knuckle_joint'] = q
            ee = tcp @ np.linalg.inv(geometry.origin(joints['ground/gripper_tcp_link'].find('origin')))
            cache = {'ground/ee_link': ee}
            for name, (raw_points, collision_origin) in meshes.items():
                matrix = model_fk(name, positions, cache) @ collision_origin
                base_points = raw_points @ matrix[:3, :3].T+matrix[:3, 3]
                inv_target = np.linalg.inv(target_base)
                target_points = base_points @ inv_target[:3, :3].T+inv_target[:3, 3]
                t = box_check(target_points, np.zeros(3), target_half)
                b = box_check(base_points, base_center, base_half)
                if name not in allowed:
                    min_noncontact = min(min_noncontact, max(t['axis_gaps_m']))
                    if t['triangle_intersections']:
                        detail['forbidden_target_hits'].append(dict(link=name, q=q, **t))
                elif t['triangle_intersections'] and name not in detail['intended_target_first_hits']:
                    detail['intended_target_first_hits'][name] = dict(q=q, **t)
                if b['triangle_intersections']:
                    detail['chassis_hits'].append(dict(link=name, q=q, **b))
                if 'finger_pad' in name and q == q_values[-1]:
                    detail['contact_pads'][name] = {
                        'target_z_minmax_m': [float(target_points[:, 2].min()), float(target_points[:, 2].max())],
                        'target_y_minmax_m': [float(target_points[:, 1].min()), float(target_points[:, 1].max())],
                        'vertical_overlap_m': float(min(target_points[:, 2].max(), target_half[2])-max(target_points[:, 2].min(), -target_half[2])),
                        'triangle_intersections': t['triangle_intersections']}
        detail['minimum_noncontact_target_axis_gap_m'] = min_noncontact
        detail['forbidden_links'] = sorted(set(h['link'] for h in detail['forbidden_target_hits']))
        detail['forbidden_collision_sample_count'] = len(set(h['q'] for h in detail['forbidden_target_hits']))
        output['cases'][case] = detail
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
