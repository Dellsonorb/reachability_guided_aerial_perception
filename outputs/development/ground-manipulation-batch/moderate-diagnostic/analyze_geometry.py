#!/usr/bin/env python3
"""Offline URDF/mesh geometry witness, not an IK or collision-free certificate.

Read public archived accepted target/base/joint data and current production URDF.
Print JSON only. No ROS node, ROS service, Bullet, Gazebo, or artifact writes.
"""
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

AGENT = Path(__file__).resolve().parents[4]
SIM = Path('/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation')
os.environ['ROS_PACKAGE_PATH'] = str(SIM / 'src') + ':/opt/ros/noetic/share'
for package in ('platform/ground_manipulator_runtime', 'demos/air_ground_pick_demo'):
    sys.path.insert(0, str(SIM / 'src' / package / 'src'))
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
from ground_manipulator_runtime.renderer import render_ground_robot
from air_ground_pick_demo.grasp import generate_top_down_grasp, quaternion_matrix
import rosbag


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        transform[:3, :3] = rpy_matrix(np.fromstring(element.get('rpy', '0 0 0'), sep=' '))
    return transform


def axis_rotation(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew)


def triangle_box_hits(points, center, half_extent):
    """Exact triangle/AABB separating-axis test; no AABB-overlap inference."""
    triangles = points.reshape(-1, 3, 3)-center
    edges = np.stack([triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 1],
                      triangles[:, 0]-triangles[:, 2]], axis=1)
    normals = np.cross(edges[:, 0], edges[:, 1])[:, None, :]
    basis = np.broadcast_to(np.eye(3), (len(triangles), 3, 3))
    edge_axes = np.cross(edges[:, :, None, :], np.eye(3)[None, None, :, :]).reshape(-1, 9, 3)
    axes = np.concatenate([basis, normals, edge_axes], axis=1)
    projections = np.einsum('nvc,nac->nva', triangles, axes)
    radius = np.sum(abs(axes)*half_extent, axis=2)
    return np.all((projections.max(axis=1) >= -radius-1e-12) &
                  (projections.min(axis=1) <= radius+1e-12), axis=1)


def main():
    assert triangle_box_hits(np.array([[-2., 0, 0], [2., 0, 0], [0, 2., 0]]), np.zeros(3), np.ones(3)).tolist() == [True]
    assert triangle_box_hits(np.array([[-2., 0, 2], [2., 0, 2], [0, 2., 2]]), np.zeros(3), np.ones(3)).tolist() == [False]
    archived = AGENT / 'outputs/development/ground-execution-batch/launch-08-moderate-grasp-diagnostic'
    events = [json.loads(line) for line in (archived / 'data/events.jsonl').read_text().splitlines()]
    refined = next(e for e in events if e['state'] == 'GROUND_REFINED')
    selected = next(e for e in events if e['state'] == 'A5_SELECTED')
    arrival = next(e for e in events if e['state'] == 'GROUND_ARRIVAL_MEASURED')
    measured = None
    with rosbag.Bag(str(archived / 'diagnostics.bag'), 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=['/ground/joint_states']):
            if msg.header.stamp.to_sec() > refined['ros_time']:
                break
            measured = msg
    assert measured is not None
    master_q = dict(zip(measured.name, measured.position))['left_outer_knuckle_joint']
    robot = ET.fromstring(render_ground_robot(SIM / 'src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro'))
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    base_collision = links['ground/base_link'].find('collision')
    center = origin(base_collision.find('origin'))[:3, 3]
    size = np.fromstring(base_collision.find('geometry/box').get('size'), sep=' ')
    lower, upper = center-size/2, center+size/2
    # Public nominal localization base height comes from archived frame calibration.
    initial = json.loads((AGENT / 'outputs/development/operational-batch/launch-09-moderate-generic/data/initial.json').read_text())
    base = np.eye(4)
    base[:3, :3] = rpy_matrix([0, 0, selected['yaw']])
    base[:3, 3] = [selected['x'], selected['y'], initial['frame_calibration']['ground_reference_height_m']]
    target = refined['target_map']
    result = {'source': str(archived), 'target_source': 'GROUND_REFINED', 'target_map': target,
              'candidate_source_id': selected['source_id'],
              'T_map_candidate_base': base.tolist(), 'measured_arrival_map': arrival['actual_pose_map'],
              'joint_header_s': measured.header.stamp.to_sec(), 'measured_gripper_master_q': master_q,
              'base_collision_lower_upper_m': [lower.tolist(), upper.tolist()], 'branches': [],
              'limit': 'Vertices strictly inside rendered BUNKER collision box prove overlap; absence of a witness does not prove clearance. Arm IK/dynamics are not solved.'}
    target_base = np.linalg.inv(base) @ np.r_[target[:3], 1]
    result['target_base_xyz'] = target_base[:3].tolist()
    result['target_arm_xyz'] = (target_base[:3]-[.15, 0, .122]).tolist()
    target_frame = np.eye(4)
    target_frame[:3, :3] = rpy_matrix([0, 0, target[3]])
    target_frame[:3, 3] = target[:3]
    base_to_target = np.linalg.inv(target_frame) @ base
    corners = np.array([[x, y, z, 1.] for x in (-.12, .12) for y in (-.0265, .0265)
                        for z in (-.0575, .0575)])
    payload_base = corners @ (np.linalg.inv(base) @ target_frame).T
    result['payload_chassis_horizontal_separating_gap_m'] = float(np.maximum(
        lower[:2]-payload_base[:, :2].max(axis=0), payload_base[:, :2].min(axis=0)-upper[:2]).max())
    result['perceived_target_bottom_map_z_m'] = target[2]-.0575
    required_opening = .053 + .002
    preshape_q = .93 * (1. - required_opening / .0952)
    result['required_opening_m'] = required_opening
    result['inverse_conservative_opening_q'] = preshape_q
    result['inverse_calibration'] = 'q = .93 * (1 - (.053 + .002)/.0952); inverse of existing conservative_jaw_opening, not collision-fitted'
    # URDF outer->finger lever = 55 mm, outer origin angle = -44.691 deg;
    # equal/opposite mimic rotations keep finger/pad orientations parallel.
    alpha = math.radians(44.691)
    contact_q = math.acos(math.cos(alpha) + (.053-.0952)/(.055*2)) - alpha
    contact_edge = .0156 + .055*(math.sin(alpha)-math.sin(alpha+contact_q))
    result['mesh_kinematic_contact_prediction'] = {'pad_gap_contact_q': contact_q,
        'pad_edge_at_contact_m': contact_edge,
        'tcp_raise_to_preserve_existing_20mm_overlap_m': .0156-contact_edge,
        'caveat': 'Ideal URDF geometry only; actual confirmed closure joint must be measured.'}
    choices = [(yaw, name, q, edge) for yaw in (0., math.pi)
               for name, q, edge in [('measured_open', master_q, .0156),
                    ('required_opening_preshape', preshape_q, .0156),
                    ('predicted_pad_contact', contact_q, .0156)]]
    for yaw_offset, gripper_case, geometry_master_q, pad_edge in choices:
        changed = list(target)
        changed[3] += yaw_offset
        poses = generate_top_down_grasp(changed, [.24, .053, .115], .15, .15, pad_edge, .020, .010)
        for stage in ('pregrasp', 'grasp', 'lift'):
            pose = getattr(poses, stage)
            tcp = np.eye(4)
            tcp[:3, :3] = quaternion_matrix(pose.orientation)
            tcp[:3, 3] = pose.position
            ee = np.linalg.inv(base) @ tcp @ np.linalg.inv(origin(joints['ground/gripper_tcp_link'].find('origin')))
            cache = {'ground/ee_link': ee}

            def fk(link):
                if link in cache:
                    return cache[link]
                joint = joints[link]
                transform = fk(joint.find('parent').get('link')) @ origin(joint.find('origin'))
                if joint.get('type') in ('revolute', 'continuous'):
                    mimic = joint.find('mimic')
                    angle = geometry_master_q if mimic is None else geometry_master_q*float(mimic.get('multiplier', 1))+float(mimic.get('offset', 0))
                    motion = np.eye(4)
                    motion[:3, :3] = axis_rotation(np.fromstring(joint.find('axis').get('xyz'), sep=' '), angle)
                    transform = transform @ motion
                cache[link] = transform
                return transform

            branch = {'yaw_offset_rad': yaw_offset, 'stage': stage,
                      'gripper_case': gripper_case, 'geometry_master_q': geometry_master_q,
                      'lift_caveat': 'Lift-height gripper envelope at stated preshape, NOT a measured closed grasp or attached payload simulation.',
                      'T_base_tcp': (np.linalg.inv(base) @ tcp).tolist(), 'overlap_witnesses': [],
                      'target_overlap_witnesses': [],
                      'mesh_separating_axis_clearances_m': {}}
            for link, declaration in links.items():
                if not any(token in link for token in ('ag95', 'finger', 'knuckle')):
                    continue
                for collision in declaration.findall('collision'):
                    mesh = collision.find('geometry/mesh')
                    if mesh is None:
                        continue
                    package, relative = mesh.get('filename')[len('package://'):].split('/', 1)
                    mesh_path = next(SIM.glob('src/**/' + package + '/package.xml')).parent / relative
                    raw = mesh_path.read_bytes()
                    triangles = int.from_bytes(raw[80:84], 'little')
                    assert len(raw) == 84 + 50*triangles
                    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
                    points = np.frombuffer(raw, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(float)
                    points *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                    transform = fk(link) @ origin(collision.find('origin'))
                    points = points @ transform[:3, :3].T + transform[:3, 3]
                    base_gaps = np.maximum(lower-points.max(axis=0), points.min(axis=0)-upper)
                    target_points = points @ base_to_target[:3, :3].T + base_to_target[:3, 3]
                    target_center = np.array([0., 0., .15 if stage == 'lift' else 0.])
                    half_target = np.array([.12, .0265, .0575])
                    target_gaps = np.maximum(target_center-half_target-target_points.max(axis=0),
                                             target_points.min(axis=0)-(target_center+half_target))
                    triangle_hits = triangle_box_hits(target_points, target_center, half_target)
                    target_inside = np.all((target_points > target_center-half_target+1e-9) &
                                           (target_points < target_center+half_target-1e-9), axis=1)
                    if target_inside.any():
                        target_depths = np.minimum(target_points[target_inside]-(target_center-half_target),
                                                   target_center+half_target-target_points[target_inside]).min(axis=1)
                        target_deepest = int(np.argmax(target_depths))
                        branch['target_overlap_witnesses'].append({'link': link,
                            'inside_vertex_count': int(target_inside.sum()),
                            'example_vertex_target_xyz': target_points[target_inside][target_deepest].tolist(),
                            'example_min_distance_to_box_face_m': float(target_depths[target_deepest])})
                    details = {'chassis_max_axis_gap': float(base_gaps.max()),
                               'chassis_horizontal_axis_gap': float(base_gaps[:2].max()),
                               'target_max_axis_gap': float(target_gaps.max()),
                               'target_intersecting_triangles': int(triangle_hits.sum())}
                    if triangle_hits.any():
                        details['target_intersecting_triangle_example'] = target_points.reshape(-1, 3, 3)[triangle_hits][0].tolist()
                    if 'finger_pad' in link:
                        details['pad_target_frame_z_minmax_m'] = [float(target_points[:, 2].min()), float(target_points[:, 2].max())]
                        details['pad_target_frame_y_minmax_m'] = [float(target_points[:, 1].min()), float(target_points[:, 1].max())]
                        details['pad_target_vertical_overlap_m'] = float(min(target_points[:, 2].max(), target_center[2]+half_target[2])-max(target_points[:, 2].min(), target_center[2]-half_target[2]))
                    branch['mesh_separating_axis_clearances_m'][link] = details
                    inside = np.all((points > lower+1e-9) & (points < upper-1e-9), axis=1)
                    if inside.any():
                        depths = np.minimum(points[inside]-lower, upper-points[inside]).min(axis=1)
                        deepest = int(np.argmax(depths))
                        branch['overlap_witnesses'].append({'link': link, 'inside_vertex_count': int(inside.sum()),
                            'example_vertex_base_xyz': points[inside][deepest].tolist(),
                            'example_min_distance_to_box_face_m': float(depths[deepest])})
            result['branches'].append(branch)
    for branch in result['branches']:
        detail = branch.pop('mesh_separating_axis_clearances_m')
        pad = detail['ground/left_finger_pad']
        branch['all_ag95_chassis_horizontal_gap_min_m'] = min(v['chassis_horizontal_axis_gap'] for v in detail.values())
        branch['all_ag95_target_axis_gap_min_m'] = min(v['target_max_axis_gap'] for v in detail.values())
        branch['target_triangle_intersections'] = {n: v['target_intersecting_triangles'] for n, v in detail.items() if v['target_intersecting_triangles']}
        branch['pad_vertical_overlap_m'] = pad['pad_target_vertical_overlap_m']
        left = detail['ground/left_finger_pad']['pad_target_frame_y_minmax_m']
        right = detail['ground/right_finger_pad']['pad_target_frame_y_minmax_m']
        branch['pad_inner_gap_m'] = max(left[0], right[0])-min(left[1], right[1])
        transform = branch.pop('T_base_tcp')
        branch['tcp_base_xyz'] = [transform[i][3] for i in range(3)]
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
