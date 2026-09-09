#!/usr/bin/env python3
"""Read finalized public ROS bags offline; print reproducible clearance-error JSON.

No simulator/GT topics, ROS nodes/services, or output-file writes. Every paired
controller sample from all four specified bags is retained; stages use events.
"""
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import rosbag

HERE = Path(__file__).resolve().parent
AGENT = HERE.parents[3]
BATCH = AGENT / 'outputs/development/ground-manipulation-batch'
SIM = Path('/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation')
spec = importlib.util.spec_from_file_location('geometry', BATCH / 'moderate-diagnostic/analyze_geometry.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)
from ground_manipulator_runtime.renderer import render_ground_robot

ARM = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
HAND = 'left_outer_knuckle_joint'
RUNS = ('launch-05-easy-integrated-feedback', 'launch-06-moderate-full-robot',
        'launch-07-easy-full-e2e', 'launch-08-moderate-ours-contact-geometry')
ARM_TOPIC = '/ground/arm_controller/state'
HAND_TOPIC = '/ground/gripper_controller/state'
TOPICS = (ARM_TOPIC, HAND_TOPIC, '/ground/joint_states', '/ground_observer/target_pose',
          '/ground/arm_controller/follow_joint_trajectory/goal',
          '/ground/arm_controller/follow_joint_trajectory/result')


def corners(lower, upper):
    return np.array(list(itertools.product(*zip(lower, upper))))


class Model:
    def __init__(self):
        xml = render_ground_robot(SIM / 'src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro')
        self.sha256 = hashlib.sha256(xml.encode()).hexdigest()
        robot = ET.fromstring(xml)
        self.joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
        self.boxes = {}
        self.mesh_float32_spacing_max_m = 0.0
        for link in robot.findall('link'):
            points = []
            for collision in link.findall('collision'):
                shape = collision.find('geometry')[0]
                if shape.tag == 'mesh':
                    package, relative = shape.get('filename')[len('package://'):].split('/', 1)
                    path = next(SIM.glob('src/**/' + package + '/package.xml')).parent / relative
                    raw = path.read_bytes()
                    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
                    count = int.from_bytes(raw[80:84], 'little')
                    assert len(raw) == 84 + 50 * count, path
                    vertices32 = np.frombuffer(raw, dtype=dtype, offset=84)['vertices'].reshape(-1, 3)
                    scale = np.fromstring(shape.get('scale', '1 1 1'), sep=' ')
                    self.mesh_float32_spacing_max_m = max(self.mesh_float32_spacing_max_m,
                        float(np.max(abs(np.spacing(vertices32)).astype(float) * abs(scale))))
                    vertices = vertices32.astype(float) * scale
                else:
                    if shape.tag == 'box':
                        half = np.fromstring(shape.get('size'), sep=' ') / 2
                    elif shape.tag == 'cylinder':
                        half = np.array([float(shape.get('radius'))] * 2 + [float(shape.get('length')) / 2])
                    elif shape.tag == 'sphere':
                        half = np.full(3, float(shape.get('radius')))
                    else:
                        raise ValueError(shape.tag)
                    vertices = corners(-half, half)
                origin = geometry.origin(collision.find('origin'))
                points.append(vertices @ origin[:3, :3].T + origin[:3, 3])
            if points:
                points = np.concatenate(points)
                self.boxes[link.get('name')] = corners(points.min(0), points.max(0))

    def fk(self, positions):
        count = len(next(iter(positions.values())))
        cache = {'ground/base_link': np.broadcast_to(np.eye(4), (count, 4, 4)).copy()}

        def visit(link):
            if link in cache:
                return cache[link]
            joint = self.joints[link]
            value = visit(joint.find('parent').get('link')) @ geometry.origin(joint.find('origin'))
            kind = joint.get('type')
            if kind in ('revolute', 'continuous'):
                mimic = joint.find('mimic')
                name = joint.get('name') if mimic is None else mimic.get('joint')
                angle = positions.get(name, np.zeros(count))
                if mimic is not None:
                    angle = angle * float(mimic.get('multiplier', 1)) + float(mimic.get('offset', 0))
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                axis /= np.linalg.norm(axis)
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                rotation = np.eye(3) + np.sin(angle)[:, None, None] * skew + (1 - np.cos(angle))[:, None, None] * (skew @ skew)
                motion = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
                motion[:, :3, :3] = rotation
                value = value @ motion
            elif kind != 'fixed':
                raise ValueError(kind)
            cache[link] = value
            return value

        for link in list(self.boxes) + ['ground/gripper_tcp_link']:
            visit(link)
        return cache

    def displacement(self, first, second):
        result = {}
        for link, points in self.boxes.items():
            delta = first[link] - second[link]
            vectors = np.einsum('nij,kj->nki', delta[:, :3, :3], points) + delta[:, None, :3, 3]
            result[link] = np.linalg.norm(vectors, axis=2).max(axis=1)
        return result

    def relative_displacement(self, first, second):
        """Fix each counterpart frame in turn; use the tighter valid bound.

        Transforming the entire pair by the second link's inverse rigid pose
        leaves its collision shape fixed, cancelling common ancestor motion.
        """
        result = {}
        for one, two in itertools.combinations(sorted(self.boxes), 2):
            bounds = []
            for moving, reference in ((one, two), (two, one)):
                relative = []
                for poses in (first, second):
                    rotation = np.swapaxes(poses[reference][:, :3, :3], 1, 2)
                    relative.append((rotation @ poses[moving][:, :3, :3],
                        np.einsum('nij,nj->ni', rotation,
                                  poses[moving][:, :3, 3] - poses[reference][:, :3, 3])))
                delta_r = relative[0][0] - relative[1][0]
                delta_t = relative[0][1] - relative[1][1]
                vectors = np.einsum('nij,kj->nki', delta_r, self.boxes[moving]) + delta_t[:, None, :]
                bounds.append(np.linalg.norm(vectors, axis=2).max(axis=1))
            result[(one, two)] = np.minimum(*bounds)
        return result


def stats(values, stamps):
    values = np.asarray(values)
    finite = np.isfinite(values)
    values, stamps = values[finite], np.asarray(stamps)[finite]
    if not len(values):
        return {'samples': 0}
    absolute = abs(values)
    peak = int(np.argmax(absolute))
    return dict(samples=len(values), signed_min=float(values.min()), signed_max=float(values.max()),
                p50_abs=float(np.percentile(absolute, 50)), p95_abs=float(np.percentile(absolute, 95)),
                p99_abs=float(np.percentile(absolute, 99)), max_abs=float(absolute[peak]),
                max_at_header_s=float(stamps[peak]))


def windows(events, stamps):
    result = [dict(name='whole_recording', start_s=float(stamps[0]), end_s=float(stamps[-1]))]
    active = {}
    for event in events:
        stage, state = event.get('stage'), event.get('state')
        if state == 'A6_STAGE_START':
            active.setdefault(stage, float(event['ros_time']))
        elif state in ('A6_STAGE_END', 'A6_STAGE_FAILED') and stage in active:
            result.append(dict(name=stage, start_s=active.pop(stage), end_s=float(event['ros_time'])))
    terminal = next((e['ros_time'] for e in events if e.get('state') == 'FAILED'), float(stamps[-1]))
    for stage, start in active.items():
        result.append(dict(name=stage, start_s=start, end_s=terminal, boundary='open stage closed by FAILED or recording end'))
    preshape = next((e['ros_time'] for e in events if e.get('state') == 'GROUND_GRIPPER_PRESHAPE'), None)
    close = next((e['ros_time'] for e in events if e.get('state') == 'A6_STAGE_START' and e.get('stage') == 'close'), terminal)
    if preshape is not None:
        result.append(dict(name='accepted_preshape_until_close_or_failure', start_s=preshape, end_s=close))
    return result


def analyze(name, model):
    run = BATCH / name
    events = [json.loads(line) for line in (run / 'data/events.jsonl').read_text().splitlines()]
    states, poses, goals, results = {ARM_TOPIC: {}, HAND_TOPIC: {}}, [], [], []
    measured_hand = {}
    with rosbag.Bag(str(run / 'diagnostics.bag')) as bag:
        for topic, message, receipt in bag.read_messages(topics=TOPICS):
            stamp = message.header.stamp.to_nsec()
            if topic in states:
                assert stamp not in states[topic], (topic, stamp)
                states[topic][stamp] = message
            elif topic == '/ground/joint_states':
                measured_hand[stamp] = message.position[list(message.name).index(HAND)]
            elif topic == '/ground_observer/target_pose':
                p, q = message.pose.position, message.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                poses.append([stamp * 1e-9, p.x, p.y, p.z, yaw])
            elif topic.endswith('/goal'):
                trajectory = message.goal.trajectory
                goals.append(dict(goal_id=message.goal_id.id, receipt_s=receipt.to_sec(),
                    start_s=trajectory.header.stamp.to_sec() or message.goal_id.stamp.to_sec(),
                    duration_s=trajectory.points[-1].time_from_start.to_sec(),
                    joint_names=list(trajectory.joint_names), endpoint=list(trajectory.points[-1].positions)))
            else:
                results.append(dict(goal_id=message.status.goal_id.id, header_s=stamp * 1e-9,
                    status=message.status.status, error_code=message.result.error_code, error_string=message.result.error_string))
    common = sorted(states[ARM_TOPIC])
    stamps = np.array(common, dtype=float) * 1e-9
    desired, actual, errors = {}, {}, {}
    for topic in (ARM_TOPIC,):
        for joint in ARM:
            indices = [list(states[topic][stamp].joint_names).index(joint) for stamp in common]
            desired[joint] = np.array([states[topic][t].desired.positions[i] for t, i in zip(common, indices)])
            actual[joint] = np.array([states[topic][t].actual.positions[i] for t, i in zip(common, indices)])
            errors[joint] = np.array([states[topic][t].error.positions[i] for t, i in zip(common, indices)])
    measured_stamps = np.array(sorted(measured_hand), dtype=np.int64)
    right = np.clip(np.searchsorted(measured_stamps, common), 0, len(measured_stamps) - 1)
    left = np.maximum(0, right - 1)
    nearest = np.where(abs(measured_stamps[left] - common) <= abs(measured_stamps[right] - common), left, right)
    offsets = (measured_stamps[nearest] - common) * 1e-9
    hand_measured = np.array([measured_hand[t] for t in measured_stamps[nearest]])
    hand_exact = np.array([t in states[HAND_TOPIC] for t in common])
    actual[HAND] = np.array([states[HAND_TOPIC][t].actual.positions[0] if t in states[HAND_TOPIC] else hand_measured[i] for i, t in enumerate(common)])
    desired[HAND] = np.array([states[HAND_TOPIC][t].desired.positions[0] if t in states[HAND_TOPIC] else actual[HAND][i] for i, t in enumerate(common)])
    errors[HAND] = np.where(hand_exact, desired[HAND] - actual[HAND], np.nan)
    error_identity = max(float(np.max(abs(((desired[j] - actual[j] + np.pi) % (2 * np.pi) - np.pi) - errors[j]))) for j in ARM)
    assert error_identity < 1e-10, error_identity
    desired_fk, actual_fk = model.fk(desired), model.fk(actual)
    arm_only = dict(desired, **{HAND: actual[HAND]})
    arm_only_fk = model.fk(arm_only)
    link_errors = model.displacement(actual_fk, desired_fk)
    for values in link_errors.values():
        values[~hand_exact] = np.nan
    arm_link_errors = model.displacement(actual_fk, arm_only_fk)
    relative_errors = model.relative_displacement(actual_fk, arm_only_fk)
    tcp_actual, tcp_desired = actual_fk['ground/gripper_tcp_link'], desired_fk['ground/gripper_tcp_link']
    tcp_xyz = tcp_actual[:, :3, 3] - tcp_desired[:, :3, 3]
    rotation = tcp_actual[:, :3, :3] @ np.swapaxes(tcp_desired[:, :3, :3], 1, 2)
    tcp_rotation = np.arccos(np.clip((np.trace(rotation, axis1=1, axis2=2) - 1) / 2, -1, 1))
    out = dict(run=name, all_arm_samples=len(common), exactly_paired_arm_hand_samples=int(hand_exact.sum()),
        hand_controller_samples=len(states[HAND_TOPIC]),
        nearest_public_hand_offset_s=stats(offsets, stamps),
        max_error_field_identity_residual_rad=error_identity, header_interval_s=stats(np.diff(stamps), stamps[1:]),
        arm_goals=goals, arm_results=results, stages=[])
    for window in windows(events, stamps):
        mask = (stamps >= window['start_s'] - 1e-9) & (stamps <= window['end_s'] + 1e-9)
        selected = stamps[mask]
        if not len(selected):
            out['stages'].append(dict(window, samples=0))
            continue
        item = dict(window, samples=len(selected),
            gripper_measured_rad=stats(actual[HAND][mask], selected),
            joint_desired_minus_actual_rad={j: stats(errors[j][mask], selected) for j in errors},
            tcp_actual_minus_desired_xyz_m=[stats(tcp_xyz[mask, i], selected) for i in range(3)],
            tcp_position_error_m=stats(np.linalg.norm(tcp_xyz[mask], axis=1), selected),
            tcp_rotation_error_rad=stats(tcp_rotation[mask], selected),
            arm_induced_link_box_displacement_m={j: stats(v[mask], selected) for j, v in arm_link_errors.items() if np.max(v[mask]) > 0},
            arm_and_gripper_link_box_displacement_m={j: stats(v[mask], selected) for j, v in link_errors.items() if np.any(np.nan_to_num(v[mask]) > 0)},
            arm_relative_pair_displacement_max_m=[float(v[mask].max()) for v in relative_errors.values()])
        out['stages'].append(item)
    for goal in goals:
        result = next((r for r in results if r['goal_id'] == goal['goal_id']), None)
        if result is None:
            continue
        index = int(np.argmin(abs(stamps - result['header_s'])))
        endpoint = {j: np.array([actual[j][index]]) for j in actual}
        endpoint.update({j: np.array([v]) for j, v in zip(goal['joint_names'], goal['endpoint'])})
        expected = model.fk(endpoint)['ground/gripper_tcp_link'][0]
        measured = tcp_actual[index]
        goal['at_result'] = dict(header_s=float(stamps[index]), result_offset_s=float(stamps[index] - result['header_s']),
            endpoint_minus_actual_joint_rad={j: float((endpoint[j][0] - actual[j][index] + np.pi) % (2 * np.pi) - np.pi) for j in ARM},
            measured_minus_endpoint_tcp_xyz_m=(measured[:3, 3] - expected[:3, 3]).tolist(),
            tcp_position_error_m=float(np.linalg.norm(measured[:3, 3] - expected[:3, 3])),
            tcp_rotation_error_rad=float(np.arccos(np.clip((np.trace(measured[:3, :3] @ expected[:3, :3].T) - 1) / 2, -1, 1))))
    refined = next(e for e in events if e.get('state') == 'GROUND_REFINED')
    # All publicly published accepted cuboids before refinement. Later observations
    # can include a moving object during grasp and are counted, not called noise.
    pose_array = np.asarray(poses, dtype=float).reshape(-1, 5)
    selected = pose_array[pose_array[:, 0] <= refined['observation_stamp'] + 1e-9]
    pose_out = dict(total_published=len(poses), pre_refinement_samples=selected.tolist(),
        post_refinement_count=int(len(poses) - len(selected)), accepted_refined=refined,
        semantics='repeatability of published fused accepted cuboids, not estimation error or calibrated covariance')
    if len(selected):
        delta = selected[:, 1:4] - np.asarray(refined['target_map'][:3])
        yaw_delta = (selected[:, 4] - refined['target_map'][3] + np.pi / 2) % np.pi - np.pi / 2
        pose_out['relative_to_accepted_refinement'] = dict(
            position_m=stats(np.linalg.norm(delta, axis=1), selected[:, 0]),
            xyz_m=[stats(delta[:, i], selected[:, 0]) for i in range(3)],
            yaw_rad=stats(yaw_delta, selected[:, 0]))
    out['perception_repeatability'] = pose_out
    return out


def main():
    model = Model()
    zero = {j: np.zeros(1) for j in ARM + (HAND,)}
    fk = model.fk(zero)
    assert max(v.max() for v in model.displacement(fk, fk).values()) == 0.0
    moved = {link: value.copy() for link, value in fk.items()}
    for value in moved.values():
        value[:, 0, 3] += 0.01
    assert all(abs(v[0] - 0.01) < 1e-12 for v in model.displacement(fk, moved).values())
    result = dict(schema_version=1, diagnostic_only=True, no_gt_inputs=True, topics=list(TOPICS),
        model_sha256=model.sha256, model_mesh_float32_spacing_max_m=model.mesh_float32_spacing_max_m,
        collision_box_links=sorted(model.boxes),
        collision_link_pair_order=list(itertools.combinations(sorted(model.boxes), 2)),
        displacement_semantics='Maximum corresponding collision-bounding-box-corner displacement bounds every enclosed model point at the sampled measured versus desired state. It is not mesh separation or a continuous-time error bound.',
        runs=[analyze(run, model) for run in RUNS])
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
