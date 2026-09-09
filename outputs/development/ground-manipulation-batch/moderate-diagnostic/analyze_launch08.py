#!/usr/bin/env python3
"""Read-only finalized-bag audit of launch08 public manipulation evidence.

No ROS node/service or simulator state/GT/contact-output reads. Prints JSON.
"""
import importlib.util
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('launch06_audit', HERE / 'analyze_launch06.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)
import rosbag

RUN = HERE.parent / 'launch-08-moderate-ours-contact-geometry'


def main():
    events = [json.loads(line) for line in (RUN / 'data/events.jsonl').read_text().splitlines()]
    selected = next(e for e in events if e['state'] == 'A5_SELECTED')
    refined = next(e for e in events if e['state'] == 'GROUND_REFINED')
    scene = next(e for e in events if e['state'] == 'GROUND_MANIPULATION_SCENE' and e['source'] == 'accepted_refined')
    payload = next(e for e in events if e['state'] == 'GROUND_PAYLOAD_MODELED')
    lift = next(e for e in events if e['state'] == 'LIFT')
    end = next(e for e in events if e['state'] == 'GROUND_REPLAY_END')
    stages = [e for e in events if e['state'].startswith('A6_STAGE_')]
    cutoffs = {e['stage']+'_'+e['state'].split('_')[-1].lower(): e['ros_time'] for e in stages}
    cutoffs.update(payload=payload['ros_time'], payload_measurement=payload['measurement_stamp'], lift=lift['ros_time'])
    joint_samples, controller_samples = {}, {}
    snapshots = {label: {} for label in cutoffs}
    arm_goals, gripper_goals, arm_results, gripper_results = [], [], [], []
    confirmations, transitions, last_confirmed = [], [], None
    topics = ['/tf', '/tf_static', '/ground/joint_states', '/ground/gripper/grasp_confirmed',
              '/ground/arm_controller/state', '/ground/gripper_controller/state']
    for controller in ('arm', 'gripper'):
        for kind in ('goal', 'result'):
            topics.append('/ground/'+controller+'_controller/follow_joint_trajectory/'+kind)
    with rosbag.Bag(str(RUN / 'diagnostics.bag'), 'r') as bag:
        recorded_topics = sorted(bag.get_type_and_topic_info().topics)
        for topic, msg, record_stamp in bag.read_messages(topics=topics):
            if topic in ('/tf', '/tf_static'):
                for t in msg.transforms:
                    p, q = t.transform.translation, t.transform.rotation
                    for label, cutoff in cutoffs.items():
                        if t.header.stamp.to_sec() <= cutoff:
                            snapshots[label][t.child_frame_id] = (t.header.frame_id, audit.transform(
                                (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)), t.header.stamp.to_sec())
                continue
            if topic == '/ground/gripper/grasp_confirmed':
                stamp = record_stamp.to_sec()
                confirmations.append((stamp, bool(msg.data)))
                if bool(msg.data) != last_confirmed:
                    transitions.append({'record_stamp': stamp, 'confirmed': bool(msg.data)})
                    last_confirmed = bool(msg.data)
                continue
            stamp = msg.header.stamp.to_sec()
            if topic == '/ground/joint_states':
                for label, cutoff in cutoffs.items():
                    if stamp <= cutoff:
                        joint_samples[label] = {'stamp': stamp, 'positions': dict(zip(msg.name, msg.position))}
            elif topic.endswith('/state'):
                if stamp <= end['ros_time']:
                    controller_samples[topic] = {'stamp': stamp, 'names': list(msg.joint_names),
                        'actual_position': list(msg.actual.positions), 'desired_position': list(msg.desired.positions),
                        'actual_velocity': list(msg.actual.velocities), 'position_error': list(msg.error.positions)}
            elif topic.endswith('/goal'):
                tr = msg.goal.trajectory
                record = {'stamp': stamp, 'duration': tr.points[-1].time_from_start.to_sec(),
                          'names': list(tr.joint_names), 'endpoint': list(tr.points[-1].positions)}
                (arm_goals if '/arm_' in topic else gripper_goals).append(record)
            else:
                record = {'stamp': stamp, 'status': msg.status.status, 'error_code': msg.result.error_code,
                          'error_string': msg.result.error_string}
                (arm_results if '/arm_' in topic else gripper_results).append(record)

    def tf_chain(frame, snapshot):
        if frame == 'ground/base_link':
            return np.eye(4)
        parent, value, _ = snapshot[frame]
        return tf_chain(parent, snapshot) @ value

    tcp_snapshots = {label: tf_chain('ground/gripper_tcp_link', values).tolist()
                     for label, values in snapshots.items()
                     if 'ground/gripper_tcp_link' in values}
    measured = audit.transform(payload['measured_tcp'][:3], payload['measured_tcp'][3:])
    target = audit.transform(scene['target_pose'][:3], scene['target_pose'][3:])
    local_target = np.linalg.inv(measured) @ target
    confirmation_intervals = {}
    for label, start, stop in (
            ('close_to_end', cutoffs['close_start'], end['ros_time']),
            ('payload_to_end', payload['ros_time'], end['ros_time']),
            ('lift', cutoffs['lift_start'], cutoffs['lift_end']),
            ('retention', cutoffs['retention_start'], cutoffs['retention_end'])):
        values = [(stamp, value) for stamp, value in confirmations if start <= stamp <= stop]
        confirmation_intervals[label] = {
            'first_stamp': values[0][0] if values else None, 'last_stamp': values[-1][0] if values else None,
            'count': len(values), 'true_count': sum(v for _, v in values),
            'false_count': sum(not v for _, v in values),
            'max_sample_gap_s': max((b[0]-a[0] for a, b in zip(values, values[1:])), default=None)}
    output = {
        'run': str(RUN), 'versions': {'SIM': 'c12d8ba', 'AGENT': '6d1dba2'},
        'candidate': {k: selected[k] for k in ('candidate_id', 'source_id', 'x', 'y', 'yaw')},
        'accepted_refined': refined, 'scene': scene, 'payload': payload, 'lift': lift, 'end': end,
        'stages': stages, 'calibration': next(e for e in events if e['state'] == 'GROUND_GRASP_CALIBRATION'),
        'preshape': next(e for e in events if e['state'] == 'GROUND_GRIPPER_PRESHAPE'),
        'closure_geometry': next(e for e in events if e['state'] == 'GROUND_GRASP_GEOMETRY'),
        'plans': [e for e in events if e['state'] == 'GROUND_MANIPULATION_PLAN'],
        'joint_samples': joint_samples, 'final_controller_samples': controller_samples,
        'arm_goals': arm_goals, 'arm_results': arm_results,
        'gripper_goals': gripper_goals, 'gripper_results': gripper_results,
        'confirmation_transitions': transitions, 'confirmation_intervals': confirmation_intervals,
        'public_tcp_snapshots': tcp_snapshots,
        'derived_expected_T_tcp_perceived_target': local_target.tolist(),
        'attachment_transform_roundtrip_max_abs': float(np.max(abs(measured @ local_target-target))),
        'recorded_planning_scene_topics': [t for t in recorded_topics if 'planning_scene' in t],
        'scope': 'Public runtime confirmation/action/joint/TF and accepted perception only. Scene activation evidenced by gated status and deployed source; no serialized service-request/scene snapshots or independent physical target trajectory are claimed.'}
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
