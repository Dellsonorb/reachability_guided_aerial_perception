#!/usr/bin/env python3
"""Separate secondary public-TF resources from existing bags; never run a node.

Original rows, events, metrics and outcomes are immutable. Formal use requires
prospective declaration; this report is not an input to primary pilot analysis.
"""

import argparse
import copy
import heapq
import json
import math
from pathlib import Path


TOPICS = ['/tf', '/tf_static', '/clock']


def reconstruct(samples, messages, events=()):
    """Use receipt-ordered bag messages at every original sample time.

    A transform becomes eligible only after BOTH its bag receipt and header
    times. BufferCore supplies TF2's coherent latest common time, including its
    ordinary chain interpolation. No new path samples or smoothing are added.
    Any reset in the original samples or events invalidates the entire trace:
    receipt-sorted bag data cannot distinguish overlapping clock epochs.
    """
    import rospy
    import tf2_py
    from geometry_msgs.msg import TransformStamped
    from a5_ros_support import pose_xyzyaw, rigid_transform
    from a6_metrics import TF_MAX_AGE_S, _reset

    samples = list(samples)
    reset_seen = _reset(samples) or _reset(events)
    buffer = tf2_py.BufferCore()
    stream = iter(()) if reset_seen else iter(messages)
    upcoming = next(stream, None)
    pending, sequence, clock_s = [], 0, None
    rows = []
    for original in samples:
        row = copy.deepcopy(original)
        now = row['ros_time']
        while not reset_seen and upcoming is not None and upcoming[2].to_sec() <= now:
            topic, message, receipt = upcoming
            if topic in TOPICS:
                values = [message] if topic == '/clock' else message.transforms
                for value in values:
                    stamp = value.clock.to_sec() if topic == '/clock' else value.header.stamp.to_sec()
                    heapq.heappush(pending, (max(receipt.to_sec(), stamp), sequence, topic, value))
                    sequence += 1
            upcoming = next(stream, None)
        while not reset_seen and pending and pending[0][0] <= now:
            _, _, topic, value = heapq.heappop(pending)
            if topic == '/clock':
                stamp = value.clock.to_sec()
                reset_seen |= clock_s is not None and stamp < clock_s
                clock_s = stamp
            else:
                # rosbag's dynamically generated message classes differ from
                # installed geometry_msgs classes expected by the TF2 binding.
                typed = TransformStamped()
                typed.header.stamp = value.header.stamp
                typed.header.frame_id = value.header.frame_id.lstrip('/')
                typed.child_frame_id = value.child_frame_id.lstrip('/')
                for name, axes in (('translation', 'xyz'), ('rotation', 'xyzw')):
                    for axis in axes:
                        setattr(getattr(typed.transform, name), axis, getattr(getattr(value.transform, name), axis))
                insert = buffer.set_transform_static if topic == '/tf_static' else buffer.set_transform
                insert(typed, 'recorded_public_tf')
        row['bag_clock_s'] = clock_s
        for body, default in (('uav', 'uav1/base_link'), ('ground', 'ground/base_link')):
            frame = (original.get(body) or {}).get('frame_id', default).lstrip('/')
            try:
                if reset_seen:
                    raise ValueError('clock reset: bag epoch association is unavailable')
                if clock_s is None:
                    raise ValueError('no causally received simulation clock')
                transform = buffer.lookup_transform_core('map', frame, rospy.Time(0))
                stamp = transform.header.stamp.to_sec()
                age = now - stamp
                if not math.isfinite(stamp) or not 0 <= age <= TF_MAX_AGE_S:
                    raise ValueError('stale or future public map TF: age=%s' % age)
                position, quaternion = transform.transform.translation, transform.transform.rotation
                pose = pose_xyzyaw(rigid_transform(
                    [position.x, position.y, position.z],
                    [quaternion.x, quaternion.y, quaternion.z, quaternion.w]))
                row[body] = dict(frame_id=frame, xyz=pose[:3], yaw=pose[3], stamp_s=stamp, age_s=age,
                                 measurement_source='passive_bag_public_tf')
            except (tf2_py.TransformException, ValueError) as error:
                row[body] = dict(frame_id=frame, xyz=None, yaw=None, missing_reason=str(error),
                                 measurement_source='passive_bag_public_tf')
        rows.append(row)
    return rows


def describe_attempt(directory):
    """Read one existing activation and return a report with its secondary trace."""
    import rosbag
    from a6_metrics import MAX_SAMPLE_GAP_S, TF_MAX_AGE_S, summarize_metrics

    directory = Path(directory)
    attempt = json.loads((directory / 'attempt.json').read_text())
    original = json.loads((directory / 'data/metrics.json').read_text())
    events = [json.loads(line) for line in (directory / 'data/events.jsonl').read_text().splitlines()]
    samples = [json.loads(line) for line in (directory / 'data/trajectory.jsonl').read_text().splitlines()]
    with rosbag.Bag(str(directory / 'diagnostics.bag'), 'r') as bag:
        trace = reconstruct(samples, bag.read_messages(topics=TOPICS), events=events)
    method, run_result = original['method'], original.get('adapter_run_result')
    raw = summarize_metrics(events, samples, method, run_result)
    derived = summarize_metrics(events, trace, method, run_result)
    non_path = lambda metrics: {key: value for key, value in metrics.items() if key != 'paths'}
    missing = lambda rows, body: sum((row.get(body) or {}).get('xyz') is None for row in rows)
    return dict(
        kind='bag_public_tf_resource_diagnostics', schema_version=1,
        measurement_role='SECONDARY_DIAGNOSTIC_ONLY', attempt=str(directory),
        bag=str(directory / 'diagnostics.bag'), topics=list(TOPICS),
        eligibility='bag_receipt_s <= original_sample_s AND tf_header_s <= original_sample_s',
        composition='TF2 latest common time with eligible messages only',
        tf_max_age_s=TF_MAX_AGE_S, max_sample_gap_s=MAX_SAMPLE_GAP_S,
        method=method, original_sample_count=len(samples), derived_sample_count=len(trace),
        missing_body_samples={body: dict(original=missing(samples, body), bag_derived=missing(trace, body))
                              for body in ('uav', 'ground')},
        raw_metrics_reproduced=raw == original,
        non_path_metrics_unchanged=non_path(raw) == non_path(derived),
        outcome_context={key: attempt.get(key) for key in
                         ('status', 'retrieval_success', 'classification_reason')},
        paths={key: dict(original=original['paths'][key], bag_derived=value)
               for key, value in derived['paths'].items()},
        trace=trace,
        note='Original online metrics and outcomes remain authoritative and unchanged. '
             'Bag poses describe recorded public TF, not the adapter callback buffer. '
             'All original sample times/order/wall stamps are retained; absent evidence stays missing. '
             'No trial reruns, inserted samples, ground truth or outcome reclassification. '
             'Formal use requires a prospectively frozen measurement procedure.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--attempt-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, help='New secondary JSON report including trace; default stdout.')
    args = parser.parse_args(argv)
    serialized = json.dumps(describe_attempt(args.attempt_dir), indent=2, allow_nan=False) + '\n'
    if args.output is None:
        print(serialized, end='')
    else:
        with args.output.open('x') as destination:
            destination.write(serialized)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
