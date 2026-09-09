#!/usr/bin/env python3
"""Read the failed trial's TF bag into native buffers; never contact ROS."""

import argparse
import io
import json

import rosbag
import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage


def native_message(message):
    serialized = io.BytesIO()
    message.serialize(serialized)
    return TFMessage().deserialize(serialized.getvalue())


def snapshot(buffer, now):
    transform = buffer.lookup_transform("ground/base_link", "map", rospy.Time(0))
    stamp = transform.header.stamp.to_sec()
    return {
        "lookup_sim_time": now,
        "stamp": stamp,
        "age_seconds": round(now - stamp, 9),
        "fresh_at_existing_0_5_second_limit": -0.1 <= now - stamp <= 0.5,
    }


def analyze(path):
    live = tf2_ros.Buffer(debug=False)
    startup = tf2_ros.Buffer(debug=False)
    cutoffs = [154.669, 154.678, 155.952, 155.978, 155.984, 156.1]
    live_snapshots = []
    samples = []
    static_chain = []
    with rosbag.Bag(path) as bag:
        for _, message, recorded in bag.read_messages(topics=["/tf_static"]):
            if recorded.to_sec() >= 154.0:
                continue
            for transform in native_message(message).transforms:
                for buffer in (live, startup):
                    buffer.set_transform_static(transform, "recorded_tf_static")
                if transform.child_frame_id == "ground/odom":
                    static_chain.append({
                        "parent": transform.header.frame_id,
                        "child": transform.child_frame_id,
                        "recorded_sim_time": recorded.to_sec(),
                        "stamp": transform.header.stamp.to_sec(),
                    })
        for _, message, recorded in bag.read_messages(
                topics=["/tf"], start_time=rospy.Time.from_sec(154.0),
                end_time=rospy.Time.from_sec(157.0)):
            while cutoffs and recorded.to_sec() > cutoffs[0]:
                live_snapshots.append(snapshot(live, cutoffs.pop(0)))
            for transform in native_message(message).transforms:
                if (transform.header.frame_id, transform.child_frame_id) != (
                        "ground/odom", "ground/base_link"):
                    continue
                live.set_transform(transform, "recorded_tf")
                if recorded.to_sec() <= 154.678:
                    startup.set_transform(transform, "recorded_tf")
                samples.append((recorded.to_sec(), transform.header.stamp.to_sec()))
        return {
            "diagnostic_only": True,
            "input_bag": path,
            "bag_time_range": [bag.get_start_time(), bag.get_end_time()],
            "target_frame": "ground/base_link",
            "source_frame": "map",
            "static_chain": static_chain,
            "ground_dynamic_tf_154_to_157": {
                "count": len(samples),
                "first_recorded_and_stamp": samples[0],
                "last_recorded_and_stamp": samples[-1],
                "maximum_stamp_gap_seconds": round(max(
                    after[1] - before[1] for before, after in zip(samples, samples[1:])), 9),
                "last_five_received_before_failure": [row for row in samples if row[0] <= 155.978][-5:],
            },
            "live_native_buffer_snapshots": live_snapshots,
            "modeled_buffer_frozen_at_moveit_start": snapshot(startup, 155.978),
            "limitation": (
                "The failed process did not record its internal TF-buffer stamp. "
                "The frozen buffer is an offline model of delayed callbacks, not an observed internal stamp."),
            "original_trial": {
                "status": "VALID_TRIAL",
                "reason": "target planning-frame transform is stale",
                "unchanged": True,
            },
        }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    arguments = parser.parse_args()
    print(json.dumps(analyze(arguments.bag), indent=2, sort_keys=True))
