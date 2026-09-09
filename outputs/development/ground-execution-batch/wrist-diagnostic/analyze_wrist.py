#!/usr/bin/env python3
"""Read archived public ROS bags only; print reproducible wrist metrics as JSON.

No node, service, simulator, configuration change, or bag/artifact write occurs.
Times are ROS simulation seconds from message headers (goal/result receipt times).
"""
import json
from pathlib import Path

import numpy as np
import rosbag


RUNS = (
    "launch-03-easy-camera",
    "launch-06-easy-ground-latched",
    "launch-07-easy-ground-origin",
)
STATE = "/ground/arm_controller/state"
JOINTS = "/ground/joint_states"
GOAL = "/ground/arm_controller/follow_joint_trajectory/goal"
RESULT = "/ground/arm_controller/follow_joint_trajectory/result"


def bounds(values):
    return [float(np.min(values)), float(np.median(values)), float(np.max(values))]


def window_metrics(rows, lo, hi):
    data = rows[(rows[:, 0] >= lo) & (rows[:, 0] <= hi)]
    assert len(data) >= 2
    t, q, v = data[:, :3].T
    dt = np.diff(t)
    assert np.all(dt > 0) and np.all(np.isfinite(data))
    fd = np.diff(q) / dt
    result = {
        "samples": len(data),
        "first_last_header_s": [float(t[0]), float(t[-1])],
        "dt_min_median_max_s": bounds(dt),
        "q_range_rad": float(np.ptp(q)),
        "q_net_change_rad": float(q[-1] - q[0]),
        "sampled_velocity_trapezoid_integral_rad": float(np.trapz(v, t)),
        "velocity_min_median_max_rad_s": bounds(v),
        "position_difference_velocity_min_median_max_rad_s": bounds(fd),
        "position_difference_velocity_abs_p95_rad_s": float(np.percentile(abs(fd), 95)),
        "velocity_above_configured_0_10_count": int(np.sum(abs(v) > 0.10)),
    }
    if rows.shape[1] == 7:
        result["desired_velocity_abs_max_rad_s"] = float(np.max(abs(data[:, 4])))
        result["position_error_abs_max_rad"] = float(np.max(abs(data[:, 5])))
    else:
        result["effort_min_median_max_Nm"] = bounds(data[:, 3])
    return result


def analyze(run_dir):
    state, joints, goals, results, clocks = [], [], [], [], []
    all_joint_states = []
    with rosbag.Bag(str(run_dir / "diagnostics.bag"), "r") as bag:
        for topic, msg, stamp in bag.read_messages(topics=[STATE, JOINTS, GOAL, RESULT, "/clock"]):
            if topic == STATE:
                i = msg.joint_names.index("wrist_3_joint")
                state.append([msg.header.stamp.to_sec(), msg.actual.positions[i],
                              msg.actual.velocities[i], msg.desired.positions[i],
                              msg.desired.velocities[i], msg.error.positions[i],
                              msg.error.velocities[i]])
                all_joint_states.append((msg.header.stamp.to_sec(), list(msg.actual.velocities)))
                arm_names = list(msg.joint_names)
            elif topic == JOINTS:
                i = msg.name.index("wrist_3_joint")
                joints.append([msg.header.stamp.to_sec(), msg.position[i], msg.velocity[i], msg.effort[i]])
            elif topic == GOAL:
                goal = msg.goal
                goals.append({
                    "id": msg.goal_id.id,
                    "receipt_s": stamp.to_sec(),
                    "trajectory_header_s": goal.trajectory.header.stamp.to_sec(),
                    "duration_s": goal.trajectory.points[-1].time_from_start.to_sec(),
                    "goal_time_tolerance_s": goal.goal_time_tolerance.to_sec(),
                    "goal_tolerance_count": len(goal.goal_tolerance),
                })
            elif topic == RESULT:
                results.append({"id": msg.status.goal_id.id, "receipt_s": stamp.to_sec(),
                                "status": msg.status.status, "error_code": msg.result.error_code,
                                "error_string": msg.result.error_string})
            else:
                clocks.append(msg.clock.to_sec())
    state, joints = np.array(state), np.array(joints)
    goal = goals[-1]
    result = next(item for item in results if item["id"] == goal["id"])
    assert result["error_code"] == -5 and "wrist_3_joint" in result["error_string"]
    assert goal["trajectory_header_s"] == 0
    # Header zero means immediate start; receipt is an approximation within
    # controller/update scheduling. Avoid boundaries by trimming 100 ms / 10 ms.
    estimated_end = goal["receipt_s"] + goal["duration_s"]
    lo, hi = estimated_end + 0.1, result["receipt_s"] - 0.01
    a = state[(state[:, 0] >= lo) & (state[:, 0] <= hi), :3]
    ids = np.searchsorted(joints[:, 0], a[:, 0]).clip(0, len(joints) - 1)
    prev = (ids - 1).clip(0, len(joints) - 1)
    ids = np.where(abs(joints[ids, 0] - a[:, 0]) < abs(joints[prev, 0] - a[:, 0]), ids, prev)
    b = joints[ids, :3]
    separation = a[:, 0] - b[:, 0]
    assert np.all(separation > 0)
    paired_fd = (a[:, 1] - b[:, 1]) / separation
    paired_v = (a[:, 2] + b[:, 2]) / 2
    pre_result = next(item for item in results if item["id"] == goals[-2]["id"])
    per_joint = np.array([v for t, v in all_joint_states if lo <= t <= hi])
    return {
        "bag": str(run_dir / "diagnostics.bag"),
        "lift_goal": goal,
        "lift_result": result,
        "estimated_lift_trajectory_end_s": estimated_end,
        "abort_minus_estimated_end_s": result["receipt_s"] - estimated_end,
        "settle_window_requested_s": [lo, hi],
        "settle_controller_state": window_metrics(state, lo, hi),
        "settle_joint_state": window_metrics(joints, lo, hi),
        "prelift_grasp_hold_controller_state": window_metrics(
            state, pre_result["receipt_s"] + 0.1, goal["receipt_s"] - 0.05),
        "same_window_paired_streams": {
            "samples": len(a),
            "controller_minus_nearest_joint_header_min_median_max_s": bounds(separation),
            "paired_position_difference_velocity_min_median_max_rad_s": bounds(paired_fd),
            "paired_reported_velocity_mean_min_median_max_rad_s": bounds(paired_v),
            "caveat": "Common hardware feedback, not independent sensors; header phases differ by 1 ms.",
        },
        "settle_all_arm_joint_velocity_min_median_max_rad_s": {
            name: bounds(per_joint[:, i]) for i, name in enumerate(arm_names)
        },
        "clock_dt_min_median_max_s": bounds(np.diff(clocks)),
    }


if __name__ == "__main__":
    root = Path(__file__).resolve().parent.parent
    output = {
        "method": "Public native ROS bag only; no GT, nodes, services, launches, or production changes.",
        "units": "simulation seconds; joint radians; rad/s; reported joint effort Nm",
        "configuration_unchanged": {"goal_position_rad": 0.05, "stopped_velocity_rad_s": 0.10,
                                    "goal_time_s": 2.0, "state_publish_Hz": 100},
        "integral_caveat": "Trapezoid integral of 100 Hz sampled velocity is not an integral of unrecorded physics-step velocity.",
        "runs": {run: analyze(root / run) for run in RUNS},
    }
    print(json.dumps(output, indent=2, sort_keys=True))
