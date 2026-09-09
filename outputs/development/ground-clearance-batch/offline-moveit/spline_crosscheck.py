"""Compare production Python samples to native installed controller math offline."""
import copy
import io
import math
from pathlib import Path
import struct
import subprocess
import sys

import numpy as np

SIM = Path('/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation')
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
sys.path.insert(0, str(SIM / 'src/demos/air_ground_pick_demo/src'))
import rospy
from air_ground_pick_demo import execution_clearance as ec
from moveit_msgs.msg import RobotState, RobotTrajectory, AttachedCollisionObject
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

def packet(message):
    stream = io.BytesIO()
    message.serialize(stream)
    data = stream.getvalue()
    return struct.pack('<I', len(data)) + data

rng = np.random.RandomState(15321)
cases, inputs = [], bytearray()
for held_mode in (False, True):
    for positive_first in ((False, True) if held_mode else (False,)):
        for order0 in range(3):
            for order1 in range(3):
                for order2 in range(3):
                    trajectory = RobotTrajectory()
                    trajectory.joint_trajectory.joint_names = ['j1', 'j2']
                    durations = rng.choice([.0011, .0073, .041, .123, .3107], 2)
                    times = [0., durations[0], sum(durations)]
                    for order, time in zip((order0, order1, order2), times):
                        point = JointTrajectoryPoint(positions=rng.uniform(-2., 2., 2).tolist(),
                                                     time_from_start=rospy.Duration(float(time)))
                        if order >= 1:
                            point.velocities = rng.uniform(-2., 2., 2).tolist()
                        if order >= 2:
                            point.accelerations = rng.uniform(-3., 3., 2).tolist()
                        trajectory.joint_trajectory.points.append(point)
                    if positive_first:
                        trajectory.joint_trajectory.points.pop(0)
                    hold = JointTrajectoryPoint(positions=rng.uniform(-2., 2., 2).tolist(),
                                                velocities=[0., 0.], accelerations=[0., 0.])
                    state = RobotState()
                    state.joint_state.name = ['j2', 'gripper', 'j1']
                    state.joint_state.position = [.1, .37, -.2]
                    attachment = AttachedCollisionObject()
                    attachment.object.id = 'payload_preservation_sentinel'
                    state.attached_collision_objects = [attachment]
                    before = copy.deepcopy((state, trajectory, hold))
                    samples = list(ec.controller_samples(state, trajectory, .001,
                                   held_desired=hold if held_mode else None))
                    effective_times = [p.time_from_start.to_sec() for p in trajectory.joint_trajectory.points]
                    if held_mode and effective_times[0] > 0:
                        effective_times.insert(0, 0.)
                    sample_times = [0.]
                    for a, b in zip(effective_times, effective_times[1:]):
                        count = int(math.ceil((b-a)/.001))
                        sample_times.extend(a+(b-a)*i/count for i in range(1, count+1))
                    assert len(sample_times) == len(samples)
                    assert all(sample.joint_state.position[1] == .37 for sample in samples)
                    assert all(sample.attached_collision_objects == [attachment] for sample in samples)
                    assert before == (state, trajectory, hold)
                    values = np.array([[sample.joint_state.position[2], sample.joint_state.position[0]] for sample in samples])
                    cases.append(values)
                    inputs += struct.pack('<I', int(held_mode))
                    inputs += packet(trajectory.joint_trajectory) + packet(hold) + packet(Float64MultiArray(data=sample_times))

result = subprocess.run([sys.argv[1]], input=inputs, capture_output=True, timeout=60)
if result.returncode:
    print(result.stderr.decode(), file=sys.stderr)
    raise SystemExit(result.returncode)
lines = [line for line in result.stdout.decode().splitlines() if line.startswith('CASE ')]
assert len(lines) == len(cases), (len(lines), len(cases), result.stdout[:1000])
maximum_error = 0.
for line, expected in zip(lines, cases):
    actual = np.array(list(map(float, line.split()[2:]))).reshape(expected.shape)
    error = float(np.max(np.abs(actual-expected)))
    maximum_error = max(maximum_error, error)
    np.testing.assert_allclose(actual, expected, atol=1e-9, rtol=0.)
print('PASS cases=%d scalar_positions=%d max_absolute_error_rad=%.17g' %
      (len(cases), sum(case.size for case in cases), maximum_error))
print('Includes native JTC immediate hold bridges, positive-first points, all 27 derivative-order triples, nonuniform times, reversed state joint ordering and payload/nonarm preservation.')
