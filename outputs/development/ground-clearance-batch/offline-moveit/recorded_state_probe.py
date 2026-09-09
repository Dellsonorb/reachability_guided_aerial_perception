"""Render current URDF and query recorded public states offline; no ROS starts/writes."""
import io
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

SIM = Path('/media/lu/P450_PAPER/SIM/p450_sim_v1/.worktrees/bunker-a-implementation')
AGENT = Path('/media/lu/P450_PAPER/AGENT/reachability_guided_aerial_perception')
os.environ['ROS_PACKAGE_PATH'] = str(SIM / 'src') + ':/opt/ros/noetic/share'
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
sys.path.insert(0, str(SIM / 'src/platform/ground_manipulator_runtime/src'))
from ground_manipulator_runtime.renderer import render_ground_robot
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def packet(raw):
    if isinstance(raw, str):
        raw = raw.encode()
    elif not isinstance(raw, bytes):
        stream = io.BytesIO()
        raw.serialize(stream)
        raw = stream.getvalue()
    return struct.pack('<I', len(raw)) + raw

urdf = render_ground_robot(SIM / 'src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro')
srdf = (SIM / 'src/ground/bunker_aubo_moveit_config/config/ground_robot.srdf').read_text()
inputs = packet(urdf) + packet(srdf)
root = AGENT / 'outputs/development/ground-manipulation-batch'
record = json.loads((root / 'moderate-diagnostic/launch06_findings.json').read_text())
events = [json.loads(line) for line in (root / 'launch-06-moderate-full-robot/data/events.jsonl').read_text().splitlines()]
target = next(e for e in events if e['state'] == 'GROUND_MANIPULATION_SCENE' and e['source'] == 'accepted_refined')
planned = dict(record['joint_samples']['preshape']['positions'])
last_goal = record['arm_goals'][-1]
planned.update(zip(last_goal['joint_names'], last_goal['endpoint']))
cases = [('source624_planned', planned), ('source624_measured', record['joint_samples']['failed']['positions'])]
guarded = len(sys.argv) > 2 and sys.argv[2] == 'guard'
if guarded:
    home = dict(planned)
    home.update(dict(shoulder_pan_joint=0, shoulder_lift_joint=-.5, elbow_joint=1,
                     wrist_1_joint=0, wrist_2_joint=1, wrist_3_joint=0, left_outer_knuckle_joint=0))
    observation = dict(planned)
    first_goal = record['arm_goals'][0]
    observation.update(zip(first_goal['joint_names'], first_goal['endpoint']))
    observation['left_outer_knuckle_joint'] = 0
    cases = [('srdf_home_open', home), ('source624_observation_endpoint_open', observation)] + cases
for name, positions in cases:
    message = PlanningScene()
    message.name = name
    message.is_diff = True
    message.robot_state.is_diff = True
    message.robot_state.joint_state.name = list(positions)
    message.robot_state.joint_state.position = list(positions.values())
    object = CollisionObject()
    object.id = target['object_id']
    object.header.frame_id = target['frame']
    object.pose.orientation.w = 1
    object.operation = object.ADD
    object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=target['target_size'])]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = target['target_pose'][:3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = target['target_pose'][3:]
    object.primitive_poses = [pose]
    message.world.collision_objects = [object]
    if guarded:
        base = ET.fromstring(urdf).find("./link[@name='ground/base_link']/collision")
        guard = CollisionObject()
        guard.id = 'chassis_clearance_probe'
        guard.header.frame_id = 'ground/base_link'
        guard.pose.orientation.w = 1
        guard.operation = guard.ADD
        guard.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v)+.024 for v in base.find('geometry/box').get('size').split()])]
        gp = Pose()
        gp.orientation.w = 1
        gp.position.x, gp.position.y, gp.position.z = map(float, base.find('origin').get('xyz').split())
        guard.primitive_poses = [gp]
        message.world.collision_objects.append(guard)
    inputs += packet(message)
result = subprocess.run([sys.argv[1]], input=inputs, capture_output=True, timeout=60)
print(result.stdout.decode(), end='')
print(result.stderr.decode(), end='', file=sys.stderr)
raise SystemExit(result.returncode)
