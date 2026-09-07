#!/usr/bin/python3
"""Compare RM4D/SIM arm-local geometry and request real IK/planning, no execution.

The BUNKER stays parked. The representative candidate's exact arm-local target
is expressed under the current SIM arm base. This tests the same relative robot
geometry, not navigation to the candidate or a grasp at the parked location.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np


def compare_local_geometry(candidate, exact_map_tcp, query_map_tcp, sim_mount, flange_tcp):
    sim_aubo = np.asarray(candidate['T_map_bunker']) @ sim_mount
    local_exact = np.linalg.inv(sim_aubo) @ exact_map_tcp
    local_query = np.linalg.inv(sim_aubo) @ query_map_tcp
    baseline_local = np.asarray(candidate['T_aubo_flange']) @ flange_tcp
    return {'T_aubo_exact_grasp': local_exact.tolist(),
            'T_aubo_query_grasp_sim': local_query.tolist(),
            'T_aubo_query_grasp_rm4d': baseline_local.tolist(),
            'query_local_max_abs_error': float(np.max(np.abs(local_query - baseline_local)))}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--initial-file', type=Path, required=True)
    parser.add_argument('--rm4d-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate-id', required=True)
    args = parser.parse_args(argv)
    import rospy
    import moveit_commander
    import tf2_ros
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
    from sensor_msgs.msg import JointState
    from tf.transformations import quaternion_from_matrix
    from a5_ros_support import rigid_transform

    initial = json.loads(args.initial_file.read_text())
    frozen = json.loads(args.rm4d_config.read_text())
    candidate = next(c for c in initial['result']['evaluated_candidates']
                     if c['candidate_id'] == args.candidate_id and c['valid'])
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('a5_frame_bridge_planning_check', anonymous=True)
    buffer = tf2_ros.Buffer(); listener = tf2_ros.TransformListener(buffer)
    transform = buffer.lookup_transform('ground/base_link', 'ground/aubo_i5_base_link',
                                        rospy.Time(0), rospy.Duration(10))
    p, q = transform.transform.translation, transform.transform.rotation
    mount = rigid_transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
    exact = initial['grasp']; query = initial['query_request']
    exact_tcp = rigid_transform(exact['position_xyz'], exact['quaternion_xyzw'])
    query_tcp = np.asarray(initial['frame_calibration']['T_map_reference']) @ rigid_transform(
        query['position_xyz'], query['quaternion_xyzw'])
    report = compare_local_geometry(candidate, exact_tcp, query_tcp, mount,
                                   np.asarray(frozen['transforms']['T_flange_tcp']))
    report.update(candidate_id=args.candidate_id, exact_map_grasp=exact,
                  T_bunker_aubo_sim=mount.tolist(), executed=False,
                  scope='candidate arm-local exact grasp under current parked SIM base; no base motion')
    if report['query_local_max_abs_error'] > 1e-9:
        raise ValueError('RM4D / SIM arm-relative grasp mismatch')
    # Express the exact target under the public current arm base. No exact-map
    # grasp is overwritten and no Gazebo model-state information is used.
    target = PoseStamped(); target.header.frame_id = 'ground/aubo_i5_base_link'
    local = np.asarray(report['T_aubo_exact_grasp'])
    target.pose.position.x, target.pose.position.y, target.pose.position.z = local[:3, 3]
    (target.pose.orientation.x, target.pose.orientation.y,
     target.pose.orientation.z, target.pose.orientation.w) = quaternion_from_matrix(local)
    group = moveit_commander.MoveGroupCommander('manipulator', wait_for_servers=20)
    group.set_end_effector_link('ground/gripper_tcp_link')
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    floor = PoseStamped(); floor.header.frame_id = 'ground/base_link'
    floor.pose.orientation.w = 1
    floor.pose.position.z = -initial['frame_calibration']['ground_reference_height_m']
    scene.add_plane('a5_frame_check_floor', floor)
    try:
        rospy.wait_for_service('/compute_ik', timeout=10)
        request = GetPositionIKRequest()
        request.ik_request.group_name = 'manipulator'
        request.ik_request.ik_link_name = 'ground/gripper_tcp_link'
        request.ik_request.pose_stamped = target
        joints_now = rospy.wait_for_message('/ground/joint_states', JointState, timeout=5)
        if not 0 <= (rospy.Time.now() - joints_now.header.stamp).to_sec() <= .5:
            raise ValueError('SIM Ground joint state is stale')
        request.ik_request.robot_state.joint_state = joints_now
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = rospy.Duration(5)
        result = rospy.ServiceProxy('/compute_ik', GetPositionIK)(request)
        report['ik_error_code'] = result.error_code.val
        report['planning_frame'] = group.get_planning_frame()
        report['plan_success'] = False
        if result.error_code.val == 1:
            group.set_start_state_to_current_state()
            group.set_planner_id('RRTConnectkConfigDefault')
            group.set_planning_time(10)
            group.set_num_planning_attempts(5)
            # Planning to the collision-aware IK solution preserves exact target.
            joints = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
            group.set_joint_value_target({name: joints[name] for name in group.get_active_joints()})
            success, trajectory, planning_time, code = group.plan()
            report.update(plan_success=bool(success), plan_error_code=code.val,
                          planning_time_s=planning_time,
                          trajectory_points=len(trajectory.joint_trajectory.points),
                          ik_joint_solution={name: joints[name] for name in group.get_active_joints()})
    finally:
        scene.remove_world_object('a5_frame_check_floor')
        group.clear_pose_targets()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['plan_success'] else 1


if __name__ == '__main__':
    sys.exit(main())
